#ifndef _COMPRESSION_STREAM_H_
#define _COMPRESSION_STREAM_H_
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <map>
#include "bitreader.h"
#include "bitwriter.h"

#include <assert.h>
#include "array_nd.h"

//#define CONTEXT_DIFF

//#define ROUNDTRIP_TEST

//#define CABAC_HACK

//#define PRIOR_STATS

#ifdef ROUNDTRIP_TEST
#  define DEBUG_PRINTS
#endif

typedef int32_t H264Error;
class CompressedReader {
public:
    virtual std::pair<uint32_t, H264Error> Read(uint8_t*data, unsigned int size) = 0;
    virtual ~CompressedReader(){}
};
class CompressedWriter {
public:
    virtual std::pair<uint32_t, H264Error> Write(int streamId, const uint8_t*data, unsigned int size) = 0;
    virtual void Close() = 0;
    virtual ~CompressedWriter(){}
};

struct BitStream {
    bool escapingEnabled;
    bool bitsWrittenSinceFlush;// if any bits have been written since last flush was called

    typedef std::pair<uint32_t, H264Error> uint32E;
    std::vector<uint8_t> buffer;
    uint8_t escapeBuffer[2];
    uint32_t escapeBufferSize;
	uint32_t bits;
	uint8_t nBits;
	uint32_t bitReadCursor;
    BitStream();
    void appendByte(uint8_t x);
    void appendBytes(const uint8_t*bytes, uint32_t nBytes);
    void clear();
    void flushToWriter(int streamId, CompressedWriter&);
    void emitBits(uint32_t bits, uint32_t nBits);
    void emitBit(uint32_t bit) {
      emitBits(bit, 1);
    }
    void padToByte();
    std::pair<uint32_t, H264Error> scanBits(uint32_t nBits);
    void pop();
    uint32_t len() const;
    size_t bitlen() const{
        return nBits + buffer.size() * 8;
    }
    void flushBits();
    void escape00xWith003x() {
        escapingEnabled = true;
    }
    void stopEscape() {
        flushBits();
        escapingEnabled = false;
        uint8_t buffer[sizeof(escapeBuffer)];
        for (uint32_t i = 0; i < escapeBufferSize; ++i) {
            buffer[i] = escapeBuffer[i];
        }
        uint32_t size = escapeBufferSize;
        escapeBufferSize = 0;
        appendBytes(buffer, size);
    }
};

class DynProb {
    uint8_t prob;
    unsigned counts[2];

    enum {
        RESCALE_CONST = 4096,
    };
public:

    DynProb() {
        counts[0] = counts[1] = 0;
        prob = 128;
    }

    inline void updateProb(bool bit) {
        counts[bit]++;

        prob = (256 * (counts[0] + 1)) / (counts[1] + counts[0] + 2);

        if (counts[0] + counts[1] > RESCALE_CONST) {
            counts[0] = (counts[0] + 1) >> 1;
            counts[1] = (counts[1] + 1) >> 1;
        }
    }

    uint8_t getProb() {
        return prob;
    }
};

template <int nBits>
class Branch {
    enum {
        NUM_CHILDREN = (1 << (nBits - 1)) - 1
    };
    typedef typename Sirikata::Array1d<DynProb, (1 << nBits) - 1>::Slice ArrayType;
    ArrayType array;
    uint32_t val;
public:
    Branch(ArrayType array) : array(array), val(0) {
    }

    Branch(uint32_t val, ArrayType array) : array(array), val(val) {
    }

    inline Branch<nBits - 1> selectBranch(bool which) {
        if (which) {
            return Branch<nBits - 1>((val << 1) | 1,
                    array.template slice<NUM_CHILDREN + 1, NUM_CHILDREN * 2 + 1>());
        } else {
            return Branch<nBits - 1>((val << 1),
                    array.template slice<1, NUM_CHILDREN + 1>());
        }
    }


    DynProb *getProb() {
        return &(array[0]);
    }
};

template <> class Branch<1> {
    typedef typename Sirikata::Array1d<DynProb, 1>::Slice ArrayType;
    DynProb *prob;
    uint32_t val;
public:
    Branch(ArrayType array) : prob(&(array[0])), val(0) {
    }

    Branch(uint32_t val, ArrayType array) : prob(&(array[0])), val(val) {
    }

    DynProb *getProb() {
        return prob;
    }

    uint32_t getVal(bool result) {
        return (val << 1) | (int)result;
    }
};


// Various int priors used to emit/scan Rice-coded integers.

// UnaryIntPrior: encode values as '0', '10', '110', '1110', etc, with priors on the first N values.
template <int N = 0>
struct UnaryIntPrior {
    std::array<DynProb, N> priors;
    DynProb* at(int i) {
      return &priors[std::min<int>(i, priors.size()-1)];
    }
};

template <>
struct UnaryIntPrior<0> {
    // Special case: just emit 0/1, don't save counts. Uses no memory.
    DynProb* at(int i) {
        static DynProb staticDynProb;
        staticDynProb = DynProb();
        return &staticDynProb;
    }
};

// PositiveIntPrior: encode 1..infinity as unary exponent + mantissa value.
template <int Exponent = 0, int Mantissa = 0>
struct PositiveIntPrior {
    static constexpr bool hasSign = false, hasZero = false;
    UnaryIntPrior<Exponent> exponent;
    std::array<DynProb, Mantissa> mantissa;

    DynProb* sign() { return nullptr; }
    DynProb* zero() { return nullptr; }
};

// UnsignedIntPrior: encode 0..infinity as zero bit + unary exponent + mantissa value.
template <int Exponent = 0, int Mantissa = 0>
struct UnsignedIntPrior : public PositiveIntPrior<Exponent, Mantissa> {
    static constexpr bool hasZero = true;
    DynProb zeroPrior;
    DynProb* zero() { return &zeroPrior; }
};

// IntPrior: encode -infinity..infinity as zero bit + sign bit + unary exponent + mantissa value.
template <int Exponent = 0, int Mantissa = 0>
struct IntPrior : public UnsignedIntPrior<Exponent, Mantissa> {
    static constexpr bool hasSign = true;
    DynProb signPrior;
    DynProb* sign() { return &signPrior; }
};


class ArithmeticCodedInput {
public:
    std::vector<uint8_t> buffer;
    vpx_reader reader;
    int32_t tag;

    static DynProb TEST_PROB;

    ArithmeticCodedInput() : reader() {
        // init() will be called by InputCompressionStream::tag()
        // after the file has been read and inserted into buffer.
        (void)vpx_reader_has_error;
        (void)vpx_read_literal;
        (void)vpx_write_literal;
    }

    ArithmeticCodedInput(const ArithmeticCodedInput &orig) {
      *this = orig;
    }

    ArithmeticCodedInput &operator=(const ArithmeticCodedInput &orig) {
      reader = orig.reader;
      buffer = orig.buffer;
      reader.buffer = &buffer.front();
      tag = orig.tag;
      return *this;
    }

    void init(int32_t stream_tag) {
        vpx_reader_init(&reader, buffer.data(), buffer.size());
        tag = stream_tag;
    }

    bool scanBit(DynProb *prob = &TEST_PROB) {
        bool bit = !!vpx_read(&reader, prob->getProb());
#ifdef CONTEXT_DIFF
        static int count = 0;
        fprintf(stderr, "bit %d prob %d -> %d\n", count++, (int)prob->getProb(), (int)bit);
#endif
        prob->updateProb(bit);

        return bit;
    }
    std::pair<uint32_t, H264Error> scanBits(int nBits) {
        uint16_t out = 0;
        for (int i = 0; i < nBits; ++i) {
            out |= ((uint16_t)scanBit(&TEST_PROB)) << (nBits - i - 1);
        }
#ifdef CONTEXT_DIFF
        fprintf(stderr, "out %d\n", (int)out);
#endif
        return std::make_pair(out, 0);
    }

    template <int nBits>
    std::pair<uint32_t, H264Error> scanBits(Branch<nBits> priors);
    template <int nBits> struct Ident {
        enum {VALUE =  nBits};
    };
    template <int nBits> struct IdentPow {
        enum {VALUE =  (1<<nBits)};
    };
    template <int nBits> struct IdentPow_1 {
        enum {VALUE =  (1<<nBits) - 1};
    };
    template<int start, int end> struct SliceRange{
        enum {START=start, END=end};
    };

    template<int nBits>
      std::pair<uint32_t, H264Error> scanBitsZeroToPow2Inclusive(typename Sirikata::Array1d<DynProb, (1<< nBits) >::Slice priors, uint32_t preferred = 0) {
        bool zeroBit = scanBit(&priors.at(0));
        std::pair<uint32_t, H264Error> retval (preferred, 0);
        if (!zeroBit) {
            return retval;
        }
        SliceRange<1, (1<<nBits)> sr;
        retval = scanBits(Branch<nBits>(priors.slice(sr)));
        retval.first += retval.first >= preferred ? 1 : 0;
        return retval;
    }

    template <int N>
    int scanUnary(UnaryIntPrior<N>* prior) {
      int i = 0;
      while (scanBit(prior->at(i))) {
        i++;
      }
      return i;
    }
};

template <>
inline std::pair<uint32_t, H264Error> ArithmeticCodedInput::scanBits(Branch<1> priors) {
    bool res = scanBit(priors.getProb());
    return std::make_pair(priors.getVal(res), 0);
}

template <int nBits>
inline std::pair<uint32_t, H264Error> ArithmeticCodedInput::scanBits(Branch<nBits> priors) {
    bool res = scanBit(priors.getProb());
    return scanBits<nBits - 1>(priors.selectBranch(res));
}

class ArithmeticCodedOutput {
public:
    std::vector<uint8_t> buffer;
    vpx_writer writer;
    std::map<std::pair<std::string, int>, std::pair<int, int>> bill;
    std::pair<int, int>* bill_subtag = &bill[std::make_pair("(none)", 0)];  // always points into bill
    int32_t tag;

#ifdef PRIOR_STATS
    uint64_t miss_counts[2];
    std::vector<bool> misses;
#endif

    static DynProb TEST_PROB;

    ArithmeticCodedOutput() : writer() {
    }
    ~ArithmeticCodedOutput();

    void init(int32_t stream_tag) {
        tag = stream_tag;
        buffer.resize(1000);
        vpx_start_encode(&writer, &buffer.front());

#ifdef PRIOR_STATS
        miss_counts[0] = miss_counts[1] = 0;
#endif
    }

    ArithmeticCodedOutput(const ArithmeticCodedOutput &orig) {
      *this = orig;
    }

    ArithmeticCodedOutput &operator=(const ArithmeticCodedOutput &orig) {
      writer = orig.writer;
      buffer = orig.buffer;
      writer.buffer = &buffer.front();
      tag = orig.tag;
      return *this;
    }

    void flushToWriter(int streamId, CompressedWriter&);

    void billTo(const std::string& label_string, int label_int) {
        bill_subtag = &bill[std::make_pair(label_string, label_int)];
    }
    void billTo(const std::string& label) {
        billTo(label, 0);
    }
    void billTo(int label) {
        billTo("", label);
    }

    void emitBit(bool bit, DynProb *prob) {
        if (writer.pos + 512 > buffer.size()) {
            buffer.resize(buffer.size() * 2);
            writer.buffer = &buffer.front();
        }

#ifdef CONTEXT_DIFF
        static int count = 0;
        fprintf(stderr, "bit %d prob %d -> %d\n", count++, (int)prob->getProb(), (int)bit);
#endif
        unsigned int pos = writer.pos;
        vpx_write(&writer, bit, prob->getProb());

#ifdef PRIOR_STATS
        // prob->getProb() is probability of a zero bit, as uint in [0, 256)
        bool is_miss = bit ^ (prob->getProb() < 128);
        miss_counts[is_miss]++;
        misses.push_back(is_miss);
#endif

        prob->updateProb(bit);

        // Sampling based billing - bill whole byte to this bit, should be roughly accurate for big users.
        if (writer.pos > pos) {
          assert(writer.pos == pos + 1);
          bill_subtag->first++;
        }
        bill_subtag->second++;
    }

    void emitBit(bool bit) {
        emitBit(bit, &TEST_PROB);
    }

    void emitBits(uint16_t data, int nBits) {
        for (int i = 0; i < nBits; ++i) {
            emitBit(data & (1 << (nBits - 1 - i)), &TEST_PROB);
        }
#ifdef CONTEXT_DIFF
        fprintf(stderr, "out %d\n", (int)data);
#endif
    }

    template <int nBits>
    void emitBits(uint32_t data, Branch<nBits> priors);
    template<int start, int end> struct SliceRange{
        enum {START=start, END=end};
    };
    template <int nBits>
    void emitBitsZeroToPow2Inclusive(uint32_t data, typename Sirikata::Array1d<DynProb, (1<< nBits)>::Slice priors, uint32_t preferred = 0) {
        bool zeroBit = (data != preferred);
        emitBit(zeroBit, &priors.at(0));
        if (data != preferred) {
            SliceRange<1, (1<<nBits)>sr;
            emitBits(data > preferred ? (data - 1) : data, Branch<nBits>(priors.slice(sr)));
        }
    }

    template <int N>
    void emitUnary(int data, UnaryIntPrior<N>* prior) {
      for (int i = 0; i < data; i++) {
          emitBit(true, prior->at(i));
      }
      emitBit(false, prior->at(data));
    }
};

template <int nBits>
inline void ArithmeticCodedOutput::emitBits(uint32_t data, Branch<nBits> priors) {
    bool curBit = !!(data & (1 << (nBits - 1)));
    emitBit(curBit, priors.getProb());
    emitBits<nBits - 1>(data & ((1 << (nBits - 1)) - 1), priors.selectBranch(curBit));
}

template <>
inline void ArithmeticCodedOutput::emitBits(uint32_t data, Branch<1> priors) {
    emitBit(!!data, priors.getProb());
}

class MacroblockModel;

struct CompressionStream {
    enum {
        DEFAULT_STREAM = 0x7fffffff
    };
    BitStream defaultStream;
    std::map<int32_t, ArithmeticCodedOutput> taggedStreams;
    bool isRecoding;
    MacroblockModel *pModel;

    CompressionStream();
    ~CompressionStream();

    MacroblockModel &model() {
        return *pModel;
    }

    BitStream&def() {
        return defaultStream;
    }
    ArithmeticCodedOutput&tag(int32_t tag) {
        if (taggedStreams.find(tag) == taggedStreams.end()) {
            taggedStreams[tag].init(tag);
        }
        return taggedStreams[tag];
    }
    void flushToWriter(CompressedWriter&);

    // Prior may be IntPrior<Exponent, Mantissa>, UnsignedIntPrior, or NonzeroIntPrior.
    template <class Prior>
    void emitInt(int data, Prior* prior, int tagExponent, int tagMantissa = -1, int tagZero = -1, int tagSign = -1) {
      if (tagMantissa == -1) tagMantissa = tagExponent;
      if (tagZero == -1) tagZero = tagExponent;
      if (tagSign == -1) tagSign = tagExponent;

      if (prior->hasZero) {
        tag(tagZero).emitBit(data == 0, prior->zero());
        if (data == 0) return;
      }
      assert(data != 0);
      if (prior->hasSign) {
        tag(tagSign).emitBit(data > 0, prior->sign());
        if (data < 0) data = -data;
      }
      assert(data > 0);

      // TODO(ctl) use jongmin's faster log2 computation.
      int log2 = 0;
      while ((2 << log2) <= data) log2++;
      assert((1 << log2) <= data && data < (2 << log2));
      tag(tagExponent).emitUnary(log2, &prior->exponent);

      int lo = 0, hi = prior->mantissa.size();
      for (int i = log2-1; i >= 0; i--) {
        bool bit = (data & (1 << i)) != 0;
        if (hi > lo) {
          int mid = (hi + lo) / 2;
          tag(tagMantissa).emitBit(bit, &prior->mantissa[mid]);
          if (bit) {
            lo = mid + 1;
          } else {
            hi = mid;
          }
        } else {
          tag(tagMantissa).emitBit(bit);
        }
      }
    }

    // The default prior emits a Rice coding (plus zero and sign bits).
    void emitInt(int data, int tag) {
      IntPrior<> prior;
      emitInt(data, &prior, tag);
    }
};
CompressionStream &oMovie();

struct InputCompressionStream {
    std::string filenamePrefix;
    std::map<int32_t, ArithmeticCodedInput> taggedStreams;
    ArithmeticCodedInput&tag(int32_t tag);

    // Prior may be IntPrior<Exponent, Mantissa>, UnsignedIntPrior, or NonzeroIntPrior.
    template <class Prior>
    int scanInt(Prior* prior, int tagExponent, int tagMantissa = -1, int tagZero = -1, int tagSign = -1) {
      if (tagMantissa == -1) tagMantissa = tagExponent;
      if (tagZero == -1) tagZero = tagExponent;
      if (tagSign == -1) tagSign = tagExponent;

      if (prior->hasZero) {
        if (tag(tagZero).scanBit(prior->zero())) return 0;
      }
      bool sign = true;
      if (prior->hasSign) {
        sign = tag(tagSign).scanBit(prior->sign());
      }

      int log2 = tag(tagExponent).scanUnary(&prior->exponent);

      int lo = 0, hi = prior->mantissa.size();
      int data = 1;
      for (int i = log2-1; i >= 0; i--) {
        bool bit;
        if (hi > lo) {
          int mid = (hi + lo) / 2;
          bit = tag(tagMantissa).scanBit(&prior->mantissa[mid]);
          if (bit) {
            lo = mid + 1;
          } else {
            hi = mid;
          }
        } else {
          bit = tag(tagMantissa).scanBit();
        }
        data = (data << 1) | bit;
      }
      return sign ? data : -data;
    }

    // The default prior emits a Rice coding (plus zero and sign bits).
    int scanInt(int tag) {
      IntPrior<> prior;
      return scanInt(&prior, tag);
    }

};
InputCompressionStream &iMovie();

// Hack!!! This totally doesn't belong here!
namespace WelsEnc {
    struct TagWelsFuncPointerList;
    typedef struct TagWelsFuncPointerList SWelsFuncPtrList;

    struct TagWelsEncCtx;
    typedef struct TagWelsEncCtx sWelsEncCtx;

    struct TagSlice;
    typedef struct TagSlice SSlice;

    struct TagMB;
    typedef struct TagMB SMB;
    void WelsInitSliceCabac (sWelsEncCtx* pEncCtx, SSlice* pSlice);
    void WelsCabacInit (void *ctx);
}

namespace WelsDec {
    struct TagSlice;
    typedef TagSlice* PSlice;
}
extern WelsEnc::SWelsFuncPtrList *gFuncPtrList;
void InitEncFuncPtrList();

//sWelsEncCtx *allocWelsEncCtx();
//void freeWelsEncCtx(sWelsEncCtx *);

#endif
