#ifndef _COMPRESSION_STREAM_H_
#define _COMPRESSION_STREAM_H_
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
//#define CABAC_LOG_DECISIONS

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


class ArithmeticCodedInput {
public:
    std::vector<uint8_t> buffer;
    vpx_reader reader;

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
      return *this;
    }

    void init() {
        vpx_reader_init(&reader, buffer.data(), buffer.size());
    }

    bool scanBit(DynProb *prob) {
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

    static DynProb TEST_PROB;

    ArithmeticCodedOutput() : writer() {
    }

    void init() {
        buffer.resize(1000);
        vpx_start_encode(&writer, &buffer.front());
    }

    ArithmeticCodedOutput(const ArithmeticCodedOutput &orig) {
      *this = orig;
    }

    ArithmeticCodedOutput &operator=(const ArithmeticCodedOutput &orig) {
      writer = orig.writer;
      buffer = orig.buffer;
      writer.buffer = &buffer.front();
      return *this;
    }

    void flushToWriter(int streamId, CompressedWriter&);

    void emitBit(bool bit, DynProb *prob) {
        if (writer.pos + 512 > buffer.size()) {
            buffer.resize(buffer.size() * 2);
            writer.buffer = &buffer.front();
        }

#ifdef CONTEXT_DIFF
        static int count = 0;
        fprintf(stderr, "bit %d prob %d -> %d\n", count++, (int)prob->getProb(), (int)bit);
#endif
        vpx_write(&writer, bit, prob->getProb());

        prob->updateProb(bit);
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
            taggedStreams[tag].init();
        }
        return taggedStreams[tag];
    }
    void flushToWriter(CompressedWriter&);
};
CompressionStream &oMovie();

struct InputCompressionStream {
    std::string filenamePrefix;
    std::map<int32_t, ArithmeticCodedInput> taggedStreams;
    ArithmeticCodedInput&tag(int32_t tag);
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
