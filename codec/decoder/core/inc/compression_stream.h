#ifndef _COMPRESSION_STREAM_H_
#define _COMPRESSION_STREAM_H_
#include <string>
#include <vector>
#include <map>
#include "bitreader.h"
#include "bitwriter.h"

//#define CONTEXT_DIFF

//#define ROUNDTRIP_TEST

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
class RawFileWriter : public CompressedWriter {
    FILE * fp;
public:
    RawFileWriter(FILE * ff){
        fp = ff;
    }
    std::pair<uint32_t, H264Error> Write(int, const uint8_t*data, unsigned int size);
    void Close() {
        fclose(fp);
    }
};
class RawFileReader : public CompressedReader {
    FILE * fp;
public:
    RawFileReader(FILE * ff){
        fp = ff;
    }
    std::pair<uint32_t, H264Error> Read(uint8_t*data, unsigned int size);
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
};

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
};

struct CompressionStream {
    enum {
        DEFAULT_STREAM = 0x7fffffff
    };
    BitStream defaultStream;
    std::map<int32_t, ArithmeticCodedOutput> taggedStreams;
    bool isRecoding;
    CompressionStream();
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
}

namespace WelsDec {
    struct TagSlice;
    typedef TagSlice* PSlice;
}
extern WelsEnc::SWelsFuncPtrList *gFuncPtrList;
void InitEncFuncPtrList();

struct RoundTripData {
  uint8_t eSliceType;
  int uiChromaQpIndexOffset;
  int32_t iPrevIntra4x4PredMode[16];
  int32_t iRemIntra4x4PredMode[16];
  int16_t sMbMvp[16][2];
  int uiSubMbType[4];
  int8_t iRefIdx[4];
  uint32_t uiCbpC, uiCbpL;
  int32_t iLastMbQp;
  uint8_t uiChmaI8x8Mode;
  uint8_t uiLumaI16x16Mode;
  int32_t iMbSkipRun;
  uint32_t uiMbType;
  uint32_t uiNumRefIdxL0Active;
  uint32_t uiLumaQp;

  RoundTripData()
      : eSliceType(), uiChromaQpIndexOffset(),
        iPrevIntra4x4PredMode(), iRemIntra4x4PredMode(), sMbMvp(),
        uiSubMbType(), iRefIdx(), uiCbpC(0), uiCbpL(0), iLastMbQp(0),
        uiChmaI8x8Mode(), uiLumaI16x16Mode(), iMbSkipRun(), uiMbType(0),
        uiNumRefIdxL0Active(), uiLumaQp() {
  }
  void preInit(const WelsDec::PSlice);
};

//sWelsEncCtx *allocWelsEncCtx();
//void freeWelsEncCtx(sWelsEncCtx *);

#endif
