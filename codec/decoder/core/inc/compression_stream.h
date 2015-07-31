#ifndef _COMPRESSION_STREAM_H_
#define _COMPRESSION_STREAM_H_
#include <string>
#include <vector>
#include <map>
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
struct CompressionStream {
    enum {
        DEFAULT_STREAM = 0x7fffffff
    };
    std::map<int32_t, BitStream> taggedStreams;
    bool isRecoding;
    CompressionStream();
    BitStream&def() {
        return taggedStreams[DEFAULT_STREAM];
    }
    BitStream&tag(int32_t tag) {
        return taggedStreams[tag];
    }
    void flushToWriter(CompressedWriter&);
};
CompressionStream &oMovie();
struct InputCompressionStream {
    std::string filenamePrefix;
    std::map<int32_t, BitStream> taggedStreams;
    BitStream&tag(int32_t tag);
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
#define ROUNDTRIP_TEST
void InitEncFuncPtrList();

struct RoundTripData {
  uint8_t eSliceType;
  int uiChromaQpIndexOffset;
  uint8_t pNonZeroCount[48];
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
  uint32_t uiChromaQp;

  RoundTripData()
      : eSliceType(), uiChromaQpIndexOffset(), pNonZeroCount(),
        iPrevIntra4x4PredMode(), iRemIntra4x4PredMode(), sMbMvp(),
        uiSubMbType(), iRefIdx(), uiCbpC(0), uiCbpL(0), iLastMbQp(0),
        uiChmaI8x8Mode(), uiLumaI16x16Mode(), iMbSkipRun(), uiMbType(0),
        uiNumRefIdxL0Active(), uiLumaQp(), uiChromaQp() {
      for (size_t i = 0; i < sizeof(pNonZeroCount) / sizeof(pNonZeroCount[0]); ++i) {
          pNonZeroCount[i] = 0;
      }
  }
  void preInit(const WelsDec::PSlice);
};

//sWelsEncCtx *allocWelsEncCtx();
//void freeWelsEncCtx(sWelsEncCtx *);

#endif
