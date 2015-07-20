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
}
extern WelsEnc::SWelsFuncPtrList *gFuncPtrList;
void InitEncFuncPtrList();
#endif
