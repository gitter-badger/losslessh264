#ifndef _COMPRESSION_STREAM_H_
#define _COMPRESSION_STREAM_H_
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
    virtual std::pair<uint32_t, H264Error> Write(const uint8_t*data, unsigned int size) = 0;
    virtual void Close() = 0;
    virtual ~CompressedWriter(){}
};
class RawFileWriter : public CompressedWriter {
    FILE * fp;
public:
    RawFileWriter(FILE * ff){
        fp = ff;
    }
    std::pair<uint32_t, H264Error> Write(const uint8_t*data, unsigned int size);
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
    typedef std::pair<uint32_t, H264Error> uint32E;
    std::vector<uint8_t> buffer;
	uint32_t bits;
	uint8_t nBits;
	uint32_t bitReadCursor;
    BitStream();
    void appendByte(uint8_t x);
    void appendBytes(const uint8_t*bytes, uint32_t nBytes);
    void clear();
    void flushToWriter(CompressedWriter&);
    void emitBits(uint32_t bits, uint32_t nBits);
    std::pair<uint32_t, H264Error> scanBits(uint32_t nBits);
    void pop();
    uint32_t len() const;
    void flushBits();
};
struct CompressionStream {
    enum {
        DEFAULT_STREAM = 0x7fffffff
    };
    std::map<int32_t, BitStream> taggedStreams;
    BitStream&def() {
        return taggedStreams[DEFAULT_STREAM];
    }
    BitStream&tag(int32_t tag) {
        return taggedStreams[tag];
    }
    void flushToWriter(CompressedWriter&);
};
CompressionStream &oMovie();
#endif
