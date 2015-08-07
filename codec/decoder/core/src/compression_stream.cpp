#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "error_code.h"
#include "compression_stream.h"
#include <sstream>
using namespace WelsDec;
void warnme() {
fprintf(stderr, "DOING 431\n");
}
#define H264ErrorNil ERR_NONE
std::pair<uint32_t, H264Error> RawFileWriter::Write(int streamId, const uint8_t*data, unsigned int size) {
    // Ignores streamId.
    signed long nwritten = fwrite(data, size, 1, fp);
    if (nwritten == 0) {
        return std::pair<uint32_t, H264Error>(0, WelsDec::ERR_BOUND);
    }
    return std::pair<uint32_t, H264Error>(size, WelsDec::ERR_NONE);
}
std::pair<uint32_t, H264Error> RawFileReader::Read(uint8_t*data, unsigned int size) {
    signed long nread = fread(data, 1, size, fp);
    if (nread <= 0) {
        return std::pair<uint32_t, H264Error>(0, WelsDec::ERR_BOUND);
    }
    return std::pair<uint32_t, H264Error>(nread, WelsDec::ERR_NONE);
}

namespace {
static CompressionStream css;
static InputCompressionStream icss;
}
CompressionStream &oMovie() {
    return css;
}
InputCompressionStream &iMovie() {
    return icss;
}

CompressionStream::CompressionStream() {
    isRecoding = false;
}
BitStream::BitStream() {
    bitsWrittenSinceFlush = false;
    bits = 0;
    nBits = 0;
    bitReadCursor = 0;
    escapingEnabled = false;
    escapeBufferSize = 0;
    buffer.reserve(64*1024*1024);
}
void BitStream::appendByte(uint8_t x) {
    if (escapingEnabled) {
        if (x <= 3 && escapeBufferSize == 2 && escapeBuffer[0] == 0 && escapeBuffer[1] == 0) {
            buffer.push_back(0);
            buffer.push_back(0);
            buffer.push_back(3);
            buffer.push_back(x);
            escapeBufferSize = 0;
        }else if (escapeBufferSize == 2) {
            buffer.push_back(escapeBuffer[0]);
            escapeBuffer[0] = escapeBuffer[1];
            escapeBuffer[1] = x;
        } else {
            escapeBuffer[escapeBufferSize++] = x;
        }
    }else {
        buffer.push_back(x);
    }
}
void BitStream::appendBytes(const uint8_t*bytes, uint32_t nBytes) {
    if (escapingEnabled) {
        for (uint32_t i = 0; i < nBytes; ++i) {
            appendByte(bytes[i]);
        }
    }else {
        buffer.insert(buffer.end(), bytes, bytes + nBytes);
    }
}

void BitStream::clear() {
    buffer.clear();
}

void BitStream::flushToWriter(int streamId, CompressedWriter &w) {
    if (!buffer.empty()) {
        w.Write(streamId, &buffer[0], buffer.size());
    }
    buffer.clear();
}

void BitStream::emitBits(uint32_t bits, uint32_t nBits) {
    // fprintf(stderr, "emit %d\n", nBits);
    bitsWrittenSinceFlush = true;
    if (nBits > 16) {
        assert(false &&"Must have nBits < 16");
    }
    if (bits >= (1U << nBits)) {
        assert(false &&"bits is out of range for nBits");
    }
    BitStream &b = *this;
    nBits += uint32_t(b.nBits);
    bits <<= 32 - nBits;
    bits |= b.bits;
    while (nBits >= 8) {
        uint8_t bb = uint8_t(bits >> 24);
        b.appendByte(bb);
        bits <<= 8;
        nBits -= 8;
    }
    //fprintf(stderr, "Leftovers %d bits %x\n", nBits, bits)
    b.bits = bits;
    b.nBits = uint8_t(nBits);
}

void BitStream::padToByte() {
    for (int i = nBits; (i & 0x07) != 0; ++i) {
        oMovie().def().emitBit(0);
    }
}

std::pair<uint32_t, H264Error> BitStream::scanBits(uint32_t nBits) {
    // fprintf(stderr, "scan %d\n", nBits);
    BitStream &b = *this;
    if (nBits > 16) {
        assert(false &&"Must have nBits < 16");
    }
    if (nBits == 0) {
        return uint32E(0, H264ErrorNil); // don't read off the array since it may be empty or at its end
    }
    uint32_t byteAddress = b.bitReadCursor / 8;
    if (int(byteAddress) >= int(b.buffer.size())) {
        return uint32E(0, ERR_BOUND);
    }
    uint32_t bitAddress = b.bitReadCursor - byteAddress*8;
    uint32_t retval = 0;
    uint32_t curByte = b.buffer[byteAddress] & ((1 << (8 - bitAddress)) - 1);
    retval |= uint32_t(curByte);
    uint8_t remainingBitsInByte = 8 - bitAddress;
    //fmt.Printf("Retval %x[%d].%d so far,  Remaining bits %d\n", retval, byteAddress,bitAddress,nBits)
    if (remainingBitsInByte >= nBits) {
        retval >>= remainingBitsInByte - nBits;
        //fmt.Printf("Returning early after single byte read\n")
        b.bitReadCursor += nBits;
        return uint32E(retval, H264ErrorNil);
    }
    if (int(byteAddress) >= int(b.buffer.size())) {
        return uint32E(0, ERR_BOUND);
    }
    b.bitReadCursor += remainingBitsInByte;
    nBits -= remainingBitsInByte;
    if (nBits > 8) {
        b.bitReadCursor += 8;
        byteAddress += 1;
        retval <<= 8;
        retval |= uint32_t(b.buffer[byteAddress]);
        nBits -= 8;
    }

    if (nBits > 8) {
        assert(false &&"unreachable: we should have only 16 bits to grab");
    }
    //fmt.Printf("Pref Final things are %x going to read %x after shifting %d\n", retval, b.buffer[byteAddress + 1], nBits)
    if (byteAddress+1 >= b.buffer.size()) {
        return uint32E(0, ERR_BOUND);
    }
    retval <<= nBits;
    retval |= uint32_t(b.buffer[byteAddress+1] >> (8 - nBits));
    b.bitReadCursor += nBits;
    //fprintf(stderr, "Final value %x\n", retval)
    return uint32E(retval, H264ErrorNil);
}

void BitStream::pop() {
    BitStream &b = *this;
    if (b.nBits > 0 && b.nBits < 8) {
        if (b.buffer.empty()) {
            assert(false && "popping from empty buffer");
            return;
        }
        uint32_t poppedByte = 0;
        poppedByte = uint32_t(b.buffer.back());
        b.buffer.pop_back();
        poppedByte <<= b.nBits;
        b.bits |= poppedByte;
        b.nBits += 8;
    }
    if (b.nBits >= 8) {
        b.nBits -= 8;
        b.bits >>= 8;
    } else {
        b.buffer.pop_back();
    }
}
uint32_t BitStream::len()const {
    return uint32_t(buffer.size()) + uint32_t(nBits/8);
}
void BitStream::flushBits() {
    if (bitsWrittenSinceFlush) {
        //emitBits(1, 1);
    }
    while (nBits > 0) {
        emitBits(0, 1);
    }
    bitsWrittenSinceFlush = false;
}

std::vector<uint8_t> streamLenToBE(uint32_t streamLen) {
    uint8_t retval[5] = {uint8_t(streamLen >> 24), uint8_t((streamLen >> 16) & 0xff),
                       uint8_t((streamLen >> 8) & 0xff), uint8_t(streamLen & 0xff), 0};
    return std::vector<uint8_t>(retval, retval + 4);
}
uint32_t bufferBEToStreamLength(uint8_t *buf) {
    uint32_t vectorLength = 0;
    for (int i = 0; i < 4; i++) { // read in the huffvector length
        vectorLength <<= 8;
        vectorLength |= uint32_t(buf[i]);
    }
    return vectorLength;
}

void CompressionStream::flushToWriter(CompressedWriter&w) {
    for (std::map<int32_t, BitStream>::iterator i = taggedStreams.begin(), ie = taggedStreams.end();
         i != ie;
         ++i) {
        i->second.flushToWriter(i->first, w);
    }
}

BitStream& InputCompressionStream::tag(int32_t tag) {
    bool mustReadData = taggedStreams.find(tag) == taggedStreams.end();
    BitStream &bs = taggedStreams[tag];
    if (filenamePrefix.empty()) {
        fprintf(stderr, "Attempting to read %d without input file set\n", tag);
        assert(!filenamePrefix.empty());
    } else if (mustReadData) {
        std::ostringstream os;
        os << filenamePrefix << "." << tag;
        std::string thisfilename = os.str();
        FILE *fp = fopen(thisfilename.c_str(), "rb");
        int nread = 0;
        if (fp) {
            long datalen = -1;
            if (fseek(fp, 0, SEEK_END) != -1) {
                datalen = ftell(fp);
                rewind(fp);
            }
            if (datalen > 0) {
                bs.buffer.resize(datalen);
                nread = fread(&(bs.buffer[0]), datalen, 1, fp);
            }
            fclose(fp);
        }
        if (nread == 0) {
            fprintf(stderr, "Failed to read from file %s\n", thisfilename.c_str());
        }
    }
    return bs;
}

