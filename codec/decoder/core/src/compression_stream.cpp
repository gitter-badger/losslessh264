#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "error_code.h"
#include "compression_stream.h"
using namespace WelsDec;
#define H264ErrorNil ERR_NONE
std::pair<uint32_t, H264Error> RawFileWriter::Write(const uint8_t*data, unsigned int size) {
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
}
CompressionStream &oMovie() {
    return css;
}

BitStream::BitStream() {
    bits = 0;
    nBits = 0;
    bitReadCursor = 0;
}
void BitStream::appendByte(uint8_t x) {
    buffer.push_back(x);
}
void BitStream::appendBytes(const uint8_t*bytes, uint32_t nBytes) {
    buffer.insert(buffer.end(), bytes, bytes + nBytes);
}

void BitStream::clear() {
    buffer.clear();
}

void BitStream::flushToWriter(CompressedWriter &w) {
    if (!buffer.empty()) {
        w.Write(&buffer[0], buffer.size());
    }
    buffer.clear();
}

void BitStream::emitBits(uint32_t bits, uint32_t nBits) {
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

std::pair<uint32_t, H264Error> BitStream::scanBits(uint32_t nBits) {
    BitStream &b = *this;
    if (nBits > 16) {
        assert(false &&"Must have nBits < 16");
    }
    if (nBits == 0) {
        return uint32E(0, H264ErrorNil); // don't read off the array since it may be empty or at its end
    }
    uint32_t byteAddress = b.bitReadCursor / 8;
    if (int(byteAddress) >= b.buffer.size()) {
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
    if (int(byteAddress) >= b.buffer.size()) {
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
    while (nBits > 0) {
        emitBits(1, 1);
    }
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
        i->second.flushToWriter(w);
    }
}
