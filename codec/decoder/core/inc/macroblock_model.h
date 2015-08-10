#ifndef _MACROBLOCK_MODEL_H_
#define _MACROBLOCK_MODEL_H_

#include "array_nd.h"
#include "compression_stream.h"

class DecodedMacroblock;
namespace WelsDec{
    struct TagWelsDecoderContext;
    typedef struct TagWelsDecoderContext *PWelsDecoderContext;
}

class MacroblockModel {

    DecodedMacroblock *mb;
    WelsDec::PWelsDecoderContext pCtx;

    Sirikata::Array2d<DynProb, 2, 15> mbTypePriors; // We could use just 8 bits for I Slices
public:
    void initCurrentMacroblock(
            DecodedMacroblock *curMb, WelsDec::PWelsDecoderContext pCtx);

    Branch<4> getMacroblockTypePrior();
    int encodeMacroblockType(int welsType);
    int decodeMacroblockType(int storedType);
};


#endif
