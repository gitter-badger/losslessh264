#ifndef _MACROBLOCK_MODEL_H_
#define _MACROBLOCK_MODEL_H_

class DecodedMacroblock;
namespace WelsDec{
    struct TagWelsDecoderContext;
    typedef struct TagWelsDecoderContext *PWelsDecoderContext;
}

class MacroblockModel {

    DecodedMacroblock *mb;
    WelsDec::PWelsDecoderContext pCtx;

public:
    void initCurrentMacroblock(
            DecodedMacroblock *curMb, WelsDec::PWelsDecoderContext pCtx);

};


#endif
