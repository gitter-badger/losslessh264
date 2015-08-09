#include <cstring>
#include <assert.h>
#include "array_nd.h"
#include "decoder_context.h"
#include "macroblock_model.h"

void MacroblockModel::initCurrentMacroblock(
            DecodedMacroblock *curMb, WelsDec::PWelsDecoderContext pCtx) {
    this->mb = curMb;
    this->pCtx = pCtx;
}

