#include <cstring>
#include <assert.h>
#include "array_nd.h"
#include "macroblock_model.h"
#include "decoder_context.h"
#include "wels_common_defs.h"

void MacroblockModel::initCurrentMacroblock(
            DecodedMacroblock *curMb, WelsDec::PWelsDecoderContext pCtx) {
    this->mb = curMb;
    this->pCtx = pCtx;
}

Branch<4> MacroblockModel::getMacroblockTypePrior() {
    return mbTypePriors.at((uint32_t)(
        pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt
             .sSliceHeader.eSliceType == P_SLICE));
}
int MacroblockModel::encodeMacroblockType(int welsType) {
    switch (welsType) {
        case MB_TYPE_INTRA4x4:
            return 0;
        case MB_TYPE_INTRA16x16:
            return 1;
        case MB_TYPE_INTRA8x8:
            return 2;
        case MB_TYPE_16x16:
            return 3;
        case MB_TYPE_16x8:
            return 4;
        case MB_TYPE_8x16:
            return 5;
        case MB_TYPE_8x8:
            return 6;
        case MB_TYPE_8x8_REF0:
            return 7;
        case MB_TYPE_INTRA_PCM:
            return 8;
        case MB_TYPE_INTRA_BL:
            return 9;
        case MB_TYPE_DIRECT2:
            return 10;
        case MB_TYPE_SKIP:
        default:
            fprintf(stderr, "Invalid macroblock type %d\n", welsType);
            assert(false && "Invalid macroblock type");
            return 0;
    }
}
int MacroblockModel::decodeMacroblockType(int storedType) {
    static const uint32_t MB_TYPES[16] = {
        MB_TYPE_INTRA4x4, MB_TYPE_INTRA16x16, MB_TYPE_INTRA8x8,
        MB_TYPE_16x16, MB_TYPE_16x8, MB_TYPE_8x16, MB_TYPE_8x8,
        MB_TYPE_8x8_REF0, MB_TYPE_INTRA_PCM, MB_TYPE_INTRA_BL,
        MB_TYPE_DIRECT2, 0, 0, 0, 0, 0
    };
    int ret = MB_TYPES[storedType];
    if (!ret) {
        fprintf(stderr, "Invalid decoded macroblock type %d\n", storedType);
        assert (ret && "Invalid decoded macroblock type");
    }
    return ret;
}
