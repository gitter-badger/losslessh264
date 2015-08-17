#include <cstring>
#include <assert.h>
#include "array_nd.h"
#include "macroblock_model.h"
#include "decoder_context.h"
#include "wels_common_defs.h"
#include "decoded_macroblock.h"

void Neighbors::init(const FreqImage *f, int x, int y) {
    using namespace Nei;
    for (int i = 0; i < Nei::NUMNEIGHBORS; ++i) {
        n[i] = NULL;
    }
    if (x > 0) {
        n[Nei::LEFT] = &f->at(x-1, y);
        if (y > 0) {
            n[Nei::ABOVELEFT] = &f->at(x-1, y-1);
        }
    }
    if (y > 0) {
        n[Nei::ABOVE] = &f->at(x, y-1);
        if (x + 1 < f->width) {
            n[Nei::ABOVERIGHT] = &f->at(x + 1, y - 1);
        }
    }
    if (f->priorValid) {
        n[Nei::PAST] = &f->last(x, y);
    }
}

int curBillTag = 0;
double bill[NUM_TOTAL_TAGS] = {0};
const char * billEnumToName(int en) {
    if (PIP_DEFAULT_TAG == en) return "default";
    if(PIP_SKIP_TAG == en) return "skip";
    if(PIP_SKIP_END_TAG == en) return "skip end";
    if(PIP_CBPC_TAG == en) return "cbpc";
    if(PIP_CBPL_TAG == en) return "cbpl";
    if(PIP_LAST_MB_TAG == en) return "last mb";
    if(PIP_QPL_TAG == en) return "qpl";
    if(PIP_MB_TYPE_TAG == en) return "mb type";
    if(PIP_REF_TAG == en) return "ref";
    if(PIP_8x8_TAG == en) return "8x8";
    if(PIP_16x16_TAG == en) return "16x16";
    if(PIP_PRED_TAG == en) return "pred";
    if(PIP_PRED_MODE_TAG == en) return "pred mode";
    if(PIP_SUB_MB_TAG == en) return "sub mb";
    if(PIP_MVX_TAG == en) return "mv[0]";
    if(PIP_MVY_TAG == en) return "mv[1]";
    if(PIP_LDC_TAG == en) return "ldc";
    if(PIP_CRDC_TAG == en) return "crdc";
    if(PIP_LAC_TAG0 == en) return "luma ac 0";
    if(PIP_LAC_TAG0 + 1 == en) return "luma ac 1";
    if(PIP_LAC_TAG0 + 2 == en) return "luma ac 2";
    if(PIP_LAC_TAG0 + 3 == en) return "luma ac 3";
    if(PIP_LAC_TAG0 + 4 == en) return "luma ac 4";
    if(PIP_LAC_TAG0 + 5 == en) return "luma ac 5";
    if(PIP_LAC_TAG0 + 6 == en) return "luma ac 6";
    if(PIP_LAC_TAG0 + 7 == en) return "luma ac 7";
    if(PIP_LAC_TAG0 + 8 == en) return "luma ac 8";
    if(PIP_LAC_TAG0 + 9 == en) return "luma ac 9";
    if(PIP_LAC_TAG0 + 10 == en) return "luma ac 10";
    if(PIP_LAC_TAG0 + 11 == en) return "luma ac 11";
    if(PIP_LAC_TAG0 + 12 == en) return "luma ac 12";
    if(PIP_LAC_TAG0 + 13 == en) return "luma ac 13";
    if(PIP_LAC_TAG0 + 14 == en) return "luma ac 14";
    if(PIP_LAC_TAG0 + 15 == en) return "luma ac 15";
   
    if(PIP_CRAC_TAG0 + 0 == en) return "chroma ac 0";
    if(PIP_CRAC_TAG0 + 1 == en) return "chroma ac 1";
    if(PIP_CRAC_TAG0 + 2 == en) return "chroma ac 2";
    if(PIP_CRAC_TAG0 + 3 == en) return "chroma ac 3";
    if(PIP_CRAC_TAG0 + 4 == en) return "chroma ac 4";
    if(PIP_CRAC_TAG0 + 5 == en) return "chroma ac 5";
    if(PIP_CRAC_TAG0 + 6 == en) return "chroma ac 6";
    if(PIP_CRAC_TAG0 + 7 == en) return "chroma ac 7";

    if(PIP_CRAC_TAG0 + 8 == en) return "chroma ac 8";
    if(PIP_CRAC_TAG0 + 9 == en) return "chroma ac 9";
    if(PIP_CRAC_TAG0 + 10 == en) return "chroma ac 10";
    if(PIP_CRAC_TAG0 + 11 == en) return "chroma ac 11";
    if(PIP_CRAC_TAG0 + 12 == en) return "chroma ac 12";
    if(PIP_CRAC_TAG0 + 13 == en) return "chroma ac 13";
    if(PIP_CRAC_TAG0 + 14 == en) return "chroma ac 14";
    if(PIP_CRAC_TAG0 + 15 == en) return "chroma ac 15";

    if(PIP_PREV_PRED_TAG == en) return "prev pred";
    if(PIP_PREV_PRED_MODE_TAG == en) return "prev pred mode";
    if(PIP_NZC_TAG == en) return "nonzero count";
    return "unknown";
}
#ifdef BILLING
struct BillTally {
    ~BillTally() {
        double total = 0;
        for (int i= 0;i  < NUM_TOTAL_TAGS;++i) {
            if (bill[i]) {
                double cur = (bill[i] / 8);
                total += cur;
                fprintf(stderr, "%d :: %f   [%s] \n", i, cur, billEnumToName(i));
            }
        }
        fprintf(stderr,"TOTAL: %f\n", total);
    }
} tallyAtEnd;
#endif
void MacroblockModel::initCurrentMacroblock(
                                            DecodedMacroblock *curMb,
                                            WelsDec::PWelsDecoderContext pCtx,
                                            const FreqImage*f,
                                            int mbx, int mby) {
    this->mb = curMb;
    this->pCtx = pCtx;
    this->n.init(f, mbx, mby);
}

uint16_t MacroblockModel::getAndUpdateMacroblockLumaNumNonzeros() {
    uint16_t retval = 0;
    memset(mb->numSubLumaNonzeros_, 0, sizeof(mb->numSubLumaNonzeros_));
    for (int i = 0; i < 256; ++i) {
        if (mb->odata.lumaAC[i]) {
            ++retval;
            ++mb->numSubLumaNonzeros_[i / 16];
        }
    }
    mb->numLumaNonzeros_ = retval;
    return retval;
}

uint8_t MacroblockModel::getAndUpdateMacroblockChromaNumNonzeros() {
    uint8_t retval = 0;
    memset(mb->numSubChromaNonzeros_, 0, sizeof(mb->numSubChromaNonzeros_));
    for (int i = 0; i < 128; ++i) {
        if (mb->odata.chromaAC[i]) {
            ++retval;
            ++mb->numSubChromaNonzeros_[i / 16];
        }
    }
    mb->numChromaNonzeros_ = retval;
    return retval;
}
Sirikata::Array1d<DynProb, 256>::Slice  MacroblockModel::getLumaNumNonzerosPrior() {
    int prior = 0;
    using namespace Nei;

    if (n[PAST]) {
        prior = n[PAST]->numLumaNonzeros_;
    } else {
        if (n[LEFT]) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 16; ++j) {
                    prior += n[LEFT]->odata.lumaAC[i * 64 + 48 + j]  ? 1 : 0;
                }
            }
        }
        if (n[ABOVE]) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 16; ++j) {
                    prior += n[ABOVE]->odata.lumaAC[64 * 3 + i * 16 + j] ? 1 : 0;
                }
            }
        }
    }
    return numNonZerosLumaPriors.at(prior, encodeMacroblockType(mb->uiMbType));
}
Sirikata::Array1d<DynProb, 128>::Slice  MacroblockModel::getChromaNumNonzerosPrior() {
    int prior = 0;
    using namespace Nei;

    if (n[PAST]) {
        prior = n[PAST]->numChromaNonzeros_;
    } else {
        prior = mb->numLumaNonzeros_ / 2;
    }
    return numNonZerosChromaPriors.at(prior, encodeMacroblockType(mb->uiMbType));
}
Sirikata::Array1d<DynProb, 16>::Slice MacroblockModel::getSubLumaNumNonzerosPrior(uint8_t i) {

    int prior = 0;
    using namespace Nei;

    if (n[PAST]) {
        prior = n[PAST]->numSubLumaNonzeros_[i];
    } else {
        prior = 0;
    }
    return numSubNonZerosLumaPriors.at(prior, encodeMacroblockType(mb->uiMbType));
}
Sirikata::Array1d<DynProb, 16>::Slice MacroblockModel::getSubChromaNumNonzerosPrior(uint8_t i) {

    int prior = 0;
    using namespace Nei;

    if (n[PAST]) {
        prior = n[PAST]->numSubChromaNonzeros_[i];
    } else {
        prior = mb->numSubLumaNonzeros_[i] / 2;
    }
    return numSubNonZerosChromaPriors.at(prior, encodeMacroblockType(mb->uiMbType));
}
Branch<4> MacroblockModel::getMacroblockTypePrior() {
    using namespace Nei;
    int leftType = 15;
    int aboveType = 15;
    int prevType = 15;
    int prior = 15;
    if (n[ABOVE]) {
        aboveType = encodeMacroblockType(n[ABOVE]->uiMbType);
        prior = aboveType;
    }
    if (n[LEFT]) {
        leftType = encodeMacroblockType(n[LEFT]->uiMbType);
        prior = leftType;
    }
    if (n[PAST]) {
        prevType = encodeMacroblockType(n[PAST]->uiMbType);
    }

    prior += prevType;
    return mbTypePriors.at(prior,(uint32_t)(
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
        case 0: // this doesn't get set in the roundtrip
            return 11;
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
