#ifndef _MACROBLOCK_MODEL_H_
#define _MACROBLOCK_MODEL_H_

#include "array_nd.h"
#include "compression_stream.h"
const char *billEnumToName(int en);
#define GENERATE_LZMA_MODE_FILE 1
#if GENERATE_LZMA_MODE_FILE
enum {
    PIP_DEFAULT_TAG,
    PIP_SKIP_TAG,
    PIP_SKIP_END_TAG,
    PIP_CBPC_TAG,
    PIP_CBPL_TAG,
    PIP_LAST_MB_TAG,
    PIP_QPL_TAG,
    PIP_MB_TYPE_TAG,
    PIP_REF_TAG,
    PIP_8x8_TAG,
    PIP_16x16_TAG,
    PIP_PRED_TAG,
    PIP_PRED_MODE_TAG,
    PIP_SUB_MB_TAG,
    PIP_MVX_TAG,
    PIP_MVY_TAG,
    PIP_LDC_TAG,
    PIP_CRDC_TAG,
    PIP_LAST_NONVAR_TAG
};
const int PIP_AC_STEP = 1;
enum {
    PIP_LAC_TAG0 = PIP_LAST_NONVAR_TAG,
    PIP_CRAC_TAG0 = PIP_LAST_NONVAR_TAG + 16,
    PIP_LAST_VAR_TAG = PIP_LAST_NONVAR_TAG + 32
};
enum {
    PIP_PREV_PRED_TAG = PIP_LAST_VAR_TAG,
    PIP_PREV_PRED_MODE_TAG,
    PIP_NZC_TAG,
    NUM_TOTAL_TAGS
};
#define BILLING
extern double bill[NUM_TOTAL_TAGS];
extern int curBillTag;
#else
enum {
    PIP_DEFAULT_TAG=1,
    PIP_SKIP_TAG=1,
    PIP_SKIP_END_TAG=1,
    PIP_CBPC_TAG=1,
    PIP_CBPL_TAG=1,
    PIP_LAST_MB_TAG=1,
    PIP_QPL_TAG=1,
    PIP_QPC_TAG=1,
    PIP_MB_TYPE_TAG=1,
    PIP_REF_TAG=1,
    PIP_8x8_TAG=1,
    PIP_16x16_TAG=1,
    PIP_PRED_TAG=1,
    PIP_PRED_MODE_TAG=1,
    PIP_SUB_MB_TAG=1,
    PIP_MVX_TAG=1,
    PIP_MVY_TAG=1,
    PIP_LDC_TAG=1,
    PIP_CRDC_TAG=1,
};
const int PIP_AC_STEP = 0;
enum {
    PIP_LAC_TAG0 = 1,
    PIP_CRAC_TAG0 = 1,
};
enum {
    PIP_PREV_PRED_TAG = 1,
    PIP_PREV_PRED_MODE_TAG=1,
    PIP_NZC_TAG=1
};
#endif
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
