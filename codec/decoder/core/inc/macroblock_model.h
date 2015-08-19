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
    PIP_LAST_MB_TAG,  // not needed & unused -- this can be computed from existing decoder state
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
    PIP_LAC_0_EOB,
    PIP_LAC_0_BITMASK,
    PIP_LAC_0_EXP, //must be contiguous
    PIP_LAC_0_RES, //must be contiguous
    PIP_LAC_0_SIGN, //must be contiguous
    PIP_LAC_N_EOB, //must be contiguous
    PIP_LAC_N_BITMASK, //must be contiguous
    PIP_LAC_N_EXP, //must be contiguous
    PIP_LAC_N_RES, //must be contiguous
    PIP_LAC_N_SIGN, //must be contiguous
    PIP_CRAC_EOB, //must be contiguous
    PIP_CRAC_BITMASK, //must be contiguous
    PIP_CRAC_EXP, //must be contiguous
    PIP_CRAC_RES, //must be contiguous
    PIP_CRAC_SIGN, //must be contiguous
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
struct DecodedMacroblock;
struct FreqImage;
namespace Nei {
    enum NeighborType{LEFT, ABOVE, ABOVELEFT, ABOVERIGHT, PAST, NUMNEIGHBORS};
}
struct Neighbors {

    const DecodedMacroblock *n[Nei::NUMNEIGHBORS];
    const DecodedMacroblock *operator[](Nei::NeighborType index) const {return n[index];}
    void init(const FreqImage *f, int x, int y);
};

namespace WelsDec{
    struct TagWelsDecoderContext;
    typedef struct TagWelsDecoderContext *PWelsDecoderContext;
}

// Utility functions for arithmetic
uint8_t log2(uint16_t v);
uint8_t bit_length(uint16_t value);
uint16_t swizzle_sign(int16_t v);
int16_t unswizzle_sign(uint16_t v);

class MacroblockModel {

    DecodedMacroblock *mb;
    WelsDec::PWelsDecoderContext pCtx;
    Neighbors n;
    Sirikata::Array3d<DynProb, 32, 2, 15> mbTypePriors; // We could use just 8 bits for I Slices
    typedef IntPrior<2, 4> MotionVectorDifferencePrior;
    MotionVectorDifferencePrior motionVectorDifferencePriors[200][16];
    Sirikata::Array2d<DynProb, 8, 8> lumaI16x16ModePriors;
    Sirikata::Array2d<DynProb, 8, 8> chromaI8x8ModePriors;
    std::pair<Sirikata::Array3d<DynProb, 8, 16, 16>, Sirikata::Array3d<DynProb, 8, 16, 16> > chromaDCPriors;
    std::pair<Sirikata::Array3d<DynProb, 16, 16, 16>, Sirikata::Array3d<DynProb, 16, 16, 16> > lumaDCPriors;
    Sirikata::Array3d<DynProb,
            512, // past
            16, //mbType
            511 // values
            > mbSkipRunPrior; // TODO: Assume max number of skips is 256
    Sirikata::Array2d<DynProb,
            2, // Whether it's the first MB.
            128 // Max I've seen the value is in the 30s, but give it some buffer. (twice as big b/c sign bit)
            > mbQPLPrior;
    Sirikata::Array3d<DynProb,
        257, // prev frame or neighbor 4x16 + 16x4
        16,//mbType
        256 // number of nonzero values possible
        > numNonZerosLumaPriors; // <-- deprecated
    Sirikata::Array3d<DynProb,
                      129, // prev frame num nonzeros
                      16, // mbType
                      128> numNonZerosChromaPriors; // <--deprecated
    Sirikata::Array4d<DynProb,
        17,//past
        17,//left
        17,//above
        16> numSubNonZerosLumaPriors; // <--deprecated
    Sirikata::Array4d<DynProb,
        17,//past
        17,//left
        17,//above
        16> numSubNonZerosChromaPriors; // <--deprecated
    Sirikata::Array6d<DynProb,
        16,//which coef
        3,//left_zero
        3,//above_zero
        3,//past_zero
        2,//coef above
        2// coef left
        > nonzeroBitmaskPriors;
    Sirikata::Array5d<DynProb, // <-- really bad priors
        17,//which coef -- 16 is the magic early exit bit
        17,//num_nonzeros
        3,//past_zero
        2,//coef above
        2// coef left
        > eobPriors;
    Sirikata::Array6d<DynProb,
        16,//which coef
        17,//num_nonzeros
        3,//past_zero
        2,//coef right nonzero
        2,// coef below nonzero
        15> acExpPriors;

    Sirikata::Array4d<DynProb,
        16,//which coef
        17,//num_nonzeros
        16,//exponent
        9> acSignificandPriors;
    Sirikata::Array3d<DynProb,
        16,//which coef
        17,//num_nonzeros
        3> acSignPriors;
    struct SingleCoefNeighbors {
        int16_t past;
        int16_t left;
        int16_t above;
        bool has_past;
        bool has_left;
        bool has_above;
    };
    Sirikata::Array2d<DynProb, 2, 15> predictionModePriors;
    SingleCoefNeighbors priorCoef(int index, int coef, int color);
public:
    void initCurrentMacroblock(DecodedMacroblock *curMb, WelsDec::PWelsDecoderContext pCtx,
                               const FreqImage *, int mbx, int mby);
    DynProb *getNonzeroBitmaskPrior(const bool *this_4x4, int index, int coef, bool emit_dc, int color);
    DynProb *getEOBPrior(const bool *this_4x4, int index, int coef, bool emit_dc, int color);
    Sirikata::Array1d<DynProb, 15>::Slice getAcExpPrior(const bool *nonzeros, const int16_t *ac, int index, int coef,
                            bool emit_dc, int color);
    DynProb *getAcSignificandPrior(const bool *nonzeros, const int16_t *ac, int index, int coef,
                                   bool emit_dc, int color,
                                   int bit_len, int which_bit, int significand_so_far);
    DynProb *getAcSignPrior(const bool *nonzeros, const int16_t *ac, int index, int coef,
                            int color);
    Branch<4> getMacroblockTypePrior();
    Branch<4> getPredictionModePrior();

#define DC_SPLIT 12
#define DC_LOWER_MASK ((1 << (15 - DC_SPLIT)) - 1)
    static std::pair<uint16_t, uint16_t> splitDC(int16_t val) {
      // Follows power distribution. 0 is prevalent, then +/-1, and so on.
      // We will move the sign bit to the end and do a 12-4 split.
      uint16_t v = swizzle_sign(val);
      return std::make_pair<uint16_t, uint16_t>(v >> (15 - DC_SPLIT),
						v & DC_LOWER_MASK);
    }
    static int16_t mergeDC(uint16_t first, uint8_t second) {
      return unswizzle_sign(first << (15 - DC_SPLIT) | second);
    }

    template<size_t size> struct DCStatistics {
      uint8_t nonzero_first_count;
      uint8_t nonzero_second_count;
      uint16_t first[size];
      uint16_t second[size];
    };

    template<size_t size> static DCStatistics<size> computeDCStatistics(int16_t* coeffs) {
      DCStatistics<size> ret;
      ret.nonzero_first_count = 0u;
      ret.nonzero_second_count = 0u;
      for (size_t i = 0; i < size; i++) {
	std::pair<uint16_t, uint16_t> split = splitDC(coeffs[i]);
	ret.first[i] = split.first;
	ret.second[i] = split.second;
	ret.nonzero_first_count += ret.first[i] == 0 ? 0 : 1;
	ret.nonzero_second_count += ret.second[i] == 0 ? 0 : 1;
      }
      return ret;
    }

    std::pair<Sirikata::Array1d<DynProb, 16>::Slice,
      Sirikata::Array1d<DynProb, 1<<(16-DC_SPLIT)>::Slice> getChromaDCPriors(size_t index);
    std::pair<Sirikata::Array1d<DynProb, 16>::Slice,
      Sirikata::Array1d<DynProb, 1<<(16-DC_SPLIT)>::Slice> getLumaDCPriors(size_t index);
    std::pair<Sirikata::Array1d<DynProb, 16>::Slice,
      Sirikata::Array1d<DynProb, 1<<(16-DC_SPLIT)>::Slice> _getDCPriorsHelper(bool is_luma, size_t index);

    // Returns a prior distribution over deltas, and a base value for the delta, for sMbMvp[subblockIndex][xyIndex].
    std::pair<MotionVectorDifferencePrior*, int> getMotionVectorDifferencePrior(int subblockIndex, int xyIndex);

    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> getLumaI16x16ModePrior();
    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> getChromaI8x8ModePrior();
    Sirikata::Array1d<DynProb, 511>::Slice getSkipRunPrior();
    Branch<9> getSkipRunPriorBranch() {
        return getSkipRunPrior().slice<0, 511>();
    }
    Sirikata::Array1d<DynProb, 128>::Slice getQPLPrior(bool isFirstMB);
    Sirikata::Array1d<DynProb, 256>::Slice getLumaNumNonzerosPrior();
    Sirikata::Array1d<DynProb, 128>::Slice getChromaNumNonzerosPrior();
    Branch<8> getLumaNumNonzerosPriorBranch() {
        return getLumaNumNonzerosPrior().slice<1, 256>();
    }
    Branch<7> getChromaNumNonzerosPriorBranch() {
        return getChromaNumNonzerosPrior().slice<1, 128>();
    }
    Sirikata::Array1d<DynProb, 16>::Slice getSubLumaNumNonzerosPrior(uint8_t i, uint8_t runningCount);
    Sirikata::Array1d<DynProb, 16>::Slice getSubChromaNumNonzerosPrior(uint8_t i, uint8_t runningCount);
    uint16_t getAndUpdateMacroblockLumaNumNonzeros(); // between 0 and 256, inclusive
    uint8_t getAndUpdateMacroblockChromaNumNonzeros(); // between 0 and 128, inclusive
    int encodeMacroblockType(int welsType);
    int decodeMacroblockType(int storedType);
    uint8_t get4x4NumNonzeros(uint8_t index, uint8_t color/*0 for Y 1 for U, 2 for V*/) const;
    // this is just a sanity check
    void checkSerializedNonzeros(const bool *nonzeros, const int16_t *ac,
                                 int index, bool emit_dc, int color);
};

#endif
