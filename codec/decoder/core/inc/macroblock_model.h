#ifndef _MACROBLOCK_MODEL_H_
#define _MACROBLOCK_MODEL_H_
#include "billing.h"
#include "array_nd.h"
#include "compression_stream.h"

const char *billEnumToName(int en);
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
uint8_t ilog2(uint16_t v);
uint8_t bit_length(uint16_t value);
uint16_t swizzle_sign(int16_t v);
int16_t unswizzle_sign(uint16_t v);

class MacroblockModel {
public:
    DecodedMacroblock *mb;
    WelsDec::PWelsDecoderContext pCtx;
    Neighbors n;
    Sirikata::Array3d<DynProb, 32, 2, 15> mbTypePriors; // We could use just 8 bits for I Slices
    typedef UEGkIntPrior<9, 4, 3, 4, 3> MotionVectorDifferencePrior;
    Sirikata::Array2d<MotionVectorDifferencePrior, 200, 16> motionVectorDifferencePriors;
    Sirikata::Array2d<DynProb, 8, 8> lumaI16x16ModePriors;
    Sirikata::Array2d<DynProb, 8, 8> chromaI8x8ModePriors;
    typedef IntPrior<3, 4> DCPrior;
    Sirikata::Array3d<DCPrior, 16, 5, 16> lumaDCIntPriors;
    Sirikata::Array3d<DCPrior, 8, 5, 16> chromaDCIntPriors;

    //typedef IntPrior<2, 4> ACPrior;
    typedef UEGkIntPrior<14, 4, 2, 4, 0> ACPrior;
    typedef UnsignedIntPrior<3, 4> NonzerosPrior;
    Sirikata::Array6d<NonzerosPrior, 5, 16, 3, 3, 3, 3> nonzerosPriors;  // eSliceType, mbType, color, past, left, above
    Sirikata::Array6d<NonzerosPrior, 5, 16, 3, 3, 3, 3> nonzerosPriors8x8;  // eSliceType, mbType, color, past, left, above
    Sirikata::Array4d<Sirikata::Array5d<ACPrior, 5, 5, 5, 5, 5>,// nonzeros, prev, prev2, left, above
                      5, 16, 3, 16> acPriors; // eSliceType, mbType, color, index,

    Sirikata::Array3d<DynProb,
            512, // past
            16, // mbType
            511 // values
            > mbSkipRunPrior; // TODO: Assume max number of skips is 256
    Sirikata::Array3d<DynProb,
            2, // Whether it's the first MB.
            3, // sign of the last delta
            128 // Max I've seen the value is in the 30s, but give it some buffer. (twice as big b/c sign bit)
            > mbQPLPrior;
    Sirikata::Array2d<DynProb,
            16, // mbType
            255 // values
            > subMbPriors;
    Sirikata::Array3d<DynProb,
            16, // past
            16, // mbType
            15 // values
            > numRefIdxL0ActivePrior;
    Sirikata::Array3d<DynProb,
            4, //past
            16, // mbType
            3 //values
            > CbpCPrior;
    Sirikata::Array3d<DynProb,
            16, //past
            16, // mbType
            15 //values
            > CbpLPrior;
    Sirikata::Array6d<DynProb,
      17, // index
      17, // # of checks
      5, // slicetype
      16, // mbtype
      3, // color
      2 // nonzero prior
      > nonzeroBitmaskPriors;
    Sirikata::Array6d<DynProb,
        17,//index
        18, // # of nonzeros from past
        17,// # of checks
        5, //slicetype
        16,//mbtype
        3 //color
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
    Sirikata::Array1d<DynProb,
        2048// num macroblocks in slice
        > stopBitPriors;
    Sirikata::Array2d<DynProb,
        16, // mb type
        128 // QPL
        > transform8x8FlagPriors;
    struct SingleCoefNeighbors {
        int16_t past;
        int16_t left;
        int16_t above;
        bool has_past;
        bool has_left;
        bool has_above;
    };
    Sirikata::Array4d<DynProb,
        16,
        8,
        9,
        15> predictionModePriors;
    template<unsigned int block_size>SingleCoefNeighbors priorCoef(int index, int coef, int color);
public:
    void initCurrentMacroblock(DecodedMacroblock *curMb, WelsDec::PWelsDecoderContext pCtx,
                               const FreqImage *, int mbx, int mby);
    Sirikata::Array1d<DynProb, 15>::Slice getAcExpPrior(const bool *nonzeros, const int16_t *ac, int index, int coef,
                            bool emit_dc, int color);
    DynProb *getAcSignificandPrior(const bool *nonzeros, const int16_t *ac, int index, int coef,
                                   bool emit_dc, int color,
                                   int bit_len, int which_bit, int significand_so_far);
    DynProb *getAcSignPrior(const bool *nonzeros, const int16_t *ac, int index, int coef,
                            int color);

    DynProb *getTransformSize8x8FlagPrior(int mbType, int qpl);
    Branch<4> getMacroblockTypePrior();
    Branch<4> getPredictionModePrior(int predMode, int leftAvail, int topAvail, int leftTopAvail);
    DCPrior* getLumaDCIntPrior(size_t index);
    DCPrior* getChromaDCIntPrior(size_t index);

    NonzerosPrior* getNonzerosPrior8x8(int color, int subblockIndex);
    NonzerosPrior* getNonzerosPrior4x4(int color, int subblockIndex);
    template<unsigned int block_size> NonzerosPrior* getNonzerosPrior(int color, int subblockIndex){
        if (block_size == 64) {
            return getNonzerosPrior8x8(color, subblockIndex);
        } else {
            return getNonzerosPrior4x4(color, subblockIndex);
        }
    }
    ACPrior* getACPrior8x8(int index, int coef, int color, const std::vector<int>& emitted, int nonzeros);
    ACPrior* getACPrior4x4(int index, int coef, int color, const std::vector<int>& emitted, int nonzeros);
    template<unsigned int block_size> ACPrior* getACPrior(int index, int coef, int color, const std::vector<int>& emitted, int nonzeros) {
        if (block_size == 64) {
            return this->getACPrior8x8(index, coef, color, emitted, nonzeros);
        }
        return this->getACPrior4x4(index, coef, color, emitted, nonzeros);
    }

    // Returns a prior distribution over deltas, and a base value for the delta, for sMbMvp[subblockIndex][xyIndex].
    std::pair<MotionVectorDifferencePrior*, int> getMotionVectorDifferencePrior(int subblockIndex, int xyIndex);

    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> getLumaI16x16ModePrior();
    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> getChromaI8x8ModePrior();
    Sirikata::Array1d<DynProb, 511>::Slice getSkipRunPrior();
    DynProb* getStopBitPrior(int numMacroblocksThisSlice);
    Branch<9> getSkipRunPriorBranch() {
        return getSkipRunPrior().slice<0, 511>();
    }
    Branch<4> getNumRefIdxL0ActivePrior();
    Branch<2> getCbpCPrior();
    Branch<4> getCbpLPrior();
    Sirikata::Array1d<DynProb, 128>::Slice getQPLPrior(bool isFirstMB, int32_t lastNonzeroDeltaLumaQp);
    Branch<8> getSubMbPrior(int which);
    uint16_t getAndUpdateMacroblockLumaNumNonzeros(); // between 0 and 256, inclusive
    uint8_t getAndUpdateMacroblockChromaNumNonzeros(); // between 0 and 128, inclusive
    int encodeMacroblockType(int welsType);
    int decodeMacroblockType(int storedType);
    uint8_t get4x4NumNonzeros(uint8_t index, uint8_t color/*0 for Y 1 for U, 2 for V*/) const;
    // this is just a sanity check
    void checkSerializedNonzeros(const bool *nonzeros, const int16_t *ac,
                                 int index, bool emit_dc, int color);
    const Neighbors& getNeighbors() { return n; }
    const int getPastNonzeroACCount(int index, int color); // return -1 if no PAST
    const bool isPastCorrespondingACNonzero(int index, int coef, int color); // return 0 if no PAST
};

#endif
