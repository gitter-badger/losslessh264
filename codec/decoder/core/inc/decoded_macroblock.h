#ifndef _DECODED_MACROBLOCK_H_
#define _DECODED_MACROBLOCK_H_

struct DecodedMacroblock {
  struct RawDCTData {
    int16_t lumaDC[16];
    int16_t chromaDC[8];
    int16_t lumaAC[256];
    int16_t chromaAC[128];
    bool initialized;
  } odata;
  uint8_t eSliceType;
  uint8_t pTransformSize8x8Flag;
  int uiChromaQpIndexOffset;
  int32_t iBestIntra4x4PredMode[16];
  int16_t sMbMvp[16][2];
  int16_t sMbAbsoluteMv[16][2]; // absolute motion vectors. no need to serialized them
  int uiSubMbType[4];
  int8_t iRefIdx[4];
  uint32_t uiCbpC, uiCbpL;
  int32_t iLastMbQp;
  uint8_t uiChmaI8x8Mode;
  uint8_t uiLumaI16x16Mode;
  int32_t iMbSkipRun;
  uint32_t uiMbType;
  uint32_t uiNumRefIdxL0Active;
  uint32_t uiLumaQp;
  uint8_t numLumaNonzeros_;
  uint8_t numChromaNonzeros_;
  uint8_t numSubLumaNonzeros_[16];
  uint8_t numSubChromaNonzeros_[8];

  // populated by updateFrame
  bool isSkipped;
  uint16_t cachedSkips;
  uint32_t cachedDeltaLumaQp;
  enum QuantizationTargets {
      LUMA_AC_QUANT,
      LUMA_DC_QUANT,
      U_AC_QUANT,
      U_DC_QUANT,
      V_AC_QUANT,
      V_DC_QUANT,
      NUM_QUANTIZATIONS
  };
  // we call this function from the decoded state to seed the quantization parameters
  static int32_t getiResidualProperty(uint32_t uiMbType, bool dc, int color);
    //handy pointers to the quantization parameters
  const uint16_t *quantizationTable [NUM_QUANTIZATIONS];
  bool quantizationUseScalingList; // if this is true then the quantization tables are 4x bigger and get divided by 4...otherwise the quantization tables are the correct values to divide by but are indexed by (coef&7)

    // returns the numerator and denominator of the quantization table for a coefficient of type t
    std::pair<uint16_t, uint16_t> getQuantizationValue(int coef, QuantizationTargets t) {
        if (quantizationUseScalingList) {
            return std::pair<uint16_t, uint16_t>(quantizationTable[t][coef], 16);
        } else {
            return std::pair<uint16_t, uint16_t>(quantizationTable[t][coef & 0x7], 1);
        }
  }
  DecodedMacroblock() {
      memset(this, 0, sizeof(*this));
  }
  void preInit(const WelsDec::PSlice);

  const int16_t* getAC(int color, int subblockIndex = 0) const {
    assert((color == 0 && subblockIndex < 16 && subblockIndex >= 0) ||
            (color == 1 && subblockIndex < 4 && subblockIndex >= 0) ||
            (color == 2 && subblockIndex < 8 && subblockIndex >= 4));
    return &(color == 0 ? odata.lumaAC : odata.chromaAC)[16 * subblockIndex];
  }
  bool needParseTransformSize8x8(WelsDec::PPps pPps) {
    int pNoSubMbPartSizeLessThan8x8Flag = 1;
    if (uiMbType == MB_TYPE_8x8 || uiMbType == MB_TYPE_8x8_REF0) {
      for (int i = 0; i < 4; i++) {
        pNoSubMbPartSizeLessThan8x8Flag &= (uiSubMbType[i] == SUB_MB_TYPE_8x8);
      }
    }
    return (((uiMbType >= MB_TYPE_16x16 && uiMbType <= MB_TYPE_8x16)
      || pNoSubMbPartSizeLessThan8x8Flag)
     && (IS_INTER(uiMbType))
     && (uiCbpL > 0)
     // && (uiMbType != B_Direct_16x16 || direct_8x8_inference_flag)
     && (pPps->bTransform8x8ModeFlag));
  }

  int countSubblockNonzeros(int color, int subblockIndex) const {
    const int16_t* ac = getAC(color, subblockIndex);
    int nonzeros = 0;
    for (int i = 0; i < 16; i++) {
      if (ac[i] != 0) nonzeros++;
    }
    return nonzeros;
  }
  int countSubblockNonzeros8x8(int color, int subblockIndex) const {
    assert(color == 0);
    assert((subblockIndex & 3) == 0); // make sure we're an actual 8x8 block
    const int16_t* ac = getAC(color, subblockIndex);
    int nonzeros = 0;
    for (int i = 0; i < 64; i++) {
      if (ac[i] != 0) nonzeros++;
    }
    return nonzeros;
  }
};

struct FreqImage {
    std::vector<DecodedMacroblock>frame[2];
    unsigned int width;
    unsigned int height;
    bool priorValid;
    unsigned char cur;
    int lastFrameId;
    FreqImage () {
        cur = lastFrameId = 0;
        width = 0;
        height = 0;
        priorValid = false;
    }
    void updateFrame(int frame) {
        if (frame != lastFrameId) {
            cur = cur?0:1;
            lastFrameId = frame;
        }
        //fprintf(stderr, "priorValid: %d\n", priorValid);
        uint16_t contiguousSkips = 0;

        std::vector<DecodedMacroblock> &cur_frame = this->frame[1-cur];
        for (size_t i=0; i < cur_frame.size(); i++) {
            DecodedMacroblock &dmb = cur_frame[i];
            bool isZeroed = true;
            for (size_t j = 0; j < sizeof(dmb.odata.lumaAC) / sizeof(dmb.odata.lumaDC[0]); ++j) {
                if (dmb.odata.lumaAC[j] != 0) {
                    isZeroed = false;
                }
            }
            for (size_t j = 0; j < sizeof(dmb.odata.chromaAC) / sizeof(dmb.odata.chromaDC[0]); ++j) {
                if (dmb.odata.chromaAC[j] != 0) {
                    isZeroed = false;
                }
            }
            for(int j=0; j<16; j++) {
                if (dmb.odata.lumaDC[j] != 0) {
                    isZeroed = false;
                    break;
                }
            }
            for(int j=0; j<8; j++) {
                if (dmb.odata.chromaDC[j] != 0) {
                    isZeroed = false;
                    break;
                }
            }
            //fprintf(stderr, "isZeroed: %d contiguousSkips: %d\n", isZeroed, contiguousSkips);

            if (isZeroed) {
                dmb.isSkipped = true;
                contiguousSkips++;
            } else {
                dmb.isSkipped = false;
                for (int j = 0; j < contiguousSkips; j++) {
                    cur_frame[i - j].cachedSkips = contiguousSkips;
                }
                contiguousSkips = 0;
            }
        }
    }
    DecodedMacroblock& at(int x, int y) {
        return frame[cur][y * width + x];
    }
    DecodedMacroblock& at(int mb_index) {
        return frame[cur][mb_index];
    }
    DecodedMacroblock& last(int x, int y) {
        return frame[1 - cur][y * width + x];
    }
    DecodedMacroblock& last(int mb_index) {
        return frame[1 - cur][mb_index];
    }
    const DecodedMacroblock& at(int x, int y) const{
        return frame[cur][y * width + x];
    }
    const DecodedMacroblock& at(int mb_index) const{
        return frame[cur][mb_index];
    }
    const DecodedMacroblock& last(int x, int y) const{
        return frame[1 - cur][y * width + x];
    }
    const DecodedMacroblock& last(int mb_index) const{
        return frame[1 - cur][mb_index];
    }
};

#endif
