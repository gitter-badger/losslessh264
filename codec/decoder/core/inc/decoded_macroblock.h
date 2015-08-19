#ifndef _DECODED_MACROBLOCK_H_
#define _DECODED_MACROBLOCK_H_

struct DecodedMacroblock {
  struct RawDCTData {
    int16_t lumaDC[16];
    int16_t chromaDC[8];
    int16_t lumaAC[256];
    int16_t chromaAC[128];
  } odata;
  uint8_t eSliceType;
  int uiChromaQpIndexOffset;
  int32_t iPrevIntra4x4PredMode[16];
  int32_t iRemIntra4x4PredMode[16];
  int16_t sMbMvp[16][2];
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

  DecodedMacroblock()
      : eSliceType(), uiChromaQpIndexOffset(),
        iPrevIntra4x4PredMode(), iRemIntra4x4PredMode(), sMbMvp(),
        uiSubMbType(), iRefIdx(), uiCbpC(0), uiCbpL(0), iLastMbQp(0),
        uiChmaI8x8Mode(), uiLumaI16x16Mode(), iMbSkipRun(), uiMbType(0),
        uiNumRefIdxL0Active(), uiLumaQp(),
        numLumaNonzeros_(0), numChromaNonzeros_(0), numSubLumaNonzeros_(), numSubChromaNonzeros_(),
        cachedSkips(0) {

      isSkipped = false;
      cachedSkips = 0;
  }
  void preInit(const WelsDec::PSlice);
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
        for (int i=0; i < cur_frame.size(); i++) {
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
