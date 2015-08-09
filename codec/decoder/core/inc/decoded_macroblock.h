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

  DecodedMacroblock()
      : eSliceType(), uiChromaQpIndexOffset(),
        iPrevIntra4x4PredMode(), iRemIntra4x4PredMode(), sMbMvp(),
        uiSubMbType(), iRefIdx(), uiCbpC(0), uiCbpL(0), iLastMbQp(0),
        uiChmaI8x8Mode(), uiLumaI16x16Mode(), iMbSkipRun(), uiMbType(0),
        uiNumRefIdxL0Active(), uiLumaQp() {
  }
  void preInit(const WelsDec::PSlice);
};

#endif
