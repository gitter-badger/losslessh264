/*!
 * \copy
 *     Copyright (c)  2008-2013, Cisco Systems
 *     All rights reserved.
 *
 *     Redistribution and use in source and binary forms, with or without
 *     modification, are permitted provided that the following conditions
 *     are met:
 *
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *     POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  Abstract
 *      current slice decoding
 *
 *  History
 *      07/10/2008 Created
 *      08/09/2013 Modified
 *
 *****************************************************************************/


#include "deblocking.h"

#include "decode_slice.h"

#include "parse_mb_syn_cavlc.h"
#include "parse_mb_syn_cabac.h"
#include "rec_mb.h"
#include "mv_pred.h"

#include "cpu_core.h"
#include "compression_stream.h"

#include "encoder_from_decoder.h"

// Pretend this isn't an ugly hack.
namespace WelsEnc {
int32_t WelsUtilWriteMbResidual (SWelsFuncPtrList* pFuncList, uint32_t uiMbType,
    const int32_t kiCbpChroma, const int32_t kiCbpLuma,
    int8_t* pNonZeroCoeffCount, int16_t iLumaI16x16Dc[16],
    int16_t iLumaBlock[256], int16_t iChromaDc[8],
    int16_t iChromaBlock[128], SBitStringAux* pBs);
}

namespace WelsDec {
struct RawDCTData {
    bool pPrevIntra4x4PredModeFlag[16];
    uint8_t pRemIntra4x4PredModeFlag[16];
    uint8_t uiChmaI8x8Mode;
    int16_t lumaDC[16];
    int16_t chromaDC[8];
    int16_t lumaAC[256];
    int16_t chromaAC[128];
};

int32_t WelsTargetSliceConstruction (PWelsDecoderContext pCtx) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  PSlice pCurSlice = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader = &pCurSlice->sSliceHeaderExt.sSliceHeader;

  int32_t iTotalMbTargetLayer = pSliceHeader->pSps->uiTotalMbCount;

  int32_t iCurLayerWidth  = pCurLayer->iMbWidth << 4;
  int32_t iCurLayerHeight = pCurLayer->iMbHeight << 4;

  int32_t iNextMbXyIndex = 0;
  PFmo pFmo = pCtx->pFmo;

  int32_t iTotalNumMb = pCurSlice->iTotalMbInCurSlice;
  int32_t iCountNumMb = 0;
  PDeblockingFilterMbFunc pDeblockMb;

  if (!pCtx->bAvcBasedFlag && iCurLayerWidth != pCtx->iCurSeqIntervalMaxPicWidth) {
    return -1;
  }

  iNextMbXyIndex   = pSliceHeader->iFirstMbInSlice;
  pCurLayer->iMbX  = iNextMbXyIndex % pCurLayer->iMbWidth;
  pCurLayer->iMbY  = iNextMbXyIndex / pCurLayer->iMbWidth;
  pCurLayer->iMbXyIndex = iNextMbXyIndex;

  if (0 == iNextMbXyIndex) {
    pCurLayer->pDec->iSpsId = pCtx->pSps->iSpsId;
    pCurLayer->pDec->iPpsId = pCtx->pPps->iPpsId;

    pCurLayer->pDec->uiQualityId = pCurLayer->sLayerInfo.sNalHeaderExt.uiQualityId;
  }

  do {
    if (iCountNumMb >= iTotalNumMb) {
      break;
    }

    if (!pCtx->bParseOnly) { //for parse only, actual recon MB unnecessary
      if (WelsTargetMbConstruction (pCtx)) {
        WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
                 "WelsTargetSliceConstruction():::MB(%d, %d) construction error. pCurSlice_type:%d",
                 pCurLayer->iMbX, pCurLayer->iMbY, pCurSlice->eSliceType);

        return -1;
      }
    }

    ++iCountNumMb;
    if (!pCurLayer->pMbCorrectlyDecodedFlag[iNextMbXyIndex]) { //already con-ed, overwrite
      pCurLayer->pMbCorrectlyDecodedFlag[iNextMbXyIndex] = true;
      pCtx->pDec->iMbEcedPropNum += (pCurLayer->pMbRefConcealedFlag[iNextMbXyIndex] ? 1 : 0);
      ++pCtx->iTotalNumMbRec;
    }

    if (pCtx->iTotalNumMbRec > iTotalMbTargetLayer) {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
               "WelsTargetSliceConstruction():::pCtx->iTotalNumMbRec:%d, iTotalMbTargetLayer:%d",
               pCtx->iTotalNumMbRec, iTotalMbTargetLayer);

      return -1;
    }

    if (pSliceHeader->pPps->uiNumSliceGroups > 1) {
      iNextMbXyIndex = FmoNextMb (pFmo, iNextMbXyIndex);
    } else {
      ++iNextMbXyIndex;
    }
    if (-1 == iNextMbXyIndex || iNextMbXyIndex >= iTotalMbTargetLayer) { // slice group boundary or end of a frame
      break;
    }
    pCurLayer->iMbX  = iNextMbXyIndex % pCurLayer->iMbWidth;
    pCurLayer->iMbY  = iNextMbXyIndex / pCurLayer->iMbWidth;
    pCurLayer->iMbXyIndex = iNextMbXyIndex;
  } while (1);

  pCtx->pDec->iWidthInPixel  = iCurLayerWidth;
  pCtx->pDec->iHeightInPixel = iCurLayerHeight;

  if ((pCurSlice->eSliceType != I_SLICE) && (pCurSlice->eSliceType != P_SLICE))
    return 0;

  if (pCtx->bParseOnly) //for parse only, deblocking should not go on
    return 0;

  pDeblockMb = WelsDeblockingMb;

  if (1 == pSliceHeader->uiDisableDeblockingFilterIdc
      || pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.iTotalMbInCurSlice <= 0) {
    return 0;//NO_SUPPORTED_FILTER_IDX
  } else {
    WelsDeblockingFilterSlice (pCtx, pDeblockMb);
  }
  // any other filter_idc not supported here, 7/22/2010

  return 0;
}

int32_t WelsMbInterSampleConstruction (PWelsDecoderContext pCtx, PDqLayer pCurLayer,
                                       uint8_t* pDstY, uint8_t* pDstU, uint8_t* pDstV, int32_t iStrideL, int32_t iStrideC) {
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t i, iIndex, iOffset;

  WelsChromaDcIdct (pCurLayer->pScaledTCoeff[iMbXy] + 256);     // 256 = 16*16
  WelsChromaDcIdct (pCurLayer->pScaledTCoeff[iMbXy] + 320);     // 320 = 16*16 + 16*4

  if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
    for (i = 0; i < 4; i++) {
      iIndex = g_kuiMbCountScan4Idx[i << 2];
      if (pCurLayer->pNzc[iMbXy][iIndex] || pCurLayer->pNzc[iMbXy][iIndex + 1] || pCurLayer->pNzc[iMbXy][iIndex + 4]
          || pCurLayer->pNzc[iMbXy][iIndex + 5]) {
        iOffset = ((iIndex >> 2) << 2) * iStrideL + ((iIndex % 4) << 2);
        pCtx->pIdctResAddPredFunc8x8 (pDstY + iOffset, iStrideL, pCurLayer->pScaledTCoeff[iMbXy] + (i << 6));
      }
    }
  } else {
    for (i = 0; i < 16; i++) { //luma
      iIndex = g_kuiMbCountScan4Idx[i];
      if (pCurLayer->pNzc[iMbXy][iIndex]) {
        iOffset = ((iIndex >> 2) << 2) * iStrideL + ((iIndex % 4) << 2);
        pCtx->pIdctResAddPredFunc (pDstY + iOffset, iStrideL, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4));
      }
    }
  }

  for (i = 0; i < 4; i++) { //chroma
    iIndex = g_kuiMbCountScan4Idx[i + 16]; //Cb
    if (pCurLayer->pNzc[iMbXy][iIndex] || * (pCurLayer->pScaledTCoeff[iMbXy] + ((i + 16) << 4))) {
      iOffset = (((iIndex - 16) >> 2) << 2) * iStrideC + (((iIndex - 16) % 4) << 2);
      pCtx->pIdctResAddPredFunc (pDstU + iOffset, iStrideC, pCurLayer->pScaledTCoeff[iMbXy] + ((i + 16) << 4));
    }

    iIndex = g_kuiMbCountScan4Idx[i + 20]; //Cr
    if (pCurLayer->pNzc[iMbXy][iIndex] || * (pCurLayer->pScaledTCoeff[iMbXy] + ((i + 20) << 4))) {
      iOffset = (((iIndex - 18) >> 2) << 2) * iStrideC + (((iIndex - 18) % 4) << 2);
      pCtx->pIdctResAddPredFunc (pDstV + iOffset, iStrideC , pCurLayer->pScaledTCoeff[iMbXy] + ((i + 20) << 4));
    }
  }

  return 0;
}
int32_t WelsMbInterConstruction (PWelsDecoderContext pCtx, PDqLayer pCurLayer) {
  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  uint8_t*  pDstY, *pDstCb, *pDstCr;

  int32_t iLumaStride   = pCtx->pDec->iLinesize[0];
  int32_t iChromaStride = pCtx->pDec->iLinesize[1];

  pDstY  = pCurLayer->pDec->pData[0] + ((iMbY * iLumaStride + iMbX) << 4);
  pDstCb = pCurLayer->pDec->pData[1] + ((iMbY * iChromaStride + iMbX) << 3);
  pDstCr = pCurLayer->pDec->pData[2] + ((iMbY * iChromaStride + iMbX) << 3);

  GetInterPred (pDstY, pDstCb, pDstCr, pCtx);
  WelsMbInterSampleConstruction (pCtx, pCurLayer, pDstY, pDstCb, pDstCr, iLumaStride, iChromaStride);

  pCtx->sBlockFunc.pWelsSetNonZeroCountFunc (
    pCurLayer->pNzc[pCurLayer->iMbXyIndex]); // set all none-zero nzc to 1; dbk can be opti!
  return 0;
}

void WelsLumaDcDequantIdct (int16_t* pBlock, int32_t iQp, PWelsDecoderContext pCtx) {
  const int32_t kiQMul = pCtx->bUseScalingList ? pCtx->pDequant_coeff4x4[0][iQp][0] >> 4 : g_kuiDequantCoeff[iQp][0];
#define STRIDE 16
  int32_t i;
  int32_t iTemp[16]; //FIXME check if this is a good idea
  int16_t* pBlk = pBlock;
  static const int32_t kiXOffset[4] = {0, STRIDE, STRIDE << 2,  5 * STRIDE};
  static const int32_t kiYOffset[4] = {0, STRIDE << 1, STRIDE << 3, 10 * STRIDE};

  for (i = 0; i < 4; i++) {
    const int32_t kiOffset = kiYOffset[i];
    const int32_t kiX1 = kiOffset + kiXOffset[2];
    const int32_t kiX2 = STRIDE + kiOffset;
    const int32_t kiX3 = kiOffset + kiXOffset[3];
    const int32_t kiI4 = i << 2; // 4*i
    const int32_t kiZ0 = pBlk[kiOffset] + pBlk[kiX1];
    const int32_t kiZ1 = pBlk[kiOffset] - pBlk[kiX1];
    const int32_t kiZ2 = pBlk[kiX2] - pBlk[kiX3];
    const int32_t kiZ3 = pBlk[kiX2] + pBlk[kiX3];

    iTemp[kiI4]  = kiZ0 + kiZ3;
    iTemp[1 + kiI4] = kiZ1 + kiZ2;
    iTemp[2 + kiI4] = kiZ1 - kiZ2;
    iTemp[3 + kiI4] = kiZ0 - kiZ3;
  }

  for (i = 0; i < 4; i++) {
    const int32_t kiOffset = kiXOffset[i];
    const int32_t kiI4 = 4 + i;
    const int32_t kiZ0 = iTemp[i] + iTemp[4 + kiI4];
    const int32_t kiZ1 = iTemp[i] - iTemp[4 + kiI4];
    const int32_t kiZ2 = iTemp[kiI4] - iTemp[8 + kiI4];
    const int32_t kiZ3 = iTemp[kiI4] + iTemp[8 + kiI4];

    pBlk[kiOffset] = ((kiZ0 + kiZ3) * kiQMul + 2) >> 2; //FIXME think about merging this into decode_resdual
    pBlk[kiYOffset[1] + kiOffset] = ((kiZ1 + kiZ2) * kiQMul + 2) >> 2;
    pBlk[kiYOffset[2] + kiOffset] = ((kiZ1 - kiZ2) * kiQMul + 2) >> 2;
    pBlk[kiYOffset[3] + kiOffset] = ((kiZ0 - kiZ3) * kiQMul + 2) >> 2;
  }
#undef STRIDE
}

int32_t WelsMbIntraPredictionConstruction (PWelsDecoderContext pCtx, PDqLayer pCurLayer, bool bOutput) {
//seems IPCM should not enter this path
  int32_t iMbXy = pCurLayer->iMbXyIndex;

  WelsFillRecNeededMbInfo (pCtx, bOutput, pCurLayer);

  if (IS_INTRA16x16 (pCurLayer->pMbType[iMbXy])) {
    WelsLumaDcDequantIdct (pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx);
    RecI16x16Mb (iMbXy, pCtx, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer);

    return 0;
  }

  if (IS_INTRA8x8 (pCurLayer->pMbType[iMbXy])) {
    RecI8x8Mb (iMbXy, pCtx, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer);
  }

  if (IS_INTRA4x4 (pCurLayer->pMbType[iMbXy]))
    RecI4x4Mb (iMbXy, pCtx, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer);

  return 0;
}

int32_t WelsMbInterPrediction (PWelsDecoderContext pCtx, PDqLayer pCurLayer) {
  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  uint8_t*  pDstY, *pDstCb, *pDstCr;

  int32_t iLumaStride   = pCtx->pDec->iLinesize[0];
  int32_t iChromaStride = pCtx->pDec->iLinesize[1];

  pDstY  = pCurLayer->pDec->pData[0] + ((iMbY * iLumaStride + iMbX) << 4);
  pDstCb = pCurLayer->pDec->pData[1] + ((iMbY * iChromaStride + iMbX) << 3);
  pDstCr = pCurLayer->pDec->pData[2] + ((iMbY * iChromaStride + iMbX) << 3);

  GetInterPred (pDstY, pDstCb, pDstCr, pCtx);

  return 0;
}

int32_t WelsTargetMbConstruction (PWelsDecoderContext pCtx) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  if (MB_TYPE_INTRA_PCM == pCurLayer->pMbType[pCurLayer->iMbXyIndex]) {
    //already decoded and reconstructed when parsing
    return 0;
  } else if (IS_INTRA (pCurLayer->pMbType[pCurLayer->iMbXyIndex])) {
    WelsMbIntraPredictionConstruction (pCtx, pCurLayer, 1);
  } else if (IS_INTER (pCurLayer->pMbType[pCurLayer->iMbXyIndex])) { //InterMB
    if (0 == pCurLayer->pCbp[pCurLayer->iMbXyIndex]) { //uiCbp==0 include SKIP
      WelsMbInterPrediction (pCtx, pCurLayer);
    } else {
      WelsMbInterConstruction (pCtx, pCurLayer);
    }
  } else {
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "WelsTargetMbConstruction():::::Unknown MB type: %d",
             pCurLayer->pMbType[pCurLayer->iMbXyIndex]);
    return -1;
  }

  return 0;
}

void WelsChromaDcIdct (int16_t* pBlock) {
  int32_t iStride = 32;
  int32_t iXStride = 16;
  int32_t iStride1 = iXStride + iStride;
  int16_t* pBlk = pBlock;
  int32_t iA, iB, iC, iD, iE;

  iA = pBlk[0];
  iB = pBlk[iXStride];
  iC = pBlk[iStride];
  iD = pBlk[iStride1];

  iE = iA - iB;
  iA += iB;
  iB = iC - iD;
  iC += iD;

  pBlk[0] = (iA + iC) >> 1;
  pBlk[iXStride] = (iE + iB) >> 1;
  pBlk[iStride] = (iA - iC) >> 1;
  pBlk[iStride1] = (iE - iB) >> 1;
}

void WelsMapNxNNeighToSampleNormal (PWelsNeighAvail pNeighAvail, int32_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail) {  //left
    pSampleAvail[ 6] =
      pSampleAvail[12] =
        pSampleAvail[18] =
          pSampleAvail[24] = 1;
  }
  if (pNeighAvail->iLeftTopAvail) { //top_left
    pSampleAvail[0] = 1;
  }
  if (pNeighAvail->iTopAvail) { //top
    pSampleAvail[1] =
      pSampleAvail[2] =
        pSampleAvail[3] =
          pSampleAvail[4] = 1;
  }
  if (pNeighAvail->iRightTopAvail) { //top_right
    pSampleAvail[5] = 1;
  }
}

void WelsMapNxNNeighToSampleConstrain1 (PWelsNeighAvail pNeighAvail, int32_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail && IS_INTRA (pNeighAvail->iLeftType)) {   //left
    pSampleAvail[ 6] =
      pSampleAvail[12] =
        pSampleAvail[18] =
          pSampleAvail[24] = 1;
  }
  if (pNeighAvail->iLeftTopAvail && IS_INTRA (pNeighAvail->iLeftTopType)) {  //top_left
    pSampleAvail[0] = 1;
  }
  if (pNeighAvail->iTopAvail && IS_INTRA (pNeighAvail->iTopType)) {  //top
    pSampleAvail[1] =
      pSampleAvail[2] =
        pSampleAvail[3] =
          pSampleAvail[4] = 1;
  }
  if (pNeighAvail->iRightTopAvail && IS_INTRA (pNeighAvail->iRightTopType)) {  //top_right
    pSampleAvail[5] = 1;
  }
}
void WelsMap16x16NeighToSampleNormal (PWelsNeighAvail pNeighAvail, uint8_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail) {
    *pSampleAvail = (1 << 2);
  }
  if (pNeighAvail->iLeftTopAvail) {
    *pSampleAvail |= (1 << 1);
  }
  if (pNeighAvail->iTopAvail) {
    *pSampleAvail |= 1;
  }
}

void WelsMap16x16NeighToSampleConstrain1 (PWelsNeighAvail pNeighAvail, uint8_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail && IS_INTRA (pNeighAvail->iLeftType)) {
    *pSampleAvail = (1 << 2);
  }
  if (pNeighAvail->iLeftTopAvail && IS_INTRA (pNeighAvail->iLeftTopType)) {
    *pSampleAvail |= (1 << 1);
  }
  if (pNeighAvail->iTopAvail && IS_INTRA (pNeighAvail->iTopType)) {
    *pSampleAvail |= 1;
  }
}

int32_t ParseIntra4x4Mode (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, int8_t* pIntraPredMode,
                           PBitStringAux pBs, PDqLayer pCurDqLayer, RoundTripData *rtd) {
  int32_t iSampleAvail[5 * 6] = { 0 }; //initialize as 0
  int32_t iMbXy = pCurDqLayer->iMbXyIndex;
  int32_t iFinalMode, i;

  uint8_t uiNeighAvail = 0;
  uint32_t uiCode;
  int32_t iCode;
  pCtx->pMapNxNNeighToSampleFunc (pNeighAvail, iSampleAvail);
  uiNeighAvail = (iSampleAvail[6] << 2) | (iSampleAvail[0] << 1) | (iSampleAvail[1]);
  for (i = 0; i < 16; i++) {
    int32_t iPrevIntra4x4PredMode = 0;
    rtd->iRemIntra4x4PredMode[i] = 0;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      WELS_READ_VERIFY (ParseIntraPredModeLumaCabac (pCtx, iCode));
      iPrevIntra4x4PredMode = iCode;
    } else {
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode));
      iPrevIntra4x4PredMode = uiCode;
    }
    const int32_t kiPredMode = PredIntra4x4Mode (pIntraPredMode, i);

    int8_t iBestMode;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      if (iPrevIntra4x4PredMode == -1)
        iBestMode = kiPredMode;
      else
        iBestMode = iPrevIntra4x4PredMode + (iPrevIntra4x4PredMode >= kiPredMode);
    } else {
      if (iPrevIntra4x4PredMode) {
        iBestMode = kiPredMode;
      } else {
        WELS_READ_VERIFY (BsGetBits (pBs, 3, &uiCode));
        rtd->iRemIntra4x4PredMode[i] = uiCode;
        iBestMode = uiCode + ((int32_t) uiCode >= kiPredMode);
      }
    }
    rtd->iPrevIntra4x4PredMode[i] = iPrevIntra4x4PredMode;

    iFinalMode = CheckIntraNxNPredMode (&iSampleAvail[0], &iBestMode, i, false);
    if (iFinalMode  == ERR_INVALID_INTRA4X4_MODE) {
      return ERR_INFO_INVALID_I4x4_PRED_MODE;
    }

    pCurDqLayer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[i]] = iFinalMode;

    pIntraPredMode[g_kuiScan8[i]] = iBestMode;

    iSampleAvail[g_kuiCache30ScanIdx[i]] = 1;
  }
  ST32 (&pCurDqLayer->pIntraPredMode[iMbXy][0], LD32 (&pIntraPredMode[1 + 8 * 4]));
  pCurDqLayer->pIntraPredMode[iMbXy][4] = pIntraPredMode[4 + 8 * 1];
  pCurDqLayer->pIntraPredMode[iMbXy][5] = pIntraPredMode[4 + 8 * 2];
  pCurDqLayer->pIntraPredMode[iMbXy][6] = pIntraPredMode[4 + 8 * 3];

  if (pCtx->pSps->uiChromaFormatIdc == 0)//no need parse chroma
    return ERR_NONE;

  if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
    WELS_READ_VERIFY (ParseIntraPredModeChromaCabac (pCtx, uiNeighAvail, iCode));
    if (iCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = iCode;
  } else {
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //intra_chroma_pred_mode
    if (uiCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = uiCode;
  }

  if (-1 == pCurDqLayer->pChromaPredMode[iMbXy]
      || CheckIntraChromaPredMode (uiNeighAvail, &pCurDqLayer->pChromaPredMode[iMbXy])) {
    return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
  }
  return ERR_NONE;
}

int32_t ParseIntra8x8Mode (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, int8_t* pIntraPredMode,
                           PBitStringAux pBs, PDqLayer pCurDqLayer, RoundTripData *rtd) {
  // Similar with Intra_4x4, can put them together when needed
  int32_t iSampleAvail[5 * 6] = { 0 }; //initialize as 0
  int32_t iMbXy = pCurDqLayer->iMbXyIndex;
  int32_t iFinalMode, i;

  uint8_t uiNeighAvail = 0;
  uint32_t uiCode;
  int32_t iCode;
  pCtx->pMapNxNNeighToSampleFunc (pNeighAvail, iSampleAvail);
  // Top-Right : Left : Top-Left : Top
  uiNeighAvail = (iSampleAvail[5] << 3) | (iSampleAvail[6] << 2) | (iSampleAvail[0] << 1) | (iSampleAvail[1]);

  pCurDqLayer->pIntraNxNAvailFlag[iMbXy] = uiNeighAvail;

  for (i = 0; i < 4; i++) {
    int32_t iPrevIntra4x4PredMode = 0;
    rtd->iRemIntra4x4PredMode[i] = 0;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      WELS_READ_VERIFY (ParseIntraPredModeLumaCabac (pCtx, iCode));
      iPrevIntra4x4PredMode = iCode;
    } else {
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode));
      iPrevIntra4x4PredMode = uiCode;
    }
    const int32_t kiPredMode = PredIntra4x4Mode (pIntraPredMode, i << 2);

    int8_t iBestMode;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      if (iPrevIntra4x4PredMode == -1)
        iBestMode = kiPredMode;
      else
        iBestMode = iPrevIntra4x4PredMode + (iPrevIntra4x4PredMode >= kiPredMode);
    } else {
      if (iPrevIntra4x4PredMode) {
        iBestMode = kiPredMode;
      } else {
        WELS_READ_VERIFY (BsGetBits (pBs, 3, &uiCode));
        rtd->iRemIntra4x4PredMode[i] = uiCode;
        iBestMode = uiCode + ((int32_t) uiCode >= kiPredMode);
      }
    }
    rtd->iPrevIntra4x4PredMode[i] = iPrevIntra4x4PredMode;

    iFinalMode = CheckIntraNxNPredMode (&iSampleAvail[0], &iBestMode, i << 2, true);

    if (iFinalMode  == ERR_INVALID_INTRA4X4_MODE) {
      return ERR_INFO_INVALID_I4x4_PRED_MODE;
    }

    for (int j = 0; j < 4; j++) {
      pCurDqLayer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[ (i << 2) + j]] = iFinalMode;
      pIntraPredMode[g_kuiScan8[ (i << 2) + j]] = iBestMode;
      iSampleAvail[g_kuiCache30ScanIdx[ (i << 2) + j]] = 1;
    }
  }
  ST32 (&pCurDqLayer->pIntraPredMode[iMbXy][0], LD32 (&pIntraPredMode[1 + 8 * 4]));
  pCurDqLayer->pIntraPredMode[iMbXy][4] = pIntraPredMode[4 + 8 * 1];
  pCurDqLayer->pIntraPredMode[iMbXy][5] = pIntraPredMode[4 + 8 * 2];
  pCurDqLayer->pIntraPredMode[iMbXy][6] = pIntraPredMode[4 + 8 * 3];
  if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
    WELS_READ_VERIFY (ParseIntraPredModeChromaCabac (pCtx, uiNeighAvail, iCode));
    if (iCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = iCode;
  } else {
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //intra_chroma_pred_mode
    if (uiCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = uiCode;
  }

  if (-1 == pCurDqLayer->pChromaPredMode[iMbXy]
      || CheckIntraChromaPredMode (uiNeighAvail, &pCurDqLayer->pChromaPredMode[iMbXy])) {
    return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
  }

  return ERR_NONE;
}

int32_t ParseIntra16x16Mode (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, PBitStringAux pBs,
                             PDqLayer pCurDqLayer) {
  int32_t iMbXy = pCurDqLayer->iMbXyIndex;
  uint8_t uiNeighAvail = 0; //0x07 = 0 1 1 1, means left, top-left, top avail or not. (1: avail, 0: unavail)
  uint32_t uiCode;
  int32_t iCode;
  pCtx->pMap16x16NeighToSampleFunc (pNeighAvail, &uiNeighAvail);

  if (CheckIntra16x16PredMode (uiNeighAvail,
                               &pCurDqLayer->pIntraPredMode[iMbXy][7])) { //invalid iPredMode, must stop decoding
    return ERR_INFO_INVALID_I16x16_PRED_MODE;
  }
  if (pCtx->pSps->uiChromaFormatIdc == 0)
    return ERR_NONE;
 // Only for intra4x4 // Only for intra4xe
  if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
    WELS_READ_VERIFY (ParseIntraPredModeChromaCabac (pCtx, uiNeighAvail, iCode));
    if (iCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = iCode;
  } else {
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //intra_chroma_pred_mode
    if (uiCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = uiCode;
  }
  if (-1 == pCurDqLayer->pChromaPredMode[iMbXy]
      || CheckIntraChromaPredMode (uiNeighAvail, &pCurDqLayer->pChromaPredMode[iMbXy])) {
    return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
  }

  return ERR_NONE;
}

int32_t WelsDecodeMbCabacISliceBaseMode0 (PWelsDecoderContext pCtx, uint32_t& uiEosFlag) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBsAux           = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;
  SWelsNeighAvail sNeighAvail;
  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpLuma = 0, uiCbpChroma = 0;

  RoundTripData rtd;

  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;

  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
  pCurLayer->pResidualPredFlag[iMbXy] = pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
  GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
  WELS_READ_VERIFY (ParseMBTypeISliceCabac (pCtx, &sNeighAvail, uiMbType));
  if (uiMbType > 25) {
    return ERR_INFO_INVALID_MB_TYPE;
  } else if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17
             && uiMbType <= 24))) {
    return ERR_INFO_INVALID_MB_TYPE;
  } else if (25 == uiMbType) {   //I_PCM
    WELS_READ_VERIFY (ParseIPCMInfoCabac (pCtx));
    pSlice->iLastDeltaQp = 0;
    WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
    if (uiEosFlag) {
      RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
    }
    return ERR_NONE;
  } else if (0 == uiMbType) { //I4x4
    ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
    if (pCtx->pPps->bTransform8x8ModeFlag) {
      // Transform 8x8 cabac will be added soon
      WELS_READ_VERIFY (ParseTransformSize8x8FlagCabac (pCtx, &sNeighAvail, pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
    }
    if (pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]) {
      uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, &sNeighAvail, pIntraPredMode, pBsAux, pCurLayer, &rtd));
    } else {
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, &sNeighAvail, pIntraPredMode, pBsAux, pCurLayer, &rtd));
    }
    //get uiCbp for I4x4
    WELS_READ_VERIFY (ParseCbpInfoCabac (pCtx, &sNeighAvail, uiCbp));
    pCurLayer->pCbp[iMbXy] = uiCbp;
    pSlice->iLastDeltaQp = uiCbp == 0 ? 0 : pSlice->iLastDeltaQp;
    uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? uiCbp >> 4 : 0;
    uiCbpLuma = uiCbp & 15;
  } else { //I16x16;
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
    pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
    pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
    pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
    pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
    uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0 ;
    uiCbpLuma = pCurLayer->pCbp[iMbXy] & 15;
    WelsFillCacheNonZeroCount (&sNeighAvail, pNonZeroCount, pCurLayer);
    WELS_READ_VERIFY (ParseIntra16x16Mode (pCtx, &sNeighAvail, pBsAux, pCurLayer));
  }

  ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][20], 0);
  pCurLayer->pCbfDc[iMbXy] = 0;

  if (pCurLayer->pCbp[iMbXy] == 0 && IS_INTRANxN (pCurLayer->pMbType[iMbXy])) {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 ((pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i]), 0, 51)];
    }
  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeff[iMbXy][0]));
    int32_t iQpDelta, iId8x8, iId4x4;
    WELS_READ_VERIFY (ParseDeltaQpCabac (pCtx, iQpDelta));
    if (iQpDelta > 25 || iQpDelta < -26) {//out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }
    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 ((pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i]), 0, 51)];
    }
    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, 0, 16, g_kuiLumaDcZigzagScan,
                        I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx));
      //step2: Luma AC
      if (uiCbpLuma) {
        for (i = 0; i < 16; i++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, i,
                            iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1), I16_LUMA_AC,
                            pCurLayer->pScaledTCoeff[iMbXy] + (i << 4), pCurLayer->pLumaQp[iMbXy], pCtx));
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else { //pNonZeroCount = 0
        ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        // Transform 8x8 support for CABAC
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            WELS_READ_VERIFY (ParseResidualBlockCabac8x8 (&sNeighAvail, pNonZeroCount, pBsAux, (iId8x8 << 2),
                              iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart, LUMA_DC_AC_INTRA_8,
                              pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), pCurLayer->pLumaQp[iMbXy], pCtx));
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            int32_t iIdx = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, iIdx, iScanIdxEnd - iScanIdxStart + 1,
                                g_kuiZigzagScan + iScanIdxStart, LUMA_DC_AC_INTRA, pCurLayer->pScaledTCoeff[iMbXy] + (iIdx << 4),
                                pCurLayer->pLumaQp[iMbXy], pCtx));
              iIdx++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }
    int32_t iMbResProperty;
    //chroma
    //step1: DC
    if (1 == uiCbpChroma || 2 == uiCbpChroma) {
      //Cb Cr
      for (i = 0; i < 2; i++) {
        iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, 16 + (i << 2), 4, g_kuiChromaDcScan,
                          iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6), pCurLayer->pChromaQp[iMbXy][i], pCtx));
      }
    }

    //step2: AC
    if (2 == uiCbpChroma) {
      for (i = 0; i < 2; i++) { //Cb Cr
        iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        int32_t iIdx = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, iIdx,
                            iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1), iMbResProperty,
                            pCurLayer->pScaledTCoeff[iMbXy] + (iIdx << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx));
          iIdx++;
        }
      }
      ST16 (&pCurLayer->pNzc[iMbXy][16], LD16 (&pNonZeroCount[6 + 8 * 1]));
      ST16 (&pCurLayer->pNzc[iMbXy][20], LD16 (&pNonZeroCount[6 + 8 * 2]));
      ST16 (&pCurLayer->pNzc[iMbXy][18], LD16 (&pNonZeroCount[6 + 8 * 4]));
      ST16 (&pCurLayer->pNzc[iMbXy][22], LD16 (&pNonZeroCount[6 + 8 * 5]));
    } else {
      ST16 (&pCurLayer->pNzc[iMbXy][16], 0);
      ST16 (&pCurLayer->pNzc[iMbXy][20], 0);
      ST16 (&pCurLayer->pNzc[iMbXy][18], 0);
      ST16 (&pCurLayer->pNzc[iMbXy][22], 0);
    }
  } else {
    ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][20], 0);
  }

  WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
  if (uiEosFlag) {
    RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
  }
  return ERR_NONE;
}

int32_t WelsDecodeMbCabacISlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag) {
  WELS_READ_VERIFY (WelsDecodeMbCabacISliceBaseMode0 (pCtx, uiEosFlag));
  return ERR_NONE;
}

int32_t WelsDecodeMbCabacPSliceBaseMode0 (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, uint32_t& uiEosFlag) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBsAux           = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;

  RoundTripData rtd;

  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t iMbResProperty;
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpLuma = 0, uiCbpChroma = 0;

  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);

  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;

  WELS_READ_VERIFY (ParseMBTypePSliceCabac (pCtx, pNeighAvail, uiMbType));
  // uiMbType = 4 is not allowded.
  if (uiMbType < 4) { //Inter mode
    int16_t pMotionVector[LIST_A][30][MV_A];
    int16_t pMvdCache[LIST_A][30][MV_A];
    int8_t  pRefIndex[LIST_A][30];
    pCurLayer->pMbType[iMbXy] = g_ksInterMbTypeInfo[uiMbType].iType;
    WelsFillCacheInterCabac (pNeighAvail, pNonZeroCount, pMotionVector, pMvdCache, pRefIndex, pCurLayer);
    WELS_READ_VERIFY (ParseInterMotionInfoCabac (pCtx, pNeighAvail, pNonZeroCount, pMotionVector, pMvdCache, pRefIndex));
    pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
  } else { //Intra mode
    uiMbType -= 5;
    if (uiMbType > 25)
      return ERR_INFO_INVALID_MB_TYPE;
    if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17 && uiMbType <= 24)))
      return ERR_INFO_INVALID_MB_TYPE;

    if (25 == uiMbType) {   //I_PCM
      WELS_READ_VERIFY (ParseIPCMInfoCabac (pCtx));
      pSlice->iLastDeltaQp = 0;
      WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
      if (uiEosFlag) {
        RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
      }
      return ERR_NONE;
    } else { //normal Intra mode
      if (0 == uiMbType) { //Intra4x4
        ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
        if (pCtx->pPps->bTransform8x8ModeFlag) {
          WELS_READ_VERIFY (ParseTransformSize8x8FlagCabac (pCtx, pNeighAvail, pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
        }
        if (pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]) {
          uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
          pCtx->pFillInfoCacheIntraNxNFunc (pNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, pNeighAvail, pIntraPredMode, pBsAux, pCurLayer, &rtd));
        } else {
          pCtx->pFillInfoCacheIntraNxNFunc (pNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, pNeighAvail, pIntraPredMode, pBsAux, pCurLayer, &rtd));
        }
      } else { //Intra16x16
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
        pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
        pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
        pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
        pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
        uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0;
        uiCbpLuma = pCurLayer->pCbp[iMbXy] & 15;
        WelsFillCacheNonZeroCount (pNeighAvail, pNonZeroCount, pCurLayer);
        WELS_READ_VERIFY (ParseIntra16x16Mode (pCtx, pNeighAvail, pBsAux, pCurLayer));
      }
    }
  }

  ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][20], 0);

  if (MB_TYPE_INTRA16x16 != pCurLayer->pMbType[iMbXy]) {
    WELS_READ_VERIFY (ParseCbpInfoCabac (pCtx, pNeighAvail, uiCbp));

    pCurLayer->pCbp[iMbXy] = uiCbp;
    pSlice->iLastDeltaQp = uiCbp == 0 ? 0 : pSlice->iLastDeltaQp;
    uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0 ;
    uiCbpLuma = pCurLayer->pCbp[iMbXy] & 15;
  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {

    if (MB_TYPE_INTRA16x16 != pCurLayer->pMbType[iMbXy]) {
      // Need modification when B picutre add in
      bool bNeedParseTransformSize8x8Flag =
        (((pCurLayer->pMbType[iMbXy] >= MB_TYPE_16x16 && pCurLayer->pMbType[iMbXy] <= MB_TYPE_8x16)
          || pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
         && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA8x8)
         && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA4x4)
         && ((pCurLayer->pCbp[iMbXy] & 0x0F) > 0)
         && (pCtx->pPps->bTransform8x8ModeFlag));

      if (bNeedParseTransformSize8x8Flag) {
        WELS_READ_VERIFY (ParseTransformSize8x8FlagCabac (pCtx, pNeighAvail,
                          pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy])); //transform_size_8x8_flag
      }
    }

    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeff[iMbXy][0]));

    int32_t iQpDelta, iId8x8, iId4x4;

    WELS_READ_VERIFY (ParseDeltaQpCabac (pCtx, iQpDelta));
    if (iQpDelta > 25 || iQpDelta < -26) { //out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }
    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }

    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, 0, 16, g_kuiLumaDcZigzagScan,
                        I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx));
      //step2: Luma AC
      if (uiCbpLuma) {
        for (i = 0; i < 16; i++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, i, iScanIdxEnd - WELS_MAX (iScanIdxStart,
                            1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1), I16_LUMA_AC, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4),
                            pCurLayer->pLumaQp[iMbXy], pCtx));
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]) {
        // Transform 8x8 support for CABAC
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            WELS_READ_VERIFY (ParseResidualBlockCabac8x8 (pNeighAvail, pNonZeroCount, pBsAux, (iId8x8 << 2),
                              iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart,
                              IS_INTRA (pCurLayer->pMbType[iMbXy]) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8,
                              pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), pCurLayer->pLumaQp[iMbXy], pCtx));
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            int32_t iIdx = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, iIdx, iScanIdxEnd - iScanIdxStart + 1,
                                g_kuiZigzagScan + iScanIdxStart, iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (iIdx << 4),
                                pCurLayer->pLumaQp[iMbXy],
                                pCtx));
              iIdx++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }

    //chroma
    //step1: DC
    if (1 == uiCbpChroma || 2 == uiCbpChroma) {
      for (i = 0; i < 2; i++) {
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        else
          iMbResProperty = i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

        WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, 16 + (i << 2), 4, g_kuiChromaDcScan,
                          iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6), pCurLayer->pChromaQp[iMbXy][i], pCtx));
      }
    }
    //step2: AC
    if (2 == uiCbpChroma) {
      for (i = 0; i < 2; i++) {
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        else
          iMbResProperty = i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;
        int32_t index = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, index,
                            iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                            iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (index << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx));
          index++;
        }
      }
      ST16 (&pCurLayer->pNzc[iMbXy][16], LD16 (&pNonZeroCount[6 + 8 * 1]));
      ST16 (&pCurLayer->pNzc[iMbXy][20], LD16 (&pNonZeroCount[6 + 8 * 2]));
      ST16 (&pCurLayer->pNzc[iMbXy][18], LD16 (&pNonZeroCount[6 + 8 * 4]));
      ST16 (&pCurLayer->pNzc[iMbXy][22], LD16 (&pNonZeroCount[6 + 8 * 5]));
    } else {
      ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
      ST32 (&pCurLayer->pNzc[iMbXy][20], 0);
    }
  } else {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }
  }

  WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
  if (uiEosFlag) {
    RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
  }

  return ERR_NONE;
}

int32_t WelsDecodeMbCabacPSlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;
  PPicture* ppRefPic = pCtx->sRefPic.pRefList[LIST_0];
  uint32_t uiCode;
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t i;
  SWelsNeighAvail uiNeighAvail;
  pCurLayer->pCbp[iMbXy] = 0;
  pCurLayer->pCbfDc[iMbXy] = 0;
  pCurLayer->pChromaPredMode[iMbXy] = C_PRED_DC;

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;

  GetNeighborAvailMbType (&uiNeighAvail, pCurLayer);
  WELS_READ_VERIFY (ParseSkipFlagCabac (pCtx, &uiNeighAvail, uiCode));

  if (uiCode) {
    int16_t pMv[2] = {0};
    pCurLayer->pMbType[iMbXy] = MB_TYPE_SKIP;
    ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][20], 0);

    pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
    memset (pCurLayer->pRefIndex[0][iMbXy], 0, sizeof (int8_t) * 16);
    pCtx->bMbRefConcealed = pCtx->bRPLRError || pCtx->bMbRefConcealed || ! (ppRefPic[0] && ppRefPic[0]->bIsComplete);
    //predict mv
    PredPSkipMvFromNeighbor (pCurLayer, pMv);
    for (i = 0; i < 16; i++) {
      ST32 (pCurLayer->pMv[0][iMbXy][i], * (uint32_t*)pMv);
      ST32 (pCurLayer->pMvd[0][iMbXy][i], 0);
    }

    //if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
    //  memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (int16_t));
    //}

    //reset rS
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp; //??????????????? dqaunt of previous mb
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }

    //for neighboring CABAC usage
    pSlice->iLastDeltaQp = 0;

    WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));

    return ERR_NONE;
  }

  WELS_READ_VERIFY (WelsDecodeMbCabacPSliceBaseMode0 (pCtx, &uiNeighAvail, uiEosFlag));
  return ERR_NONE;
}
// Calculate deqaunt coeff scaling list value
int32_t  WelsCalcDeqCoeffScalingList (PWelsDecoderContext pCtx) {
  if (pCtx->pSps->bSeqScalingMatrixPresentFlag || pCtx->pPps->bPicScalingMatrixPresentFlag) {
    pCtx->bUseScalingList = true;

    if (!pCtx->bDequantCoeff4x4Init || (pCtx->iDequantCoeffPpsid != pCtx->pPps->iPpsId)) {
      int i, q, x, y;
      // Rewrite pps scaling list for scalingList present flag=0
      if (pCtx->bSpsLatePps) {
        for (i = 0; i < 12; i++) {
          //if (!pCtx->pSps->bSeqScalingListPresentFlag[i]) {
          if (!pCtx->pPps->bPicScalingListPresentFlag[i]) {
            if (i < 6) {
              if (i == 0 || i == 3)
                memcpy (pCtx->pPps->iScalingList4x4[i], pCtx->pSps->iScalingList4x4[i], 16 * sizeof (uint8_t));
              else
                memcpy (pCtx->pPps->iScalingList4x4[i], pCtx->pPps->iScalingList4x4[i - 1], 16 * sizeof (uint8_t));
            } else {
              if (i == 6 || i == 7)
                memcpy (pCtx->pPps->iScalingList8x8[ i - 6 ], pCtx->pSps->iScalingList8x8[ i - 6 ], 64 * sizeof (uint8_t));
              else
                memcpy (pCtx->pPps->iScalingList8x8[ i - 6 ], pCtx->pPps->iScalingList8x8[i - 8], 64 * sizeof (uint8_t));
            }
          }
        }
      }
      //Init dequant coeff value for different QP
      for (i = 0; i < 6; i++) {
        pCtx->pDequant_coeff4x4[i] = pCtx->pDequant_coeff_buffer4x4[i];
        pCtx->pDequant_coeff8x8[i] = pCtx->pDequant_coeff_buffer8x8[i];
        for (q = 0; q < 51; q++) {
          for (x = 0; x < 16; x++) {
            pCtx->pDequant_coeff4x4[i][q][x] = pCtx->pPps->bPicScalingMatrixPresentFlag ? pCtx->pPps->iScalingList4x4[i][x] *
                                               g_kuiDequantCoeff[q][x & 0x07] : pCtx->pSps->iScalingList4x4[i][x] * g_kuiDequantCoeff[q][x & 0x07];
          }
          for (y = 0; y < 64; y++) {
            pCtx->pDequant_coeff8x8[i][q][y] = pCtx->pPps->bPicScalingMatrixPresentFlag ? pCtx->pPps->iScalingList8x8[i][y] *
                                               g_kuiMatrixV[q % 6][y / 8][y % 8] : pCtx->pSps->iScalingList8x8[i][y] * g_kuiMatrixV[q % 6][y / 8][y % 8];
          }
        }
      }
      pCtx->bDequantCoeff4x4Init = true;
      pCtx->iDequantCoeffPpsid = pCtx->pPps->iPpsId;
    }
  } else
    pCtx->bUseScalingList = false;
  return ERR_NONE;
}
typedef unsigned int bititem; // this makes it easier to gdb
struct EmitDefBitsToOMovie {
    void operator()(const uint8_t data, uint8_t nBits) const {
        oMovie().def().emitBits(data, nBits);
    }
};
struct EmitDefBitsToBoolVector {
    std::vector<bititem> *output;
    void operator()(const uint8_t data, uint8_t nBits) const {
        for (uint8_t i = 0; i < nBits; ++i) {
            if (data & (1<<(7 - i))) {
                output->push_back(true);
            } else {
                output->push_back(false);
            }

        }
    }
};
template<class Functor> void copySBitStringAux(const SBitStringAux& orig, Functor f) {
    for (const uint8_t *ptr = orig.pStartBuf; ptr != orig.pCurBuf; ++ptr) {
        f(*ptr, 8);
    }
    if (orig.iLeftBits < 32) {
        int nBits = 32 - orig.iLeftBits;
        uint32_t curBits = orig.uiCurBits;
        while (nBits > 0) {
            f(((curBits) >> (nBits - 8)) & 0xff, nBits < 8 ? nBits : 8);
            nBits -= 8;
        }
    }
}
std::vector<bititem> bitStringToVector(const SBitStringAux& orig) {
    std::vector<bititem> retval;
    EmitDefBitsToBoolVector f;
    f.output = &retval;
    copySBitStringAux(orig, f);
    return retval;
}
/*
 * Currently returns true if rt is a bitwise substring of orig
 * Eventually will check for bitwise equality
 */
bool stringBitCompare(const std::vector<bititem> &ovec,
                      const std::vector<bititem> &rvec) {
    size_t longest_substring = 0;
    size_t longest_offset = 0;
    size_t longest_rt_offset = 0;
    std::vector<bititem>::const_iterator oi = ovec.begin();
    std::vector<bititem>::const_iterator oend = ovec.end();
    for (std::vector<bititem>::const_iterator ri = rvec.begin(),
             rend = rvec.end(); ri != rend; ++ri){
        size_t cur_substring = 0;
        std::vector<bititem>::const_iterator orig_cmp_iter = oi;
        std::vector<bititem>::const_iterator r_cmp_iter = ri;
        for (;orig_cmp_iter != oend && r_cmp_iter != rend; ++r_cmp_iter,++orig_cmp_iter) {
            if (*orig_cmp_iter != *r_cmp_iter) {
                break;
            }
            ++cur_substring;
        }
        if (cur_substring > longest_substring) {
            longest_substring = cur_substring;
            longest_offset = oi - ovec.begin();
            longest_rt_offset = ri - rvec.begin();
        }
    }
    if (longest_substring < rvec.size()) {
        fprintf(stderr, "Longest prefix of rt[%ld] contained is %ld/%ld at orig[%ld] orig.size = %ld\n",
                longest_rt_offset, longest_substring, rvec.size(), longest_offset, ovec.size());
    }
    bool ret = longest_substring == ovec.size();
    /*if (!ret) {
        std::string s = "";
        for (std::vector<bititem>::const_iterator oi = ovec.begin(), oend = ovec.end(), ri = rvec.begin(), rend = rvec.end(); oi != oend || ri != rend;){
            int bits = 0;
            bool odone = false;
            bool rdone = false;
            if (oi != oend) {
                bits |= ((int)(*oi)) << 1;
                ++oi;
                odone = (oi == oend);
            }
            if (ri != rend) {
                bits |= ((int)(*ri));
                ++ri;
                rdone = (ri == rend);
            }
            s += (char)('0' + bits);
            if (odone) {
                s += '<';
            }
            if (rdone) {
                s += '>';
            }
        }
        fprintf(stderr, "Bitstrings not equal! %s\n", s.c_str());
    }*/
    std::string s = "";
    for (std::vector<bititem>::const_iterator oi = ovec.begin(); oi != ovec.end(); ++oi) {
        s += ('0' + (int)(*oi));
    }
    fprintf(stderr, "Origin bitstring %s\n", s.c_str());
    s = "";
    for (std::vector<bititem>::const_iterator ri = rvec.begin(); ri != rvec.end(); ++ri) {
        s += ('0' + (int)(*ri));
    }
    fprintf(stderr, "Result bitstring %s\n", s.c_str());
    return ret;
}

bool stringBitCompare(const std::vector<bititem> &ovec,
                      const SBitStringAux& rt) {
    std::vector<bititem> rvec = bitStringToVector(rt);
    return stringBitCompare(ovec, rvec);
}

bool stringBitCompare(const PBitStringAux& orig,
                      const SBitStringAux& rt) {
    std::vector<bititem> ovec = bitStringToVector(*orig);
    std::vector<bititem> rvec = bitStringToVector(rt);
    return stringBitCompare(ovec, rvec);
}

struct EncoderState {
    WelsEnc::SDCTCoeff pDct;
    WelsEnc::sWelsEncCtx pEncCtx;
    WelsEnc::SSlice pSlice;
    WelsEnc::SMB pCurMb;
    WelsEnc::SDqLayer pCurDqLayer;
    WelsEnc::SWelsPPS pPpsP;
    WelsEnc::SMVUnitXY sMv[16];
    SBitStringAux wrBs;
    uint8_t buf[MAX_MACROBLOCK_SIZE_IN_BYTE_x2 * 2];
    bool prevIntra4x4PredModeFlag[16];
    int8_t remIntra4x4PredModeFlag[16];
    int8_t pRefIndex[4];

    EncoderState()
            : pDct(), pEncCtx(), pSlice(), pCurMb(), pCurDqLayer(), pPpsP(),
             wrBs(), buf(), prevIntra4x4PredModeFlag(),
             remIntra4x4PredModeFlag(), pRefIndex() {
        pEncCtx.pCurDqLayer = &pCurDqLayer;
        pCurDqLayer.sLayerInfo.pPpsP = &pPpsP;
        pEncCtx.pFuncList = gFuncPtrList;
        pSlice.sMbCacheInfo.pDct = &pDct;
        pCurMb.pRefIndex = &pRefIndex[0];
    }

    void init(uint8_t* pNonZeroCount, RawDCTData *dct,
              PWelsDecoderContext pCtx, PNalUnit pNalCur,
              RoundTripData *rtd, int uiCbpC, int uiCbpL) {
        PDqLayer pCurLayer = pCtx->pCurDqLayer;
        PSlice decoderpSlice = &pCurLayer->sLayerInfo.sSliceInLayer;
        PSliceHeader pSliceHeader = &decoderpSlice->sSliceHeaderExt.sSliceHeader;
        //PPicture* ppRefPic = pCtx->sRefPic.pRefList[LIST_0];

        InitBits (&wrBs, buf, sizeof(buf));
        memcpy(pSlice.sMbCacheInfo.iNonZeroCoeffCount, pNonZeroCount, 48);

        pPpsP.uiChromaQpIndexOffset = pSliceHeader->pPps->iChromaQpIndexOffset[0];

        // pEncCtx->pCurDqLayer->sLayerInfo.pPpsP->uiChromaQpIndexOffset FIXME?
        pEncCtx.eSliceType = pSliceHeader->eSliceType;
        pSlice.sSliceHeaderExt.sSliceHeader.eSliceType = pEncCtx.eSliceType;
        pEncCtx.pFuncList = gFuncPtrList;

        memcpy(pSlice.sMbCacheInfo.iNonZeroCoeffCount, pNonZeroCount, 48);
        // pSlice.sMbCacheInfo.sMvComponents may be need for cabac
        pSlice.sMbCacheInfo.pPrevIntra4x4PredModeFlag = prevIntra4x4PredModeFlag;
        pSlice.sMbCacheInfo.pRemIntra4x4PredModeFlag = remIntra4x4PredModeFlag;

        size_t num_components = 0;
        const int32_t iMbXy = pCurLayer->iMbXyIndex;
        if (pCurLayer->pMbType[iMbXy] == MB_TYPE_INTRA8x8) {
            num_components = 4;
        } else if (pCurLayer->pMbType[iMbXy] == MB_TYPE_INTRA4x4) {
            num_components = 16;
        }
        for (size_t i = 0; i < num_components; i++) {
            prevIntra4x4PredModeFlag[i] = !!(rtd->iPrevIntra4x4PredMode[i]);
            remIntra4x4PredModeFlag[i] = (int8_t)(rtd->iRemIntra4x4PredMode[i]);
        }

        pSlice.sMbCacheInfo.uiChmaI8x8Mode = pCtx->pCurDqLayer->pChromaPredMode[iMbXy];
        // pMbCache->sMbMvp is left as 0 so that we can just write the MV deltas we read in.
        pSlice.sMbCacheInfo.pDct = &pDct;
        pSlice.pSliceBsa = &wrBs;
        pSlice.sSliceHeaderExt.sSliceHeader.uiNumRefIdxL0Active = pSliceHeader->uiRefCount[0]; // Number of reference frames.
        pSlice.uiLastMbQp = decoderpSlice->iLastMbQp;
        pSlice.iMbSkipRun = decoderpSlice->iMbSkipRun;
        pSlice.uiSliceIdx = 0;
        for (int i = 0; i < 4; i++) {
            // only 8x8
            pCurMb.uiSubMbType[i] = pCtx->pCurDqLayer->pSubMbType[iMbXy][i];
        }

        for (int i = 0; i < 4; i++) {
            pCurMb.pRefIndex[i] = rtd->iRefIdx[i];
        }
        pCurMb.sMv = &sMv[0];
        for (int i = 0; i < 16; i++) {
            pCurMb.sMv[i].iMvX = rtd->sMbMvp[i][0];
            pCurMb.sMv[i].iMvY = rtd->sMbMvp[i][1];
        }
        pCurMb.uiMbType = pCurLayer->pMbType[iMbXy];
        pCurMb.uiCbp = ((uiCbpC << 4) | (uiCbpL & 15));
        // CbpC bits for which 8x8 block to encode luma?
        // CbpL 0 = no chroma; 1 = dc only; 2 = dc&ac
        pCurMb.uiLumaQp = pCurLayer->pLumaQp[iMbXy];
        // FIXME: Cb and Cr. Baseline only?
        pCurMb.uiChromaQp = pCurLayer->pChromaQp[iMbXy][0];
    }

};

bool knownCodeUnitTest(RawDCTData &odata,
                       const std::vector<bititem> expectedPrefix) {
    SBitStringAux wrBs;
    uint8_t buf[MAX_MACROBLOCK_SIZE_IN_BYTE_x2 * 2] = {0}; // Cannot be larger than raw input.
    WelsEnc::SDCTCoeff pDct = {};
    memcpy(pDct.iLumaBlock, odata.lumaAC, sizeof(odata.lumaAC));
    memcpy(pDct.iChromaBlock, odata.chromaAC, sizeof(odata.chromaAC));
    memcpy(pDct.iLumaI16x16Dc, odata.lumaDC, sizeof(odata.lumaDC));
    memcpy(pDct.iChromaDc, odata.chromaDC, sizeof(odata.chromaDC));

    InitBits (&wrBs, buf, sizeof(buf));
    ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
    memset(pNonZeroCount, 0, 48 * sizeof(*pNonZeroCount));
    //pNonZeroCount[0] = 5;
    //pNonZeroCount[1] = 3;
    pNonZeroCount[9] = 5;

    WelsEnc::sWelsEncCtx pEncCtx = {};
    WelsEnc::SSlice pSlice = {};
    WelsEnc::SMB pCurMb = {};

    // This stuff isn't really used.
    WelsEnc::SDqLayer pCurDqLayer = {};
    WelsEnc::SWelsPPS pPpsP = {};
    pEncCtx.pCurDqLayer = &pCurDqLayer;
    pCurDqLayer.sLayerInfo.pPpsP = &pPpsP;
    pPpsP.uiChromaQpIndexOffset = 0;

    // pEncCtx->pCurDqLayer->sLayerInfo.pPpsP->uiChromaQpIndexOffset
    pEncCtx.eSliceType = I_SLICE;
    pSlice.sSliceHeaderExt.sSliceHeader.eSliceType = pEncCtx.eSliceType;
    pEncCtx.pFuncList = gFuncPtrList;

    memcpy(pSlice.sMbCacheInfo.iNonZeroCoeffCount, pNonZeroCount, 48);
    // pSlice.sMbCacheInfo.sMvComponents may be need for cabac
    // pMbCache->pPrevIntra4x4PredModeFlag // Only for intra4x4
    // pMbCache->pRemIntra4x4PredModeFlag // Only for intra4x4
    // pMbCache->uiChmaI8x8Mode // Only for intra4x4 or intra16x16
    // pMbCache->sMbMvp
    pSlice.sMbCacheInfo.pDct = &pDct;
    pSlice.pSliceBsa = &wrBs;
    // pSlice.sSliceHeaderExt.sSliceHeader.uiNumRefIdxL0Active // Number of reference frames.
    // pSlice.uiLastMbQp = decoder.pSlice->iLastMbQp
    // iMbSkipRun
    // uiSliceIdx
    // pCurMb.uiSubMbType // only 8x8
    // pCurMb.pRefIndex[0..3]
    // pCurMb.sMv
    pCurMb.uiMbType = MB_TYPE_INTRA16x16;
    // pCurMb->uiCbp = uiCbpC << 4 | (uiCbpL & 15)
    // CbpC bits for which 8x8 block to encode luma?
    // CbpL 0 = no chroma; 1 = dc only; 2 = dc&ac
    pCurMb.uiCbp = 15;
    // pCurMb.uiLumaQp
    // pCurMb->uiChromaQp

    if (WelsEnc::WelsSpatialWriteMbSyn (
                &pEncCtx, &pSlice, &pCurMb)) {
        fprintf(stderr, "Encode failed!");
        return false;
    }
    bool retval = stringBitCompare(expectedPrefix, wrBs);
    return retval;
}

bool knownCodeUnitTest0() {
    RawDCTData odata;
    memset(&odata, 0, sizeof(odata));

    std::vector<bititem> expectedPrefix;
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    bool retval = knownCodeUnitTest(odata, expectedPrefix);
    return retval; // I think the expectedPrefix is wrong
    //return true;
}

bool knownCodeUnitTest1() {
    RawDCTData odata;
    memset(&odata, 0, sizeof(odata));
    odata.lumaAC[0] = 0;
    odata.lumaAC[1] = 3;
    odata.lumaAC[2] = 0;
    odata.lumaAC[3] = 1;
    odata.lumaAC[4] = -1;
    odata.lumaAC[5] = -1;
    odata.lumaAC[6] = 0;
    odata.lumaAC[7] = 1;
    odata.lumaAC[8] = 0;
/*
    odata.lumaAC[1] = 3;
    odata.lumaAC[2] = -1;
    int w = 16;// is this 4?
    odata.lumaAC[1 + w] = -1;
    odata.lumaAC[2 + w] = 1;
    odata.lumaAC[0 + 2*w] = 1;
*/

    std::vector<bititem> expectedPrefix;
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    return knownCodeUnitTest(odata, expectedPrefix);
}
bool knownCodeUnitTest2() {
    RawDCTData odata;
    memset(&odata, 0, sizeof(odata));
    odata.lumaAC[0] = -2;
    odata.lumaAC[1] = 4;
    odata.lumaAC[2] = 3;
    odata.lumaAC[3] = -3;
    odata.lumaAC[4] = 0;
    odata.lumaAC[5] = 0;
    odata.lumaAC[6] =-1;
    odata.lumaAC[7] = 0;
    odata.lumaAC[8] = 0;
/* if row major
    odata.lumaAC[0] = -2;
    odata.lumaAC[1] = 4;
    odata.lumaAC[2] =-1;

    int w = 16; // is this 4?
    odata.lumaAC[w] = 3;
    odata.lumaAC[2*w] = -3;
*/

    std::vector<bititem> expectedPrefix;
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    
    return knownCodeUnitTest(odata, expectedPrefix);    
}







int32_t WelsDecodeSlice (PWelsDecoderContext pCtx, bool bFirstSliceInLayer, PNalUnit pNalCur) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  PFmo pFmo = pCtx->pFmo;
  int32_t iRet;
  int32_t iNextMbXyIndex, iSliceIdc;

  PSlice pSlice = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeaderExt pSliceHeaderExt = &pSlice->sSliceHeaderExt;
  PSliceHeader pSliceHeader = &pSliceHeaderExt->sSliceHeader;
  int32_t iMbX, iMbY;
  const int32_t kiCountNumMb = pSliceHeader->pSps->uiTotalMbCount; //need to be correct when fmo or multi slice
  uint32_t uiEosFlag = 0;
  PWelsDecMbFunc pDecMbFunc;

  pSlice->iTotalMbInCurSlice = 0; //initialize at the starting of slice decoding.

  if (pCtx->pPps->bEntropyCodingModeFlag) {
    if (pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag ||
        pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag ||
        pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag) {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_ERROR,
               "WelsDecodeSlice()::::ILP flag exist, not supported with CABAC enabled!");
      pCtx->iErrorCode |= dsBitstreamError;
      return dsBitstreamError;
    }
    if (P_SLICE == pSliceHeader->eSliceType)
      pDecMbFunc = WelsDecodeMbCabacPSlice;
    else //I_SLICE. B_SLICE not supported now
      pDecMbFunc = WelsDecodeMbCabacISlice;
  } else {
    if (P_SLICE == pSliceHeader->eSliceType) {
      pDecMbFunc = WelsDecodeMbCavlcPSlice;
    } else { //I_SLICE
      pDecMbFunc = WelsDecodeMbCavlcISlice;
    }
  }

  if (pSliceHeader->pPps->bConstainedIntraPredFlag) {
    pCtx->pFillInfoCacheIntraNxNFunc = WelsFillCacheConstrain1IntraNxN;
    pCtx->pMapNxNNeighToSampleFunc    = WelsMapNxNNeighToSampleConstrain1;
    pCtx->pMap16x16NeighToSampleFunc  = WelsMap16x16NeighToSampleConstrain1;
  } else {
    pCtx->pFillInfoCacheIntraNxNFunc = WelsFillCacheConstrain0IntraNxN;
    pCtx->pMapNxNNeighToSampleFunc    = WelsMapNxNNeighToSampleNormal;
    pCtx->pMap16x16NeighToSampleFunc  = WelsMap16x16NeighToSampleNormal;
  }

  pCtx->eSliceType = pSliceHeader->eSliceType;
  if (pCurLayer->sLayerInfo.pPps->bEntropyCodingModeFlag == 1) {
    int32_t iQp = pSlice->sSliceHeaderExt.sSliceHeader.iSliceQp;
    int32_t iCabacInitIdc = pSlice->sSliceHeaderExt.sSliceHeader.iCabacInitIdc;
    WelsCabacContextInit (pCtx, pSlice->eSliceType, iCabacInitIdc, iQp);
    //InitCabacCtx (pCtx->pCabacCtx, pSlice->eSliceType, iCabacInitIdc, iQp);
    pSlice->iLastDeltaQp = 0;
    WELS_READ_VERIFY (InitCabacDecEngineFromBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux));
  }
  //try to calculate  the dequant_coeff
  WelsCalcDeqCoeffScalingList (pCtx);

  iNextMbXyIndex = pSliceHeader->iFirstMbInSlice;
  iMbX = iNextMbXyIndex % pCurLayer->iMbWidth;
  iMbY = iNextMbXyIndex / pCurLayer->iMbWidth; // error is introduced by multiple slices case, 11/23/2009
  pSlice->iMbSkipRun = -1;
  iSliceIdc = (pSliceHeader->iFirstMbInSlice << 7) + pCurLayer->uiLayerDqId;

  pCurLayer->iMbX =  iMbX;
  pCurLayer->iMbY = iMbY;
  pCurLayer->iMbXyIndex = iNextMbXyIndex;

  do {
    if ((-1 == iNextMbXyIndex) || (iNextMbXyIndex >= kiCountNumMb)) { // slice group boundary or end of a frame
      break;
    }

    pCurLayer->pSliceIdc[iNextMbXyIndex] = iSliceIdc;
    pCtx->bMbRefConcealed = false;
    iRet = pDecMbFunc (pCtx,  pNalCur, uiEosFlag);
    pCurLayer->pMbRefConcealedFlag[iNextMbXyIndex] = pCtx->bMbRefConcealed;
    if (iRet != ERR_NONE) {
      return iRet;
    }

    ++pSlice->iTotalMbInCurSlice;
    if (uiEosFlag) { //end of slice
      break;
    }
    if (pSliceHeader->pPps->uiNumSliceGroups > 1) {
      iNextMbXyIndex = FmoNextMb (pFmo, iNextMbXyIndex);
    } else {
      ++iNextMbXyIndex;
    }
    iMbX = iNextMbXyIndex % pCurLayer->iMbWidth;
    iMbY = iNextMbXyIndex / pCurLayer->iMbWidth;
    pCurLayer->iMbX =  iMbX;
    pCurLayer->iMbY = iMbY;
    pCurLayer->iMbXyIndex = iNextMbXyIndex;
  } while (1);
  return ERR_NONE;
}
int32_t WelsActualDecodeMbCavlcISlice (PWelsDecoderContext pCtx, PNalUnit pNalCur) {
  SVlcTable* pVlcTable     = &pCtx->sVlcTable;
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBs              = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;

  RoundTripData rtd;

  SWelsNeighAvail sNeighAvail;
  int32_t iMbResProperty;

  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;

  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  const int32_t iMbXy = pCurLayer->iMbXyIndex;
  int8_t* pNzc = pCurLayer->pNzc[iMbXy];
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpL = 0, uiCbpC = 0;
  uint32_t uiCode;
  int32_t iCode;

  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
  GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
  pCurLayer->pResidualPredFlag[iMbXy] = pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;

  WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //uiMbType
  uiMbType = uiCode;
  if (uiMbType > 25)
    return ERR_INFO_INVALID_MB_TYPE;
  if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17 && uiMbType <= 24)))
    return ERR_INFO_INVALID_MB_TYPE;

  if (25 == uiMbType) {
    int32_t iDecStrideL = pCurLayer->pDec->iLinesize[0];
    int32_t iDecStrideC = pCurLayer->pDec->iLinesize[1];

    int32_t iOffsetL = (iMbX + iMbY * iDecStrideL) << 4;
    int32_t iOffsetC = (iMbX + iMbY * iDecStrideC) << 3;

    uint8_t* pDecY = pCurLayer->pDec->pData[0] + iOffsetL;
    uint8_t* pDecU = pCurLayer->pDec->pData[1] + iOffsetC;
    uint8_t* pDecV = pCurLayer->pDec->pData[2] + iOffsetC;

    uint8_t* pTmpBsBuf;


    int32_t i;
    int32_t iCopySizeY  = (sizeof (uint8_t) << 4);
    int32_t iCopySizeUV = (sizeof (uint8_t) << 3);

    int32_t iIndex = ((-pBs->iLeftBits) >> 3) + 2;

    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA_PCM;

    //step 1: locating bit-stream pointer [must align into integer byte]
    pBs->pCurBuf -= iIndex;

    //step 2: copy pixel from bit-stream into fdec [reconstruction]
    pTmpBsBuf = pBs->pCurBuf;
    for (i = 0; i < 16; i++) { //luma
      memcpy (pDecY , pTmpBsBuf, iCopySizeY);
      pDecY += iDecStrideL;
      pTmpBsBuf += 16;
    }
    for (i = 0; i < 8; i++) { //cb
      memcpy (pDecU, pTmpBsBuf, iCopySizeUV);
      pDecU += iDecStrideC;
      pTmpBsBuf += 8;
    }
    for (i = 0; i < 8; i++) { //cr
      memcpy (pDecV, pTmpBsBuf, iCopySizeUV);
      pDecV += iDecStrideC;
      pTmpBsBuf += 8;
    }

    pBs->pCurBuf += 384;

    //step 3: update QP and pNonZeroCount
    pCurLayer->pLumaQp[iMbXy] = 0;
    memset (pCurLayer->pChromaQp[iMbXy], 0, sizeof (pCurLayer->pChromaQp[iMbXy]));
    memset (pNzc, 16, sizeof (pCurLayer->pNzc[iMbXy]));   //Rec. 9.2.1 for PCM, nzc=16
    WELS_READ_VERIFY (InitReadBits (pBs, 0));
    return 0;
  } else if (0 == uiMbType) { //reference to JM
    ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
    if (pCtx->pPps->bTransform8x8ModeFlag) {
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //transform_size_8x8_flag
      pCurLayer->pTransformSize8x8Flag[iMbXy] = !!uiCode;
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
      }
    }
    if (!pCurLayer->pTransformSize8x8Flag[iMbXy]) {
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, &rtd));
    } else {
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, &rtd));
    }

    //uiCbp
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //coded_block_pattern
    uiCbp = uiCode;
    //G.9.1 Alternative parsing process for coded pBlock pattern
    if (pCtx->pSps->uiChromaFormatIdc && (uiCbp > 47))
      return ERR_INFO_INVALID_CBP;
    if (!pCtx->pSps->uiChromaFormatIdc && (uiCbp > 15))
      return ERR_INFO_INVALID_CBP;

    if (pCtx->pSps->uiChromaFormatIdc)
      uiCbp = g_kuiIntra4x4CbpTable[uiCbp];
    else
      uiCbp = g_kuiIntra4x4CbpTable400[uiCbp];
    pCurLayer->pCbp[iMbXy] = uiCbp;
    uiCbpC = uiCbp >> 4;
    uiCbpL = uiCbp & 15;
  } else { //I_PCM exclude, we can ignore it
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
    pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
    pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
    pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
    pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
    uiCbpC = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0;
    uiCbpL = pCurLayer->pCbp[iMbXy] & 15;
    WelsFillCacheNonZeroCount (&sNeighAvail, pNonZeroCount, pCurLayer);
    WELS_READ_VERIFY (ParseIntra16x16Mode (pCtx, &sNeighAvail, pBs, pCurLayer));
  }

  ST32A4 (&pNzc[0], 0);
  ST32A4 (&pNzc[4], 0);
  ST32A4 (&pNzc[8], 0);
  ST32A4 (&pNzc[12], 0);
  ST32A4 (&pNzc[16], 0);
  ST32A4 (&pNzc[20], 0);

  if (pCurLayer->pCbp[iMbXy] == 0 && IS_INTRA4x4 (pCurLayer->pMbType[iMbXy])) {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }

  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeff[iMbXy][0]));
    int32_t iQpDelta, iId8x8, iId4x4;

    WELS_READ_VERIFY (BsGetSe (pBs, &iCode)); //mb_qp_delta
    iQpDelta = iCode;

    if (iQpDelta > 25 || iQpDelta < -26) { //out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }

    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0,
                                       51)];
    }

    BsStartCavlc (pBs);
    RawDCTData odata; // for both recoding and ROUNDTRIP_TEST
#ifdef ROUNDTRIP_TEST
    memset(&odata, 0, sizeof(odata));
    //static bool ok0 = knownCodeUnitTest0();
    //assert(ok0 && "Known block0 should be ok");
    static bool ok1 = knownCodeUnitTest1();
    assert(ok1 && "Known block1 should be ok");
    static bool ok2 = knownCodeUnitTest2();
    assert(ok2 && "Known block2 should be ok");
#endif
    if (oMovie().isRecoding) {
      uint8_t buf[384 * 2] = {0}; // Cannot be larger than raw input.
      memset(&odata, 0, sizeof(odata));
      BitStream::uint32E res;
      for (i = 0; i < 48; i++) {
        res = iMovie().tag(1).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read ldc!\n");
          pNonZeroCount[i] = 255;
        } else {
          pNonZeroCount[i] = res.first;
        }
      }
      if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
        for (int i = 0; i < 16; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read ldc!\n");
          }
          odata.lumaDC[i] = res.first;
        }
      }
      if (1 == uiCbpC || 2 == uiCbpC) {
        for (int i = 0; i < 8; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read crdc!\n");
          }
          odata.chromaDC[i] = res.first;
        }
      }
      if (uiCbpL) {
        for (int i = 0; i < 256; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read lac!\n");
          }
          odata.lumaAC[i] = res.first;
        }
      }
      if (uiCbpC == 2) {
        for (int i = 0; i < 128; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read crac!\n");
          }
          odata.chromaAC[i] = iMovie().tag(1).scanBits(16).first;
        }
      }
      SBitStringAux wrBs;
      InitBits (&wrBs, buf, sizeof(buf));
      WelsEnc::WelsUtilWriteMbResidual (
          gFuncPtrList, pCurLayer->pMbType[iMbXy], uiCbpC, uiCbpL,
          (int8_t*)pNonZeroCount, odata.lumaDC, odata.lumaAC, odata.chromaDC, odata.chromaAC, &wrBs);
      BsEndCavlc (pBs);
      EmitDefBitsToOMovie emission;
      copySBitStringAux(wrBs, emission);
/*
      for (uint8_t *ptr = wrBs.pStartBuf; ptr != wrBs.pCurBuf; ++ptr) {
        // fprintf(stderr, "Wrote some bytes\n");
        oMovie().def().emitBits(*ptr, 8);
      }
      if (wrBs.iLeftBits < 8) {
        int nBits = 8 - wrBs.iLeftBits;
        oMovie().def().emitBits(wrBs.uiCurBits >> (32 - nBits), nBits);
      }
*/
      return 0;
    }
    // for (i = 0; i < 48; i++) oMovie().tag(1).writeBits(pNonZeroCount[i], 8);

    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, 0, 16,
                                  g_kuiLumaDcZigzagScan, I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx)) {
        return -1;//abnormal
      }
      for (i = 0; i < 256; i += 16) {
        oMovie().tag(1).emitBits(pCurLayer->pScaledTCoeff[iMbXy][i], 16);
#ifdef ROUNDTRIP_TEST
        odata.chromaDC[i / 16] = pCurLayer->pScaledTCoeff[iMbXy][i];
#endif
      }
      //step2: Luma AC
      if (uiCbpL) {
        for (i = 0; i < 16; i++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, i,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      I16_LUMA_AC, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
            return -1;//abnormal
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              if (WelsResidualBlockCavlc8x8 (pVlcTable, pNonZeroCount, pBs, iIndex,
                                             iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart, iMbResProperty,
                                             pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), iId4x4, pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                          iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan + iScanIdxStart,
                                          LUMA_DC_AC_INTRA, pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;//abnormal
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }

    //chroma
    //step1: DC
    if (1 == uiCbpC || 2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs,
                                    16 + (i << 2), 4, g_kuiChromaDcScan, iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6),
                                    pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
          return -1;//abnormal
        }
      }
      for (i = 256; i < 384; i += 16) {
        oMovie().tag(1).emitBits(
              pCurLayer->pScaledTCoeff[iMbXy][i], 16);
#ifdef ROUNDTRIP_TEST
        odata.chromaDC[(i / 16) - 16] = pCurLayer->pScaledTCoeff[iMbXy][i];
#endif
      }
    } else if (uiCbpC != 0) {
      // FIXME(patrick): Encoder does not check for this condition...
      fprintf(stderr, "uiCbpC is %d not 1 or 2. Encoder might expect DC!\n",
              uiCbpC);
    }

    //step2: AC
    if (2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        int32_t iIndex = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
            return -1;//abnormal
          }
          iIndex++;
        }
      }
      ST16A2 (&pNzc[16], LD16A2 (&pNonZeroCount[6 + 8 * 1]));
      ST16A2 (&pNzc[20], LD16A2 (&pNonZeroCount[6 + 8 * 2]));
      ST16A2 (&pNzc[18], LD16A2 (&pNonZeroCount[6 + 8 * 4]));
      ST16A2 (&pNzc[22], LD16A2 (&pNonZeroCount[6 + 8 * 5]));
    }
    BsEndCavlc (pBs);




    // Write stuff here.

    {
        odata.uiChmaI8x8Mode = pCtx->pCurDqLayer->pChromaPredMode[iMbXy];
/* values 4,5,6 are alll the case where top, left, or niether are present--they get remapped to mode 0 by the enc 
        if (pCurLayer->pMbType[iMbXy] == MB_TYPE_INTRA4x4 || pCurLayer->pMbType[iMbXy] == MB_TYPE_INTRA16x16) {
            bool chromaPredModeMatchFound = false;
            for (size_t i = 0; i < sizeof(WelsEnc::g_kiMapModeIntraChroma) / sizeof(WelsEnc::g_kiMapModeIntraChroma[0]); ++i) {
                if (WelsEnc::g_kiMapModeIntraChroma[i] == odata.uiChmaI8x8Mode) {
                    odata.uiChmaI8x8Mode = (uint8_t)i;
                    chromaPredModeMatchFound = true;
                    break;
                }
            }
            oMovie().tag(1).emitBits(odata.uiChmaI8x8Mode, 8);
            assert(chromaPredModeMatchFound && "Reverse lookup table g_kiMapModeIntraChroma not rich enough to recode this file");
        }
*/
        size_t num_components = 0;
        if (pCurLayer->pMbType[iMbXy] == MB_TYPE_INTRA8x8) {
            num_components = 4;
        } else if (pCurLayer->pMbType[iMbXy] == MB_TYPE_INTRA4x4) {
            num_components = 16;
        }
        for (size_t i = 0; i < num_components; i++) {
            oMovie().tag(1).emitBits(odata.pPrevIntra4x4PredModeFlag[i] ? 1 : 0, 1);
            oMovie().tag(1).emitBits(odata.pRemIntra4x4PredModeFlag[i], 3);
        }

    }
    if (uiCbpL) {
      for (i = 0; i < 256; i++) {
        oMovie().tag(1).emitBits(pCurLayer->pScaledTCoeff[iMbXy][i], 16);
      }
#ifdef ROUNDTRIP_TEST
      memcpy(odata.lumaAC, pCurLayer->pScaledTCoeff[iMbXy], sizeof(odata.lumaAC));
#endif
    }
    if (2 == uiCbpC) {
      for (i = 256; i < 384; i++) {
        oMovie().tag(1).emitBits(pCurLayer->pScaledTCoeff[iMbXy][i], 16);
      }
#ifdef ROUNDTRIP_TEST
      memcpy(odata.chromaAC, pCurLayer->pScaledTCoeff[iMbXy], sizeof(odata.chromaAC));
#endif
    }
    for (i = 0; i < 48; i++) {
      oMovie().tag(1).emitBits(pNonZeroCount[i], 8);
    }
#ifdef ROUNDTRIP_TEST
    {
        EncoderState es;
        es.init(pNonZeroCount, &odata, pCtx, pNalCur, &rtd, uiCbpC, uiCbpL);
        WelsEnc::WelsSpatialWriteMbSyn (
            &es.pEncCtx, &es.pSlice, &es.pCurMb);
        assert(stringBitCompare(pBs, es.wrBs));
    }
#endif
  }

  return 0;
}

int32_t WelsDecodeMbCavlcISlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  PBitStringAux pBs = pCurLayer->pBitStringAux;
  PSliceHeaderExt pSliceHeaderExt = &pCurLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt;
  int32_t iBaseModeFlag;
  int32_t iRet = 0; //should have the return value to indicate decoding error or not, It's NECESSARY--2010.4.15
  uint32_t uiCode;
  intX_t iUsedBits;
  if (pSliceHeaderExt->bAdaptiveBaseModeFlag == 1) {
    WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //base_mode_flag
    iBaseModeFlag = uiCode;
  } else {
    iBaseModeFlag = pSliceHeaderExt->bDefaultBaseModeFlag;
  }
  if (!iBaseModeFlag) {
      iRet = WelsActualDecodeMbCavlcISlice (pCtx, pNalCur);
  } else {
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "iBaseModeFlag (%d) != 0, inter-layer prediction not supported.",
             iBaseModeFlag);
    return GENERATE_ERROR_NO (ERR_LEVEL_SLICE_HEADER, ERR_INFO_UNSUPPORTED_ILP);
  }
  if (iRet) { //occur error when parsing, MUST STOP decoding
    return iRet;
  }

  // check whether there is left bits to read next time in case multiple slices
  iUsedBits = ((pBs->pCurBuf - pBs->pStartBuf) << 3) - (16 - pBs->iLeftBits);
  // sub 1, for stop bit
  if ((iUsedBits == (pBs->iBits - 1)) && (0 >= pCurLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)) { // slice boundary
    uiEosFlag = 1;
  }
  if (iUsedBits > (pBs->iBits -
                   1)) { //When BS incomplete, as long as find it, SHOULD stop decoding to avoid mosaic or crash.
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
             "WelsDecodeMbCavlcISlice()::::pBs incomplete, iUsedBits:%" PRId64 " > pBs->iBits:%d, MUST stop decoding.",
             (int64_t) iUsedBits, pBs->iBits);
    return -1;
  }
  return 0;
}

int32_t WelsActualDecodeMbCavlcPSlice (PWelsDecoderContext pCtx) {
  SVlcTable* pVlcTable     = &pCtx->sVlcTable;
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBs              = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;

  RoundTripData rtd;

  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;

  SWelsNeighAvail sNeighAvail;
  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  const int32_t iMbXy = pCurLayer->iMbXyIndex;
  int8_t* pNzc = pCurLayer->pNzc[iMbXy];
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpL = 0, uiCbpC = 0;
  uint32_t uiCode;
  int32_t iCode;
  int32_t iMbResProperty;

  GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;//2009.10.23
  WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //uiMbType
  uiMbType = uiCode;
  if (uiMbType < 5) { //inter MB type
    int16_t iMotionVector[LIST_A][30][MV_A];
    int8_t  iRefIndex[LIST_A][30];
    pCurLayer->pMbType[iMbXy] = g_ksInterMbTypeInfo[uiMbType].iType;
    WelsFillCacheInter (&sNeighAvail, pNonZeroCount, iMotionVector, iRefIndex, pCurLayer);

    if (ParseInterInfo (pCtx, iMotionVector, iRefIndex, pBs, &rtd)) {
      return -1;//abnormal
    }

    if (pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag == 1) {
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //residual_prediction_flag
      pCurLayer->pResidualPredFlag[iMbXy] =  uiCode;
    } else {
      pCurLayer->pResidualPredFlag[iMbXy] = pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
    }

    if (pCurLayer->pResidualPredFlag[iMbXy] == 0) {
      pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
    } else {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "residual_pred_flag = 1 not supported.");
      return -1;
    }
  } else { //intra MB type
    uiMbType -= 5;
    if (uiMbType > 25)
      return ERR_INFO_INVALID_MB_TYPE;
    if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17 && uiMbType <= 24)))
      return ERR_INFO_INVALID_MB_TYPE;

    if (25 == uiMbType) {
      int32_t iDecStrideL = pCurLayer->pDec->iLinesize[0];
      int32_t iDecStrideC = pCurLayer->pDec->iLinesize[1];

      int32_t iOffsetL = (iMbX + iMbY * iDecStrideL) << 4;
      int32_t iOffsetC = (iMbX + iMbY * iDecStrideC) << 3;

      uint8_t* pDecY = pCurLayer->pDec->pData[0] + iOffsetL;
      uint8_t* pDecU = pCurLayer->pDec->pData[1] + iOffsetC;
      uint8_t* pDecV = pCurLayer->pDec->pData[2] + iOffsetC;

      uint8_t* pTmpBsBuf;

      int32_t i;
      int32_t iCopySizeY  = (sizeof (uint8_t) << 4);
      int32_t iCopySizeUV = (sizeof (uint8_t) << 3);

      int32_t iIndex = ((-pBs->iLeftBits) >> 3) + 2;

      pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA_PCM;

      //step 1: locating bit-stream pointer [must align into integer byte]
      pBs->pCurBuf -= iIndex;

      //step 2: copy pixel from bit-stream into fdec [reconstruction]
      pTmpBsBuf = pBs->pCurBuf;
      for (i = 0; i < 16; i++) { //luma
        memcpy (pDecY , pTmpBsBuf, iCopySizeY);
        pDecY += iDecStrideL;
        pTmpBsBuf += 16;
      }

      for (i = 0; i < 8; i++) { //cb
        memcpy (pDecU, pTmpBsBuf, iCopySizeUV);
        pDecU += iDecStrideC;
        pTmpBsBuf += 8;
      }
      for (i = 0; i < 8; i++) { //cr
        memcpy (pDecV, pTmpBsBuf, iCopySizeUV);
        pDecV += iDecStrideC;
        pTmpBsBuf += 8;
      }

      pBs->pCurBuf += 384;

      //step 3: update QP and pNonZeroCount
      pCurLayer->pLumaQp[iMbXy] = 0;
      pCurLayer->pChromaQp[iMbXy][0] = pCurLayer->pChromaQp[iMbXy][1] = 0;
      //Rec. 9.2.1 for PCM, nzc=16
      ST32A4 (&pNzc[0], 0x10101010);
      ST32A4 (&pNzc[4], 0x10101010);
      ST32A4 (&pNzc[8], 0x10101010);
      ST32A4 (&pNzc[12], 0x10101010);
      ST32A4 (&pNzc[16], 0x10101010);
      ST32A4 (&pNzc[20], 0x10101010);
      WELS_READ_VERIFY (InitReadBits (pBs, 0));
      return 0;
    } else {
      if (0 == uiMbType) {
        ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
        if (pCtx->pPps->bTransform8x8ModeFlag) {
          WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //transform_size_8x8_flag
          pCurLayer->pTransformSize8x8Flag[iMbXy] = !!uiCode;
          if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
            uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
          }
        }
        if (!pCurLayer->pTransformSize8x8Flag[iMbXy]) {
          pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, &rtd));
        } else {
          pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, &rtd));
        }
      } else { //I_PCM exclude, we can ignore it
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
        pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
        pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
        pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
        pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
        uiCbpC = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0;
        uiCbpL = pCurLayer->pCbp[iMbXy] & 15;
        WelsFillCacheNonZeroCount (&sNeighAvail, pNonZeroCount, pCurLayer);
        if (ParseIntra16x16Mode (pCtx, &sNeighAvail, pBs, pCurLayer)) {
          return -1;
        }
      }
    }
  }

  if (MB_TYPE_INTRA16x16 != pCurLayer->pMbType[iMbXy]) {
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //coded_block_pattern
    uiCbp = uiCode;
    {
      if (pCtx->pSps->uiChromaFormatIdc && (uiCbp > 47))
        return ERR_INFO_INVALID_CBP;
      if (!pCtx->pSps->uiChromaFormatIdc && (uiCbp > 15))
        return ERR_INFO_INVALID_CBP;
      if (MB_TYPE_INTRA4x4 == pCurLayer->pMbType[iMbXy] || MB_TYPE_INTRA8x8 == pCurLayer->pMbType[iMbXy]) {

        uiCbp = pCtx->pSps->uiChromaFormatIdc ? g_kuiIntra4x4CbpTable[uiCbp] : g_kuiIntra4x4CbpTable400[uiCbp];
      } else //inter
        uiCbp = pCtx->pSps->uiChromaFormatIdc ?  g_kuiInterCbpTable[uiCbp] : g_kuiInterCbpTable400[uiCbp];
    }

    pCurLayer->pCbp[iMbXy] = uiCbp;
    uiCbpC = pCurLayer->pCbp[iMbXy] >> 4;
    uiCbpL = pCurLayer->pCbp[iMbXy] & 15;

    // Need modification when B picutre add in
    bool bNeedParseTransformSize8x8Flag =
      (((pCurLayer->pMbType[iMbXy] >= MB_TYPE_16x16 && pCurLayer->pMbType[iMbXy] <= MB_TYPE_8x16)
        || pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
       && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA8x8)
       && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA4x4)
       && (uiCbpL > 0)
       && (pCtx->pPps->bTransform8x8ModeFlag));

    if (bNeedParseTransformSize8x8Flag) {
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //transform_size_8x8_flag
      pCurLayer->pTransformSize8x8Flag[iMbXy] = !!uiCode;
    }
  }

  ST32A4 (&pNzc[0], 0);
  ST32A4 (&pNzc[4], 0);
  ST32A4 (&pNzc[8], 0);
  ST32A4 (&pNzc[12], 0);
  ST32A4 (&pNzc[16], 0);
  ST32A4 (&pNzc[20], 0);
  if (pCurLayer->pCbp[iMbXy] == 0 && !IS_INTRA16x16 (pCurLayer->pMbType[iMbXy]) && !IS_I_BL (pCurLayer->pMbType[iMbXy])) {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }
  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
    int32_t iQpDelta, iId8x8, iId4x4;
    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, MB_COEFF_LIST_SIZE * sizeof (int16_t));
    WELS_READ_VERIFY (BsGetSe (pBs, &iCode)); //mb_qp_delta
    iQpDelta = iCode;

    if (iQpDelta > 25 || iQpDelta < -26) { //out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }

    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0,
                                       51)];
    }

    BsStartCavlc (pBs);
    if (oMovie().isRecoding) {
      int16_t lumaDC[16];
      int16_t chromaDC[8];
      int16_t lumaAC[256];
      int16_t chromaAC[128];
      uint8_t buf[384 * 2]; // Cannot be larger than raw input.
      memset(buf,0,sizeof(buf));
      memset(lumaDC,0,sizeof(lumaDC));
      memset(lumaAC,0,sizeof(lumaAC));
      memset(chromaDC,0,sizeof(chromaDC));
      memset(chromaAC,0,sizeof(chromaAC));
      BitStream::uint32E res;
      for (i = 0; i < 48; i++) {
        res = iMovie().tag(1).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read ldc!\n");
          pNonZeroCount[i] = 255;
        } else {
          pNonZeroCount[i] = res.first;
        }
      }
      if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
        for (int i = 0; i < 16; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read ldc!\n");
          }
          lumaDC[i] = res.first;
        }
      }
      if (1 == uiCbpC || 2 == uiCbpC) {
        for (int i = 0; i < 8; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read crdc!\n");
          }
          chromaDC[i] = res.first;
        }
      }
      if (uiCbpL) {
        for (int i = 0; i < 256; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read lac!\n");
          }
          lumaAC[i] = res.first;
        }
      }
      if (uiCbpC == 2) {
        for (int i = 0; i < 128; i++) {
          res = iMovie().tag(1).scanBits(16);
          if (res.second) {
            fprintf(stderr, "failed to read crac!\n");
          }
          chromaAC[i] = iMovie().tag(1).scanBits(16).first;
        }
      }

      SBitStringAux wrBs;
      InitBits (&wrBs, buf, sizeof(buf));
      WelsEnc::WelsUtilWriteMbResidual (
          gFuncPtrList, pCurLayer->pMbType[iMbXy], uiCbpC, uiCbpL,
          (int8_t*)pNonZeroCount, lumaDC, lumaAC, chromaDC, chromaAC, &wrBs);
      BsEndCavlc (pBs);
      for (uint8_t *ptr = wrBs.pStartBuf; ptr != wrBs.pCurBuf; ++ptr) {
        // fprintf(stderr, "Wrote some bytes\n");
        oMovie().def().emitBits(*ptr, 8);
      }
      if (wrBs.iLeftBits < 8) {
        int nBits = 8 - wrBs.iLeftBits;
        oMovie().def().emitBits(wrBs.uiCurBits >> (32 - nBits), nBits);
      }
      return 0;
    }
    // for (i = 0; i < 48; i++) oMovie().tag(1).writeBits(pNonZeroCount[i], 8);

    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, 0, 16, g_kuiLumaDcZigzagScan,
                                  I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx)) {
        return -1;//abnormal
      }
      for (i = 0; i < 256; i += 16) {
        oMovie().tag(1).emitBits(pCurLayer->pScaledTCoeff[iMbXy][i], 16);
      }
      //step2: Luma AC
      if (uiCbpL) {
        for (i = 0; i < 16; i++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, i,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      I16_LUMA_AC, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
            return -1;//abnormal
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              if (WelsResidualBlockCavlc8x8 (pVlcTable, pNonZeroCount, pBs, iIndex,
                                             iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart, iMbResProperty,
                                             pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), iId4x4, pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else { // Normal T4x4
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                          iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan + iScanIdxStart, iMbResProperty,
                                          pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;//abnormal
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }


    //chroma
    //step1: DC
    if (1 == uiCbpC || 2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        else
          iMbResProperty = i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

        if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs,
                                    16 + (i << 2), 4, g_kuiChromaDcScan, iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6),
                                    pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
          return -1;//abnormal
        }
      }
      for (i = 256; i < 384; i += 16) {
        oMovie().tag(1).emitBits(
              pCurLayer->pScaledTCoeff[iMbXy][i], 16);
      }
    } else if (uiCbpC != 0) {
      // FIXME(patrick): Encoder does not check for this condition...
      fprintf(stderr, "uiCbpC is %d not 1 or 2. Encoder might expect DC!\n",
              uiCbpC);
    }
    //step2: AC
    if (2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        else
          iMbResProperty = i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;

        int32_t iIndex = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
            return -1;//abnormal
          }
          iIndex++;
        }
      }
      ST16A2 (&pNzc[16], LD16A2 (&pNonZeroCount[6 + 8 * 1]));
      ST16A2 (&pNzc[20], LD16A2 (&pNonZeroCount[6 + 8 * 2]));
      ST16A2 (&pNzc[18], LD16A2 (&pNonZeroCount[6 + 8 * 4]));
      ST16A2 (&pNzc[22], LD16A2 (&pNonZeroCount[6 + 8 * 5]));
    }
    BsEndCavlc (pBs);
    if (uiCbpL) {
      for (i = 0; i < 256; i++) {
        oMovie().tag(1).emitBits(pCurLayer->pScaledTCoeff[iMbXy][i], 16);
      }
    }
    if (2 == uiCbpC) {
      for (i = 256; i < 384; i++) {
        oMovie().tag(1).emitBits(pCurLayer->pScaledTCoeff[iMbXy][i], 16);
      }
    }
    for (i = 0; i < 48; i++) {
      oMovie().tag(1).emitBits(pNonZeroCount[i], 8);
    }
  }

  return 0;
}

int32_t WelsDecodeMbCavlcPSlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBs              = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;
  PPicture* ppRefPic = pCtx->sRefPic.pRefList[LIST_0];
  intX_t iUsedBits;
  const int32_t iMbXy = pCurLayer->iMbXyIndex;
  int8_t* pNzc = pCurLayer->pNzc[iMbXy];
  int32_t iBaseModeFlag, i;
  int32_t iRet = 0; //should have the return value to indicate decoding error or not, It's NECESSARY--2010.4.15
  uint32_t uiCode;

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;

  if (-1 == pSlice->iMbSkipRun) {
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //mb_skip_run
    pSlice->iMbSkipRun = uiCode;
    if (-1 == pSlice->iMbSkipRun) {
      return -1;
    }
  }
  if (pSlice->iMbSkipRun--) {
    int16_t iMv[2];

    pCurLayer->pMbType[iMbXy] = MB_TYPE_SKIP;
    ST32A4 (&pNzc[0], 0);
    ST32A4 (&pNzc[4], 0);
    ST32A4 (&pNzc[8], 0);
    ST32A4 (&pNzc[12], 0);
    ST32A4 (&pNzc[16], 0);
    ST32A4 (&pNzc[20], 0);

    pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
    memset (pCurLayer->pRefIndex[0][iMbXy], 0, sizeof (int8_t) * 16);
    pCtx->bMbRefConcealed = pCtx->bRPLRError || pCtx->bMbRefConcealed || ! (ppRefPic[0] && ppRefPic[0]->bIsComplete);
    //predict iMv
    PredPSkipMvFromNeighbor (pCurLayer, iMv);
    for (i = 0; i < 16; i++) {
      ST32A2 (pCurLayer->pMv[0][iMbXy][i], * (uint32_t*)iMv);
    }

    //if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
    //  memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (int16_t));
    //}

    //reset rS
    if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag ||
        (pNalCur->sNalHeaderExt.uiQualityId == 0 && pNalCur->sNalHeaderExt.uiDependencyId == 0)) {
      pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
      for (i = 0; i < 2; i++) {
        pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                         pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
      }
    }

    pCurLayer->pCbp[iMbXy] = 0;
  } else {
    if (pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag == 1) {
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //base_mode_flag
      iBaseModeFlag = uiCode;
    } else {
      iBaseModeFlag = pSlice->sSliceHeaderExt.bDefaultBaseModeFlag;
    }
    if (!iBaseModeFlag) {
      iRet = WelsActualDecodeMbCavlcPSlice (pCtx);
    } else {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "iBaseModeFlag (%d) != 0, inter-layer prediction not supported.",
               iBaseModeFlag);
      return GENERATE_ERROR_NO (ERR_LEVEL_SLICE_HEADER, ERR_INFO_UNSUPPORTED_ILP);
    }
    if (iRet) { //occur error when parsing, MUST STOP decoding
      return iRet;
    }
  }
  // check whether there is left bits to read next time in case multiple slices
  iUsedBits = ((pBs->pCurBuf - pBs->pStartBuf) << 3) - (16 - pBs->iLeftBits);
  // sub 1, for stop bit
  if ((iUsedBits == (pBs->iBits - 1)) && (0 >= pCurLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)) { // slice boundary
    uiEosFlag = 1;
  }
  if (iUsedBits > (pBs->iBits -
                   1)) { //When BS incomplete, as long as find it, SHOULD stop decoding to avoid mosaic or crash.
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
             "WelsDecodeMbCavlcISlice()::::pBs incomplete, iUsedBits:%" PRId64 " > pBs->iBits:%d, MUST stop decoding.",
             (int64_t) iUsedBits, pBs->iBits);
    return -1;
  }
  return 0;
}

void WelsBlockFuncInit (SBlockFunc*   pFunc,  int32_t iCpu) {
  pFunc->pWelsSetNonZeroCountFunc   = WelsNonZeroCount_c;
  pFunc->pWelsBlockZero16x16Func    = WelsBlockZero16x16_c;
  pFunc->pWelsBlockZero8x8Func      = WelsBlockZero8x8_c;

#ifdef HAVE_NEON
  if (iCpu & WELS_CPU_NEON) {
    pFunc->pWelsSetNonZeroCountFunc = WelsNonZeroCount_neon;
    pFunc->pWelsBlockZero16x16Func  = WelsBlockZero16x16_neon;
    pFunc->pWelsBlockZero8x8Func    = WelsBlockZero8x8_neon;
  }
#endif

#ifdef HAVE_NEON_AARCH64
  if (iCpu & WELS_CPU_NEON) {
    pFunc->pWelsSetNonZeroCountFunc = WelsNonZeroCount_AArch64_neon;
    pFunc->pWelsBlockZero16x16Func  = WelsBlockZero16x16_AArch64_neon;
    pFunc->pWelsBlockZero8x8Func    = WelsBlockZero8x8_AArch64_neon;
  }
#endif

#if defined(X86_ASM)
  if (iCpu & WELS_CPU_SSE2) {
    pFunc->pWelsSetNonZeroCountFunc = WelsNonZeroCount_sse2;
    pFunc->pWelsBlockZero16x16Func  = WelsBlockZero16x16_sse2;
    pFunc->pWelsBlockZero8x8Func    = WelsBlockZero8x8_sse2;
  }
#endif

}

void WelsBlockInit (int16_t* pBlock, int iW, int iH, int iStride, uint8_t uiVal) {
  int32_t i;
  int16_t* pDst = pBlock;

  for (i = 0; i < iH; i++) {
    memset (pDst, uiVal, iW * sizeof (int16_t));
    pDst += iStride;
  }
}
void WelsBlockZero16x16_c (int16_t* pBlock, int32_t iStride) {
  WelsBlockInit (pBlock, 16, 16, iStride, 0);
}

void WelsBlockZero8x8_c (int16_t* pBlock, int32_t iStride) {
  WelsBlockInit (pBlock, 8, 8, iStride, 0);
}

} // namespace WelsDec
