#ifndef ENCODER_FROM_DECODER_H__
#define ENCODER_FROM_DECODER_H__

#undef WELS_ACCESS_UNIT_PARSER_H__
#undef WELS_BIT_STREAM_H__
#undef WELS_CABAC_DECODER_H__
#undef _COMPRESSION_STREAM_H_
#undef WELS_DEBLOCKING_H__
#undef WELS_DEC_FRAME_H__
#undef WELS_EXPONENTIAL_GOLOMB_ENTROPY_CODING_H__
#undef WELS_DECODE_MB_AUX_H__
#undef WELS_DECODE_SLICE_H__
#undef WELS_DECODER_SYSTEM_ARCHITECTURE_H__
#undef WELS_DECODER_FRAMEWORK_H__
#undef WELS_DECODER_CORE_H__
#undef WELS_ERROR_CODE_H__
#undef WELS_ERROR_CONCEALMENT_H__
#undef WELS_FLEXIBLE_MACROBLOCK_ORDERING_H__
#undef WELS_GET_INTRA_PREDICTOR_H__
#undef WELS_MANAGE_DEC_REF_H__
#undef WELS_MACROBLOCK_CACHE_H__
#undef WELS_MEMORY_MANAGER_NAL_UNIT_H__
#undef WELS_MV_PRED_H__
#undef WELS_NAL_UNIT_PREFIX_H__
#undef WELS_NAL_UNIT_H__
#undef WELS_PARAMETER_SETS_H__
#undef WELS_PARSE_MB_SYN_CABAC_H__
#undef WELS_PARSE_MB_SYN_CAVLC_H__
#undef WELS_PICTURE_QUEUE_H__
#undef WELS_PICTURE_H__
#undef WELS_REC_MB_H__
#undef WELS_SLICE_H__
#undef WELS_VLC_DECODER_H__
#undef WELS_COMMON_BASIS_H__
#undef WELS_CONST_H__

#undef MAX_PPS_COUNT // different in encoder and decoder
#undef MAX_SHORT_REF_COUNT // different in encoder and decoder

#include "../../../encoder/core/inc/wels_common_basis.h"
#include "../../../encoder/core/inc/encoder_context.h"
#include "../../../encoder/core/inc/md.h"
#include "../../../encoder/core/inc/mv_pred.h"

namespace WelsEnc {
int32_t WelsSpatialWriteMbSynCabac (sWelsEncCtx* pCtx, SSlice* pSlice, SMB* pCurMb);
int32_t WelsSpatialWriteMbSyn (sWelsEncCtx* pCtx, SSlice* pSlice, SMB* pCurMb);
}

#undef MAX_PPS_COUNT // make sure it is not used.
#undef MAX_SHORT_REF_COUNT

#endif
