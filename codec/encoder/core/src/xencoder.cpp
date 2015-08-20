
#include "xencoder.h"

extern "C" {
#include "x264.h"
}

class XEncoderState
{
public:
    x264_picture_t pic_buf;
    x264_t *h_encoder;
};

XEncoder::XEncoder()
{
    this->state = new XEncoderState();
    
    x264_param_t param;

    if( x264_param_default_preset( &param, "medium", NULL ) < 0 )
        throw;

    /* Configure non-default params */
    param.i_csp = X264_CSP_I420;
    param.i_width  = 320;
    param.i_height = 240;
    param.b_vfr_input = 0;
    param.b_repeat_headers = 1;
    param.b_annexb = 1;

    /* Apply profile restrictions. */
    if( x264_param_apply_profile( &param, "high" ) < 0 )
        throw;

    if( x264_picture_alloc( &this->state->pic_buf, param.i_csp, param.i_width, param.i_height ) < 0 )
        throw;

    this->state->h_encoder = x264_encoder_open( &param );
    if( !this->state->h_encoder )
        throw;
}

XEncoder::~XEncoder()
{
    x264_encoder_close( this->state->h_encoder );
    x264_picture_clean( &this->state->pic_buf );
    delete this->state;
}
