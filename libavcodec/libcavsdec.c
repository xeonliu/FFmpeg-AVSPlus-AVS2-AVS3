/*
 * Chinese AVS video (AVS1-P2, JiZhun profile, frame and field) decoder.
 * Chinese AVS video (AVS1-P16, GuanDian profile, frame and field) decoder.
 * Copyright (c) 2013  
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */


#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "internal.h"
#include "mpeg12data.h"
#include "mpegvideo.h"
#include "libcavs.h"
#include "startcode.h"

#define SLICE_MAX_START_CODE    0x000001af
#define EXT_START_CODE          0x000001b5
#define USER_START_CODE         0x000001b2
#define CAVS_START_CODE         0x000001b0
#define PIC_I_START_CODE        0x000001b3
#define PIC_PB_START_CODE       0x000001b6
#define VIDEO_EDIT_CODE          0x000001b7

#define A_AVAIL                          1
#define B_AVAIL                          2
#define C_AVAIL                          4
#define D_AVAIL                          8
#define NOT_AVAIL                       -1
#define REF_INTRA                       -2
#define REF_DIR                         -3

#define ESCAPE_CODE                     59

#define FWD0                          0x01
#define FWD1                          0x02
#define BWD0                          0x04
#define BWD1                          0x08
#define SYM0                          0x10
#define SYM1                          0x20
#define SPLITH                        0x40
#define SPLITV                        0x80

#define MV_BWD_OFFS                     12
#define MV_STRIDE                        4

typedef struct AVSFrame {
    AVFrame *f;
    int poc;
} AVSFrame;

typedef struct AVSContext {
    AVCodecContext *avctx;

    uint8_t *buf; /* decode stream */
    int length;
    void* p_decoder; /* decoder handle */
    int b_interlaced; /* 0:frame 1:field */
    cavs_param param;
    int b_probe_flag; /* init as 0, when finish probe set to 1 */
    
    AVSFrame cur;     ///< currently decoded frame
    AVSFrame DPB[2];  ///< reference frames
    
    int dist[2];     ///< temporal distances from current frame to ref frames
    int low_delay;
    int profile, level;
    int aspect_ratio;
    int mb_width, mb_height;
    int width, height;
    int stream_revision; ///<0 for samples from 2006, 1 for rm52j encoder
    int progressive;
    int pic_structure;

    int stc;           ///< last start code
    uint8_t *top_qp;
    int got_keyframe;
    int probe_keyframe;

    int output_type; /* mark type of output frame */
    int b_delay_frame;
    int b_last_delay_frame;
	
	int last_frame_error; /* default 0 */
    int last_frame_type;
} AVSContext;

int ff_libcavs_init(AVCodecContext *avctx);
int ff_libcavs_end (AVCodecContext *avctx);

av_cold int ff_libcavs_init(AVCodecContext *avctx) {
    AVSContext *h = avctx->priv_data;
    int i_result;
    
    h->avctx = avctx;
    avctx->pix_fmt= AV_PIX_FMT_YUV420P;

    h->cur.f    = av_frame_alloc();
    h->DPB[0].f = av_frame_alloc();
    h->DPB[1].f = av_frame_alloc();
    
    if (!h->cur.f || !h->DPB[0].f || !h->DPB[1].f) {
        ff_libcavs_end(avctx);
        return AVERROR(ENOMEM);
    }

    /* creat decoder */
	h->param.b_accelerate = 1;
    i_result = cavs_decoder_create( &h->p_decoder, &h->param );
    if( h->p_decoder == NULL || i_result != 0 )
    {
        return -1;
    }

    /*init multi-thread*/
    cavs_decoder_thread_param_init( h->p_decoder );

    /* alloc buf for decode */
    h->buf = (unsigned char *)malloc(MAX_CODED_FRAME_SIZE*sizeof(unsigned char));
    memset(h->buf, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));

    /* init probe flag */
    h->b_probe_flag = 0;
    h->b_last_delay_frame = 0;
    h->b_delay_frame = 0;
    
    h->last_frame_error= 0;
    h->probe_keyframe = 0;
	
    return 0;
}

av_cold int ff_libcavs_end(AVCodecContext *avctx) {
    AVSContext *h = avctx->priv_data;
    
    av_frame_free(&h->cur.f);
    av_frame_free(&h->DPB[0].f);
    av_frame_free(&h->DPB[1].f);

    free( h->buf );
    cavs_decoder_destroy( h->p_decoder );
    cavs_decoder_buffer_end( &h->param );

    return 0;
}



/*****************************************************************************
 *
 * frame level
 *
 ****************************************************************************/

static int decoder_get_cur_frame( AVFrame *frame, cavs_param * param)
{
    int width;
    int height;
    int linesize;
    int i;
    uint8_t *dst_y, *dst_u, *dst_v;
    uint8_t *p_yuv;

    width = param->seqsize.lWidth;
    height = param->seqsize.lHeight;

    /* Y */
    dst_y = frame->data[0];
    p_yuv = param->p_out_yuv[0];
    linesize = frame->linesize[0];
    for( i = 0; i < height; i++ )
    {
        memcpy( dst_y, p_yuv, width);
        dst_y += linesize;
        p_yuv += width;
    }

    /* U */
    dst_u = frame->data[1];
    p_yuv = param->p_out_yuv[1];
    linesize = frame->linesize[1];
    for( i = 0; i < (height>>1); i++ )
    {
        memcpy( dst_u, p_yuv, width>>1);
        dst_u += linesize;
        p_yuv += (width>>1);
    }

    /* V */
    dst_v = frame->data[2];
    p_yuv = param->p_out_yuv[2];
    linesize = frame->linesize[2];
    for( i = 0; i < (height>>1); i++ )
    {
        memcpy( dst_v, p_yuv, width>>1);
        dst_v += linesize;
        p_yuv += (width>>1);
    }

    return 0;
}

static int cavs_decode_pic(AVSContext *h, int *ret_val)
{
    int ret;
    int i_result;

    if( h->param.b_accelerate )
    {
        i_result = cavs_decode_one_frame( h->p_decoder, h->stc, &h->param, h->buf, h->length );
        av_frame_unref(h->cur.f);
        
        h->output_type = h->param.output_type;
        switch( h->output_type )
        {
            case 0: /* I-frame */
                h->output_type = AV_PICTURE_TYPE_I;
                break;
            case 1: /* P-frame */
                h->output_type = 1 + AV_PICTURE_TYPE_I;
                break;
            case 2: /* B-frame */
                h->output_type = 2 + AV_PICTURE_TYPE_I;
                break;
            default:
    			;
        }
        
        if( h->output_type == -1 && i_result == 0 )
            return 0;

        if ((ret = ff_get_buffer(h->avctx, h->cur.f,
                                 h->output_type == AV_PICTURE_TYPE_B ?
                                 0 : AV_GET_BUFFER_FLAG_REF)) < 0)
            return ret;

        /* set h->cur with decoded image */
        if( i_result == CAVS_FRAME_OUT )
        {
            decoder_get_cur_frame( h->cur.f, &h->param );
         
            if ( h->output_type  != AV_PICTURE_TYPE_B )
            {           
                av_frame_unref(h->DPB[1].f);
                FFSWAP(AVSFrame, h->cur, h->DPB[1]);
                FFSWAP(AVSFrame, h->DPB[0], h->DPB[1]);  
            }  
            else /* B frame */
            {
                decoder_get_cur_frame( h->cur.f, &h->param );
            }
        }
    }
    else
    {
        i_result = cavs_decode_one_frame( h->p_decoder, h->stc, &h->param, h->buf, h->length );
        av_frame_unref(h->cur.f);

        if (h->stc == PIC_PB_START_CODE) {
            h->cur.f->pict_type = cavs_decoder_cur_frame_type( h->p_decoder ) + AV_PICTURE_TYPE_I;
            if (h->cur.f->pict_type > AV_PICTURE_TYPE_B) {
                av_log(h->avctx, AV_LOG_ERROR, "illegal picture type\n");
                printf("illegal picture type\n");
                return -1;
            }
        } else {
            h->cur.f->pict_type = AV_PICTURE_TYPE_I;
        }

        if ((ret = ff_get_buffer(h->avctx, h->cur.f,
                                 h->cur.f->pict_type == AV_PICTURE_TYPE_B ?
                                 0 : AV_GET_BUFFER_FLAG_REF)) < 0)
            return ret;

        /* set h->cur with decoded image */
        if( i_result == CAVS_FRAME_OUT )
        {
            decoder_get_cur_frame( h->cur.f, &h->param );
         
            if (h->cur.f->pict_type != AV_PICTURE_TYPE_B)
            {           
                av_frame_unref(h->DPB[1].f);
                FFSWAP(AVSFrame, h->cur, h->DPB[1]);
                FFSWAP(AVSFrame, h->DPB[0], h->DPB[1]);  
            }  
            else /* B frame */
            {
                decoder_get_cur_frame( h->cur.f, &h->param );
            }
        }
    }

    *ret_val = i_result;
    
    return 0;
}

/*****************************************************************************
 *
 * headers and interface
 *
 ****************************************************************************/

static int cavs_decode_seq_header(AVSContext *h)
{
    int frame_rate_code;
    int width, height;
    cavs_seq_info si;
    
    cavs_decoder_process( h->p_decoder, h->buf, h->length );
    
    cavs_decoder_get_seq( h->p_decoder,  &si);

    h->b_interlaced = si.b_interlaced;
    h->profile = si.profile;
    h->avctx->profile = si.profile;
    h->level = si.level;  
    width  = si.lWidth;
    height = si.lHeight;
    if ((h->width || h->height) && (h->width != width || h->height != height)) {
        avpriv_report_missing_feature(h->avctx,
                                      "Width/height changing in BAVS");
        return AVERROR_PATCHWELCOME;
    }
    if (width <= 0 || height <= 0) {
        av_log(h->avctx, AV_LOG_ERROR, "Dimensions invalid\n");
        return AVERROR_INVALIDDATA;
    }
    h->width  = width;
    h->height = height;
    h->aspect_ratio = si.aspect_ratio;
    frame_rate_code = si.frame_rate_code;
    h->low_delay = si.low_delay;
    h->mb_width  = (h->width  + 15) >> 4;
    h->mb_height = (h->height + 15) >> 4;
    h->avctx->time_base.den = ff_mpeg12_frame_rate_tab[frame_rate_code].num;
    h->avctx->time_base.num = ff_mpeg12_frame_rate_tab[frame_rate_code].den;
    h->avctx->width  = h->width;
    h->avctx->height = h->height;

	h->cur.f->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
	h->DPB[0].f->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
	h->DPB[1].f->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
		
    return 0;
}

static int cavs_decode_extension_data(AVSContext *h)
{    
    cavs_decoder_process( h->p_decoder, h->buf, h->length );

    return 0;
}

static void cavs_flush(AVCodecContext * avctx)
{
    AVSContext *h = avctx->priv_data;
    h->got_keyframe = 0;
}

static int cavs_decoder_param_init( AVSContext *h )
{
    cavs_param *param = &h->param;

    //param->b_interlaced = h->b_interlaced; /* frame or field decision at probe stage */
    param->cpu = 0;
    param->i_color_space = CAVS_CS_YUV420; /*only support 420*/
    //param->i_thread_model = 0; /* don't modify!! 1:multi-threads 0:single( new acceleration ) */
    param->i_thread_num = 64; /* don't modify!! at most */
    param->seqsize.lWidth = h->width;
    param->seqsize.lHeight = h->height;
    param->seqsize.lWidth = h->width;
    param->seqsize.lHeight = h->height;
    param->output_type = -1; /* -1 init, I-0, P-1 B-2 */
    
    return 0;
}

#define STREAM_DATA_MAX 3000000

static int cavs_decode_frame(AVCodecContext *avctx, AVFrame *frame, int *got_frame, AVPacket *avpkt)
{
    AVSContext *h      = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    uint32_t stc       = -1;
    int ret = 0;
    uint8_t *buf_end;
    uint8_t *buf_ptr;
    int i_pass_idr = 0;
    int b_delay_out = 0;
    int i_first_adr = 0;

    /* probe frame or field decoded format */
    if( !h->b_probe_flag && buf_size != 0 ) /* first probe flag, when finish set it to 1 */
    {
        uint8_t *p_buffer;
        uint8_t *buf_cur, *buf_cur_end;
        int probe_size;
        int decode_size;

        probe_size = buf_size;
        decode_size = probe_size;

        /* copy  stream */
        p_buffer = (uint8_t*)malloc(buf_size);
        
        memcpy( p_buffer, buf, probe_size );

        buf_cur = p_buffer; /* used to cal */
        buf_cur_end = buf_cur + probe_size;

        while( !h->b_probe_flag )
        {
            buf_cur = (uint8_t*)avpriv_find_start_code((const uint8_t*)buf_cur, (const uint8_t*)buf_cur_end, &stc);
            if ((stc & 0xFFFFFE00) || buf_cur == buf_cur_end)
            {
                return FFMAX(0, buf_cur - p_buffer);
            }

            decode_size = buf_cur_end - buf_cur + 4;

            switch(stc)
            {
                case CAVS_START_CODE:
                    cavs_decoder_init_stream( h->p_decoder, buf_cur - 4, decode_size);
                    cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );
                    ret = cavs_decoder_probe_seq( h->p_decoder, h->buf, h->length );
                    if(ret == CAVS_ERROR )
                    {
                        break;
                    }
                    
                    break;
                case PIC_I_START_CODE:
                    cavs_decoder_init_stream( h->p_decoder, buf_cur - 4, decode_size);
                    cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length ); 
                    ret = cavs_decoder_pic_header( h->p_decoder,  h->buf, h->length, &h->param, stc );
                    if( ret == CAVS_ERROR )
                    {
                        break;
                    }
                    cavs_decoder_set_format_type( h->p_decoder, &h->param );

                    /* finish probe flag */
                    h->b_probe_flag = 1;
                    if(!h->probe_keyframe )
                    {
                        h->probe_keyframe = 1;
                    }
                        
                    break;
                    
                case PIC_PB_START_CODE:
                    if(!h->probe_keyframe )
                    {
                        break;    
                    }
                    cavs_decoder_init_stream( h->p_decoder, buf_cur - 4, decode_size);
                    cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );
                    ret = cavs_decoder_pic_header( h->p_decoder,  h->buf, h->length, &h->param, stc );
                    if( ret == CAVS_ERROR )
                    {
                        break;
                    }
                    cavs_decoder_set_format_type( h->p_decoder, &h->param );

                    /* finish probe flag */   
                    h->b_probe_flag = 1;
                    break;
                    
                case EXT_START_CODE:
                
                    cavs_decoder_init_stream( h->p_decoder, buf_cur - 4, decode_size);
                    cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );

                    cavs_decode_extension_data( h );

                    break;
                    
                case USER_START_CODE:
                    cavs_decoder_init_stream( h->p_decoder, buf_cur - 4, decode_size);
                    cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );

                    cavs_decode_extension_data( h );

                    break;

                case VIDEO_EDIT_CODE:
                    cavs_decoder_init_stream( h->p_decoder, buf_cur - 4, decode_size);
                    cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );

                    break;
                default:
                    if (stc <= SLICE_MAX_START_CODE) {
                    }
                    break;
            }    
        }

        free(p_buffer);
    }

    if (buf_size == 0) {

        if( h->param.b_accelerate )
        {
             int ret = 0;

            /* when last frame is IDR */
            if(!h->b_last_delay_frame)
            {
    	         h->b_last_delay_frame = 1;
    	         ret = cavs_decode_one_frame_delay( h->p_decoder, &h->param );
                
                h->output_type = h->param.output_type;

    	        if( ret == CAVS_FRAME_OUT )
                {

          
                    av_frame_unref(h->cur.f);
                    if ((ret = ff_get_buffer(h->avctx, h->cur.f,
                                 h->output_type == AV_PICTURE_TYPE_B ?
                                 0 : AV_GET_BUFFER_FLAG_REF)) < 0)
                    return ret;

                    decoder_get_cur_frame( h->cur.f, &h->param );
                    *got_frame = 1;
                    av_frame_move_ref(frame, h->cur.f);
                     
					frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
	
                    return 0;
               }
            }        
        }

        if (!h->low_delay && h->DPB[0].f->data[0]) {
            if(cavs_out_delay_frame_end( h->p_decoder, h->param.p_out_yuv) )
            {
                
                decoder_get_cur_frame( h->DPB[0].f, &h->param );
                *got_frame = 1;
                av_frame_move_ref(frame, h->DPB[0].f);
				frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
            }
        }

        return 0;
    }
    
    buf_ptr = (uint8_t*)buf;
    buf_end = (uint8_t*)(buf + buf_size);
	
    for(;;) {

        int decode_size;

        if(!i_first_adr)
        {
            i_first_adr = 1;
        }

        buf_ptr = (uint8_t*)avpriv_find_start_code((const uint8_t*)buf_ptr, (const uint8_t*)buf_end, &stc);
        
        if ((stc & 0xFFFFFE00) || buf_ptr == buf_end)
        {
            return FFMAX(0, buf_ptr - buf);
        }

        decode_size = buf_end - buf_ptr + 4;

        switch (stc) {
        case CAVS_START_CODE:
            cavs_decoder_init_stream( h->p_decoder, buf_ptr - 4, decode_size);
            cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );
            cavs_decode_seq_header( h );

            /* init h->param */
            cavs_decoder_param_init( h );

            if( h->param.seq_header_flag == 0 )
            {
                /* NOTE : must call at first seq header */
                cavs_decoder_seq_init( h->p_decoder, &h->param );
                
                /* alloc frame for decoded yuv */
                if (cavs_decoder_buffer_init(&h->param) < 0)
                    break;
                
                h->param.seq_header_flag = 1;
            }
            else
            {
                if( h->param.b_accelerate )
                {
                    h->b_delay_frame = 0; // xun
                    if( h->last_frame_error )
                    {
                        /* reset pipeline */
                        cavs_decoder_seq_header_reset_pipeline( h->p_decoder );
                        h->b_delay_frame = 0;
                    }
                }
#if 0	//xun
                else
                {
                    //output final p frame if bframe num != 0
                    if ( cavs_out_delay_frame( h->p_decoder, h->param.p_out_yuv) )
                    {                
                        b_delay_out = 1; /* will output this frame after current frame decoded */
                    }
                }   
#endif
            }

            if( h->last_frame_error ) /* error end */
            {
                 h->last_frame_error = 0;
            }
            break;
        case PIC_I_START_CODE:
            cavs_decoder_init_stream( h->p_decoder, buf_ptr - 4, decode_size);
            cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length ); 
            if (!h->got_keyframe) {
                av_frame_unref(h->DPB[0].f);
                av_frame_unref(h->DPB[1].f);
                h->got_keyframe = 1;
            }
            i_pass_idr = 1; 
            if( h->last_frame_error ) /* error end */
            {
                 h->last_frame_error = 0;
            }
        case PIC_PB_START_CODE:
            if( !i_pass_idr )
            {
                cavs_decoder_init_stream( h->p_decoder, buf_ptr - 4, decode_size);
                cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );
                i_pass_idr = 0;
                if( h->last_frame_error )
                {
                    *got_frame = 0;
                    return 0;
                }
            }
            
            *got_frame = 0;
            if (!h->got_keyframe)
                break;
            h->stc = stc;
#if 0
            if (cavs_decode_pic(h, &ret))
            {
                break;
            }
#else
            cavs_decode_pic(h, &ret);
            if( ret == CAVS_ERROR )
            {
                h->last_frame_error = 1;
                h->got_keyframe = 0;
                break;
            }

#endif            
            if( ret == CAVS_FRAME_OUT )
            {
                *got_frame = 1;
            }

            if( h->param.b_accelerate )  /* output frame according to reconstructed order */
            {
                if( h->output_type != AV_PICTURE_TYPE_B )
                {
                    if (h->DPB[0].f->data[0] && *got_frame == 1) 
                    {
                        if ((ret = av_frame_ref(frame, h->DPB[0].f)) < 0)
                        {
                            return ret;
                        }
						frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
                    }
                    else
                    {
                        {
                            *got_frame = 0;
                        }
                    }      
                }
                else  /* B-frame, output */
                {
                    if(h->output_type == AV_PICTURE_TYPE_B && *got_frame == 1)
                    {   
                        av_frame_move_ref(frame, h->cur.f);
						frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
                    } 
                }
            }
            else
            {
                if (h->cur.f->pict_type != AV_PICTURE_TYPE_B)
                {
                    if (h->DPB[0].f->data[0] && *got_frame == 1) 
                    {
                        if ((ret = av_frame_ref(frame, h->DPB[0].f)) < 0)
                        {
                            return ret;
                        }
						frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
                    }
                    else
                    {
                        if( b_delay_out )
                        {                    
                            b_delay_out = 0;
                            
                            /* set h->cur.f */
                            decoder_get_cur_frame( h->cur.f, &h->param );

                            *got_frame = 1;
                            av_frame_move_ref(frame, h->cur.f);
							frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
                        }
                        else
                        {
                            *got_frame = 0;
                        }
                    }
                }
                else /* B-frame, output */
                {
                    if(*got_frame == 1)
                    {   
                        av_frame_move_ref(frame, h->cur.f);
						frame->flags = h->b_interlaced ? AV_FRAME_FLAG_INTERLACED : 0;
                    }
                }
            }

            break;
        case EXT_START_CODE:
            cavs_decoder_init_stream( h->p_decoder, buf_ptr - 4, decode_size);
            cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );

            cavs_decode_extension_data( h );

            break;
        case USER_START_CODE:
            cavs_decoder_init_stream( h->p_decoder, buf_ptr - 4, decode_size);
            cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );

            cavs_decode_extension_data( h );

            break;

        case VIDEO_EDIT_CODE:
            cavs_decoder_init_stream( h->p_decoder, buf_ptr - 4, decode_size);
            cavs_decoder_get_one_nal( h->p_decoder, h->buf, &h->length );
            
            break;
        default:
            if (stc <= SLICE_MAX_START_CODE) {
            }
            break;
        }
    }
}

const FFCodec ff_cavs_decoder = {
    .p.name           = "cavs",    
    .p.long_name    = NULL_IF_CONFIG_SMALL("Chinese AVS (Audio Video Standard) (AVS1-P2, JiZhun profile) and (AVS1-P16, Guangdian profile)"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_CAVS,
    .priv_data_size = sizeof(AVSContext),
    .init           = ff_libcavs_init,
    .close          = ff_libcavs_end,
    FF_CODEC_DECODE_CB(cavs_decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_OTHER_THREADS,
    .caps_internal  = FF_CODEC_CAP_AUTO_THREADS,
    .p.pix_fmts     = (const enum AVPixelFormat[]) { AV_PIX_FMT_YUV420P,
                                                     AV_PIX_FMT_NONE },
    .flush          = cavs_flush,
};
