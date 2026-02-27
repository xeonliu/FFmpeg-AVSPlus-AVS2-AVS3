/*****************************************************************************
* libcavs.h: the header file of avs/avs+ decoder 
* Copyright (c) 2021 maliwen2015/ffmpeg_cavs_dra
*****************************************************************************
*/

/*
 * Chinese AVS video (AVS1-P2, JiZhun profile) decoder.
 * Chinese AVS video (AVS1-P16, Guangdian profile) decoder.
*/

#ifndef _CAVS_DECODER_H
#define _CAVS_DECODER_H

#ifdef __cplusplus
extern "C" {    // only need to export C interface if used by C++ source code
#endif

#include <stdio.h>

typedef struct tagcavs_decoder cavs_decoder;

#define SLICE_MAX_START_CODE    0x000001af
#define EXT_START_CODE          0x000001b5
#define USER_START_CODE         0x000001b2
#define CAVS_START_CODE         0x000001b0
#define PIC_I_START_CODE        0x000001b3
#define PIC_PB_START_CODE       0x000001b6

#define CAVS_SEQ_HEADER  1
#define CAVS_FRAME_OUT   2
#define CAVS_SEQ_END	 4
#define CAVS_ERROR		 8
#define CAVS_USER_DATA   16

#define CAVS_CS_YUV420 0

#define CAVS_SEQUENCE_DISPLAY_EXTENTION     0x00000002
#define CAVS_COPYRIGHT_EXTENTION            0x00000004
#define CAVS_PICTURE_DISPLAY_EXTENTION      0x00000007
#define CAVS_CAMERA_PARAMETERS_EXTENTION    0x0000000B
#define CAVS_SLICE_MIN_START_CODE           0x00000100
#define CAVS_SLICE_MAX_START_CODE           0x000001AF
#define CAVS_VIDEO_SEQUENCE_START_CODE      0x000001B0
#define CAVS_VIDEO_SEQUENCE_END_CODE        0x000001B1
#define CAVS_USER_DATA_CODE                 0x000001B2
#define CAVS_I_PICUTRE_START_CODE           0x000001B3
#define CAVS_EXTENSION_START_CODE           0x000001B5
#define CAVS_PB_PICUTRE_START_CODE          0x000001B6
#define CAVS_VIDEO_EDIT_CODE                0x000001B7
#define CAVS_VIDEO_TIME_CODE                0x000001E0

#define MAX_CODED_FRAME_SIZE 4000000         //!< bytes for one frame

typedef struct tagcavs_seq_info
{
    long lWidth;
    long lHeight;
    int i_frame_rate_den;
    int i_frame_rate_num;
    int b_interlaced;

    int profile;
    int level;
    int aspect_ratio;
    int low_delay;
    int mb_width;
    int mb_height;
    int frame_rate_code;
}cavs_seq_info;

typedef struct tagcavs_param
{
    unsigned int   i_color_space; /* color space , only support 4:2:0 now */
    unsigned char *p_out_yuv[3]; /* memory for decoded frame */
    unsigned int  seq_header_flag; /* first seq header flag */
    
    int i_thread_num;	/* number of threads (max 64)*/

    cavs_seq_info	seqsize; /* infos of seq header */

    int cpu; /* detect cpu type */
    int fld_mb_end[2]; /* 0 : top 1 : bot */
    int b_interlaced; /* 0 : frame 1 : field */
    int b_accelerate; /* 0: off 1: on */

    int output_type; /* mark output type, init as -1, I-0, P-1, B-2 */
}cavs_param;

/* creat handle of decoder */
int cavs_decoder_create( void **pp_decoder, cavs_param *param );

/* init stream for decoding */
int cavs_decoder_init_stream( void *p_decoder, unsigned char *rawstream, unsigned int i_len );

/* get one nal from input stream */
int cavs_decoder_get_one_nal( void *p_decoder, unsigned char *buf,/*int *startcodepos,*/int *length );

/* process seq header start and seq end */
int cavs_decoder_process( void *p_decoder, unsigned char *p_in, int i_in_len );

/* alloc buffer for decoded yuv */
int cavs_decoder_buffer_init( cavs_param *param );

/* free buffer for decoded yuv */
int cavs_decoder_buffer_end( cavs_param *param );

/* destroy decoder handle */
void cavs_decoder_destroy( void *p_decoder );

/* destroy decoder handle when across different seq header */
void cavs_decoder_slice_destroy( void *p_decoder );

/* get info from seq header */
int cavs_decoder_get_seq( void *p_decoder, cavs_seq_info *p_si );

/* output delay frame when has B-frames in stream */
int cavs_out_delay_frame( void *p_decoder, unsigned char *p_out_yuv[3] );
int cavs_out_delay_frame_end(void *p_decoder, unsigned char *p_out_yuv[3]);

/* init first seq header for frame decoding */
int cavs_decoder_seq_init( void *p_decoder , cavs_param *param );

int cavs_decoder_seq_end( void *p_decoder );

/* decode one frame or field when across pic header */
/*
 void *p_decoder : decoder handle
 int i_startcode : type of current picture header 
 cavs_param *param : decoder configure param
 uint8_t* buf : buffer with current picture header
 int length : length of current NAL
*/
int cavs_decode_one_frame( void *p_decoder , int i_startcode, cavs_param *param, unsigned char* buf, int length );

/*return current frame type is P or B */
int cavs_decoder_cur_frame_type( void* p_decoder );

int cavs_decoder_thread_param_init( void* p_decoder );

/*use for format probe */
int cavs_decoder_probe_seq(void *p_decoder, unsigned char *p_in, int i_in_length);

int cavs_decoder_pic_header( void* p_decoder, unsigned char *p_buf,  int i_len, cavs_param* param, unsigned int cur_startcode );

int cavs_decoder_set_format_type( void* p_decoder, cavs_param *param );

int cavs_decoder_seq_header_reset( void* p_decoder );

int cavs_decoder_low_delay_value( void* p_decoder );

int cavs_decode_one_frame_delay( void *p_decoder, cavs_param *param );

int cavs_set_last_delay_frame( void *p_decoder );

int cavs_decoder_seq_header_reset_pipeline( void* p_decoder );

int cavs_decoder_slice_num_probe( void* p_decoder,  int i_startcode, cavs_param *param, unsigned char  *buf,  int length );

#ifdef __cplusplus
}
#endif

#endif
