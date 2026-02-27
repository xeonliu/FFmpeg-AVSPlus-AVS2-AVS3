/*****************************************************************************
* libcavs.c: the source file of avs/avs+ decoder 
* Copyright (c) 2021 maliwen2015/ffmpeg_cavs_dra
*****************************************************************************
*/

#include "config.h"
#ifdef _WIN32
#define SYS_WINDOWS 1
#define SYS_LINUX 0
#elif defined(__linux__)
#define SYS_WINDOWS 0
#define SYS_LINUX 1
#else
#define SYS_WINDOWS 0
#define SYS_LINUX 0
#endif

#include "libcavs.h"

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#elif defined(__linux__)
#define __USE_GNU
#include <sched.h>
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <limits.h>
#include <locale.h>
#include <pthread.h>

int cavs_cpu_num_processors (void);

#define DECLARE_ALIGNED( var, n ) var __attribute__((aligned(n)))

#define DECLARE_ALIGNED_16( var ) DECLARE_ALIGNED( var, 16 )
#define DECLARE_ALIGNED_8( var )  DECLARE_ALIGNED( var, 8 )
#define DECLARE_ALIGNED_4( var )  DECLARE_ALIGNED( var, 4 )

#define ALIGNED_16( var ) DECLARE_ALIGNED( var, 16 )
#define ALIGNED_ARRAY_16( type, name, sub1, ... )\
	ALIGNED_16( type name sub1 __VA_ARGS__ )

#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 0)
#define UNUSED __attribute__((unused))
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#define NOINLINE __attribute__((noinline))
#define cavs_constant_p(x) __builtin_constant_p(x)
#else
#define UNUSED
#define ALWAYS_INLINE inline
#define NOINLINE
#define cavs_constant_p(x) 0
#endif

#define cavs_pthread_t               pthread_t
#define cavs_pthread_create          pthread_create
#define cavs_pthread_join            pthread_join
#define cavs_pthread_mutex_t         pthread_mutex_t
#define cavs_pthread_mutex_init      pthread_mutex_init
#define cavs_pthread_mutex_destroy   pthread_mutex_destroy
#define cavs_pthread_mutex_lock      pthread_mutex_lock
#define cavs_pthread_mutex_unlock    pthread_mutex_unlock
#define cavs_pthread_cond_t          pthread_cond_t
#define cavs_pthread_cond_init       pthread_cond_init
#define cavs_pthread_cond_destroy    pthread_cond_destroy
#define cavs_pthread_cond_broadcast  pthread_cond_broadcast
#define cavs_pthread_cond_wait       pthread_cond_wait
#define cavs_pthread_attr_t          pthread_attr_t
#define cavs_pthread_attr_init       pthread_attr_init
#define cavs_pthread_attr_destroy    pthread_attr_destroy
#define WORD_SIZE sizeof(void*)

#if HAVE_BIGENDIAN
#define endian_fix(x) (x)
#define endian_fix64(x) (x)
#define endian_fix32(x) (x)
#define endian_fix16(x) (x)
#else
static ALWAYS_INLINE uint32_t endian_fix32( uint32_t x )
{
	return (x<<24) + ((x<<8)&0xff0000) + ((x>>8)&0xff00) + (x>>24);
}
static ALWAYS_INLINE uint64_t endian_fix64( uint64_t x )
{
	return endian_fix32((uint32_t)(x>>32)) + ((uint64_t)endian_fix32((uint32_t)x)<<32);
}
static ALWAYS_INLINE intptr_t endian_fix( intptr_t x )
{
	return WORD_SIZE == 8 ? endian_fix64(x) : endian_fix32((uint32_t)x);
}
static ALWAYS_INLINE uint16_t endian_fix16( uint16_t x )
{
	return (x<<8)|(x>>8);
}
#endif

#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 3)
#define cavs_clz(x) __builtin_clz(x)
#else
static int ALWAYS_INLINE
	cavs_clz (uint32_t x)
{
	static uint8_t lut[16] = { 4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
	int y, z = ((x - 0x10000) >> 27) & 16;
	x >>= z ^ 16;
	z += y = ((x - 0x100) >> 28) & 8;
	x >>= y ^ 8;
	z += y = ((x - 0x10) >> 29) & 4;
	x >>= y ^ 4;
	return z + lut[x];
}
#endif

#if SYS_WINDOWS
#define cavs_lower_thread_priority(p)\
{\
	cavs_pthread_t handle = pthread_self();\
struct sched_param sp;\
	int policy = SCHED_OTHER;\
	pthread_getschedparam( handle, &policy, &sp );\
	sp.sched_priority -= p;\
	pthread_setschedparam( handle, policy, &sp );\
}
#else
#define cavs_lower_thread_priority(p) { UNUSED int nice_ret = nice(p); }
#endif /* SYS_WINDOWS */


typedef struct cavs_threadpool_t cavs_threadpool_t;

int   cavs_threadpool_init( cavs_threadpool_t **p_pool, int threads,
                            void (*init_func)(void *), void *init_arg );
void  cavs_threadpool_run( cavs_threadpool_t *pool, void *(*func)(void *), void *arg );
void *cavs_threadpool_wait( cavs_threadpool_t *pool, void *arg );
void  cavs_threadpool_delete( cavs_threadpool_t *pool );

/////////////////////////////////////////////////////////
#define SVA_STREAM_BUF_SIZE 1024

#define I_PICTURE_START_CODE    0xB3
#define PB_PICTURE_START_CODE   0xB6
#define SLICE_START_CODE_MIN    0x00
#define SLICE_START_CODE_MAX    0xAF
#define USER_DATA_START_CODE    0xB2
#define SEQUENCE_HEADER_CODE    0xB0
#define EXTENSION_START_CODE    0xB5
#define SEQUENCE_END_CODE       0xB1
#define VIDEO_EDIT_CODE         0xB7

#define MIN2(a,b) ( (a)<(b) ? (a) : (b) )

typedef struct {
    unsigned char *f; /* save stream address */
    unsigned char *f_end;
    unsigned char buf[SVA_STREAM_BUF_SIZE];  //流缓冲区,size must be large than 3 bytes
    unsigned int uClearBits;                //不含填充位的位缓冲，32位，初始值是0xFFFFFFFF
    unsigned int uPre3Bytes;                //含填充位的位缓冲，32位，初始值是0x00000000
    int iBytePosition;		                   //当前字节位置
    int iBufBytesNum;		                   //最近一次读入缓冲区的字节数
    int iClearBitsNum;		                   //不含填充位的位的个数
    int iStuffBitsNum;		                   //已剔除的填充位的个数，遇到开始码时置0
    int iBitsCount;			                   //码流总位数

    int demulate_enable;
} InputStream;

int cavs_init_bitstream( InputStream *p , unsigned char *rawstream, unsigned int i_len );
unsigned int cavs_get_one_nal ( InputStream* p, unsigned char *buf, int *length );

//////////////////////////////////////////////////////////

#define MAX_CODED_FRAME_SIZE 4000000         //!< bytes for one frame

typedef signed char int8_t;
typedef unsigned char  uint8_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;
typedef int  int32_t;
typedef unsigned  int uint32_t;
typedef short DCTELEM;
#define unaligned16(a) (*(const uint16_t*)(a))
#define unaligned32(a) (*(const uint32_t*)(a))
#define unaligned64(a) (*(const uint64_t*)(a))

enum profile_e
{
  PROFILE_JIZHUN	  = 0x20,
  PROFILE_SHENZHAN	  = 0x24,
  PROFILE_YIDONG	  = 0x34,
  PROFILE_GUANGDIAN   = 0x48,
  PROFILE_JIAQIANG	  = 0x88,
};

//Tech. of GuangDian Profile
#define	PSKIP_REF	 p->ph.b_pb_field_enhanced
#define NEW_DIRECT	 p->ph.b_pb_field_enhanced

#define CAVS_EDGE    16
#define CAVS_MB_SIZE  16
#define CAVS_MB4x4_SIZE 8

#define CAVS_I_PICTURE 0
#define CAVS_P_PICTURE 1
#define CAVS_B_PICTURE 2
#define BACKGROUND_IMG 3
#define BP_IMG 4

#define MAX_NEG_CROP 1024//384

#define CAVS_MIN(a,b) ( (a)<(b) ? (a) : (b) )
#define CAVS_MAX(a,b) ( (a)>(b) ? (a) : (b) )
#define CAVS_MIN3(a,b,c) CAVS_MIN((a),CAVS_MIN((b),(c)))
#define CAVS_MAX3(a,b,c) CAVS_MAX((a),CAVS_MAX((b),(c)))
#define CAVS_MIN4(a,b,c,d) CAVS_MIN((a),CAVS_MIN3((b),(c),(d)))
#define CAVS_MAX4(a,b,c,d) CAVS_MAX((a),CAVS_MAX3((b),(c),(d)))
#define CAVS_XCHG(type,a,b) { type t = a; a = b; b = t; }
#define CAVS_FIX8(f) ((int)(f*(1<<8)+.5))
#define CAVS_SAFE_FREE(p) if(p){cavs_free(p);p=0;}
#define LEFT_ADAPT_INDEX_L 0
#define TOP_ADAPT_INDEX_L  1
#define LEFT_ADAPT_INDEX_C 2
#define TOP_ADAPT_INDEX_C  3
#define CAVS_THREAD_MAX 128
#define CAVS_BFRAME_MAX 16

enum cavs_intra_lum_4x4_pred_mode
{
	I_PRED_4x4_V  = 8,
	I_PRED_4x4_H  = 9,
	I_PRED_4x4_DC = 10,
	I_PRED_4x4_DDL= 11,
	I_PRED_4x4_DDR= 12,
	I_PRED_4x4_HD = 13,
	I_PRED_4x4_VL = 14,
	I_PRED_4x4_HU = 15,
	I_PRED_4x4_VR = 16,

	I_PRED_4x4_DC_LEFT = 17,
	I_PRED_4x4_DC_TOP  = 18,
	I_PRED_4x4_DC_128  = 19,
};
enum cavs_intra_lum_pred_mode {
     INTRA_L_VERT=0,
     INTRA_L_HORIZ,
     INTRA_L_LP,
     INTRA_L_DOWN_LEFT,
     INTRA_L_DOWN_RIGHT,
     INTRA_L_LP_LEFT,
     NTRA_L_LP_TOP,
     INTRA_L_DC_128
};
enum cavs_intra_chrom_pred_mode {
     INTRA_C_DC=0,
     INTRA_C_HORIZ,
     INTRA_C_VERT,
     INTRA_C_PLANE,
     INTRA_C_DC_LEFT,
     INTRA_C_DC_TOP,
     INTRA_C_DC_128,
};

static const int cavs_mb_pred_mode8x8c[7] = {
	INTRA_C_DC, INTRA_C_HORIZ, INTRA_C_VERT, INTRA_C_PLANE,
	INTRA_C_DC, INTRA_C_DC, INTRA_C_DC
};

#define NO_INTRA_PMODE 8
# define INTRA_PMODE_4x4 8
static const int8_t pred4x4[10][10] = {
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{2, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // mdou new 3
	{2, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{2, 0, 1, 2, 3, 4, 5, 6, 7, 8},
	{2, 0, 0, 0, 3, 3, 3, 3, 3, 3},
	{2, 0, 1, 4, 4, 4, 4, 4, 4, 4},
	{2, 0, 1, 5, 5, 5, 5, 5, 5, 5},
	{2, 0, 0, 0, 0, 0, 0, 6, 0, 0},
	{2, 0, 1, 7, 7, 7, 7, 7, 7, 7},
	{2, 0, 0, 0, 0, 4, 5, 6, 7, 8}//*/
};
static const char pred_4x4to8x8[9] = 
{
	0,1,2,3,4,1,0,1,0
};

#define NO_INTRA_8x8_MODE 8
#define IS_INTRA_4x4_PRED_MODE(mode) ((mode)>=NO_INTRA_8x8_MODE)
#define cavs_mb_pred_mode(t) cavs_mb_pred_mode[(t)+1]
static const int cavs_mb_pred_mode[21] = {
	-1, INTRA_L_VERT, INTRA_L_HORIZ, INTRA_L_LP,
	INTRA_L_DOWN_LEFT, INTRA_L_DOWN_RIGHT, INTRA_L_LP,
	INTRA_L_LP, INTRA_L_LP,
	I_PRED_4x4_V-NO_INTRA_8x8_MODE,   I_PRED_4x4_H-NO_INTRA_8x8_MODE,   I_PRED_4x4_DC-NO_INTRA_8x8_MODE,
	I_PRED_4x4_DDL-NO_INTRA_8x8_MODE, I_PRED_4x4_DDR-NO_INTRA_8x8_MODE, I_PRED_4x4_HD-NO_INTRA_8x8_MODE,
	I_PRED_4x4_VL-NO_INTRA_8x8_MODE,  I_PRED_4x4_HU-NO_INTRA_8x8_MODE,  I_PRED_4x4_VR-NO_INTRA_8x8_MODE,
	I_PRED_4x4_DC-NO_INTRA_8x8_MODE,  I_PRED_4x4_DC-NO_INTRA_8x8_MODE,  I_PRED_4x4_DC-NO_INTRA_8x8_MODE
};

static const int8_t cavs_pred_mode_8x8to4x4[5] =
{
	0, 1, 2, 3, 4
};
static const int8_t cavs_pred_mode_4x4to8x8[9] =
{
	0, 1, 2, 3, 4, 1, 0, 1, 0
};



#define ESCAPE_CODE_PART1             39
#define ESCAPE_CODE_PART2             71
#define ESCAPE_CODE                   59
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

enum cavs_mb_type 
{    
     I_8X8=0 ,
     P_SKIP,
     P_16X16,
     P_16X8,
     P_8X16,
     P_8X8,
     B_SKIP,
     B_DIRECT,
     B_FWD_16X16,
     B_BWD_16X16,
     B_SYM_16X16,
     B_8X8 = 29
};

enum cavs_mb_sub_type {
     B_SUB_DIRECT,
     B_SUB_FWD,
     B_SUB_BWD,
     B_SUB_SYM
};
enum cavs_block_size{
   BLK_16X16,
   BLK_16X8,
   BLK_8X16,
	BLK_8X8,
   BLK_4X4
};
enum cavs_mv_intra_mode_location {
INTRA_MODE_D3 =0,
INTRA_MODE_B2,
INTRA_MODE_B3,
INTRA_MODE_A1,
INTRA_MODE_X0,
INTRA_MODE_X1,
INTRA_MODE_A3,
INTRA_MODE_X2,
INTRA_MODE_X3
};

enum cavs_mv_pred {
MV_PRED_MEDIAN,
MV_PRED_LEFT,
MV_PRED_TOP,
MV_PRED_TOPRIGHT,
MV_PRED_PSKIP,
MV_PRED_BSKIP
};

enum cavs_mv_block_location {
MV_FWD_D3 = 0,
MV_FWD_B2,
MV_FWD_B3,
MV_FWD_C2,
MV_FWD_A1,
MV_FWD_X0,
MV_FWD_X1,
MV_FWD_A3 = 8,
MV_FWD_X2,
MV_FWD_X3,
MV_BWD_D3 = MV_BWD_OFFS,
MV_BWD_B2,
MV_BWD_B3,
MV_BWD_C2,
MV_BWD_A1,
MV_BWD_X0,
MV_BWD_X1,
MV_BWD_A3 = MV_BWD_OFFS+8,
MV_BWD_X2,
MV_BWD_X3
};

#define A_AVAIL                          1
#define B_AVAIL                          2
#define C_AVAIL                          4
#define D_AVAIL                          8
#define NOT_AVAIL                       -1
#define REF_INTRA                       -2
#define REF_DIR                         -3

#define CAVS_ICACHE_STRIDE	5
#define CAVS_ICACHE_START	6


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

#define pixel uint8_t
/* Unions for type-punning.
 * Mn: load or store n bits, aligned, native-endian
 * CPn: copy n bits, aligned, native-endian
 * we don't use memcpy for CPn because memcpy's args aren't assumed to be aligned */
typedef union { uint16_t i; uint8_t  c[2]; }  cavs_union16_t;
typedef union { uint32_t i; uint16_t b[2]; uint8_t  c[4]; }  cavs_union32_t;
typedef union { uint64_t i; uint32_t a[2]; uint16_t b[4]; uint8_t c[8]; }  cavs_union64_t;
typedef struct { uint64_t i[2]; } cavs_uint128_t;
typedef union { cavs_uint128_t i; uint64_t a[2]; uint32_t b[4]; uint16_t c[8]; uint8_t d[16]; }  cavs_union128_t;
#define M16(src) (((cavs_union16_t*)(src))->i)
#define M32(src) (((cavs_union32_t*)(src))->i)
#define M64(src) (((cavs_union64_t*)(src))->i)
#define M128(src) (((cavs_union128_t*)(src))->i)
#define M128_ZERO ((cavs_uint128_t){{0,0}})
#define CP16(dst,src) M16(dst) = M16(src)
#define CP32(dst,src) M32(dst) = M32(src)
#define CP64(dst,src) M64(dst) = M64(src)
#define CP128(dst,src) M128(dst) = M128(src)

#define MPIXEL_X4(src) M32(src)
#define PIXEL_SPLAT_X4(x) ((x)*0x01010101U)
#define CPPIXEL_X4(dst,src) MPIXEL_X4(dst) = MPIXEL_X4(src)

#define PACK8TO16(a, b)	(a + ((b)<<8))

//#define DECLARE_ALIGNED( var, n ) __declspec(align(n)) var
#define DECLARE_ALIGNED_16( var ) DECLARE_ALIGNED( var, 16 )
#define DECLARE_ALIGNED_8( var )  DECLARE_ALIGNED( var, 8 )
#define DECLARE_ALIGNED_4( var )  DECLARE_ALIGNED( var, 4 )

/*
	 0  1  4  5
	 2  3  6  7
	 8  9 12 13
	10 11 14 15
*/
static const int offset_block4x4[16][2] = {
	{0,0}, {1,0}, {0,1}, {1,1},
	{2,0}, {3,0}, {2,1}, {3,1},
	{0,2}, {1,2}, {0,3}, {1,3},
	{2,2}, {3,2}, {2,3}, {3,3}
};

typedef struct tagcavs_video_sequence_header
{
	/************************************************************************/
	/* jizhun_profile and yidong_profile       0x34                                                         */
	/************************************************************************/
	uint8_t  i_profile_id;/*8bits u(8)*/
	uint8_t  i_level_id;/*8bits u(8)*/
	uint8_t  b_progressive_sequence;/*1bit u(1)*/
	uint32_t i_horizontal_size;/*14bits u(14)*/
	uint32_t i_vertical_size;/*14bits u(14)*/
	uint8_t  i_chroma_format;/*2bits u(2)*/
	uint8_t  i_sample_precision;/*3bits u(3)*/
	uint8_t  i_aspect_ratio;/*4bits u(4)*/
	uint8_t  i_frame_rate_code;/*4bits u(4)*/
	uint32_t i_bit_rate;
	/*30bits u(30) 分两部分获得
	 18bits bit_rate_lower;
	 12bits bit_rate_upper。
	其间有1bit的marker_bit防止起始码的竞争*/
	uint8_t  b_low_delay;/*1bit u(1)*/
	//这里有1bit的marker_bit
	uint32_t i_bbv_buffer_size;/*18bits u(18)*/
	//还有3bits的reserved_bits，固定为'000' 
		/************************************************************************/
	/* yidong_profile       0x34                                                                                       */
	/************************************************************************/
	
	uint32_t  weighting_quant_flag;
	uint32_t  mb_adapt_wq_disable;
	uint32_t  chroma_quant_param_disable;
	uint8_t   chroma_quant_param_delta_u;	
	uint8_t   chroma_quant_param_delta_v;
	uint8_t  weighting_quant_param_index;
	uint8_t   weighting_quant_param_delta1[6];
	uint8_t   weighting_quant_param_delta2[6];
	uint32_t   scene_adapt_disable;
	uint32_t   scene_model_update_flag;
	uint32_t  weighting_quant_model;
	uint32_t  vbs_enable;
	uint32_t  vbs_qp_shift;
	uint32_t asi_enable;
	uint32_t limitDC_refresh_frame;

	uint8_t  b_interlaced;

	uint8_t b_flag_aec; /* used for check stream change */
	uint32_t b_aec_enable; /* backup aec enable */
}cavs_video_sequence_header;

typedef struct tagcavs_picture_header
{
    uint16_t i_bbv_delay;/*16bits u(16)*/

    uint8_t  i_picture_coding_type;/*2bits u(2) pb_piture_header专有*/
    uint8_t  b_time_code_flag;/*1bit u(1) i_piture_header专有*/
    uint32_t i_time_code;/*24bits u(24) i_piture_header专有*/
    uint8_t  i_picture_distance;/*8bits u(8)*/
    uint32_t i_bbv_check_times;/*ue(v)*/
    uint8_t  b_progressive_frame;/*1bit u(1)*/
    uint8_t  b_picture_structure;/*1bit u(1)*/
    uint8_t  b_advanced_pred_mode_disable;/*1bit u(1) pb_piture_header专有*/
    uint8_t  b_top_field_first;/*1bit u(1)*/
    uint8_t  b_repeat_first_field;/*1bit u(1)*/
    uint8_t  b_fixed_picture_qp;/*1bit u(1)*/
    uint8_t  i_picture_qp;/*6bits u(6)*/
    uint8_t  b_picture_reference_flag;/*1bit u(1) pb_piture_header专有*/
    /*4bits保留字节reserved_bits固定为'0000'*/
    uint8_t  b_no_forward_reference_flag;
    uint8_t  b_skip_mode_flag;/*1bit u(1)*/
    uint8_t  b_loop_filter_disable;/*1bit u(1)*/
    uint8_t  b_loop_filter_parameter_flag;/*1bit u(1)*/
    uint32_t  i_alpha_c_offset;/*se(v)*/
    uint32_t  i_beta_offset;/*se(v)*/

    /************************************************************************/
    /* yidong_profile       0x34                                                                                   */
    /************************************************************************/

    uint32_t  weighting_quant_flag;
    uint32_t  mb_adapt_wq_disable;
    uint32_t  chroma_quant_param_disable;
    uint8_t   chroma_quant_param_delta_u;	
    uint8_t   chroma_quant_param_delta_v;
    uint8_t   weighting_quant_param_index;
    uint8_t   weighting_quant_param_delta1[6];
    uint8_t   weighting_quant_param_delta2[6];
	
	uint8_t   weighting_quant_param_undetail[6];
	uint8_t   weighting_quant_param_detail[6];

    uint32_t  scene_adapt_disable;
    uint32_t  scene_model_update_flag;
    uint32_t  weighting_quant_model;
    uint32_t  vbs_enable;
    uint32_t  vbs_qp_shift;
    uint32_t  asi_enable;
    uint32_t  limitDC_refresh_frame;

    /************************************************************************/
    /* guangdian profile       0x48       add by jcma                       */
    /************************************************************************/
    uint32_t  b_aec_enable;
    uint32_t  b_pb_field_enhanced;
}cavs_picture_header;

typedef struct tagcavs_slice_header
{
	uint8_t  i_slice_vertical_position_extension;/*3bits u(3)*/
	uint8_t  i_slice_horizontal_position_extension;/*2bits u(2)*/
	uint8_t i_slice_horizontal_position;/*8bits u(8)*/
	uint8_t  b_fixed_slice_qp;/*1bit u(1)*/
	uint8_t  i_slice_qp;/*6bits u(6)*/
	uint8_t  b_slice_weighting_flag;/*1bit u(1)*/
	uint32_t i_luma_scale[4];/*8bits u(8)*/
	int32_t  i_luma_shift[4];/*8bits i(8)*/
	uint32_t i_chroma_scale[4];/*8bits u(8)*/
	int32_t  i_chroma_shift[4];/*8bits i(8)*/
	uint8_t  b_mb_weighting_flag;/*1bit u(1)*/
	uint32_t i_mb_skip_run;/*ue(v)*/
	uint32_t i_profile_id;
	uint8_t  i_number_of_reference;
	uint32_t quant_coff_pred_flag;
}cavs_slice_header;

typedef struct tagcavs_sequence_display_extension
{
	uint8_t  i_video_format;/*3bits u(3)*/
	uint8_t  b_sample_range;/*1bit u(1)*/
	uint8_t  b_colour_description;/*1bit u(1)*/
	uint8_t  i_colour_primaries;/*8bits u(8)*/
	uint8_t  i_transfer_characteristics;/*8bits u(8)*/
	uint8_t  i_matrix_coefficients;/*8bits u(8)*/
	uint32_t i_display_horizontal_size;/*14bits u(14)*/
	uint32_t i_display_vertical_size;/*14bits u(14)*/
	//还有2bits的reserved_bits，
	uint8_t  i_stereo_packing_mode;
}cavs_sequence_display_extension;

typedef struct tagcavs_copyright_extension
{
	uint8_t  b_copyright_flag;/*1bit u(1)*/
	uint8_t  i_copyright_id;/*8bits u(8)*/
	uint8_t  b_original_or_copy;/*1bit u(1)*/
	//这里有保留7bits的reserved_bits
	uint64_t i_copyright_number;
	/*64bits u(64)分三个部分出现，
	20bits copyright_number_1;
	22bits copyright_number_2;
	22bits copyright_number_3
	其间分别嵌入2个1bit的marker_bit*/
}cavs_copyright_extension;

typedef struct tagcavs_camera_parameters_extension
{
	//1bit的reserved_bits
	uint8_t  i_camera_id;/*7bits u(7)*/
	//这里有1bit的marker_bit
	uint32_t i_height_of_image_device;/*22bits u(22)*/
	//这里有1bit的marker_bit
	uint32_t i_focal_length;/*22bits u(22)*/
	//这里有1bit的marker_bit
	uint32_t i_f_number;/*22bits u(22)*/
	//这里有1bit的marker_bit
	uint32_t i_vertical_angle_of_view;/*22bits u(22)*/
	//这里有1bit的marker_bit
	int32_t  i_camera_position_x;
	/*32bits i(32) 分两部分获得
	16bits camera_position_x_upper;
	16bits camera_position_x_lower;
    其间有1bit的marker_bit防止起始码的竞争*/
	int32_t  i_camera_position_y;
	/*32bits i(32) 分两部分获得
	16bits camera_position_x_upper;
	16bits camera_position_x_lower;
    其间有1bit的marker_bit防止起始码的竞争*/
	int32_t  i_camera_position_z;
	/*32bits i(32) 分两部分获得
	16bits camera_position_x_upper;
	16bits camera_position_x_lower;
    其间有1bit的marker_bit防止起始码的竞争*/
	//这里有1bit的marker_bit
	int32_t i_camera_direction_x;/*22bits i(22)*/
	//这里有1bit的marker_bit
	int32_t i_camera_direction_y;/*22bits i(22)*/
	//这里有1bit的marker_bit
	int32_t i_camera_direction_z;/*22bits i(22)*/

	//这里有1bit的marker_bit
	int32_t i_image_plane_vertical_x;/*22bits i(22)*/
	//这里有1bit的marker_bit
	int32_t i_image_plane_vertical_y;/*22bits i(22)*/
	//这里有1bit的marker_bit
	int32_t i_image_plane_vertical_z;/*22bits i(22)*/
	//这里有1bit的marker_bit
	//还有32bits的reserved_bits，
}cavs_camera_parameters_extension;

typedef struct tagcavs_picture_display_extension
{
	uint32_t i_number_of_frame_centre_offsets;
	/*元素个数，由语法派生获得cavs_sequence_display_extension，不占用bits*/
	uint32_t i_frame_centre_horizontal_offset[4];/*16bits i(16)*/
	uint32_t i_frame_centre_vertical_offset[4];/*16bits i(16)*/
}cavs_picture_display_extension;
 typedef struct tagcavs_vector
{
     int16_t x;
     int16_t y;
     int16_t dist;
     int16_t ref;
} cavs_vector;

typedef struct tagcavs_vlc 
{
	int8_t rltab[59][3];
	int8_t level_add[27];
	int8_t golomb_order;
	int inc_limit;
	int8_t max_run;

} cavs_vlc;

typedef struct tagcavs_image
{
	uint32_t   i_colorspace;
	uint32_t   i_width;
	uint32_t   i_height;
	uint8_t   *p_data[4];
	uint32_t   i_stride[4];
	uint32_t   i_distance_index;
	uint32_t   i_code_type;
	uint32_t   b_top_field_first;
	uint32_t   i_ref_distance[2]; /* two ref-frames at most */
	uint32_t   b_picture_structure;

}cavs_image;

typedef struct tagcavs_bitstream
{
    uint8_t *p_start;
    uint8_t *p;
    uint8_t *p_end;
    int      i_left;
    uint32_t i_startcode;
}cavs_bitstream;

static inline void cavs_bitstream_init( cavs_bitstream *s, void *p_data, int i_data )
{
    s->p_start = (uint8_t *)p_data;
    s->p       = (uint8_t *)p_data;
    s->p_end   = s->p + i_data;
    s->i_left  = 8;
    s->i_startcode=*s->p;
}

static inline int cavs_bitstream_pos( cavs_bitstream *s )
{
    return (int)( 8 * ( s->p - s->p_start ) + 8 - s->i_left );
}

static inline int cavs_bitstream_eof( cavs_bitstream *s )
{
    return( s->p >= s->p_end -1? 1: 0 ) && (*(s->p-1) & ((1<<s->i_left)-1)) == (1<<(s->i_left-1));
}

static inline void cavs_bitstream_align(cavs_bitstream *s)
{
    if (s->i_left < 8)
    {
        s->p++;
        s->i_left = 8;
    }
}

static inline void cavs_bitstream_next_byte( cavs_bitstream *s)
{
    s->p++;
	s->i_startcode = (s->i_startcode << 8) + *s->p;
	s->i_left = 8;
}

static inline uint32_t cavs_bitstream_get_bits( cavs_bitstream *s, int i_count )
{
    static uint32_t i_mask[33] ={0x00,
    	0x01,      0x03,      0x07,      0x0f,
    	0x1f,      0x3f,      0x7f,      0xff,
    	0x1ff,     0x3ff,     0x7ff,     0xfff,
    	0x1fff,    0x3fff,    0x7fff,    0xffff,
    	0x1ffff,   0x3ffff,   0x7ffff,   0xfffff,
    	0x1fffff,  0x3fffff,  0x7fffff,  0xffffff,
    	0x1ffffff, 0x3ffffff, 0x7ffffff, 0xfffffff,
    	0x1fffffff,0x3fffffff,0x7fffffff,0xffffffff};
    int      i_shr;
    uint32_t i_result = 0;

    while( i_count > 0 )
    {
        if( s->p >= s->p_end )
        {
            break;
        }
        if( ( i_shr = s->i_left - i_count ) >= 0 )
        {
            i_result |= ( *s->p >> i_shr )&i_mask[i_count];
            s->i_left -= i_count;
            if( s->i_left == 0 )
            {
                cavs_bitstream_next_byte(s);
            }
            
            return( i_result );
        }
        else
        {
            i_result |= (*s->p&i_mask[s->i_left]) << -i_shr;
            i_count  -= s->i_left;
            cavs_bitstream_next_byte(s);
       }
    }

    return( i_result );
}

static inline uint32_t cavs_bitstream_get_bit1( cavs_bitstream *s )
{
    if( s->p < s->p_end )
    {
        uint32_t i_result;

        s->i_left--;
        i_result = ( *s->p >> s->i_left )&0x01;
        if( s->i_left == 0 )
        {
            cavs_bitstream_next_byte(s);  
        }
        
        return i_result;
    }

    return 0;
}



static inline int32_t cavs_bitstream_get_int( cavs_bitstream *s, int i_count )
{
    uint32_t i_temp=1<<(i_count-1);
    uint32_t i_result=cavs_bitstream_get_bits(s,i_count);
    
    if(i_result&i_temp)
    {
        return 0-(int)((~i_result+1)<<( 32 - i_count)>>(32 - i_count));
    }
    else
    {
        return i_result;
    }
}

static inline void cavs_bitstream_clear_bits( cavs_bitstream *s, int i_count )
{
    while( i_count > 0 )
    {
        if( s->p >= s->p_end )
        {
            break;
        }
        if( (  s->i_left - i_count ) >= 0 )
        {
            s->i_left -= i_count;
            if( s->i_left == 0 )
            {
                cavs_bitstream_next_byte(s);
            }
            break;
        }
        else
        {
            i_count  -= s->i_left;
            cavs_bitstream_next_byte(s);
       }
    }
}

static inline int cavs_bitstream_get_ue( cavs_bitstream *s )
{
    int i = 0;
    int temp;

    while( cavs_bitstream_get_bit1(s) == 0 && s->p < s->p_end && i < 32)
    {
        i++;
    }
    temp=( ( 1 << i) - 1 + cavs_bitstream_get_bits( s, i) );

    return( temp );
}

static inline int cavs_bitstream_get_se( cavs_bitstream *s)
{
    int val = cavs_bitstream_get_ue( s);
    int temp=val&0x01 ? (val+1)/2 : -(val/2);

    return temp;
}

static inline int cavs_bitstream_get_ue_k( cavs_bitstream *s, int k)
{
    int i = 0;
    int temp;

    while( cavs_bitstream_get_bit1( s ) == 0 && s->p < s->p_end && i < 32)
    {
        i++;
    }
    temp = ( 1 << (i+k)) - (1<<k) + cavs_bitstream_get_bits( s, i+k);
    
    return( temp );
}

static inline int cavs_bitstream_get_se_k( cavs_bitstream *s, int k)
{
    int val = cavs_bitstream_get_ue_k( s,k);

    return val&0x01 ? (val+1)/2 : -(val/2);
}

void *cavs_malloc( int );
void *cavs_realloc( void *p, int i_size );
void  cavs_free( void * );
int cavs_get_video_sequence_header(cavs_bitstream *s,cavs_video_sequence_header *h);
int cavs_get_sequence_display_extension(cavs_bitstream *s, cavs_sequence_display_extension *sde);
int cavs_get_copyright_extension(cavs_bitstream *s, cavs_copyright_extension *ce);
int cavs_get_camera_parameters_extension(cavs_bitstream *s, cavs_camera_parameters_extension *cpe);
int cavs_get_user_data(cavs_bitstream *s, uint8_t *user_data);
int cavs_get_i_picture_header(cavs_bitstream *s,cavs_picture_header *h,cavs_video_sequence_header *sh);
int cavs_get_pb_picture_header(cavs_bitstream *s,cavs_picture_header *h,cavs_video_sequence_header *sh);

int cavs_cavlc_get_ue(cavs_decoder *p);
int cavs_cavlc_get_se(cavs_decoder *p);
int cavs_cavlc_get_mb_type_p(cavs_decoder *p);
int cavs_cavlc_get_mb_type_b(cavs_decoder *p);
int cavs_cavlc_get_intra_luma_pred_mode(cavs_decoder *p);
int cavs_cavlc_get_intra_cbp(cavs_decoder *p);
int cavs_cavlc_get_inter_cbp(cavs_decoder *p);
int cavs_cavlc_get_ref_p(cavs_decoder *p, int ref_scan_idx);
int cavs_cavlc_get_ref_b(cavs_decoder *p, int i_list, int ref_scan_idx);
int cavs_cavlc_get_mb_part_type(cavs_decoder *p);
int cavs_cavlc_get_mvd(cavs_decoder *p, int i_list, int i_down, int xy_idx);
int cavs_cavlc_get_coeffs(cavs_decoder *p, const cavs_vlc *p_vlc_table, int i_escape_order, int b_chroma);

/////////////////////////////////////////////////////////////////////////////////////////
#define NUM_BLOCK_TYPES 8
#define NUM_MB_TYPE_CTX  11
#define NUM_B8_TYPE_CTX  9
#define NUM_MV_RES_CTX   10
#define NUM_REF_NO_CTX   6
#define NUM_DELTA_QP_CTX 4
#define NUM_MB_AFF_CTX 4
#define NUM_IPR_CTX    2
#define NUM_CIPR_CTX   4
#define NUM_CBP_CTX    4
#define NUM_BCBP_CTX   4
#define NUM_MAP_CTX   20
#define NUM_LAST_CTX  20
#define NUM_ONE_CTX    5
#define NUM_ABS_CTX    5

#define LG_PMPS_SHIFTNO 2
#define B_BITS 10
#define HALF      512 //(1 << (B_BITS-1))
#define QUARTER   256 //(1 << (B_BITS-2))

#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 0)
#define UNUSED __attribute__((unused))
#else
#define UNUSED
#endif

typedef struct
{
	uint16_t	lg_pmps;	/* 10 bits */
    uint8_t		mps;		/* 1 bit */
    uint8_t		cycno;		/* 2 bits */	
} bi_ctx_t;

typedef struct
{
    /* state */
	uint16_t s1, t1; /*unsigned int*/
	uint16_t value_s, value_t; /*unsigned int*/
    uint8_t buffer;
	char bits_to_go;
    int		dec_bypass;

    cavs_bitstream bs;

    bi_ctx_t  mb_type_contexts [3][NUM_MB_TYPE_CTX];
    bi_ctx_t  b8_type_contexts [2][NUM_B8_TYPE_CTX];
    bi_ctx_t  mv_res_contexts  [2][NUM_MV_RES_CTX];
    bi_ctx_t  ref_no_contexts  [2][NUM_REF_NO_CTX];
    bi_ctx_t  delta_qp_contexts   [NUM_DELTA_QP_CTX];
    bi_ctx_t  mb_aff_contexts     [NUM_MB_AFF_CTX];
    bi_ctx_t  ipr_contexts [NUM_IPR_CTX]; 
    bi_ctx_t  cipr_contexts[NUM_CIPR_CTX]; 
    bi_ctx_t  cbp_contexts [3][NUM_CBP_CTX];
    bi_ctx_t  bcbp_contexts[NUM_BLOCK_TYPES][NUM_BCBP_CTX];
    bi_ctx_t  one_contexts [NUM_BLOCK_TYPES][NUM_ONE_CTX];
    bi_ctx_t  abs_contexts [NUM_BLOCK_TYPES][NUM_ABS_CTX];
    bi_ctx_t  fld_map_contexts [NUM_BLOCK_TYPES][NUM_MAP_CTX];
    bi_ctx_t  fld_last_contexts[NUM_BLOCK_TYPES][NUM_LAST_CTX];
    bi_ctx_t  map_contexts [NUM_BLOCK_TYPES][NUM_MAP_CTX];
    bi_ctx_t  last_contexts[NUM_BLOCK_TYPES][NUM_LAST_CTX];    

    bi_ctx_t mb_weighting_pred;

	char b_cabac_error;
} cavs_cabac_t;

static UNUSED const uint16_t lg_pmps_tab[6][1024] = {
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0}, 
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0},
{197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,
217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,
237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,
257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,274,275,276,
277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,294,295,296,
297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,
317,318,319,320,321,322,323,324,
325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,
345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,
365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,
385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,
405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,
425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,
445,446,447,448,449,450,451,452,
453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,
473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,
493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,
513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,
533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,
553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,
573,574,575,576,577,578,579,580,
581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,
601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,
621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,
641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,
661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,
681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,
701,702,703,704,705,706,707,708,
709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,
729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,
749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,
769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,
789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,806,807,808,
809,810,811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,
829,830,831,832,833,834,835,836,
837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,856,
857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,874,875,876,
877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,894,895,896,
897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,
917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,932,933,934,935,936,
937,938,939,940,941,942,943,944,945,946,947,948,949,950,951,952,953,954,955,956,
957,958,959,960,961,962,963,964,
965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,983,984,
985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,1000,1001,1002,1003,
1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,
1020,1021,1022,1023,1024,1025,1026,1027,1028,1029,1030,1031,1032,1033,1034,1035,
1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047,1048,1049,1050,1051,
1052,1053,1054,1055,1056,1057,1058,1059,1060,1061,1062,1063,1064,1065,1066,1067,
1068,1069,1070,1071,1072,1073,1074,1075,1076,1077,1078,1079,1080,1081,1082,1083,
1084,1085,1086,1087,1088,1089,1090,1091,1092,
1093,1094,1095,1096,1097,1098,1099,1100,1101,1102,1103,1104,1105,1106,1107,1108,
1109,1110,1111,1112,1113,1114,1115,1116,1117,1118,1119,1120,1121,1122,1123,1124,
1125,1126,1127,1128,1129,1130,1131,1132,1133,1134,1135,1136,1137,1138,1139,1140,
1141,1142,1143,1144,1145,1146,1147,1148,1149,1150,1151,1152,1153,1154,1155,1156,
1157,1158,1159,1160,1161,1162,1163,1164,1165,1166,1167,1168,1169,1170,1171,1172,
1173,1174,1175,1176,1177,1178,1179,1180,1181,1182,1183,1184,1185,1186,1187,1188,
1189,1190,1191,1192,1193,1194,1195,1196,1197,1198,1199,1200,1201,1202,1203,1204,
1205,1206,1207,1208,1209,1210,1211,1212,1213,1214,1215,1216,1217,1218,1219,1220}, /* cwr=3  +197 */
{95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,
117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,
137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,
157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,
177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,
197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,
217,218,219,220,221,222,
223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,
243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,
263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,
283,284,285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,
303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,
323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,
343,344,345,346,347,348,349,350,
351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,
371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,
391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,
411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,
431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,
451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,
471,472,473,474,475,476,477,478,
479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,
499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,
519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,
539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,
559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,
579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,
599,600,601,602,603,604,605,606,
607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,
627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,
647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,
667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,
687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,
707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,
727,728,729,730,731,732,733,734,
735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,
755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,
775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,
795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,
815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,
835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,
855,856,857,858,859,860,861,862,
863,864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,881,882,
883,884,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,901,902,
903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,920,921,922,
923,924,925,926,927,928,929,930,931,932,933,934,935,936,937,938,939,940,941,942,
943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,962,
963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,
983,984,985,986,987,988,989,990,
991,992,993,994,995,996,997,998,999,1000,1001,1002,1003,1004,1005,1006,1007,1008
,1009,1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,1020,1021,1022,1023,1024
,1025,1026,1027,1028,1029,1030,1031,1032,1033,1034,1035,1036,1037,1038,1039,1040
,1041,1042,1043,1044,1045,1046,1047,1048,1049,1050,1051,1052,1053,1054,1055,1056
,1057,1058,1059,1060,1061,1062,1063,1064,1065,1066,1067,1068,1069,1070,1071,1072
,1073,1074,1075,1076,1077,1078,1079,1080,1081,1082,1083,1084,1085,1086,1087,1088
,1089,1090,1091,1092,1093,1094,1095,1096,1097,1098,1099,1100,1101,1102,1103,1104
,1105,1106,1107,1108,1109,1110,1111,1112,1113,1114,1115,1116,1117,1118}, /* cwr=4  +95 */
{46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72
,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,
100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,
120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,
140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
160,161,162,163,164,165,166,167,168,169,170,171,172,173,
174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,
194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,
214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,
234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,
254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,
274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,
294,295,296,297,298,299,300,301,
302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,
322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,
342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,
362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,
382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,
402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,
422,423,424,425,426,427,428,429,
430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,
450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,
470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,
490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,
510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,
530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,
550,551,552,553,554,555,556,557,
558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,
578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,
598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,
618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,
638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,
658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,
678,679,680,681,682,683,684,685,
686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,
706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,
726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,
746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,
766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,
786,787,788,789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,
806,807,808,809,810,811,812,813,
814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,
834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,
854,855,856,857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,
874,875,876,877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,
894,895,896,897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,
914,915,916,917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,932,933,
934,935,936,937,938,939,940,941,
942,943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,
962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,
982,983,984,985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,1000,1001,
1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015,1016,1017,
1018,1019,1020,1021,1022,1023,1024,1025,1026,1027,1028,1029,1030,1031,1032,1033,
1034,1035,1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047,1048,1049,
1050,1051,1052,1053,1054,1055,1056,1057,1058,1059,1060,1061,1062,1063,1064,1065,
1066,1067,1068,1069}  /* cwr=5  +46 */
};

/* cycno = (cycno<=1)?3:(cycno==2)?4:5 */
static UNUSED uint8_t cwr_trans[4] ={
    3, /*cycno = 0*/
    3, /*cycno = 1*/
    4, /*cycno = 2*/
    5  /*cycno = 3*/
};

static UNUSED uint8_t cycno_trans1[4] ={
    1, /*cycno = 0*/
    1, /*cycno = 1*/
    2, /*cycno = 2*/
    3  /*cycno = 3*/
};

static UNUSED uint8_t cycno_trans2[4] ={
    1, /*cycno = 0*/
    2, /*cycno = 1*/
    3, /*cycno = 2*/
    3  /*cycno = 3*/
};

static UNUSED uint8_t cycno_trans_2d[2][4]=
{
    {1, 1, 2, 3}, /*cycno_trans1*/
    {1, 2, 3, 3}  /*cycno_trans2*/
};

static UNUSED const uint16_t lg_pmps_tab_mps[6][1024] ={
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0}, 
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0},
{
0,1,2,3,4,5,6,7,7,8,9,10,11,12,13,14,14,15,16,17,18,19,20,21,21,22,23,24,25,26,27,
28,27,28,29,30,31,32,33,34,34,35,36,37,38,39,40,41,41,42,43,44,45,46,47,48,48,
49,50,51,52,53,54,55,54,55,56,57,58,59,60,61,61,62,63,64,65,66,67,68,68,69,70,71
,72,73,74,75,75,76,77,78,79,80,81,82,81,82,83,84,85,86,87,88,88,89,90,91,92,93,94,
95,95,96,97,98,99,100,101,102,102,103,104,105,106,107,108,109,
108,109,110,111,112,113,114,115,115,116,117,118,119,120,121,122,122,123,124,125,
126,127,128,129,129,130,131,132,133,134,135,136,135,136,137,138,139,140,141,142,
142,143,144,145,146,147,148,149,149,150,151,152,153,154,155,156,156,157,158,159,
160,161,162,163,162,163,164,165,166,167,168,169,169,170,171,172,173,174,175,176,
176,177,178,179,180,181,182,183,183,184,185,186,187,188,189,190,189,190,191,192,
193,194,195,196,196,197,198,199,200,201,202,203,203,204,205,206,207,208,209,210,
210,211,212,213,214,215,216,217,
216,217,218,219,220,221,222,223,223,224,225,226,227,228,229,230,230,231,232,233,
234,235,236,237,237,238,239,240,241,242,243,244,243,244,245,246,247,248,249,250,
250,251,252,253,254,255,256,257,257,258,259,260,261,262,263,264,264,265,266,267,
268,269,270,271,270,271,272,273,274,275,276,277,277,278,279,280,281,282,283,284,
284,285,286,287,288,289,290,291,291,292,293,294,295,296,297,298,297,298,299,300,
301,302,303,304,304,305,306,307,308,309,310,311,311,312,313,314,315,316,317,318,
318,319,320,321,322,323,324,325,
324,325,326,327,328,329,330,331,331,332,333,334,335,336,337,338,338,339,340,341,
342,343,344,345,345,346,347,348,349,350,351,352,351,352,353,354,355,356,357,358,
358,359,360,361,362,363,364,365,365,366,367,368,369,370,371,372,372,373,374,375,
376,377,378,379,378,379,380,381,382,383,384,385,385,386,387,388,389,390,391,392,
392,393,394,395,396,397,398,399,399,400,401,402,403,404,405,406,405,406,407,408,
409,410,411,412,412,413,414,415,416,417,418,419,419,420,421,422,423,424,425,426,
426,427,428,429,430,431,432,433,
432,433,434,435,436,437,438,439,439,440,441,442,443,444,445,446,446,447,448,449,
450,451,452,453,453,454,455,456,457,458,459,460,459,460,461,462,463,464,465,466,
466,467,468,469,470,471,472,473,473,474,475,476,477,478,479,480,480,481,482,483,
484,485,486,487,486,487,488,489,490,491,492,493,493,494,495,496,497,498,499,500,
500,501,502,503,504,505,506,507,507,508,509,510,511,512,513,514,513,514,515,516,
517,518,519,520,520,521,522,523,524,525,526,527,527,528,529,530,531,532,533,534,
534,535,536,537,538,539,540,541,
540,541,542,543,544,545,546,547,547,548,549,550,551,552,553,554,554,555,556,557,
558,559,560,561,561,562,563,564,565,566,567,568,567,568,569,570,571,572,573,574,
574,575,576,577,578,579,580,581,581,582,583,584,585,586,587,588,588,589,590,591,
592,593,594,595,594,595,596,597,598,599,600,601,601,602,603,604,605,606,607,608,
608,609,610,611,612,613,614,615,615,616,617,618,619,620,621,622,621,622,623,624,
625,626,627,628,628,629,630,631,632,633,634,635,635,636,637,638,639,640,641,642,
642,643,644,645,646,647,648,649,
648,649,650,651,652,653,654,655,655,656,657,658,659,660,661,662,662,663,664,665,
666,667,668,669,669,670,671,672,673,674,675,676,675,676,677,678,679,680,681,682,
682,683,684,685,686,687,688,689,689,690,691,692,693,694,695,696,696,697,698,699,
700,701,702,703,702,703,704,705,706,707,708,709,709,710,711,712,713,714,715,716,
716,717,718,719,720,721,722,723,723,724,725,726,727,728,729,730,729,730,731,732,
733,734,735,736,736,737,738,739,740,741,742,743,743,744,745,746,747,748,749,750,
750,751,752,753,754,755,756,757,
756,757,758,759,760,761,762,763,763,764,765,766,767,768,769,770,770,771,772,773,
774,775,776,777,777,778,779,780,781,782,783,784,783,784,785,786,787,788,789,790,
790,791,792,793,794,795,796,797,797,798,799,800,801,802,803,804,804,805,806,807,
808,809,810,811,810,811,812,813,814,815,816,817,817,818,819,820,821,822,823,824,
824,825,826,827,828,829,830,831,831,832,833,834,835,836,837,838,837,838,839,840,
841,842,843,844,844,845,846,847,848,849,850,851,851,852,853,854,855,856,857,858,
858,859,860,861,862,863,864,865
}, /* cwr=3 */
{
0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15,16,17,18,19,20,21,22,23,24,25,26,27,28,
29,30,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,45,46,47,48,49,50,51,52,53
,54,55,56,57,58,59,60,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,74,75,76,77,
78,79,80,81,82,83,84,85,86,87,88,89,89,90,91,92,93,94,95,96,97,98,99,100,101,102,
103,104,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,
118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,133,134,135,136,
137,138,139,140,141,142,143,144,145,146,147,148,148,149,150,151,152,153,154,155,
156,157,158,159,160,161,162,163,163,164,165,166,167,168,169,170,171,172,173,174,
175,176,177,178,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,
192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,207,208,209,210,
211,212,213,214,215,216,217,218,219,220,221,222,222,223,224,225,226,227,228,229,
230,231,232,233,234,235,236,237,
236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,251,252,253,254,
255,256,257,258,259,260,261,262,263,264,265,266,266,267,268,269,270,271,272,273,
274,275,276,277,278,279,280,281,281,282,283,284,285,286,287,288,289,290,291,292,
293,294,295,296,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,
310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,325,326,327,328,
329,330,331,332,333,334,335,336,337,338,339,340,340,341,342,343,344,345,346,347,
348,349,350,351,352,353,354,355,
354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,369,370,371,372,
373,374,375,376,377,378,379,380,381,382,383,384,384,385,386,387,388,389,390,391,
392,393,394,395,396,397,398,399,399,400,401,402,403,404,405,406,407,408,409,410,
411,412,413,414,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,
428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,443,444,445,446,
447,448,449,450,451,452,453,454,455,456,457,458,458,459,460,461,462,463,464,465,
466,467,468,469,470,471,472,473,
472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,487,488,489,490,
491,492,493,494,495,496,497,498,499,500,501,502,502,503,504,505,506,507,508,509,
510,511,512,513,514,515,516,517,517,518,519,520,521,522,523,524,525,526,527,528,
529,530,531,532,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,
546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,561,562,563,564,
565,566,567,568,569,570,571,572,573,574,575,576,576,577,578,579,580,581,582,583,
584,585,586,587,588,589,590,591,
590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,605,606,607,608,
609,610,611,612,613,614,615,616,617,618,619,620,620,621,622,623,624,625,626,627,
628,629,630,631,632,633,634,635,635,636,637,638,639,640,641,642,643,644,645,646,
647,648,649,650,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,
664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,679,680,681,682,
683,684,685,686,687,688,689,690,691,692,693,694,694,695,696,697,698,699,700,701,
702,703,704,705,706,707,708,709,
708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,723,724,725,726,
727,728,729,730,731,732,733,734,735,736,737,738,738,739,740,741,742,743,744,745,
746,747,748,749,750,751,752,753,753,754,755,756,757,758,759,760,761,762,763,764,
765,766,767,768,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,
782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,797,798,799,800,
801,802,803,804,805,806,807,808,809,810,811,812,812,813,814,815,816,817,818,819,
820,821,822,823,824,825,826,827,
826,827,828,829,830,831,832,833,834,835,836,837,838,839,840,841,841,842,843,844,
845,846,847,848,849,850,851,852,853,854,855,856,856,857,858,859,860,861,862,863,
864,865,866,867,868,869,870,871,871,872,873,874,875,876,877,878,879,880,881,882,
883,884,885,886,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,
900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,915,916,917,918,
919,920,921,922,923,924,925,926,927,928,929,930,930,931,932,933,934,935,936,937,
938,939,940,941,942,943,944,945
}, /* cwr=4  */
{
0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,
30,31,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55
,56,57,58,59,60,61,62,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,
82,83,84,85,86,87,88,89,90,91,92,93,93,94,95,96,97,98,99,100,101,102,103,104,105,
106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,
123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,
143,144,145,146,147,148,149,150,151,152,153,154,154,155,156,157,158,159,160,161,
162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,
182,183,184,185,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,
201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,216,217,218,219,
220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,
240,241,242,243,244,245,246,247,
246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,
266,267,268,269,270,271,272,273,274,275,276,277,277,278,279,280,281,282,283,284,
285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,
305,306,307,308,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,
324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,339,340,341,342,
343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,
363,364,365,366,367,368,369,370,
369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,
389,390,391,392,393,394,395,396,397,398,399,400,400,401,402,403,404,405,406,407,
408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,
428,429,430,431,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,
447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,462,463,464,465,
466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,
486,487,488,489,490,491,492,493,
492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,
512,513,514,515,516,517,518,519,520,521,522,523,523,524,525,526,527,528,529,530,
531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,
551,552,553,554,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,
570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,585,586,587,588,
589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,
609,610,611,612,613,614,615,616,
615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,
635,636,637,638,639,640,641,642,643,644,645,646,646,647,648,649,650,651,652,653,
654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,
674,675,676,677,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,
693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,708,709,710,711,
712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,
732,733,734,735,736,737,738,739,
738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,
758,759,760,761,762,763,764,765,766,767,768,769,769,770,771,772,773,774,775,776,
777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,
797,798,799,800,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,815,
816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,831,832,833,834,
835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,
855,856,857,858,859,860,861,862,
861,862,863,864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,
881,882,883,884,885,886,887,888,889,890,891,892,892,893,894,895,896,897,898,899,
900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,
920,921,922,923,923,924,925,926,927,928,929,930,931,932,933,934,935,936,937,938,
939,940,941,942,943,944,945,946,947,948,949,950,951,952,953,954,954,955,956,957,
958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,
978,979,980,981,982,983,984,985
} /*cwr = 5*/
};

static UNUSED const uint8_t lg_pmps_shift2[1024] ={ /* lg_pmps>>2 */
0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,8,8,8,8,9,9,9,9,
10,10,10,10,11,11,11,11,12,12,12,12,13,13,13,13,14,14,14,14,15,15,15,15,16,16,16
,16,17,17,17,17,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,21,22,22,22,22,23,23,
23,23,24,24,24,24,25,25,25,25,26,26,26,26,27,27,27,27,28,28,28,28,29,29,29,29,
30,30,30,30,31,31,31,31,
32,32,32,32,33,33,33,33,34,34,34,34,35,35,35,35,36,36,36,36,37,37,37,37,38,38,38
,38,39,39,39,39,40,40,40,40,41,41,41,41,42,42,42,42,43,43,43,43,44,44,44,44,45,45,
45,45,46,46,46,46,47,47,47,47,48,48,48,48,49,49,49,49,50,50,50,50,51,51,51,51,
52,52,52,52,53,53,53,53,54,54,54,54,55,55,55,55,56,56,56,56,57,57,57,57,58,58,58
,58,59,59,59,59,60,60,60,60,61,61,61,61,62,62,62,62,63,63,63,63,
64,64,64,64,65,65,65,65,66,66,66,66,67,67,67,67,68,68,68,68,69,69,69,69,70,70,70
,70,71,71,71,71,72,72,72,72,73,73,73,73,74,74,74,74,75,75,75,75,76,76,76,76,77,77,
77,77,78,78,78,78,79,79,79,79,80,80,80,80,81,81,81,81,82,82,82,82,83,83,83,83,
84,84,84,84,85,85,85,85,86,86,86,86,87,87,87,87,88,88,88,88,89,89,89,89,90,90,90
,90,91,91,91,91,92,92,92,92,93,93,93,93,94,94,94,94,95,95,95,95,
96,96,96,96,97,97,97,97,98,98,98,98,99,99,99,99,100,100,100,100,101,101,101,101,
102,102,102,102,103,103,103,103,104,104,104,104,105,105,105,105,106,106,106,106,
107,107,107,107,108,108,108,108,109,109,109,109,110,110,110,110,111,111,111,111,
112,112,112,112,113,113,113,113,114,114,114,114,115,115,115,115,116,116,116,116,
117,117,117,117,118,118,118,118,119,119,119,119,120,120,120,120,121,121,121,121,
122,122,122,122,123,123,123,123,124,124,124,124,125,125,125,125,126,126,126,126,
127,127,127,127,
128,128,128,128,129,129,129,129,130,130,130,130,131,131,131,131,132,132,132,132,
133,133,133,133,134,134,134,134,135,135,135,135,136,136,136,136,137,137,137,137,
138,138,138,138,139,139,139,139,140,140,140,140,141,141,141,141,142,142,142,142,
143,143,143,143,144,144,144,144,145,145,145,145,146,146,146,146,147,147,147,147,
148,148,148,148,149,149,149,149,150,150,150,150,151,151,151,151,152,152,152,152,
153,153,153,153,154,154,154,154,155,155,155,155,156,156,156,156,157,157,157,157,
158,158,158,158,159,159,159,159,
160,160,160,160,161,161,161,161,162,162,162,162,163,163,163,163,164,164,164,164,
165,165,165,165,166,166,166,166,167,167,167,167,168,168,168,168,169,169,169,169,
170,170,170,170,171,171,171,171,172,172,172,172,173,173,173,173,174,174,174,174,
175,175,175,175,176,176,176,176,177,177,177,177,178,178,178,178,179,179,179,179,
180,180,180,180,181,181,181,181,182,182,182,182,183,183,183,183,184,184,184,184,
185,185,185,185,186,186,186,186,187,187,187,187,188,188,188,188,189,189,189,189,
190,190,190,190,191,191,191,191,
192,192,192,192,193,193,193,193,194,194,194,194,195,195,195,195,196,196,196,196,
197,197,197,197,198,198,198,198,199,199,199,199,200,200,200,200,201,201,201,201,
202,202,202,202,203,203,203,203,204,204,204,204,205,205,205,205,206,206,206,206,
207,207,207,207,208,208,208,208,209,209,209,209,210,210,210,210,211,211,211,211,
212,212,212,212,213,213,213,213,214,214,214,214,215,215,215,215,216,216,216,216,
217,217,217,217,218,218,218,218,219,219,219,219,220,220,220,220,221,221,221,221,
222,222,222,222,223,223,223,223,
224,224,224,224,225,225,225,225,226,226,226,226,227,227,227,227,228,228,228,228,
229,229,229,229,230,230,230,230,231,231,231,231,232,232,232,232,233,233,233,233,
234,234,234,234,235,235,235,235,236,236,236,236,237,237,237,237,238,238,238,238,
239,239,239,239,240,240,240,240,241,241,241,241,242,242,242,242,243,243,243,243,
244,244,244,244,245,245,245,245,246,246,246,246,247,247,247,247,248,248,248,248,
249,249,249,249,250,250,250,250,251,251,251,251,252,252,252,252,253,253,253,253,
254,254,254,254,255,255,255,255,
};

static UNUSED uint8_t t_rlps_shift[512]= {
0,8,7,7,6,6,6,6,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,
3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

/* init the contexts given i_slice_type, the quantif and the model */
void cavs_cabac_context_init( cavs_decoder *h/*, cavs_cabac_t *cb, int i_slice_type, int i_qp, int i_model*/ );
int cavs_cabac_start_decoding(cavs_cabac_t * cb, cavs_bitstream *s);
int cavs_cabac_get_skip(cavs_decoder *p);
int cavs_cabac_get_mb_type_p(cavs_decoder *p);
int cavs_cabac_get_mb_type_b(cavs_decoder *p);
int cavs_cabac_get_intra_luma_pred_mode(cavs_decoder *p);
int cavs_cabac_get_intra_chroma_pred_mode(cavs_decoder *p);
int cavs_cabac_get_cbp(cavs_decoder *p);
int cavs_cabac_get_dqp(cavs_decoder *p);
int cavs_cabac_get_mvd(cavs_decoder *p, int i_list, int mvd_scan_idx, int xy_idx);
int cavs_cabac_get_ref_p(cavs_decoder *p, int ref_scan_idx);
int cavs_cabac_get_ref_b(cavs_decoder *p, int i_list, int ref_scan_idx);
int cavs_cabac_get_mb_part_type(cavs_decoder *p);
int cavs_cabac_get_coeffs(cavs_decoder *p, const cavs_vlc *p_vlc_table, int i_escape_order, int b_chroma);
int cavs_biari_decode_stuffing_bit(cavs_cabac_t *cb);
int cavs_cabac_get_mb_weighting_prediction(cavs_decoder *p);

//////////////////////////////////////////////////////////////////

typedef struct
{
    uint8_t *p_ori;
    unsigned char buf[SVA_STREAM_BUF_SIZE];
    unsigned int uClearBits;                //不含填充位的位缓冲，32位，初始值是0xFFFFFFFF
    unsigned int uPre3Bytes;                //含填充位的位缓冲，32位，初始值是0x00000000
    int iBytePosition;		                   //当前字节位置
    int iBufBytesNum;		                   //最近一次读入缓冲区的字节数
    int iClearBitsNum;		                   //不含填充位的位的个数
    int iStuffBitsNum;		                   //已剔除的填充位的个数，遇到开始码时置0
    int iBitsCount;			                   //码流总位数
    int demulate_enable;

    int64_t min_length;
}bitstream_backup;

typedef struct
{
    uint32_t i_startcode;
    uint8_t *p_start; /* exclude startcode */
    int i_len;
    
}cavs_slice_t;

#define CAVS_MAX_SLICE_SIZE 64 /* decoder support max size of slice in one frame */

typedef struct
{
	unsigned char m_buf[MAX_CODED_FRAME_SIZE];

    uint8_t *p_start;	//当前帧的输入流起始地址
    uint8_t *p_cur;
    uint8_t *p_end;
   
    int i_len;			//帧包总字节数
    int frame_num;       //当前帧的解码序号
    int slice_num;		 //当前帧的slice数
    int max_frame_size;

    uint32_t i_startcode_type; /* frametype */

    bitstream_backup bits;

    cavs_slice_t slice[CAVS_MAX_SLICE_SIZE];

    int i_mb_end[2]; /*0:top 1:bot    mb row end of top or bot field */
    int b_bottom; /* 0 : top 1: bot */
} frame_pack;

/* frame */
uint32_t cavs_frame_pack( InputStream *p, frame_pack *frame );

/* decode packed frame */
int decode_one_frame(void *p_decoder,frame_pack *, cavs_param * );

/* top field */
uint32_t cavs_topfield_pack( InputStream *p, frame_pack *field );
int decode_top_field(void *p_decoder,frame_pack *, cavs_param * );

/* bot field */
uint32_t cavs_botfield_pack( InputStream *p, frame_pack *field );
int decode_bot_field(void *p_decoder,frame_pack *, cavs_param * );

/* multi-threads */
/* synchronized frame list */
typedef struct
{
	cavs_decoder **list;
	int i_max_size;
	int i_size;
	cavs_pthread_mutex_t mutex;
	cavs_pthread_cond_t cv_fill;  /* event signaling that the list became fuller */
	cavs_pthread_cond_t cv_empty; /* event signaling that the list became emptier */
} cavs_synch_frame_list_t;

cavs_decoder * cavs_frame_get (cavs_decoder ** list);

int cavs_synch_frame_list_init( cavs_synch_frame_list_t *slist, int size );
void cavs_synch_frame_list_delete( cavs_synch_frame_list_t *slist );
void cavs_synch_frame_list_push( cavs_synch_frame_list_t *slist, cavs_decoder *frame );
cavs_decoder *cavs_synch_frame_list_pop( cavs_synch_frame_list_t *slist );
void cavs_frame_cond_broadcast( cavs_image *frame, int i_lines_completed, int b_bot_field );
void cavs_frame_cond_wait( cavs_image *frame, int i_lines_completed, int b_bot_field );


#define CHECKED_MALLOC( var, size )\
	do {\
	var = cavs_malloc( size );\
	if( !var )\
	goto fail;\
	} while( 0 )

#define CHECKED_MALLOCZERO( var, size )\
	do {\
	CHECKED_MALLOC( var, size );\
	memset( var, 0, size );\
	} while( 0 )

//////////////////////////////////////////////////////////////////////////////////////
#define B_MB_WEIGHTING 1 /* mb weighting prediction */

#define B4_SIZE	    4
extern uint8_t crop_table[];
static const cavs_vector MV_NOT_AVAIL    = {0,0,1,NOT_AVAIL};
static const cavs_vector MV_REF_DIR   = {0,0,1,REF_DIR};
static const cavs_vector MV_INTRA = {0,0,1,REF_INTRA};
#define EOB 0,0,0

typedef int (*cavs_bitstream_get_t) (cavs_decoder *p);
typedef int (*cavs_bitstream_get_coeffs_t) (cavs_decoder *p, const cavs_vlc *p_vlc_table, int i_escape_order, int b_chroma);
typedef int (*cavs_bitstream_get_mvd_t)(cavs_decoder *p, int i_list, int mvd_scan_idx, int xy_idx);
typedef int (*cavs_bitstream_get_ref_p_t)(cavs_decoder *p, int ref_scan_idx);
typedef int (*cavs_bitstream_get_ref_b_t)(cavs_decoder *p, int i_list, int ref_scan_idx);

enum syntax_element
{
	SYNTAX_SKIP_RUN,
	SYNTAX_MBTYPE_P,
	SYNTAX_MBTYPE_B,
	SYNTAX_INTRA_LUMA_PRED_MODE,
	SYNTAX_INTRA_CHROMA_PRED_MODE,
	SYNTAX_INTRA_CBP,
	SYNTAX_INTER_CBP,
	SYNTAX_DQP,
	SYNTAX_REF_P,
	SYNTAX_REF_B,
	SYNTAX_MB_PART_TYPE
};

enum macroblock_position_e
{
	MB_LEFT = 0x01,
	MB_TOP = 0x02,
	MB_TOPRIGHT = 0x04,
	MB_TOPLEFT = 0x08,
	MB_DOWNLEFT = 0x10,

  //MB_PRIVATE  = 0x10,
  ALL_NEIGHBORS = 0x1f,
};

#if B_MB_WEIGHTING
typedef void (*cavs_avg_func)( uint8_t *dst,  int i_dst,  uint8_t *src, int i_src );
#endif

typedef void (*cavs_luma_mc_func)(uint8_t *dst/*align width (8 or 16)*/, uint8_t *src/*align 1*/, int stride);
typedef void (*cavs_chroma_mc_func)(uint8_t *dst/*align 8*/, uint8_t *src/*align 1*/, int srcStride, int h, int x, int y);

typedef void (*cavs_intra_pred_luma)(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top, uint8_t *p_left);
typedef void (*cavs_intra_pred_chroma)(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left);
typedef void (*cavs_filter_lv)(cavs_decoder *p,uint8_t *d, int stride, /*int alpha, int beta, int tc,*/int i_qp, int bs1, int bs2) ;
typedef void (*cavs_filter_lh)(cavs_decoder *p,uint8_t *d, int stride, /*int alpha, int beta, int tc,*/int i_qp, int bs1, int bs2) ;
typedef void (*cavs_filter_cv)(cavs_decoder *p,uint8_t *d, int stride, /*int alpha, int beta, int tc,*/int i_qp, int bs1, int bs2) ;
typedef void (*cavs_filter_ch)(cavs_decoder *p,uint8_t *d, int stride, /*int alpha, int beta, int tc,*/int i_qp, int bs1, int bs2) ;

struct tagcavs_decoder
{
    cavs_bitstream s;
    cavs_video_sequence_header vsh;
	cavs_video_sequence_header old;
    cavs_picture_header ph;
    cavs_slice_header sh;
    uint8_t user_data;
    cavs_sequence_display_extension sde;
    cavs_copyright_extension ce;
    cavs_camera_parameters_extension cpe;
    cavs_picture_display_extension pde;
    uint8_t b_extention_flag;
   
    uint8_t i_video_edit_code_flag;

    uint32_t i_frame_num;

    uint8_t b_get_video_sequence_header;
    uint8_t b_get_i_picture_header;
    uint8_t b_have_pred;
    uint32_t i_mb_width, i_mb_height, i_mb_num, i_mb_num_half;
    uint32_t i_mb_x, i_mb_y, i_mb_index, i_mb_offset;  
    uint32_t b_complete; /* finish decoding one frame */ 
    uint32_t i_mb_flags; 
    uint32_t b_first_line; 
    uint32_t i_slice_mb_end;
    uint32_t i_slice_mb_end_fld[2];

    uint32_t i_neighbour4[16];

    cavs_image image[3]; /* alloc 3 frame memory for yuv */
    cavs_image ref[4]; /* reference field or frame, not alloc memory */
    
    cavs_image *p_save[3];/* backup point , not alloc memory */
    cavs_image cur; /* current decode frame */

    /* add for aec moudle */    
    cavs_image image_aec[3]; /* alloc 3 frame memory for yuv */
    cavs_image ref_aec[4]; /* reference field or frame, not alloc memory */
    cavs_image *p_save_aec[3];/* backup point , not alloc memory */
    cavs_image cur_aec; /* current decode frame */

    cavs_image *last_delayed_pframe;
    int i_luma_offset[4];//四个块相对于X0的地址偏移

    int b_bottom;
    
    int copy_flag3;
       
    ///以下数据结构 部分来源于ffmpeg
    //这些都是计算运动矢量的系数，用于加快速度
    int i_sym_factor;    ///<用于B块的对称模式
    int i_direct_den[4]; ///< 用于B块的直接模式
    int i_scale_den[4];  ///< 用于临近块的预测运动矢量的计算
    int i_ref_distance[4]; /* four ref-field at most */

    uint8_t *p_y, *p_cb, *p_cr, *p_edge;

#if B_MB_WEIGHTING
	uint8_t *p_back_y, *p_back_cb, *p_back_cr;
#endif

    int i_mb_type;	//当前宏块的类型
    int i_cbp_code;
    int i_left_qp;
    uint8_t *p_top_qp;//用于宏块的环路滤波
    int i_qp;//当前使用的qp，
    int b_fixed_qp;//固定qp标志，可以在slice中改变
    int i_cbp;//当前宏块的cbp
    int b_low_delay;

#if B_MB_WEIGHTING
    int weighting_prediction; /* MB weighting prediction flag */
    uint8_t *mb_line_buffer[3]; /* temp buffer for backward mc */     
#endif

    /** 
       0:    D3  B2  B3  C2
       4:    A1  X0  X1   -
       8:    A3  X2  X3   - */
    //24个保存前向后向各块运动矢量
    cavs_vector mv[24];			/*mv context of current block*/
    cavs_vector *p_top_mv[2];
    cavs_vector *p_col_mv;		/*mv buffer of colocated ref frame(for direct mode)*/

    /** luma pred mode cache
       0:	 D   B1  B2  B3  B4
       5:    A1  X0  X1  X2  X3
       10:   A2  X4  X5  X6  X7
       15:   A3  X8  X9 X10 X11
       20:   A4 X12 X13 X14 X15
    */
    int i_intra_pred_mode_y[25];	//
    int *p_top_intra_pred_mode_y;//保存上面一行的亮度预测模式
    int i_pred_mode_chroma;	//当前色度宏块的预测模式

    //for CABAC
    int8_t i_mb_type_left;		//左边宏块的类型
    int8_t *p_mb_type_top;		//上边行宏块的类型
    int8_t i_chroma_pred_mode_left;
    int8_t *p_chroma_pred_mode_top;
    int8_t i_cbp_left;
    int8_t *p_cbp_top;
    int i_last_dqp;
    /** mvd cache
       0:    A1  X0  X1
       3:    A3  X2  X3   */
    int16_t p_mvd[2][6][2];			//[FWD/BWD][pos][x/y]
    int8_t (*p_ref_top)[2][2];		//[FWD/BWD][b8_pos]
    /** ref cache
       0:	 D3	 B2  B3
       3:    A1  X0  X1
       6:    A3  X2  X3   */
    int8_t p_ref[2][9];				//[FWD/BWD][pos]

    //这里宏块内不同的块对应的边缘不一样甚至用到刚解码的数据，或者由于有些右边块并未解码所以限制了一些预测模式的是使用
    //所有的这些都笔削在deblock前保存下来
    //用于保存帧内预测的当前宏块临近的上面行的y样本标准中c[0~16]
    uint8_t *p_top_border_y,*p_top_border_cb,*p_top_border_cr; 
    //用于保存帧内预测的当前宏块临近的左边行的y样本
    uint8_t i_left_border_y[26],i_left_border_cb[10],i_left_border_cr[10];
    //用于中间边界样本的保存，用于X1,X3的预测
    uint8_t i_internal_border_y[26];
    uint8_t i_topleft_border_y,i_topleft_border_cb,i_topleft_border_cr;//保存标准中的c[0]或r[0]
    uint8_t *p_col_type_base;
    //用于保存宏块的类型，事实只有B_SKIP和B_Direct会用到，所以只需要保存P帧的就可以了
    uint8_t *p_col_type;//当前宏块类型的偏移地址	
    //保存残差数据
    DCTELEM *p_block;
    int level_buf[64];
    int run_buf[64];

	int protect_array[64]; /* FIXIT */

    /* run and level coeffs */
    int (*level_buf_tab)[6][64];/*[block][coeff]*/
    int (*run_buf_tab)[6][64];
    int (*num_buf_tab)[6];

    /* mb type */
    int *i_mb_type_tab;

    /* mb qp */
    int *i_qp_tab;

    /* cbp */
    int *i_cbp_tab;

    /* mv */
    cavs_vector (*mv_tab)[24];

    /* luma pred mode */
    int (*i_intra_pred_mode_y_tab)[25];

    /* chroma pred mode */
    int *i_pred_mode_chroma_tab;

    /* mvd */
    int16_t (*p_mvd_tab)[2][6][2];

    /* refer index */
    int8_t (*p_ref_tab)[2][9];

	/* weighting prediction */
#if B_MB_WEIGHTING
    int *weighting_prediction_tab;

	/* keep this for accelerate mode of field */
    uint8_t  b_slice_weighting_flag[2]; /*[top/bot]*/
	uint32_t i_luma_scale[2][4]; /*[top/bot][refnum]*/
	int32_t  i_luma_shift[2][4]; /*[top/bot][refnum]*/
	uint32_t i_chroma_scale[2][4]; /*[top/bot][refnum]*/
	int32_t  i_chroma_shift[2][4]; /*[top/bot][refnum]*/
	uint8_t  b_mb_weighting_flag[2]; /*[top/bot]*/
#endif

    /*2 for 1/4 subpel, 3 for 1/8 subpel...*/
    int i_subpel_precision;
    /*4 for 1/4 subpel, 8 for 1/8 subpel...*/
    int i_uint_length;
    int vbs_mb_intra_pred_flag ;
    int vbs_mb_part_intra_pred_flag[4] ;
    int vbs_mb_part_transform_4x4_flag[4];
    int quant_coeff_pred_flag;
    int pred_mode_4x4_flag[4];
    int intra_luma_pred_mode_4x4[4];
    int pred_mode_flag;
    int i_cbp_4x4[4];
    int i_cbp_part[4];

    //aec for guangdian profile
    cavs_cabac_t cabac;

    struct
    {
        unsigned int i_neighbour;
        unsigned int i_neighbour8[4];       /* neighbours of each 8x8 or 4x4 block that are available */
    }mb;

    int copy_flag1;
    
    /* MC */
    cavs_luma_mc_func put_cavs_qpel_pixels_tab[2][16];
    cavs_luma_mc_func avg_cavs_qpel_pixels_tab[2][16];
    cavs_chroma_mc_func put_h264_chroma_pixels_tab[3];
    cavs_chroma_mc_func avg_h264_chroma_pixels_tab[3];

    /* intra predict */
    cavs_intra_pred_luma cavs_intra_luma[8];
    cavs_intra_pred_chroma cavs_intra_chroma[7];

#if B_MB_WEIGHTING
    /* weighting prediction avg */
    cavs_avg_func cavs_avg_pixels_tab[3]; /* 0 :16x16 1: 8x8 2: 4x4 */
#endif
    
    /*deblock*/
    cavs_filter_lv filter_lv;
    cavs_filter_lh filter_lh;
    cavs_filter_cv filter_cv;
    cavs_filter_ch filter_ch;

#define SYNTAX_NUM 11
    cavs_bitstream_get_t bs_read[SYNTAX_NUM];
    cavs_bitstream_get_coeffs_t bs_read_coeffs;
    cavs_bitstream_get_mvd_t bs_read_mvd;
    cavs_bitstream_get_ref_p_t bs_read_ref_p;
    cavs_bitstream_get_ref_b_t bs_read_ref_b;

    cavs_param param;

    int b_skip_first;

    frame_pack *Frame; /* used for frame pack of multi-slice */
    
    InputStream *p_stream; /* locate stream */

    /* thread */
    int b_thread_flag;
    cavs_decoder *thread[CAVS_THREAD_MAX];
	int b_threadpool_flag;
    cavs_threadpool_t *threadpool;

    cavs_pthread_mutex_t mutex;
    cavs_pthread_cond_t cv;
    int i_thread_idx;
    int b_thread_active;
    cavs_pthread_t id;

    /*add for error detect*/
    int b_error_flag; /* set to 1 when error detected , default 0 */

    /* add for weighting quant */ 
    int b_weight_quant_enable;
    short UseDefaultScalingMatrixFlag[2];
    short cur_wq_matrix[64];
    short wq_matrix[2][64];
    short wq_param[2][6];

    int i_frame_decoded; /*  add for debug */

    /* AEC and REC thread */
    int b_accelerate_flag;
    cavs_decoder *unused[2]; /* for AEC decode */
    cavs_decoder *unused_backup[2]; /* for handle free */
    cavs_decoder *current[2]; /* for REC recon */
    
    int i_mb_y_start; /* backup start value of AEC thread */
    int i_mb_index_start; /* */
    int i_mb_y_start_fld[2]; /* backup start value of AEC thread */
    int i_mb_index_start_fld[2]; /* */
    cavs_decoder *p_m;
    frame_pack *fld;
    int slice_count;
    int field_count;
};

static UNUSED int hd;
static UNUSED int hu;
static UNUSED int vr;

static const unsigned char LEVRUN_INTRA[7][16]= 
{
	{ 4, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
	{ 6, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 6, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 8, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{10, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{12, 5, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static const unsigned char LEVRUN_INTER[7][16]= 
{
	{ 4, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
	{ 5, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
	{ 5, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 6, 4, 3, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 7, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 9, 5, 3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{13, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static const char VLC_GC_Order_INTRA[7][2] =      //[tableindex][grad/maxlevel]
{
	{1,22}, {0,22}, {1,22}, {1,22}, {0,22}, {1,22}, {0,22}
};

static const char VLC_GC_Order_INTER[7][2] =     //[tableindex][grad/maxlevel]
{
	{1,22}, {0,22}, {0,22}, {0,22}, {0,22}, {0,22}, {0,22}
};

static const uint8_t dequant_shift[64] = 
	{
		14,14,14,14,14,14,14,14,
		13,13,13,13,13,13,13,13,
		13,12,12,12,12,12,12,12,
		11,11,11,11,11,11,11,11,
		11,10,10,10,10,10,10,10,
		10, 9, 9, 9, 9, 9, 9, 9,
		9, 8, 8, 8, 8, 8, 8, 8,
		7, 7, 7, 7, 7, 7, 7, 7
	};

	static const uint16_t dequant_mul[64] = 
	{
		32768,36061,38968,42495,46341,50535,55437,60424,
		32932,35734,38968,42495,46177,50535,55109,59933,
		65535,35734,38968,42577,46341,50617,55027,60097,
		32809,35734,38968,42454,46382,50576,55109,60056,
		65535,35734,38968,42495,46320,50515,55109,60076,
		65535,35744,38968,42495,46341,50535,55099,60087,
		65535,35734,38973,42500,46341,50535,55109,60097,
		32771,35734,38965,42497,46341,50535,55109,60099
	};
	static const uint8_t zigzag_progressive_4x4[16]=//图35
	{
		 0, 1,4, 8, 
		 5, 2, 3, 6, 
		 9, 12 ,13, 10,
		 7, 11, 14, 15, 
	};
	static const uint8_t zigzag_progressive[64]=//图33
	{
			0, 1, 8, 16, 9, 2, 3, 10,
			17, 24, 32, 25, 18, 11, 4, 5,
			12, 19, 26, 33, 40, 48, 41, 34,
			27, 20, 13, 6, 7, 14, 21, 28,
			35, 42, 49, 56, 57, 50, 43, 36,
			29, 22, 15, 23, 30, 37, 44, 51,
			58, 59, 52, 45, 38, 31, 39, 46,
			53, 60, 61, 54, 47, 55, 62, 63
	};
	
	static const uint8_t zigzag_field[64]=//图34
	{
	    0, 8, 16, 1, 24, 32, 9, 17,
		40, 48, 25, 2, 10, 56, 33, 18,
		3, 41, 49, 26, 11, 19, 4, 57,
		34, 12, 42, 27, 20, 50, 35, 28,
		5, 13, 58, 43, 36, 21, 6, 29,
		51, 44, 14, 22, 37, 59, 52, 30,
		45, 60, 38, 53, 46, 61, 54, 7,
		62, 15, 23, 31, 39, 47, 55, 63
	};

static const uint8_t chroma_qp[64] = 
{
	0,  1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
	16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
	32,33,34,35,36,37,38,39,40,41,42,42,43,43,44,44,
	45,45,46,46,47,47,48,48,48,49,49,49,50,50,50,51
};
static const char intra_2dvlc_4x4[7][16][16]=
{
	{
		{  0,  8, 16, 30, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  2, 18, 36, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  4, 20, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  6, 26, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 12, 32, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 10, 34, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 14, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 22, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 24, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 28, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  5,  9, 15, 27, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  3, 13, 23, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  7, 19, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 11, 25, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 17, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		//AVSM-1498
		{  0/*2*/, 1 /*0*/,  3,  9, 15, 23, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  5, 11, 19, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  7, 17, 27, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 13, 25, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 21, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  3,  5,  9, 13, 23, 25, 31, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  7, 11, 17, 27, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 15, 19, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 21, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  3,  5,  7, 11, 15, 17, 23, 31, 35, -1, -1, -1, -1, -1},
		{ -1,  9, 13, 19, 27, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 21, 25, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  3,  5,  7,  9, 11, 13, 19, 21, 25, 31, 35, -1, -1, -1},
		{ -1, 15, 17, 23, 29, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 27, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  3,  1,  5,  7,  9, 11, 13, 15, 17, 19, 21, 23, 31, 29, 33},
		{ -1, 25, 27, 35, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	}
};
static const char inter_2dvlc_4x4[7][16][14] = 
{
	{
		{  0, 10, 22, 36, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  2, 20, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  4, 24, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  6, 26, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{  8, 34, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 12, 30, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 14, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 16, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 18, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 32, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 28, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  7, 15, 27, 37, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  3, 17, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  5, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  9, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 11, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 13, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 19, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 23, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 25, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  9, 17, 23, 37, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  3, 15, 27, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  5, 19, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  7, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 11, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 13, 35, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 25, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  7, 11, 21, 27, 35, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  3, 13, 25, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  5, 17, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  9, 19, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 15, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 23, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  3,  7, 13, 21, 27, 35, -1, -1, -1, -1, -1, -1},
		{ -1,  5, 11, 19, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1,  9, 17, 33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 15, 29, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 23, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 25, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  3,  5,  9, 11, 17, 21, 29, 33, -1, -1, -1, -1},
		{ -1,  7, 13, 19, 25, 35, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 15, 23, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 27, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 37, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	},
	{
		{  0,  1,  3,  5,  7,  9, 11, 13, 17, 21, 23, 27, 33, 37},
		{ -1, 15, 19, 25, 29, 35, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
	}
};
static const cavs_vlc intra_2dvlc[7] = 
{
	{
		   { //level / run / table_inc
			   {  1, 1, 1},{ -1, 1, 1},{  1, 2, 1},{ -1, 2, 1},{  1, 3, 1},{ -1, 3, 1},
			   {  1, 4, 1},{ -1, 4, 1},{  1, 5, 1},{ -1, 5, 1},{  1, 6, 1},{ -1, 6, 1},
			   {  1, 7, 1},{ -1, 7, 1},{  1, 8, 1},{ -1, 8, 1},{  1, 9, 1},{ -1, 9, 1},
			   {  1,10, 1},{ -1,10, 1},{  1,11, 1},{ -1,11, 1},{  2, 1, 2},{ -2, 1, 2},
			   {  1,12, 1},{ -1,12, 1},{  1,13, 1},{ -1,13, 1},{  1,14, 1},{ -1,14, 1},
			   {  1,15, 1},{ -1,15, 1},{  2, 2, 2},{ -2, 2, 2},{  1,16, 1},{ -1,16, 1},
			   {  1,17, 1},{ -1,17, 1},{  3, 1, 3},{ -3, 1, 3},{  1,18, 1},{ -1,18, 1},
			   {  1,19, 1},{ -1,19, 1},{  2, 3, 2},{ -2, 3, 2},{  1,20, 1},{ -1,20, 1},
			   {  1,21, 1},{ -1,21, 1},{  2, 4, 2},{ -2, 4, 2},{  1,22, 1},{ -1,22, 1},
			   {  2, 5, 2},{ -2, 5, 2},{  1,23, 1},{ -1,23, 1},{   EOB }
		   },
			   //level_add
		   { 0, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		   2, 2, 2, 2, 2, 2, 2,-1,-1,-1},
		   2, //golomb_order
		   0, //inc_limit  和8.3.1对应
		   23, //max_run
	   },
	   {
		   { //level / run
			   {  1, 1, 0},{ -1, 1, 0},{  1, 2, 0},{ -1, 2, 0},{  2, 1, 1},{ -2, 1, 1},
			   {  1, 3, 0},{ -1, 3, 0},{   EOB   },{  1, 4, 0},{ -1, 4, 0},{  1, 5, 0},
			   { -1, 5, 0},{  1, 6, 0},{ -1, 6, 0},{  3, 1, 2},{ -3, 1, 2},{  2, 2, 1},
			   { -2, 2, 1},{  1, 7, 0},{ -1, 7, 0},{  1, 8, 0},{ -1, 8, 0},{  1, 9, 0},
			   { -1, 9, 0},{  2, 3, 1},{ -2, 3, 1},{  4, 1, 2},{ -4, 1, 2},{  1,10, 0},
			   { -1,10, 0},{  1,11, 0},{ -1,11, 0},{  2, 4, 1},{ -2, 4, 1},{  3, 2, 2},
			   { -3, 2, 2},{  1,12, 0},{ -1,12, 0},{  2, 5, 1},{ -2, 5, 1},{  5, 1, 3},
			   { -5, 1, 3},{  1,13, 0},{ -1,13, 0},{  2, 6, 1},{ -2, 6, 1},{  1,14, 0},
			   { -1,14, 0},{  2, 7, 1},{ -2, 7, 1},{  2, 8, 1},{ -2, 8, 1},{  3, 3, 2},
			   { -3, 3, 2},{  6, 1, 3},{ -6, 1, 3},{  1,15, 0},{ -1,15, 0}
		   },
			   //level_add
		   { 0, 7, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2,-1,
		   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
		   2, //golomb_order
		   1, //inc_limit
		   15, //max_run
		   },
		   {
			   { //level / run
				   {  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  1, 2, 0},{ -1, 2, 0},
				   {  3, 1, 1},{ -3, 1, 1},{   EOB   },{  1, 3, 0},{ -1, 3, 0},{  2, 2, 0},
				   { -2, 2, 0},{  4, 1, 1},{ -4, 1, 1},{  1, 4, 0},{ -1, 4, 0},{  5, 1, 2},
				   { -5, 1, 2},{  1, 5, 0},{ -1, 5, 0},{  3, 2, 1},{ -3, 2, 1},{  2, 3, 0},
				   { -2, 3, 0},{  1, 6, 0},{ -1, 6, 0},{  6, 1, 2},{ -6, 1, 2},{  2, 4, 0},
				   { -2, 4, 0},{  1, 7, 0},{ -1, 7, 0},{  4, 2, 1},{ -4, 2, 1},{  7, 1, 2},
				   { -7, 1, 2},{  3, 3, 1},{ -3, 3, 1},{  2, 5, 0},{ -2, 5, 0},{  1, 8, 0},
				   { -1, 8, 0},{  2, 6, 0},{ -2, 6, 0},{  8, 1, 3},{ -8, 1, 3},{  1, 9, 0},
				   { -1, 9, 0},{  5, 2, 2},{ -5, 2, 2},{  3, 4, 1},{ -3, 4, 1},{  2, 7, 0},
				   { -2, 7, 0},{  9, 1, 3},{ -9, 1, 3},{  1,10, 0},{ -1,10, 0}
			   },
				   //level_add
			   { 0,10, 6, 4, 4, 3, 3, 3, 2, 2, 2,-1,-1,-1,-1,-1,-1,
			   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			   2, //golomb_order
			   2, //inc_limit
			   10, //max_run
		   },
		   {
			   { //level / run
				   {  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},{ -3, 1, 0},
				   {  1, 2, 0},{ -1, 2, 0},{   EOB   },{  4, 1, 0},{ -4, 1, 0},{  5, 1, 1},
				   { -5, 1, 1},{  2, 2, 0},{ -2, 2, 0},{  1, 3, 0},{ -1, 3, 0},{  6, 1, 1},
				   { -6, 1, 1},{  3, 2, 0},{ -3, 2, 0},{  7, 1, 1},{ -7, 1, 1},{  1, 4, 0},
				   { -1, 4, 0},{  8, 1, 2},{ -8, 1, 2},{  2, 3, 0},{ -2, 3, 0},{  4, 2, 0},
				   { -4, 2, 0},{  1, 5, 0},{ -1, 5, 0},{  9, 1, 2},{ -9, 1, 2},{  5, 2, 1},
				   { -5, 2, 1},{  2, 4, 0},{ -2, 4, 0},{ 10, 1, 2},{-10, 1, 2},{  3, 3, 0},
				   { -3, 3, 0},{  1, 6, 0},{ -1, 6, 0},{ 11, 1, 3},{-11, 1, 3},{  6, 2, 1},
				   { -6, 2, 1},{  1, 7, 0},{ -1, 7, 0},{  2, 5, 0},{ -2, 5, 0},{  3, 4, 0},
				   { -3, 4, 0},{ 12, 1, 3},{-12, 1, 3},{  4, 3, 0},{ -4, 3, 0}
			   },
				   //level_add
			   { 0,13, 7, 5, 4, 3, 2, 2,-1,-1,-1 -1,-1,-1,-1,-1,-1,
			   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			   2, //golomb_order
			   4, //inc_limit
			   7, //max_run
			   },
			   {
				   { //level / run
					   {  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},{ -3, 1, 0},
					   {   EOB   },{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},{ -5, 1, 0},{  6, 1, 0},
					   { -6, 1, 0},{  1, 2, 0},{ -1, 2, 0},{  7, 1, 0},{ -7, 1, 0},{  8, 1, 1},
					   { -8, 1, 1},{  2, 2, 0},{ -2, 2, 0},{  9, 1, 1},{ -9, 1, 1},{ 10, 1, 1},
					   {-10, 1, 1},{  1, 3, 0},{ -1, 3, 0},{  3, 2, 0},{ -3, 2, 0},{ 11, 1, 2},
					   {-11, 1, 2},{  4, 2, 0},{ -4, 2, 0},{ 12, 1, 2},{-12, 1, 2},{ 13, 1, 2},
					   {-13, 1, 2},{  5, 2, 0},{ -5, 2, 0},{  1, 4, 0},{ -1, 4, 0},{  2, 3, 0},
					   { -2, 3, 0},{ 14, 1, 2},{-14, 1, 2},{  6, 2, 0},{ -6, 2, 0},{ 15, 1, 2},
					   {-15, 1, 2},{ 16, 1, 2},{-16, 1, 2},{  3, 3, 0},{ -3, 3, 0},{  1, 5, 0},
					   { -1, 5, 0},{  7, 2, 0},{ -7, 2, 0},{ 17, 1, 2},{-17, 1, 2}
				   },
					   //level_add
				   { 0,18, 8, 4, 2, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
				   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
				   2, //golomb_order
				   7, //inc_limit
				   5, //max_run
			   },
			   {
				   { //level / run
					   {   EOB   },{  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
					   { -3, 1, 0},{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},{ -5, 1, 0},{  6, 1, 0},
					   { -6, 1, 0},{  7, 1, 0},{ -7, 1, 0},{  8, 1, 0},{ -8, 1, 0},{  9, 1, 0},
					   { -9, 1, 0},{ 10, 1, 0},{-10, 1, 0},{  1, 2, 0},{ -1, 2, 0},{ 11, 1, 1},
					   {-11, 1, 1},{ 12, 1, 1},{-12, 1, 1},{ 13, 1, 1},{-13, 1, 1},{  2, 2, 0},
					   { -2, 2, 0},{ 14, 1, 1},{-14, 1, 1},{ 15, 1, 1},{-15, 1, 1},{  3, 2, 0},
					   { -3, 2, 0},{ 16, 1, 1},{-16, 1, 1},{  1, 3, 0},{ -1, 3, 0},{ 17, 1, 1},
					   {-17, 1, 1},{  4, 2, 0},{ -4, 2, 0},{ 18, 1, 1},{-18, 1, 1},{  5, 2, 0},
					   { -5, 2, 0},{ 19, 1, 1},{-19, 1, 1},{ 20, 1, 1},{-20, 1, 1},{  6, 2, 0},
					   { -6, 2, 0},{ 21, 1, 1},{-21, 1, 1},{  2, 3, 0},{ -2, 3, 0}
				   },
					   //level_add
				   { 0,22, 7, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
				   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
				   2, //golomb_order
				   10, //inc_limit
				   3, //max_run
				   },
				   {
					   { //level / run
						   {   EOB   },{  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
						   { -3, 1, 0},{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},{ -5, 1, 0},{  6, 1, 0},
						   { -6, 1, 0},{  7, 1, 0},{ -7, 1, 0},{  8, 1, 0},{ -8, 1, 0},{  9, 1, 0},
						   { -9, 1, 0},{ 10, 1, 0},{-10, 1, 0},{ 11, 1, 0},{-11, 1, 0},{ 12, 1, 0},
						   {-12, 1, 0},{ 13, 1, 0},{-13, 1, 0},{ 14, 1, 0},{-14, 1, 0},{ 15, 1, 0},
						   {-15, 1, 0},{ 16, 1, 0},{-16, 1, 0},{  1, 2, 0},{ -1, 2, 0},{ 17, 1, 0},
						   {-17, 1, 0},{ 18, 1, 0},{-18, 1, 0},{ 19, 1, 0},{-19, 1, 0},{ 20, 1, 0},
						   {-20, 1, 0},{ 21, 1, 0},{-21, 1, 0},{  2, 2, 0},{ -2, 2, 0},{ 22, 1, 0},
						   {-22, 1, 0},{ 23, 1, 0},{-23, 1, 0},{ 24, 1, 0},{-24, 1, 0},{ 25, 1, 0},
						   {-25, 1, 0},{  3, 2, 0},{ -3, 2, 0},{ 26, 1, 0},{-26, 1, 0}
					   },
						   //level_add
					   { 0,27, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
					   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
					   2, //golomb_order
					   INT_MAX, //inc_limit
					   2, //max_run
				   }
};

static const cavs_vlc inter_2dvlc[7] = 
{
	   {
		   { //level / run
			   {  1, 1, 1},{ -1, 1, 1},{  1, 2, 1},{ -1, 2, 1},{  1, 3, 1},{ -1, 3, 1},
			   {  1, 4, 1},{ -1, 4, 1},{  1, 5, 1},{ -1, 5, 1},{  1, 6, 1},{ -1, 6, 1},
			   {  1, 7, 1},{ -1, 7, 1},{  1, 8, 1},{ -1, 8, 1},{  1, 9, 1},{ -1, 9, 1},
			   {  1,10, 1},{ -1,10, 1},{  1,11, 1},{ -1,11, 1},{  1,12, 1},{ -1,12, 1},
			   {  1,13, 1},{ -1,13, 1},{  2, 1, 2},{ -2, 1, 2},{  1,14, 1},{ -1,14, 1},
			   {  1,15, 1},{ -1,15, 1},{  1,16, 1},{ -1,16, 1},{  1,17, 1},{ -1,17, 1},
			   {  1,18, 1},{ -1,18, 1},{  1,19, 1},{ -1,19, 1},{  3, 1, 3},{ -3, 1, 3},
			   {  1,20, 1},{ -1,20, 1},{  1,21, 1},{ -1,21, 1},{  2, 2, 2},{ -2, 2, 2},
			   {  1,22, 1},{ -1,22, 1},{  1,23, 1},{ -1,23, 1},{  1,24, 1},{ -1,24, 1},
			   {  1,25, 1},{ -1,25, 1},{  1,26, 1},{ -1,26, 1},{   EOB   }
		   },
			   //level_add
		   { 0, 4, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		   2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
		   3, //golomb_order
		   0, //inc_limit
		   26 //max_run
	   },
	   {
		   { //level / run
			   {  1, 1, 0},{ -1, 1, 0},{   EOB   },{  1, 2, 0},{ -1, 2, 0},{  1, 3, 0},
			   { -1, 3, 0},{  1, 4, 0},{ -1, 4, 0},{  1, 5, 0},{ -1, 5, 0},{  1, 6, 0},
			   { -1, 6, 0},{  2, 1, 1},{ -2, 1, 1},{  1, 7, 0},{ -1, 7, 0},{  1, 8, 0},
			   { -1, 8, 0},{  1, 9, 0},{ -1, 9, 0},{  1,10, 0},{ -1,10, 0},{  2, 2, 1},
			   { -2, 2, 1},{  1,11, 0},{ -1,11, 0},{  1,12, 0},{ -1,12, 0},{  3, 1, 2},
			   { -3, 1, 2},{  1,13, 0},{ -1,13, 0},{  1,14, 0},{ -1,14, 0},{  2, 3, 1},
			   { -2, 3, 1},{  1,15, 0},{ -1,15, 0},{  2, 4, 1},{ -2, 4, 1},{  1,16, 0},
			   { -1,16, 0},{  2, 5, 1},{ -2, 5, 1},{  1,17, 0},{ -1,17, 0},{  4, 1, 3},
			   { -4, 1, 3},{  2, 6, 1},{ -2, 6, 1},{  1,18, 0},{ -1,18, 0},{  1,19, 0},
			   { -1,19, 0},{  2, 7, 1},{ -2, 7, 1},{  3, 2, 2},{ -3, 2, 2}
		   },
			   //level_add
		   { 0, 5, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		   2, 2, 2,-1,-1,-1,-1,-1,-1,-1},
		   2, //golomb_order
		   1, //inc_limit
		   19 //max_run
		   },
		   {
			   { //level / run
				   {  1, 1, 0},{ -1, 1, 0},{   EOB   },{  1, 2, 0},{ -1, 2, 0},{  2, 1, 0},
				   { -2, 1, 0},{  1, 3, 0},{ -1, 3, 0},{  1, 4, 0},{ -1, 4, 0},{  3, 1, 1},
				   { -3, 1, 1},{  2, 2, 0},{ -2, 2, 0},{  1, 5, 0},{ -1, 5, 0},{  1, 6, 0},
				   { -1, 6, 0},{  1, 7, 0},{ -1, 7, 0},{  2, 3, 0},{ -2, 3, 0},{  4, 1, 2},
				   { -4, 1, 2},{  1, 8, 0},{ -1, 8, 0},{  3, 2, 1},{ -3, 2, 1},{  2, 4, 0},
				   { -2, 4, 0},{  1, 9, 0},{ -1, 9, 0},{  1,10, 0},{ -1,10, 0},{  5, 1, 2},
				   { -5, 1, 2},{  2, 5, 0},{ -2, 5, 0},{  1,11, 0},{ -1,11, 0},{  2, 6, 0},
				   { -2, 6, 0},{  1,12, 0},{ -1,12, 0},{  3, 3, 1},{ -3, 3, 1},{  6, 1, 2},
				   { -6, 1, 2},{  4, 2, 2},{ -4, 2, 2},{  1,13, 0},{ -1,13, 0},{  2, 7, 0},
				   { -2, 7, 0},{  3, 4, 1},{ -3, 4, 1},{  1,14, 0},{ -1,14, 0}
			   },
				   //level_add
			   { 0, 7, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2,-1,-1,
			   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			   2, //golomb_order
			   2, //inc_limit
			   14 //max_run
		   },
		   {
			   { //level / run
				   {  1, 1, 0},{ -1, 1, 0},{   EOB   },{  2, 1, 0},{ -2, 1, 0},{  1, 2, 0},
				   { -1, 2, 0},{  3, 1, 0},{ -3, 1, 0},{  1, 3, 0},{ -1, 3, 0},{  2, 2, 0},
				   { -2, 2, 0},{  4, 1, 1},{ -4, 1, 1},{  1, 4, 0},{ -1, 4, 0},{  5, 1, 1},
				   { -5, 1, 1},{  1, 5, 0},{ -1, 5, 0},{  3, 2, 0},{ -3, 2, 0},{  2, 3, 0},
				   { -2, 3, 0},{  1, 6, 0},{ -1, 6, 0},{  6, 1, 1},{ -6, 1, 1},{  2, 4, 0},
				   { -2, 4, 0},{  1, 7, 0},{ -1, 7, 0},{  4, 2, 1},{ -4, 2, 1},{  7, 1, 2},
				   { -7, 1, 2},{  3, 3, 0},{ -3, 3, 0},{  1, 8, 0},{ -1, 8, 0},{  2, 5, 0},
				   { -2, 5, 0},{  8, 1, 2},{ -8, 1, 2},{  1, 9, 0},{ -1, 9, 0},{  3, 4, 0},
				   { -3, 4, 0},{  2, 6, 0},{ -2, 6, 0},{  5, 2, 1},{ -5, 2, 1},{  1,10, 0},
				   { -1,10, 0},{  9, 1, 2},{ -9, 1, 2},{  4, 3, 1},{ -4, 3, 1}
			   },
				   //level_add
			   { 0,10, 6, 5, 4, 3, 3, 2, 2, 2, 2,-1,-1,-1,-1,-1,-1,
			   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			   2, //golomb_order
			   3, //inc_limit
			   10 //max_run
			   },
			   {
				   { //level / run
					   {  1, 1, 0},{ -1, 1, 0},{   EOB   },{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
					   { -3, 1, 0},{  1, 2, 0},{ -1, 2, 0},{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},
					   { -5, 1, 0},{  2, 2, 0},{ -2, 2, 0},{  1, 3, 0},{ -1, 3, 0},{  6, 1, 0},
					   { -6, 1, 0},{  3, 2, 0},{ -3, 2, 0},{  7, 1, 1},{ -7, 1, 1},{  1, 4, 0},
					   { -1, 4, 0},{  8, 1, 1},{ -8, 1, 1},{  2, 3, 0},{ -2, 3, 0},{  4, 2, 0},
					   { -4, 2, 0},{  1, 5, 0},{ -1, 5, 0},{  9, 1, 1},{ -9, 1, 1},{  5, 2, 0},
					   { -5, 2, 0},{  2, 4, 0},{ -2, 4, 0},{  1, 6, 0},{ -1, 6, 0},{ 10, 1, 2},
					   {-10, 1, 2},{  3, 3, 0},{ -3, 3, 0},{ 11, 1, 2},{-11, 1, 2},{  1, 7, 0},
					   { -1, 7, 0},{  6, 2, 0},{ -6, 2, 0},{  3, 4, 0},{ -3, 4, 0},{  2, 5, 0},
					   { -2, 5, 0},{ 12, 1, 2},{-12, 1, 2},{  4, 3, 0},{ -4, 3, 0}
				   },
					   //level_add
				   { 0,13, 7, 5, 4, 3, 2, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,
				   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
				   2, //golomb_order
				   6, //inc_limit
				   7  //max_run
			   },
			   {
				   { //level / run
					   {   EOB   },{  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
					   { -3, 1, 0},{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},{ -5, 1, 0},{  1, 2, 0},
					   { -1, 2, 0},{  6, 1, 0},{ -6, 1, 0},{  7, 1, 0},{ -7, 1, 0},{  8, 1, 0},
					   { -8, 1, 0},{  2, 2, 0},{ -2, 2, 0},{  9, 1, 0},{ -9, 1, 0},{  1, 3, 0},
					   { -1, 3, 0},{ 10, 1, 1},{-10, 1, 1},{  3, 2, 0},{ -3, 2, 0},{ 11, 1, 1},
					   {-11, 1, 1},{  4, 2, 0},{ -4, 2, 0},{ 12, 1, 1},{-12, 1, 1},{  1, 4, 0},
					   { -1, 4, 0},{  2, 3, 0},{ -2, 3, 0},{ 13, 1, 1},{-13, 1, 1},{  5, 2, 0},
					   { -5, 2, 0},{ 14, 1, 1},{-14, 1, 1},{  6, 2, 0},{ -6, 2, 0},{  1, 5, 0},
					   { -1, 5, 0},{ 15, 1, 1},{-15, 1, 1},{  3, 3, 0},{ -3, 3, 0},{ 16, 1, 1},
					   {-16, 1, 1},{  2, 4, 0},{ -2, 4, 0},{  7, 2, 0},{ -7, 2, 0}
				   },
					   //level_add
				   { 0,17, 8, 4, 3, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
				   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
				   2, //golomb_order
				   9, //inc_limit
				   5  //max_run
				   },
				   {
					   { //level / run
						   {   EOB   },{  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
						   { -3, 1, 0},{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},{ -5, 1, 0},{  6, 1, 0},
						   { -6, 1, 0},{  7, 1, 0},{ -7, 1, 0},{  1, 2, 0},{ -1, 2, 0},{  8, 1, 0},
						   { -8, 1, 0},{  9, 1, 0},{ -9, 1, 0},{ 10, 1, 0},{-10, 1, 0},{ 11, 1, 0},
						   {-11, 1, 0},{ 12, 1, 0},{-12, 1, 0},{  2, 2, 0},{ -2, 2, 0},{ 13, 1, 0},
						   {-13, 1, 0},{  1, 3, 0},{ -1, 3, 0},{ 14, 1, 0},{-14, 1, 0},{ 15, 1, 0},
						   {-15, 1, 0},{  3, 2, 0},{ -3, 2, 0},{ 16, 1, 0},{-16, 1, 0},{ 17, 1, 0},
						   {-17, 1, 0},{ 18, 1, 0},{-18, 1, 0},{  4, 2, 0},{ -4, 2, 0},{ 19, 1, 0},
						   {-19, 1, 0},{ 20, 1, 0},{-20, 1, 0},{  2, 3, 0},{ -2, 3, 0},{  1, 4, 0},
						   { -1, 4, 0},{  5, 2, 0},{ -5, 2, 0},{ 21, 1, 0},{-21, 1, 0}
					   },
						   //level_add
					   { 0,22, 6, 3, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
					   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
					   2, //golomb_order
					   INT_MAX, //inc_limit
					   4 //max_run
				   }
};
 
static const cavs_vlc chroma_2dvlc[5] = 
{
	   {
		   { //level / run
			   {  1, 1, 1},{ -1, 1, 1},{  1, 2, 1},{ -1, 2, 1},{  1, 3, 1},{ -1, 3, 1},
			   {  1, 4, 1},{ -1, 4, 1},{  1, 5, 1},{ -1, 5, 1},{  1, 6, 1},{ -1, 6, 1},
			   {  1, 7, 1},{ -1, 7, 1},{  2, 1, 2},{ -2, 1, 2},{  1, 8, 1},{ -1, 8, 1},
			   {  1, 9, 1},{ -1, 9, 1},{  1,10, 1},{ -1,10, 1},{  1,11, 1},{ -1,11, 1},
			   {  1,12, 1},{ -1,12, 1},{  1,13, 1},{ -1,13, 1},{  1,14, 1},{ -1,14, 1},
			   {  1,15, 1},{ -1,15, 1},{  3, 1, 3},{ -3, 1, 3},{  1,16, 1},{ -1,16, 1},
			   {  1,17, 1},{ -1,17, 1},{  1,18, 1},{ -1,18, 1},{  1,19, 1},{ -1,19, 1},
			   {  1,20, 1},{ -1,20, 1},{  1,21, 1},{ -1,21, 1},{  1,22, 1},{ -1,22, 1},
			   {  2, 2, 2},{ -2, 2, 2},{  1,23, 1},{ -1,23, 1},{  1,24, 1},{ -1,24, 1},
			   {  1,25, 1},{ -1,25, 1},{  4, 1, 3},{ -4, 1, 3},{   EOB   }
		   },
			   //level_add
		   { 0, 5, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		   2, 2, 2, 2, 2, 2, 2, 2, 2,-1},
		   2, //golomb_order
		   0, //inc_limit
		   25 //max_run
	   },
	   {
		   { //level / run
			   {   EOB   },{  1, 1, 0},{ -1, 1, 0},{  1, 2, 0},{ -1, 2, 0},{  2, 1, 1},
			   { -2, 1, 1},{  1, 3, 0},{ -1, 3, 0},{  1, 4, 0},{ -1, 4, 0},{  1, 5, 0},
			   { -1, 5, 0},{  1, 6, 0},{ -1, 6, 0},{  3, 1, 2},{ -3, 1, 2},{  1, 7, 0},
			   { -1, 7, 0},{  1, 8, 0},{ -1, 8, 0},{  2, 2, 1},{ -2, 2, 1},{  1, 9, 0},
			   { -1, 9, 0},{  1,10, 0},{ -1,10, 0},{  1,11, 0},{ -1,11, 0},{  4, 1, 2},
			   { -4, 1, 2},{  1,12, 0},{ -1,12, 0},{  1,13, 0},{ -1,13, 0},{  1,14, 0},
			   { -1,14, 0},{  2, 3, 1},{ -2, 3, 1},{  1,15, 0},{ -1,15, 0},{  2, 4, 1},
			   { -2, 4, 1},{  5, 1, 3},{ -5, 1, 3},{  3, 2, 2},{ -3, 2, 2},{  1,16, 0},
			   { -1,16, 0},{  1,17, 0},{ -1,17, 0},{  1,18, 0},{ -1,18, 0},{  2, 5, 1},
			   { -2, 5, 1},{  1,19, 0},{ -1,19, 0},{  1,20, 0},{ -1,20, 0}
		   },
			   //level_add
		   { 0, 6, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		   2, 2, 2, 2,-1,-1,-1,-1,-1,-1},
		   0, //golomb_order
		   1, //inc_limit
		   20 //max_run
		},
		{
			{ //level / run
				{  1, 1, 0},{ -1, 1, 0},{   EOB   },{  2, 1, 0},{ -2, 1, 0},{  1, 2, 0},
				{ -1, 2, 0},{  3, 1, 1},{ -3, 1, 1},{  1, 3, 0},{ -1, 3, 0},{  4, 1, 1},
				{ -4, 1, 1},{  2, 2, 0},{ -2, 2, 0},{  1, 4, 0},{ -1, 4, 0},{  5, 1, 2},
				{ -5, 1, 2},{  1, 5, 0},{ -1, 5, 0},{  3, 2, 1},{ -3, 2, 1},{  2, 3, 0},
				{ -2, 3, 0},{  1, 6, 0},{ -1, 6, 0},{  6, 1, 2},{ -6, 1, 2},{  1, 7, 0},
				{ -1, 7, 0},{  2, 4, 0},{ -2, 4, 0},{  7, 1, 2},{ -7, 1, 2},{  1, 8, 0},
				{ -1, 8, 0},{  4, 2, 1},{ -4, 2, 1},{  1, 9, 0},{ -1, 9, 0},{  3, 3, 1},
				{ -3, 3, 1},{  2, 5, 0},{ -2, 5, 0},{  2, 6, 0},{ -2, 6, 0},{  8, 1, 2},
				{ -8, 1, 2},{  1,10, 0},{ -1,10, 0},{  1,11, 0},{ -1,11, 0},{  9, 1, 2},
				{ -9, 1, 2},{  5, 2, 2},{ -5, 2, 2},{  3, 4, 1},{ -3, 4, 1},
			},
     //level_add
			{ 0,10, 6, 4, 4, 3, 3, 2, 2, 2, 2, 2,-1,-1,-1,-1,-1,
			-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			1, //golomb_order
			2, //inc_limit
			11 //max_run
		 },
		 {
			 { //level / run
				 {   EOB   },{  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
				 { -3, 1, 0},{  4, 1, 0},{ -4, 1, 0},{  1, 2, 0},{ -1, 2, 0},{  5, 1, 1},
				 { -5, 1, 1},{  2, 2, 0},{ -2, 2, 0},{  6, 1, 1},{ -6, 1, 1},{  1, 3, 0},
				 { -1, 3, 0},{  7, 1, 1},{ -7, 1, 1},{  3, 2, 0},{ -3, 2, 0},{  8, 1, 1},
				 { -8, 1, 1},{  1, 4, 0},{ -1, 4, 0},{  2, 3, 0},{ -2, 3, 0},{  9, 1, 1},
				 { -9, 1, 1},{  4, 2, 0},{ -4, 2, 0},{  1, 5, 0},{ -1, 5, 0},{ 10, 1, 1},
				 {-10, 1, 1},{  3, 3, 0},{ -3, 3, 0},{  5, 2, 1},{ -5, 2, 1},{  2, 4, 0},
				 { -2, 4, 0},{ 11, 1, 1},{-11, 1, 1},{  1, 6, 0},{ -1, 6, 0},{ 12, 1, 1},
				 {-12, 1, 1},{  1, 7, 0},{ -1, 7, 0},{  6, 2, 1},{ -6, 2, 1},{ 13, 1, 1},
				 {-13, 1, 1},{  2, 5, 0},{ -2, 5, 0},{  1, 8, 0},{ -1, 8, 0},
			 },
			 //level_add
			 { 0,14, 7, 4, 3, 3, 2, 2, 2,-1,-1,-1,-1,-1,-1,-1,-1,
			 -1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			 1, //golomb_order
			 4, //inc_limit
			 8  //max_run
		},
		{
			{ //level / run
				{   EOB   },{  1, 1, 0},{ -1, 1, 0},{  2, 1, 0},{ -2, 1, 0},{  3, 1, 0},
				{ -3, 1, 0},{  4, 1, 0},{ -4, 1, 0},{  5, 1, 0},{ -5, 1, 0},{  6, 1, 0},
				{ -6, 1, 0},{  7, 1, 0},{ -7, 1, 0},{  8, 1, 0},{ -8, 1, 0},{  1, 2, 0},
				{ -1, 2, 0},{  9, 1, 0},{ -9, 1, 0},{ 10, 1, 0},{-10, 1, 0},{ 11, 1, 0},
				{-11, 1, 0},{  2, 2, 0},{ -2, 2, 0},{ 12, 1, 0},{-12, 1, 0},{ 13, 1, 0},
				{-13, 1, 0},{  3, 2, 0},{ -3, 2, 0},{ 14, 1, 0},{-14, 1, 0},{  1, 3, 0},
				{ -1, 3, 0},{ 15, 1, 0},{-15, 1, 0},{  4, 2, 0},{ -4, 2, 0},{ 16, 1, 0},
				{-16, 1, 0},{ 17, 1, 0},{-17, 1, 0},{  5, 2, 0},{ -5, 2, 0},{  1, 4, 0},
				{ -1, 4, 0},{  2, 3, 0},{ -2, 3, 0},{ 18, 1, 0},{-18, 1, 0},{  6, 2, 0},
				{ -6, 2, 0},{ 19, 1, 0},{-19, 1, 0},{  1, 5, 0},{ -1, 5, 0},
			},
			//level_add
			{ 0,20, 7, 3, 2, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
			-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
			0, //golomb_order
			INT_MAX, //inc_limit
			5, //max_run
		}
};
#undef EOB

static const uint8_t partition_flags[30] = {
   0,                                 //I_8X8
   0,                                 //P_SKIP
   0,                                 //P_16X16
                       SPLITH,        //P_16X8
                              SPLITV, //P_8X16
                       SPLITH|SPLITV, //P_8X8
                       SPLITH|SPLITV, //B_SKIP
                       SPLITH|SPLITV, //B_DIRECT
   0,                                 //B_FWD_16X16
   0,                                 //B_BWD_16X16
   0,                                 //B_SYM_16X16
   FWD0|FWD1          |SPLITH,
   FWD0|FWD1                 |SPLITV,
   BWD0|BWD1          |SPLITH,
   BWD0|BWD1                 |SPLITV,
   FWD0|BWD1          |SPLITH,
   FWD0|BWD1                 |SPLITV,
   BWD0|FWD1          |SPLITH,
   BWD0|FWD1                 |SPLITV,
   FWD0|FWD1     |SYM1|SPLITH,
   FWD0|FWD1     |SYM1       |SPLITV,
   BWD0|FWD1     |SYM1|SPLITH,
   BWD0|FWD1     |SYM1       |SPLITV,
   FWD0|FWD1|SYM0     |SPLITH,
   FWD0|FWD1|SYM0            |SPLITV,
   FWD0|BWD1|SYM0     |SPLITH,
   FWD0|BWD1|SYM0            |SPLITV,
   FWD0|FWD1|SYM0|SYM1|SPLITH,
   FWD0|FWD1|SYM0|SYM1       |SPLITV,
                       SPLITH|SPLITV, //B_8X8 = 29
 };

static const int frame_rate_tab[][2] = {
    {    0,    0},
    {24000, 1001},
    {   24,    1},
    {   25,    1},
    {30000, 1001},
    {   30,    1},
    {   50,    1},
    {60000, 1001},
    {   60,    1},
 
    {   15,    1},
 
    {    5,    1},
    {   10,    1},
    {   12,    1},
    {   15,    1},
    {    0,    0},
};


static inline int cavs_clip(int a, int amin, int amax)
{
     if (a < amin)      return amin;
     else if (a > amax) return amax;
     else               return a;
}
static inline uint8_t cavs_clip_uint8(int a)
{
     if (a&(~255)) return (-a)>>31;
     else          return a;
}



static inline int clip3_int( int v, int i_min, int i_max )
{
    return ( (v < i_min) ? i_min : (v > i_max) ? i_max : v );
}

static inline float clip3_float( float v, float f_min, float f_max )
{
    return ( (v < f_min) ? f_min : (v > f_max) ? f_max : v );
}
static inline int median( int a, int b, int c )
{
    int min = a, max =a;
    if( b < min )
        min = b;
    else
        max = b;    

    if( c < min )
        min = c;
    else if( c > max )
        max = c;

    return a + b + c - min - max;
}


#define LOWPASS_4x4(ARRAY,INDEX)                                            \
    ( ARRAY[(INDEX)-1] + 2*ARRAY[(INDEX)] + ARRAY[(INDEX)+1] )
#define LOWPASS(ARRAY,INDEX)                                            \
    (( ARRAY[(INDEX)-1] + 2*ARRAY[(INDEX)] + ARRAY[(INDEX)+1] + 2) >> 2)

/****************************************************************************
 * * 4x4亮度块帧内预测
 ****************************************************************************/

/****************************************************************************
 * * cavs_intra_4x4_pred_vertical:帧内预测模式之垂直
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_vertical(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
	int y;
	uint64_t i_value = unaligned64(&p_top[1]);
	for(y=0;y<4;y++) 
	{
		*((uint64_t *)(p_dest+y*i_stride)) = i_value;
	}
}
/****************************************************************************
 * * cavs_intra_4x4_pred_horizontal:帧内预测模式之水平
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_horizontal(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
    int y;
    uint64_t i_value;
    for(y=0;y<4;y++) 
	{
        i_value = p_left[y+1] * (uint64_t)0x0101010101010101;
        *((uint64_t *)(p_dest+y*i_stride)) = i_value;
    }
}
/****************************************************************************
 * * cavs_intra_4x4_pred_dc_128:帧内预测模式之直流,顶块和左块不能得到的情况下处理
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_dc_128(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{

	int y;
	uint64_t i_value = 128;
	for(y=0;y<4;y++)
		*((uint64_t *)(p_dest+y*i_stride)) = i_value;
}
/****************************************************************************
 * * cavs_intra_4x4_pred_dc_lp:帧内预测模式之直流,顶块和左块都能得到的情况下处理
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_dc_lp(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
    int x,y;
	for(y=0; y<4; y++)
		for(x=0; x<4; x++)
            p_dest[y*i_stride+x] = (LOWPASS_4x4(p_top,x+1)+LOWPASS_4x4(p_left,y+1)+4) >> 3;
}

/****************************************************************************
 * * cavs_intra_4x4_pred_dc_lp_left:帧内预测模式之直流,顶块不能得到的情况下处理
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_dc_lp_left(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
    int x,y;
    for(y=0; y<4; y++)
        for(x=0; x<4; x++)
            p_dest[y*i_stride+x] = LOWPASS(p_left,y+1);
}
/****************************************************************************
 * * cavs_intra_4x4_pred_dc_lp_top:帧内预测模式之直流,左块不能得到的情况下处理
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_dc_lp_top(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
    int x,y;
    for(y=0; y<4; y++)
        for(x=0; x<4; x++)
            p_dest[y*i_stride+x] = LOWPASS(p_top,x+1);
}
/****************************************************************************
 * * cavs_intra_4x4_pred_down_left:帧内预测模式之左下 均可用
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_down_left(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
    int x,y;
    for(y=0; y<4; y++)
        for(x=0; x<4; x++)
		{
			if (x==3&&y==3)
		       p_dest[y*i_stride+x]=(p_top[7]+p_top[8]+p_left[7]+p_left[8]+2)>>2;
			else
            p_dest[y*i_stride+x] = ((LOWPASS_4x4(p_top,x+y+2) + LOWPASS_4x4(p_left,x+y+2)) +4) >> 3;
		}
} 
/****************************************************************************
 * * cavs_intra_4x4_pred_down_left:帧内预测模式之右下
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_down_right(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
    int x,y;
    for(y=0; y<4; y++)
        for(x=0; x<4; x++)
            if(x==y)
                p_dest[y*i_stride+x] = (p_left[1]+2*p_top[0]+p_top[1]+2)>>2;
            else if(x>y)
                p_dest[y*i_stride+x] = LOWPASS(p_top,x-y);
            else
                p_dest[y*i_stride+x] = LOWPASS(p_left,y-x);
}
/****************************************************************************
 * * cavs_intra_4x4_pred_vertical_left:帧内预测模式之垂直左
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_vertical_left(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
	int x,y;
	for(y=0; y<4; y++)
		for(x=0; x<4; x++)
		{
				if(y==0||y==2)
					p_dest[y*i_stride+x] = (p_top[x+y/2+1]+p_top[x+y/2+2]+1)>>1;
				else 
					p_dest[y*i_stride+x] = LOWPASS(p_top,x+y/2+2);
		}
}
/****************************************************************************
 * * cavs_intra_4x4_pred_horizontal_down:帧内预测模式之水平下
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_horizontal_down(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
	int x,y;
	for(y=0; y<4; y++)
		for(x=0; x<4; x++)
		{
			hd=2*y-x;
			if(hd==0||hd==2||hd==4||hd==6)
				p_dest[y*i_stride+x] =(p_left[y-x/2]+p_left[y-x/2+1]+1)>>1;
			else if(hd==1||hd==3||hd==5)
				p_dest[y*i_stride+x] = LOWPASS(p_left,(y-x/2));
			else if(hd==-1)
				p_dest[y*i_stride+x] =(p_top[1]+2*p_top[0]+p_left[1]+2)>>2;
			else
				p_dest[y*i_stride+x]=LOWPASS(p_top,x-1);
		}
}
/****************************************************************************
 * * cavs_intra_4x4_pred_vertical_right:帧内预测模式之垂直右
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_vertical_right(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
	int x,y;
	for(y=0; y<4; y++)
		for(x=0; x<4; x++)
		{
			vr=2*x-y;
			if(vr==0||vr==2||vr==4)
				p_dest[y*i_stride+x] =(p_top[x-y/2]+p_top[x-y/2+1]+1)>>1;
			else if(vr==1||vr==3||vr==5)
				p_dest[y*i_stride+x] = LOWPASS(p_top,(x-y/2));
			else if(vr==-1)
				p_dest[y*i_stride+x] =LOWPASS(p_left,0);
			else
				p_dest[y*i_stride+x]=LOWPASS(p_left,y-1);
		}
}
/****************************************************************************
 * * cavs_intra_4x4_pred_horizontal_up:帧内预测模式之水平上
 ****************************************************************************/
static UNUSED void cavs_intra_4x4_pred_horizontal_up(uint8_t *p_dest,uint8_t *p_top,uint8_t *p_left,int i_stride)
{
	int x,y;
	for(y=0; y<4; y++)
		for(x=0; x<4; x++)
		{
			hu=x+2*y;
			if(hu==0||hu==2||hu==4)
				p_dest[y*i_stride+x] =(p_left[y+x/2+1]+p_left[y+x/2+2]+1)>>1;
			else if(hu==1||hu==3)
				p_dest[y*i_stride+x] = LOWPASS(p_left,(y+x/2+2));
			else if(hd==5)
				p_dest[y*i_stride+x] =(p_left[3]+3*p_left[4]+2)>>2;
			else
				p_dest[y*i_stride+x]=p_left[4];
		}
}

/****************************************************************************
 * * luma 8x8 intra
 ****************************************************************************/
/****************************************************************************
 * * cavs_intra_pred_dc_128:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_dc_128(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left )
{

	int y;
	uint64_t i_value = 0x8080808080808080;
	for(y=0;y<8;y++)
		*((uint64_t *)(p_dest+y*i_stride)) = i_value;
}
/****************************************************************************
 * * cavs_intra_pred_dc_lp:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_dc_lp(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y;
    for(y=0; y<8; y++)
        for(x=0; x<8; x++)
            p_dest[y*i_stride+x] = (LOWPASS(p_top,x+1) + LOWPASS(p_left,y+1)) >> 1;
}

/****************************************************************************
 * * cavs_intra_pred_dc_lp_left:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_dc_lp_left(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y;
    for(y=0; y<8; y++)
        for(x=0; x<8; x++)
            p_dest[y*i_stride+x] = LOWPASS(p_left,y+1);
}
/****************************************************************************
 * * cavs_intra_pred_dc_lp_top:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_dc_lp_top(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y;
    for(y=0; y<8; y++)
        for(x=0; x<8; x++)
            p_dest[y*i_stride+x] = LOWPASS(p_top,x+1);
}
/****************************************************************************
 * * cavs_intra_pred_horizontal:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_horizontal(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    uint64_t i_value;
    for(int y=0;y<8;y++) 
    {	
        i_value = p_left[y+1] * 0x0101010101010101ULL;
        *((uint64_t *)(p_dest+y*i_stride)) = i_value;
    }
}
/****************************************************************************
 * * cavs_intra_pred_vertical:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_vertical(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
	uint64_t i_value = unaligned64(&p_top[1]);
	for(int y=0;y<8;y++) 
	{
		*((uint64_t *)(p_dest+y*i_stride)) = i_value;
	}
}
/****************************************************************************
 * * cavs_intra_pred_down_left:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_down_left(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    for(int y=0; y<8; y++)
        for(int x=0; x<8; x++)
            p_dest[y*i_stride+x] = (LOWPASS(p_top,x+y+2) + LOWPASS(p_left,x+y+2)) >> 1;
}
/****************************************************************************
 * * cavs_intra_pred_down_left:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_down_right(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    for(int y=0; y<8; y++)
        for(int x=0; x<8; x++)
            if(x==y)
                p_dest[y*i_stride+x] = (p_left[1]+2*p_top[0]+p_top[1]+2)>>2;
            else if(x>y)
                p_dest[y*i_stride+x] = LOWPASS(p_top,x-y);
            else
                p_dest[y*i_stride+x] = LOWPASS(p_left,y-x);
}

/****************************************************************************
 * * cavs_intra_pred_plane:
 ****************************************************************************/
static UNUSED void cavs_intra_pred_plane(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y,i_value;
    int i_h = 0;
    int i_v = 0;
    uint8_t *p_crop = crop_table + MAX_NEG_CROP;

    for(x=0; x<4; x++)
    {
        i_h += (x+1)*(p_top[5+x]-p_top[3-x]);
        i_v += (x+1)*(p_left[5+x]-p_left[3-x]);
    }
    i_value = (p_top[8]+p_left[8])<<4;
    i_h = (17*i_h+16)>>5;
    i_v = (17*i_v+16)>>5;
    for(y=0; y<8; y++)
    {
        for(x=0; x<8; x++)
        {
            p_dest[y*i_stride+x] = p_crop[(i_value+(x-3)*i_h+(y-3)*i_v+16)>>5];
        }
    }
}



/* intra chroma predict */
static UNUSED void cavs_intra_pred_chroma_dc_128(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left )
{

	int y;
	uint64_t i_value = 0x8080808080808080;
	for(y=0;y<8;y++)
		*((uint64_t *)(p_dest+y*i_stride)) = i_value;
}

static UNUSED void cavs_intra_pred_chroma_dc_lp(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y;
    for(y=0; y<8; y++)
        for(x=0; x<8; x++)
            p_dest[y*i_stride+x] = (LOWPASS(p_top,x+1) + LOWPASS(p_left,y+1)) >> 1;
}

static UNUSED void cavs_intra_pred_chroma_dc_lp_left(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y;
    for(y=0; y<8; y++)
        for(x=0; x<8; x++)
            p_dest[y*i_stride+x] = LOWPASS(p_left,y+1);
}

static UNUSED void cavs_intra_pred_chroma_dc_lp_top(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y;
    for(y=0; y<8; y++)
        for(x=0; x<8; x++)
            p_dest[y*i_stride+x] = LOWPASS(p_top,x+1);
}

static UNUSED void cavs_intra_pred_chroma_horizontal(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int y;
    uint64_t i_value;
    for(y=0;y<8;y++) 
    {	
        i_value = p_left[y+1] * 0x0101010101010101ULL;
        *((uint64_t *)(p_dest+y*i_stride)) = i_value;
    }
}

static UNUSED void cavs_intra_pred_chroma_vertical(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left)
{
	int y;
	uint64_t i_value = unaligned64(&p_top[1]);
	for(y=0;y<8;y++) 
	{
		*((uint64_t *)(p_dest+y*i_stride)) = i_value;
	}
}

static UNUSED void cavs_intra_pred_chroma_plane(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left)
{
    int x,y,i_value;
    int i_h = 0;
    int i_v = 0;
    uint8_t *p_crop = crop_table + MAX_NEG_CROP;

    for(x=0; x<4; x++)
    {
        i_h += (x+1)*(p_top[5+x]-p_top[3-x]);
        i_v += (x+1)*(p_left[5+x]-p_left[3-x]);
    }
    i_value = (p_top[8]+p_left[8])<<4;
    i_h = (17*i_h+16)>>5;
    i_v = (17*i_v+16)>>5;
    for(y=0; y<8; y++)
    {
        for(x=0; x<8; x++)
        {
            p_dest[y*i_stride+x] = p_crop[(i_value+(x-3)*i_h+(y-3)*i_v+16)>>5];
        }
    }
}

#undef LOWPASS




/****************************************************************************
 * 4x4 prediction for intra luma block
 ****************************************************************************/
#define F1(a,b)			(((a)+(b)+1)>>1)
#define F2(a,b,c)		(((a)+((b)<<1)+(c)+2)>>2)
#define F2_MID(a,b,c)	((a)+((b)<<1)+(c))
#define F4(a1,b1,c1,a2,b2,c2)	(((a1)+(c1)+(a2)+(c2)+((b1+b2)<<1)+4)>>3)

#define SRC(x,y) src[(x) + (y)*i_stride]
#define SRC_X4(x,y) MPIXEL_X4( &SRC(x,y) )
#define PREDICT_4x4_DC(v)\
	SRC_X4(0,0) = SRC_X4(0,1) = SRC_X4(0,2) = SRC_X4(0,3) = v;

#define EP (edge+8)

static UNUSED void
predict_4x4_dc_128(uint8_t * src, uint8_t edge[17], int i_stride)
{
	PREDICT_4x4_DC(0x80808080);
}

static UNUSED void
predict_4x4_dc_top(uint8_t *src, uint8_t edge[17], int i_stride)
{
	uint32_t top_edge[4];
	top_edge[0] = F2(EP[0],EP[1],EP[2]);
	top_edge[1] = F2(EP[1],EP[2],EP[3]);
	top_edge[2] = F2(EP[2],EP[3],EP[4]);
	top_edge[3] = F2(EP[3],EP[4],EP[5]);
	PREDICT_4x4_DC(MPIXEL_X4(&top_edge[0]));
}
static UNUSED void
predict_4x4_dc_left(uint8_t *src, uint8_t edge[17], int i_stride)
{
	SRC_X4(0, 0) = PIXEL_SPLAT_X4(F2(EP[0],EP[-1],EP[-2]));
	SRC_X4(0, 1) = PIXEL_SPLAT_X4(F2(EP[-1],EP[-2],EP[-3]));
	SRC_X4(0, 2) = PIXEL_SPLAT_X4(F2(EP[-2],EP[-3],EP[-4]));
	SRC_X4(0, 3) = PIXEL_SPLAT_X4(F2(EP[-3],EP[-4],EP[-5]));
}

static UNUSED void
predict_4x4_dc(uint8_t *src, uint8_t edge[17], int i_stride)
{
	int t0 = F2_MID(EP[0], EP[1], EP[2]);
	int t1 = F2_MID(EP[1], EP[2], EP[3]);
	int t2 = F2_MID(EP[2], EP[3], EP[4]);
	int t3 = F2_MID(EP[3], EP[4], EP[5]);
	int l0 = F2_MID(EP[0], EP[-1], EP[-2]);
	int l1 = F2_MID(EP[-1], EP[-2], EP[-3]);
	int l2 = F2_MID(EP[-2], EP[-3], EP[-4]);
	int l3 = F2_MID(EP[-3], EP[-4], EP[-5]);
	src[0] = (l0+t0+4)>>3;
	src[1] = (l0+t1+4)>>3;
	src[2] = (l0+t2+4)>>3;
	src[3] = (l0+t3+4)>>3;
	src += i_stride;
	src[0] = (l1+t0+4)>>3;
	src[1] = (l1+t1+4)>>3;
	src[2] = (l1+t2+4)>>3;
	src[3] = (l1+t3+4)>>3;
	src += i_stride;
	src[0] = (l2+t0+4)>>3;
	src[1] = (l2+t1+4)>>3;
	src[2] = (l2+t2+4)>>3;
	src[3] = (l2+t3+4)>>3;
	src += i_stride;
	src[0] = (l3+t0+4)>>3;
	src[1] = (l3+t1+4)>>3;
	src[2] = (l3+t2+4)>>3;
	src[3] = (l3+t3+4)>>3;
}

static UNUSED void predict_4x4_h( pixel *src, uint8_t edge[17], int i_stride)
{
	SRC_X4(0,0) = PIXEL_SPLAT_X4( EP[-1] );
	SRC_X4(0,1) = PIXEL_SPLAT_X4( EP[-2] );
	SRC_X4(0,2) = PIXEL_SPLAT_X4( EP[-3] );
	SRC_X4(0,3) = PIXEL_SPLAT_X4( EP[-4] );
}
static UNUSED void predict_4x4_v( pixel *src, uint8_t edge[17], int i_stride)
{
	PREDICT_4x4_DC(MPIXEL_X4(&EP[1]));
}

static UNUSED void predict_4x4_ddl( pixel *src, uint8_t edge[17], int i_stride)
{
	SRC(0,0)= F4(EP[1],EP[2],EP[3],EP[-1],EP[-2],EP[-3]);
	SRC(1,0)=SRC(0,1)= F4(EP[2],EP[3],EP[4],EP[-2],EP[-3],EP[-4]);
	SRC(2,0)=SRC(1,1)=SRC(0,2)= F4(EP[3],EP[4],EP[5],EP[-3],EP[-4],EP[-5]);
	SRC(3,0)=SRC(2,1)=SRC(1,2)=SRC(0,3)= F4(EP[4],EP[5],EP[6],EP[-4],EP[-5],EP[-6]);
	SRC(3,1)=SRC(2,2)=SRC(1,3)= F4(EP[5],EP[6],EP[7],EP[-5],EP[-6],EP[-7]);
	SRC(3,2)=SRC(2,3)= F4(EP[6],EP[7],EP[8],EP[-6],EP[-7],EP[-8]);
	SRC(3,3)= (EP[7]+EP[8]+EP[-7]+EP[-8]+2)>>2;
}
static UNUSED void predict_4x4_ddr( pixel *src, uint8_t edge[17], int i_stride)
{
	SRC(3,0)= F2(EP[4],EP[3],EP[2]);
	SRC(2,0)=SRC(3,1)= F2(EP[3],EP[2],EP[1]);
	SRC(1,0)=SRC(2,1)=SRC(3,2)= F2(EP[2],EP[1],EP[0]);
	SRC(0,0)=SRC(1,1)=SRC(2,2)=SRC(3,3)= F2(EP[1],EP[0],EP[-1]);
	SRC(0,1)=SRC(1,2)=SRC(2,3)= F2(EP[0],EP[-1],EP[-2]);
	SRC(0,2)=SRC(1,3)= F2(EP[-1],EP[-2],EP[-3]);
	SRC(0,3)= F2(EP[-2],EP[-3],EP[-4]);
}

static UNUSED void predict_4x4_vr( pixel *src, uint8_t edge[17], int i_stride )
{
	SRC(0,3)= F2(EP[-3],EP[-2],EP[-1]);
	SRC(0,2)= F2(EP[-2],EP[-1],EP[0]);
	SRC(0,1)=SRC(1,3)= F2(EP[-1],EP[0],EP[1]);
	SRC(0,0)=SRC(1,2)= F1(EP[0],EP[1]);
	SRC(1,1)=SRC(2,3)= F2(EP[0],EP[1],EP[2]);
	SRC(1,0)=SRC(2,2)= F1(EP[1],EP[2]);
	SRC(2,1)=SRC(3,3)= F2(EP[1],EP[2],EP[3]);
	SRC(2,0)=SRC(3,2)= F1(EP[2],EP[3]);
	SRC(3,1)= F2(EP[2],EP[3],EP[4]);
	SRC(3,0)= F1(EP[3],EP[4]);
}

static UNUSED void predict_4x4_hd( pixel *src, uint8_t edge[17], int i_stride)
{
	SRC(0,3)= F1(EP[-3],EP[-4]);
	SRC(1,3)= F2(EP[-2],EP[-3],EP[-4]);
	SRC(0,2)=SRC(2,3)= F1(EP[-2],EP[-3]);
	SRC(1,2)=SRC(3,3)= F2(EP[-1],EP[-2],EP[-3]);
	SRC(0,1)=SRC(2,2)= F1(EP[-1],EP[-2]);
	SRC(1,1)=SRC(3,2)= F2(EP[0],EP[-1],EP[-2]);
	SRC(0,0)=SRC(2,1)= F1(EP[0],EP[-1]);
	SRC(1,0)=SRC(3,1)= F2(EP[1],EP[0],EP[-1]);
	SRC(2,0)= F2(EP[2],EP[1],EP[0]);
	SRC(3,0)= F2(EP[3],EP[2],EP[1]);
}

static UNUSED void predict_4x4_vl( pixel *src, uint8_t edge[17], int i_stride)
{
	SRC(0,0)= F1(EP[1],EP[2]);
	SRC(0,1)= F2(EP[1],EP[2],EP[3]);
	SRC(1,0)=SRC(0,2)= F1(EP[2],EP[3]);
	SRC(1,1)=SRC(0,3)= F2(EP[2],EP[3],EP[4]);
	SRC(2,0)=SRC(1,2)= F1(EP[3],EP[4]);
	SRC(2,1)=SRC(1,3)= F2(EP[3],EP[4],EP[5]);
	SRC(3,0)=SRC(2,2)= F1(EP[4],EP[5]);
	SRC(3,1)=SRC(2,3)= F2(EP[4],EP[5],EP[6]);
	SRC(3,2)= F1(EP[5],EP[6]);
	SRC(3,3)= F2(EP[5],EP[6],EP[7]);
}

static UNUSED void predict_4x4_hu( pixel *src, uint8_t edge[17], int i_stride)
{
	SRC(0,0)= F1(EP[-1],EP[-2]);
	SRC(1,0)= F2(EP[-1],EP[-2],EP[-3]);
	SRC(2,0)=SRC(0,1)= F1(EP[-2],EP[-3]);
	SRC(3,0)=SRC(1,1)= F2(EP[-2],EP[-3],EP[-4]);
	SRC(2,1)=SRC(0,2)= F1(EP[-3],EP[-4]);
	SRC(3,1)=SRC(1,2)= F2(EP[-3],EP[-4],EP[-4]);
	SRC(3,2)=SRC(1,3)=SRC(0,3)=	
		SRC(2,2)=SRC(2,3)=SRC(3,3)= EP[-4];
}
#undef SRC
#undef EP

static UNUSED void inv_transform_B4(int curr_blk1[B4_SIZE][B4_SIZE])//xuyan1118 short->int
{
	short int xx,yy;
	short int tmp[B4_SIZE];
	short int t;
	short int curr_blk[B4_SIZE][B4_SIZE];

	int c = 3;

	for(yy=0; yy<B4_SIZE; yy++)
		for(xx=0; xx<B4_SIZE; xx++)
		{
			curr_blk[yy][xx] = (short int)curr_blk1[yy][xx];
		}
		
		for(yy=0; yy<B4_SIZE; yy++)
		{
			tmp[0] = (curr_blk[yy][0] + curr_blk[yy][2]) << 1; 
			tmp[1] = (curr_blk[yy][0] - curr_blk[yy][2]) << 1;    
			tmp[2] = curr_blk[yy][1] - (curr_blk[yy][3]*3);		
			tmp[3] = (curr_blk[yy][1]*3) + curr_blk[yy][3]; 

			for (xx=0; xx<2; xx++)
			{
				t=3-xx;
				curr_blk[yy][xx] = tmp[xx] + tmp[t];
				curr_blk[yy][t]= tmp[xx] - tmp[t];
			}
		}
		//vertical
		for(xx=0; xx<B4_SIZE; xx++)
		{
			tmp[0]= (curr_blk[0][xx] + curr_blk[2][xx]) << 1;
			tmp[1]= (curr_blk[0][xx] - curr_blk[2][xx]) << 1;
			tmp[2]= curr_blk[1][xx] - (curr_blk[3][xx]*c);		
			tmp[3]= (curr_blk[1][xx]*c) + curr_blk[3][xx];
			for( yy=0; yy<2; yy++)
			{
				t=3-yy;
				curr_blk[yy][xx] =(tmp[yy]+tmp[t]+16) >> 5;
				curr_blk[t][xx]=(tmp[yy]-tmp[t]+16) >> 5;
			}
		}

		for(xx=0;xx<B4_SIZE;xx++)
			for(yy=0;yy<B4_SIZE;yy++)
				curr_blk1[yy][xx] = curr_blk[yy][xx];
}


#define IDCT4_Horizontal_1D {\
	int a0 = (SRC(0) + SRC(2))<<1;\
	int a1 = (SRC(0) - SRC(2))<<1;\
	int a2 = SRC(1) - (SRC(3)*3);\
	int a3 = (SRC(1)*3) + SRC(3);\
	DST(0, a0+a3);\
	DST(3, a0-a3);\
	DST(1, a1+a2);\
	DST(2, a1-a2);\
}

#define IDCT4_Vertical_1D {\
	int a0 = (SRC(0) + SRC(2))<<1;\
	int a1 = (SRC(0) - SRC(2))<<1;\
	int a2 = SRC(1) - (SRC(3)*3);\
	int a3 = (SRC(1)*3) + SRC(3);\
	DST(0, (a0+a3+16)>>5);\
	DST(3, (a0-a3+16)>>5);\
	DST(1, (a1+a2+16)>>5);\
	DST(2, (a1-a2+16)>>5);\
}

static void add4x4_idct(uint8_t *dst, DCTELEM *dct/*[4][4]*/, int i_dst_stride)
{
	int i;
#define SRC(x)     dct[i*8 + x]
#define DST(x,rhs) dct[i*8 + x] = (rhs)
	for (i = 0; i < 4; i++)
		IDCT4_Horizontal_1D
#undef SRC
#undef DST
#define SRC(x)     dct[x*8 + i]
#define DST(x,rhs) dst[x*i_dst_stride + i] = cavs_clip_uint8( dst[x*i_dst_stride + i] + (rhs) );
		for (i = 0; i < 4; i++)
			IDCT4_Vertical_1D
#undef SRC
#undef DST
}

static UNUSED void cavs_idct4_add_c(uint8_t *dst, DCTELEM *block, int stride)
{
    int i;

    for( i = 0; i < 4; i++ )
    {
        add4x4_idct(dst + (i>>1)*stride*4 + (i&1)*4, block + (i>>1)*8*4 + (i&1)*4, stride);
    }
    memset(block,0,64*sizeof(DCTELEM));
}

#define Clip3(min,max,val) (((val)<(min))?(min):(((val)>(max))?(max):(val)))

static UNUSED void cavs_idct8_add_c(uint8_t *dst, DCTELEM *block, int stride)
{
	int i;
	DCTELEM (*src)[8] = (DCTELEM(*)[8])block;
	uint8_t *cm = crop_table + MAX_NEG_CROP;

	int clip1, clip2; 
    
	clip1=0-(1<<(8+7));
    clip2=(1<<(8+7))-1;

	for( i = 0; i < 8; i++ ) {
		const int a0 =  3*src[i][1] - (src[i][7]<<1);
		const int a1 =  3*src[i][3] + (src[i][5]<<1);
		const int a2 =  (src[i][3]<<1) - 3*src[i][5];
		const int a3 =  (src[i][1]<<1) + 3*src[i][7];

		const int b4 = ((a0 + a1 + a3)<<1) + a1;
		const int b5 = ((a0 - a1 + a2)<<1) + a0;
		const int b6 = ((a3 - a2 - a1)<<1) + a3;
		const int b7 = ((a0 - a2 - a3)<<1) - a2;

		const int a7 = (src[i][2]<<2) - 10*src[i][6];
		const int a6 = (src[i][6]<<2) + 10*src[i][2];
		const int a5 = ((src[i][0] - src[i][4]) << 3) /*+ 4*/;
		const int a4 = ((src[i][0] + src[i][4]) << 3)/* + 4*/;

		const int b0 = a4 + a6;
		const int b1 = a5 + a7;
		const int b2 = a5 - a7;
		const int b3 = a4 - a6;

         src[i][0]=((Clip3(clip1,clip2,((b0+b4)+(1<<2))))>>3); //dailiang 10bits
         src[i][1]=((Clip3(clip1,clip2,((b1+b5)+(1<<2))))>>3);
         src[i][2]=((Clip3(clip1,clip2,((b2+b6)+(1<<2))))>>3);
         src[i][3]=((Clip3(clip1,clip2,((b3+b7)+(1<<2))))>>3);
         src[i][7]=((Clip3(clip1,clip2,((b0-b4)+(1<<2))))>>3);
         src[i][6]=((Clip3(clip1,clip2,((b1-b5)+(1<<2))))>>3);
         src[i][5]=((Clip3(clip1,clip2,((b2-b6)+(1<<2))))>>3);
         src[i][4]=((Clip3(clip1,clip2,((b3-b7)+(1<<2))))>>3);
     }
	for( i = 0; i < 8; i++ ) {
		const int a0 =  3*src[1][i] - (src[7][i]<<1);
		const int a1 =  3*src[3][i] + (src[5][i]<<1);
		const int a2 =  (src[3][i]<<1) - 3*src[5][i];
		const int a3 =  (src[1][i]<<1) + 3*src[7][i];

		const int b4 = ((a0 + a1 + a3)<<1) + a1;
		const int b5 = ((a0 - a1 + a2)<<1) + a0;
		const int b6 = ((a3 - a2 - a1)<<1) + a3;
		const int b7 = ((a0 - a2 - a3)<<1) - a2;

		const int a7 = (src[2][i]<<2) - 10*src[6][i];
		const int a6 = (src[6][i]<<2) + 10*src[2][i];
		const int a5 = (src[0][i] - src[4][i]) << 3;
		const int a4 = (src[0][i] + src[4][i]) << 3;

		const int b0 = a4 + a6;
		const int b1 = a5 + a7;
		const int b2 = a5 - a7;
		const int b3 = a4 - a6;

		dst[i + 0*stride] = cm[ dst[i + 0*stride] + ((Clip3(clip1,clip2,(b0+b4)+64))>>7)];
		dst[i + 1*stride] = cm[ dst[i + 1*stride] + ((Clip3(clip1,clip2,(b1+b5)+64))>>7)];
		dst[i + 2*stride] = cm[ dst[i + 2*stride] + ((Clip3(clip1,clip2,(b2+b6)+64))>>7)];
		dst[i + 3*stride] = cm[ dst[i + 3*stride] + ((Clip3(clip1,clip2,(b3+b7)+64))>>7)];
		dst[i + 4*stride] = cm[ dst[i + 4*stride] + ((Clip3(clip1,clip2,(b3-b7)+64))>>7)];
		dst[i + 5*stride] = cm[ dst[i + 5*stride] + ((Clip3(clip1,clip2,(b2-b6)+64))>>7)];
		dst[i + 6*stride] = cm[ dst[i + 6*stride] + ((Clip3(clip1,clip2,(b1-b5)+64))>>7)];
		dst[i + 7*stride] = cm[ dst[i + 7*stride] + ((Clip3(clip1,clip2,(b0-b4)+64))>>7)];
	}
     memset(block,0,64*sizeof(DCTELEM));
}


#define P2 p0_p[-3*stride]
#define P1 p0_p[-2*stride]
#define P0 p0_p[-1*stride]
#define Q0 p0_p[ 0*stride]
#define Q1 p0_p[ 1*stride]
#define Q2 p0_p[ 2*stride]

static inline void loop_filter_l2(uint8_t *p0_p,int stride,int alpha, int beta) 
{
    int p0 = P0;
    int q0 = Q0;
    
    if(abs(p0-q0)<alpha && abs(P1-p0)<beta && abs(Q1-q0)<beta) 
    {
        int s = p0 + q0 + 2;

        alpha = (alpha>>2) + 2;
        if(abs(P2-p0) < beta && abs(p0-q0) < alpha) 
        {
            P0 = (P1 + p0 + s) >> 2;
            P1 = (2*P1 + s) >> 2;
        } 
        else
        {
            P0 = (2*P1 + s) >> 2;
        }
        if(abs(Q2-q0) < beta && abs(q0-p0) < alpha) 
        {
            Q0 = (Q1 + q0 + s) >> 2;
            Q1 = (2*Q1 + s) >> 2;
        }
        else
        {
            Q0 = (2*Q1 + s) >> 2;
        }
    }
}
 
static inline void loop_filter_l1(uint8_t *p0_p, int stride, int alpha, int beta, int tc) 
{
    int p0 = P0;
    int q0 = Q0;

    if(abs(p0-q0)<alpha && abs(P1-p0)<beta && abs(Q1-q0)<beta) 
    {
        int delta = cavs_clip(((q0-p0)*3+P1-Q1+4)>>3,-tc, tc);
        P0 = cavs_clip_uint8(p0+delta);
        Q0 = cavs_clip_uint8(q0-delta);
        
        if(abs(P2-p0)<beta) 
        {
            delta = cavs_clip(((P0-P1)*3+P2-Q0+4)>>3, -tc, tc);
            P1 = cavs_clip_uint8(P1+delta);
        }
        if(abs(Q2-q0)<beta) 
        {
            delta = cavs_clip(((Q1-Q0)*3+P0-Q2+4)>>3, -tc, tc);
            Q1 = cavs_clip_uint8(Q1-delta);
        }
    }
}
 
static inline void loop_filter_c2(uint8_t *p0_p,int stride,int alpha, int beta) 
{
    int p0 = P0;
    int q0 = Q0;

    if(abs(p0-q0)<alpha && abs(P1-p0)<beta && abs(Q1-q0)<beta) 
    {
        int s = p0 + q0 + 2;
        alpha = (alpha>>2) + 2;
        
        if(abs(P2-p0) < beta && abs(p0-q0) < alpha) 
        {
            P0 = (P1 + p0 + s) >> 2;
        }
        else
        {
            P0 = (2*P1 + s) >> 2;
        }
        
        if(abs(Q2-q0) < beta && abs(q0-p0) < alpha) 
        {
            Q0 = (Q1 + q0 + s) >> 2;
        }
        else
        {
            Q0 = (2*Q1 + s) >> 2;
        }
    }
}

static inline void loop_filter_c1(uint8_t *p0_p,int stride,int alpha, int beta,int tc) 
{
    if(abs(P0-Q0)<alpha && abs(P1-P0)<beta && abs(Q1-Q0)<beta) 
    {
        int delta = cavs_clip(((Q0-P0)*3+P1-Q1+4)>>3, -tc, tc);
        
        P0 = cavs_clip_uint8(P0+delta);
        Q0 = cavs_clip_uint8(Q0-delta);
    }
}

#undef P0
#undef P1
#undef P2
#undef Q0
#undef Q1
#undef Q2

void cavs_qpel16_put_mc10_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc20_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc30_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc01_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc11_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc21_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc31_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc02_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc12_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc22_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc32_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc03_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc13_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc23_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_put_mc33_c(uint8_t *dst, uint8_t *src, int stride);

void cavs_qpel8_put_mc10_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc20_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc30_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc01_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc11_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc21_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc31_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc02_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc12_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc22_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc32_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc03_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc13_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc23_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_put_mc33_c(uint8_t *dst, uint8_t *src, int stride);

void cavs_qpel8_put_mc00_c(uint8_t *dst, uint8_t *src, int stride);  
void cavs_qpel16_put_mc00_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc00_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc00_c(uint8_t *dst, uint8_t *src, int stride);

void cavs_qpel16_avg_mc10_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc20_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc30_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc01_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc11_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc21_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc31_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc02_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc12_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc22_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc32_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc03_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc13_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc23_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel16_avg_mc33_c(uint8_t *dst, uint8_t *src, int stride);

void cavs_qpel8_avg_mc10_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc20_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc30_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc01_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc11_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc21_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc31_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc02_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc12_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc22_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc32_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc03_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc13_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc23_c(uint8_t *dst, uint8_t *src, int stride);
void cavs_qpel8_avg_mc33_c(uint8_t *dst, uint8_t *src, int stride);

//////////////////////////////////////////////////////////////////////////////////////////////////
int
cavs_cpu_num_processors (void)
{
#if SYS_WINDOWS
  return pthread_num_processors_np ();

#elif SYS_LINUX
  unsigned int bit;
  int np;
  cpu_set_t p_aff;
  memset (&p_aff, 0, sizeof (p_aff));
  sched_getaffinity (0, sizeof (p_aff), &p_aff);
  for (np = 0, bit = 0; bit < sizeof (p_aff); bit++)
    np += (((unsigned char *) & p_aff)[bit / 8] >> (bit % 8)) & 1;
  return np;

#else
  return 1;
#endif
}

////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    void *(*func)(void *);
    void *arg;
    void *ret;
} cavs_threadpool_job_t;

struct cavs_threadpool_t
{
    int            exit;
    int            threads;
    cavs_pthread_t *thread_handle;
    void           (*init_func)(void *);
    void           *init_arg;

    /* requires a synchronized list structure and associated methods,
       so use what is already implemented for frames */
    cavs_synch_frame_list_t uninit; /* list of jobs that are awaiting use */
    cavs_synch_frame_list_t run;    /* list of jobs that are queued for processing by the pool */
    cavs_synch_frame_list_t done;   /* list of jobs that have finished processing */
};

static void cavs_threadpool_thread( cavs_threadpool_t *pool )
{
    if( pool->init_func )
        pool->init_func( pool->init_arg );

    while( !pool->exit )
    {
        cavs_threadpool_job_t *job = NULL;
        cavs_pthread_mutex_lock( &pool->run.mutex );
        while( !pool->exit && !pool->run.i_size )
            cavs_pthread_cond_wait( &pool->run.cv_fill, &pool->run.mutex );
        if( pool->run.i_size )
        {
            job = (void*)cavs_frame_get( pool->run.list );
            pool->run.i_size--;
        }
        cavs_pthread_mutex_unlock( &pool->run.mutex );
        if( !job )
            continue;
        job->ret = job->func( job->arg ); /* execute the function */
        cavs_synch_frame_list_push( &pool->done, (void*)job );
    }
}

int cavs_threadpool_init( cavs_threadpool_t **p_pool, int threads,
						  void (*init_func)(void *), void *init_arg)
{
    int i;
    cavs_threadpool_t *pool = NULL;
    if( threads <= 0 )
        return -1;

    CHECKED_MALLOCZERO( pool, sizeof(cavs_threadpool_t) );
    *p_pool = pool;

    pool->init_func = init_func;
    pool->init_arg  = init_arg;
    pool->threads   = CAVS_MIN( threads, CAVS_THREAD_MAX );

    CHECKED_MALLOC( pool->thread_handle, pool->threads * sizeof(cavs_pthread_t) );

    if( cavs_synch_frame_list_init( &pool->uninit, pool->threads ) ||
        cavs_synch_frame_list_init( &pool->run, pool->threads ) ||
        cavs_synch_frame_list_init( &pool->done, pool->threads ) )
        goto fail;

    for( i = 0; i < pool->threads; i++ )
    {
		cavs_decoder *job = NULL;
        CHECKED_MALLOC( job, sizeof(cavs_decoder) );
        cavs_synch_frame_list_push( &pool->uninit, (void*)job );
    }
    for( i = 0; i < pool->threads; i++ )
        if( cavs_pthread_create( &pool->thread_handle[i], NULL, (void*)cavs_threadpool_thread, pool ) )
            goto fail;

    return 0;
fail:
    return -1;
}

void cavs_threadpool_run( cavs_threadpool_t *pool, void *(*func)(void *), void *arg )
{
	cavs_threadpool_job_t *job = (void*)cavs_synch_frame_list_pop( &pool->uninit );
	job->func = func;
	job->arg  = arg;
	cavs_synch_frame_list_push( &pool->run, (void*)job );
}

void *cavs_threadpool_wait( cavs_threadpool_t *pool, void *arg )
{
    cavs_threadpool_job_t *job = NULL;
    int i;
    void *ret;

    cavs_pthread_mutex_lock( &pool->done.mutex );
    while( !job )
    {
        for( i = 0; i < pool->done.i_size; i++ )
        {
            cavs_threadpool_job_t *t = (void*)pool->done.list[i];
            if( t->arg == arg )
            {
                job = (void*)cavs_frame_get( pool->done.list+i );
                pool->done.i_size--;
            }
        }
        if( !job )
            cavs_pthread_cond_wait( &pool->done.cv_fill, &pool->done.mutex );
    }
    cavs_pthread_mutex_unlock( &pool->done.mutex );

    ret = job->ret;
    cavs_synch_frame_list_push( &pool->uninit, (void*)job );
    return ret;
}

static void cavs_threadpool_list_delete( cavs_synch_frame_list_t *slist )
{
    int i;
    for( i = 0; slist->list[i]; i++ )
    {
        cavs_free( slist->list[i] );
        slist->list[i] = NULL;
    }
    cavs_synch_frame_list_delete( slist );
}

void cavs_threadpool_delete( cavs_threadpool_t *pool )
{
    int i;
	if (!pool)
		return;
    cavs_pthread_mutex_lock( &pool->run.mutex );
    pool->exit = 1;
    cavs_pthread_cond_broadcast( &pool->run.cv_fill );
    cavs_pthread_mutex_unlock( &pool->run.mutex );
    for( i = 0; i < pool->threads; i++ )
        cavs_pthread_join( pool->thread_handle[i], NULL );

    cavs_threadpool_list_delete( &pool->uninit );
    cavs_threadpool_list_delete( &pool->run );
    cavs_threadpool_list_delete( &pool->done );
    cavs_free( pool->thread_handle );
    cavs_free( pool );
}
////////////////////////////////////////////////////////////////////////////////
int cavs_init_bitstream( InputStream *p , unsigned char *rawstream, unsigned int i_len )
{
    p->f = rawstream;
    p->f_end = rawstream + i_len;
    p->uClearBits			= 0xffffffff;
    p->iBytePosition		= 0;
    p->iBufBytesNum		= 0;
    p->iClearBitsNum		= 0;
    p->iStuffBitsNum		= 0;
    p->iBitsCount			= 0;
    p->uPre3Bytes			= 0;

    return 0;
}

static int ClearNextByte(InputStream *p)
{
	int i = 0, k = 0;
	unsigned char temp[3];
	i = p->iBytePosition;
	k = p->iBufBytesNum - i;
	if(k < 3)
	{
		int last_len = 0;

		for(int j=0;j<k;j++) 
                 temp[j] = p->buf[i+j];

		if( p->f < p->f_end )
			 {
				last_len = (int)(p->f_end - p->f);
				p->iBufBytesNum = MIN2(SVA_STREAM_BUF_SIZE - k, last_len);
				memcpy( p->buf+k, p->f, p->iBufBytesNum );
				p->f += p->iBufBytesNum;
			 }
			 else
				p->iBufBytesNum = 0;

		if(p->iBufBytesNum == 0)
		{
			if(k>0)
			{
                while(k>0)
                {
				    p->uPre3Bytes = ((p->uPre3Bytes<<8) | p->buf[i]) & 0x00ffffff;
				    if(p->uPre3Bytes < 4 && p->demulate_enable) //modified by X ZHENG, 20080515
				    {
					    p->uClearBits = (p->uClearBits << 6) | (p->buf[i] >> 2);
					    p->iClearBitsNum += 6;
				    }
				    else
				    {
					    p->uClearBits = (p->uClearBits << 8) | p->buf[i];
					    p->iClearBitsNum += 8;
				    }
				    p->iBytePosition++;
                    k--;
                    i++;
                }
				return 0;
			}
			else
			{
				return -1;//arrive at stream end
			}
		}
		else
		{
			for(int j=0;j<k;j++) p->buf[j] = temp[j];
			p->iBufBytesNum += k;
			i = p->iBytePosition = 0;
		}
	}
	if(p->buf[i]==0 && p->buf[i+1]==0 && p->buf[i+2]==1)	return -2;// meet another start code
	p->uPre3Bytes = ((p->uPre3Bytes<<8) | p->buf[i]) & 0x00ffffff;
	if(p->uPre3Bytes < 4 && p->demulate_enable) //modified by XZHENG, 20080515
	{
		p->uClearBits = (p->uClearBits << 6) | (p->buf[i] >> 2);
		p->iClearBitsNum += 6;
	}
	else
	{
		p->uClearBits = (p->uClearBits << 8) | p->buf[i];
		p->iClearBitsNum += 8;
	}
	p->iBytePosition++;
	
	return 0;
}

static int read_n_bit(InputStream *p,int n,int *v)
{
	int r;
	unsigned int t;
	while(n > p->iClearBitsNum)
	{
		r = ClearNextByte( p );
		if(r)
		{
			if(r==-1)
			{
				if(p->iBufBytesNum - p->iBytePosition > 0)
					break;
			}
			return r;
		}
	}
	t = p->uClearBits;
	r = 32 - p->iClearBitsNum;
	*v = (t << r) >> (32 - n);
	p->iClearBitsNum -= n;
	return 0;
}

static int NextStartCode(InputStream *p)
{
	int i, m;
	unsigned char a = '\0', b = '\0';	// a b 0 1 2 3 4 ... M-3 M-2 M-1
	m=0;                // cjw 20060323 for linux envi
 
	while(1)
	{
		if(p->iBytePosition >= p->iBufBytesNum - 2)	//if all bytes in buffer has been searched
		{
			int last_len = 0;

			m = p->iBufBytesNum - p->iBytePosition;
			if(m <0) 
			{
				return 254; // current stream end // FIXIT
			}
			if(m==1)  b=p->buf[p->iBytePosition+1];
			if(m==2)
			{
				b = p->buf[p->iBytePosition + 1];
				a = p->buf[p->iBytePosition];
			}

			if( p->f < p->f_end )
			{
				last_len = (int)(p->f_end - p->f);
				p->iBufBytesNum = MIN2(SVA_STREAM_BUF_SIZE, last_len);
				memcpy( p->buf, p->f, p->iBufBytesNum );
				p->f += p->iBufBytesNum;
			}
			else
				p->iBufBytesNum = 0;

			p->iBytePosition = 0;
		}

		if(p->iBufBytesNum + m < 3) 
			return -1;  //arrive at stream end and start code is not found

		if(m==1 && b==0 && p->buf[0]==0 && p->buf[1]==1)
		{
			p->iBytePosition	= 2;
			p->iClearBitsNum	= 0;
			p->iStuffBitsNum	= 0;
			p->iBitsCount		+= 24;
			p->uPre3Bytes		= 1;
			return 0;
		}

		if(m==2 && b==0 && a==0 && p->buf[0]==1)
		{
			p->iBytePosition	= 1;
			p->iClearBitsNum	= 0;
			p->iStuffBitsNum	= 0;
			p->iBitsCount		+= 24;
			p->uPre3Bytes		= 1;
			return 0;
		}

		if(m==2 && b==0 && p->buf[0]==0 && p->buf[1]==1)
		{
			p->iBytePosition	= 2;
			p->iClearBitsNum	= 0;
			p->iStuffBitsNum	= 0;
			p->iBitsCount		+= 24;
			p->uPre3Bytes		= 1;
			return 0;
		}

		for(i = p->iBytePosition; i < p->iBufBytesNum - 2; i++)
		{
			if(p->buf[i]==0 && p->buf[i+1]==0 && p->buf[i+2]==1)
			{
				p->iBytePosition	= i+3;
				p->iClearBitsNum	= 0;
				p->iStuffBitsNum	= 0;
				p->iBitsCount		+= 24;
				p->uPre3Bytes		= 1;
				return 0;
			}
			p->iBitsCount += 8;
		}
		p->iBytePosition = i;
	}
}

static void CheckType(InputStream* p, int startcode)
{
    startcode = startcode&0x000000ff;
    switch(startcode)
    {
        case 0xb0:
        case 0xb2:
        case 0xb5:
            p->demulate_enable = 0;
            break;  
        default:
            p->demulate_enable = 1;
            break;
    }
}

unsigned int cavs_get_one_nal (InputStream* p, unsigned char *buf, int *length)
{
    InputStream* pIRABS = p;
    int i,k, j = 0;
    unsigned int i_startcode;
    int i_tmp = 0;

    i = NextStartCode(pIRABS);
    i_tmp = i;
    if(i!=0)
    {
        i_tmp = i;
        if( i == -1 )
        {
            i_tmp = 254;
			i_startcode = 4;
			return i_startcode;
        }
    }
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 1;
    //*startcodepos = 3;
    i = read_n_bit( pIRABS, 8, &j );
    buf[3] = j;
    CheckType( pIRABS, buf[3] );  //X ZHENG, 20080515, for demulation

    /* get startcode */
    i_startcode = buf[0];
    i_startcode = (i_startcode << 8) | buf[1];
    i_startcode = (i_startcode << 8) | buf[2];
    i_startcode = (i_startcode << 8) | buf[3];
       
    if((unsigned char)buf[3]==SEQUENCE_END_CODE)
    {
    	*length = 4;
        return i_startcode;
    }
    k = 4;
    while(1)
    {
    	i = read_n_bit(pIRABS,8,&j);
    	if(i<0) break;
    	buf[k++] = /*(char)*/j;
    }
    if(pIRABS->iClearBitsNum>0)
    {
    	int shift;
    	shift = 8 - pIRABS->iClearBitsNum;
    	i = read_n_bit(pIRABS,pIRABS->iClearBitsNum,&j);

    	if(j!=0)   
    		buf[k++] = /*(char)*/(j<<shift);
    }
    *length = k;
    
    if( i_tmp == 254 )
    {
        return 0x000001fe;
    }
    return i_startcode;
}

////////////////////////////////////////////////////////////////////////////////
#define CACHE_LINE        16
#define CACHE_LINE_MASK   15

#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 0)
#else
#pragma warning(disable:4996)
#endif

/****************************************************************************
 * * cavs_malloc:
 ****************************************************************************/
void *cavs_malloc( int i_size )
{
    uint8_t * buf;
    uint8_t * align_buf = NULL;

	buf = (uint8_t *) malloc( i_size + CACHE_LINE_MASK + sizeof( void ** ));
    if( buf )
	{
		//memset(buf, 0, i_size + CACHE_LINE_MASK + sizeof(void **));
		align_buf = buf + CACHE_LINE_MASK + sizeof( void ** );
		align_buf -= (uintptr_t)align_buf & CACHE_LINE_MASK;
		*( (void **) ( align_buf - sizeof( void ** ) ) ) = buf;
	}
    
    return align_buf;
}

/****************************************************************************
 * * cavs_free:
 ****************************************************************************/
void cavs_free( void *p )
{
    if( p )
    {
        free( *( ( ( void **) p ) - 1 ) );
    }
}

/****************************************************************************
 * cavs_realloc:
 ****************************************************************************/
void *cavs_realloc( void *p, int i_size )
{
    int       i_old_size = 0;
    uint8_t * p_new;
    if( p )
    {
        i_old_size = *( (int*) ( (uint8_t*) p  - sizeof( void ** ) -
                     sizeof( int ) ) );
    }
    p_new = (uint8_t *)cavs_malloc( i_size );
    if( i_old_size > 0 && i_size > 0 )
    {
        memcpy( p_new, p, ( i_old_size < i_size ) ? i_old_size : i_size );
    }
    cavs_free( p );

    return p_new;
}


int cavs_get_sequence_display_extension(cavs_bitstream *s, cavs_sequence_display_extension *e)
{
    e->i_video_format = cavs_bitstream_get_bits(s, 3);
    e->b_sample_range = cavs_bitstream_get_bits(s, 1);
    e->b_colour_description = cavs_bitstream_get_bit1(s);
    if(e->b_colour_description)
    {
        e->i_colour_primaries = cavs_bitstream_get_bits(s, 8);
        e->i_transfer_characteristics = cavs_bitstream_get_bits(s, 8);
        e->i_matrix_coefficients = cavs_bitstream_get_bits(s, 8);
    }
    e->i_display_horizontal_size = cavs_bitstream_get_bits(s, 14);
    cavs_bitstream_clear_bits(s, 1);
    e->i_display_vertical_size=cavs_bitstream_get_bits(s, 14);
	e->i_stereo_packing_mode = cavs_bitstream_get_bits(s, 2);

    return 0;
}

int cavs_get_copyright_extension(cavs_bitstream *s,cavs_copyright_extension *e)
{
    e->b_copyright_flag = cavs_bitstream_get_bit1(s);
    e->i_copyright_id = cavs_bitstream_get_bits(s, 8);
    e->b_original_or_copy = cavs_bitstream_get_bit1(s);
    cavs_bitstream_clear_bits(s, 7);
    cavs_bitstream_clear_bits(s, 1);
    e->i_copyright_number = cavs_bitstream_get_bits(s, 20);
    e->i_copyright_number <<= 22;
    cavs_bitstream_clear_bits(s, 1);
    e->i_copyright_number += cavs_bitstream_get_bits(s, 22);
    e->i_copyright_number <<= 22;
    cavs_bitstream_clear_bits(s, 1);
    e->i_copyright_number += cavs_bitstream_get_bits(s, 22);

    return 0;
}

int cavs_get_camera_parameters_extension(cavs_bitstream *s,cavs_camera_parameters_extension *e)
{	
    uint32_t i_value;

    cavs_bitstream_clear_bits(s, 1);
    e->i_camera_id = cavs_bitstream_get_bits(s, 7);
    cavs_bitstream_clear_bits(s, 1);
    e->i_height_of_image_device = cavs_bitstream_get_bits(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_focal_length = cavs_bitstream_get_bits(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_f_number = cavs_bitstream_get_bits(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_vertical_angle_of_view = cavs_bitstream_get_bits(s, 22);
    cavs_bitstream_clear_bits(s, 1);

    i_value = cavs_bitstream_get_bits(s, 16);
    i_value <<= 16;
    cavs_bitstream_clear_bits(s, 1);
    i_value += cavs_bitstream_get_bits(s, 16);
    cavs_bitstream_clear_bits(s, 1);
    e->i_camera_position_x = (int32_t)i_value;

    i_value = cavs_bitstream_get_bits(s, 16);
    i_value <<= 16;
    cavs_bitstream_clear_bits(s, 1);
    i_value += cavs_bitstream_get_bits(s, 16);
    cavs_bitstream_clear_bits(s, 1);
    e->i_camera_position_y = (int32_t)i_value;

    i_value = cavs_bitstream_get_bits(s, 16);
    i_value <<= 16;
    cavs_bitstream_clear_bits(s, 1);
    i_value += cavs_bitstream_get_bits(s, 16);
    cavs_bitstream_clear_bits(s, 1);
    e->i_camera_position_z = (int32_t)i_value;

    e->i_camera_direction_x = cavs_bitstream_get_int(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_camera_direction_y = cavs_bitstream_get_int(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_camera_direction_z = cavs_bitstream_get_int(s, 22);
    cavs_bitstream_clear_bits(s, 1);

    e->i_image_plane_vertical_x = cavs_bitstream_get_int(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_image_plane_vertical_y = cavs_bitstream_get_int(s, 22);
    cavs_bitstream_clear_bits(s, 1);
    e->i_image_plane_vertical_z = cavs_bitstream_get_int(s, 22);
    cavs_bitstream_clear_bits(s, 1);

    return 0;
}

int cavs_get_user_data(cavs_bitstream *s, uint8_t *user_data)
{
    while (((*(uint32_t*)(s->p)) & 0x00ffffff) != 0x000001/*0x00010000*/&& s->p < s->p_end)
    	*user_data = cavs_bitstream_get_bits(s, 8);

	return 0;
}

static int cavs_check_level_id( uint8_t level_id )
{
    if(      level_id != 0x10
        && level_id != 0x12
        && level_id != 0x14
        && level_id != 0x20
        && level_id != 0x22
        && level_id != 0x2A
        && level_id != 0x40
        && level_id != 0x41
        && level_id != 0x42
        && level_id != 0x44
        && level_id != 0x46
    )
    {
        return -1;    
    }
    

    return 0;
}

static int validResu[][2] = 
{
	{128,96}, //0
	{176,144}, //1
	{320,240}, //2
	{640,480}, //3
	{720,480}, //4
	{720,576}, //6
	{800,480}, //7
	{1280,720}, //8
	{1440,1080}, //9
	{1440,1080}, //10
	{1920,1080}, //11
	{1920,1088}, //12
	{960,1080},//13
	{960,1088}, //14
	{544, 576},	//15
	{528, 576},	//16
	{480, 576},	//17
	{704, 576},
	{-1,-1}
};
static int cavs_check_resolution(int width, int height)
{
	int i = 0;
	for(i=0; i<32; i++)
	{
		if (validResu[i][0] < 0) {
			break;
		}
		if(validResu[i][0]==width && validResu[i][1]==height)
			return 0;
	}
	return -1;
}

int cavs_get_video_sequence_header(cavs_bitstream *s,cavs_video_sequence_header *h)
{
	uint32_t   width, height;

    h->i_profile_id = cavs_bitstream_get_bits(s, 8);

    if((h->i_profile_id!=PROFILE_JIZHUN) && (h->i_profile_id!=PROFILE_GUANGDIAN) )
    {
       return -1;
    }

    h->i_level_id = cavs_bitstream_get_bits(s, 8);
    if(cavs_check_level_id(h->i_level_id))
    {
        return -1;
    }
    h->b_progressive_sequence = cavs_bitstream_get_bit1(s);

    width = cavs_bitstream_get_bits(s, 14);
    if( width < 176 || width > 1920)
    {
        return -1;    
    }
    height = cavs_bitstream_get_bits(s, 14);
    if( height < 128 || height >  1088)
    {
        return -1;    
    }
	if(cavs_check_resolution(width, height))
	{
		return -1;
	}

    h->i_chroma_format = cavs_bitstream_get_bits(s, 2);
    if( h->i_chroma_format != 1 )
    {
        return -1;
    }
    h->i_sample_precision = cavs_bitstream_get_bits(s, 3);
    if( h->i_sample_precision != 1)
    {
        return -1;
    }
    h->i_aspect_ratio = cavs_bitstream_get_bits(s, 4);
    if( h->i_aspect_ratio != 0x1
        && h->i_aspect_ratio != 0x2
        && h->i_aspect_ratio != 0x3
        && h->i_aspect_ratio != 0x4 )
    {
        return -1;    
    }
    h->i_frame_rate_code = cavs_bitstream_get_bits(s, 4);
    if( h->i_frame_rate_code != 0x1 
        && h->i_frame_rate_code != 0x2
        && h->i_frame_rate_code != 0x3
        && h->i_frame_rate_code != 0x4
        && h->i_frame_rate_code != 0x5
        && h->i_frame_rate_code != 0x6
        && h->i_frame_rate_code != 0x7
        && h->i_frame_rate_code != 0x8 )
    {
        return -1;  
    }
    
    h->i_bit_rate = cavs_bitstream_get_bits(s, 18);//bit_rate_lower
    h->i_bit_rate <<= 12;
    cavs_bitstream_clear_bits(s, 1); //marker_bit
    h->i_bit_rate += cavs_bitstream_get_bits(s, 12);
    h->b_low_delay = cavs_bitstream_get_bit1(s);
   
    cavs_bitstream_clear_bits(s, 1); //marker_bit
    h->i_bbv_buffer_size = cavs_bitstream_get_bits(s, 18);

    if(cavs_bitstream_get_bits(s,3)!=0) //reserved bits
    {/* 3bitsµÄ'000' */
    }

	h->i_horizontal_size = width;
	h->i_vertical_size = height;

    return 0;
}

short wq_param_default[2][6]=
{
	{135,143,143,160,160,213},   
	{128, 98,106,116,116,128}     
};

int cavs_get_i_picture_header(cavs_bitstream *s,cavs_picture_header *h,cavs_video_sequence_header *sh)
{
    h->i_picture_coding_type = CAVS_I_PICTURE;
    h->b_picture_reference_flag = 1;
    h->b_advanced_pred_mode_disable = 1;
    h->i_bbv_delay = cavs_bitstream_get_bits(s, 16);

    if (sh->i_profile_id == PROFILE_GUANGDIAN)
    {
    	cavs_bitstream_get_bit1(s);	//marker bit
    	cavs_bitstream_get_bits(s, 7);	//bbv_delay extension
    }

    h->b_time_code_flag = cavs_bitstream_get_bit1(s);
    if(h->b_time_code_flag)
    {
        h->i_time_code = cavs_bitstream_get_bits(s, 24);
    }
    cavs_bitstream_get_bit1(s);
    h->i_picture_distance = cavs_bitstream_get_bits(s, 8);

    if(sh->b_low_delay)
    {	
        //ue
        h->i_bbv_check_times = cavs_bitstream_get_ue(s);
    }
    h->b_progressive_frame = cavs_bitstream_get_bit1(s);
    if(h->b_progressive_frame==0)
    {
        h->b_picture_structure = cavs_bitstream_get_bit1(s);
    }
    else
    {
        h->b_picture_structure=1;
    }
    sh->b_interlaced = !h->b_picture_structure;
    h->b_top_field_first = cavs_bitstream_get_bit1(s);
    h->b_top_field_first = !h->b_picture_structure; // FIXIT:not support 0
    h->b_repeat_first_field = cavs_bitstream_get_bit1(s);
    h->b_fixed_picture_qp = cavs_bitstream_get_bit1(s);
    h->i_picture_qp = cavs_bitstream_get_bits(s, 6);

    if( h->b_progressive_frame == 0 && h->b_picture_structure == 0 )
    {
        h->b_skip_mode_flag = cavs_bitstream_get_bit1(s);
    }
    else
    {
        h->b_skip_mode_flag=0;
    }
    if(cavs_bitstream_get_bits(s,4)!=0)
    {
    }

    h->b_loop_filter_disable = cavs_bitstream_get_bit1(s);

    if(!h->b_loop_filter_disable)
    {
        h->b_loop_filter_parameter_flag = cavs_bitstream_get_bit1(s);
        if(h->b_loop_filter_parameter_flag)
        {
            //se
            h->i_alpha_c_offset = cavs_bitstream_get_se(s);	
            h->i_beta_offset = cavs_bitstream_get_se(s);	
        }
        else
        {
            h->i_alpha_c_offset = 0;	
            h->i_beta_offset = 0;	
        }
    }
    
    if(sh->i_profile_id != PROFILE_JIZHUN)
    {
        h->weighting_quant_flag = cavs_bitstream_get_bit1(s);
        if (h->weighting_quant_flag)
        {
            h->mb_adapt_wq_disable = cavs_bitstream_get_bit1(s);
            h->chroma_quant_param_disable = cavs_bitstream_get_bit1(s);
            if (! h->chroma_quant_param_disable)
            {
                h->chroma_quant_param_delta_u = cavs_bitstream_get_se(s);	
                h->chroma_quant_param_delta_v = cavs_bitstream_get_se(s);	
            }
            h->weighting_quant_param_index = cavs_bitstream_get_bits(s, 2);
			h->weighting_quant_model = cavs_bitstream_get_bits(s, 2);
		}
        if((h->weighting_quant_param_index == 1)||(/*(!h->mb_adapt_wq_disable)&&*/(h->weighting_quant_param_index == 3)))
        {
			for(int i=0;i<6;i++)
                h->weighting_quant_param_undetail[i]=(uint8_t)cavs_bitstream_get_se(s) + wq_param_default[0][i];

        }
        if((h->weighting_quant_param_index == 2)||(/*(!h->mb_adapt_wq_disable)&&*/(h->weighting_quant_param_index == 3)))
        {
			for(int i=0;i<6;i++)
                h->weighting_quant_param_detail[i]=(uint8_t)cavs_bitstream_get_se(s) + wq_param_default[1][i];//?
        }	
    }
    else 
    {
        h->weighting_quant_flag=0;
        h->mb_adapt_wq_disable=1;
        h->chroma_quant_param_delta_u=0;	
        h->chroma_quant_param_delta_v=0;
    }

    //YiDong related
    if (sh->i_profile_id == PROFILE_YIDONG)
    {
        h->vbs_enable=cavs_bitstream_get_bit1(s);
        if (h->vbs_enable)
        {
            h->vbs_qp_shift=cavs_bitstream_get_bits(s,4);
        }
        h->asi_enable=cavs_bitstream_get_bit1(s);
    }
    else
    {
        h->vbs_enable = 0;
        h->asi_enable = 0;
    }

    //GuangDian related
    if (sh->i_profile_id == PROFILE_GUANGDIAN)
        h->b_aec_enable = cavs_bitstream_get_bit1(s);
    else
        h->b_aec_enable = 0;
    
	/* NOTE : just process once to get status of AEC */
	if( !sh->b_flag_aec )
	{
		sh->b_flag_aec = 1;
		sh->b_aec_enable = h->b_aec_enable;
	}

    h->b_pb_field_enhanced = 0;

    return 0;
}

int cavs_get_pb_picture_header(cavs_bitstream *s,cavs_picture_header *h,cavs_video_sequence_header *sh)
{   
    h->i_bbv_delay = cavs_bitstream_get_bits(s, 16);

    if (sh->i_profile_id == PROFILE_GUANGDIAN)
    {
        cavs_bitstream_get_bit1(s);	//marker bit
        cavs_bitstream_get_bits(s, 7);	//bbv_delay extension
    }

    h->i_picture_coding_type = cavs_bitstream_get_bits(s, 2);
    if(  h->i_picture_coding_type != 1 &&  h->i_picture_coding_type != 2 )
    {
        return -1;
    }
    h->i_picture_distance = cavs_bitstream_get_bits(s, 8);

    if(sh->b_low_delay)
    {
        h->i_bbv_check_times = cavs_bitstream_get_ue(s);	
    }
    h->b_progressive_frame = cavs_bitstream_get_bit1(s);

    if(h->b_progressive_frame==0)
    {
        h->b_picture_structure = cavs_bitstream_get_bit1(s);

        if(h->b_picture_structure==0)
        {
            h->b_advanced_pred_mode_disable = cavs_bitstream_get_bit1(s);
            if(h->b_advanced_pred_mode_disable != 1)
            {
                return -1;
            }
        }
    }
    else
    {
        h->b_picture_structure = 1;
    }
    sh->b_interlaced = !h->b_picture_structure;
    h->b_top_field_first = cavs_bitstream_get_bit1(s);
    h->b_top_field_first = !h->b_picture_structure;// FIXIT:not support 0
    h->b_repeat_first_field = cavs_bitstream_get_bit1(s);
    h->b_fixed_picture_qp = cavs_bitstream_get_bit1(s);
    h->i_picture_qp = cavs_bitstream_get_bits(s, 6);

    if(!(h->i_picture_coding_type==CAVS_B_PICTURE&&h->b_picture_structure==1))
    {
        h->b_picture_reference_flag = cavs_bitstream_get_bit1(s);
    }

    h->b_no_forward_reference_flag = cavs_bitstream_get_bit1(s);
    if (sh->i_profile_id == PROFILE_GUANGDIAN)
    {
        h->b_pb_field_enhanced = cavs_bitstream_get_bit1(s); //pb_field_enhanced_flag
        cavs_bitstream_get_bits(s, 2); //reserved bits
    }
    else
    {
        h->b_pb_field_enhanced = 0;
        if(cavs_bitstream_get_bits(s,3)!=0)
            ;//return -1;
    }
    h->b_skip_mode_flag = cavs_bitstream_get_bit1(s);
    h->b_loop_filter_disable = cavs_bitstream_get_bit1(s);

    if(!h->b_loop_filter_disable)
    {
        h->b_loop_filter_parameter_flag = cavs_bitstream_get_bit1(s);
        if(h->b_loop_filter_parameter_flag)
        {
            //se
            h->i_alpha_c_offset = cavs_bitstream_get_se(s);	
            h->i_beta_offset = cavs_bitstream_get_se(s);	
        }
        else
        {
            h->i_alpha_c_offset = 0;	
            h->i_beta_offset = 0;	
        }
    }
    
    if(sh->i_profile_id != PROFILE_JIZHUN)
    {
        h->weighting_quant_flag = cavs_bitstream_get_bit1(s);
        if (h->weighting_quant_flag)
        {
            uint32_t scene_adapt_disable = 0;
            uint32_t scene_model_update_flag = 0;
            
            h->mb_adapt_wq_disable = cavs_bitstream_get_bit1(s);
            h->chroma_quant_param_disable = cavs_bitstream_get_bit1(s);
            if (! h->chroma_quant_param_disable)
            {
                h->chroma_quant_param_delta_u = cavs_bitstream_get_se(s);	
                h->chroma_quant_param_delta_v = cavs_bitstream_get_se(s);	
            }
            h->weighting_quant_param_index = cavs_bitstream_get_bits(s, 2);

            if (scene_adapt_disable==0&&scene_model_update_flag==0)
            {
                h->weighting_quant_model = cavs_bitstream_get_bits(s, 2);
            }
            else
            {
                h->weighting_quant_model = 3;
            }
            if((h->weighting_quant_param_index == 1)||(/*(!h->mb_adapt_wq_disable)&&*/(h->weighting_quant_param_index == 3)))
            {
                int i;
				for( i = 0; i < 6; i++)
                    h->weighting_quant_param_undetail[i] = (uint8_t)cavs_bitstream_get_se(s) + wq_param_default[0][i];
            }
            if((h->weighting_quant_param_index == 2)||(/*(!h->mb_adapt_wq_disable)&&*/(h->weighting_quant_param_index == 3)))
            {
                int i;
				for( i = 0; i < 6; i++)
                    h->weighting_quant_param_detail[i] = (uint8_t)cavs_bitstream_get_se(s) + wq_param_default[1][i];
            }	
        }
    }

    if (sh->i_profile_id == PROFILE_YIDONG)
    {
        if(h->i_picture_coding_type==CAVS_P_PICTURE)
        {
            h->limitDC_refresh_frame = cavs_bitstream_get_bit1(s);
        }
    }

    if (sh->i_profile_id == PROFILE_YIDONG)
    {
        h->vbs_enable = cavs_bitstream_get_bit1(s);
        if (h->vbs_enable)
        {
            h->vbs_qp_shift = cavs_bitstream_get_bits(s, 4);
        }
        h->asi_enable = cavs_bitstream_get_bit1(s);
    }
    else
    {
        h->vbs_enable = 0;
        h->asi_enable = 0;
    }

    if (sh->i_profile_id == PROFILE_GUANGDIAN)
    	h->b_aec_enable = cavs_bitstream_get_bit1(s);
    else
    	h->b_aec_enable = 0;

	return 0;
}

int cavs_cavlc_get_ue(cavs_decoder *p)
{
    return cavs_bitstream_get_ue(&p->s);
}

int cavs_cavlc_get_se(cavs_decoder *p)
{
    return cavs_bitstream_get_se(&p->s);
}

int cavs_cavlc_get_mb_type_p(cavs_decoder *p)
{
    const int i_mb_type = cavs_bitstream_get_ue(&p->s) + P_SKIP + p->ph.b_skip_mode_flag;

    if (i_mb_type > P_8X8)
    {
        p->i_cbp_code = i_mb_type - P_8X8 - 1;
        return I_8X8;
    }
    
    return i_mb_type;
}

int cavs_cavlc_get_mb_type_b(cavs_decoder *p)
{
    const int i_mb_type = cavs_bitstream_get_ue(&p->s) + B_SKIP + p->ph.b_skip_mode_flag;

    if (i_mb_type > B_8X8)
    {
        p->i_cbp_code = i_mb_type - B_8X8 - 1;
        return I_8X8;
    }
    
    return i_mb_type;
}

int cavs_cavlc_get_intra_luma_pred_mode(cavs_decoder *p)
{
    p->pred_mode_flag = cavs_bitstream_get_bit1(&p->s);

    if(!p->pred_mode_flag)
    {
        return cavs_bitstream_get_bits(&p->s, 2);
    }
    else
        return -1;
}

static const uint8_t cbp_tab[64][2] =	/*[cbp_code][inter]*/
{
	{63, 0},{15,15},{31,63},{47,31},{ 0,16},{14,32},{13,47},{11,13},
	{ 7,14},{ 5,11},{10,12},{ 8, 5},{12,10},{61, 7},{ 4,48},{55, 3},
	{ 1, 2},{ 2, 8},{59, 4},{ 3, 1},{62,61},{ 9,55},{ 6,59},{29,62},
	{45,29},{51,27},{23,23},{39,19},{27,30},{46,28},{53, 9},{30, 6},
	{43,60},{37,21},{60,44},{16,26},{21,51},{28,35},{19,18},{35,20},
	{42,24},{26,53},{44,17},{32,37},{58,39},{24,45},{20,58},{17,43},
	{18,42},{48,46},{22,36},{33,33},{25,34},{49,40},{40,52},{36,49},
	{34,50},{50,56},{52,25},{54,22},{41,54},{56,57},{38,41},{57,38}
};

int cavs_cavlc_get_intra_cbp(cavs_decoder *p)
{
    if (!p->b_have_pred)
    {
        p->i_cbp_code = cavs_bitstream_get_ue(&p->s);
    }
    if(p->i_cbp_code > 63 || p->i_cbp_code < 0)
	{
		p->b_error_flag = 1;

		return -1;
	}
    return cbp_tab[p->i_cbp_code][0];
}

int cavs_cavlc_get_inter_cbp(cavs_decoder *p)
{
    p->i_cbp_code = cavs_bitstream_get_ue(&p->s);
    if(p->i_cbp_code > 63 || p->i_cbp_code < 0)
	{
		p->b_error_flag = 1;

		return -1;
	}
    return cbp_tab[p->i_cbp_code][1];
}

int cavs_cavlc_get_ref_p(cavs_decoder *p, int ref_scan_idx)
{
    if(p->ph.b_picture_reference_flag )
        return 0;
    else
    {
        if(p->ph.b_picture_structure==0 /*&& p->ph.i_picture_coding_type==CAVS_P_PICTURE*/)
            return cavs_bitstream_get_bits(&p->s,2);
        else
            return cavs_bitstream_get_bit1(&p->s);
    }
}

int cavs_cavlc_get_ref_b(cavs_decoder *p, int i_list, int ref_scan_idx)
{
    if(p->ph.b_picture_structure == 0 && p->ph.b_picture_reference_flag ==0)
        return cavs_bitstream_get_bit1(&p->s);	//mb_reference_index
    else
        return 0;
}

int cavs_cavlc_get_mb_part_type(cavs_decoder *p)
{
    return cavs_bitstream_get_bits(&p->s, 2);
}

//i_list, i_down and xy_idx is not used here(just for cabac)
int cavs_cavlc_get_mvd(cavs_decoder *p, int i_list, int i_down, int xy_idx)
{
    return cavs_bitstream_get_se(&p->s);
}

int cavs_cavlc_get_coeffs(cavs_decoder *p, const cavs_vlc *p_vlc_table, int i_escape_order, int b_chroma)
{
    int level_code, esc_code, level, run, mask;
    int *level_buf = p->level_buf;
    int *run_buf = p->run_buf;
    int i;
    
    for( i = 0; i < 65; i++ ) 
    {
        level_code = cavs_bitstream_get_ue_k(&p->s,p_vlc_table->golomb_order);
        if(level_code >= ESCAPE_CODE) 
        {
            run = ((level_code - ESCAPE_CODE) >> 1) + 1;
            esc_code = cavs_bitstream_get_ue_k(&p->s,i_escape_order);

            level = esc_code + (run > p_vlc_table->max_run ? 1 : p_vlc_table->level_add[run]);
            while(level > p_vlc_table->inc_limit)
                p_vlc_table++;
            mask = -(level_code & 1);
            level = (level^mask) - mask;
        } 
        else
        {
            level = p_vlc_table->rltab[level_code][0];
            if(!level) //end of block signal
                break;
            run   = p_vlc_table->rltab[level_code][1];
            p_vlc_table += p_vlc_table->rltab[level_code][2];
        }
        level_buf[i] = level;
        run_buf[i] = run;
    }
    
    return i;
}
////////////////////////////////////////////////////////////////////////////////

#define TEST_CABAC 0

static const uint8_t sub_mb_type_b_to_golomb[13] = {
	10, 4, 5, 1, 11, 6, 7, 2, 12, 8, 9, 3, 0
};

static const uint8_t mb_type_b_to_golomb[3][9] = {
	{4, 8, 12, 10, 6, 14, 16, 18, 20},    /* D_16x8 */
	{5, 9, 13, 11, 7, 15, 17, 19, 21},    /* D_8x16 */
	{1, -1, -1, -1, 2, -1, -1, -1, 3}     /* D_16x16 */
};


#define BIARI_CTX_INIT1_LOG(jj,ctx)\
{\
	for (j=0; j<jj; j++)\
{\
	ctx[j].lg_pmps = 1023;\
	ctx[j].mps = 0;\
	ctx[j].cycno = 0;\
	}\
}

#define BIARI_CTX_INIT2_LOG(ii,jj,ctx)\
{\
	for (i=0; i<ii; i++)\
	for (j=0; j<jj; j++)\
{\
	ctx[i][j].lg_pmps = 1023;\
	ctx[i][j].mps = 0;\
	ctx[i][j].cycno = 0;\
	}\
}
#define get_byte(){                         \
	if(cb->bs.p > cb->bs.p_end )\
	{\
		cb->b_cabac_error = 1;\
		return 0; \
	}\
	cb->buffer = *(cb->bs.p++);				\
	cb->bits_to_go = 7;                     \
}

static void biari_init_context_logac (bi_ctx_t* ctx)
{
	ctx->lg_pmps = (QUARTER<<LG_PMPS_SHIFTNO)-1; //10 bits precision
	ctx->mps	 = 0;	
	ctx->cycno  =  0;
}

/*****************************************************************************
*
*****************************************************************************/
void cavs_cabac_context_init( cavs_decoder * h)
{
    int i, j;

    //--- motion coding contexts ---
    BIARI_CTX_INIT2_LOG (3, NUM_MB_TYPE_CTX,   h->cabac.mb_type_contexts);
    BIARI_CTX_INIT2_LOG (2, NUM_B8_TYPE_CTX,   h->cabac.b8_type_contexts);
    BIARI_CTX_INIT2_LOG (2, NUM_MV_RES_CTX,    h->cabac.mv_res_contexts);
    BIARI_CTX_INIT2_LOG (2, NUM_REF_NO_CTX,    h->cabac.ref_no_contexts);
    BIARI_CTX_INIT1_LOG (   NUM_DELTA_QP_CTX,  h->cabac.delta_qp_contexts);
    BIARI_CTX_INIT1_LOG (   NUM_MB_AFF_CTX,    h->cabac.mb_aff_contexts);

    //--- texture coding contexts ---
    BIARI_CTX_INIT1_LOG (                 NUM_IPR_CTX,  h->cabac.ipr_contexts);
    BIARI_CTX_INIT1_LOG (                 NUM_CIPR_CTX, h->cabac.cipr_contexts);
    BIARI_CTX_INIT2_LOG (3,               NUM_CBP_CTX,  h->cabac.cbp_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_BCBP_CTX, h->cabac.bcbp_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_ONE_CTX,  h->cabac.one_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_ABS_CTX,  h->cabac.abs_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_MAP_CTX,  h->cabac.fld_map_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_LAST_CTX, h->cabac.fld_last_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_MAP_CTX,  h->cabac.map_contexts);
    BIARI_CTX_INIT2_LOG (NUM_BLOCK_TYPES, NUM_LAST_CTX, h->cabac.last_contexts);

#if B_MB_WEIGHTING
    biari_init_context_logac( &h->cabac.mb_weighting_pred);
#endif
}

int cavs_cabac_start_decoding(cavs_cabac_t * cb, cavs_bitstream *s)
{
    int i;

    cb->bs = *s;
    cb->s1 = 0;
    cb->t1 = 0xFF;
    cb->value_s = cb->value_t = 0;
    cb->bits_to_go = 0;
    for (i = 0; i< B_BITS - 1; i++)
    {
        if (--cb->bits_to_go < 0)
            get_byte();
        cb->value_t = (cb->value_t<<1)  | ((cb->buffer >> cb->bits_to_go) & 0x01); 
    }

    while (cb->value_t < QUARTER)
    {
        if (--cb->bits_to_go < 0) 
            get_byte();   
        // Shift in next bit and add to value 
        cb->value_t = (cb->value_t << 1) | ((cb->buffer >> cb->bits_to_go) & 0x01);
        cb->value_s++;
    }
    cb->value_t = cb->value_t & 0xff;

    cb->dec_bypass  = 0;
    cb->b_cabac_error = 0;

	return 0;
}

static inline unsigned int cavs_biari_decode_symbol(cavs_cabac_t *cb, bi_ctx_t *bi_ct)
{
    unsigned char bit;
    unsigned char s_flag, is_LPS = 0;
    unsigned char cwr, cycno = bi_ct->cycno;
    unsigned int lg_pmps = bi_ct->lg_pmps;
    unsigned int t_rlps;
    unsigned int s2, t2;
    bit = bi_ct->mps;

#if TEST_CABAC
    cwr = cwr_trans[cycno];
    t1_d = cb->t1 - lg_pmps_shift2[lg_pmps];
    if ( t1_d >= 0 )
    {
        s2 = cb->s1;
        t2 = t1_d;
        s_flag = 0;
    }
    else
    {
        s2 = cb->s1+1;
        t2 = 256 + t1_d;
        s_flag = 1;
    }
#else
    cwr = (cycno<=1)?3:(cycno==2)?4:5;

    if (cb->t1 >= (lg_pmps>>LG_PMPS_SHIFTNO))
    {
        s2 = cb->s1;
        t2 = cb->t1 - (lg_pmps>>LG_PMPS_SHIFTNO);
        s_flag = 0;
    }
    else
    {
        s2 = cb->s1+1;
        t2 = 256 + cb->t1 - (lg_pmps>>LG_PMPS_SHIFTNO);
        s_flag = 1;
    }
#endif

    if( s2 > cb->value_s || ( s2 == cb->value_s && cb->value_t >= t2)) //LPS
    {
        is_LPS = 1;
        bit=!bit;//LPS
        
#if TEST_CABAC
        t_rlps = (s_flag==0)? lg_pmps_shift2[lg_pmps]
        	:(cb->t1 + lg_pmps_shift2[lg_pmps]);
#else
        t_rlps = (s_flag==0)? (lg_pmps>>LG_PMPS_SHIFTNO)
        	:(cb->t1 + (lg_pmps>>LG_PMPS_SHIFTNO));
#endif

        if (s2==cb->value_s)
            cb->value_t = (cb->value_t-t2);
        else
        {
        	if (--cb->bits_to_go < 0)
        		get_byte();
        	// Shift in next bit and add to value 
        	cb->value_t = (cb->value_t << 1) | ((cb->buffer >> cb->bits_to_go) & 0x01);
        	cb->value_t = 256 + cb->value_t - t2;
        }

        //restore range		
        while (t_rlps < QUARTER){
        	t_rlps=t_rlps<<1;
        	if (--cb->bits_to_go < 0) 
        		get_byte();   
        	// Shift in next bit and add to value 
        	cb->value_t = (cb->value_t << 1) | ((cb->buffer >> cb->bits_to_go) & 0x01);
        }

        cb->s1 = 0;
        cb->t1 = t_rlps & 0xff;

        //restore value
        cb->value_s = 0;
        while (cb->value_t<QUARTER){
        	int j;
        	if (--cb->bits_to_go < 0) 
        		get_byte();   
        	j=(cb->buffer >> cb->bits_to_go) & 0x01;
        	// Shift in next bit and add to value 

        	cb->value_t = (cb->value_t << 1) | j;
        	cb->value_s++;
        }
        cb->value_t = cb->value_t & 0xff;		
    }
    else //MPS
    {		
        cb->s1 = s2;
        cb->t1 = t2;				
    }

    if (cb->dec_bypass) return(bit);

    //update other parameters
    if (is_LPS)
    {  
#if TEST_CABAC
        cycno = cycno_trans2[cycno];
#else
        cycno=(cycno<=2)?(cycno+1):3;
#endif
    }
    else if (cycno==0) 
        cycno =1;			
    bi_ct->cycno=cycno;

    /* update probability estimation */
#if TEST_CABAC
    if (is_LPS)
    {
		
        lg_pmps = lg_pmps_tab[cwr][lg_pmps];    		
        if (lg_pmps >= 1024)
        {
            lg_pmps = 2047 - lg_pmps;
            bi_ct->mps = !(bi_ct->mps);
        }	      
    }
    else
    {
        lg_pmps = lg_pmps_tab_mps[cwr][lg_pmps];	
    }

#else
    if (is_LPS)
    {
    	switch(cwr) {
    	case 3:	lg_pmps = lg_pmps + 197;					
    		break;
    	case 4: lg_pmps = lg_pmps + 95;
    		break;
    	default:lg_pmps = lg_pmps + 46; 
    	}

    	if (lg_pmps>=(256<<LG_PMPS_SHIFTNO))
    	{
    		lg_pmps = (512<<LG_PMPS_SHIFTNO) - 1 - lg_pmps;
    		bi_ct->mps = !(bi_ct->mps);
    	}	      
    }
    else
    {
    	lg_pmps = lg_pmps - (unsigned int)(lg_pmps>>cwr) - (unsigned int)(lg_pmps>>(cwr+2));	
    }
#endif
    

    bi_ct->lg_pmps = lg_pmps;

    return(bit);

}

static inline unsigned int cavs_biari_decode_symbol_bypass(cavs_cabac_t *cb)
{
    bi_ctx_t ctx;
    uint32_t bit;
	ctx.cycno = 0; // added by xun
    ctx.lg_pmps = (QUARTER << LG_PMPS_SHIFTNO) - 1;
    ctx.mps = 0;
    cb->dec_bypass = 1;
    bit = cavs_biari_decode_symbol(cb, &ctx);
    cb->dec_bypass = 0;
    return bit;
}

int cavs_biari_decode_stuffing_bit(cavs_cabac_t *cb)
{
    bi_ctx_t ctx;
    uint32_t bit;
	ctx.cycno = 0;
    ctx.lg_pmps = 1 << LG_PMPS_SHIFTNO;
    ctx.mps = 0;
    bit = cavs_biari_decode_symbol(cb, &ctx);
    
    return bit;
}

static int cavs_biari_decode_symbol_w(cavs_cabac_t *cb, bi_ctx_t *bi_ct1 ,bi_ctx_t *bi_ct2 )
{
    uint8_t bit1,bit2; 
    uint8_t pred_MPS,bit;
    uint32_t lg_pmps;
    uint8_t cwr1,cycno1=bi_ct1->cycno;
    uint8_t cwr2,cycno2=bi_ct2->cycno;
    uint32_t lg_pmps1= bi_ct1->lg_pmps,lg_pmps2= bi_ct2->lg_pmps;
    uint32_t t_rlps;
    uint8_t s_flag,is_LPS=0;
    uint32_t s2,t2;
    bit1 = bi_ct1->mps;
    bit2 = bi_ct2->mps;

#if TEST_CABAC
    if( cycno1 > 3 )
    {
        return -1;
    }
    cwr1 = cwr_trans[cycno1];
    if( cycno2 > 3 )
    {
        return -1;
    }
    cwr2 = cwr_trans[cycno2];
#else    
    cwr1 = (cycno1<=1)?3:(cycno1==2)?4:5;
    cwr2 = (cycno2<=1)?3:(cycno2==2)?4:5; 
#endif

    if (bit1 == bit2) 
    {
    	pred_MPS = bit1;
    	lg_pmps = (lg_pmps1 + lg_pmps2)/2;
    }
    else 
    {
    	if (lg_pmps1<lg_pmps2) {
    		pred_MPS = bit1;
    		lg_pmps = 1023 - ((lg_pmps2 - lg_pmps1)>>1);
    	}
    	else {
    		pred_MPS = bit2;
    		lg_pmps = 1023 - ((lg_pmps1 - lg_pmps2)>>1);
    	}
    }

#if TEST_CABAC
	if(lg_pmps>1023)
	{
		return -1;
	}
    t1_d = cb->t1 - lg_pmps_shift2[lg_pmps];
    if ( t1_d >= 0 )
    {
    	s2 = cb->s1;
    	t2 = t1_d;
    	s_flag = 0;
    }
    else
    {
    	s2 = cb->s1 + 1;
    	t2 = 256 + t1_d;
    	s_flag = 1;
    }
#else
    if (cb->t1>=(lg_pmps>>LG_PMPS_SHIFTNO))
    {
    	s2 = cb->s1;
    	t2 = cb->t1 - (lg_pmps>>LG_PMPS_SHIFTNO);
    	s_flag = 0;
    }
    else
    {
    	s2 = cb->s1 + 1;
    	t2 = 256 + cb->t1 - (lg_pmps>>LG_PMPS_SHIFTNO);
    	s_flag = 1;
    }
#endif

    bit = pred_MPS;
    if(s2 > cb->value_s || (s2 == cb->value_s && cb->value_t >= t2))//LPS
    {			
    	is_LPS = 1;
    	bit=!bit;//LPS

#if TEST_CABAC
       t_rlps = (s_flag==0)? lg_pmps_shift2[lg_pmps]:(cb->t1 + lg_pmps_shift2[lg_pmps]);	
#else
    	t_rlps = (s_flag==0)? (lg_pmps>>LG_PMPS_SHIFTNO):(cb->t1 + (lg_pmps>>LG_PMPS_SHIFTNO));		
#endif

    	if (s2 == cb->value_s)
    		cb->value_t = (cb->value_t-t2);
    	else
    	{		
    		if (--cb->bits_to_go < 0) 
    			get_byte();   
    		// Shift in next bit and add to value 
    		cb->value_t = (cb->value_t << 1) | ((cb->buffer >> cb->bits_to_go) & 0x01);
    		cb->value_t = 256 + cb->value_t - t2;
    	}

    	//restore range		
    	while (t_rlps < QUARTER){
    		t_rlps=t_rlps<<1;
    		if (--cb->bits_to_go < 0) 
    			get_byte();   
    		// Shift in next bit and add to value 
    		cb->value_t = (cb->value_t << 1) | ((cb->buffer >> cb->bits_to_go) & 0x01);
    	}
    	cb->s1 = 0;
    	cb->t1 = t_rlps & 0xff;

    	//restore value
    	cb->value_s = 0;
    	while (cb->value_t<QUARTER){
    		int j;
    		if (--cb->bits_to_go < 0) 
    			get_byte();   
    		j=(cb->buffer >> cb->bits_to_go) & 0x01;
    		// Shift in next bit and add to value 

    		cb->value_t = (cb->value_t << 1) | j;
    		cb->value_s++;
    	}
    	cb->value_t = cb->value_t & 0xff;			
    }//--LPS 
    else //MPS
    {
    	cb->s1 = s2;
    	cb->t1 = t2;
    }

#if TEST_CABAC
    cycno1 = cycno_trans_2d[bit!=bit1][cycno1];
    cycno2 = cycno_trans_2d[bit!=bit2][cycno2];
#else
    if (bit!=bit1)
    {			
    	cycno1 = (cycno1<=2)?(cycno1+1):3;//LPS occurs
    }
    else{
    	if (cycno1==0) cycno1 =1;
    }

    if (bit !=bit2)
    {			
    	cycno2 = (cycno2<=2)?(cycno2+1):3;//LPS occurs
    }
    else
    {
    	if (cycno2==0) cycno2 =1;
    }
#endif
    
    bi_ct1->cycno = cycno1;
    bi_ct2->cycno = cycno2;

    //update probability estimation
#if TEST_CABAC
    //bi_ct1
    if( lg_pmps1 > 1023 || cwr1 > 5 )
    {
    	return -1;
    }
    
    if (bit==bit1)
    {	
        lg_pmps1 = lg_pmps_tab_mps[cwr1][lg_pmps1];
    }
    else
    {
        lg_pmps1 = lg_pmps_tab[cwr1][lg_pmps1];
        if (lg_pmps1 >= 1024)
        {
            lg_pmps1 = 2047 - lg_pmps1;
            bi_ct1->mps = !(bi_ct1->mps);
        }	
    }
    bi_ct1->lg_pmps = lg_pmps1;

    //bi_ct2
    if( lg_pmps2 > 1023 || cwr2 > 5 )
    {
    	return -1;
    }
    
    if (bit==bit2)
    {        	
        lg_pmps2 = lg_pmps_tab_mps[cwr2][lg_pmps2];
    }
    else
    {
        lg_pmps2 = lg_pmps_tab[cwr2][lg_pmps2];
        if (lg_pmps2 >= 1024)
        {
            lg_pmps2 = 2047 - lg_pmps2;
            bi_ct2->mps = !(bi_ct2->mps);
        }	
    }
    bi_ct2->lg_pmps = lg_pmps2;  
    

#else    
    //bi_ct1
    if (bit==bit1)
    {
        lg_pmps1 = lg_pmps1 - (unsigned int)(lg_pmps1>>cwr1) - (unsigned int)(lg_pmps1>>(cwr1+2));	
    }
    else
    {
        switch(cwr1) {
            case 3:	lg_pmps1 = lg_pmps1 + 197;					
            	break;
            case 4: lg_pmps1 = lg_pmps1 + 95;
            	break;
            default:lg_pmps1 = lg_pmps1 + 46; 
        }

        if (lg_pmps1>=(256<<LG_PMPS_SHIFTNO))
        {
            lg_pmps1 = (512<<LG_PMPS_SHIFTNO) - 1 - lg_pmps1;
            bi_ct1->mps = !(bi_ct1->mps);
        }	
    }
    bi_ct1->lg_pmps = lg_pmps1;

    //bi_ct2
    if (bit==bit2)
    {
        lg_pmps2 = lg_pmps2 - (unsigned int)(lg_pmps2>>cwr2) - (unsigned int)(lg_pmps2>>(cwr2+2));	
    }
    else
    {
        switch(cwr2) {
            case 3:	lg_pmps2 = lg_pmps2 + 197;					
                break;
            case 4: lg_pmps2 = lg_pmps2 + 95;
                break;
            default:lg_pmps2 = lg_pmps2 + 46; 
        }

        if (lg_pmps2>=(256<<LG_PMPS_SHIFTNO))
        {
            lg_pmps2 = (512<<LG_PMPS_SHIFTNO) - 1 - lg_pmps2;
            bi_ct2->mps = !(bi_ct2->mps);
        }	
    }
    bi_ct2->lg_pmps = lg_pmps2;  
#endif

    return(bit);
}

int cavs_cabac_get_skip(cavs_decoder *p)
{
    int i = 0, symbol = 0;
    bi_ctx_t * ctx = p->cabac.one_contexts[0];

    while( cavs_biari_decode_symbol(&p->cabac, ctx+i)==0 ) {
        symbol += 1;
        i ++;
        if( i >= 3 ) i=3;

		if( symbol > (int)(p->i_mb_num>>p->param.b_interlaced) ) /* NOTE : remove endless loop */
		{
			 p->b_error_flag = 1;
			 return -1;
		}
    }
    
    p->b_error_flag = (&(p->cabac))->b_cabac_error;
    if( p->b_error_flag )
    {
        return -1;
    }

    return symbol;
}

int cavs_cabac_get_mb_type_p(cavs_decoder *p)
{
    bi_ctx_t * ctx = p->cabac.mb_type_contexts[1];
    int symbol = 0, i = 0;

	const int DMapSkip[5] = {4,0,1,2,3};
	const int DMapNonSkip[6] = {5,0, 1,2,3,4};

	while( cavs_biari_decode_symbol (&p->cabac, ctx+i)==0 ) {
            symbol++;
            i++;
            if( i>=4 ) i=4;

			if(symbol > 5) /* NOTE : remove endless loop */
			{
				p->b_error_flag = 1;
				return -1;
			}
       }

	if( p->ph.b_skip_mode_flag )
	{
		if(symbol > 4)
		{
			p->b_error_flag = 1;
			return -1;
		}
		symbol =  DMapSkip[symbol];
	}
	else
	{
		if(symbol > 5)
		{
			p->b_error_flag = 1;
			return -1;
		}
		symbol =  DMapNonSkip[symbol];
	}

    p->b_error_flag = (&(p->cabac))->b_cabac_error;
   
    if( symbol + p->ph.b_skip_mode_flag == 5  )
        return I_8X8;
    else
    {
        return symbol + P_SKIP + p->ph.b_skip_mode_flag; 
    }
}

int cavs_cabac_get_mb_type_b(cavs_decoder *p)
{
    const int i_mb_type_left = p->i_mb_type_left;
    const int i_mb_type_top  = p->p_mb_type_top[p->i_mb_x];
    int i = (i_mb_type_left != -1 && i_mb_type_left != B_SKIP && i_mb_type_left != B_DIRECT)
    	+ (i_mb_type_top != -1 && i_mb_type_top != B_SKIP && i_mb_type_top != B_DIRECT);
    
    bi_ctx_t * ctx = p->cabac.mb_type_contexts[2];
    int symbol;

    if( cavs_biari_decode_symbol(&p->cabac, ctx + i)==0 ) {
    	symbol = 0;
    }
    else
    {
        symbol = 1;
        i = 4;
        while( cavs_biari_decode_symbol (&p->cabac, ctx + i)==0 )
        {
            ++symbol;
            ++i;
            if( i>=10 ) i=10;

			if( symbol > 29 ) /* NOTE : remove endless loop */
			{
				p->b_error_flag = 1;
				return -1;
			}
        }
    }

    p->b_error_flag = (&(p->cabac))->b_cabac_error;
    if( p->b_error_flag )
    {
        return -1;
    }
    
    if (symbol + p->ph.b_skip_mode_flag == 24 )
    	return I_8X8;
    else
    	return symbol + B_SKIP + p->ph.b_skip_mode_flag;
}

static const int map_intra_pred_mode[5] = {-1, 1, 2, 3, 0};
int cavs_cabac_get_intra_luma_pred_mode(cavs_decoder *p)
{
    bi_ctx_t * ctx = p->cabac.one_contexts[1];
    int i = 0;
    int symbol = 0;

    while (cavs_biari_decode_symbol(&p->cabac, ctx + i) == 0)
    {
        ++symbol;
        ++i;
        if (i>=3) i = 3;
        if (symbol == 4) break;
    }

    p->b_error_flag = (&(p->cabac))->b_cabac_error;
    p->pred_mode_flag = !symbol;

    return map_intra_pred_mode[symbol];
}

int cavs_cabac_get_intra_chroma_pred_mode(cavs_decoder *p)
{
    bi_ctx_t * ctx = p->cabac.cipr_contexts;
    /* 0 for chroma DC and -1 for not available(outside of slice or not intra) */
    int i = (cavs_mb_pred_mode8x8c[p->i_chroma_pred_mode_left] > 0)
    	+ (cavs_mb_pred_mode8x8c[p->p_chroma_pred_mode_top[p->i_mb_x]] > 0);

    int symbol;
    int bit;
    
    bit = cavs_biari_decode_symbol(&p->cabac, ctx + i);

    if (!bit)
        symbol = 0;
    else
    {
        bit = cavs_biari_decode_symbol(&p->cabac, ctx+3);
        if (!bit) symbol = 1;
        else
        {
            bit = cavs_biari_decode_symbol(&p->cabac, ctx+3);
            /*if (!tempval) symbol = 2;
            else symbol = 3;*/
            symbol = 2 + bit;
        }
    }

    p->b_error_flag = (&(p->cabac))->b_cabac_error;

    return symbol;
}

int cavs_cabac_get_cbp(cavs_decoder *p)
{
    bi_ctx_t * ctx = p->cabac.cbp_contexts[0];
    int bit, a, b;
    int symbol = 0;

    /* -1 for cbp not available(outside of slice) */
    /* block 0 */
    a = !(p->i_cbp_left & (1<<1));
    b = !(p->p_cbp_top[p->i_mb_x] & (1<<2));
    bit = cavs_biari_decode_symbol(&p->cabac, ctx + a + 2*b);
    symbol |= bit;

    /* block 1 */
    a = !bit;	/*we just get the zero count of left block*/
    b = !(p->p_cbp_top[p->i_mb_x] & (1<<3));
    bit = cavs_biari_decode_symbol(&p->cabac, ctx + a + 2*b);
    symbol |= (bit<<1);

    /* block 2 */
    a = !(p->i_cbp_left & (1<<3));
    b = !(symbol & 1);
    bit = cavs_biari_decode_symbol(&p->cabac, ctx + a + 2*b);
    symbol |= (bit<<2);

    /* block 3 */
    a = !bit;
    b = !(symbol & (1<<1));
    bit = cavs_biari_decode_symbol(&p->cabac, ctx + a + 2*b);
    symbol |= (bit<<3);

    ctx = p->cabac.cbp_contexts[1];
    bit = cavs_biari_decode_symbol(&p->cabac, ctx);
    if (bit)
    {
        bit = cavs_biari_decode_symbol(&p->cabac, ctx + 1);
        if (bit)
            symbol |= (3<<4);
        else
        {
            bit = cavs_biari_decode_symbol(&p->cabac, ctx + 1);
            symbol |= 1 << (4 + bit);
        }
    }

    p->b_error_flag = (&(p->cabac))->b_cabac_error;
    if( p->b_error_flag )
    {
        return -1;
    }

    return symbol;
}

int cavs_cabac_get_dqp(cavs_decoder *p)
{
    bi_ctx_t * ctx = p->cabac.delta_qp_contexts;
    int i = !!p->i_last_dqp;

    int dquant, l;
    int symbol = 1 - cavs_biari_decode_symbol(&p->cabac, ctx + i);
    
    if (symbol)
    {
        symbol = 1 - cavs_biari_decode_symbol(&p->cabac, ctx + 2);
        if (symbol)
        {
            symbol = 0;
            do {
                l = 1 - cavs_biari_decode_symbol(&p->cabac, ctx + 3);
                ++symbol;

				if( symbol > 126 ) /* NOTE : qp_delta can not exceed 63 */
				{
					p->b_error_flag = 1;
					return 0;
				}
            } while (l);
        }
        ++symbol;
    }

    dquant = (symbol+1)>>1;
    if((symbol & 0x01)==0)
    	dquant = -dquant;

    p->i_last_dqp = dquant;

    p->b_error_flag = (&(p->cabac))->b_cabac_error;

    return dquant;
}

int cavs_cabac_get_mvd(cavs_decoder *p, int i_list, int mvd_scan_idx, int xy_idx)
{
    const int mda = abs(p->p_mvd[i_list][mvd_scan_idx-1][xy_idx]);

    int i =  mda < 2 ? 0 : mda < 16 ? 1 : 2;
    bi_ctx_t * ctx = p->cabac.mv_res_contexts[xy_idx];
    int symbol, l, golomb_order = 0, bit = 0;

    if (!cavs_biari_decode_symbol(&p->cabac, ctx + i))
        symbol = 0;
    else if (!cavs_biari_decode_symbol(&p->cabac, ctx + 3))
        symbol = 1;
    else if (!cavs_biari_decode_symbol(&p->cabac, ctx + 4))
        symbol = 2;
    else if (!cavs_biari_decode_symbol(&p->cabac, ctx + 5))
    {
        symbol = 0;
        do {
            l = cavs_biari_decode_symbol_bypass(&p->cabac);
            if (!l)
            {
                symbol += (1<<golomb_order);
                ++golomb_order;

				if(symbol > 4096) /*remove endless loop*/
				{
					p->b_error_flag = 1;
					return -1;
				}
            }

            if( l!=0 && l!=1 )
            {
                	p->b_error_flag = 1;
                	return -1;
            }
        } while(l!=1);
        while (golomb_order--)
        {//next binary part
        	l = cavs_biari_decode_symbol_bypass(&p->cabac);
        	if (l == 1)
        		bit |= (1<<golomb_order);
				
								
            //if( l!=0 && l!=1 )
			//{
			//	p->b_error_flag = 1;
			//	return;
			//}
        }
        symbol += bit;
        symbol = 3 + symbol*2;
    }
    else
    {
        symbol = 0;
        do
        {
        	l = cavs_biari_decode_symbol_bypass(&p->cabac);
        	if (!l) 
        	{
        		symbol += (1<<golomb_order); 
        		++golomb_order;

				if(symbol > 4096) /*remove endless loop*/
				{
					p->b_error_flag = 1;
					return -1;
				}
        	}

		    if( l!=0 && l!=1 )
		    {
			    p->b_error_flag = 1;
			    return -1;
		    }
        } while (l!=1);
        while (golomb_order--)
        {//next binary part
        	l = cavs_biari_decode_symbol_bypass(&p->cabac);
        	if (l == 1)
        		bit |= (1<<golomb_order);
				
			//if( l!=0 && l!=1 )
			//{
			//	p->b_error_flag = 1;
			//	return;
			//}	
        }
        symbol += bit;
        symbol = 4 + symbol*2;
    }
    if (symbol)
    {
    	if (cavs_biari_decode_symbol_bypass(&p->cabac))
    		symbol = -symbol;
    }

    p->p_mvd[i_list][mvd_scan_idx][xy_idx] = symbol;

    p->b_error_flag = (&(p->cabac))->b_cabac_error;

    return symbol;
}

int cavs_cabac_get_ref_p(cavs_decoder *p, int ref_scan_idx)
{
    const int a = p->p_ref[0][ref_scan_idx-1] > 0;
    const int b = p->p_ref[0][ref_scan_idx-3] > 0;

    int i = a + (b<<1), symbol;
    bi_ctx_t * ctx = p->cabac.ref_no_contexts[0];

    if(p->ph.b_picture_reference_flag )
    {
        return 0;
    }

    if (cavs_biari_decode_symbol(&p->cabac, ctx + i))
        symbol = 0;
    else
    {
        symbol = 1;
        i = 4;
        while (!cavs_biari_decode_symbol(&p->cabac, ctx + i))
        {
            symbol++;
            i++;
            if (i >= 5) i = 5;

			if( symbol > 3 ) /* remove endless loop */
			{
				p->b_error_flag = 1;
				return -1;
			}
        }
    }

    p->p_ref[0][ref_scan_idx] = symbol;

    p->b_error_flag = (&(p->cabac))->b_cabac_error;

    return symbol;
}

static inline int cavs_cabac_get_ref_b_core(cavs_decoder *p, int i_list, int ref_scan_idx)
{
    const int a = p->p_ref[i_list][ref_scan_idx-1] > 0;
    const int b = p->p_ref[i_list][ref_scan_idx-3] > 0;

    int i = a + (b<<1);
    bi_ctx_t * ctx = p->cabac.ref_no_contexts[0];
    int symbol = !cavs_biari_decode_symbol(&p->cabac, ctx + i);

    p->p_ref[i_list][ref_scan_idx] = symbol;

    p->b_error_flag = (&(p->cabac))->b_cabac_error;

    return symbol;
}

int cavs_cabac_get_ref_b(cavs_decoder *p, int i_list, int ref_scan_idx)
{
    if(p->ph.b_picture_structure == 0 && p->ph.b_picture_reference_flag ==0)
        return cavs_cabac_get_ref_b_core(p, i_list, ref_scan_idx);
    return 0;
}


int cavs_cabac_get_mb_part_type(cavs_decoder *p)
{
    bi_ctx_t * ctx = p->cabac.b8_type_contexts[0];
    int i, symbol;
    
    if (cavs_biari_decode_symbol(&p->cabac, ctx + 0))
    {
        symbol = 2;
        i = 2;
    }
    else
    {
        symbol = 0;
        i = 1;
    }
    
    if (cavs_biari_decode_symbol(&p->cabac, ctx + i))
        ++symbol;

    p->b_error_flag = (&(p->cabac))->b_cabac_error;

    return symbol;
}

static const int t_chr[5] = { 0,1,2,4,3000};
int cavs_cabac_get_coeffs(cavs_decoder *p, const cavs_vlc *p_vlc_table, int i_escape_order, int b_chroma)
{
    int pairs, rank, pos;
    int run, level, abslevel, symbol;
    int *run_buf = p->run_buf, *level_buf = p->level_buf;

    /* read coefficients for whole block */
    bi_ctx_t (*primary)[NUM_MAP_CTX];
    bi_ctx_t *p_ctx;
    bi_ctx_t *p_ctx2;
    int ctx, ctx2, offset;
	int i_ret = 0;

    if( !b_chroma )
    {
        if( p->ph.b_picture_structure == 0 )
        {
            primary = p->cabac.fld_map_contexts;
        }
        else
        {
            primary = p->cabac.map_contexts;
        }
    }
    else 
    {
        if( p->ph.b_picture_structure == 0 )
        {
            primary = p->cabac.fld_last_contexts;
        }
        else
        {
            primary = p->cabac.last_contexts;
        }
    }
    
    //! Decode 
    rank = 0;
    pos = 0;
    for( pairs=0; pairs<65; pairs++ ) {
    	p_ctx = primary[rank];
    	//! EOB
    	if( rank>0) {
    		p_ctx2 = primary[5+(pos>>5)];
    		ctx2 = (pos>>1)&0x0f;
             /* note : can't cross border of array */
             if( pos < 0 || pos > 63 )
             {
                return -1;   
             }
    		ctx = 0;
			i_ret = cavs_biari_decode_symbol_w(&p->cabac, p_ctx+ctx, p_ctx2+ctx2);
			if( i_ret == -1)
			{
				p->b_error_flag = 1;
				return -1;
			}
			else if(i_ret!= 0 )
			{
				break;
			}
    	}
    	//! Level
    	ctx = 1;
    	symbol = 0;
    	while( cavs_biari_decode_symbol(&p->cabac, p_ctx+ctx)==0 ) {
    		++symbol;
    		++ctx;
    		if( ctx>=2 ) ctx =2;

			if(  symbol > (1<<15) ) /* remove endless loop */
			{
				p->b_error_flag = 1;
				return -1;
			}
    	}
    	abslevel = symbol + 1;
    	//! Sign
    	if( cavs_biari_decode_symbol_bypass(&p->cabac) ) {
    		level = - abslevel;	
    	}
    	else {
    		level = abslevel;
    	}

    	//! Run
    	if( abslevel==1 ) { 
    		offset = 4;
    	}
    	else {
    		offset = 6;
    	}
    	symbol = 0;
    	ctx = 0;
    	while( cavs_biari_decode_symbol(&p->cabac, p_ctx+ctx+offset)==0 ) {
    		++symbol;
    		++ctx;
    		if( ctx>=1 ) ctx =1;

			if(  symbol > 63 )	/* remove endless loop */
			{
				p->b_error_flag = 1;
				return -1;
			}
    	}
    	run = symbol;

    	level_buf[pairs] = level;
    	run_buf[pairs] = run + 1;
        
    	if( abslevel>t_chr[rank] ) {
    		if( abslevel <= 2 )
    			rank = abslevel;
    		else if( abslevel<=4 )
    			rank = 3;
    		else
    			rank = 4;
    	}
    	pos += (run+1);
    	if( pos>=64 ) pos = 63;
    }

    p->b_error_flag = (&(p->cabac))->b_cabac_error;
  
    return pairs;
}

#if B_MB_WEIGHTING
int cavs_cabac_get_mb_weighting_prediction(cavs_decoder *p)
{
	bi_ctx_t * ctx = &p->cabac.mb_weighting_pred;
	int ret = 0;

	ret = cavs_biari_decode_symbol (&p->cabac, ctx);

	return ret;
}
#endif
////////////////////////////////////////////////////////////////////////////////

cavs_decoder * cavs_frame_get (cavs_decoder ** list)
{
	cavs_decoder *frame = list[0];
	int i;
	for (i = 0; list[i]; i++)
		list[i] = list[i + 1];
	return frame;
}


int cavs_synch_frame_list_init( cavs_synch_frame_list_t *slist, int max_size )
{
	if( max_size < 0 )
		return -1;
	slist->i_max_size = max_size;
	slist->i_size = 0;
	CHECKED_MALLOCZERO( slist->list, (max_size+1) * sizeof(cavs_decoder *) );
	if( cavs_pthread_mutex_init( &slist->mutex, NULL ) ||
		cavs_pthread_cond_init( &slist->cv_fill, NULL ) ||
		cavs_pthread_cond_init( &slist->cv_empty, NULL ) )
		return -1;
	return 0;
fail:
	return -1;
}

void cavs_synch_frame_list_delete( cavs_synch_frame_list_t *slist )
{
	int i;
	if (!slist)
		return;

	cavs_pthread_mutex_destroy( &slist->mutex );
	cavs_pthread_cond_destroy( &slist->cv_fill );
	cavs_pthread_cond_destroy( &slist->cv_empty );

	for (i = 0; i < slist->i_max_size; i++) 
	{
		if (slist->list[i])
			cavs_decoder_destroy(slist->list[i]);
	}
	cavs_free(slist->list);
}

void cavs_synch_frame_list_push( cavs_synch_frame_list_t *slist, cavs_decoder *frame )
{
	cavs_pthread_mutex_lock( &slist->mutex );
	while( slist->i_size == slist->i_max_size )
		cavs_pthread_cond_wait( &slist->cv_empty, &slist->mutex );
	slist->list[ slist->i_size++ ] = frame;
	cavs_pthread_mutex_unlock( &slist->mutex );
	cavs_pthread_cond_broadcast( &slist->cv_fill );
}

cavs_decoder *cavs_synch_frame_list_pop( cavs_synch_frame_list_t *slist )
{
	cavs_decoder *frame;
	cavs_pthread_mutex_lock( &slist->mutex );
	while( !slist->i_size )
		cavs_pthread_cond_wait( &slist->cv_fill, &slist->mutex );
	frame = slist->list[ --slist->i_size ];
	slist->list[ slist->i_size ] = NULL;
	cavs_pthread_cond_broadcast( &slist->cv_empty );
	cavs_pthread_mutex_unlock( &slist->mutex );
	return frame;
}
////////////////////////////////////////////////////////////////////////////////
#define AH -1
#define BH -2
#define CH 96
#define DH 42
#define EH -7
#define FH 0
#define AV 0 
#define BV -1
#define CV 5
#define DV 5
#define EV -1
#define FV 0
#define MC_W 8
#define MC_H 8
/************************************************************************
D	a	b	c	E
d	e	f	g
h	i	j	k	m
n	p	q	r
************************************************************************/

#define put(a, b)  a = cm[((b)+512)>>10]

static void cavs_put_filt8_hv_ik(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    int32_t temp[8*(8+5)];
    int32_t *tmp = temp;
    uint8_t *cm = crop_table + MAX_NEG_CROP;
    int i,j;

    tmp = temp+2;  
    for(j=0;j<MC_H;j++)
    {
        for(i=-2; i<MC_W+3; i++)
        {
            tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];
        }
        tmp+=13;
        src1+=srcStride;
    }
    tmp = temp+2;                         
    for(j=0; j<MC_H; j++)                        
    {
        for(i=0;i<MC_W;i++)
        {
            put(dst[i], AH*tmp[-2+i] + BH*tmp[-1+i] + CH*tmp[0+i] + DH*tmp[1+i] + EH*tmp[2+i] );

        }
        dst+=dstStride;
        tmp+=13;                                                        
    }    
}
#undef put

#define put(a, b)  a = cm[((b)+512)>>10]
static void cavs_put_filt8_hv_ki(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{//k
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];			 
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], EH*tmp[-1+i] + DH*tmp[0+i] + CH*tmp[1+i] + BH*tmp[2+i] + AH*tmp[3+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }
     
}
#undef put



#define put(a, b)  a = cm[((b)+512)>>10]
static void cavs_put_filt8_hv_fq(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
	//f=-xx-2aa'+96b'+42s'-7dd'
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp;
	 src1-=2*srcStride;
	 for(j=-2;j<MC_H+3;j++)
	 {
		 for(i=0; i<MC_W; i++)
		 {
			 tmp[i]= BV*src1[-1+i] + CV*src1[0+i] + DV*src1[1+i] + EV*src1[2+i];			 
		 }
		 tmp+=8;
		 src1+=srcStride;
	 }
     tmp = temp+2*8;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], AH*tmp[-2*8+i] + BH*tmp[-1*8+i] + CH*tmp[0+i] + DH*tmp[1*8+i] + EH*tmp[2*8+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=8;                                                        
	 }                                                               
     
}
#undef put

#define put(a, b)  a = cm[((b)+512)>>10]
static void cavs_put_filt8_hv_qf(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
	//q
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp;
	 src1-=2*srcStride;
	 for(j=-2;j<MC_H+3;j++)
	 {
		 for(i=0; i<MC_W; i++)
		 {
			 tmp[i]= BV*src1[-1+i] + CV*src1[0+i] + DV*src1[1+i] + EV*src1[2+i];			 
		 }
		 tmp+=8;
		 src1+=srcStride;
	 }
     tmp = temp+2*8;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], EH*tmp[-1*8+i] + DH*tmp[0+i] + CH*tmp[1*8+i] + BH*tmp[2*8+i] + AH*tmp[3*8+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=8;                                                        
	 }                                                               
     
}
#undef put


#define put(a, b)  a = cm[((b)+32)>>6]
static void cavs_put_filt8_hv_j(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
	//j
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];			 
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i],  BV*tmp[-1+i] + CV*tmp[0+i] + DV*tmp[1+i] + EV*tmp[2+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }
     
}
#undef put


#define put(a, b)  a = cm[((b)+64)>>7]
static void cavs_put_filt8_h_ac(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ 
	//a

     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], AH*src1[-2+i] + BH*src1[-1+i] + CH*src1[0+i] + DH*src1[1+i] + EH*src1[2+i] ); 	//-1 -2 96 42 -7
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef put

#define put(a, b)  a = cm[((b)+64)>>7]
static void cavs_put_filt8_h_ca(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ 
	//c
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], EH*src1[-1+i] + DH*src1[0+i] + CH*src1[1+i] + BH*src1[2+i] + AH*src1[3+i] ); //-7 42 96 -2 -1
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef put


#define put(a, b)  a = cm[((b)+4)>>3]
static void cavs_put_filt8_h_b(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ //b -1 5 5 -1
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], BV*src1[-1+i] + CV*src1[0+i] + DV*src1[1+i] + EV*src1[2+i]);			 		
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef put
#define put(a, b)  a = cm[((b)+4)>>3]
static void cavs_put_filt8_v_h(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ //h
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i]);			 		
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef put

#define put(a, b)  a = cm[((b)+64)>>7]
static void cavs_put_filt8_v_dn(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{//d
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], AH*src1[-2*srcStride+i] + BH*src1[-1*srcStride+i] + CH*src1[0+i] + DH*src1[1*srcStride+i] + EH*src1[2*srcStride+i] ); //-1 -2 96 42 -7
			
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               

}
#undef put

#define put(a, b)  a = cm[((b)+64)>>7]
static void cavs_put_filt8_v_nd(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{//n
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], EH*src1[-1*srcStride+i] + DH*src1[0+i] + CH*src1[1*srcStride+i] + BH*src1[2*srcStride+i] + AH*src1[3*srcStride+i] ); //-7 42 96 -2 -1
			
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               

}
#undef put


#define put(a, b)  a = cm[((b)+64)>>7]
static void cavs_put_filt8_hv_egpr(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{//e¡¢g¡¢p¡¢r
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];	//h 
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 put(dst[i], BV*tmp[-1+i] + CV*tmp[0+i] + DV*tmp[1+i] + EV*tmp[2+i]+64*src2[0+i]); //(h->j+D)/2
			
		 }
		 src2+=dstStride;
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }                                                               
}
#undef put

#define avg(a, b)  a = ((a)+cm[((b)+512)>>10]+1)>>1
static void cavs_avg_filt8_hv_ik(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{	
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i]; //h
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], AH*tmp[-2+i] + BH*tmp[-1+i] + CH*tmp[0+i] + DH*tmp[1+i] + EH*tmp[2+i] ); //?? src1[]
			
		 }
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }
     
     
}
#undef avg

#define avg(a, b)  a = ((a)+cm[((b)+512)>>10]+1)>>1
static void cavs_avg_filt8_hv_ki(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{	
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];			 
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], EH*tmp[-1+i] + DH*tmp[0+i] + CH*tmp[1+i] + BH*tmp[2+i] + AH*tmp[3+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }
         
}
#undef avg




#define avg(a, b)  a = ((a)+cm[((b)+512)>>10]+1)>>1
static void cavs_avg_filt8_hv_fq(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
	//f=-xx-2aa'+96b'+42s'-7dd'
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp;
	 src1-=2*srcStride;
	 for(j=-2;j<MC_H+3;j++)
	 {
		 for(i=0; i<MC_W; i++)
		 {
			 tmp[i]= BV*src1[-1+i] + CV*src1[0+i] + DV*src1[1+i] + EV*src1[2+i];			 
		 }
		 tmp+=8;
		 src1+=srcStride;
	 }
     tmp = temp+2*8;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], AH*tmp[-2*8+i] + BH*tmp[-1*8+i] + CH*tmp[0+i] + DH*tmp[1*8+i] + EH*tmp[2*8+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=8;                                                        
	 }                                                               
     
}
#undef avg

#define avg(a, b)  a = ((a)+cm[((b)+512)>>10]+1)>>1
static void cavs_avg_filt8_hv_qf(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
	//f=-xx-2aa'+96b'+42s'-7dd'
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp;
	 src1-=2*srcStride;
	 for(j=-2;j<MC_H+3;j++)
	 {
		 for(i=0; i<MC_W; i++)
		 {
			 tmp[i]= BV*src1[-1+i] + CV*src1[0+i] + DV*src1[1+i] + EV*src1[2+i];			 
		 }
		 tmp+=8;
		 src1+=srcStride;
	 }
     tmp = temp+2*8;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], EH*tmp[-1*8+i] + DH*tmp[0+i] + CH*tmp[1*8+i] + BH*tmp[2*8+i] + AH*tmp[3*8+i] ); 
			
		 }
		 dst+=dstStride;
		 tmp+=8;                                                        
	 }                                                               
     
}
#undef avg



#define avg(a, b)  a = ((a)+cm[((b)+32)>>6]  +1)>>1
static void cavs_avg_filt8_hv_j(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{	//j
      int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];	//h		 
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i],  BV*tmp[-1+i] + CV*tmp[0+i] + DV*tmp[1+i] + EV*tmp[2+i] ); //j
			
		 }
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }
     
}
#undef avg


#define avg(a, b)  a = ((a)+cm[((b)+64)>>7]  +1)>>1
static void cavs_avg_filt8_h_ac(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ //a
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], AH*src1[-2+i] + BH*src1[-1+i] + CH*src1[0+i] + DH*src1[1+i] + EH*src1[2+i] ); //a
			
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef avg

#define avg(a, b)  a = ((a)+cm[((b)+64)>>7]  +1)>>1
static void cavs_avg_filt8_h_ca(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ //c
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], EH*src1[-1+i] + DH*src1[0+i] + CH*src1[1+i] + BH*src1[2+i] + AH*src1[3+i] ); //c
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef avg


#define avg(a, b)  a = ((a)+cm[((b)+4)>>3]   +1)>>1
static void cavs_avg_filt8_h_b(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ //b
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], BV*src1[-1+i] + CV*src1[0+i] + DV*src1[1+i] + EV*src1[2+i]);		//b	 		
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef avg



#define avg(a, b)  a = ((a)+cm[((b)+4)>>3]   +1)>>1
static void cavs_avg_filt8_v_h(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{ //h
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i]);			 		
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               
}
#undef avg



#define avg(a, b)  a = ((a)+cm[((b)+64)>>7]  +1)>>1
static void cavs_avg_filt8_v_dn(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{//dst+src1:d
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], AH*src1[-2*srcStride+i] + BH*src1[-1*srcStride+i] + CH*src1[0+i] + DH*src1[1*srcStride+i] + EH*src1[2*srcStride+i] ); 
			
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               

}
#undef avg

#define avg(a, b)  a = ((a)+cm[((b)+64)>>7]  +1)>>1
static void cavs_avg_filt8_v_nd(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{//n
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], EH*src1[-1*srcStride+i] + DH*src1[0+i] + CH*src1[1*srcStride+i] + BH*src1[2*srcStride+i] + AH*src1[3*srcStride+i] ); 
			
		 }
		 src1+=srcStride;
		 dst+=dstStride;
	 }                                                               

}
#undef avg


#define avg(a, b)  a = ((a)+cm[((b)+64)>>7]  +1)>>1
static void cavs_avg_filt8_hv_egpr(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{  //error
     int32_t temp[8*(8+5)];
     int32_t *tmp = temp;
     uint8_t *cm = crop_table + MAX_NEG_CROP;
     int i,j;
	 tmp = temp+2;
	 for(j=0;j<MC_H;j++)
	 {
		 for(i=-2; i<MC_W+3; i++)
		 {
			 tmp[i]= BV*src1[-1*srcStride+i] + CV*src1[0+i] + DV*src1[1*srcStride+i] + EV*src1[2*srcStride+i];  //h	 
		 }
		 tmp+=13;
		 src1+=srcStride;
	 }
     tmp = temp+2;                         
	 for(j=0; j<MC_H; j++)                        
	 {
		 for(i=0;i<MC_W;i++)
		 {
			 avg(dst[i], BV*tmp[-1+i] + CV*tmp[0+i] + DV*tmp[1+i] + EV*tmp[2+i]+64*src2[0+i]); // j??[-1 5 5 -1]
			
		 }
		 src2+=dstStride;
		 dst+=dstStride;
		 tmp+=13;                                                        
	 }                                                               
}
#undef avg

#undef AH 
#undef BH 
#undef CH 
#undef DH 
#undef EH 
#undef FH 
#undef AV  
#undef BV 
#undef CV 
#undef DV 
#undef EV 
#undef FV 
#undef MC_W 
#undef MC_H 

static void cavs_put_filt16_hv_fq(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_hv_fq(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_fq(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_hv_fq(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_fq(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_put_filt16_hv_qf(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_hv_qf(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_qf(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_hv_qf(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_qf(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_put_filt16_v_dn(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_v_dn(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_v_dn(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_v_dn(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_v_dn(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_put_filt16_v_nd(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_v_nd(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_v_nd(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_v_nd(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_v_nd(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_put_filt16_hv_egpr(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_hv_egpr(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_egpr(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_hv_egpr(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_egpr(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}
static void cavs_put_filt16_hv_j(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_hv_j(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_j(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_hv_j(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_j(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}



static void cavs_put_filt16_h_b(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_h_b(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_h_b(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_h_b(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_h_b(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_put_filt16_v_h(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_v_h(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_v_h(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_v_h(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_v_h(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_put_filt16_h_ac(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_h_ac(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_h_ac(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_h_ac(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_h_ac(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_put_filt16_h_ca(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_h_ca(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_h_ca(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_h_ca(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_h_ca(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_put_filt16_hv_ki(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_hv_ki(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_ki(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_hv_ki(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_ki(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_put_filt16_hv_ik(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_put_filt8_hv_ik(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_ik(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_put_filt8_hv_ik(dst  , src1,   src2  , dstStride, srcStride);
    cavs_put_filt8_hv_ik(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

void cavs_qpel16_put_mc10_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_h_ac(dst,src,src+stride+1,stride,stride);

}
void cavs_qpel16_put_mc20_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_h_b(dst,src,src+stride+1,stride,stride);

}

void cavs_qpel16_put_mc30_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_h_ca(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel16_put_mc01_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_v_dn(dst,src,src,stride,stride);

}

void cavs_qpel16_put_mc11_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_egpr(dst,src,src,stride,stride);

}

void cavs_qpel16_put_mc21_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_fq(dst,src,src,stride,stride);
}
void cavs_qpel16_put_mc31_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_egpr(dst,src,src+1,stride,stride);
}

void cavs_qpel16_put_mc02_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_v_h(dst,src,src+1,stride,stride);
}

void cavs_qpel16_put_mc12_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_ik(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_put_mc22_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_j(dst,src,src+1,stride,stride);
}

void cavs_qpel16_put_mc32_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_ki(dst,src,src+1,stride,stride);
}

void cavs_qpel16_put_mc03_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_v_nd(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_put_mc13_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_egpr(dst,src,src+stride,stride,stride);
}

void cavs_qpel16_put_mc23_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_qf(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_put_mc33_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt16_hv_egpr(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_put_mc10_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_h_ac(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel8_put_mc20_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_h_b(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_put_mc30_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_h_ca(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel8_put_mc01_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_v_dn(dst,src,src,stride,stride);
}

void cavs_qpel8_put_mc11_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_egpr(dst,src,src,stride,stride);
}

void cavs_qpel8_put_mc21_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_fq(dst,src,src,stride,stride);
}
void cavs_qpel8_put_mc31_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_egpr(dst,src,src+1,stride,stride);
}

void cavs_qpel8_put_mc02_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_v_h(dst,src,src+1,stride,stride);
}

void cavs_qpel8_put_mc12_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_ik(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_put_mc22_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_j(dst,src,src+1,stride,stride);
}

void cavs_qpel8_put_mc32_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_ki(dst,src,src+1,stride,stride);
}

void cavs_qpel8_put_mc03_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_v_nd(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_put_mc13_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_egpr(dst,src,src+stride,stride,stride);
}

void cavs_qpel8_put_mc23_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_qf(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_put_mc33_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_put_filt8_hv_egpr(dst,src,src+stride+1,stride,stride);
}

#define	BYTE_VEC32(c)	((c)*0x01010101UL)
static inline uint32_t rnd_avg32(uint32_t a, uint32_t b)
{
    return (a | b) - (((a ^ b) & ~BYTE_VEC32(0x01)) >> 1);
}

#define LD32(a) (*((uint32_t*)(a)))
#define PUT_PIX(OPNAME,OP) \
static void OPNAME ## _pixels8_c(uint8_t *block, const uint8_t *pixels, int line_size, int h){\
    int i;\
    for(i=0; i<h; i++){\
        OP(*((uint32_t*)(block  )), LD32(pixels  ));\
        OP(*((uint32_t*)(block+4)), LD32(pixels+4));\
        pixels+=line_size;\
        block +=line_size;\
    }\
}\
static void OPNAME ## _pixels16_c(uint8_t *block, const uint8_t *pixels, int line_size, int h){\
    OPNAME ## _pixels8_c(block  , pixels  , line_size, h);\
    OPNAME ## _pixels8_c(block+8, pixels+8, line_size, h);\
}

#define op_avg(a, b) a = rnd_avg32(a, b)
#define op_put(a, b) a = b
PUT_PIX(put,op_put);
PUT_PIX(avg,op_avg);

#undef op_avg
#undef op_put
#undef LD32

void cavs_qpel8_put_mc00_c(uint8_t *dst, uint8_t *src, int stride) 
 {
    put_pixels8_c(dst, src, stride, 8);
}
void cavs_qpel16_put_mc00_c(uint8_t *dst, uint8_t *src, int stride) 
{
    put_pixels16_c(dst, src, stride, 16);
}

void cavs_qpel8_avg_mc00_c(uint8_t *dst, uint8_t *src, int stride) 
{
    avg_pixels8_c(dst, src, stride, 8);
}

void cavs_qpel16_avg_mc00_c(uint8_t *dst, uint8_t *src, int stride) 
{
    avg_pixels16_c(dst, src, stride, 16);
}

static void cavs_avg_filt16_hv_fq(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_hv_fq(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_fq(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_hv_fq(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_fq(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_avg_filt16_hv_qf(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_hv_qf(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_qf(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_hv_qf(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_qf(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_avg_filt16_v_dn(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_v_dn(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_v_dn(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_v_dn(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_v_dn(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}
static void cavs_avg_filt16_v_nd(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_v_nd(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_v_nd(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_v_nd(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_v_nd(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}
static void cavs_avg_filt16_hv_egpr(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_hv_egpr(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_egpr(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_hv_egpr(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_egpr(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}
static void cavs_avg_filt16_hv_j(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_hv_j(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_j(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_hv_j(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_j(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_avg_filt16_h_b(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_h_b(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_h_b(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_h_b(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_h_b(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_avg_filt16_v_h(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_v_h(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_v_h(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_v_h(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_v_h(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_avg_filt16_h_ac(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_h_ac(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_h_ac(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_h_ac(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_h_ac(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_avg_filt16_h_ca(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_h_ca(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_h_ca(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_h_ca(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_h_ca(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}


static void cavs_avg_filt16_hv_ik(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_hv_ik(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_ik(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_hv_ik(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_ik(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

static void cavs_avg_filt16_hv_ki(uint8_t *dst, uint8_t *src1, uint8_t *src2, int dstStride, int srcStride)
{
    cavs_avg_filt8_hv_ki(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_ki(dst +8 , src1+8,   src2+8  , dstStride, srcStride);
    src1 += 8*srcStride;
    src2 += 8*srcStride;
    dst += 8*dstStride;
    cavs_avg_filt8_hv_ki(dst  , src1,   src2  , dstStride, srcStride);
    cavs_avg_filt8_hv_ki(dst +8, src1+8,   src2+8  , dstStride, srcStride);
}

void cavs_qpel16_avg_mc10_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_h_ac(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel16_avg_mc20_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_h_b(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_avg_mc30_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_h_ca(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel16_avg_mc01_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_v_dn(dst,src,src,stride,stride);
}

void cavs_qpel16_avg_mc11_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_egpr(dst,src,src,stride,stride);
}

void cavs_qpel16_avg_mc21_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_fq(dst,src,src,stride,stride);
}
void cavs_qpel16_avg_mc31_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_egpr(dst,src,src+1,stride,stride);
}

void cavs_qpel16_avg_mc02_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_v_h(dst,src,src+1,stride,stride);
}

void cavs_qpel16_avg_mc12_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_ik(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_avg_mc22_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_j(dst,src,src+1,stride,stride);
}

void cavs_qpel16_avg_mc32_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_ki(dst,src,src+1,stride,stride);
}

void cavs_qpel16_avg_mc03_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_v_nd(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_avg_mc13_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_egpr(dst,src,src+stride,stride,stride);
}

void cavs_qpel16_avg_mc23_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_qf(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel16_avg_mc33_c(uint8_t *dst, uint8_t *src, int stride)
{
    cavs_avg_filt16_hv_egpr(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_avg_mc10_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_h_ac(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel8_avg_mc20_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_h_b(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_avg_mc30_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_h_ca(dst,src,src+stride+1,stride,stride);
}
void cavs_qpel8_avg_mc01_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_v_dn(dst,src,src,stride,stride);
}

void cavs_qpel8_avg_mc11_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_egpr(dst,src,src,stride,stride);
}

void cavs_qpel8_avg_mc21_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_fq(dst,src,src,stride,stride);
}
void cavs_qpel8_avg_mc31_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_egpr(dst,src,src+1,stride,stride);
}

void cavs_qpel8_avg_mc02_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_v_h(dst,src,src+1,stride,stride);
}

void cavs_qpel8_avg_mc12_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_ik(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_avg_mc22_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_j(dst,src,src+1,stride,stride);
}

void cavs_qpel8_avg_mc32_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_ki(dst,src,src+1,stride,stride);
}

void cavs_qpel8_avg_mc03_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_v_nd(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_avg_mc13_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_egpr(dst,src,src+stride,stride,stride);
}

void cavs_qpel8_avg_mc23_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_qf(dst,src,src+stride+1,stride,stride);
}

void cavs_qpel8_avg_mc33_c(uint8_t *dst, uint8_t *src, int stride)
{
	cavs_avg_filt8_hv_egpr(dst,src,src+stride+1,stride,stride);
}

////////////////////////////////////////////////////////////////////////////////

#define MIN_QP          0
#define MAX_QP          63

//#pragma warning(disable:4996)

uint8_t crop_table[256 + 2 * MAX_NEG_CROP];

static int cavs_decoder_reset( void *p_decoder );

static int get_mb_i_aec_opt( cavs_decoder *p );
static int get_mb_i_rec_opt( cavs_decoder *p );
static int get_mb_p_aec_opt( cavs_decoder *p );
static int get_mb_p_rec_opt( cavs_decoder *p );
static int get_mb_b_aec_opt( cavs_decoder *p );
static int get_mb_b_rec_opt( cavs_decoder *p );

#if B_MB_WEIGHTING

static inline void pixel_avg_wxh( uint8_t *dst,  int i_dst,  uint8_t *src, int i_src, int width, int height )
{
    int x, y;
    
    for( y = 0; y < height; y++ )
    {
        for( x = 0; x < width; x++ )
            dst[x] = ( dst[x] + src[x] + 1 ) >> 1;
        src += i_src;
        dst += i_dst;
    }
}

static void pixel_avg_16x16_c( uint8_t *dst,  int i_dst,  uint8_t *src, int i_src )
{
    pixel_avg_wxh( dst,  i_dst,  src, i_src, 16, 16 );
}

static void pixel_avg_8x8_c( uint8_t *dst,  int i_dst,  uint8_t *src, int i_src )
{
    pixel_avg_wxh( dst,  i_dst,  src, i_src, 8, 8 );
}

static void pixel_avg_4x4_c( uint8_t *dst,  int i_dst,  uint8_t *src, int i_src )
{
    pixel_avg_wxh( dst,  i_dst,  src, i_src, 4, 4 );
}
#endif

static void init_crop_table(void)
{
    int i;
    
    for( i = 0; i < 256; i++ )
    {
        crop_table[i + MAX_NEG_CROP] = i;
    }

    memset(crop_table, 0, MAX_NEG_CROP*sizeof(uint8_t));
    memset(crop_table + MAX_NEG_CROP + 256, 255, MAX_NEG_CROP*sizeof(uint8_t));
}

void (*cavs_idct4_add)(uint8_t *dst, DCTELEM *block, int stride);
void (*cavs_idct8_add)(uint8_t *dst, DCTELEM *block, int stride);

void (*intra_pred_lum[8])(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top, uint8_t *p_left);
void (*intra_pred_lum_asm[8])(uint8_t *p_dest, uint8_t edge[33], int i_stride, uint8_t *p_top, uint8_t *p_left);

void (*intra_pred_lum_4x4[20])(uint8_t * src, uint8_t edge[17], int i_stride);
void (*intra_pred_chroma[7])(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left);
void (*intra_pred_chroma_asm[7])(uint8_t *p_dest, int neighbor, int i_stride, uint8_t *p_top,uint8_t *p_left);

static const uint8_t alpha_tab[64] = 
{
	0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,  3,
		4,  4,  5,  5,  6,  7,  8,  9, 10, 11, 12, 13, 15, 16, 18, 20,
		22, 24, 26, 28, 30, 33, 33, 35, 35, 36, 37, 37, 39, 39, 42, 44,
		46, 48, 50, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64
};

static const uint8_t beta_tab[64] = 
{
	0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,
		2,  2,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,  5,  5,  6,  6,
		6,  7,  7,  7,  8,  8,  8,  9,  9, 10, 10, 11, 11, 12, 13, 14,
		15, 16, 17, 18, 19, 20, 21, 22, 23, 23, 24, 24, 25, 25, 26, 27
};
static const uint8_t tc_tab[64] = 
{
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
		2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4,
		5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9
};

static void cavs_filter_lv_c(cavs_decoder *p, uint8_t *d, int stride, int  i_qp,
                          int bs1, int bs2) 
{
   int i;
   int alpha, beta, tc;
   int index_a, index_b;

   index_a = cavs_clip(i_qp + p->ph.i_alpha_c_offset,0,63);
   index_b = cavs_clip(i_qp + p->ph.i_beta_offset, 0,63);
   alpha = alpha_tab[index_a];
   beta  =  beta_tab[index_b];
   tc    =    tc_tab[index_a];
    
    if(bs1==2)
        for(i=0;i<16;i++)
             loop_filter_l2(d + i*stride,1,alpha,beta);
    else 
    {
        if(bs1)
            for(i=0;i<8;i++)
               loop_filter_l1(d + i*stride,1,alpha,beta,tc);
        if (bs2)
           for(i=8;i<16;i++)
               loop_filter_l1(d + i*stride,1,alpha,beta,tc);
    }
}

static void cavs_filter_lh_c(cavs_decoder *p, uint8_t *d, int stride, int i_qp, 
                                                int bs1, int bs2) 
{
   int i;
   int alpha, beta, tc;
   int index_a, index_b;

   index_a = cavs_clip(i_qp + p->ph.i_alpha_c_offset,0,63);
   index_b = cavs_clip(i_qp + p->ph.i_beta_offset, 0,63);
   alpha = alpha_tab[index_a];
   beta  =  beta_tab[index_b];
   tc    =    tc_tab[index_a];
   
   if(bs1==2)
        for(i=0;i<16;i++)
             loop_filter_l2(d + i,stride,alpha,beta);
    else {
         if(bs1)
            for(i=0;i<8;i++)
                 loop_filter_l1(d + i,stride,alpha,beta,tc);
         if (bs2)
            for(i=8;i<16;i++)
                 loop_filter_l1(d + i,stride,alpha,beta,tc);
     }
}
 
static void cavs_filter_cv_c(cavs_decoder *p, uint8_t *d, int stride, int i_qp, 
                                               int bs1, int bs2) 
{
    int i;
    int alpha, beta, tc;
    int index_a, index_b;

    index_a = cavs_clip(i_qp + p->ph.i_alpha_c_offset,0,63);
    index_b = cavs_clip(i_qp + p->ph.i_beta_offset, 0,63);
    alpha = alpha_tab[index_a];
    beta  =  beta_tab[index_b];
    tc    =    tc_tab[index_a];

     if(bs1==2)
        for(i=0;i<8;i++)
             loop_filter_c2(d + i*stride,1,alpha,beta);
     else 
     {
         if(bs1)
            for(i=0;i<4;i++)
                 loop_filter_c1(d + i*stride,1,alpha,beta,tc);
         if (bs2)
             for(i=4;i<8;i++)
                 loop_filter_c1(d + i*stride,1,alpha,beta,tc);
    }
}

static void cavs_filter_ch_c(cavs_decoder *p, uint8_t *d, int stride, int i_qp, 
                                                int bs1, int bs2) 
{
    int i;
    int alpha, beta, tc;
    int index_a, index_b;

    index_a = cavs_clip(i_qp + p->ph.i_alpha_c_offset,0,63);
    index_b = cavs_clip(i_qp + p->ph.i_beta_offset, 0,63);
    alpha = alpha_tab[index_a];
    beta  =  beta_tab[index_b];
    tc    =    tc_tab[index_a];
    
    if(bs1==2)
        for(i=0;i<8;i++)
             loop_filter_c2(d + i,stride,alpha,beta);
    else 
	{
         if(bs1)
            for(i=0;i<4;i++)
                loop_filter_c1(d + i,stride,alpha,beta,tc);
       if (bs2)
            for(i=4;i<8;i++)
                 loop_filter_c1(d + i,stride,alpha,beta,tc);
     }
}

static inline int get_cbp_4x4(int i_cbp_code)
{
	static const uint8_t cbp_tab_4x4[16] = 
	{
		15,0,13,12,11,7,14,3,10,5,4,1,2,8,9,6
	};
	return cbp_tab_4x4[i_cbp_code];
}

#define UNDETAILED 0
#define DETAILED   1

static const unsigned char WeightQuantModel[4][64]={
//   l a b c d h
//	 0 1 2 3 4 5
	{ 	
     // Mode 0
		0,0,0,4,4,4,5,5,
		0,0,3,3,3,3,5,5,
		0,3,2,2,1,1,5,5,
		4,3,2,2,1,5,5,5,
		4,3,1,1,5,5,5,5,
		4,3,1,5,5,5,5,5,
		5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5},
	{  
	  // Mode 1
		0,0,0,4,4,4,5,5,
		0,0,4,4,4,4,5,5,
		0,3,2,2,2,1,5,5,
		3,3,2,2,1,5,5,5,
		3,3,2,1,5,5,5,5,
		3,3,1,5,5,5,5,5,
		5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5},
	{   
       // Mode 2
		0,0,0,4,4,3,5,5,
		0,0,4,4,3,2,5,5,
		0,4,4,3,2,1,5,5,
		4,4,3,2,1,5,5,5,
		4,3,2,1,5,5,5,5,
		3,2,1,5,5,5,5,5,
		5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5},
	{  
		// Mode 3
		0,0,0,3,2,1,5,5,
		0,0,4,3,2,1,5,5,
		0,4,4,3,2,1,5,5,
		3,3,3,3,2,5,5,5,
		2,2,2,2,5,5,5,5,
		1,1,1,5,5,5,5,5,
		5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5}
};

short cavs_wq_param_default[2][6]=
{
	{135,143,143,160,160,213},   
	{128, 98,106,116,116,128}     
};

static void cavs_init_frame_quant_param( cavs_decoder *p )
{
    int i, j, k;

    if( p->ph.weighting_quant_flag )
       p->b_weight_quant_enable = 1;
    else
       p->b_weight_quant_enable = 0;	

     if( p->b_weight_quant_enable )
     {     
        p->UseDefaultScalingMatrixFlag[0] = p->UseDefaultScalingMatrixFlag[1] = 0;  // use weighed scaling matrix
     }
     else
     {
        p->UseDefaultScalingMatrixFlag[0] = p->UseDefaultScalingMatrixFlag[1] = 1;  // use default scaling matrix  
     }

    /* Patch the Weighting Parameters */
    if( !p->ph.mb_adapt_wq_disable )  //MBAWQ
    {
    	// Use default weighted parameters,    
    	for( i = 0; i < 2; i++ )
    		for( j = 0; j < 6; j++ )
    			p->wq_param[i][j] = cavs_wq_param_default[i][j];
    		
    	if(p->ph.weighting_quant_param_index != 0 )
    	{
    		if(p->ph.weighting_quant_param_index == 1 )
    		{
    			for( i = 0; i < 6; i++)
    				p->wq_param[UNDETAILED][i] = p->ph.weighting_quant_param_undetail[i];
    		}
    		else if( p->ph.weighting_quant_param_index == 2 )
    		{
    			for( i = 0; i < 6; i++)
    				p->wq_param[DETAILED][i] = p->ph.weighting_quant_param_detail[i];
    		}
    		else if( p->ph.weighting_quant_param_index == 3 )
    		{
    			for( i = 0; i < 6; i++ )
        			p->wq_param[UNDETAILED][i] = p->ph.weighting_quant_param_undetail[i];
    			for(i=0;i<6;i++)
        			p->wq_param[DETAILED][i] = p->ph.weighting_quant_param_detail[i];
    		}
    	}
    }
    else   // PAWQ
    {
    	for(i=0;i<2;i++)
    		for(j=0;j<6;j++)
    			p->wq_param[i][j]=128;
    		
    	if(p->ph.weighting_quant_param_index==0)
    	{
    		for(i=0;i<6;i++)
    			p->wq_param[DETAILED][i]=cavs_wq_param_default[DETAILED][i];
    	}
    	else if(p->ph.weighting_quant_param_index==1)
    	{
    		for(i=0;i<6;i++)
    			p->wq_param[UNDETAILED][i] = p->ph.weighting_quant_param_undetail[i];
    	}
    	if(p->ph.weighting_quant_param_index==2)
    	{
    		for(i=0;i<6;i++)
    			p->wq_param[DETAILED][i] = p->ph.weighting_quant_param_detail[i];
    	}
    		
    }

    for (i=0;i<64;i++)  
        p->cur_wq_matrix[i]=1<<7;

    /* Reconstruct the Weighting matrix */
    if( !p->b_weight_quant_enable )
    {  
        for(k=0;k<2;k++)        
    	  for(j=0;j<8;j++)
            for (i=0;i<8;i++)
              p->wq_matrix[k][j*8+i]=1<<7;
    }
    else
    {			
    for(k=0;k<2;k++) 
      for(j=0;j<8;j++)
        for(i=0;i<8;i++)
          p->wq_matrix[k][j*8+i]=(p->wq_param[k][WeightQuantModel[p->ph.weighting_quant_model/*CurrentSceneModel*/][j*8+i]]); 
    } 
}

static void cavs_frame_update_wq_matrix( cavs_decoder *p )
{
    int i;

    if( p->ph.weighting_quant_param_index == 0 )
    {
        for( i = 0; i < 64; i++ ) 
            p->cur_wq_matrix[i] = p->wq_matrix[DETAILED][i];     // Detailed weighted matrix
    }
    else if( p->ph.weighting_quant_param_index == 1 )
    {
        for( i = 0; i < 64; i++ ) 
            p->cur_wq_matrix[i] = p->wq_matrix[UNDETAILED][i];     // unDetailed weighted matrix
    }
    
    if( p->ph.weighting_quant_param_index == 2 )
    {
        for( i = 0; i < 64; i++ ) 
            p->cur_wq_matrix[i] = p->wq_matrix[DETAILED][i];     // Detailed weighted matrix
    }
}

static int cavs_free_resource(cavs_decoder *p)
{
    int i, i_edge;
    
    CAVS_SAFE_FREE(p->p_top_qp);
    CAVS_SAFE_FREE(p->p_top_mv[0]);
    CAVS_SAFE_FREE(p->p_top_mv[1]);
    CAVS_SAFE_FREE(p->p_top_intra_pred_mode_y);
    CAVS_SAFE_FREE(p->p_mb_type_top);
    CAVS_SAFE_FREE(p->p_chroma_pred_mode_top);
    CAVS_SAFE_FREE(p->p_cbp_top);
    CAVS_SAFE_FREE(p->p_ref_top);
    CAVS_SAFE_FREE(p->p_top_border_y);
    CAVS_SAFE_FREE(p->p_top_border_cb);
    CAVS_SAFE_FREE(p->p_top_border_cr);
    CAVS_SAFE_FREE(p->p_col_mv);
    CAVS_SAFE_FREE(p->p_col_type_base);
    CAVS_SAFE_FREE(p->p_block);

    if( p->param.b_accelerate )
    {
        CAVS_SAFE_FREE(p->level_buf_tab);
		CAVS_SAFE_FREE(p->run_buf_tab);
		CAVS_SAFE_FREE(p->num_buf_tab);

        CAVS_SAFE_FREE(p->i_mb_type_tab);
        CAVS_SAFE_FREE(p->i_qp_tab);
        CAVS_SAFE_FREE(p->i_cbp_tab);
        CAVS_SAFE_FREE(p->mv_tab);

        CAVS_SAFE_FREE(p->i_intra_pred_mode_y_tab);
        CAVS_SAFE_FREE(p->i_pred_mode_chroma_tab);
        CAVS_SAFE_FREE(p->p_mvd_tab);
        CAVS_SAFE_FREE(p->p_ref_tab);

#if B_MB_WEIGHTING
        CAVS_SAFE_FREE(p->weighting_prediction_tab);
#endif      
    }
    
    i_edge = (p->i_mb_width*CAVS_MB_SIZE + CAVS_EDGE*2)*2*17*2*2;
    if(p->p_edge)
    {
       p->p_edge -= i_edge/2;
       CAVS_SAFE_FREE(p->p_edge);	

    }

	for( i = 0; i < 3; i++ )
    {
    	if(p->image[i].p_data[0])
    	{
    		p->image[i].p_data[0]-=p->image[i].i_stride[0]*CAVS_EDGE+CAVS_EDGE;
    		CAVS_SAFE_FREE(p->image[i].p_data[0]);
    	}
    	
    	if(p->image[i].p_data[1])
    	{
    		p->image[i].p_data[1]-=p->image[i].i_stride[1]*CAVS_EDGE/2+CAVS_EDGE/2;
    		CAVS_SAFE_FREE(p->image[i].p_data[1]);
    	}
    	
    	if(p->image[i].p_data[2])
    	{
    		p->image[i].p_data[2]-=p->image[i].i_stride[2]*CAVS_EDGE/2+CAVS_EDGE/2;
    		CAVS_SAFE_FREE(p->image[i].p_data[2]);	
    	}
    	
    	memset(&p->image[i], 0, sizeof(cavs_image));
	}

	for (i = 0; i < 3; i++)
	{
		if (p->image_aec[i].p_data[0])
		{
			p->image_aec[i].p_data[0] -= p->image_aec[i].i_stride[0] * CAVS_EDGE + CAVS_EDGE;
			CAVS_SAFE_FREE(p->image_aec[i].p_data[0]);
		}

		if (p->image_aec[i].p_data[1])
		{
			p->image_aec[i].p_data[1] -= p->image_aec[i].i_stride[1] * CAVS_EDGE / 2 + CAVS_EDGE / 2;
			CAVS_SAFE_FREE(p->image_aec[i].p_data[1]);
		}

		if (p->image_aec[i].p_data[2])
		{
			p->image_aec[i].p_data[2] -= p->image_aec[i].i_stride[2] * CAVS_EDGE / 2 + CAVS_EDGE / 2;
			CAVS_SAFE_FREE(p->image_aec[i].p_data[2]);
		}

		memset(&p->image_aec[i], 0, sizeof(cavs_image));
	}

    return 0;
}

static int cavs_alloc_resource( cavs_decoder *p )
{
    uint32_t i;
    uint32_t i_mb_width, i_mb_height, i_edge;
    int b_interlaced = p->param.b_interlaced;
    
    i_mb_width = (p->vsh.i_horizontal_size+15)>>4;
    if ( p->vsh.b_progressive_sequence )
        i_mb_height = (p->vsh.i_vertical_size+15)>>4;
    else
        i_mb_height = ((p->vsh.i_vertical_size+31) & 0xffffffe0)>>4;

    if( i_mb_width == p->i_mb_width && i_mb_height == p->i_mb_height && p->p_block ) /* when p_block is NULL, can't return directly */
    {
		p->last_delayed_pframe = p->p_save[1]; /* for output delay frame */
        
		return 0;
    }
	else
	{
		if(p->i_mb_width != 0 && p->i_mb_height != 0) // resolution change is not allowed.
		{	
			memcpy(&p->vsh, &p->old, offsetof(cavs_video_sequence_header, b_flag_aec) - offsetof(cavs_video_sequence_header, i_profile_id)); /*NOTE : don't change aec flag and enable */
			if(p->p_block)
				return -1;
		}
		else
		{
			memcpy(&p->old, &p->vsh, sizeof(cavs_video_sequence_header));
		}
		
		/* need this */
		i_mb_width = (p->vsh.i_horizontal_size+15)>>4;
		if ( p->vsh.b_progressive_sequence )
			i_mb_height = (p->vsh.i_vertical_size+15)>>4;
		else
			i_mb_height = ((p->vsh.i_vertical_size+31) & 0xffffffe0)>>4;
	}

    cavs_free_resource(p);

    p->i_mb_width = i_mb_width;
    p->i_mb_height = i_mb_height;
    p->i_mb_num = i_mb_width*i_mb_height;
    p->i_mb_num_half = p->i_mb_num>>1;
    p->p_top_qp = (uint8_t *)cavs_malloc( p->i_mb_width);
    p->b_low_delay = p->vsh.b_low_delay;
    
    p->p_top_mv[0] = (cavs_vector *)cavs_malloc((p->i_mb_width*2+1)*sizeof(cavs_vector));
    p->p_top_mv[1] = (cavs_vector *)cavs_malloc((p->i_mb_width*2+1)*sizeof(cavs_vector));
    p->p_top_intra_pred_mode_y = (int *)cavs_malloc( p->i_mb_width*4*sizeof(*p->p_top_intra_pred_mode_y));
    p->p_mb_type_top = (int8_t *)cavs_malloc(p->i_mb_width*sizeof(int8_t));
    p->p_chroma_pred_mode_top = (int8_t *)cavs_malloc(p->i_mb_width*sizeof(int8_t));
    p->p_cbp_top = (int8_t *)cavs_malloc(p->i_mb_width*sizeof(int8_t));
    p->p_ref_top = (int8_t (*)[2][2])cavs_malloc(p->i_mb_width*2*2*sizeof(int8_t));
    p->p_top_border_y = (uint8_t *)cavs_malloc((p->i_mb_width+1)*CAVS_MB_SIZE);
    p->p_top_border_cb = (uint8_t *)cavs_malloc((p->i_mb_width)*10);
    p->p_top_border_cr = (uint8_t *)cavs_malloc((p->i_mb_width)*10);

    p->p_col_mv = (cavs_vector *)cavs_malloc( p->i_mb_width*(p->i_mb_height)*4*sizeof(cavs_vector));
    p->p_col_type_base = (uint8_t *)cavs_malloc(p->i_mb_width*(p->i_mb_height));
    p->p_block = (DCTELEM *)cavs_malloc(64*sizeof(DCTELEM));

    if( p->param.b_accelerate )
    {
        p->level_buf_tab = (int (*)[6][64])cavs_malloc(p->i_mb_num*6*64*sizeof(int));
        p->run_buf_tab = (int (*)[6][64])cavs_malloc(p->i_mb_num*6*64*sizeof(int));
        p->num_buf_tab = (int (*)[6])cavs_malloc(p->i_mb_num*6*sizeof(int));

        p->i_mb_type_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->i_qp_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->i_cbp_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->mv_tab = (cavs_vector (*)[24])cavs_malloc( p->i_mb_num*24*sizeof(cavs_vector));
	
        p->i_intra_pred_mode_y_tab = (int (*)[25])cavs_malloc(p->i_mb_num*25*sizeof(int));
        p->i_pred_mode_chroma_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->p_mvd_tab = (int16_t (*)[2][6][2])cavs_malloc(p->i_mb_num*2*6*2*sizeof(int16_t));
        p->p_ref_tab = (int8_t (*)[2][9])cavs_malloc(p->i_mb_num*2*9*sizeof(int8_t));

#if B_MB_WEIGHTING
        p->weighting_prediction_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
#endif      
    }

    i_edge = ( p->i_mb_width*CAVS_MB_SIZE + CAVS_EDGE*2 )*2*17*2*2;
    p->p_edge = (uint8_t *)cavs_malloc(i_edge);
    if( p->p_edge )
    {
        memset(p->p_edge, 0, i_edge);
    }
    p->p_edge += i_edge/2;

    if(   !p->p_top_qp                 ||!p->p_top_mv[0]          ||!p->p_top_mv[1]
    	||!p->p_top_intra_pred_mode_y  ||!p->p_top_border_y       ||!p->p_top_border_cb
    	||!p->p_top_border_cr          ||!p->p_col_mv             ||!p->p_col_type_base
    	||!p->p_block                  ||!p->p_edge				  ||!p->p_mb_type_top
    	||!p->p_chroma_pred_mode_top   ||!p->p_cbp_top			  ||!p->p_ref_top)
    {
    	return -1;
    }

    memset(p->p_block, 0, 64*sizeof(DCTELEM));

    if( p->param.b_accelerate )
    {
        for( i = 0; i < 3; i++ )
        {
            p->image_aec[i].i_colorspace = CAVS_CS_YUV420;
            p->image_aec[i].i_width = p->i_mb_width*CAVS_MB_SIZE;
            p->image_aec[i].i_height = p->i_mb_height*CAVS_MB_SIZE;

            if( !b_interlaced )
            {
                p->image_aec[i].i_stride[0] = p->image_aec[i].i_width + CAVS_EDGE*2;
                p->image_aec[i].i_stride[1] = p->image_aec[i].i_width/2 + CAVS_EDGE;
                p->image_aec[i].i_stride[2] = p->image_aec[i].i_width/2 + CAVS_EDGE;

                p->image_aec[i].p_data[0] = (uint8_t *)cavs_malloc(p->image_aec[i].i_stride[0]*(p->image_aec[i].i_height+CAVS_EDGE*2));
                p->image_aec[i].p_data[0] += p->image_aec[i].i_stride[0]*CAVS_EDGE + CAVS_EDGE;
                
                p->image_aec[i].p_data[1] = (uint8_t *)cavs_malloc(p->image_aec[i].i_stride[1]*(p->image_aec[i].i_height/2+CAVS_EDGE));
                p->image_aec[i].p_data[1] += p->image_aec[i].i_stride[1]*CAVS_EDGE/2 + CAVS_EDGE/2;

                p->image_aec[i].p_data[2] = (uint8_t *)cavs_malloc(p->image_aec[i].i_stride[2]*(p->image_aec[i].i_height/2+CAVS_EDGE));
                p->image_aec[i].p_data[2] += p->image_aec[i].i_stride[2]*CAVS_EDGE/2 + CAVS_EDGE/2;    
            }
            else
            {
                p->image_aec[i].i_stride[0] = (p->image_aec[i].i_width + CAVS_EDGE*2)<<b_interlaced;
                p->image_aec[i].i_stride[1] = (p->image_aec[i].i_width/2 + CAVS_EDGE)<<b_interlaced;
                p->image_aec[i].i_stride[2] = (p->image_aec[i].i_width/2 + CAVS_EDGE)<<b_interlaced;

                /*Y*/
                p->image_aec[i].p_data[0] = (uint8_t *)cavs_malloc( (p->image_aec[i].i_stride[0]>>b_interlaced)*(p->image_aec[i].i_height+((CAVS_EDGE*2)<<b_interlaced)));
                p->image_aec[i].p_data[0] += p->image_aec[i].i_stride[0]*CAVS_EDGE + CAVS_EDGE;

                /*U*/
                p->image_aec[i].p_data[1] = (uint8_t *)cavs_malloc( (p->image_aec[i].i_stride[1]>>b_interlaced)*(p->image_aec[i].i_height/2+(CAVS_EDGE<<b_interlaced)));            
                p->image_aec[i].p_data[1] += (p->image_aec[i].i_stride[1]*CAVS_EDGE/2) + CAVS_EDGE/2;

                /*V*/
                p->image_aec[i].p_data[2] = (uint8_t *)cavs_malloc( (p->image_aec[i].i_stride[2]>>b_interlaced)*(p->image_aec[i].i_height/2+(CAVS_EDGE<<b_interlaced)));            
                p->image_aec[i].p_data[2] += (p->image_aec[i].i_stride[2]*CAVS_EDGE/2) + CAVS_EDGE/2;
            }
             
            if( !p->image_aec[i].p_data[0] || !p->image_aec[i].p_data[1] || !p->image_aec[i].p_data[2] )
            {
            	return -1;
            }
        }

        p->p_save_aec[0] = &p->image_aec[0];
    }

    for( i = 0; i < 3; i++ )
    {
        p->image[i].i_colorspace = CAVS_CS_YUV420;
        p->image[i].i_width = p->i_mb_width*CAVS_MB_SIZE;
        p->image[i].i_height = p->i_mb_height*CAVS_MB_SIZE;

        if( !b_interlaced )
        {
            p->image[i].i_stride[0] = p->image[i].i_width + CAVS_EDGE*2;
            p->image[i].i_stride[1] = p->image[i].i_width/2 + CAVS_EDGE;
            p->image[i].i_stride[2] = p->image[i].i_width/2 + CAVS_EDGE;

            p->image[i].p_data[0] = (uint8_t *)cavs_malloc(p->image[i].i_stride[0]*(p->image[i].i_height+CAVS_EDGE*2));
            p->image[i].p_data[0] += p->image[i].i_stride[0]*CAVS_EDGE + CAVS_EDGE;

            p->image[i].p_data[1] = (uint8_t *)cavs_malloc(p->image[i].i_stride[1]*(p->image[i].i_height/2+CAVS_EDGE));
            p->image[i].p_data[1] += p->image[i].i_stride[1]*CAVS_EDGE/2 + CAVS_EDGE/2;

            p->image[i].p_data[2] = (uint8_t *)cavs_malloc(p->image[i].i_stride[2]*(p->image[i].i_height/2+CAVS_EDGE));
            p->image[i].p_data[2] += p->image[i].i_stride[2]*CAVS_EDGE/2 + CAVS_EDGE/2;    
        }
        else
        {
            p->image[i].i_stride[0] = (p->image[i].i_width + CAVS_EDGE*2)<<b_interlaced;
            p->image[i].i_stride[1] = (p->image[i].i_width/2 + CAVS_EDGE)<<b_interlaced;
            p->image[i].i_stride[2] = (p->image[i].i_width/2 + CAVS_EDGE)<<b_interlaced;

            /*Y*/
            p->image[i].p_data[0] = (uint8_t *)cavs_malloc( (p->image[i].i_stride[0]>>b_interlaced)*(p->image[i].i_height+((CAVS_EDGE*2)<<b_interlaced)));
            p->image[i].p_data[0] += p->image[i].i_stride[0]*CAVS_EDGE + CAVS_EDGE;

            /*U*/
            p->image[i].p_data[1] = (uint8_t *)cavs_malloc( (p->image[i].i_stride[1]>>b_interlaced)*(p->image[i].i_height/2+(CAVS_EDGE<<b_interlaced)));            
            p->image[i].p_data[1] += (p->image[i].i_stride[1]*CAVS_EDGE/2) + CAVS_EDGE/2;

            /*V*/
            p->image[i].p_data[2] = (uint8_t *)cavs_malloc( (p->image[i].i_stride[2]>>b_interlaced)*(p->image[i].i_height/2+(CAVS_EDGE<<b_interlaced)));            
            p->image[i].p_data[2] += (p->image[i].i_stride[2]*CAVS_EDGE/2) + CAVS_EDGE/2;
        }
         
        if( !p->image[i].p_data[0] || !p->image[i].p_data[1] || !p->image[i].p_data[2] )
        {
        	return -1;
        }
    }

    p->p_save[0] = &p->image[0];


    /*     
       0:    D3  B2  B3  C2
       4:    A1  X0  X1   -
       8:    A3  X2  X3   -
    */
    p->mv[ 7] = MV_NOT_AVAIL; 
    p->mv[19] = MV_NOT_AVAIL;

    if( p->param.b_accelerate )
    {
        for( i = 0;i < p->i_mb_num; i++)
        {
            p->mv_tab[i][ 7] = MV_NOT_AVAIL; 
            p->mv_tab[i][19] = MV_NOT_AVAIL;
        }
    }

#if B_MB_WEIGHTING
    /*creat a temp buffer for weighting prediction */
	if( !b_interlaced )
    {
        p->mb_line_buffer[0] = (uint8_t *)cavs_malloc(p->image[0].i_stride[0]*(p->image[0].i_height+CAVS_EDGE*2));
        p->mb_line_buffer[0] += p->image[0].i_stride[0]*CAVS_EDGE + CAVS_EDGE;

        p->mb_line_buffer[1] = (uint8_t *)cavs_malloc(p->image[0].i_stride[1]*(p->image[0].i_height/2+CAVS_EDGE));
        p->mb_line_buffer[1] += p->image[0].i_stride[1]*CAVS_EDGE/2 + CAVS_EDGE/2;

        p->mb_line_buffer[2] = (uint8_t *)cavs_malloc(p->image[0].i_stride[2]*(p->image[0].i_height/2+CAVS_EDGE));
        p->mb_line_buffer[2] += p->image[0].i_stride[2]*CAVS_EDGE/2 + CAVS_EDGE/2;    
    }
    else
    {
        /*Y*/
        p->mb_line_buffer[0] = (uint8_t *)cavs_malloc( (p->image[0].i_stride[0]>>b_interlaced)*(p->image[0].i_height+((CAVS_EDGE*2)<<b_interlaced)));
        p->mb_line_buffer[0] += p->image[0].i_stride[0]*CAVS_EDGE + CAVS_EDGE;

        /*U*/
        p->mb_line_buffer[1] = (uint8_t *)cavs_malloc( (p->image[0].i_stride[1]>>b_interlaced)*(p->image[0].i_height/2+(CAVS_EDGE<<b_interlaced)));            
        p->mb_line_buffer[1] += (p->image[0].i_stride[1]*CAVS_EDGE/2) + CAVS_EDGE/2;

        /*V*/
        p->mb_line_buffer[2] = (uint8_t *)cavs_malloc( (p->image[0].i_stride[2]>>b_interlaced)*(p->image[0].i_height/2+(CAVS_EDGE<<b_interlaced)));            
        p->mb_line_buffer[2] += (p->image[0].i_stride[2]*CAVS_EDGE/2) + CAVS_EDGE/2;
    }
#endif

    /* use to reset for different seq header */
    cavs_decoder_reset(p);

    return 0;
}


static void copy_mvs(cavs_vector *mv, enum cavs_block_size size) 
{
    switch(size)
    {
    case BLK_16X16:
        mv[MV_STRIDE  ] = mv[0];
        mv[MV_STRIDE+1] = mv[0];
    case BLK_16X8:
        mv[1] = mv[0];
        break;
    case BLK_8X16:
        mv[MV_STRIDE] = mv[0];
        break;
    default:
    	break;
    }
}

static void cavs_image_frame_to_field(cavs_image *p_dst, cavs_image *p_src, uint32_t b_bottom, uint32_t b_next_field)
{
    memcpy(p_dst, p_src, sizeof(cavs_image));   
    if( b_bottom )
    {
        p_dst->p_data[0] += (p_dst->i_stride[0]>>1);
        p_dst->p_data[1] += (p_dst->i_stride[1]>>1);
        p_dst->p_data[2] += (p_dst->i_stride[2]>>1);      
    }

    p_dst->i_distance_index = p_src->i_distance_index + b_next_field;
}

static void init_ref_list_for_aec(cavs_decoder *p,uint32_t b_next_field)
{
    int i;
    uint32_t b_top_field_first = p->ph.b_top_field_first;
    uint32_t b_bottom = (b_top_field_first && b_next_field) || ( !b_top_field_first && !b_next_field);
	
    for( i = 0; i < 4; i++ )
    {
        p->i_ref_distance[0] = 0;
        memset(&p->ref_aec[i], 0, sizeof(cavs_image));
    }
    
    p->p_save_aec[0]->b_picture_structure = p->ph.b_picture_structure;
    if(p->p_save_aec[1])
    {
        p->p_save_aec[0]->i_ref_distance[0] = p->p_save_aec[1]->i_distance_index;
    }
    if(p->p_save_aec[2])
    {
        p->p_save_aec[0]->i_ref_distance[1] = p->p_save_aec[2]->i_distance_index;
    }

    if(p->ph.b_picture_structure != 0) /* frame */
    {
        /* NOTE : use p->p_save[0] to init p->cur, especially, point to image memory */
        memcpy(&p->cur_aec, p->p_save_aec[0], sizeof(cavs_image)); 

        if( p->p_save_aec[1] )
        {
            memcpy(&p->ref_aec[0], p->p_save_aec[1], sizeof(cavs_image));
        }

        if( p->p_save_aec[2] )
        {
            memcpy(&p->ref_aec[1], p->p_save_aec[2], sizeof(cavs_image));	
        }

        if( p->ref_aec[0].p_data[0] )
        {	
        	p->i_ref_distance[0] = p->ph.i_picture_coding_type !=CAVS_B_PICTURE
        		?(p->cur_aec.i_distance_index - p->ref_aec[0].i_distance_index + 512) % 512
        		:(p->ref_aec[0].i_distance_index - p->cur_aec.i_distance_index + 512) % 512; 
        }
        
        if( p->ref_aec[1].p_data[0] )
        	p->i_ref_distance[1] = (p->cur_aec.i_distance_index - p->ref_aec[1].i_distance_index + 512) % 512;
    }
    else /* field */
    {
        /* init p_cur */
        cavs_image_frame_to_field(&p->cur_aec, p->p_save_aec[0], b_bottom, b_next_field);
        if( p->cur_aec.i_code_type == CAVS_I_PICTURE && b_next_field )
        {
            cavs_image_frame_to_field(&p->ref_aec[0], p->p_save_aec[0], !b_bottom, 0);	
        }
        else if( p->cur_aec.i_code_type == CAVS_P_PICTURE && p->p_save_aec[1] )
        {
            /* for P frame */
            if(b_next_field)
            {
                cavs_image_frame_to_field( &p->ref_aec[0], p->p_save_aec[0], !b_bottom, 0);
                cavs_image_frame_to_field( &p->ref_aec[1], p->p_save_aec[1], p->p_save_aec[1]->b_top_field_first, 1);	
                cavs_image_frame_to_field(&p->ref_aec[2], p->p_save_aec[1],  !p->p_save_aec[1]->b_top_field_first, 0);	

                if(p->p_save_aec[2])
                {
                    cavs_image_frame_to_field( &p->ref_aec[3], p->p_save_aec[2],p->p_save_aec[2]->b_top_field_first, 1);	
                }
            }
            else
            {
                cavs_image_frame_to_field( &p->ref_aec[0], p->p_save_aec[1], p->p_save_aec[1]->b_top_field_first, 1);	
                cavs_image_frame_to_field( &p->ref_aec[1], p->p_save_aec[1], !p->p_save_aec[1]->b_top_field_first, 0);	
                if(p->p_save_aec[2])
                {
                    cavs_image_frame_to_field( &p->ref_aec[2], p->p_save_aec[2],p->p_save_aec[2]->b_top_field_first, 1);	
                    cavs_image_frame_to_field( &p->ref_aec[3], p->p_save_aec[2], !p->p_save_aec[2]->b_top_field_first, 0);	
                }
            }
        }
        else if(p->cur_aec.i_code_type == CAVS_B_PICTURE && p->p_save_aec[2] && p->p_save_aec[1])
        {
        	cavs_image_frame_to_field( &p->ref_aec[0], p->p_save_aec[1], !p->p_save_aec[1]->b_top_field_first, 0);	
        	cavs_image_frame_to_field( &p->ref_aec[1], p->p_save_aec[1], p->p_save_aec[1]->b_top_field_first, 1);	
        	
        	cavs_image_frame_to_field( &p->ref_aec[2], p->p_save_aec[2], p->p_save_aec[2]->b_top_field_first, 1);	
        	cavs_image_frame_to_field( &p->ref_aec[3], p->p_save_aec[2], !p->p_save_aec[2]->b_top_field_first, 0);	
        }	
        for( i = 0; i < 2; i++ )
        {
            if(p->ref_aec[i].p_data[0])
            {
                p->i_ref_distance[i] = p->ph.i_picture_coding_type != CAVS_B_PICTURE
                	?(p->cur_aec.i_distance_index - p->ref_aec[i].i_distance_index + 512) % 512
                	:(p->ref_aec[i].i_distance_index - p->cur_aec.i_distance_index + 512) % 512; 
            }
        }
        for( i = 0; i < 2; i++)
        {
            if(p->ref_aec[2+i].p_data[0])
            {
                p->i_ref_distance[2+i] = (p->cur_aec.i_distance_index - p->ref_aec[2+i].i_distance_index + 512) % 512;
            }
        }
    }
    
    for( i = 0; i < 4; i++)
    {
        p->i_scale_den[i] = p->i_ref_distance[i] ? 512/p->i_ref_distance[i] : 0;
        if(p->ph.i_picture_coding_type != CAVS_B_PICTURE)
             p->i_direct_den[i] = p->i_ref_distance[i] ? 16384/p->i_ref_distance[i] : 0;
    }
    
    if( p->ph.i_picture_coding_type == CAVS_B_PICTURE) 
    {
        p->i_sym_factor = p->i_ref_distance[0]*p->i_scale_den[1];
    }
}

static void init_ref_list(cavs_decoder *p,uint32_t b_next_field)
{
    int i;
    uint32_t b_top_field_first = p->ph.b_top_field_first;
    uint32_t b_bottom = (b_top_field_first && b_next_field) || ( !b_top_field_first && !b_next_field);

    for( i = 0; i < 4; i++ )
    {
        p->i_ref_distance[0] = 0;
        memset(&p->ref[i], 0, sizeof(cavs_image));
    }
    
    p->p_save[0]->b_picture_structure = p->ph.b_picture_structure;
    if(p->p_save[1])
    {
        p->p_save[0]->i_ref_distance[0] = p->p_save[1]->i_distance_index;
    }
    if(p->p_save[2])
    {
        p->p_save[0]->i_ref_distance[1] = p->p_save[2]->i_distance_index;
    }

    if(p->ph.b_picture_structure != 0) /* frame */
    {
        /* NOTE : use p->p_save[0] to init p->cur, especially, point to image memory */
        memcpy(&p->cur, p->p_save[0], sizeof(cavs_image)); 

        if( p->p_save[1] )
        {
            memcpy(&p->ref[0], p->p_save[1], sizeof(cavs_image));
        }

        if( p->p_save[2] )
        {
            memcpy(&p->ref[1], p->p_save[2], sizeof(cavs_image));	
        }

        if( p->ref[0].p_data[0] )
        {	
        	p->i_ref_distance[0] = p->ph.i_picture_coding_type !=CAVS_B_PICTURE
        		?(p->cur.i_distance_index - p->ref[0].i_distance_index + 512) % 512
        		:(p->ref[0].i_distance_index - p->cur.i_distance_index + 512) % 512; 
        }
        
        if( p->ref[1].p_data[0] )
        	p->i_ref_distance[1] = (p->cur.i_distance_index - p->ref[1].i_distance_index + 512) % 512;
    }
    else /* field */
    {
        /* init p_cur */
        cavs_image_frame_to_field(&p->cur, p->p_save[0], b_bottom, b_next_field);
        if( p->cur.i_code_type == CAVS_I_PICTURE && b_next_field )
        {
            cavs_image_frame_to_field(&p->ref[0], p->p_save[0], !b_bottom, 0);	
        }
        else if( p->cur.i_code_type == CAVS_P_PICTURE && p->p_save[1] )
        {
            /* for P frame */
            if(b_next_field)
            {
                cavs_image_frame_to_field( &p->ref[0], p->p_save[0], !b_bottom, 0);	
                cavs_image_frame_to_field( &p->ref[1], p->p_save[1], p->p_save[1]->b_top_field_first, 1);	
                cavs_image_frame_to_field(&p->ref[2], p->p_save[1],  !p->p_save[1]->b_top_field_first, 0);
                if(p->p_save[2])
                {
                    cavs_image_frame_to_field( &p->ref[3], p->p_save[2],p->p_save[2]->b_top_field_first, 1);	
                }
            }
            else
            {
                cavs_image_frame_to_field( &p->ref[0], p->p_save[1], p->p_save[1]->b_top_field_first, 1);	
                cavs_image_frame_to_field( &p->ref[1], p->p_save[1], !p->p_save[1]->b_top_field_first, 0);	
                if(p->p_save[2])
                {
                    cavs_image_frame_to_field( &p->ref[2], p->p_save[2],p->p_save[2]->b_top_field_first, 1);	
                    cavs_image_frame_to_field( &p->ref[3], p->p_save[2], !p->p_save[2]->b_top_field_first, 0);	
                }
            }
        }
        else if(p->cur.i_code_type == CAVS_B_PICTURE && p->p_save[2] && p->p_save[1])
        {
        	cavs_image_frame_to_field( &p->ref[0], p->p_save[1], !p->p_save[1]->b_top_field_first, 0);	
        	cavs_image_frame_to_field( &p->ref[1], p->p_save[1], p->p_save[1]->b_top_field_first, 1);	

        	cavs_image_frame_to_field( &p->ref[2], p->p_save[2], p->p_save[2]->b_top_field_first, 1);	
        	cavs_image_frame_to_field( &p->ref[3], p->p_save[2], !p->p_save[2]->b_top_field_first, 0);	
        }	
        for( i = 0; i < 2; i++ )
        {
            if(p->ref[i].p_data[0])
            {
                p->i_ref_distance[i] = p->ph.i_picture_coding_type != CAVS_B_PICTURE
                	?(p->cur.i_distance_index - p->ref[i].i_distance_index + 512) % 512
                	:(p->ref[i].i_distance_index - p->cur.i_distance_index + 512) % 512; 
            }
        }
        for( i = 0; i < 2; i++)
        {
            if(p->ref[2+i].p_data[0])
            {
                p->i_ref_distance[2+i] = (p->cur.i_distance_index - p->ref[2+i].i_distance_index + 512) % 512;
            }
        }
    }
    
    for( i = 0; i < 4; i++)
    {
        p->i_scale_den[i] = p->i_ref_distance[i] ? 512/p->i_ref_distance[i] : 0;
        if(p->ph.i_picture_coding_type != CAVS_B_PICTURE)
             p->i_direct_den[i] = p->i_ref_distance[i] ? 16384/p->i_ref_distance[i] : 0;
    }
    
    if( p->ph.i_picture_coding_type == CAVS_B_PICTURE) 
    {
        p->i_sym_factor = p->i_ref_distance[0]*p->i_scale_den[1];
    }
    p->i_luma_offset[0] = 0;
    p->i_luma_offset[1] = 8;
    p->i_luma_offset[2] = 8*p->cur.i_stride[0];
    p->i_luma_offset[3] = p->i_luma_offset[2]+8;	
}

static int cavs_init_picture(cavs_decoder *p)
{
    int i;
    //int b_interlaced = p->param.b_interlaced;
    int b_bottom = p->b_bottom;
    
    for( i = 0; i <= 20; i += 4 )
        p->mv[i] = MV_NOT_AVAIL;
    p->mv[MV_FWD_X0]=p->mv[MV_BWD_X0] = MV_REF_DIR;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);

    p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
    p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;

    p->i_mb_x = p->i_mb_y = 0;
    p->i_mb_flags = 0;
    p->i_mb_index = 0;

    p->p_save[0]->i_distance_index = p->ph.i_picture_distance*2;
    p->p_save[0]->b_top_field_first = p->ph.b_top_field_first;
    p->p_save[0]->i_code_type = p->ph.i_picture_coding_type;

    init_ref_list(p, b_bottom);

    p->b_fixed_qp = p->ph.b_fixed_picture_qp;
    p->i_qp = p->ph.i_picture_qp;

    if (p->ph.asi_enable)
    {
        p->i_subpel_precision = 3;
        p->i_uint_length = 8;
    }
    else
    {
        p->i_subpel_precision = 2;
        p->i_uint_length = 4;
    }

    if (p->ph.b_aec_enable)
    {
        p->bs_read[SYNTAX_SKIP_RUN] = cavs_cabac_get_skip;
        p->bs_read[SYNTAX_MBTYPE_P] = cavs_cabac_get_mb_type_p;
        p->bs_read[SYNTAX_MBTYPE_B] = cavs_cabac_get_mb_type_b;
        p->bs_read[SYNTAX_INTRA_LUMA_PRED_MODE] = cavs_cabac_get_intra_luma_pred_mode;
        p->bs_read[SYNTAX_INTRA_CHROMA_PRED_MODE] = cavs_cabac_get_intra_chroma_pred_mode;
        p->bs_read[SYNTAX_INTRA_CBP] = cavs_cabac_get_cbp;
        p->bs_read[SYNTAX_INTER_CBP] = cavs_cabac_get_cbp;
        p->bs_read[SYNTAX_DQP] = cavs_cabac_get_dqp;
        p->bs_read[SYNTAX_MB_PART_TYPE] = cavs_cabac_get_mb_part_type;
        p->bs_read_mvd = cavs_cabac_get_mvd;
        p->bs_read_ref_p = cavs_cabac_get_ref_p;
        p->bs_read_ref_b = cavs_cabac_get_ref_b;
        p->bs_read_coeffs = cavs_cabac_get_coeffs;
    }
    else
    {
        p->bs_read[SYNTAX_SKIP_RUN] = cavs_cavlc_get_ue;
        p->bs_read[SYNTAX_MBTYPE_P] = cavs_cavlc_get_mb_type_p;
        p->bs_read[SYNTAX_MBTYPE_B] = cavs_cavlc_get_mb_type_b;
        p->bs_read[SYNTAX_INTRA_LUMA_PRED_MODE] = cavs_cavlc_get_intra_luma_pred_mode;
        p->bs_read[SYNTAX_INTRA_CHROMA_PRED_MODE] = cavs_cavlc_get_ue;
        p->bs_read[SYNTAX_INTRA_CBP] = cavs_cavlc_get_intra_cbp;
        p->bs_read[SYNTAX_INTER_CBP] = cavs_cavlc_get_inter_cbp;
        p->bs_read[SYNTAX_DQP] = cavs_cavlc_get_se;
        p->bs_read[SYNTAX_MB_PART_TYPE] = cavs_cavlc_get_mb_part_type;
        p->bs_read_mvd = cavs_cavlc_get_mvd;
        p->bs_read_ref_p = cavs_cavlc_get_ref_p;
        p->bs_read_ref_b = cavs_cavlc_get_ref_b;
        p->bs_read_coeffs = cavs_cavlc_get_coeffs;
    }
    
    return 0;
}

static int cavs_init_picture_context( cavs_decoder *p )
{
    int i;
    int b_interlaced = p->param.b_interlaced;
    int b_bottom = p->b_bottom;

    uint32_t k;

    for( k = (b_interlaced && b_bottom) ? p->i_mb_num_half : 0 ; k < p->i_mb_num; k++ ) /*when field, only init bottom.top has occupied by data yet*/
    {
        for( i = 0; i <= 20; i += 4 )
            p->mv_tab[k][i] = MV_NOT_AVAIL;
        p->mv_tab[k][MV_FWD_X0]=p->mv_tab[k][MV_BWD_X0] = MV_REF_DIR;
        copy_mvs(&p->mv_tab[k][MV_BWD_X0], BLK_16X16);
        copy_mvs(&p->mv_tab[k][MV_FWD_X0], BLK_16X16);

        p->i_intra_pred_mode_y_tab[k][5] = p->i_intra_pred_mode_y_tab[k][10] = 
        p->i_intra_pred_mode_y_tab[k][15] = p->i_intra_pred_mode_y_tab[k][20] = NOT_AVAIL;
    }
    
    for( i = 0; i <= 20; i += 4 )
        p->mv[i] = MV_NOT_AVAIL;
    p->mv[MV_FWD_X0]=p->mv[MV_BWD_X0] = MV_REF_DIR;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);

    p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
    p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;
    


    p->i_mb_x = p->i_mb_y = 0;
    p->i_mb_flags = 0;
    p->i_mb_index = 0;

	if( p->p_save_aec[0] == NULL )
		return CAVS_ERROR;
    p->p_save_aec[0]->i_distance_index = p->ph.i_picture_distance*2;
    p->p_save_aec[0]->b_top_field_first = p->ph.b_top_field_first;
    p->p_save_aec[0]->i_code_type = p->ph.i_picture_coding_type;
    
    p->b_fixed_qp = p->ph.b_fixed_picture_qp;
    p->i_qp = p->ph.i_picture_qp;

    if (p->ph.asi_enable)
    {
        p->i_subpel_precision = 3;
        p->i_uint_length = 8;
    }
    else
    {
        p->i_subpel_precision = 2;
        p->i_uint_length = 4;
    }

    if (p->ph.b_aec_enable)
    {
        p->bs_read[SYNTAX_SKIP_RUN] = cavs_cabac_get_skip;
        p->bs_read[SYNTAX_MBTYPE_P] = cavs_cabac_get_mb_type_p;
        p->bs_read[SYNTAX_MBTYPE_B] = cavs_cabac_get_mb_type_b;
        p->bs_read[SYNTAX_INTRA_LUMA_PRED_MODE] = cavs_cabac_get_intra_luma_pred_mode;
        p->bs_read[SYNTAX_INTRA_CHROMA_PRED_MODE] = cavs_cabac_get_intra_chroma_pred_mode;
        p->bs_read[SYNTAX_INTRA_CBP] = cavs_cabac_get_cbp;
        p->bs_read[SYNTAX_INTER_CBP] = cavs_cabac_get_cbp;
        p->bs_read[SYNTAX_DQP] = cavs_cabac_get_dqp;
        p->bs_read[SYNTAX_MB_PART_TYPE] = cavs_cabac_get_mb_part_type;
        p->bs_read_mvd = cavs_cabac_get_mvd;
        p->bs_read_ref_p = cavs_cabac_get_ref_p;
        p->bs_read_ref_b = cavs_cabac_get_ref_b;
        p->bs_read_coeffs = cavs_cabac_get_coeffs;
    }
    else
    {
        p->bs_read[SYNTAX_SKIP_RUN] = cavs_cavlc_get_ue;
        p->bs_read[SYNTAX_MBTYPE_P] = cavs_cavlc_get_mb_type_p;
        p->bs_read[SYNTAX_MBTYPE_B] = cavs_cavlc_get_mb_type_b;
        p->bs_read[SYNTAX_INTRA_LUMA_PRED_MODE] = cavs_cavlc_get_intra_luma_pred_mode;
        p->bs_read[SYNTAX_INTRA_CHROMA_PRED_MODE] = cavs_cavlc_get_ue;
        p->bs_read[SYNTAX_INTRA_CBP] = cavs_cavlc_get_intra_cbp;
        p->bs_read[SYNTAX_INTER_CBP] = cavs_cavlc_get_inter_cbp;
        p->bs_read[SYNTAX_DQP] = cavs_cavlc_get_se;
        p->bs_read[SYNTAX_MB_PART_TYPE] = cavs_cavlc_get_mb_part_type;
        p->bs_read_mvd = cavs_cavlc_get_mvd;
        p->bs_read_ref_p = cavs_cavlc_get_ref_p;
        p->bs_read_ref_b = cavs_cavlc_get_ref_b;
        p->bs_read_coeffs = cavs_cavlc_get_coeffs;
    }

    return 0;
}

static int cavs_init_picture_context_frame( cavs_decoder *p )
{
    p->p_save[0]->i_distance_index = p->ph.i_picture_distance*2;
    p->p_save[0]->b_top_field_first = p->ph.b_top_field_first;
    p->p_save[0]->i_code_type = p->ph.i_picture_coding_type;
    p->i_mb_x = p->i_mb_y = 0;
    p->i_mb_flags = 0;
    p->i_mb_index = 0;
    
    return 0;
}

static int cavs_init_picture_context_fld( cavs_decoder *p )
{
    p->p_save[0]->i_distance_index = p->ph.i_picture_distance*2;
    p->p_save[0]->b_top_field_first = p->ph.b_top_field_first;
    p->p_save[0]->i_code_type = p->ph.i_picture_coding_type;
    p->i_mb_x = p->i_mb_y = 0;
    p->i_mb_flags = 0;
    p->i_mb_index = 0;
    
    return 0;
}

static int cavs_init_picture_ref_list( cavs_decoder *p )
{
    int b_bottom = p->b_bottom;
    
    init_ref_list(p, b_bottom);

    return 0;
}

static int cavs_init_picture_ref_list_for_aec( cavs_decoder *p )
{
    //int b_interlaced = p->param.b_interlaced;
    int b_bottom = p->b_bottom;

    init_ref_list_for_aec(p, b_bottom);

    return 0;
}


static int cavs_init_slice(cavs_decoder *p)
{
    int i_y;
    
    p->i_mb_offset = ( !p->ph.b_picture_structure && p->i_mb_index >= p->i_mb_num_half ) ? (p->i_mb_height>>1) : 0;
    i_y = p->i_mb_y - p->i_mb_offset;
    if( p->i_mb_offset )
    {
        init_ref_list(p, 1); /* init ref list of bottom-field */
    }
    
    memset( p->p_top_intra_pred_mode_y, NOT_AVAIL, p->i_mb_width*4*sizeof(*p->p_top_intra_pred_mode_y) );
    p->p_y = p->cur.p_data[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
    p->p_cb = p->cur.p_data[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
    p->p_cr = p->cur.p_data[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];

#if B_MB_WEIGHTING
    p->p_back_y = p->mb_line_buffer[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
    p->p_back_cb = p->mb_line_buffer[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
    p->p_back_cr = p->mb_line_buffer[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];
#endif

    p->i_mb_flags = 0;
    p->i_mb_x = 0;
    p->b_first_line = 1;
    p->b_error_flag = 0;
#if B_MB_WEIGHTING
	p->weighting_prediction = 0;
#endif

    if (p->ph.b_aec_enable)
    {
        cavs_bitstream_align(&p->s);
        cavs_cabac_context_init(p);
        cavs_cabac_start_decoding(&p->cabac, &p->s);

        /*syntax context init*/
        memset(p->p_mb_type_top, NOT_AVAIL, p->i_mb_width*sizeof(int8_t));
        memset(p->p_chroma_pred_mode_top, 0, p->i_mb_width*sizeof(int8_t));
        memset(p->p_cbp_top, -1, p->i_mb_width*sizeof(int8_t));
        memset(p->p_ref_top, NOT_AVAIL, p->i_mb_width*2*2*sizeof(int8_t));
        p->i_last_dqp = 0;
        p->i_mb_type_left = -1;
    }
    
    return 0;
}

static int cavs_init_slice_for_aec(cavs_decoder *p)
{
    int i_y;
    
    p->i_mb_offset = ( !p->ph.b_picture_structure && p->i_mb_index >= p->i_mb_num_half ) ? (p->i_mb_height>>1) : 0;
    i_y = p->i_mb_y - p->i_mb_offset;
    
    memset( p->p_top_intra_pred_mode_y, NOT_AVAIL, p->i_mb_width*4*sizeof(*p->p_top_intra_pred_mode_y) );
    p->p_y = p->cur.p_data[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
    p->p_cb = p->cur.p_data[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
    p->p_cr = p->cur.p_data[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];

#if B_MB_WEIGHTING
    p->p_back_y = p->mb_line_buffer[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
    p->p_back_cb = p->mb_line_buffer[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
    p->p_back_cr = p->mb_line_buffer[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];
#endif

    p->i_mb_flags = 0;
    p->i_mb_x = 0;
    p->b_first_line = 1;
    p->b_error_flag = 0;
#if B_MB_WEIGHTING	
	p->weighting_prediction = 0;
#endif

    if (p->ph.b_aec_enable)
    {
        cavs_bitstream_align(&p->s);
        cavs_cabac_context_init(p);
        cavs_cabac_start_decoding(&p->cabac, &p->s);

        /*syntax context init*/
        memset(p->p_mb_type_top, NOT_AVAIL, p->i_mb_width*sizeof(int8_t));
        memset(p->p_chroma_pred_mode_top, 0, p->i_mb_width*sizeof(int8_t));
        memset(p->p_cbp_top, -1, p->i_mb_width*sizeof(int8_t));
        memset(p->p_ref_top, NOT_AVAIL, p->i_mb_width*2*2*sizeof(int8_t));
        p->i_last_dqp = 0;
        p->i_mb_type_left = -1;
    }
    
    return 0;
}

static int cavs_init_slice_for_rec(cavs_decoder *p)
{
    int i_y;

    p->i_mb_offset = ( !p->ph.b_picture_structure && p->i_mb_index >= p->i_mb_num_half ) ? (p->i_mb_height>>1) : 0;
    i_y = p->i_mb_y - p->i_mb_offset;
    
    memset( p->p_top_intra_pred_mode_y, NOT_AVAIL, p->i_mb_width*4*sizeof(*p->p_top_intra_pred_mode_y) );
    p->p_y = p->cur.p_data[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
    p->p_cb = p->cur.p_data[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
    p->p_cr = p->cur.p_data[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];

#if B_MB_WEIGHTING
    p->p_back_y = p->mb_line_buffer[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
    p->p_back_cb = p->mb_line_buffer[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
    p->p_back_cr = p->mb_line_buffer[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];
#endif

    p->i_mb_flags = 0;
    p->i_mb_x = 0;
    p->b_first_line = 1;
    p->b_error_flag = 0;

    return 0;
}

/**
 * in-loop deblocking filter for a single macroblock
 *
 * boundary strength (bs) mapping:
 *
 * --4---5--
 * 0   2   |
 * | 6 | 7 |
 * 1   3   |
 * ---------
 *
 */
static inline int get_bs(cavs_vector *mvP, cavs_vector *mvQ, int b, int i_unit_length) 
{
    if((mvP->ref == REF_INTRA) || (mvQ->ref == REF_INTRA))
        return 2;
    if( mvP->ref != mvQ->ref || (abs(mvP->x - mvQ->x) >= i_unit_length) ||  (abs(mvP->y - mvQ->y) >= i_unit_length) )
        return 1;
    if(b)
	{
        mvP += MV_BWD_OFFS;
        mvQ += MV_BWD_OFFS;
        if( mvP->ref != mvQ->ref || (abs(mvP->x - mvQ->x) >= i_unit_length) ||  (abs(mvP->y - mvQ->y) >= i_unit_length) )
            return 1;
    }/*else{
        if(mvP->ref != mvQ->ref)
            return 1;
    }*/
    return 0;
}

static void filter_mb(cavs_decoder *p, int i_mb_type) 
{
    uint8_t bs[8];
    int qp_avg;
    int qp_avg_c;
    int i;
    int qp_avg_cb, qp_avg_cr;

    /* save c[0] or r[0] */
    p->i_topleft_border_y = p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE+CAVS_MB_SIZE-1];//0~15
    p->i_topleft_border_cb = p->p_top_border_cb[p->i_mb_x*10+8];//0~9
    p->i_topleft_border_cr = p->p_top_border_cr[p->i_mb_x*10+8];
    memcpy(&p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE], p->p_y + 15* p->cur.i_stride[0], 16);
    
    /* point 0 decided by topleft_border */
    memcpy(&p->p_top_border_cb[p->i_mb_x*10+1], p->p_cb +  7*  p->cur.i_stride[1], 8);
    memcpy(&p->p_top_border_cr[p->i_mb_x*10+1], p->p_cr +  7*  p->cur.i_stride[2], 8);
    for(i=0;i<8;i++) 
    {
    	p->i_left_border_y[i*2+1] = *(p->p_y + 15 + (i*2+0)*p->cur.i_stride[0]);
    	p->i_left_border_y[i*2+2] = *(p->p_y + 15 + (i*2+1)*p->cur.i_stride[0]);
    	p->i_left_border_cb[i+1] = *(p->p_cb + 7 + i*p->cur.i_stride[1]);
    	p->i_left_border_cr[i+1] = *(p->p_cr + 7 + i*p->cur.i_stride[2]);
    }
    memset(&p->i_left_border_y[17], p->i_left_border_y[16], 9);

    if( p->param.b_accelerate )
    {
        if(!p->ph.b_loop_filter_disable) 
        {
            /* determine bs */
            if(i_mb_type == I_8X8)
            {
                *((uint64_t *)bs) = 0x0202020202020202;
            }
            else
            {
                /** mv motion vector cache
                0:    D3  B2  B3  C2
                4:    A1  X0  X1   -
                8:    A3  X2  X3   -
                i_mb_type > P_8X8即表示B帧
                bs[8]一共8个元素分别表示最多以4个8*8的块分割的4个垂直边界和4个水平边界，每个边界是8个象素
                对于16*8或者8*16应该考虑其是否具有中间分割的两个水平和垂直边界
                0，1，4，5应该是宏块边界上的2个水平和垂直边界
                而2，3是宏块内垂直边界而，6，7是宏块内水平边界
                */
                *((uint64_t *)bs) = 0;
                if(partition_flags[i_mb_type] & SPLITV)
            	{
                    bs[2] = get_bs(&p->mv[MV_FWD_X0], &p->mv[MV_FWD_X1], i_mb_type > P_8X8, p->i_uint_length);
                    bs[3] = get_bs(&p->mv[MV_FWD_X2], &p->mv[MV_FWD_X3], i_mb_type > P_8X8, p->i_uint_length);
                }
                if(partition_flags[i_mb_type] & SPLITH)
            	{
                    bs[6] = get_bs(&p->mv[MV_FWD_X0], &p->mv[MV_FWD_X2], i_mb_type > P_8X8, p->i_uint_length);
                    bs[7] = get_bs(&p->mv[MV_FWD_X1], &p->mv[MV_FWD_X3], i_mb_type > P_8X8, p->i_uint_length);
                }
                bs[0] = get_bs(&p->mv[MV_FWD_A1], &p->mv[MV_FWD_X0], i_mb_type > P_8X8, p->i_uint_length);
                bs[1] = get_bs(&p->mv[MV_FWD_A3], &p->mv[MV_FWD_X2], i_mb_type > P_8X8, p->i_uint_length);
                bs[4] = get_bs(&p->mv[MV_FWD_B2], &p->mv[MV_FWD_X0], i_mb_type > P_8X8, p->i_uint_length);
                bs[5] = get_bs(&p->mv[MV_FWD_B3], &p->mv[MV_FWD_X1], i_mb_type > P_8X8, p->i_uint_length);
            }
            if( *((uint64_t *)bs) ) 
            {
            	if(p->i_mb_flags & A_AVAIL) 
            	{
            	   /* MB verticl edge */
                    qp_avg = (p->i_qp_tab[p->i_mb_index] + p->i_left_qp + 1) >> 1;

                    /* FIXIT : weighting quant qp for chroma
                        NOTE : not modify p->i_qp directly
                    */
                    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
                    { 
                        qp_avg_cb = (chroma_qp[clip3_int(p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_u, 0, 63 )]
                            +  chroma_qp[clip3_int(p->i_left_qp + p->ph.chroma_quant_param_delta_u, 0, 63)] + 1) >>1;
                        qp_avg_cr = (chroma_qp[clip3_int( p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_v, 0, 63 )]
                            +  chroma_qp[clip3_int( p->i_left_qp + p->ph.chroma_quant_param_delta_v, 0, 63)] + 1) >> 1; 

                        p->filter_lv(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cb, p->cur.i_stride[1], qp_avg_cb, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cr, p->cur.i_stride[2], qp_avg_cr, bs[0], bs[1]);            
                    }
                    else
                    {
                        qp_avg_c = (chroma_qp[p->i_qp_tab[p->i_mb_index]] + chroma_qp[p->i_left_qp] + 1) >> 1;

                        p->filter_lv(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cb, p->cur.i_stride[1], qp_avg_c, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cr, p->cur.i_stride[2], qp_avg_c, bs[0], bs[1]);
                    }
                }
                qp_avg = p->i_qp_tab[p->i_mb_index];

                p->filter_lv(p, p->p_y + 8, p->cur.i_stride[0], qp_avg, bs[2], bs[3]);
                p->filter_lh(p, p->p_y + 8*p->cur.i_stride[0], p->cur.i_stride[0], qp_avg, bs[6], bs[7]);

                if(p->i_mb_flags & B_AVAIL) 
                {
            	    /* MB horizontal edge */
                    qp_avg = (p->i_qp_tab[p->i_mb_index] + p->p_top_qp[p->i_mb_x] + 1) >> 1;
                    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
                    {
                        qp_avg_cb = (chroma_qp[clip3_int( p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_u,0, 63 )]
                            +  chroma_qp[clip3_int( p->p_top_qp[p->i_mb_x] + p->ph.chroma_quant_param_delta_u, 0, 63 )] + 1) >>1;
                        qp_avg_cr = (chroma_qp[clip3_int(p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_v ,0, 63 )]
                            +  chroma_qp[clip3_int( p->p_top_qp[p->i_mb_x] + p->ph.chroma_quant_param_delta_v, 0, 63 )] + 1) >> 1;

                        p->filter_lh(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cb, p->cur.i_stride[1], qp_avg_cb, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cr, p->cur.i_stride[2], qp_avg_cr, bs[4], bs[5]);
                    }
                    else
                    {
                        qp_avg_c = (chroma_qp[p->i_qp_tab[p->i_mb_index]] + chroma_qp[p->p_top_qp[p->i_mb_x]] + 1) >> 1;

                        p->filter_lh(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cb, p->cur.i_stride[1], qp_avg_c, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cr, p->cur.i_stride[2], qp_avg_c, bs[4], bs[5]);
                    }
                }
            }
        }

        p->i_left_qp = p->i_qp_tab[p->i_mb_index];
        p->p_top_qp[p->i_mb_x] = p->i_qp_tab[p->i_mb_index];
    }
    else
    {
        if(!p->ph.b_loop_filter_disable) 
        {
            /* determine bs */
            if(i_mb_type == I_8X8)
            {
                *((uint64_t *)bs) = 0x0202020202020202;
            }
            else
            {
                /** mv motion vector cache
                0:    D3  B2  B3  C2
                4:    A1  X0  X1   -
                8:    A3  X2  X3   -
                i_mb_type > P_8X8即表示B帧
                bs[8]一共8个元素分别表示最多以4个8*8的块分割的4个垂直边界和4个水平边界，每个边界是8个象素
                对于16*8或者8*16应该考虑其是否具有中间分割的两个水平和垂直边界
                0，1，4，5应该是宏块边界上的2个水平和垂直边界
                而2，3是宏块内垂直边界而，6，7是宏块内水平边界
                */
                *((uint64_t *)bs) = 0;
                if(partition_flags[i_mb_type] & SPLITV)
            	{
                    bs[2] = get_bs(&p->mv[MV_FWD_X0], &p->mv[MV_FWD_X1], i_mb_type > P_8X8, p->i_uint_length);
                    bs[3] = get_bs(&p->mv[MV_FWD_X2], &p->mv[MV_FWD_X3], i_mb_type > P_8X8, p->i_uint_length);
                }
                if(partition_flags[i_mb_type] & SPLITH)
            	{
                    bs[6] = get_bs(&p->mv[MV_FWD_X0], &p->mv[MV_FWD_X2], i_mb_type > P_8X8, p->i_uint_length);
                    bs[7] = get_bs(&p->mv[MV_FWD_X1], &p->mv[MV_FWD_X3], i_mb_type > P_8X8, p->i_uint_length);
                }
                bs[0] = get_bs(&p->mv[MV_FWD_A1], &p->mv[MV_FWD_X0], i_mb_type > P_8X8, p->i_uint_length);
                bs[1] = get_bs(&p->mv[MV_FWD_A3], &p->mv[MV_FWD_X2], i_mb_type > P_8X8, p->i_uint_length);
                bs[4] = get_bs(&p->mv[MV_FWD_B2], &p->mv[MV_FWD_X0], i_mb_type > P_8X8, p->i_uint_length);
                bs[5] = get_bs(&p->mv[MV_FWD_B3], &p->mv[MV_FWD_X1], i_mb_type > P_8X8, p->i_uint_length);
            }
            if( *((uint64_t *)bs) ) 
            {
            	if(p->i_mb_flags & A_AVAIL) 
            	{
            	   /* MB verticl edge */
                    qp_avg = (p->i_qp + p->i_left_qp + 1) >> 1;

                    /* FIXIT : weighting quant qp for chroma
                        NOTE : not modify p->i_qp directly
                    */
                    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
                    { 
                        qp_avg_cb = (chroma_qp[clip3_int(p->i_qp + p->ph.chroma_quant_param_delta_u, 0, 63 )]
                            +  chroma_qp[clip3_int(p->i_left_qp + p->ph.chroma_quant_param_delta_u, 0, 63)] + 1) >>1;
                        qp_avg_cr = (chroma_qp[clip3_int( p->i_qp + p->ph.chroma_quant_param_delta_v, 0, 63 )]
                            +  chroma_qp[clip3_int( p->i_left_qp + p->ph.chroma_quant_param_delta_v, 0, 63)] + 1) >> 1; 

                        p->filter_lv(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cb, p->cur.i_stride[1], qp_avg_cb, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cr, p->cur.i_stride[2], qp_avg_cr, bs[0], bs[1]);
                    }
                    else
                    {
                        qp_avg_c = (chroma_qp[p->i_qp] + chroma_qp[p->i_left_qp] + 1) >> 1;
                    
                        p->filter_lv(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cb, p->cur.i_stride[1], qp_avg_c, bs[0], bs[1]);
                        p->filter_cv(p, p->p_cr, p->cur.i_stride[2], qp_avg_c, bs[0], bs[1]);
                    }
                }
                qp_avg = p->i_qp;

                p->filter_lv(p, p->p_y + 8, p->cur.i_stride[0], qp_avg, bs[2], bs[3]);
                p->filter_lh(p, p->p_y + 8*p->cur.i_stride[0], p->cur.i_stride[0], qp_avg, bs[6], bs[7]);

                if(p->i_mb_flags & B_AVAIL) 
                {
            	    /* MB horizontal edge */
                    qp_avg = (p->i_qp + p->p_top_qp[p->i_mb_x] + 1) >> 1;
                    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
                    {
                        qp_avg_cb = (chroma_qp[clip3_int( p->i_qp + p->ph.chroma_quant_param_delta_u,0, 63 )]
                            +  chroma_qp[clip3_int( p->p_top_qp[p->i_mb_x] + p->ph.chroma_quant_param_delta_u, 0, 63 )] + 1) >>1;
                        qp_avg_cr = (chroma_qp[clip3_int(p->i_qp + p->ph.chroma_quant_param_delta_v ,0, 63 )]
                            +  chroma_qp[clip3_int( p->p_top_qp[p->i_mb_x] + p->ph.chroma_quant_param_delta_v, 0, 63 )] + 1) >> 1;

                        p->filter_lh(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cb, p->cur.i_stride[1], qp_avg_cb, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cr, p->cur.i_stride[2], qp_avg_cr, bs[4], bs[5]);
                    }
                    else
                    {
                        qp_avg_c = (chroma_qp[p->i_qp] + chroma_qp[p->p_top_qp[p->i_mb_x]] + 1) >> 1;

                        p->filter_lh(p, p->p_y, p->cur.i_stride[0], qp_avg, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cb, p->cur.i_stride[1], qp_avg_c, bs[4], bs[5]);
                        p->filter_ch(p, p->p_cr, p->cur.i_stride[2], qp_avg_c, bs[4], bs[5]);
                    }
                    

                }
            }
        }

        p->i_left_qp = p->i_qp;
        p->p_top_qp[p->i_mb_x] = p->i_qp;
        
    }
        
}


#define CAVS_CHROMA_MC(OPNAME, OP,L)\
static UNUSED void  cavs_chroma_mc2_## OPNAME ##_c(uint8_t *dst, uint8_t *src, int stride, int h, int x, int y){\
    const int A=(L-x)*(L-y);\
    const int B=(  x)*(L-y);\
    const int C=(L-x)*(  y);\
    const int D=(  x)*(  y);\
    int i;\
    \
\
    for(i=0; i<h; i++)\
    {\
        OP(dst[0], (A*src[0] + B*src[1] + C*src[stride+0] + D*src[stride+1]));\
        OP(dst[1], (A*src[1] + B*src[2] + C*src[stride+1] + D*src[stride+2]));\
        dst+= stride;\
        src+= stride;\
    }\
}\
\
static UNUSED void cavs_chroma_mc4_## OPNAME ##_c(uint8_t *dst, uint8_t *src, int stride, int h, int x, int y){\
    const int A=(L-x)*(L-y);\
    const int B=(  x)*(L-y);\
    const int C=(L-x)*(  y);\
    const int D=(  x)*(  y);\
    int i;\
    \
\
    for(i=0; i<h; i++)\
    {\
        OP(dst[0], (A*src[0] + B*src[1] + C*src[stride+0] + D*src[stride+1]));\
        OP(dst[1], (A*src[1] + B*src[2] + C*src[stride+1] + D*src[stride+2]));\
        OP(dst[2], (A*src[2] + B*src[3] + C*src[stride+2] + D*src[stride+3]));\
        OP(dst[3], (A*src[3] + B*src[4] + C*src[stride+3] + D*src[stride+4]));\
        dst+= stride;\
        src+= stride;\
    }\
}\
\
static UNUSED void cavs_chroma_mc8_## OPNAME ##_c(uint8_t *dst, uint8_t *src, int stride, int h, int x, int y){\
    const int A=(L-x)*(L-y);\
    const int B=(  x)*(L-y);\
    const int C=(L-x)*(  y);\
    const int D=(  x)*(  y);\
    int i;\
    \
\
    for(i=0; i<h; i++)\
    {\
        OP(dst[0], (A*src[0] + B*src[1] + C*src[stride+0] + D*src[stride+1]));\
        OP(dst[1], (A*src[1] + B*src[2] + C*src[stride+1] + D*src[stride+2]));\
        OP(dst[2], (A*src[2] + B*src[3] + C*src[stride+2] + D*src[stride+3]));\
        OP(dst[3], (A*src[3] + B*src[4] + C*src[stride+3] + D*src[stride+4]));\
        OP(dst[4], (A*src[4] + B*src[5] + C*src[stride+4] + D*src[stride+5]));\
        OP(dst[5], (A*src[5] + B*src[6] + C*src[stride+5] + D*src[stride+6]));\
        OP(dst[6], (A*src[6] + B*src[7] + C*src[stride+6] + D*src[stride+7]));\
        OP(dst[7], (A*src[7] + B*src[8] + C*src[stride+7] + D*src[stride+8]));\
        dst+= stride;\
        src+= stride;\
    }\
}

#define op_avg(a, b) a = (((a)+(((b) + 32)>>6)+1)>>1)
#define op_put(a, b) a = (((b) + 32)>>6)
#define op_avg16(a, b) a = (((a)+(((b) + 128)>>8)+1)>>1)
#define op_put16(a, b) a = (((b) + 128)>>8)

CAVS_CHROMA_MC(put       , op_put,8)
CAVS_CHROMA_MC(avg       , op_avg,8)
CAVS_CHROMA_MC(16put       , op_put16,16)
CAVS_CHROMA_MC(16avg       , op_avg16,16)
#undef op_avg
#undef op_put
#undef op_avg16
#undef op_put16

static void cavs_emulated_edge_mc(uint8_t *buf, uint8_t *src, int linesize, int block_w, int block_h, 
	int src_x, int src_y, int w, int h)
{
    int x, y;
    int start_y, start_x, end_y, end_x;
    
    if(src_y>= h){
        src+= (h-1-src_y)*linesize;
        src_y=h-1;
    }else if(src_y<=-block_h){
        src+= (1-block_h-src_y)*linesize;
        src_y=1-block_h;
    }
    if(src_x>= w){
        src+= (w-1-src_x);
        src_x=w-1;
    }else if(src_x<=-block_w){
        src+= (1-block_w-src_x);
        src_x=1-block_w;
    }

    start_y= CAVS_MAX(0, -src_y);
    start_x= CAVS_MAX(0, -src_x);
    end_y= CAVS_MIN(block_h, h-src_y);
    end_x= CAVS_MIN(block_w, w-src_x);


    // copy existing part
    for(y=start_y; y<end_y; y++){
        memcpy(  &buf[start_x + y*linesize], &src[start_x + y*linesize], end_x - start_x );
    }
    
    //top
    for(y=0; y<start_y; y++){
        memcpy(  &buf[start_x + y*linesize], &buf[start_x + start_y*linesize], end_x - start_x );
    }

    //bottom
    for(y=end_y; y<block_h; y++){
        memcpy( &buf[start_x + y*linesize], &buf[start_x + (end_y-1)*linesize], end_x - start_x );
    }

    for(y=0; y<block_h; y++){
       //left
        for(x=0; x<start_x; x++){
            buf[x + y*linesize]= buf[start_x + y*linesize];
        }
       
       //right
        for(x=end_x; x<block_w; x++){
            buf[x + y*linesize]= buf[end_x - 1 + y*linesize];
        }
    }
}


static inline void mc_dir_part(cavs_decoder *p,cavs_image *ref,
                        int chroma_height,int list,uint8_t *dest_y,
                        uint8_t *dest_cb,uint8_t *dest_cr,int src_x_offset,
                        int src_y_offset,cavs_luma_mc_func *luma,
                        cavs_chroma_mc_func chroma,cavs_vector *mv)
{
    const int mx = (mv->x + (src_x_offset<<3));
    const int my =(mv->y + (src_y_offset<<3));
    const int luma_xy = ( (mx&3) + ((my&3)<<2) );
    uint8_t * src_y,* src_cb,* src_cr;
    int extra_width = 0; //(s->flags&CODEC_FLAG_EMU_EDGE) ? 0 : 16;
    int extra_height = extra_width;
    int emu = 0;
    const int full_mx = mx>>2;
    const int full_my = my>>2;
    const int pic_width  = p->i_mb_width<<4;
    const int pic_height = ( p->ph.b_picture_structure==0?(p->i_mb_height>>1):p->i_mb_height)<<4;

    {
        src_y = ref->p_data[0] + (mx>>2) + (my>>2)*(int)ref->i_stride[0]; // NOTE : unsigned sign must trans to sign
        src_cb= ref->p_data[1] + (mx>>3) + (my>>3)*(int)ref->i_stride[1];
        src_cr= ref->p_data[2] + (mx>>3) + (my>>3)*(int)ref->i_stride[2];
    }

    if(!ref->p_data[0])
        return;

    if(mx&15) extra_width -= 3;
    if(my&15) extra_height -= 3;


    if(   full_mx < 0-extra_width
          || full_my < 0-extra_height
          || full_mx + 16 > pic_width + extra_width
          || full_my + 16 > pic_height + extra_height)
    {
        cavs_emulated_edge_mc(p->p_edge, src_y - 2 - 2*(int)ref->i_stride[0], ref->i_stride[0],
                            16+9, 16+9, full_mx-2, full_my-2, pic_width, pic_height);
        src_y = p->p_edge + 2 + 2*(int)ref->i_stride[0];
        emu = 1;
    }

    luma[luma_xy](dest_y, src_y, ref->i_stride[0]);

    if(emu)
    {
        {
            cavs_emulated_edge_mc(p->p_edge, src_cb, ref->i_stride[1],
                        9, 9, (mx>>3), (my>>3), pic_width>>1, pic_height>>1);
        }

        src_cb= p->p_edge;
    }

    chroma(dest_cb, src_cb, ref->i_stride[1], chroma_height, mx&7, my&7);

    if(emu)
    {
        {
            cavs_emulated_edge_mc(p->p_edge, src_cr, ref->i_stride[2],
                            9, 9, (mx>>3), (my>>3), pic_width>>1, pic_height>>1);
        }
        src_cr= p->p_edge;
    }

    chroma(dest_cr, src_cr, ref->i_stride[2], chroma_height, mx&7, my&7);
}

static inline void mc_part_std(cavs_decoder *p,int chroma_height,
                       uint8_t *dest_y,uint8_t *dest_cb,uint8_t *dest_cr,
					   int x_offset, int y_offset,cavs_vector *mv,
					   cavs_luma_mc_func *luma_put,cavs_chroma_mc_func chroma_put,
					   cavs_luma_mc_func *luma_avg,cavs_chroma_mc_func chroma_avg)
{
    cavs_luma_mc_func *lum=  luma_put;
    cavs_chroma_mc_func chroma= chroma_put;

#if B_MB_WEIGHTING
    int i_dir = 0;
    uint8_t *dst_back_y, *dst_back_cb, *dst_back_cr;
    int x_offset_back = x_offset;
    int y_offset_back = y_offset;
    int fw_luma_scale, fw_chroma_scale;
    int fw_luma_shift, fw_chroma_shift;
    int ii, jj;
	int i_first_field = p->i_mb_index < p->i_mb_num_half ? 1 : 0; /* 0 is bottom */

#endif

    dest_y  += (x_offset<<1) + ((y_offset*p->cur.i_stride[0])<<1);
    dest_cb +=   x_offset +   y_offset*p->cur.i_stride[1];
    dest_cr +=   x_offset +   y_offset*p->cur.i_stride[2];
    x_offset += (p->i_mb_x)<<3;
    y_offset += 8*(p->i_mb_y-p->i_mb_offset);

#if B_MB_WEIGHTING
    /* get temp buffer for dest_y, dest_cb, dest_cr */
    dst_back_y = p->p_back_y;
    dst_back_cb = p->p_back_cb;
    dst_back_cr = p->p_back_cr;

    dst_back_y += (2*x_offset_back) + (2*y_offset_back*p->cur.i_stride[0]); /* set buffer address */
    dst_back_cb += x_offset_back +  y_offset_back*p->cur.i_stride[1];
    dst_back_cr += x_offset_back +  y_offset_back*p->cur.i_stride[2];

    x_offset_back += 8*p->i_mb_x;
    y_offset_back += 8*(p->i_mb_y-p->i_mb_offset);
#endif

#if B_MB_WEIGHTING
	  /* forward */
    if(mv->ref >= 0)
    {
        cavs_image *ref= &p->ref[mv->ref];

        mc_dir_part(p, ref, chroma_height, 0, dest_y, dest_cb, dest_cr, x_offset, y_offset,
                    lum, chroma, mv);
        i_dir = 1; /* forward flag */

        /* weighting prediction */
        if( ((p->sh.b_slice_weighting_flag == 1) 
            && (p->sh.b_mb_weighting_flag == 1) && (p->weighting_prediction == 1))
            || ((p->sh.b_slice_weighting_flag == 1) &&  (p->sh.b_mb_weighting_flag == 0)))
            {
                if((p->ph.i_picture_coding_type ==  1) || (i_first_field == 0 && p->ph.i_picture_coding_type ==  0)) /* P-slice */
                {
                    /* scale shift */
                    fw_luma_scale = p->sh.i_luma_scale[mv->ref];
                    fw_luma_shift = p->sh.i_luma_shift[mv->ref];

                    /* luma */
                    for( ii = 0; ii < chroma_height<<1; ii++ )
                    {
                        for( jj = 0; jj < chroma_height<<1; jj++ )
                        {
                            *(dest_y + ii*p->cur.i_stride[0]+ jj) = 
                                cavs_clip_uint8(((  (*(dest_y + ii*p->cur.i_stride[0]+ jj)) *fw_luma_scale+16)>>5) + fw_luma_shift);
                        }
                    }

                    /* scale shift */
                    fw_chroma_scale = p->sh.i_chroma_scale[mv->ref];
                    fw_chroma_shift = p->sh.i_chroma_shift[mv->ref];

                    /* cb */
			        for( ii = 0; ii < chroma_height; ii++ )
                    {
                        for( jj = 0; jj < chroma_height; jj++ )
                        {
                            *(dest_cb + ii*p->cur.i_stride[1]+ jj) = 
                                cavs_clip_uint8(((  (*(dest_cb + ii*p->cur.i_stride[1]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                        }
                    }

                    /* cr */
					for( ii = 0; ii < chroma_height; ii++ )
                    {
                        for( jj = 0; jj < chroma_height; jj++ )
                        {
                            *(dest_cr + ii*p->cur.i_stride[2]+ jj) = 
                                cavs_clip_uint8(((  (*(dest_cr + ii*p->cur.i_stride[2]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                        }
                    }
                      
                }
                else if(p->ph.i_picture_coding_type ==  2)/* B-frame */
                {
					int refframe;
					int i_ref_offset = p->ph.b_picture_structure == 0 ? 2 : 1;

					refframe = mv->ref - i_ref_offset;

					refframe= (!p->param.b_interlaced )?(refframe):(2*refframe);

					/* scale shift */
                    fw_luma_scale = p->sh.i_luma_scale[refframe];
                    fw_luma_shift = p->sh.i_luma_shift[refframe];

					/* luma */
					for( ii = 0; ii < chroma_height<<1; ii++ )
                    {
                        for( jj = 0; jj < chroma_height<<1; jj++ )
                        {
                            *(dest_y + ii*p->cur.i_stride[0]+ jj) = 
                                cavs_clip_uint8(((  (*(dest_y + ii*p->cur.i_stride[0]+ jj)) *fw_luma_scale+16)>>5) + fw_luma_shift);
                        }
                    }

					/* cb */
					fw_chroma_scale = p->sh.i_chroma_scale[refframe];
					fw_chroma_shift = p->sh.i_chroma_shift[refframe];
				
					for( ii = 0; ii < chroma_height; ii++ )
                    {
                        for( jj = 0; jj < chroma_height; jj++ )
                        {
                            *(dest_cb + ii*p->cur.i_stride[1]+ jj) = 
                                cavs_clip_uint8(((  (*(dest_cb + ii*p->cur.i_stride[1]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                        }
                    }

					/* cr */
					for( ii = 0; ii < chroma_height; ii++ )
                    {
                        for( jj = 0; jj < chroma_height; jj++ )
                        {
                            *(dest_cr + ii*p->cur.i_stride[2]+ jj) = 
                                cavs_clip_uint8(((  (*(dest_cr + ii*p->cur.i_stride[2]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                        }
                    }

                }
            
            }
        
    }

    /* backward */
    if((mv+MV_BWD_OFFS)->ref >= 0)
    {
        cavs_image *ref= &p->ref[(mv+MV_BWD_OFFS)->ref];
        
        if( i_dir == 0 )/* has no forward */
        {
            mc_dir_part(p, ref,  chroma_height,  1,
                                dest_y, dest_cb, dest_cr, x_offset, y_offset,
                                lum, chroma, mv+MV_BWD_OFFS);

             /* weighting prediction */
			if( ((p->sh.b_slice_weighting_flag == 1) 
				&& (p->sh.b_mb_weighting_flag == 1) && (p->weighting_prediction == 1))
				|| ((p->sh.b_slice_weighting_flag == 1) &&  (p->sh.b_mb_weighting_flag == 0)))
			{
				int refframe;

				refframe = (mv+MV_BWD_OFFS)->ref;

				refframe= (!p->param.b_interlaced )?(refframe+1):(2*refframe+1);

				/* scale shift */
                fw_luma_scale = p->sh.i_luma_scale[refframe];
                fw_luma_shift = p->sh.i_luma_shift[refframe];

				/* luma */
				for( ii = 0; ii < chroma_height<<1; ii++ )
                {
                    for( jj = 0; jj < chroma_height<<1; jj++ )
                    {
                        *(dest_y + ii*p->cur.i_stride[0]+ jj) = 
                            cavs_clip_uint8(((  (*(dest_y + ii*p->cur.i_stride[0]+ jj)) *fw_luma_scale+16)>>5) + fw_luma_shift);
                    }
                }

				/* cb */
				fw_chroma_scale = p->sh.i_chroma_scale[refframe];
				fw_chroma_shift = p->sh.i_chroma_shift[refframe];
				
				for( ii = 0; ii < chroma_height; ii++ )
                {
                    for( jj = 0; jj < chroma_height; jj++ )
                    {
                        *(dest_cb + ii*p->cur.i_stride[1]+ jj) = 
                            cavs_clip_uint8(((  (*(dest_cb + ii*p->cur.i_stride[1]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                    }
                }

				/* cr */
				for( ii = 0; ii < chroma_height; ii++ )
                {
                    for( jj = 0; jj < chroma_height; jj++ )
                    {
                        *(dest_cr + ii*p->cur.i_stride[2]+ jj) = 
                            cavs_clip_uint8(((  (*(dest_cr + ii*p->cur.i_stride[2]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                    }
                }
			}
             
        }
        else/* already has forward */
        {
            mc_dir_part(p, ref,  chroma_height,  1,
                            dst_back_y, dst_back_cb, dst_back_cr, x_offset_back, y_offset_back,
                            lum, chroma, mv+MV_BWD_OFFS);

             /* weighting prediction */
			 if( ((p->sh.b_slice_weighting_flag == 1) 
				&& (p->sh.b_mb_weighting_flag == 1) && (p->weighting_prediction == 1))
				|| ((p->sh.b_slice_weighting_flag == 1) &&  (p->sh.b_mb_weighting_flag == 0)))
             {
				int refframe;

				refframe = (mv+MV_BWD_OFFS)->ref;

				refframe= (!p->param.b_interlaced )?(refframe+1):(2*refframe+1);

				/* scale shift */
                fw_luma_scale = p->sh.i_luma_scale[refframe];
                fw_luma_shift = p->sh.i_luma_shift[refframe];

				/* luma */
				for( ii = 0; ii < chroma_height<<1; ii++ )
                {
                    for( jj = 0; jj < chroma_height<<1; jj++ )
                    {
                        *(dst_back_y + ii*p->cur.i_stride[0]+ jj) = 
                            cavs_clip_uint8(((  (*(dst_back_y + ii*p->cur.i_stride[0]+ jj)) *fw_luma_scale+16)>>5) + fw_luma_shift);
                    }
                }

				/* cb */
				fw_chroma_scale = p->sh.i_chroma_scale[refframe];
				fw_chroma_shift = p->sh.i_chroma_shift[refframe];
				
				for( ii = 0; ii < chroma_height; ii++ )
                {
                    for( jj = 0; jj < chroma_height; jj++ )
                    {
                        *(dst_back_cb + ii*p->cur.i_stride[1]+ jj) = 
                            cavs_clip_uint8(((  (*(dst_back_cb + ii*p->cur.i_stride[1]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                    }
                }

				/* cr */
				for( ii = 0; ii < chroma_height; ii++ )
                {
                    for( jj = 0; jj < chroma_height; jj++ )
                    {
                        *(dst_back_cr + ii*p->cur.i_stride[2]+ jj) = 
                            cavs_clip_uint8(((  (*(dst_back_cr + ii*p->cur.i_stride[2]+ jj)) *fw_chroma_scale+16)>>5) + fw_chroma_shift);
                    }
                }
			}
        }
        
        i_dir = i_dir|2;  /* backward flag */
    }

    /* avg */
    if( i_dir == 3 )
    {
        /* luma */
        p->cavs_avg_pixels_tab[!(chroma_height==8)]( dest_y, p->cur.i_stride[0], dst_back_y, p->cur.i_stride[0] );

        /* cb */
        p->cavs_avg_pixels_tab[(!(chroma_height==8)) + 1]( dest_cb, p->cur.i_stride[1], dst_back_cb, p->cur.i_stride[1] );
        
        /* cr */
        p->cavs_avg_pixels_tab[(!(chroma_height==8)) + 1]( dest_cr, p->cur.i_stride[2], dst_back_cr, p->cur.i_stride[2] );
    }

#else
    if(mv->ref >= 0)
    {
        cavs_image *ref= &p->ref[mv->ref];

        mc_dir_part(p, ref,chroma_height,0,dest_y, dest_cb, dest_cr, x_offset, y_offset,
                    lum, chroma, mv);

        lum=  luma_avg;
        chroma= chroma_avg;
    }

    if((mv+MV_BWD_OFFS)->ref >= 0)
    {
        cavs_image *ref= &p->ref[(mv+MV_BWD_OFFS)->ref];
        
        mc_dir_part(p, ref,  chroma_height,  1,
                dest_y, dest_cb, dest_cr, x_offset, y_offset,
                 lum, chroma, mv+MV_BWD_OFFS);		
    }
#endif

}

static void inter_pred(cavs_decoder *p, int i_mb_type) 
{
    int y_16, c_16;

    y_16 = c_16 = 0;
    //if(p->ph.asi_enable == 1)
    //{
    //    y_16=2;
    //    c_16=3;
    //}

    if(partition_flags[i_mb_type] == 0)
    { 		
    	/* 16x16 */
    	mc_part_std(p, 8, p->p_y, p->p_cb, p->p_cr,
    		0, 0,&p->mv[MV_FWD_X0],
    		p->put_cavs_qpel_pixels_tab[0+y_16],p->put_h264_chroma_pixels_tab[0+c_16],
    		p->avg_cavs_qpel_pixels_tab[0+y_16],p->avg_h264_chroma_pixels_tab[0+c_16]);
    }
    else
    {
    	mc_part_std(p, 4, p->p_y, p->p_cb, p->p_cr, 
    		0, 0,&p->mv[MV_FWD_X0],
    		p->put_cavs_qpel_pixels_tab[1+y_16],p->put_h264_chroma_pixels_tab[1+c_16],
    		p->avg_cavs_qpel_pixels_tab[1+y_16],p->avg_h264_chroma_pixels_tab[1+c_16]);
    	
    	mc_part_std(p, 4, p->p_y, p->p_cb, p->p_cr,
    		4, 0,&p->mv[MV_FWD_X1],
    		p->put_cavs_qpel_pixels_tab[1+y_16],p->put_h264_chroma_pixels_tab[1+c_16],
    		p->avg_cavs_qpel_pixels_tab[1+y_16],p->avg_h264_chroma_pixels_tab[1+c_16]);
    	
    	mc_part_std(p, 4, p->p_y, p->p_cb, p->p_cr,
    		0, 4,&p->mv[MV_FWD_X2],
    		p->put_cavs_qpel_pixels_tab[1+y_16],p->put_h264_chroma_pixels_tab[1+c_16],
    		p->avg_cavs_qpel_pixels_tab[1+y_16],p->avg_h264_chroma_pixels_tab[1+c_16]);
    	
    	mc_part_std(p, 4, p->p_y, p->p_cb, p->p_cr,
    		4, 4,&p->mv[MV_FWD_X3],
    		p->put_cavs_qpel_pixels_tab[1+y_16],p->put_h264_chroma_pixels_tab[1+c_16],
    		p->avg_cavs_qpel_pixels_tab[1+y_16],p->avg_h264_chroma_pixels_tab[1+c_16]);
    }

    /* set intra prediction modes to default values */
    p->i_intra_pred_mode_y[5] =  p->i_intra_pred_mode_y[10] = 
    p->i_intra_pred_mode_y[15] =  p->i_intra_pred_mode_y[20] = NOT_AVAIL;

    p->p_top_intra_pred_mode_y[(p->i_mb_x<<2)+0] = p->p_top_intra_pred_mode_y[(p->i_mb_x<<2)+1] = 
    p->p_top_intra_pred_mode_y[(p->i_mb_x<<2)+2] = p->p_top_intra_pred_mode_y[(p->i_mb_x<<2)+3] = NOT_AVAIL;    
}


static inline int next_mb( cavs_decoder *p )
{
    int i, i_y;
    int i_offset = p->i_mb_x<<1;

    p->i_mb_flags |= A_AVAIL;
    p->p_y += 16;
    p->p_cb += 8;
    p->p_cr += 8;

    if( p->param.b_accelerate )
    {
        for( i = 0; i <= 20; i += 4 )
        {//原来B3,X1,X3的运动矢量成为右边新的宏块的预测运动矢量左边的候选子即D2,A1,A3
            p->mv[i] = p->mv[i+2];
        }

        //保存X2,X3作为下面宏块的预测运动矢量的候选子即B2,B3
        p->mv[MV_FWD_D3] = p->p_top_mv[0][i_offset+1];
        p->mv[MV_BWD_D3] = p->p_top_mv[1][i_offset+1];//修改前先保存到D3;
        p->p_top_mv[0][i_offset+0] = p->mv[MV_FWD_X2];
        p->p_top_mv[0][i_offset+1] = p->mv[MV_FWD_X3];
        p->p_top_mv[1][i_offset+0] = p->mv[MV_BWD_X2];
        p->p_top_mv[1][i_offset+1] = p->mv[MV_BWD_X3];

        /* for AEC */
        if (p->ph.b_aec_enable)
        {
            p->p_mb_type_top[p->i_mb_x] = p->i_mb_type_tab[p->i_mb_index];
			if( p->ph.i_picture_coding_type == 0 && p->b_bottom == 0 )
            {
                p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = p->i_pred_mode_chroma_tab[p->i_mb_index];
                p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = p->i_cbp;
            }
            else
            {
                 if( p->i_mb_type_tab[p->i_mb_index] != P_SKIP && p->i_mb_type_tab[p->i_mb_index] != B_SKIP )
                {
                    p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = p->i_pred_mode_chroma_tab[p->i_mb_index];
                    p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = p->i_cbp;//p->i_cbp_tab[p->i_mb_index];
                } 
                else
                {   
                    p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = 0;
                    p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = 0; /* skip has not cbp */
                    p->i_last_dqp = 0;
                }
            }

            CP16(p->p_ref_top[p->i_mb_x][0], &p->p_ref[0][7]);
            CP16(p->p_ref_top[p->i_mb_x][1], &p->p_ref[1][7]);
       }

        /*Move to next MB*/
        p->i_mb_x++;
        if(p->i_mb_x == p->i_mb_width) 
        { 
            //下一行宏块
            p->i_mb_flags = B_AVAIL|C_AVAIL;//如果i_mb_width=1;则C_AVAIL不可得到
            /* 清除左边块预测模式A1,A3*/
            p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
            p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;
            
            p->i_intra_pred_mode_y_tab[p->i_mb_index][5] = p->i_intra_pred_mode_y_tab[p->i_mb_index][10] = 
            p->i_intra_pred_mode_y_tab[p->i_mb_index][15] = p->i_intra_pred_mode_y_tab[p->i_mb_index][20] = NOT_AVAIL;
              
            /* 清除左边块运动矢量C2,A1,A3 */
            for( i = 0; i <= 20; i += 4 )
            {
                p->mv[i] = MV_NOT_AVAIL;
            }
            p->i_mb_x = 0;
            p->i_mb_y++;
            i_y = p->i_mb_y - p->i_mb_offset;
            p->p_y = p->cur.p_data[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
            p->p_cb = p->cur.p_data[1] + i_y*CAVS_MB_SIZE/2*p->cur.i_stride[1];
            p->p_cr = p->cur.p_data[2] + i_y*CAVS_MB_SIZE/2*p->cur.i_stride[2];

#if B_MB_WEIGHTING
            p->p_back_y = p->mb_line_buffer[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
            p->p_back_cb = p->mb_line_buffer[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
            p->p_back_cr = p->mb_line_buffer[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];
#endif

            p->b_first_line = 0;

            /* for AEC */
            p->i_mb_type_left = -1;
        }
        else
        {
            /* for AEC */
            p->i_mb_type_left = p->i_mb_type_tab[p->i_mb_index];
        }

        if( p->i_mb_x == p->i_mb_width-1 )
        {
            p->i_mb_flags &= ~C_AVAIL;
        }

        if( p->i_mb_x && p->i_mb_y && p->b_first_line==0 ) 
        {
            p->i_mb_flags |= D_AVAIL;
        }
    }
    else
    {
        for( i = 0; i <= 20; i += 4 )
        {//原来B3,X1,X3的运动矢量成为右边新的宏块的预测运动矢量左边的候选子即D2,A1,A3
            p->mv[i] = p->mv[i+2];
        }
        //保存X2,X3作为下面宏块的预测运动矢量的候选子即B2,B3
        p->mv[MV_FWD_D3] = p->p_top_mv[0][i_offset+1];
        p->mv[MV_BWD_D3] = p->p_top_mv[1][i_offset+1];//修改前先保存到D3;
        p->p_top_mv[0][i_offset+0] = p->mv[MV_FWD_X2];
        p->p_top_mv[0][i_offset+1] = p->mv[MV_FWD_X3];
        p->p_top_mv[1][i_offset+0] = p->mv[MV_BWD_X2];
        p->p_top_mv[1][i_offset+1] = p->mv[MV_BWD_X3];

        /* for AEC */
        if (p->ph.b_aec_enable)
        {
            p->p_mb_type_top[p->i_mb_x] = p->i_mb_type;
			if(p->ph.i_picture_coding_type == 0 && p->b_bottom == 0)/* frame or field-top is I-slice */
			{
				p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = p->i_pred_mode_chroma;
                p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = p->i_cbp;
			}
			else
			{
				if( p->i_mb_type != P_SKIP && p->i_mb_type != B_SKIP )
				{
					p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = p->i_pred_mode_chroma;
					p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = p->i_cbp;
				} 
				else
				{   
					p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = 0;
					p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = 0; /* skip has not cbp */
					p->i_last_dqp = 0;
				}
			}
            
            CP16(p->p_ref_top[p->i_mb_x][0], &p->p_ref[0][7]);
            CP16(p->p_ref_top[p->i_mb_x][1], &p->p_ref[1][7]);
        }

        /*Move to next MB*/
        p->i_mb_x++;
        if(p->i_mb_x == p->i_mb_width) 
        { 
            //下一行宏块
            p->i_mb_flags = B_AVAIL|C_AVAIL;//如果i_mb_width=1;则C_AVAIL不可得到
            /* 清除左边块预测模式A1,A3*/
            p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
            p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;
            
            /* 清除左边块运动矢量C2,A1,A3 */
            for( i = 0; i <= 20; i += 4 )
            {
                p->mv[i] = MV_NOT_AVAIL;
            }
            p->i_mb_x = 0;
            p->i_mb_y++;
            i_y = p->i_mb_y - p->i_mb_offset;
            p->p_y = p->cur.p_data[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
            p->p_cb = p->cur.p_data[1] + i_y*CAVS_MB_SIZE/2*p->cur.i_stride[1];
            p->p_cr = p->cur.p_data[2] + i_y*CAVS_MB_SIZE/2*p->cur.i_stride[2];

#if B_MB_WEIGHTING
            p->p_back_y = p->mb_line_buffer[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
            p->p_back_cb = p->mb_line_buffer[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
            p->p_back_cr = p->mb_line_buffer[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];
#endif

            p->b_first_line = 0;

            /* for AEC */
            p->i_mb_type_left = -1;
        }
        else
        {
            /* for AEC */
            p->i_mb_type_left = p->i_mb_type;
        }

        if( p->i_mb_x == p->i_mb_width-1 )
        {
            p->i_mb_flags &= ~C_AVAIL;
        }

        if( p->i_mb_x && p->i_mb_y && p->b_first_line==0 ) 
        {
            p->i_mb_flags |= D_AVAIL;
        }
    }

    return 0;
}

static inline void init_mb(cavs_decoder *p) 
{
    //int i = 0;
    int i_offset = p->i_mb_x<<1;
    int i_offset_b4 = p->i_mb_x<<2;

    if( p->param.b_accelerate )
    {
        if(!(p->i_mb_flags & A_AVAIL))
        {
            p->mv[MV_FWD_A1] = MV_NOT_AVAIL;
            p->mv[MV_FWD_A3] = MV_NOT_AVAIL;
            p->mv[MV_BWD_A1] = MV_NOT_AVAIL;
            p->mv[MV_BWD_A3] = MV_NOT_AVAIL;

            p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
            p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;

            p->i_intra_pred_mode_y_tab[p->i_mb_index][5] = p->i_intra_pred_mode_y_tab[p->i_mb_index][10] = 
            p->i_intra_pred_mode_y_tab[p->i_mb_index][15] = p->i_intra_pred_mode_y_tab[p->i_mb_index][20] = NOT_AVAIL;
        }

        if(p->i_mb_flags & B_AVAIL)
        {
            p->mv[MV_FWD_B2] = p->p_top_mv[0][i_offset];
            p->mv[MV_BWD_B2] = p->p_top_mv[1][i_offset];
            p->mv[MV_FWD_B3] = p->p_top_mv[0][i_offset+1];
            p->mv[MV_BWD_B3] = p->p_top_mv[1][i_offset+1];

            p->i_intra_pred_mode_y[1] = p->p_top_intra_pred_mode_y[i_offset_b4];
            p->i_intra_pred_mode_y[2] = p->p_top_intra_pred_mode_y[i_offset_b4+1];
            p->i_intra_pred_mode_y[3] = p->p_top_intra_pred_mode_y[i_offset_b4+2];
            p->i_intra_pred_mode_y[4] = p->p_top_intra_pred_mode_y[i_offset_b4+3];

            p->i_intra_pred_mode_y_tab[p->i_mb_index][1] = p->p_top_intra_pred_mode_y[i_offset_b4];
            p->i_intra_pred_mode_y_tab[p->i_mb_index][2] = p->p_top_intra_pred_mode_y[i_offset_b4+1];
            p->i_intra_pred_mode_y_tab[p->i_mb_index][3] = p->p_top_intra_pred_mode_y[i_offset_b4+2];
            p->i_intra_pred_mode_y_tab[p->i_mb_index][4] = p->p_top_intra_pred_mode_y[i_offset_b4+3];
        }
        else
        {
            p->mv[MV_FWD_B2] = MV_NOT_AVAIL;
            p->mv[MV_FWD_B3] = MV_NOT_AVAIL;
            p->mv[MV_BWD_B2] = MV_NOT_AVAIL;
            p->mv[MV_BWD_B3] = MV_NOT_AVAIL;
            
            p->i_intra_pred_mode_y[1] = p->i_intra_pred_mode_y[2] = 
            p->i_intra_pred_mode_y[3] = p->i_intra_pred_mode_y[4] = NOT_AVAIL;
 
            p->i_intra_pred_mode_y_tab[p->i_mb_index][1] = p->i_intra_pred_mode_y_tab[p->i_mb_index][2] = 
            p->i_intra_pred_mode_y_tab[p->i_mb_index][3] = p->i_intra_pred_mode_y_tab[p->i_mb_index][4] = NOT_AVAIL;
        }

        if(p->i_mb_flags & C_AVAIL)
        {
            p->mv[MV_FWD_C2] = p->p_top_mv[0][i_offset+2];
            p->mv[MV_BWD_C2] = p->p_top_mv[1][i_offset+2];
        }
        else
        {
            p->mv[MV_FWD_C2] = MV_NOT_AVAIL;
            p->mv[MV_BWD_C2] = MV_NOT_AVAIL;
        }

        if(!(p->i_mb_flags & D_AVAIL))
        {
            p->mv[MV_FWD_D3] = MV_NOT_AVAIL;
            p->mv[MV_BWD_D3] = MV_NOT_AVAIL;
        }

        p->p_col_type = &p->p_col_type_base[p->i_mb_y*p->i_mb_width + p->i_mb_x];

        p->vbs_mb_intra_pred_flag = 0;
        p->vbs_mb_part_intra_pred_flag[0] = p->vbs_mb_part_intra_pred_flag[1] =
        p->vbs_mb_part_intra_pred_flag[2] = p->vbs_mb_part_intra_pred_flag[3] =
        p->vbs_mb_part_transform_4x4_flag[0] = p->vbs_mb_part_transform_4x4_flag[1] =
        p->vbs_mb_part_transform_4x4_flag[2] = p->vbs_mb_part_transform_4x4_flag[3] = 0;

        if (p->ph.b_aec_enable)
        {
            if (!p->i_mb_x)
            {
                p->i_chroma_pred_mode_left = 0;
                p->i_cbp_left = -1;

                M32(p->p_mvd[0][0]) 
                    = M32(p->p_mvd[0][3]) 
                    = M32(p->p_mvd[1][0]) 
                    = M32(p->p_mvd[1][3]) = 0;

                p->p_ref[0][3] 
                    = p->p_ref[0][6] 
                    = p->p_ref[1][3] 
                    = p->p_ref[1][6] = -1;
                     
            }
            else
            {
                M32(p->p_mvd[0][0]) = M32(p->p_mvd[0][2]);
                M32(p->p_mvd[0][3]) = M32(p->p_mvd[0][5]);
                M32(p->p_mvd[1][0]) = M32(p->p_mvd[1][2]);
                M32(p->p_mvd[1][3]) = M32(p->p_mvd[1][5]);

                p->p_ref[0][3] = p->p_ref[0][5];
                p->p_ref[0][6] = p->p_ref[0][8];
                p->p_ref[1][3] = p->p_ref[1][5];
                p->p_ref[1][6] = p->p_ref[1][8];
            }

            CP16(&p->p_ref[0][1], p->p_ref_top[p->i_mb_x][0]);
            CP16(&p->p_ref[1][1], p->p_ref_top[p->i_mb_x][1]);

            M64(p->p_mvd[0][1]) 
                = M64(p->p_mvd[1][1]) 
                = M64(p->p_mvd[0][4]) 
                = M64(p->p_mvd[1][4]) = 0;

            M16(&p->p_ref[0][4]) 
                = M16(&p->p_ref[0][7]) 
                = M16(&p->p_ref[1][4]) 
                = M16(&p->p_ref[1][7]) = -1;
            
            p->i_pred_mode_chroma_tab[p->i_mb_index] = 0;
        }
        
    }
    else
    {
        if(!(p->i_mb_flags & A_AVAIL))
        {
            p->mv[MV_FWD_A1] = MV_NOT_AVAIL;
            p->mv[MV_FWD_A3] = MV_NOT_AVAIL;
            p->mv[MV_BWD_A1] = MV_NOT_AVAIL;
            p->mv[MV_BWD_A3] = MV_NOT_AVAIL;

            p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
            p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;
        }

        if(p->i_mb_flags & B_AVAIL)
        {
            p->mv[MV_FWD_B2] = p->p_top_mv[0][i_offset];
            p->mv[MV_BWD_B2] = p->p_top_mv[1][i_offset];
            p->mv[MV_FWD_B3] = p->p_top_mv[0][i_offset+1];
            p->mv[MV_BWD_B3] = p->p_top_mv[1][i_offset+1];

            p->i_intra_pred_mode_y[1] = p->p_top_intra_pred_mode_y[i_offset_b4];
            p->i_intra_pred_mode_y[2] = p->p_top_intra_pred_mode_y[i_offset_b4+1];
            p->i_intra_pred_mode_y[3] = p->p_top_intra_pred_mode_y[i_offset_b4+2];
            p->i_intra_pred_mode_y[4] = p->p_top_intra_pred_mode_y[i_offset_b4+3];
        }
        else
        {
            p->mv[MV_FWD_B2] = MV_NOT_AVAIL;
            p->mv[MV_FWD_B3] = MV_NOT_AVAIL;
            p->mv[MV_BWD_B2] = MV_NOT_AVAIL;
            p->mv[MV_BWD_B3] = MV_NOT_AVAIL;
            p->i_intra_pred_mode_y[1] = p->i_intra_pred_mode_y[2] = 
            p->i_intra_pred_mode_y[3] = p->i_intra_pred_mode_y[4] = NOT_AVAIL;
        }

        if(p->i_mb_flags & C_AVAIL)
        {
            p->mv[MV_FWD_C2] = p->p_top_mv[0][i_offset+2];
            p->mv[MV_BWD_C2] = p->p_top_mv[1][i_offset+2];
        }
        else
        {
            p->mv[MV_FWD_C2] = MV_NOT_AVAIL;
            p->mv[MV_BWD_C2] = MV_NOT_AVAIL;
        }

        if(!(p->i_mb_flags & D_AVAIL))
        {
            p->mv[MV_FWD_D3] = MV_NOT_AVAIL;
            p->mv[MV_BWD_D3] = MV_NOT_AVAIL;	
        }

        p->p_col_type = &p->p_col_type_base[p->i_mb_y*p->i_mb_width + p->i_mb_x];

        p->vbs_mb_intra_pred_flag = 0;
        p->vbs_mb_part_intra_pred_flag[0] = p->vbs_mb_part_intra_pred_flag[1] =
        p->vbs_mb_part_intra_pred_flag[2] = p->vbs_mb_part_intra_pred_flag[3] =
        p->vbs_mb_part_transform_4x4_flag[0] = p->vbs_mb_part_transform_4x4_flag[1] =
        p->vbs_mb_part_transform_4x4_flag[2] = p->vbs_mb_part_transform_4x4_flag[3] = 0;

        if (p->ph.b_aec_enable)
        {
            if (!p->i_mb_x)
            {
                p->i_chroma_pred_mode_left = 0;
                p->i_cbp_left = -1;
                M32(p->p_mvd[0][0]) = M32(p->p_mvd[0][3]) = M32(p->p_mvd[1][0]) = M32(p->p_mvd[1][3]) = 0;
                p->p_ref[0][3] = p->p_ref[0][6] = p->p_ref[1][3] = p->p_ref[1][6] = -1;
            }
            else
            {
                M32(p->p_mvd[0][0]) = M32(p->p_mvd[0][2]);
                M32(p->p_mvd[0][3]) = M32(p->p_mvd[0][5]);
                M32(p->p_mvd[1][0]) = M32(p->p_mvd[1][2]);
                M32(p->p_mvd[1][3]) = M32(p->p_mvd[1][5]);
                p->p_ref[0][3] = p->p_ref[0][5];
                p->p_ref[0][6] = p->p_ref[0][8];
                p->p_ref[1][3] = p->p_ref[1][5];
                p->p_ref[1][6] = p->p_ref[1][8];
            }
            CP16(&p->p_ref[0][1], p->p_ref_top[p->i_mb_x][0]);
            CP16(&p->p_ref[1][1], p->p_ref_top[p->i_mb_x][1]);
            M64(p->p_mvd[0][1]) = M64(p->p_mvd[1][1]) = M64(p->p_mvd[0][4]) = M64(p->p_mvd[1][4]) = 0;
            M16(&p->p_ref[0][4]) = M16(&p->p_ref[0][7]) = M16(&p->p_ref[1][4]) = M16(&p->p_ref[1][7]) = -1;
            p->i_pred_mode_chroma = 0;
        }
    }
    
}

char AVS_2DVLC_INTRA_dec_4x4[7][40][2] = {{{-1,-1}}};	
char AVS_2DVLC_INTER_dec_4x4[7][40][2] = {{{-1,-1}}};
const int SCAN_4x4[16][2] = 
{
	{0,0},{1,0},{0,1},{0,2},
	{1,1},{2,0},{3,0},{2,1},
	{1,2},{0,3},{1,3},{2,2},
	{3,1},{3,2},{2,3},{3,3}
};  

const int SCAN[2][64][2] = // [scan_pos][x/y] ATTENTION: the ScanPositions are (pix,lin)!
{
	{
		{0,0},{0,1},{0,2},{1,0},{0,3},{0,4},{1,1},{1,2},
		{0,5},{0,6},{1,3},{2,0},{2,1},{0,7},{1,4},{2,2},
		{3,0},{1,5},{1,6},{2,3},{3,1},{3,2},{4,0},{1,7},
		{2,4},{4,1},{2,5},{3,3},{4,2},{2,6},{3,4},{4,3},
		{5,0},{5,1},{2,7},{3,5},{4,4},{5,2},{6,0},{5,3},
		{3,6},{4,5},{6,1},{6,2},{5,4},{3,7},{4,6},{6,3},
		{5,5},{4,7},{6,4},{5,6},{6,5},{5,7},{6,6},{7,0},
		{6,7},{7,1},{7,2},{7,3},{7,4},{7,5},{7,6},{7,7}
	},
	{
		{ 0, 0}, { 1, 0}, { 0, 1}, { 0, 2}, { 1, 1}, { 2, 0}, { 3, 0}, { 2, 1},
		{ 1, 2}, { 0, 3}, { 0, 4}, { 1, 3}, { 2, 2}, { 3, 1}, { 4, 0}, { 5, 0},
		{ 4, 1}, { 3, 2}, { 2, 3}, { 1, 4}, { 0, 5}, { 0, 6}, { 1, 5}, { 2, 4},
		{ 3, 3}, { 4, 2}, { 5, 1}, { 6, 0}, { 7, 0}, { 6, 1}, { 5, 2}, { 4, 3},
		{ 3, 4}, { 2, 5}, { 1, 6}, { 0, 7}, { 1, 7}, { 2, 6}, { 3, 5}, { 4, 4},
		{ 5, 3}, { 6, 2}, { 7, 1}, { 7, 2}, { 6, 3}, { 5, 4}, { 4, 5}, { 3, 6},
		{ 2, 7}, { 3, 7}, { 4, 6}, { 5, 5}, { 6, 4}, { 7, 3}, { 7, 4}, { 6, 5},
		{ 5, 6}, { 4, 7}, { 5, 7}, { 6, 6}, { 7, 5}, { 7, 6}, { 6, 7}, { 7, 7}
	}
};

static UNUSED void cavs_inv_transform_B8 (int curr_blk[8][8])
{
    int xx, yy;
    int tmp[8];
    int t;
    int b[8];
    int clip1, clip2; 

    clip1 = 0-(1<<(8+7));
    clip2 = (1<<(8+7))-1;

    for( yy = 0; yy < 8; yy++ )
    {
         /* Horizontal inverse transform */
         /* Reorder */
         tmp[0] = curr_blk[yy][0];
         tmp[1] = curr_blk[yy][4];
         tmp[2] = curr_blk[yy][2];
         tmp[3] = curr_blk[yy][6];
         tmp[4] = curr_blk[yy][1];
         tmp[5] = curr_blk[yy][3];
         tmp[6] = curr_blk[yy][5];
         tmp[7] = curr_blk[yy][7];
         
         /* Downleft Butterfly */
         b[0] = ((tmp[4] - tmp[7])<<1) + tmp[4];
         b[1] = ((tmp[5] + tmp[6])<<1) + tmp[5];
         b[2] = ((tmp[5] - tmp[6])<<1) - tmp[6];
         b[3] = ((tmp[4] + tmp[7])<<1) + tmp[7];

         b[4] = ((b[0] + b[1] + b[3])<<1) + b[1];
         b[5] = ((b[0] - b[1] + b[2])<<1) + b[0];
         b[6] = ((-b[1] - b[2] + b[3])<<1) + b[3];
         b[7] = ((b[0] - b[2] - b[3])<<1) - b[2];
         
         /* Upleft Butterfly */
         t=((tmp[2]*10)+(tmp[3]<<2));
         tmp[3]=((tmp[2]<<2)-(tmp[3]*10));
         tmp[2]=t;
         
         t = (tmp[0]+tmp[1])<<3;
         tmp[1] = (tmp[0]-tmp[1])<<3;
         tmp[0] = t;
         
         b[0]=tmp[0]+tmp[2];
         b[1]=tmp[1]+tmp[3];
         b[2]=tmp[1]-tmp[3];
         b[3]=tmp[0]-tmp[2];	 
         
         /* Last Butterfly */
         curr_blk[yy][0] = (int)((Clip3(clip1,clip2,((b[0]+b[4])+ 4)))>>3);
         curr_blk[yy][1] = (int)((Clip3(clip1,clip2,((b[1]+b[5])+ 4)))>>3);
         curr_blk[yy][2] = (int)((Clip3(clip1,clip2,((b[2]+b[6])+ 4)))>>3);
         curr_blk[yy][3] = (int)((Clip3(clip1,clip2,((b[3]+b[7])+ 4)))>>3);
         curr_blk[yy][7] = (int)((Clip3(clip1,clip2,((b[0]-b[4])+ 4)))>>3);
         curr_blk[yy][6] = (int)((Clip3(clip1,clip2,((b[1]-b[5])+ 4)))>>3);
         curr_blk[yy][5] = (int)((Clip3(clip1,clip2,((b[2]-b[6])+ 4)))>>3);
         curr_blk[yy][4] = (int)((Clip3(clip1,clip2,((b[3]-b[7])+ 4)))>>3);   
    }
    
    // Vertical inverse transform
    for(xx=0; xx<8; xx++)
    {
         /* Reorder */
         tmp[0] = curr_blk[0][xx];
         tmp[1] = curr_blk[4][xx];
         tmp[2] = curr_blk[2][xx];
         tmp[3] = curr_blk[6][xx];
         tmp[4] = curr_blk[1][xx];
         tmp[5] = curr_blk[3][xx];
         tmp[6] = curr_blk[5][xx];
         tmp[7] = curr_blk[7][xx];
         
        /* Downleft Butterfly */
        b[0] = ((tmp[4] - tmp[7])<<1) + tmp[4];
        b[1] = ((tmp[5] + tmp[6])<<1) + tmp[5];
        b[2] = ((tmp[5] - tmp[6])<<1) - tmp[6];
        b[3] = ((tmp[4] + tmp[7])<<1) + tmp[7];

        b[4] = ((b[0] + b[1] + b[3])<<1) + b[1];
        b[5] = ((b[0] - b[1] + b[2])<<1) + b[0];
        b[6] = ((-b[1] - b[2] + b[3])<<1) + b[3];
        b[7] = ((b[0] - b[2] - b[3])<<1) - b[2];

        /* Upleft Butterfly */
        t = ((tmp[2]*10)+(tmp[3]<<2));
        tmp[3] = ((tmp[2]<<2)-(tmp[3]*10));
        tmp[2] = t;

        t = (tmp[0]+tmp[1])<<3;
        tmp[1] = (tmp[0]-tmp[1])<<3;
        tmp[0] = t;

        b[0] = tmp[0] + tmp[2];
        b[1] = tmp[1] + tmp[3];
        b[2] = tmp[1] - tmp[3];
        b[3] = tmp[0] - tmp[2];

        curr_blk[0][xx]=(Clip3(clip1,clip2,(b[0]+b[4])+64))>>7;
        curr_blk[1][xx]=(Clip3(clip1,clip2,(b[1]+b[5])+64))>>7;
        curr_blk[2][xx]=(Clip3(clip1,clip2,(b[2]+b[6])+64))>>7;
        curr_blk[3][xx]=(Clip3(clip1,clip2,(b[3]+b[7])+64))>>7;
        curr_blk[7][xx]=(Clip3(clip1,clip2,(b[0]-b[4])+64))>>7;
        curr_blk[6][xx]=(Clip3(clip1,clip2,(b[1]-b[5])+64))>>7;
        curr_blk[5][xx]=(Clip3(clip1,clip2,(b[2]-b[6])+64))>>7;
        curr_blk[4][xx]=(Clip3(clip1,clip2,(b[3]-b[7])+64))>>7;
    }
}

static UNUSED void cavs_add_c(uint8_t *dst, int idct8x8[8][8], int stride )
{
    int i;

    uint8_t *cm = crop_table + MAX_NEG_CROP;

    for( i = 0; i < 8; i++ ) 
    {
        dst[i + 0*stride] = cm[ dst[i + 0*stride] + idct8x8[0][i]];
        dst[i + 1*stride] = cm[ dst[i + 1*stride] + idct8x8[1][i]];
        dst[i + 2*stride] = cm[ dst[i + 2*stride] + idct8x8[2][i]];
        dst[i + 3*stride] = cm[ dst[i + 3*stride] + idct8x8[3][i]];
        dst[i + 4*stride] = cm[ dst[i + 4*stride] + idct8x8[4][i]];
        dst[i + 5*stride] = cm[ dst[i + 5*stride] + idct8x8[5][i]];
        dst[i + 6*stride] = cm[ dst[i + 6*stride] + idct8x8[6][i]];
        dst[i + 7*stride] = cm[ dst[i + 7*stride] + idct8x8[7][i]];
    }
}

static int get_residual_block(cavs_decoder *p,const cavs_vlc *p_vlc_table, 
	int i_escape_order,int i_qp, uint8_t *p_dest, int i_stride, int b_chroma)
{
    int i,pos = -1;
    int *level_buf = p->level_buf;
    int *run_buf = p->run_buf;
    int dqm = dequant_mul[i_qp];
    int dqs = dequant_shift[i_qp];
    int dqa = 1 << (dqs - 1);
    const uint8_t *zigzag = p->ph.b_picture_structure==0 ? zigzag_field : zigzag_progressive;
    DCTELEM *block = p->p_block;

    DECLARE_ALIGNED_16(short dct8x8[8][8]);

    memset(dct8x8, 0, 64*sizeof(int16_t));

    i = p->bs_read_coeffs(p, p_vlc_table, i_escape_order, b_chroma);
    if( i == -1 || p->b_error_flag )
    {
        return -1;
    }
  
    if( !p->b_weight_quant_enable )
    {
    	while(--i >= 0)
    	{
    		pos += run_buf[i];
    		if(pos > 63) 
    		{
    			p->b_error_flag = 1;
            
    			return -1;
    		}

    		block[zigzag[pos]] = (level_buf[i]*dqm + dqa) >> dqs;
    	}
    }
    else
    {
    	int m, n;

    	while(--i >= 0)
    	{
    		pos += run_buf[i];
    		if(pos > 63) 
    		{
    			p->b_error_flag = 1;
            
    			return -1;
    		}

    		m = SCAN[p->ph.b_picture_structure][pos][0];
    		n = SCAN[p->ph.b_picture_structure][pos][1];

             /* NOTE : bug will conflict with idct asm when Win32-Release, don't why now */
    		block[zigzag[pos]] = (((((int)(level_buf[i]*p->cur_wq_matrix[n*8+m])>>3)*dqm)>>4) + dqa) >> dqs;
    	}
    }

	cavs_idct8_add(p_dest, block, i_stride);

    return 0;
}

static int get_residual_block4x4(cavs_decoder *p, int i_qp, uint8_t *p_dest4x4, int i_stride, int intra)
{
    int i,ipos;
    int level, run, mask, symbol2D;
    int level_buf[64];
    int run_buf[64];
    int dqm, dqs, dqa;

    static const int incLevel_intra[7]={0,1,2,3,4,6,3000};
    static const int incRun_intra[7]={-1,3,17,17,17,17,17};
    static const int incLevel_inter[7]={0,1,1,2,3,5,3000};
    static const int incRun_inter[7]={-1,3,6,17,17,17,17};
    static const int level1_inter[16]={1,1,1,1,2,2,2,3,3,3,3,3,3,3,3,3};
    static const int level1_intra[16]={1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2};
    DCTELEM *block = p->p_block;
    int CurrentVLCTable=0;
    int abslevel;

    int EOB_Pos_4x4[7] = {-1,0,0,0,0,0,0};
    const char (*table2D_4x4)[16];
    const char (*table2D_inter_4x4)[14];
#define assert(_Expression)     ((void)0)

    int PreviousRunCnt = 0;
    int vlc_numcoef = 0;
    int ii, jj;
    int abs_lev_diff;

    dqm = dequant_mul[i_qp - p->ph.vbs_qp_shift];
    dqs = dequant_shift[i_qp - p->ph.vbs_qp_shift]/* - 1*/;
    dqa = 1 << (dqs - 1);

    if(AVS_2DVLC_INTRA_dec_4x4[0][0][1]<0)            
    {
    	memset(AVS_2DVLC_INTRA_dec_4x4,-1,sizeof(AVS_2DVLC_INTRA_dec_4x4));
    	for(i=0;i<7;i++)
    	{
    		table2D_4x4=intra_2dvlc_4x4[i];
    		for(run=0;run<16;run++)
    			for(level=0;level<16;level++)
    			{
    				ipos=table2D_4x4[run][level];
    				assert(ipos<40);
    				if(ipos>=0)
    				{
    					if(i==0)
    					{
    						AVS_2DVLC_INTRA_dec_4x4[i][ipos][0]=level+1;
    						AVS_2DVLC_INTRA_dec_4x4[i][ipos][1]=run;

    						AVS_2DVLC_INTRA_dec_4x4[i][ipos+1][0]=-(level+1);
    						AVS_2DVLC_INTRA_dec_4x4[i][ipos+1][1]=run;
    					}
    					else
    					{
    						AVS_2DVLC_INTRA_dec_4x4[i][ipos][0]=level;
    						AVS_2DVLC_INTRA_dec_4x4[i][ipos][1]=run;              
    						if(level)
    						{
    							AVS_2DVLC_INTRA_dec_4x4[i][ipos+1][0]=-(level);
    							AVS_2DVLC_INTRA_dec_4x4[i][ipos+1][1]=run;
    						}
    					}
    				}
    			}
    	}
    	assert(AVS_2DVLC_INTRA_dec_4x4[0][0][1]>=0);        //otherwise, tables are bad.
    }
    if(AVS_2DVLC_INTER_dec_4x4[0][0][1]<0)                                                          // Don't need to set this every time. rewrite later.
    {
    	memset(AVS_2DVLC_INTER_dec_4x4,-1,sizeof(AVS_2DVLC_INTER_dec_4x4));
    	for(i=0;i<7;i++)
    	{
    		table2D_inter_4x4=inter_2dvlc_4x4[i];
    		for(run=0;run<16;run++)
    			for(level=0;level<14;level++)
    			{
    				ipos=table2D_inter_4x4[run][level];
    				assert(ipos<40);
    				if(ipos>=0)
    				{
    					if(i==0)
    					{
    						AVS_2DVLC_INTER_dec_4x4[i][ipos][0]=level+1;
    						AVS_2DVLC_INTER_dec_4x4[i][ipos][1]=run;

    						AVS_2DVLC_INTER_dec_4x4[i][ipos+1][0]=-(level+1);
    						AVS_2DVLC_INTER_dec_4x4[i][ipos+1][1]=run;
    					}
    					else
    					{
    						AVS_2DVLC_INTER_dec_4x4[i][ipos][0]=level;
    						AVS_2DVLC_INTER_dec_4x4[i][ipos][1]=run;

    						if(level)
    						{
    							AVS_2DVLC_INTER_dec_4x4[i][ipos+1][0]=-(level);
    							AVS_2DVLC_INTER_dec_4x4[i][ipos+1][1]=run;
    						}
    					}
    				}
    			}
    	}
    	assert(AVS_2DVLC_INTER_dec_4x4[0][0][1]>=0);        //otherwise, tables are bad.
    }

    if (intra)
    {
    	CurrentVLCTable = 0;
    	PreviousRunCnt = 0;
    	for(i=0;i<17;i++)
    	{
    		symbol2D = cavs_bitstream_get_ue_k(&p->s,VLC_GC_Order_INTRA[CurrentVLCTable][0]);
    		if(symbol2D==EOB_Pos_4x4[CurrentVLCTable])
    		{
    			vlc_numcoef = i;  // 0~i last is "0"? rm上的
    			break;
    		}
    		if(symbol2D >= ESCAPE_CODE_PART2)
    		{
    			abs_lev_diff = 2+(symbol2D-ESCAPE_CODE_PART2)/2;
    			mask = -(symbol2D & 1);//0或-1
    			symbol2D = cavs_bitstream_get_ue_k(&p->s,0);
    			run = symbol2D;
				if (run > 16)
				{
					p->b_error_flag = 1;
					return -1;
				}
    			level = abs_lev_diff + LEVRUN_INTRA[CurrentVLCTable][run];
    			level = (level^mask) - mask;
    		}
    		else if (symbol2D>=ESCAPE_CODE_PART1)
    		{
    			run =((symbol2D-ESCAPE_CODE_PART1)&31)/2;
    			level =1+LEVRUN_INTRA[CurrentVLCTable][run];
    			mask = -(symbol2D & 1);//0或-1
    			level = (level^mask) - mask;
    		} 
    		else
    		{
    			level = AVS_2DVLC_INTRA_dec_4x4[CurrentVLCTable][symbol2D][0];
    			run   = AVS_2DVLC_INTRA_dec_4x4[CurrentVLCTable][symbol2D][1];
    		}
    		level_buf[i] = level;
    		run_buf[i] = run;
    		PreviousRunCnt += run+1;
    		
    		abslevel = abs(level);
    		if(abslevel>=incLevel_intra[CurrentVLCTable])
    		{
				if (abslevel == 1)
				{
					if (run > 16)
					{
						p->b_error_flag = 1;
						return -1;
					}

					CurrentVLCTable = (run > incRun_intra[CurrentVLCTable]) ? level1_intra[run] : CurrentVLCTable;
				}
				else if (abslevel < 4)
				{
					CurrentVLCTable = (abslevel + 1);
				}
				else if (abslevel < 6)
				{
					CurrentVLCTable = 5;
				}
				else
				{
					CurrentVLCTable = 6;
				}
    		}			

    		if (PreviousRunCnt==16)
    		{
    			vlc_numcoef=i+1;
    			break;
    		}
    	
    	}
    	ipos = -1;
    	for (i = 0; i<vlc_numcoef; i++)
    	{
    		ipos += run_buf[vlc_numcoef-1-i]+1;
			if (ipos < 0 || ipos >= 16)
			{
				p->b_error_flag = 1;
				break;
			}
			ii = SCAN_4x4[ipos][0];
    		jj = SCAN_4x4[ipos][1];
    		
    		//Juncheng: It's for weighted quantization
    		block[jj*8 + ii] = (level_buf[vlc_numcoef-1-i]*dqm + dqa) >> dqs;
    	}
    }
    else	//inter
    {
    	CurrentVLCTable = 0;
    	PreviousRunCnt = 0;

    	for(i=0;i<17;i++)
    	{
    		symbol2D = cavs_bitstream_get_ue_k(&p->s,VLC_GC_Order_INTER[CurrentVLCTable][0]);
    		if(symbol2D==EOB_Pos_4x4[CurrentVLCTable])
    		{
    			vlc_numcoef = i;  // 0~i last is "0"? rm上的
    			break;
    		}
    		if(symbol2D >= ESCAPE_CODE_PART2)
    		{
    			abs_lev_diff = 2+(symbol2D-ESCAPE_CODE_PART2)/2;
    			mask = -(symbol2D & 1);//0或-1
    			symbol2D = cavs_bitstream_get_ue_k(&p->s,0);
    			run = symbol2D;
				if (run > 16)
				{
					p->b_error_flag = 1;
					return -1;
				}
    			level = abs_lev_diff + LEVRUN_INTER[CurrentVLCTable][run];
    			level = (level^mask) - mask;
    		}
    		else if (symbol2D>=ESCAPE_CODE_PART1)
    		{
    			run =((symbol2D-ESCAPE_CODE_PART1)&31)/2;
    			level =1 + LEVRUN_INTER[CurrentVLCTable][run];
    			mask = -(symbol2D & 1);//0或-1
    			level = (level^mask) - mask;
    		} 
    		else
    		{
    			level = AVS_2DVLC_INTER_dec_4x4[CurrentVLCTable][symbol2D][0];
    			run   = AVS_2DVLC_INTER_dec_4x4[CurrentVLCTable][symbol2D][1];
    		}
    		level_buf[i] = level;
    		run_buf[i] = run;
    		PreviousRunCnt += run+1;

    		abslevel = abs(level);
    		if(abslevel>=incLevel_inter[CurrentVLCTable])
    		{
				if (abslevel == 1)
				{
					if (run > 16)
					{
						p->b_error_flag = 1;
						return -1;
					}

					CurrentVLCTable = (run > incRun_inter[CurrentVLCTable]) ? level1_inter[run] : CurrentVLCTable;
				}
				else if (abslevel == 2)
				{
					CurrentVLCTable = 4;
				}
				else if (abslevel <= 4)
				{
					CurrentVLCTable = 5;
				}
				else
				{
					CurrentVLCTable = 6;
				}
			}

    		if (PreviousRunCnt==16)
    		{
    			vlc_numcoef=i+1;
    			break;
    		}

    	}
    	ipos = -1;
    	for (i = 0; i<vlc_numcoef; i++)
    	{
    		ipos += run_buf[vlc_numcoef-1-i]+1;
			if (ipos < 0 || ipos >= 16)
			{
				p->b_error_flag = 1;
				break;
			}
    		ii = SCAN_4x4[ipos][0];
    		jj = SCAN_4x4[ipos][1];
    		block[jj*8 + ii] = (level_buf[vlc_numcoef-1-i]*dqm + dqa) >> dqs;
    	}
    }

    add4x4_idct(p_dest4x4, block, i_stride);
    memset(block, 0, 64*sizeof(DCTELEM));
    
    return 0;
}

static inline void scale_mv(cavs_decoder *p, int *d_x, int *d_y, cavs_vector *src, int distp) 
{

    int den;

    if(abs(src->ref) > 3)
    {
    	p->b_error_flag = 1;
    	return;
    }

    den = p->i_scale_den[src->ref];

    *d_x = (src->x*distp*den + 256 + (src->x>>31)) >> 9;
    *d_y = (src->y*distp*den + 256 + (src->y>>31)) >> 9;
}

static inline int mid_pred(int a, int b, int c)
{
    if(a>b){
        if(c>b){
            if(c>a) b=a;
            else    b=c;
        }
    }else{
        if(b>c){
            if(c>a) b=c;
            else    b=a;
        }
    }
    return b;

}

static inline void mv_pred_median(cavs_decoder *p, cavs_vector *mvP, cavs_vector *mvA, cavs_vector *mvB, cavs_vector *mvC)
{
    int ax, ay, bx, by, cx, cy;
    int len_ab, len_bc, len_ca, len_mid;

    /* scale candidates according to their temporal span */
    scale_mv(p, &ax, &ay, mvA, mvP->dist);
    if(p->b_error_flag)
    	return;
    scale_mv(p, &bx, &by, mvB, mvP->dist);
    if(p->b_error_flag)
    	return;
    scale_mv(p, &cx, &cy, mvC, mvP->dist);
    if(p->b_error_flag)
    	return;
    
    /* find the geometrical median of the three candidates */
    len_ab = abs(ax - bx) + abs(ay - by);
    len_bc = abs(bx - cx) + abs(by - cy);
    len_ca = abs(cx - ax) + abs(cy - ay);
    len_mid = mid_pred(len_ab, len_bc, len_ca);
    
    if(len_mid == len_ab) 
    {
        mvP->x = cx;
        mvP->y = cy;
    }
    else if(len_mid == len_bc) 
    {
        mvP->x = ax;
        mvP->y = ay;
    }
    else 
    {
        mvP->x = bx;
        mvP->y = by;
    }
}


static inline int get_residual_inter(cavs_decoder *p,int i_mb_type) 
{      
    int block;
    int i_stride = p->cur.i_stride[0];
    uint8_t *p_d;
    int i_qp_cb, i_qp_cr;
	int i_ret = 0;

    for(block = 0; block < 4; block++)
    {
        if(p->i_cbp & (1<<block))
        {
            p_d = p->p_y + p->i_luma_offset[block];
            if (p->vbs_mb_part_transform_4x4_flag[block])
            {
                int i;
                
                for (i = 0; i < 4; i++)
                    if (p->i_cbp_part[block] & (1<<i))
                        get_residual_block4x4(p, p->i_qp, p_d + (i>>1)*4*i_stride + (i&1)*4, i_stride, 0);
            }
            else
            {
                i_ret = get_residual_block(p, inter_2dvlc, 0, p->i_qp, p_d, i_stride, 0);
				if(i_ret == -1)
					return -1;
            }
        }
    }

    /* FIXIT : weighting quant qp for chroma
        NOTE : not modify p->i_qp directly
    */
    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
    { 
        i_qp_cb = clip3_int( p->i_qp + p->ph.chroma_quant_param_delta_u, 0, 63  );
        i_qp_cr = clip3_int( p->i_qp + p->ph.chroma_quant_param_delta_v, 0, 63 );
    }

    if(p->i_cbp & (1<<4))
    {
        i_ret = get_residual_block(p, chroma_2dvlc, 0, 
                              chroma_qp[p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable ? i_qp_cb : p->i_qp],
                              p->p_cb, p->cur.i_stride[1], 1);
		if(i_ret == -1)
			return -1;
    }
    if(p->i_cbp & (1<<5))
    {
        i_ret = get_residual_block(p, chroma_2dvlc, 0, 
                              chroma_qp[ p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable ? i_qp_cr : p->i_qp],
                              p->p_cr, p->cur.i_stride[2], 1);
		if(i_ret == -1)
			return -1;
    }

    return 0;
}


static inline void adapt_pred_mode(int i_num, int *p_mode) 
{
    static const uint32_t adapt_tale[4][/*8*/20] =
    {
    	{ 0,-1, 6,-1,-1, 7, 6, 7,  8, 9, I_PRED_4x4_DC_TOP, 11, 12, 13, 14, 15, 16, 17, 18, 19},
    	{-1, 1, 5,-1,-1, 5, 7, 7,  8, 9, I_PRED_4x4_DC_LEFT, 11, 12, 13, 14, 15, 16, 17, I_PRED_4x4_DC_128, 19},
    	{ 5,-1, 2,-1, 6, 5, 6,-1,  8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19},
    	{ 4, 1,-1,-1, 4, 6, 6,-1,  8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}
    };
    
     *p_mode = adapt_tale[i_num][*p_mode];
    if(*p_mode  < 0) 
    {
        *p_mode  = 0;
    }
}

static inline void load_intra_pred_chroma(cavs_decoder *p) 
{
    int i_offset=p->i_mb_x*10;
    
    p->i_left_border_cb[9] = p->i_left_border_cb[8];
    p->i_left_border_cr[9] = p->i_left_border_cr[8];
    if(p->i_mb_x==p->i_mb_width-1)
    {
        p->p_top_border_cb[i_offset+9] = p->p_top_border_cb[i_offset+8];
        p->p_top_border_cr[i_offset+9] = p->p_top_border_cr[i_offset+8];
    }
    else
    {
        p->p_top_border_cb[i_offset+9] = p->p_top_border_cb[i_offset+11];
        p->p_top_border_cr[i_offset+9] = p->p_top_border_cr[i_offset+11];
    }

    if(p->i_mb_x && p->i_mb_y&&p->b_first_line==0) 
    {
        p->p_top_border_cb[i_offset] = p->i_left_border_cb[0] = p->i_topleft_border_cb;
        p->p_top_border_cr[i_offset] = p->i_left_border_cr[0] = p->i_topleft_border_cr;
    } 
    else
    {
        p->i_left_border_cb[0] = p->i_left_border_cb[1];
        p->i_left_border_cr[0] = p->i_left_border_cr[1];
        p->p_top_border_cb[i_offset] = p->p_top_border_cb[i_offset+1];
        p->p_top_border_cr[i_offset] = p->p_top_border_cr[i_offset+1];
    }
}

static void load_intra_4x4_pred_luma(cavs_decoder *p ,uint8_t *src, uint8_t edge[17], int idx)
{
#define SRC(x,y) src[(x) + (y)*i_stride]
#define EP (edge+8)
	int i_stride = p->cur.i_stride[0];
	int i_neighbor = p->i_neighbour4[idx];
	int have_lt = i_neighbor & MB_TOPLEFT;
	int have_tr = i_neighbor & MB_TOPRIGHT;
	int have_dl = i_neighbor & MB_DOWNLEFT;

	if( i_neighbor & MB_LEFT )
	{
		if (offset_block4x4[idx][0] == 0)
		{
			EP[-1] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 0];
			EP[-2] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 1];
			EP[-3] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 2];
			EP[-4] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 3];
			if (have_dl)
			{
				EP[-5] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 4];
				EP[-6] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 5];
				EP[-7] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 6];
				EP[-8] = p->i_left_border_y[1 + offset_block4x4[idx][1]*4 + 7];
			}
			else
			{
				MPIXEL_X4(&EP[-8]) = PIXEL_SPLAT_X4(EP[-4]);
			}
		}
		else
		{
			EP[-1] = SRC(-1, 0);
			EP[-2] = SRC(-1, 1);
			EP[-3] = SRC(-1, 2);
			EP[-4] = SRC(-1, 3);
			if (have_dl)
			{
				EP[-5] = SRC(-1, 4);
				EP[-6] = SRC(-1, 5);
				EP[-7] = SRC(-1, 6);
				EP[-8] = SRC(-1, 7);
			}
			else
			{
				//EP[-5] = EP[-6] = EP[-7] = EP[-8] = EP[-4];
				MPIXEL_X4(&EP[-8]) = PIXEL_SPLAT_X4(EP[-4]);
			}
		}
		
		EP[0] = EP[-1];
	}
	if (i_neighbor & MB_TOP)
	{
		if (offset_block4x4[idx][1] == 0)
		{
			MPIXEL_X4(&EP[1]) =
				MPIXEL_X4(&p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE+offset_block4x4[idx][0]*4]);
			if (have_tr)
			{
				MPIXEL_X4(&EP[5]) =
					MPIXEL_X4(&p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE+offset_block4x4[idx][0]*4+4]);
			}
			else
			{
				MPIXEL_X4(&EP[5]) =
					PIXEL_SPLAT_X4(p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE+offset_block4x4[idx][0]*4+3]);
			}
		}
		else
		{
			MPIXEL_X4(&EP[1]) = SRC_X4(0, -1);
			if (have_tr)
			{
				MPIXEL_X4(&EP[5]) = SRC_X4(4, -1);
			}
			else
			{
				MPIXEL_X4(&EP[5]) = PIXEL_SPLAT_X4(SRC(3, -1));
			}
		}
		
		EP[0] = EP[1];
	}

	if (have_lt)
	{
		if (idx == 0)
		{
			EP[0] = p->i_topleft_border_y;
		}
		else if (offset_block4x4[idx][0] == 0)
		{
			EP[0] = p->i_left_border_y[/*1 + */offset_block4x4[idx][1]*4 /*- 1*/];
		}
		else if (offset_block4x4[idx][1] == 0)
		{
			EP[0] = p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE + offset_block4x4[idx][0]*4 - 1];
		}
		else
		{
			EP[0] = SRC(-1,-1);
		}
	}
#undef SRC
#undef EP
}

static inline void load_intra_pred_luma(cavs_decoder *p, uint8_t *p_top,uint8_t **pp_left, int block) 
{
    int i;
    
    switch( block ) 
    {
        case 0:
            *pp_left = p->i_left_border_y;
            p->i_left_border_y[0] = p->i_left_border_y[1];
            //memset( &p->i_left_border_y[17], p->i_left_border_y[16], 9); 
            memcpy( &p_top[1], &p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE], 16);
            p_top[17] = p_top[16];
            p_top[0] = p_top[1];
            if((p->i_mb_flags & A_AVAIL) && (p->i_mb_flags & B_AVAIL))
                p->i_left_border_y[0] = p_top[0] = p->i_topleft_border_y;
            break;
            
        case 1:
            *pp_left = p->i_internal_border_y;
            for( i = 0; i < 8; i++)
            {
                p->i_internal_border_y[i+1] = *(p->p_y + 7 + i*p->cur.i_stride[0]);
            }
            
            //由于下边块（索引为3）未解码所以9以后的象素直接用象素8的值
            //当解码1，3块的时候由于左边块c[i]只有1~8是可用的，其他的还未解码，所以不可用
            //所以不可能采用Intra_8x8_Down_Left,Intra_8x8_Down_Right模式，所以不可能采用17位的象素
            memset( &p->i_internal_border_y[9], p->i_internal_border_y[8], 9);
            p->i_internal_border_y[0] = p->i_internal_border_y[1];
            memcpy( &p_top[1], &p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE+8], 8 );
            if( p->i_mb_flags & C_AVAIL )
                memcpy( &p_top[9], &p->p_top_border_y[(p->i_mb_x + 1)*CAVS_MB_SIZE], 8 );
            else
                memset( &p_top[9], p_top[8], 9 );
            p_top[17] =p_top[16];
            p_top[0] = p_top[1];
            if(p->i_mb_flags & B_AVAIL)
                p->i_internal_border_y[0] = p_top[0] = p->p_top_border_y[p->i_mb_x*CAVS_MB_SIZE+7];
            break;
            
        case 2:
            *pp_left = &p->i_left_border_y[8];
            memcpy( &p_top[1], p->p_y + 7*p->cur.i_stride[0], 16);
            p_top[17] = p_top[16];
            p_top[0] = p_top[1];
            if(p->i_mb_flags & A_AVAIL)
                p_top[0] = p->i_left_border_y[8];
            break;
        
        case 3:
            *pp_left = &p->i_internal_border_y[8];
            for(i=0;i<8;i++)
            {
                p->i_internal_border_y[i+9] = *(p->p_y + 7 + (i+8)*p->cur.i_stride[0]);
            }
            memset( &p->i_internal_border_y[17], p->i_internal_border_y[16], 9 );
            memcpy( &p_top[0], p->p_y + 7 + 7*p->cur.i_stride[0], 9 );
            memset( &p_top[9], p_top[8], 9 );
            break;
    }
}

static inline void mv_pred_sym(cavs_decoder *p, cavs_vector *src, enum cavs_block_size size,int i_ref) 
{
    cavs_vector *dst = src + MV_BWD_OFFS;

    /* backward mv is the scaled and negated forward mv */
    if(p->ph.b_picture_structure==0)
    {
        int  iBlockDistanceBw, iBlockDistanceFw;
        
        dst->ref = 1-i_ref;

        if( (abs(i_ref) > 1) 
        || (abs(dst->ref) > 3) )
        {
            p->b_error_flag = 1;
            return;
        }

		if( p->param.b_accelerate )
		{
			iBlockDistanceFw = (p->cur_aec.i_distance_index-p->ref_aec[i_ref+2].i_distance_index+512)%512;
			iBlockDistanceBw = (p->ref_aec[dst->ref].i_distance_index-p->cur_aec.i_distance_index+512)%512;
		}
		else
		{
			iBlockDistanceFw = (p->cur.i_distance_index-p->ref[i_ref+2].i_distance_index+512)%512;
			iBlockDistanceBw = (p->ref[dst->ref].i_distance_index-p->cur.i_distance_index+512)%512;
		}

        if( iBlockDistanceFw == 0 )
        {
            p->b_error_flag = 1;
            return;
        }
        
        dst->x = -((src->x * iBlockDistanceBw*(512/iBlockDistanceFw) + 256) >> 9);
        dst->y = -((src->y * iBlockDistanceBw*(512/iBlockDistanceFw) + 256) >> 9);
    }
    else
    {
        dst->ref = i_ref;
        dst->x = -((src->x * p->i_sym_factor + 256) >> 9);
        dst->y = -((src->y * p->i_sym_factor + 256) >> 9);
    }
    
    copy_mvs(dst, size);
}

//down8x8(for mvd context of cabac only): 1 if the block index is 2 or 3, 0 if it's 0 or 1
static void mv_pred(cavs_decoder *p, uint8_t nP,uint8_t nC,
                    enum cavs_mv_pred mode, enum cavs_block_size size, int ref, int mvd_scan_idx) 
{
    cavs_vector *mvP = &p->mv[nP];
    cavs_vector *mvA = &p->mv[nP-1];
    cavs_vector *mvB = &p->mv[nP-4];
    cavs_vector *mvC = &p->mv[nC];
    const cavs_vector *mvP2 = NULL;
    const int i_list = nP >= MV_BWD_OFFS;

    mvP->ref = ref;
    mvP->dist = p->i_ref_distance[mvP->ref];
    if(mvC->ref == NOT_AVAIL)
        mvC = &p->mv[nP-5]; // set to top-left (mvD)
    if((mode == MV_PRED_PSKIP) &&
       ((mvA->ref == NOT_AVAIL) || (mvB->ref == NOT_AVAIL) ||
           ((mvA->x | mvA->y /*| mvA->ref*/) == 0 && mvA->ref == PSKIP_REF)  ||
           ((mvB->x | mvB->y /*| mvB->ref*/) == 0 && mvB->ref == PSKIP_REF) )) 
    {
        mvP2 = &MV_NOT_AVAIL;
        /* if there is only one suitable candidate, take it */
    }
    else if((mvA->ref >= 0) && (mvB->ref < 0) && (mvC->ref < 0)) 
    {
        mvP2= mvA;
    }
    else if((mvA->ref < 0) && (mvB->ref >= 0) && (mvC->ref < 0)) 
    {
        mvP2= mvB;
    }
    else if((mvA->ref < 0) && (mvB->ref < 0) && (mvC->ref >= 0)) 
    {
        mvP2= mvC;
    } 
    else if(mode == MV_PRED_LEFT     && mvA->ref == ref)
    {
        mvP2= mvA;
    }
    else if(mode == MV_PRED_TOP      && mvB->ref == ref)
    {
        mvP2= mvB;
    }
    else if(mode == MV_PRED_TOPRIGHT && mvC->ref == ref)
    {
        mvP2= mvC;
    }
    
    if(mvP2)
    {
        mvP->x = mvP2->x;
        mvP->y = mvP2->y;
    }
    else
    {
        mv_pred_median(p, mvP, mvA, mvB, mvC);
        if(p->b_error_flag)
            return;
    }

    if(mode < MV_PRED_PSKIP)
    {
        mvP->x += p->bs_read_mvd(p, i_list, mvd_scan_idx, 0);
        if( p->b_error_flag )
        	return;
        mvP->y += p->bs_read_mvd(p, i_list, mvd_scan_idx, 1);
        if( p->b_error_flag )
        	return;        
    }
    copy_mvs(mvP,size);
}

static void mv_pred_sub_direct(cavs_decoder *p,cavs_vector *mv, int i_Offset,uint8_t nP,uint8_t nC,
                    enum cavs_mv_pred mode, enum cavs_block_size size, int ref) 
{
    cavs_vector *mvP = &p->mv[nP];
    cavs_vector *mvA = &mv[MV_FWD_A1+i_Offset];
    cavs_vector *mvB = &mv[MV_FWD_B2+i_Offset];
    cavs_vector *mvC = &mv[MV_FWD_C2+i_Offset];
    const cavs_vector *mvP2 = NULL;

    mvP->ref = ref;
    mvP->dist = p->i_ref_distance[mvP->ref];
    if(mvC->ref == NOT_AVAIL)
        mvC = &mv[MV_FWD_D3+i_Offset]; // set to top-left (mvD)
    if((mode == MV_PRED_PSKIP) &&
       ((mvA->ref == NOT_AVAIL) || (mvB->ref == NOT_AVAIL) ||
           ((mvA->x | mvA->y | mvA->ref) == 0)  ||
           ((mvB->x | mvB->y | mvB->ref) == 0) )) 
    {
        mvP2 = &MV_NOT_AVAIL;
        /* if there is only one suitable candidate, take it */
    }
    else if((mvA->ref >= 0) && (mvB->ref < 0) && (mvC->ref < 0)) 
    {
        mvP2= mvA;
    }
    else if((mvA->ref < 0) && (mvB->ref >= 0) && (mvC->ref < 0)) 
    {
        mvP2= mvB;
    }
    else if((mvA->ref < 0) && (mvB->ref < 0) && (mvC->ref >= 0)) 
    {
        mvP2= mvC;
    } 
    else if(mode == MV_PRED_LEFT     && mvA->ref == ref)
    {
        mvP2= mvA;
    }
    else if(mode == MV_PRED_TOP      && mvB->ref == ref)
    {
        mvP2= mvB;
    }
    else if(mode == MV_PRED_TOPRIGHT && mvC->ref == ref)
    {
        mvP2= mvC;
    }
    if(mvP2)
    {
        mvP->x = mvP2->x;
        mvP->y = mvP2->y;
    }
    else
    {
        mv_pred_median(p, mvP, mvA, mvB, mvC);
		if(p->b_error_flag)
			return;
    }
}

static inline void get_b_frame_ref(cavs_decoder *p, int *p_ref)
{
    if(p->ph.b_picture_structure==0&&p->ph.b_picture_reference_flag==0 )
    {
        *p_ref = cavs_bitstream_get_bit1(&p->s);	//mb_reference_index
    }
    else
    {	
        *p_ref=0;
    }
}

static const uint8_t mv_scan[4] = 
{
	MV_FWD_X0,MV_FWD_X1,
	MV_FWD_X2,MV_FWD_X3
};

static const uint8_t mvd_scan[4] =
{
	1, 2, 4, 5
};

static const uint8_t ref_scan[4] =
{
	4, 5, 7, 8
};

static inline void mv_pred_direct(cavs_decoder *p, cavs_vector *pmv_fw, cavs_vector *col_mv,
	int iDistanceIndexRef, int iDistanceIndexFw, int iDistanceIndexBw,
	int deltaRef, int deltaMvFw, int deltaMvBw) 
{
    int iDistanceIndexCur = p->param.b_accelerate?p->cur_aec.i_distance_index:p->cur.i_distance_index;
    int iBlockDistanceRef;
    cavs_vector *pmv_bw = pmv_fw + MV_BWD_OFFS;	

    int den;
    int m = col_mv->x >> 31;

    if(abs(col_mv->ref) > 3)
    {
        p->b_error_flag = 1;
        return;
    }
    
    den = p->i_direct_den[col_mv->ref];
    
    iBlockDistanceRef = (iDistanceIndexBw - iDistanceIndexRef + 512) % 512;
    pmv_fw->dist = (iDistanceIndexCur - iDistanceIndexFw + 512) % 512;
    pmv_bw->dist = (iDistanceIndexBw - iDistanceIndexCur + 512) % 512;
    den = (iBlockDistanceRef == 0) ? 0 : 16384/iBlockDistanceRef;

    if(m)
    {	
        pmv_fw->x = -((den-den*col_mv->x*pmv_fw->dist-1)>>14);
        pmv_bw->x = (den-den*col_mv->x*pmv_bw->dist-1)>>14;
    }
    else
    {
        pmv_fw->x = ((den+den*col_mv->x*pmv_fw->dist-1)>>14);
        pmv_bw->x = -((den+den*col_mv->x*pmv_bw->dist-1)>>14);
    }
    
    m = (col_mv->y + deltaRef) >> 31;
    if(m)
    {
        pmv_fw->y = -((den-den*(col_mv->y+deltaRef)*pmv_fw->dist-1)>>14) - deltaMvFw;
        pmv_bw->y = ((den-den*(col_mv->y+deltaRef)*pmv_bw->dist-1)>>14) - deltaMvBw;
    }
    else
    {
        pmv_fw->y = ((den+den*(col_mv->y+deltaRef)*pmv_fw->dist-1)>>14) - deltaMvFw;
        pmv_bw->y = -((den+den*(col_mv->y+deltaRef)*pmv_bw->dist-1)>>14) - deltaMvBw;
    }
}

static inline void get_col_info(cavs_decoder *p, uint8_t *p_col_type, cavs_vector **pp_mv, int block)
{
    int i_mb_offset = p->i_mb_index >= p->i_mb_num_half ? p->i_mb_num_half : 0;
    int b_interlaced = p->param.b_interlaced;

    if( !b_interlaced )
    {
    	*pp_mv = &p->p_col_mv[(p->i_mb_index /*- NEW_DIRECT * i_mb_offset*/)*4+block];
    	*p_col_type = p->p_col_type_base[(p->i_mb_index/* - NEW_DIRECT * i_mb_offset*/)];
    }
    else
    {
    	*pp_mv = &p->p_col_mv[(p->i_mb_index - NEW_DIRECT * i_mb_offset)*4+block];
    	*p_col_type = p->p_col_type_base[(p->i_mb_index - NEW_DIRECT * i_mb_offset)];
    }

    if( p->param.b_accelerate )
    {
        if( p->p_save_aec[1] == NULL )
        {
            p->b_error_flag = 1;
            return;
        }

        if(p->ph.b_picture_structure && p->p_save_aec[1]->b_picture_structure == 0)
        {
            i_mb_offset = p->i_mb_y/2*p->i_mb_width;
            *p_col_type = p->p_col_type_base[i_mb_offset + p->i_mb_x];
            *pp_mv = &p->p_col_mv[(i_mb_offset+p->i_mb_x)*4+(block%2)+2*(p->i_mb_y%2)];
        }
        
        if(p->ph.b_picture_structure == 0 && p->p_save_aec[1]->b_picture_structure)
        {
            i_mb_offset = p->i_mb_y*p->i_mb_width-i_mb_offset;
            i_mb_offset *= 2;
            if(block > 1)
                i_mb_offset += p->i_mb_width;
            *p_col_type = p->p_col_type_base[i_mb_offset + p->i_mb_x];
            *pp_mv = &p->p_col_mv[(i_mb_offset+p->i_mb_x)*4+(block%2)];
        }
    }
    else
    {
        if( p->p_save[1] == NULL )
        {
            p->b_error_flag = 1;
            return;
        }

        if(p->ph.b_picture_structure && p->p_save[1]->b_picture_structure == 0)
        {
            i_mb_offset = p->i_mb_y/2*p->i_mb_width;
            *p_col_type = p->p_col_type_base[i_mb_offset + p->i_mb_x];
            *pp_mv = &p->p_col_mv[(i_mb_offset+p->i_mb_x)*4+(block%2)+2*(p->i_mb_y%2)];
        }
        
        if(p->ph.b_picture_structure == 0 && p->p_save[1]->b_picture_structure)
        {
            i_mb_offset = p->i_mb_y*p->i_mb_width-i_mb_offset;
            i_mb_offset *= 2;
            if(block > 1)
                i_mb_offset += p->i_mb_width;
            *p_col_type = p->p_col_type_base[i_mb_offset + p->i_mb_x];
            *pp_mv = &p->p_col_mv[(i_mb_offset+p->i_mb_x)*4+(block%2)];
        }
    }
    
}

static void get_b_direct_skip_sub_mb(cavs_decoder *p,int block,cavs_vector *p_mv)
{
    cavs_vector temp_mv;
    uint32_t b_next_field = p->i_mb_index >= p->i_mb_num_half;
    int iDistanceIndexFw, iDistanceIndexBw, iDistanceIndexRef;
    int iDistanceIndexFw0, iDistanceIndexFw1, iDistanceIndexBw0, iDistanceIndexBw1;
    int deltaRef = 0, deltaMvBw = 0, deltaMvFw = 0;

    if( p->param.b_accelerate )
    {
        if( p->p_save_aec[1] == NULL 
        || p->p_save_aec[2] == NULL )
        {
            p->b_error_flag = 1;
            return;
        }

        temp_mv = p_mv[0];
        if(p->ph.b_picture_structure) /* frame */
        {
            /* direct prediction from co-located P MB, block-wise */
            if(p->p_save_aec[1]->b_picture_structure == 0) /* ref-field */
            {
                temp_mv.y *= 2;
                if( abs(temp_mv.ref)/2 > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                iDistanceIndexRef = p->p_save_aec[1]->i_ref_distance[temp_mv.ref/2];
            }
            else
            {
                if( abs(temp_mv.ref) > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                iDistanceIndexRef = p->p_save_aec[1]->i_ref_distance[temp_mv.ref];
            }
            iDistanceIndexFw = p->p_save_aec[2]->i_distance_index;
            iDistanceIndexBw = p->p_save_aec[1]->i_distance_index;
            p->mv[mv_scan[block]].ref = 1;
            p->mv[mv_scan[block]+MV_BWD_OFFS].ref = 0;

            mv_pred_direct(p,&p->mv[mv_scan[block]], &temp_mv,
                    iDistanceIndexRef, iDistanceIndexFw, iDistanceIndexBw, 0, 0, 0);
            if( p->b_error_flag )
                return;
        } 
        else /* field */ 
        {
            if(p->p_save_aec[1]->b_picture_structure == 1) /* ref-frame */
            {
                temp_mv.y /= 2;
                if( temp_mv.ref > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                iDistanceIndexRef = p->p_save_aec[1]->i_ref_distance[temp_mv.ref] + 1;	
            }
            else
            {
                int i_ref = temp_mv.ref;

                if( (abs(temp_mv.ref)>>1) > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                    
                if (NEW_DIRECT)
                {
                    iDistanceIndexRef = p->p_save_aec[1]->i_ref_distance[i_ref>>1] + !(i_ref & 1); /* NOTE : refer to reverse field */
                }
                else if(b_next_field && i_ref == 0)
                {
                    iDistanceIndexRef = p->p_save_aec[1]->i_distance_index;
                }
                else
                {
                    i_ref -= b_next_field;
                    iDistanceIndexRef = p->p_save_aec[1]->i_ref_distance[i_ref>>1] - (i_ref&1) + 1;
                }
            }

            /* set as field reference list */
            iDistanceIndexFw0 = p->p_save_aec[2]->i_distance_index+1;
            iDistanceIndexFw1 = p->p_save_aec[2]->i_distance_index;
            iDistanceIndexBw0 = p->p_save_aec[1]->i_distance_index;
            iDistanceIndexBw1 = p->p_save_aec[1]->i_distance_index+1;

            if(iDistanceIndexRef == iDistanceIndexFw0 || (NEW_DIRECT & b_next_field))
            {
                iDistanceIndexFw = iDistanceIndexFw0;
                p->mv[mv_scan[block]].ref = 2;
            }
            else
            {
                iDistanceIndexFw = iDistanceIndexFw1;
                p->mv[mv_scan[block]].ref = 3;
            }

            if(b_next_field==0 || NEW_DIRECT)
            {
                iDistanceIndexBw = iDistanceIndexBw0;
                p->mv[mv_scan[block]+MV_BWD_OFFS].ref = 0;
            }
            else
            {
                iDistanceIndexBw = iDistanceIndexBw1;
                p->mv[mv_scan[block]+MV_BWD_OFFS].ref = 1;
            }

            if (NEW_DIRECT)
            {
                deltaRef = ((temp_mv.ref & 1) ^ 1) << 1;
                deltaMvFw = (p->mv[mv_scan[block]].ref-2 == b_next_field) << 1;
                deltaMvBw = b_next_field ? -2 : 0;
            }

            mv_pred_direct(p,&p->mv[mv_scan[block]], &temp_mv,
                                    iDistanceIndexRef, iDistanceIndexFw, iDistanceIndexBw, deltaRef, deltaMvFw, deltaMvBw);
            if( p->b_error_flag )
                return;
        }
         
    }
    else
    {
		if( p->p_save[1] == NULL 
			|| p->p_save[2] == NULL )
		{
			p->b_error_flag = 1;
			return;
		}

        temp_mv = p_mv[0];
        if(p->ph.b_picture_structure) /* frame */
        {
            /* direct prediction from co-located P MB, block-wise */
            if(p->p_save[1]->b_picture_structure == 0) /* ref-field */
            {
                temp_mv.y *= 2;
                if( abs(temp_mv.ref) /2 > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                iDistanceIndexRef = p->p_save[1]->i_ref_distance[temp_mv.ref/2];
            }
            else
            {
                if( abs(temp_mv.ref) > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                iDistanceIndexRef = p->p_save[1]->i_ref_distance[temp_mv.ref];
            }
            iDistanceIndexFw = p->p_save[2]->i_distance_index;
            iDistanceIndexBw = p->p_save[1]->i_distance_index;
            p->mv[mv_scan[block]].ref = 1;
            p->mv[mv_scan[block]+MV_BWD_OFFS].ref = 0;

            mv_pred_direct(p,&p->mv[mv_scan[block]], &temp_mv,
                                    iDistanceIndexRef, iDistanceIndexFw, iDistanceIndexBw, 0, 0, 0);
            if( p->b_error_flag )
                return;
        }	
        else /* field */ 
        {
            if(p->p_save[1]->b_picture_structure == 1) /* ref-frame */
            {
                temp_mv.y /= 2;
                if( abs(temp_mv.ref) > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
                iDistanceIndexRef = p->p_save[1]->i_ref_distance[temp_mv.ref] + 1;	
            }
            else
            {
                int i_ref = temp_mv.ref;

                if( (abs(temp_mv.ref)>>1) > 1 )
                {
                    p->b_error_flag = 1;
                    return;
                }
        
                if (NEW_DIRECT)
                {
                    iDistanceIndexRef = p->p_save[1]->i_ref_distance[i_ref>>1] + !(i_ref & 1); /* NOTE : refer to reverse field */
                }
                else if(b_next_field && i_ref == 0)
                {
                    iDistanceIndexRef = p->p_save[1]->i_distance_index;
                }
                else
                {
                    i_ref -= b_next_field;
                    iDistanceIndexRef = p->p_save[1]->i_ref_distance[i_ref>>1] - (i_ref&1) + 1;
                }
            }

            /* set as field reference list */
            iDistanceIndexFw0 = p->p_save[2]->i_distance_index+1;
            iDistanceIndexFw1 = p->p_save[2]->i_distance_index;
            iDistanceIndexBw0 = p->p_save[1]->i_distance_index;
            iDistanceIndexBw1 = p->p_save[1]->i_distance_index+1;

            if(iDistanceIndexRef == iDistanceIndexFw0 || (NEW_DIRECT & b_next_field))
            {
                iDistanceIndexFw = iDistanceIndexFw0;
                p->mv[mv_scan[block]].ref = 2;
            }
            else
            {
                iDistanceIndexFw = iDistanceIndexFw1;
                p->mv[mv_scan[block]].ref = 3;
            }

            if(b_next_field==0 || NEW_DIRECT)
            {
                iDistanceIndexBw = iDistanceIndexBw0;
                p->mv[mv_scan[block]+MV_BWD_OFFS].ref = 0;
            }
            else
            {
                iDistanceIndexBw = iDistanceIndexBw1;
                p->mv[mv_scan[block]+MV_BWD_OFFS].ref = 1;
            }

            if (NEW_DIRECT)
            {
                deltaRef = ((temp_mv.ref & 1) ^ 1) << 1;
                deltaMvFw = (p->mv[mv_scan[block]].ref-2 == b_next_field) << 1;
                deltaMvBw = b_next_field ? -2 : 0;
            }

            mv_pred_direct(p,&p->mv[mv_scan[block]], &temp_mv,
                                    iDistanceIndexRef, iDistanceIndexFw, iDistanceIndexBw, deltaRef, deltaMvFw, deltaMvBw);
            if( p->b_error_flag )
                return;
        }
    }
}

static void get_b_direct_skip_mb(cavs_decoder *p)
{
    cavs_vector mv[24];
    int block;
    //uint32_t b_next_field = p->i_mb_index >= p->i_mb_num_half;
    int i_ref_offset = p->ph.b_picture_structure == 0 ? 2 : 1;
    uint8_t i_col_type;
    cavs_vector *p_mv;

    mv[MV_FWD_A1] = p->mv[MV_FWD_A1];
    mv[MV_FWD_B2] = p->mv[MV_FWD_B2];
    mv[MV_FWD_C2] = p->mv[MV_FWD_C2];
    mv[MV_FWD_D3] = p->mv[MV_FWD_D3];
    mv[MV_BWD_A1] = p->mv[MV_BWD_A1];
    mv[MV_BWD_B2] = p->mv[MV_BWD_B2];
    mv[MV_BWD_C2] = p->mv[MV_BWD_C2];
    mv[MV_BWD_D3] = p->mv[MV_BWD_D3];
    for( block = 0; block < 4; block++ )
    {
        get_col_info(p, &i_col_type, &p_mv, block);
        if( !i_col_type ) /* col_type is I_8x8 */ 
        {
            mv_pred_sub_direct(p, mv, 0, mv_scan[block], mv_scan[block]-3,
                                                MV_PRED_BSKIP, BLK_8X8, i_ref_offset);
			if(p->b_error_flag)
				return;
            mv_pred_sub_direct(p, mv, MV_BWD_OFFS, mv_scan[block]+MV_BWD_OFFS,
                                        	mv_scan[block]-3+MV_BWD_OFFS,
                                        	MV_PRED_BSKIP, BLK_8X8, 0);
        }
        else
        {
            get_b_direct_skip_sub_mb(p,block,p_mv);
        }
		
		if(p->b_error_flag)
        {
            return;
        }
    }
}

static int get_mb_b(cavs_decoder *p)
{   
    int kk = 0; 
    cavs_vector mv[24];
    int block;
    enum cavs_mb_sub_type sub_type[4];
    int flags;
	int ref[4] = { 0 };
    int i_ref_offset = p->ph.b_picture_structure == 0 ? 2 : 1;
    uint8_t i_col_type;
    cavs_vector *p_mv;
    int i_mb_type = p->i_mb_type;
    int16_t (*p_mvd)[6][2] = p->p_mvd;
    int8_t	(*p_ref)[9] = p->p_ref;
	int i_ret = 0;
    
    init_mb(p);

    p->mv[MV_FWD_X0] = MV_REF_DIR;
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);
    p->mv[MV_BWD_X0] = MV_REF_DIR;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);

#define FWD		0
#define BWD		1
#define MVD_X0	1
#define MVD_X1	2
#define MVD_X2	4
#define MVD_X3	5
#define REF_X0  4
#define REF_X1  5
#define REF_X2  7
#define REF_X3  8

    /* The MVD of pos X[0-3] have been initialized as 0
        The REF of pos X[0-3] have been initialized as -1 */
    switch(i_mb_type) 
    {
        case B_SKIP:
        case B_DIRECT:
            get_b_direct_skip_mb(p);
            if(p->b_error_flag)
            {
                return -1;
            }
            break;
            
        case B_FWD_16X16:
            ref[0] = p->bs_read_ref_b(p, FWD, REF_X0);
            if( ref[0] > 1 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0]+i_ref_offset, MVD_X0);
            if( p->b_error_flag )
            {                
                return -1;
            }
            
            M32(p_mvd[FWD][MVD_X1]) = M32(p_mvd[FWD][MVD_X3]) = M32(p_mvd[FWD][MVD_X0]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X2] = p_ref[FWD][REF_X3] = p_ref[FWD][REF_X0];
            break;
            
        case B_SYM_16X16:
            ref[0] = p->bs_read_ref_b(p, FWD, REF_X0);
            if( ref[0] > 1 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0]+i_ref_offset, MVD_X0);
            if( p->b_error_flag )
            {                
                return -1;
            }
            mv_pred_sym(p, &p->mv[MV_FWD_X0], BLK_16X16,ref[0]);
            if(p->b_error_flag)
            {
                return -1;
            }
            
            M32(p_mvd[FWD][MVD_X1]) = M32(p_mvd[FWD][MVD_X3]) = M32(p_mvd[FWD][MVD_X0]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X2] = p_ref[FWD][REF_X3] = p_ref[FWD][REF_X0];
            break;
            
        case B_BWD_16X16:
            ref[0] = p->bs_read_ref_b(p, BWD, REF_X0);
            if( ref[0] > 1 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_BWD_X0, MV_BWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            M32(p_mvd[BWD][MVD_X1]) = M32(p_mvd[BWD][MVD_X3]) = M32(p_mvd[BWD][MVD_X0]);
            p_ref[BWD][REF_X1] = p_ref[BWD][REF_X2] = p_ref[BWD][REF_X3] = p_ref[BWD][REF_X0];
            break;
            
        case B_8X8:
            mv[MV_FWD_A1] = p->mv[MV_FWD_A1];
            mv[MV_FWD_B2] = p->mv[MV_FWD_B2];
            mv[MV_FWD_C2] = p->mv[MV_FWD_C2];
            mv[MV_FWD_D3] = p->mv[MV_FWD_D3];
            mv[MV_BWD_A1] = p->mv[MV_BWD_A1];
            mv[MV_BWD_B2] = p->mv[MV_BWD_B2];
            mv[MV_BWD_C2] = p->mv[MV_BWD_C2];
            mv[MV_BWD_D3] = p->mv[MV_BWD_D3];

            for(block = 0; block < 4; block++)
            {    
                sub_type[block] = (enum cavs_mb_sub_type)p->bs_read[SYNTAX_MB_PART_TYPE](p);
                if(sub_type[block] < 0 || sub_type[block] > 3 || p->b_error_flag )
                {
                    p->b_error_flag = 1;
                
                    return -1;
                }
            }
            
            for( block = 0; block < 4; block++)
            {
                if(sub_type[block] == B_SUB_DIRECT || sub_type[block] == B_SUB_BWD)
                    ref[block] = 0;
                else
                    ref[block] = p->bs_read_ref_b(p, FWD, ref_scan[block]);
                if( ref[block] > 1 || p->b_error_flag )
                {
                    p->b_error_flag = 1;
                
                    return -1;
                }
            }
            
            for(block = 0; block < 4; block++)
            {
                if(sub_type[block] == B_SUB_BWD)
                {
                    ref[block] = p->bs_read_ref_b(p, BWD, ref_scan[block]);
                }
                if( ref[block] > 1 || p->b_error_flag )
                {
                    p->b_error_flag = 1;
                
                    return -1;
                }
            }
            
            for( block = 0; block < 4; block++ ) 
            {
                switch(sub_type[block])
                {
                    case B_SUB_DIRECT:
                        get_col_info(p, &i_col_type, &p_mv, block);
                        if(!i_col_type)
                        {
                            mv_pred_sub_direct(p, mv, 0, mv_scan[block], mv_scan[block]-3,
                                    MV_PRED_BSKIP, BLK_8X8, i_ref_offset);
							if(p->b_error_flag)
								return -1;
                            mv_pred_sub_direct(p, mv, MV_BWD_OFFS, mv_scan[block]+MV_BWD_OFFS,
                                    mv_scan[block]-3+MV_BWD_OFFS,
                                    MV_PRED_BSKIP, BLK_8X8, 0);
                        } 
                        else
                        {
                            get_b_direct_skip_sub_mb(p, block, p_mv);
                        }
						
                        if(p->b_error_flag)
                        {
                            return -1;
                        }
                        break;
                        
                    case B_SUB_FWD:
                        mv_pred(p, mv_scan[block], mv_scan[block]-3,
                                    MV_PRED_MEDIAN, BLK_8X8, ref[block]+i_ref_offset, mvd_scan[block]);
                        if( p->b_error_flag )
                        {
                            return -1;
                        }
                        break;
                        
                    case B_SUB_SYM:
                        mv_pred(p, mv_scan[block], mv_scan[block]-3,
                                        MV_PRED_MEDIAN, BLK_8X8, ref[block]+i_ref_offset, mvd_scan[block]);
                        if( p->b_error_flag )
                        {
                            return -1;
                        }
                        mv_pred_sym(p, &p->mv[mv_scan[block]], BLK_8X8,ref[block]);
                        if(p->b_error_flag)
                        {
                            return -1;
                        }
                             
                        break;
					default:
                    	break;
                }
            }
            for( block = 0; block < 4; block++ )
            {
                if(sub_type[block] == B_SUB_BWD)
                {
                    mv_pred(p, mv_scan[block]+MV_BWD_OFFS,
                            mv_scan[block]+MV_BWD_OFFS-3,
                            MV_PRED_MEDIAN, BLK_8X8, ref[block], mvd_scan[block]);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                }
            }
            break;
        default:
            flags = partition_flags[i_mb_type];
            if( i_mb_type <= B_SYM_16X16 )
            {
                p->b_error_flag = 1;

                return -1;
            }
            if(i_mb_type & 1) /* 16x8 macroblock types */
            { 
                int i = 0, k = 0;
                
                if (flags & FWD0)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X0);
                if (flags & FWD1)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X2);
                if (flags & BWD0)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X0);
                if (flags & BWD1)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X2);

                k = i;
                for( i = 0 ; i < k; i++ )
                {
                    if( ref[i] > 1 || p->b_error_flag )
                    {
                        p->b_error_flag = 1;
                
                        return -1;
                    }    
                }
                
                if(flags & FWD0)
                {
                    mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_TOP,  BLK_16X8, ref[kk++]+i_ref_offset, MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                    CP32(p_mvd[FWD][MVD_X1], p_mvd[FWD][MVD_X0]);
                    p_ref[FWD][REF_X1] = p_ref[FWD][REF_X0];
                }
                
                if(flags & SYM0)
                {    
                    mv_pred_sym(p, &p->mv[MV_FWD_X0], BLK_16X8,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & FWD1)
                {
                    mv_pred(p, MV_FWD_X2, MV_FWD_A1, MV_PRED_LEFT, BLK_16X8, ref[kk++]+i_ref_offset, MVD_X2);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                    CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X2]);
                    p_ref[FWD][REF_X3] = p_ref[FWD][REF_X2];
                }
                if(flags & SYM1)
                {    
                    mv_pred_sym(p, &p->mv[MV_FWD_X2], BLK_16X8,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & BWD0)
                {
                    mv_pred(p, MV_BWD_X0, MV_BWD_C2, MV_PRED_TOP,  BLK_16X8, ref[kk++], MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                    CP32(p_mvd[BWD][MVD_X1], p_mvd[BWD][MVD_X0]);
                    p_ref[BWD][REF_X1] = p_ref[BWD][REF_X0];
                }
                if(flags & BWD1)
                {
                    mv_pred(p, MV_BWD_X2, MV_BWD_A1, MV_PRED_LEFT, BLK_16X8, ref[kk++], MVD_X2);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                    CP32(p_mvd[BWD][MVD_X3], p_mvd[BWD][MVD_X2]);
                    p_ref[BWD][REF_X3] = p_ref[BWD][REF_X2];
                }
            } 
            else  /* 8x16 macroblock types */
            {
                int i = 0, k = 0;
                
                if (flags & FWD0)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X0);
                if (flags & FWD1)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X1);
                if (flags & BWD0)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X0);
                if (flags & BWD1)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X1);

                k = i;
                for( i = 0 ; i < k; i++ )
                {
                    if( ref[i] > 1 || p->b_error_flag )
                    {
                        p->b_error_flag = 1;
                
                        return -1;
                    }    
                }
                           
                if(flags & FWD0)
                {
                    mv_pred(p, MV_FWD_X0, MV_FWD_B3, MV_PRED_LEFT, BLK_8X16, ref[kk++]+i_ref_offset, MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	p_ref[FWD][REF_X2] = p_ref[FWD][REF_X0];
                }
                if(flags & SYM0)
                {    
                    mv_pred_sym(p, &p->mv[MV_FWD_X0], BLK_8X16,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & FWD1)
                {
                    mv_pred(p, MV_FWD_X1, MV_FWD_C2, MV_PRED_TOPRIGHT,BLK_8X16, ref[kk++]+i_ref_offset, MVD_X1);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X1]);
                	p_ref[FWD][REF_X3] = p_ref[FWD][REF_X1];
                }
                if(flags & SYM1)
                {
					mv_pred_sym(p, &p->mv[MV_FWD_X1], BLK_8X16,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & BWD0)
                {
                    mv_pred(p, MV_BWD_X0, MV_BWD_B3, MV_PRED_LEFT, BLK_8X16, ref[kk++], MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	 p_ref[BWD][REF_X2] = p_ref[BWD][REF_X0];
                }
                if(flags & BWD1)
                {
                	mv_pred(p, MV_BWD_X1, MV_BWD_C2, MV_PRED_TOPRIGHT,BLK_8X16, ref[kk++], MVD_X1);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }                    
                	CP32(p_mvd[BWD][MVD_X3], p_mvd[BWD][MVD_X1]);
                	p_ref[BWD][REF_X3] = p_ref[BWD][REF_X1];
                }
            }
    }
    
#undef FWD
#undef BWD
#undef MVD_X3
#undef MVD_X2
#undef MVD_X1
#undef MVD_X0
#undef REF_X3
#undef REF_X2
#undef REF_X1
#undef REF_X0

#if B_MB_WEIGHTING
    p->weighting_prediction = 0;
#endif
    if (i_mb_type != B_SKIP)
    {
#if B_MB_WEIGHTING
        if ( p->sh.b_slice_weighting_flag && p->sh.b_mb_weighting_flag )
        {
           	if( p->ph.b_aec_enable )
			{
				p->weighting_prediction = cavs_cabac_get_mb_weighting_prediction( p );
			}
			else
				p->weighting_prediction = cavs_bitstream_get_bit1(&p->s);	//weighting_prediction
        }
#else 
        if (p->sh.b_mb_weighting_flag)
        {
            cavs_bitstream_get_bit1(&p->s);	//weighting_prediction
        }
#endif
    }

    inter_pred(p, i_mb_type);

    if(i_mb_type != B_SKIP)
    {
        /* get coded block pattern */
        p->i_cbp = p->bs_read[SYNTAX_INTER_CBP](p);
        if( p->i_cbp > 63 || p->b_error_flag ) 
        {
            p->b_error_flag = 1;

            return -1;
        }

        /* get quantizer */
        if(p->i_cbp && !p->b_fixed_qp)
        {    
            int delta_qp =  p->bs_read[SYNTAX_DQP](p);
            
			p->i_qp = (p->i_qp-MIN_QP + delta_qp + (MAX_QP-MIN_QP+1))%(MAX_QP-MIN_QP+1)+MIN_QP;

            if( p->i_qp < 0 || p->i_qp > 63  || p->b_error_flag )
            {
				p->b_error_flag = 1;
                return -1;  
            }
        }
        else
            p->i_last_dqp = 0;

        i_ret = get_residual_inter(p,i_mb_type);    	
		if(i_ret == -1)
			return -1;
    }
    filter_mb(p, i_mb_type);
    
    return next_mb(p);
}

static inline void get_p_frame_ref(cavs_decoder *p, int *p_ref)
{
	if(p->ph.b_picture_reference_flag )
	{
		*p_ref=0;
	}
	else
	{
		if(p->ph.b_picture_structure==0&&p->ph.i_picture_coding_type==CAVS_P_PICTURE)
		{
			*p_ref=cavs_bitstream_get_bits(&p->s,2);
		}
		else
		{
			*p_ref=cavs_bitstream_get_bit1(&p->s);	//mb_reference_index
		}
	}
}

static int get_mb_p(cavs_decoder *p)
{
    int i;
    int i_offset;
    int ref[4];
    int i_mb_type = p->i_mb_type;
    int16_t (*p_mvd)[6][2] = p->p_mvd;
    int8_t (*p_ref)[9] = p->p_ref;
	int i_ret = 0;

    init_mb(p);

#define FWD		0
//#define BWD		1	//no need for backward mvd in P frame
#define MVD_X0  1
#define MVD_X1	2
#define MVD_X2  4
#define MVD_X3	5
#define REF_X0  4
#define REF_X1	5
#define REF_X2	7
#define REF_X3  8

    switch(i_mb_type)
    {
        case P_SKIP:
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_PSKIP, BLK_16X16, PSKIP_REF, MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            break;
        case P_16X16:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if( ref[0] > 3 ||  p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN,   BLK_16X16, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            M32(p_mvd[FWD][MVD_X1]) = M32(p_mvd[FWD][MVD_X3]) = M32(p_mvd[FWD][MVD_X0]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X2] = p_ref[FWD][REF_X3] = p_ref[FWD][REF_X0];
            break;
        case P_16X8:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if( ref[0] > 3 || p->b_error_flag  )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            ref[2] = p->bs_read_ref_p(p, REF_X2);
            if( ref[2] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_TOP,      BLK_16X8, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            mv_pred(p, MV_FWD_X2, MV_FWD_A1, MV_PRED_LEFT,     BLK_16X8, ref[2], MVD_X2);
            if( p->b_error_flag )
            {
                return -1;
            }
            CP32(p_mvd[FWD][MVD_X1], p_mvd[FWD][MVD_X0]);
            CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X2]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X0];
            p_ref[FWD][REF_X3] = p_ref[FWD][REF_X2];
            break;
        case P_8X16:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if( ref[0] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            ref[1] = p->bs_read_ref_p(p, REF_X1);
            if( ref[1] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_B3, MV_PRED_LEFT,     BLK_8X16, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            mv_pred(p, MV_FWD_X1, MV_FWD_C2, MV_PRED_TOPRIGHT, BLK_8X16, ref[1], MVD_X1);
            if( p->b_error_flag )
            {
                return -1;
            }
            CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X1]);
            p_ref[FWD][REF_X2] = p_ref[FWD][REF_X0];
            p_ref[FWD][REF_X3] = p_ref[FWD][REF_X1];
            break;
        case P_8X8:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if( ref[0] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            ref[1] = p->bs_read_ref_p(p, REF_X1);
            if( ref[1] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            ref[2] = p->bs_read_ref_p(p, REF_X2);
            if( ref[2] > 3  || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            ref[3] = p->bs_read_ref_p(p, REF_X3);
            if( ref[3] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_B3, MV_PRED_MEDIAN,   BLK_8X8, ref[0], MVD_X0);
            if( p->b_error_flag )
            {                
                return -1;
            }
            mv_pred(p, MV_FWD_X1, MV_FWD_C2, MV_PRED_MEDIAN,   BLK_8X8, ref[1], MVD_X1);
            if( p->b_error_flag )
            {                
                return -1;
            }
            mv_pred(p, MV_FWD_X2, MV_FWD_X1, MV_PRED_MEDIAN,   BLK_8X8, ref[2], MVD_X2);
            if( p->b_error_flag )
            {
                return -1;
            }
            mv_pred(p, MV_FWD_X3, MV_FWD_X0, MV_PRED_MEDIAN,   BLK_8X8, ref[3], MVD_X3);
            if( p->b_error_flag )
            {
                return -1;
            }
            break;
    }
    
#undef FWD
//#undef BWD
#undef MVD_X3
#undef MVD_X2
#undef MVD_X1
#undef MVD_X0
#undef REF_X3
#undef REF_X2
#undef REF_X1
#undef REF_X0

#if B_MB_WEIGHTING
	p->weighting_prediction = 0;
#endif
    if (i_mb_type != P_SKIP)
    {
#if B_MB_WEIGHTING
		if ( p->sh.b_slice_weighting_flag && p->sh.b_mb_weighting_flag )
		{
			if( p->ph.b_aec_enable )
			{
				p->weighting_prediction = cavs_cabac_get_mb_weighting_prediction( p );
			}
			else
				p->weighting_prediction = cavs_bitstream_get_bit1(&p->s);	//weighting_prediction
		}
#else 
        if (p->sh.b_mb_weighting_flag)
        {
            cavs_bitstream_get_bit1(&p->s);	//weighting_prediction
        }
#endif
    }

    inter_pred(p, i_mb_type);

    i_offset = (p->i_mb_y*p->i_mb_width + p->i_mb_x)*4;
    p->p_col_mv[i_offset+ 0] = p->mv[MV_FWD_X0];
    p->p_col_mv[i_offset + 1] = p->mv[MV_FWD_X1];
    p->p_col_mv[i_offset + 2] = p->mv[MV_FWD_X2];
    p->p_col_mv[i_offset + 3] = p->mv[MV_FWD_X3];

    if (i_mb_type != P_SKIP)
    {
        /* get coded block pattern */
        p->i_cbp = p->bs_read[SYNTAX_INTER_CBP](p);
        if( p->i_cbp > 63 || p->b_error_flag ) 
        {
            p->b_error_flag = 1;

            return -1;
        }
    
        if(p->ph.vbs_enable)
        {
            if(i_mb_type == P_8X8)
            {
                for(i=0;i<4;i++)
                {
                    if(p->i_cbp & (1<<i))
                        p->vbs_mb_part_transform_4x4_flag[i] = cavs_bitstream_get_bit1(&p->s);
                }
                for(i=0;i<4;i++)
                {
                    if(p->vbs_mb_part_transform_4x4_flag[i] && (p->i_cbp&(1<<i)))
                    {
                        p->i_cbp_4x4[i] = cavs_bitstream_get_ue_k(&p->s,1);
                        p->i_cbp_part[i] = get_cbp_4x4(p->i_cbp_4x4[i]);
                    }
                }
            }
        }

        /* get quantizer */
        if(p->i_cbp && !p->b_fixed_qp)
        {
            int delta_qp =  p->bs_read[SYNTAX_DQP](p);
            
			p->i_qp = (p->i_qp-MIN_QP + delta_qp + (MAX_QP-MIN_QP+1))%(MAX_QP-MIN_QP+1)+MIN_QP;

            if( p->i_qp < 0 || p->i_qp > 63  || p->b_error_flag )
            {
				p->b_error_flag = 1;
                return -1;  
            }
        }
        else
            p->i_last_dqp = 0;

        i_ret = get_residual_inter(p,i_mb_type);
        if(i_ret == -1)
            return -1;
    }

    filter_mb(p,i_mb_type);

    *p->p_col_type = i_mb_type;
    
    return next_mb(p);
}

static int cavs_mb_predict_intra8x8_mode (cavs_decoder *p, int i_pos)
{
    const int ma = p->i_intra_pred_mode_y[i_pos-1];
    const int mb = p->i_intra_pred_mode_y[i_pos-CAVS_ICACHE_STRIDE];
    const int m = CAVS_MIN (cavs_mb_pred_mode (ma),
    	cavs_mb_pred_mode (mb));
    
    if (m < 0)
        return INTRA_L_LP;
    else if (ma < NO_INTRA_8x8_MODE && mb < NO_INTRA_8x8_MODE)
        return  m;
    else if (ma < NO_INTRA_8x8_MODE)
        return cavs_mb_pred_mode(ma);
    else if (mb < NO_INTRA_8x8_MODE)
        return cavs_mb_pred_mode(mb);
    else
        return cavs_pred_mode_4x4to8x8[pred4x4[cavs_mb_pred_mode(ma)+1][cavs_mb_pred_mode(mb)+1]];
}

static int cavs_mb_predict_intra4x4_mode (cavs_decoder *p, int i_pos)
{
    const int ma = p->i_intra_pred_mode_y[i_pos-1];
    const int mb = p->i_intra_pred_mode_y[i_pos-CAVS_ICACHE_STRIDE];
    
    return pred4x4[cavs_mb_pred_mode(ma)+1][cavs_mb_pred_mode(mb)+1];
}

static int get_mb_i(cavs_decoder *p)
{
    uint8_t i_top[18];
    uint8_t *p_left = NULL;
    uint8_t *p_d;

    static const uint8_t scan5x5[4][4] = {
    	{6, 7, 11, 12},
    	{8, 9, 13, 14},
    	{16, 17, 21, 22},
    	{18, 19, 23, 24}
    };

    int i, j;
    int i_offset = p->i_mb_x<<2;
    //int i_mb_type = p->i_cbp_code;
    int i_rem_mode;
    int i_pred_mode;
    DECLARE_ALIGNED_16(uint8_t edge[33]);
    DECLARE_ALIGNED_16(uint8_t edge_8x8[33]);
    int i_qp_cb = 0, i_qp_cr = 0;
	int i_ret = 0;

    init_mb(p);

    if (p->ph.vbs_enable)
    {
        p->vbs_mb_intra_pred_flag=cavs_bitstream_get_bit1(&p->s);
        if(p->vbs_mb_intra_pred_flag)	//vbs_mb_intra_pred_flag == '1'
        {
            for( j = 0; j < 4; ++j )
            {
                p->vbs_mb_part_intra_pred_flag[j]=cavs_bitstream_get_bit1(&p->s);
                p->vbs_mb_part_transform_4x4_flag[j]=p->vbs_mb_part_intra_pred_flag[j];
            }
        }
    }
    	
    for( i = 0; i < 4; i++ ) 
    {
        if (p->vbs_mb_part_intra_pred_flag[i])
        {
            int m;

            for( m = 0; m < 4; m++ )
            {
                int i_pos = scan5x5[i][m];
                
                p->pred_mode_4x4_flag[m] = cavs_bitstream_get_bit1(&p->s);
                if(!p->pred_mode_4x4_flag[m])
                    p->intra_luma_pred_mode_4x4[m] = cavs_bitstream_get_bits(&p->s, 3);

                i_pred_mode = cavs_mb_predict_intra4x4_mode(p, i_pos);
                if(!p->pred_mode_4x4_flag[m])
                {
                    i_pred_mode = p->intra_luma_pred_mode_4x4[m] + (p->intra_luma_pred_mode_4x4[m] >= i_pred_mode);
                }
                p->i_intra_pred_mode_y[i_pos] = i_pred_mode + NO_INTRA_8x8_MODE;
            }
        }
        else
        {
            int i_pos = scan5x5[i][0];

            i_rem_mode = p->bs_read[SYNTAX_INTRA_LUMA_PRED_MODE](p);
            if( p->b_error_flag )
            {
                return -1;    
            }

            i_pred_mode = cavs_mb_predict_intra8x8_mode(p, i_pos);
            if(!p->pred_mode_flag)
            {
                i_pred_mode = i_rem_mode + (i_rem_mode >= i_pred_mode);
            }
            if( i_pred_mode > 4) 
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            p->i_intra_pred_mode_y[scan5x5[i][0]] =
            p->i_intra_pred_mode_y[scan5x5[i][1]] =
            p->i_intra_pred_mode_y[scan5x5[i][2]] =
            p->i_intra_pred_mode_y[scan5x5[i][3]] = i_pred_mode;
        }
    }

    p->i_pred_mode_chroma = p->bs_read[SYNTAX_INTRA_CHROMA_PRED_MODE](p);
    if(p->i_pred_mode_chroma > 6 || p->b_error_flag ) 
    {
        p->b_error_flag = 1;
                
        return -1;
    }

    p->i_intra_pred_mode_y[5] =  p->i_intra_pred_mode_y[9];
    p->i_intra_pred_mode_y[10] =  p->i_intra_pred_mode_y[14];
    p->i_intra_pred_mode_y[15] =  p->i_intra_pred_mode_y[19];
    p->i_intra_pred_mode_y[20] =  p->i_intra_pred_mode_y[24];

    p->p_top_intra_pred_mode_y[i_offset+0] = p->i_intra_pred_mode_y[21];
    p->p_top_intra_pred_mode_y[i_offset+1] = p->i_intra_pred_mode_y[22];
    p->p_top_intra_pred_mode_y[i_offset+2] = p->i_intra_pred_mode_y[23];
    p->p_top_intra_pred_mode_y[i_offset+3] = p->i_intra_pred_mode_y[24];

    if(!(p->i_mb_flags & A_AVAIL))
    {
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[6] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[11] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[16] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[21] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_C, &p->i_pred_mode_chroma );
    }
    if(!(p->i_mb_flags & B_AVAIL))
    {
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[6] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[7] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[8] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[9] );
        adapt_pred_mode(TOP_ADAPT_INDEX_C, &p->i_pred_mode_chroma );
    }

    p->i_cbp = p->bs_read[SYNTAX_INTRA_CBP](p);
    if( p->i_cbp > 63 ||  p->b_error_flag ) 
    {
        p->b_error_flag = 1;

        return -1;
    }

    if(p->ph.vbs_enable)
    {
        for( i = 0; i < 4; i++ )
        {
            if(p->vbs_mb_part_transform_4x4_flag[i]&&(p->i_cbp&(1<<i)))
            {
                p->i_cbp_4x4[i]=cavs_bitstream_get_ue_k(&p->s,1);
                p->i_cbp_part[i] = get_cbp_4x4(p->i_cbp_4x4[i]);
            }
        }
    }

    if(p->i_cbp && !p->b_fixed_qp)
    {	
        int delta_qp =  p->bs_read[SYNTAX_DQP](p);

		p->i_qp = (p->i_qp-MIN_QP + delta_qp + (MAX_QP-MIN_QP+1))%(MAX_QP-MIN_QP+1)+MIN_QP;

        if( p->i_qp < 0 || p->i_qp > 63  || p->b_error_flag )
        {
			p->b_error_flag = 1;
            return -1;  
        }
    }
    else
        p->i_last_dqp = 0; // for cabac only

    for( i = 0; i < 4; i++ )
    {
        int i_mode;

        p_d = p->p_y + p->i_luma_offset[i];

        load_intra_pred_luma(p, i_top, &p_left, i);

        if (p->vbs_mb_part_intra_pred_flag[i])
        {
            for (j = 0; j < 4; j++)
            {
                uint8_t *p_d_4x4 = p_d + 4*(j&1) + 4*(j>>1)*p->cur.i_stride[0];
                
                i_mode = p->i_intra_pred_mode_y[scan5x5[i][j]];
                load_intra_4x4_pred_luma(p, p_d_4x4, edge, i*4+j);
                intra_pred_lum_4x4[i_mode](p_d_4x4, edge, p->cur.i_stride[0]);
                if ((p->i_cbp & (1<<i)) && (p->i_cbp_part[i] & (1<<j)))
                {
                    get_residual_block4x4(p,p->i_qp,p_d_4x4,p->cur.i_stride[0], 1);
                }
            }
        }
        else
        {
            i_mode = p->i_intra_pred_mode_y[scan5x5[i][0]];
            p->cavs_intra_luma[i_mode](p_d, edge_8x8, p->cur.i_stride[0], i_top, p_left);

            if(p->i_cbp & (1<<i))
            {
                i_ret = get_residual_block(p,intra_2dvlc,1,p->i_qp,p_d,p->cur.i_stride[0], 0);
                if(i_ret == -1)
                    return -1;
            }
        }
    }

    load_intra_pred_chroma(p);

    p->cavs_intra_chroma[p->i_pred_mode_chroma](p->p_cb, p->mb.i_neighbour, p->cur.i_stride[1], &p->p_top_border_cb[p->i_mb_x*10], p->i_left_border_cb);
    p->cavs_intra_chroma[p->i_pred_mode_chroma](p->p_cr, p->mb.i_neighbour, p->cur.i_stride[2], &p->p_top_border_cr[p->i_mb_x*10], p->i_left_border_cr);
    
    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
    { 
        i_qp_cb = clip3_int( p->i_qp + p->ph.chroma_quant_param_delta_u, 0, 63 );
        i_qp_cr = clip3_int( p->i_qp + p->ph.chroma_quant_param_delta_v, 0, 63 );
    }

    if(p->i_cbp & (1<<4))
    {
        i_ret = get_residual_block(p,chroma_2dvlc,0, 
             chroma_qp[ p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable? i_qp_cb : p->i_qp],
    		p->p_cb,p->cur.i_stride[1], 1);
        if(i_ret == -1)
        	return -1;
    }
    if(p->i_cbp & (1<<5))
    {
        i_ret = get_residual_block(p,chroma_2dvlc,0, 
             chroma_qp[ p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable? i_qp_cr : p->i_qp],
    		p->p_cr,p->cur.i_stride[2], 1);
        if(i_ret == -1)
            return -1;
    }

    filter_mb(p, I_8X8);
    p->mv[MV_FWD_X0] = MV_INTRA;
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);
    p->mv[MV_BWD_X0] = MV_INTRA;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);

    if(p->ph.i_picture_coding_type != CAVS_B_PICTURE)
    	*p->p_col_type = I_8X8;

    return next_mb(p);
}

/* decode slice only for aec */
static int cavs_get_slice_aec( cavs_decoder *p )
{
    int i, i_num = 0, i_result;
    uint32_t i_limit, i_skip;
    int i_first_field, i_skip_count = 0;
    int (*get_mb_type)(cavs_decoder *p);

    int (*get_mb_pb_aec_opt)(cavs_decoder *p);
    //int (*get_mb_pb_rec_opt)(cavs_decoder *p);
    int i_slice_mb_y;
    int i_slice_mb_index;

    
    /* picture_type, picture_structure, i_first_field */
    static int8_t i_num_of_reference_table[3][2][2]={
        {{1,0}/*picture_structure=0*/,{0,0}/*picture_structure=1*/},//i_picuture
        {{4,4}/*picture_structure=0*/,{2,2}/*picture_structure=1*/},//p_picuture
        {{4,4}/*picture_structure=0*/,{2,2}/*picture_structure=1*/}//b_picuture
    };
    
    int b_interlaced = p->param.b_interlaced;    

    if(p->vsh.i_vertical_size > 2800)
    {
        p->sh.i_slice_vertical_position_extension = cavs_bitstream_get_bits(&p->s,3);
    }
    else
    {
        p->sh.i_slice_vertical_position_extension = 0;
    }
    //if (p->vsh.i_profile_id == PROFILE_YIDONG)
    //{
    //    p->sh.i_slice_horizontal_position = cavs_bitstream_get_bits(&p->s,8);
    //    if(p->vsh.i_horizontal_size > 4080)
    //       p->sh.i_slice_vertical_position_extension = cavs_bitstream_get_bits(&p->s,2);
    //}
    if(p->ph.b_fixed_picture_qp == 0)
    {
        p->b_fixed_qp = p->sh.b_fixed_slice_qp = cavs_bitstream_get_bit1(&p->s);
        p->i_qp = p->sh.i_slice_qp = cavs_bitstream_get_bits(&p->s,6);
    }
    else
    {
        p->sh.b_fixed_slice_qp = 1;
        p->sh.i_slice_qp = p->i_qp;
    }
    p->i_mb_y += (p->sh.i_slice_vertical_position_extension<<7);
    p->i_mb_index = p->i_mb_y*p->i_mb_width;

    if(b_interlaced)
    {
        if( !p->b_bottom ) /* top field can't exceed half num */
        {
        	if(p->i_mb_y >= (p->i_mb_height>>1) )
        	{
                 p->b_error_flag = 1;
                 return -1;
        	}

        	if( p->i_mb_index >= p->i_mb_num_half )
        	{
                 p->b_error_flag = 1;
                 return -1;
        	}
        }
        else
        {
        	if(p->i_mb_y < (p->i_mb_height>>1) )
        	{
                 p->b_error_flag = 1;
                 return -1;
        	}

        	if( p->i_mb_index < p->i_mb_num_half )
        	{
                 p->b_error_flag = 1;
                 return -1;
        	}
        }
    }
    
    p->i_mb_y_start = p->i_mb_y;
    p->i_mb_index_start = p->i_mb_index;
    p->i_mb_y_start_fld[p->b_bottom] = p->i_mb_y;
    p->i_mb_index_start_fld[p->b_bottom] = p->i_mb_index;

    i_slice_mb_y = p->i_mb_y;
    i_slice_mb_index = p->i_mb_index;

    i_first_field = p->i_mb_index < p->i_mb_num_half ? 1 : 0;
    i_num = i_num_of_reference_table[p->ph.i_picture_coding_type][p->ph.b_picture_structure][i_first_field];
    p->b_have_pred = (p->ph.i_picture_coding_type != CAVS_I_PICTURE) || (p->ph.b_picture_structure == 0 && i_first_field == 0);
    if( p->b_have_pred )
    {
        p->sh.b_slice_weighting_flag = cavs_bitstream_get_bit1(&p->s);
#if B_MB_WEIGHTING
		/* copy weighting prediction info */
    	p->b_slice_weighting_flag[p->b_bottom] = p->sh.b_slice_weighting_flag;
#endif			
        if(p->sh.b_slice_weighting_flag)
        {
            for(i = 0; i < i_num; i++)
            {
                p->sh.i_luma_scale[i] = cavs_bitstream_get_bits(&p->s, 8);
                p->sh.i_luma_shift[i] = cavs_bitstream_get_int(&p->s, 8);
                cavs_bitstream_clear_bits(&p->s, 1);
                p->sh.i_chroma_scale[i] = cavs_bitstream_get_bits(&p->s, 8);
                p->sh.i_chroma_shift[i] = cavs_bitstream_get_int(&p->s, 8);
                cavs_bitstream_clear_bits(&p->s, 1);
            }
            p->sh.b_mb_weighting_flag = cavs_bitstream_get_bit1(&p->s);
#if B_MB_WEIGHTING
			/* copy weighting prediction info */
			for(i = 0; i < i_num; i++)
			{
				 p->i_luma_scale[p->b_bottom][i] = p->sh.i_luma_scale[i];
				 p->i_luma_shift[p->b_bottom][i] = p->sh.i_luma_shift[i];
				 p->i_chroma_scale[p->b_bottom][i] = p->sh.i_chroma_scale[i];
				 p->i_chroma_shift[p->b_bottom][i] = p->sh.i_chroma_shift[i];
			}
			p->b_mb_weighting_flag[p->b_bottom] = p->sh.b_mb_weighting_flag;
#endif				
        }
        else
        {
            p->sh.b_mb_weighting_flag = 0;
#if B_MB_WEIGHTING
    		/* copy weighting prediction info */
        	p->b_mb_weighting_flag[p->b_bottom] = 0;
#endif			
        }
    }
    else
    {
        p->sh.b_slice_weighting_flag = 0;
        p->sh.b_mb_weighting_flag = 0;
#if B_MB_WEIGHTING
		/* copy weighting prediction info */
		p->b_slice_weighting_flag[p->b_bottom] = 0;
		p->b_mb_weighting_flag[p->b_bottom] = 0;
#endif		
    }
    
    //if (p->vsh.i_profile_id == PROFILE_YIDONG && p->ph.i_picture_coding_type == CAVS_P_PICTURE)
    //{
    //    p->quant_coeff_pred_flag = cavs_bitstream_get_bit1(&p->s);
    //}
    p->sh.i_number_of_reference = i_num;
    
    if (p->ph.i_picture_coding_type < CAVS_B_PICTURE)   //p frame
    {
        i_skip = P_SKIP;
        i_limit = P_8X8;
        
        get_mb_pb_aec_opt = get_mb_p_aec_opt;

        get_mb_type = p->bs_read[SYNTAX_MBTYPE_P];
    }
    else    //b frame
    {
        i_skip = B_SKIP;
        i_limit = B_8X8;

        get_mb_pb_aec_opt = get_mb_b_aec_opt;

        get_mb_type = p->bs_read[SYNTAX_MBTYPE_B];
    }

    /* init slice */
    i_result = cavs_init_slice_for_aec(p);
	if(i_result == -1)
		goto end;

    /* MB loop */
    for(;;)
    {
        /* AEC loop */
        for( ; ; )
        {
             if(p->b_have_pred)
            {
                if(p->ph.b_skip_mode_flag)
                {
                    i_skip_count = p->bs_read[SYNTAX_SKIP_RUN](p);
                    if( i_skip_count == -1 ||
                        (p->i_mb_index + i_skip_count > p->i_slice_mb_end)
                        || i_skip_count < -1 )
                    {
                        i_result = -1;
                        goto end;
                    }
                    
                    if (p->ph.b_aec_enable && i_skip_count)
                    {
                        cavs_biari_decode_stuffing_bit(&p->cabac);
                        p->i_last_dqp = 0;
                        p->i_cbp = 0;
                    }
                    while(i_skip_count--)
                    {
                        p->i_mb_type_tab[p->i_mb_index] = p->i_mb_type = i_skip;

                        i_result = get_mb_pb_aec_opt(p);

                        if( p->b_error_flag )
                        {
                            i_result = -1;
                        }
                        
                        if(i_result != 0)
                        {
                            goto end;
                        }
                        
                        p->i_mb_index++;
                    }

                    if(p->i_mb_index >= p->i_slice_mb_end) /* slice finished */
                    {
                        if( !b_interlaced )
                        {
                            if (p->i_mb_index >= p->i_mb_num)   /* frame finished */
                                p->b_complete = 0;
                        }
                        else
                        {
                            if( !p->b_bottom )
                            {
                                if (p->i_mb_index >= (p->i_mb_num>>1))  /* top-field finished */
                                    p->b_complete = 0;
                            }
                            else
                            {
                                if (p->i_mb_index >= p->i_mb_num)   /* bot-field finished */
                                    p->b_complete = 0;
                            }
                        }

                        i_result = 0;
                        goto end;
                    }
                }

                p->i_mb_type_tab[p->i_mb_index]  = p->i_mb_type = get_mb_type(p);
                if( p->i_mb_type < 0 || p->i_mb_type > 29)
                {
                    p->b_error_flag = 1;
                    i_result = -1;
                    
                    goto end;
                }
                        
                if( p->i_mb_type == I_8X8 )
                {
                    i_result = get_mb_i_aec_opt(p);
                }
                else
                {
                    i_result = get_mb_pb_aec_opt(p);
                }

                if( p->b_error_flag )
                {
                    i_result = -1;
                }

                if (p->ph.b_aec_enable)
                {
                    cavs_biari_decode_stuffing_bit(&p->cabac);
                }
                if(i_result!=0)
                {
                    goto end;
                }
            } 
            else
            {
                p->i_mb_type_tab[p->i_mb_index] = I_8X8; /* init for I_8x8 decision */
                i_result = get_mb_i_aec_opt(p);

                if( p->b_error_flag )
                {
                    i_result = -1;
                }

                if (p->ph.b_aec_enable)
                {
                    cavs_biari_decode_stuffing_bit(&p->cabac);
                }

                if( i_result != 0 )
                {
                    goto end;
                }
            }
            
            p->i_mb_index++;
	      
            if( p->i_mb_index >= p->i_slice_mb_end )  /* slice finished */
            {
                if( !b_interlaced )
                {
                    if (p->i_mb_index >= p->i_mb_num)   /* frame finished */
                        p->b_complete = 0;
                }
                else
                {
                    if( !p->b_bottom )
                    {
                        if (p->i_mb_index >= (p->i_mb_num>>1))  /* top-field finished */
                            p->b_complete = 0;
                    }
                    else
                    {
                        if (p->i_mb_index >= p->i_mb_num)   /* bot-field finished */
                            p->b_complete = 0;
                    }
                }

                i_result = 0;

                goto end; /* */
            }
        }
    }

end:
    if (p->ph.b_aec_enable)
    {
        p->s = p->cabac.bs;
    }

    return i_result;
}

/* only rec */
static int cavs_get_slice_rec(cavs_decoder *p)
{
    int i_num = 0, i_result;
    uint32_t i_limit, i_skip;
    int i_first_field;// i_skip_count = 0;

    int (*get_mb_type)(cavs_decoder *p);
    int (*get_mb_pb_rec_opt)(cavs_decoder *p);
    
    /* picture_type, picture_structure, i_first_field */
    static int8_t i_num_of_reference_table[3][2][2]={
        {{1,0}/*picture_structure=0*/,{0,0}/*picture_structure=1*/},//i_picuture
        {{4,4}/*picture_structure=0*/,{2,2}/*picture_structure=1*/},//p_picuture
        {{4,4}/*picture_structure=0*/,{2,2}/*picture_structure=1*/}//b_picuture
    };
    
    int b_interlaced = p->param.b_interlaced;
    
    if( !b_interlaced )
    {	
        p->i_mb_y = p->i_mb_y_start;
        p->i_mb_index = p->i_mb_index_start;
    }
    else
    {
        if( !p->b_bottom )
        {
        	if((uint32_t)p->i_mb_y_start_fld[p->b_bottom] > (p->i_mb_height>>1) 
        		|| (uint32_t)p->i_mb_index_start_fld[p->b_bottom] > p->i_mb_num_half
        		|| (uint32_t)p->i_slice_mb_end_fld[p->b_bottom] > p->i_mb_num_half )
        	{
        		p->b_error_flag = 1;
        		return -1;
        	}
        }
        else
        {
        	if((uint32_t)p->i_mb_y_start_fld[p->b_bottom] < (p->i_mb_height>>1) 
        		|| (uint32_t)p->i_mb_index_start_fld[p->b_bottom] < p->i_mb_num_half
        		|| (uint32_t)p->i_slice_mb_end_fld[p->b_bottom] < p->i_mb_num_half )
        	{
        		p->b_error_flag = 1;
        		return -1;
        	}
        }

        p->i_mb_y = p->i_mb_y_start_fld[p->b_bottom];
        p->i_mb_index = p->i_mb_index_start_fld[p->b_bottom];
        p->i_slice_mb_end = p->i_slice_mb_end_fld[p->b_bottom];       
    }

    i_first_field = p->i_mb_index < p->i_mb_num_half ? 1 : 0;
    i_num = i_num_of_reference_table[p->ph.i_picture_coding_type][p->ph.b_picture_structure][i_first_field];
    p->b_have_pred = (p->ph.i_picture_coding_type != CAVS_I_PICTURE) || (p->ph.b_picture_structure == 0 && i_first_field == 0);
    p->sh.i_number_of_reference = i_num;
    
    if (p->ph.i_picture_coding_type < CAVS_B_PICTURE)   //p frame
    {
        i_skip = P_SKIP;
        i_limit = P_8X8;
        
        get_mb_pb_rec_opt = get_mb_p_rec_opt;

        get_mb_type = p->bs_read[SYNTAX_MBTYPE_P];
    }
    else    //b frame
    {
        i_skip = B_SKIP;
        i_limit = B_8X8;

        get_mb_pb_rec_opt = get_mb_b_rec_opt;

        get_mb_type = p->bs_read[SYNTAX_MBTYPE_B];
    }

    /* MB loop */
    for(;;)
    {  
        /* REC loop */
        cavs_init_slice_for_rec( p );
        for( ; ; )
        {   
            if( p->b_have_pred )
            {                
                p->i_mb_type =  p->i_mb_type_tab[p->i_mb_index];
                if(p->i_mb_type == I_8X8)
                {
                    i_result = get_mb_i_rec_opt(p);
                }
                else
                {
                    i_result = get_mb_pb_rec_opt(p);
                }
                
                if( p->b_error_flag )
                {
                    i_result = -1;
                }

                if(i_result!=0)
                {
                    goto end;
                }
            }
            else
            {
                p->i_mb_type_tab[p->i_mb_index] = I_8X8;
                i_result = get_mb_i_rec_opt(p);
                if( p->b_error_flag )
                {
                      i_result = -1;
                }
                
                if( i_result != 0 )
                {
                    goto end;
                }
            }

            p->i_mb_index++;
            if( p->i_mb_index >= p->i_slice_mb_end )  /* slice finished */
            {
                if( !b_interlaced )
                {
                    if (p->i_mb_index >= p->i_mb_num)   /* frame finished */
                    p->b_complete = 1;
                }
                else
                {
                    if( !p->b_bottom )
                    {
                        if (p->i_mb_index >= (p->i_mb_num>>1))  /* top-field finished */
                            p->b_complete = 1;
                    }
                    else
                    {
                        if (p->i_mb_index >= p->i_mb_num)   /* bot-field finished */
                            p->b_complete = 1;
                    }
                }

                i_result = 0;

                goto end;
            }
        }/* REC loop end */
    }

end:

    return i_result;
}	

static int cavs_get_slice_rec_threads( cavs_decoder *p )
{
	int ret;

#if B_MB_WEIGHTING
	/* load weighting prediction info */
	{
        p->sh.b_slice_weighting_flag = p->b_slice_weighting_flag[p->b_bottom];
    	p->sh.b_mb_weighting_flag = p->b_mb_weighting_flag[p->b_bottom];
		for(int i = 0; i < 4; i++ )
		{
			p->sh.i_luma_scale[i] = p->i_luma_scale[p->b_bottom][i];
			p->sh.i_luma_shift[i] = p->i_luma_shift[p->b_bottom][i];
			p->sh.i_chroma_scale[i] = p->i_chroma_scale[p->b_bottom][i];
			p->sh.i_chroma_shift[i] = p->i_chroma_shift[p->b_bottom][i];
		}
	}
#endif

    cavs_init_picture_ref_list( p );
    ret  = cavs_get_slice_rec( p );
    
    return ret;
}



static int cavs_get_slice(cavs_decoder *p)
{
    int i, i_num = 0, i_result;
    uint32_t i_limit, i_skip;
    int i_first_field, i_skip_count = 0;
    int (*get_mb_pb)(cavs_decoder *p);
    int (*get_mb_type)(cavs_decoder *p);
    
    /* picture_type, picture_structure, i_first_field */
    static int8_t i_num_of_reference_table[3][2][2]={
        {{1,0}/*picture_structure=0*/,{0,0}/*picture_structure=1*/},//i_picuture
        {{4,4}/*picture_structure=0*/,{2,2}/*picture_structure=1*/},//p_picuture
        {{4,4}/*picture_structure=0*/,{2,2}/*picture_structure=1*/}//b_picuture
    };
    
    int b_interlaced = p->param.b_interlaced;    

    if(p->vsh.i_vertical_size > 2800)
    {
        p->sh.i_slice_vertical_position_extension = cavs_bitstream_get_bits(&p->s,3);
    }
    else
    {
        p->sh.i_slice_vertical_position_extension = 0;
    }
    //if (p->vsh.i_profile_id == PROFILE_YIDONG)
    //{
    //    p->sh.i_slice_horizontal_position = cavs_bitstream_get_bits(&p->s,8);
    //    if(p->vsh.i_horizontal_size > 4080)
    //        p->sh.i_slice_vertical_position_extension = cavs_bitstream_get_bits(&p->s,2);
    //}
    if(p->ph.b_fixed_picture_qp == 0)
    {
        p->b_fixed_qp = p->sh.b_fixed_slice_qp = cavs_bitstream_get_bit1(&p->s);
        p->i_qp = p->sh.i_slice_qp = cavs_bitstream_get_bits(&p->s,6);
    }
    else
    {
        p->sh.b_fixed_slice_qp = 1;
        p->sh.i_slice_qp = p->i_qp;
    }
    p->i_mb_y += (p->sh.i_slice_vertical_position_extension<<7);
    p->i_mb_index = p->i_mb_y*p->i_mb_width;

    if(b_interlaced)
    {
        if( !p->b_bottom ) /* top field can't exceed half num */
        {
            if( (p->i_mb_y >= (p->i_mb_height>>1))
            || ( p->i_mb_index >= p->i_mb_num_half ) )
            {
                p->b_error_flag = 1;
                return -1;
            }
        }
        else
        {
            if( (p->i_mb_y < (p->i_mb_height>>1) )
            || ( p->i_mb_index < p->i_mb_num_half ))
            {
                p->b_error_flag = 1;
                return -1;
            }
        }
    }
    
    i_first_field = p->i_mb_index < p->i_mb_num_half ? 1 : 0;
    i_num = i_num_of_reference_table[p->ph.i_picture_coding_type][p->ph.b_picture_structure][i_first_field];
    p->b_have_pred = (p->ph.i_picture_coding_type != CAVS_I_PICTURE) || (p->ph.b_picture_structure == 0 && i_first_field == 0);
    if( p->b_have_pred )
    {
        p->sh.b_slice_weighting_flag = cavs_bitstream_get_bit1(&p->s);
        if(p->sh.b_slice_weighting_flag)
        {
            for(i = 0; i < i_num; i++)
            {
                p->sh.i_luma_scale[i] = cavs_bitstream_get_bits(&p->s, 8);
                p->sh.i_luma_shift[i] = cavs_bitstream_get_int(&p->s, 8);
                cavs_bitstream_clear_bits(&p->s, 1);
                p->sh.i_chroma_scale[i] = cavs_bitstream_get_bits(&p->s, 8);
                p->sh.i_chroma_shift[i] = cavs_bitstream_get_int(&p->s, 8);
                cavs_bitstream_clear_bits(&p->s, 1);
            }
            p->sh.b_mb_weighting_flag = cavs_bitstream_get_bit1(&p->s);
        }
        else
        {
            p->sh.b_mb_weighting_flag = 0;
        }
    }
    else
    {
        p->sh.b_slice_weighting_flag = 0;
        p->sh.b_mb_weighting_flag = 0;
    }
    
    //if (p->vsh.i_profile_id == PROFILE_YIDONG && p->ph.i_picture_coding_type == CAVS_P_PICTURE)
    //{
    //    p->quant_coeff_pred_flag = cavs_bitstream_get_bit1(&p->s);
    //}
    p->sh.i_number_of_reference = i_num;
    
    if (p->ph.i_picture_coding_type < CAVS_B_PICTURE)   //p frame
    {
        i_skip = P_SKIP;
        i_limit = P_8X8;
        
        get_mb_pb = get_mb_p;

        get_mb_type = p->bs_read[SYNTAX_MBTYPE_P];
    }
    else    //b frame
    {
        i_skip = B_SKIP;
        i_limit = B_8X8;

        get_mb_pb = get_mb_b;

        get_mb_type = p->bs_read[SYNTAX_MBTYPE_B];
    }

    /* init slice */
    i_result = cavs_init_slice(p);
	if(i_result == -1)
		goto end;

    /* MB loop */
    for(;;)
    {
        if(p->b_have_pred)
        {
            if(p->ph.b_skip_mode_flag)
            {
                i_skip_count = p->bs_read[SYNTAX_SKIP_RUN](p);
                if( i_skip_count == -1 ||
                    (p->i_mb_index + i_skip_count > p->i_slice_mb_end)
                    || i_skip_count < -1 )
                {
                    i_result = -1;
                    goto end;
                }
                                    
                if (p->ph.b_aec_enable && i_skip_count)
                {
                    cavs_biari_decode_stuffing_bit(&p->cabac);
                    p->i_last_dqp = 0;
                    p->i_cbp = 0;
                }
                while(i_skip_count--)
                {
                    p->i_mb_type = i_skip;
                    i_result = get_mb_pb(p);

                    if( p->b_error_flag )
                    {
                        i_result = -1;
                    }
                    
                    if(i_result != 0)
                    {
                        goto end;
                    }
                    
                    p->i_mb_index++;
                }
                
                if(p->i_mb_index >= p->i_slice_mb_end) //slice finished
                {
                    if( !b_interlaced )
                    {
                        if (p->i_mb_index >= p->i_mb_num)   /* frame finished */
                            p->b_complete = 1;
                    }
                    else
                    {
                        if( !p->b_bottom )
                        {
                            if (p->i_mb_index >= (p->i_mb_num>>1))  /* top-field finished */
                                p->b_complete = 1;
                        }
                        else
                        {
                            if (p->i_mb_index >= p->i_mb_num)   /* bot-field finished */
                                p->b_complete = 1;
                        }
                    }

                    i_result = 0;
                    goto end;
                }
            }

            p->i_mb_type = get_mb_type(p);
            if( p->i_mb_type < 0 || p->i_mb_type > 29)
            {
                p->b_error_flag = 1;
                i_result = -1;
                goto end;
            }
            
            if(p->i_mb_type == I_8X8)
            {
                i_result = get_mb_i(p);
            }
            else
            {
                i_result = get_mb_pb(p);
            }
            
            if( p->b_error_flag )
            {
                i_result = -1;
            }

            if (p->ph.b_aec_enable)
            {
                cavs_biari_decode_stuffing_bit(&p->cabac);
            }
            if(i_result != 0)
            {
                goto end;
            }
        }
        else
        {
            i_result = get_mb_i(p);

            if( p->b_error_flag )
            {
                  i_result = -1;
            }
            
            if (p->ph.b_aec_enable)
            {
                cavs_biari_decode_stuffing_bit(&p->cabac);
            }
            
            if( i_result != 0 )
            {
                goto end;
            }
        }

        p->i_mb_index++;
        if( p->i_mb_index >= p->i_slice_mb_end )  /* slice finished */
        {
            if( !b_interlaced )
            {
                if (p->i_mb_index >= p->i_mb_num)   /* frame finished */
                    p->b_complete = 1;
            }
            else
            {
                if( !p->b_bottom )
                {
                    if (p->i_mb_index >= (p->i_mb_num>>1))  /* top-field finished */
                        p->b_complete = 1;
                }
                else
                {
                    if (p->i_mb_index >= p->i_mb_num)   /* bot-field finished */
                        p->b_complete = 1;
                }
            }

            i_result = 0;

            goto end;
        }
    }

end:
    if (p->ph.b_aec_enable)
    {
        p->s = p->cabac.bs;
    }

    return i_result;
}

static void cavs_decoder_init( cavs_decoder *p )
{
    init_crop_table();   
	
    p->i_frame_decoded = -1;

    /* intra 4x4 */
    intra_pred_lum_4x4[I_PRED_4x4_V] = predict_4x4_v;
    intra_pred_lum_4x4[I_PRED_4x4_H] = predict_4x4_h;
    intra_pred_lum_4x4[I_PRED_4x4_DC] = predict_4x4_dc;
    intra_pred_lum_4x4[I_PRED_4x4_DDL]    = predict_4x4_ddl;
    intra_pred_lum_4x4[I_PRED_4x4_DDR]    = predict_4x4_ddr;
    intra_pred_lum_4x4[I_PRED_4x4_VR]     = predict_4x4_vr;
    intra_pred_lum_4x4[I_PRED_4x4_HD]     = predict_4x4_hd;
    intra_pred_lum_4x4[I_PRED_4x4_VL]     = predict_4x4_vl;
    intra_pred_lum_4x4[I_PRED_4x4_HU]     = predict_4x4_hu;
    intra_pred_lum_4x4[I_PRED_4x4_DC_LEFT]= predict_4x4_dc_left;
    intra_pred_lum_4x4[I_PRED_4x4_DC_TOP] = predict_4x4_dc_top;
    intra_pred_lum_4x4[I_PRED_4x4_DC_128] = predict_4x4_dc_128;

    p->cavs_intra_luma[INTRA_L_VERT] = cavs_intra_pred_vertical;
    p->cavs_intra_luma[INTRA_L_HORIZ] = cavs_intra_pred_horizontal;
    p->cavs_intra_luma[INTRA_L_LP] = cavs_intra_pred_dc_lp;
    p->cavs_intra_luma[INTRA_L_DOWN_LEFT] = cavs_intra_pred_down_left;
    p->cavs_intra_luma[INTRA_L_DOWN_RIGHT] = cavs_intra_pred_down_right;
    p->cavs_intra_luma[INTRA_L_LP_LEFT] = cavs_intra_pred_dc_lp_left;
    p->cavs_intra_luma[NTRA_L_LP_TOP] = cavs_intra_pred_dc_lp_top;
    p->cavs_intra_luma[INTRA_L_DC_128] = cavs_intra_pred_dc_128;

    p->cavs_intra_chroma[INTRA_C_DC] = cavs_intra_pred_chroma_dc_lp;
    p->cavs_intra_chroma[INTRA_C_HORIZ] = cavs_intra_pred_chroma_horizontal;
    p->cavs_intra_chroma[INTRA_C_VERT] = cavs_intra_pred_chroma_vertical;
    p->cavs_intra_chroma[INTRA_C_PLANE] = cavs_intra_pred_chroma_plane;
    p->cavs_intra_chroma[INTRA_C_DC_LEFT] = cavs_intra_pred_chroma_dc_lp_left;
    p->cavs_intra_chroma[INTRA_C_DC_TOP] = cavs_intra_pred_chroma_dc_lp_top;
    p->cavs_intra_chroma[INTRA_C_DC_128] = cavs_intra_pred_chroma_dc_128;

    /* deblock */
    p->filter_lv = cavs_filter_lv_c;
    p->filter_lh = cavs_filter_lh_c;
    p->filter_cv = cavs_filter_cv_c;
    p->filter_ch = cavs_filter_ch_c;

    cavs_idct8_add = cavs_idct8_add_c;
    cavs_idct4_add = cavs_idct4_add_c;

    p->put_h264_chroma_pixels_tab[0] = cavs_chroma_mc8_put_c;
    p->put_h264_chroma_pixels_tab[1] = cavs_chroma_mc4_put_c;
    p->put_h264_chroma_pixels_tab[2] = cavs_chroma_mc2_put_c;

    p->avg_h264_chroma_pixels_tab[0] = cavs_chroma_mc8_avg_c;
    p->avg_h264_chroma_pixels_tab[1] = cavs_chroma_mc4_avg_c;
    p->avg_h264_chroma_pixels_tab[2] = cavs_chroma_mc2_avg_c;

    p->put_h264_chroma_pixels_tab[2] = cavs_chroma_mc2_put_c;
    p->avg_h264_chroma_pixels_tab[2] = cavs_chroma_mc2_avg_c;

    p->put_cavs_qpel_pixels_tab[0][0] =  cavs_qpel16_put_mc00_c; 
    p->put_cavs_qpel_pixels_tab[0][ 1] = cavs_qpel16_put_mc10_c;
    p->put_cavs_qpel_pixels_tab[0][ 2] = cavs_qpel16_put_mc20_c;
    p->put_cavs_qpel_pixels_tab[0][ 3] = cavs_qpel16_put_mc30_c;
    p->put_cavs_qpel_pixels_tab[0][ 4] = cavs_qpel16_put_mc01_c; 
    p->put_cavs_qpel_pixels_tab[0][ 5] = cavs_qpel16_put_mc11_c;
    p->put_cavs_qpel_pixels_tab[0][ 6] = cavs_qpel16_put_mc21_c;
    p->put_cavs_qpel_pixels_tab[0][ 7] = cavs_qpel16_put_mc31_c;
    p->put_cavs_qpel_pixels_tab[0][ 8] = cavs_qpel16_put_mc02_c;
    p->put_cavs_qpel_pixels_tab[0][ 9] = cavs_qpel16_put_mc12_c;
    p->put_cavs_qpel_pixels_tab[0][10] = cavs_qpel16_put_mc22_c;
    p->put_cavs_qpel_pixels_tab[0][11] = cavs_qpel16_put_mc32_c;
    p->put_cavs_qpel_pixels_tab[0][12] = cavs_qpel16_put_mc03_c;
    p->put_cavs_qpel_pixels_tab[0][13] = cavs_qpel16_put_mc13_c;
    p->put_cavs_qpel_pixels_tab[0][14] = cavs_qpel16_put_mc23_c;
    p->put_cavs_qpel_pixels_tab[0][15] = cavs_qpel16_put_mc33_c;
    p->put_cavs_qpel_pixels_tab[1][ 0] = cavs_qpel8_put_mc00_c; 
    p->put_cavs_qpel_pixels_tab[1][ 1] = cavs_qpel8_put_mc10_c;
    p->put_cavs_qpel_pixels_tab[1][ 2] = cavs_qpel8_put_mc20_c; 
    p->put_cavs_qpel_pixels_tab[1][ 3] = cavs_qpel8_put_mc30_c;
    p->put_cavs_qpel_pixels_tab[1][ 4] = cavs_qpel8_put_mc01_c; 
    p->put_cavs_qpel_pixels_tab[1][ 5] = cavs_qpel8_put_mc11_c;
    p->put_cavs_qpel_pixels_tab[1][ 6] = cavs_qpel8_put_mc21_c;
    p->put_cavs_qpel_pixels_tab[1][ 7] = cavs_qpel8_put_mc31_c;
    p->put_cavs_qpel_pixels_tab[1][ 8] = cavs_qpel8_put_mc02_c; 
    p->put_cavs_qpel_pixels_tab[1][ 9] = cavs_qpel8_put_mc12_c;
    p->put_cavs_qpel_pixels_tab[1][10] = cavs_qpel8_put_mc22_c;
    p->put_cavs_qpel_pixels_tab[1][11] = cavs_qpel8_put_mc32_c;
    p->put_cavs_qpel_pixels_tab[1][12] = cavs_qpel8_put_mc03_c; 
    p->put_cavs_qpel_pixels_tab[1][13] = cavs_qpel8_put_mc13_c;
    p->put_cavs_qpel_pixels_tab[1][14] = cavs_qpel8_put_mc23_c;
    p->put_cavs_qpel_pixels_tab[1][15] = cavs_qpel8_put_mc33_c;

    p->avg_cavs_qpel_pixels_tab[0][ 0] = cavs_qpel16_avg_mc00_c; 
    p->avg_cavs_qpel_pixels_tab[0][ 1] = cavs_qpel16_avg_mc10_c;
    p->avg_cavs_qpel_pixels_tab[0][ 2] = cavs_qpel16_avg_mc20_c;
    p->avg_cavs_qpel_pixels_tab[0][ 3] = cavs_qpel16_avg_mc30_c;
    p->avg_cavs_qpel_pixels_tab[0][ 4] = cavs_qpel16_avg_mc01_c;
    p->avg_cavs_qpel_pixels_tab[0][ 5] = cavs_qpel16_avg_mc11_c;
    p->avg_cavs_qpel_pixels_tab[0][ 6] = cavs_qpel16_avg_mc21_c;
    p->avg_cavs_qpel_pixels_tab[0][ 7] = cavs_qpel16_avg_mc31_c;
    p->avg_cavs_qpel_pixels_tab[0][ 8] = cavs_qpel16_avg_mc02_c;
    p->avg_cavs_qpel_pixels_tab[0][ 9] = cavs_qpel16_avg_mc12_c;
    p->avg_cavs_qpel_pixels_tab[0][10] = cavs_qpel16_avg_mc22_c;
    p->avg_cavs_qpel_pixels_tab[0][11] = cavs_qpel16_avg_mc32_c;
    p->avg_cavs_qpel_pixels_tab[0][12] = cavs_qpel16_avg_mc03_c;
    p->avg_cavs_qpel_pixels_tab[0][13] = cavs_qpel16_avg_mc13_c;
    p->avg_cavs_qpel_pixels_tab[0][14] = cavs_qpel16_avg_mc23_c;
    p->avg_cavs_qpel_pixels_tab[0][15] = cavs_qpel16_avg_mc33_c;

    p->avg_cavs_qpel_pixels_tab[1][ 0] = cavs_qpel8_avg_mc00_c;
    p->avg_cavs_qpel_pixels_tab[1][ 1] = cavs_qpel8_avg_mc10_c;
    p->avg_cavs_qpel_pixels_tab[1][ 2] = cavs_qpel8_avg_mc20_c;
    p->avg_cavs_qpel_pixels_tab[1][ 3] = cavs_qpel8_avg_mc30_c;
    p->avg_cavs_qpel_pixels_tab[1][ 4] = cavs_qpel8_avg_mc01_c;
    p->avg_cavs_qpel_pixels_tab[1][ 5] = cavs_qpel8_avg_mc11_c;
    p->avg_cavs_qpel_pixels_tab[1][ 6] = cavs_qpel8_avg_mc21_c;
    p->avg_cavs_qpel_pixels_tab[1][ 7] = cavs_qpel8_avg_mc31_c;
    p->avg_cavs_qpel_pixels_tab[1][ 8] = cavs_qpel8_avg_mc02_c;
    p->avg_cavs_qpel_pixels_tab[1][ 9] = cavs_qpel8_avg_mc12_c;
    p->avg_cavs_qpel_pixels_tab[1][10] = cavs_qpel8_avg_mc22_c;
    p->avg_cavs_qpel_pixels_tab[1][11] = cavs_qpel8_avg_mc32_c;
    p->avg_cavs_qpel_pixels_tab[1][12] = cavs_qpel8_avg_mc03_c;
    p->avg_cavs_qpel_pixels_tab[1][13] = cavs_qpel8_avg_mc13_c;
    p->avg_cavs_qpel_pixels_tab[1][14] = cavs_qpel8_avg_mc23_c;
    p->avg_cavs_qpel_pixels_tab[1][15] = cavs_qpel8_avg_mc33_c;

#if B_MB_WEIGHTING
    /* weighting prediction */
    p->cavs_avg_pixels_tab[0] = pixel_avg_16x16_c;
    p->cavs_avg_pixels_tab[1] = pixel_avg_8x8_c;
    p->cavs_avg_pixels_tab[2] = pixel_avg_4x4_c;
#endif /* B_MB_WEIGHTING */     
}

static void cavs_out_image_yuv420(cavs_decoder *p, cavs_image *p_cur, uint8_t *p_yuv_in[3], int b_bottom )
{
    uint32_t j = 0;
    unsigned char* p_bufimg;
    uint8_t *p_yuv;
    int b_interlaced = p->param.b_interlaced;
    int b_align = 0;
    
    if( !b_interlaced )
    {
        /* Y */
        p_bufimg = p_cur->p_data[0];
        p_yuv = p_yuv_in[0];
        for (j = 0; j < p->vsh.i_vertical_size; j++)
        {
            memcpy(p_yuv, p_bufimg, p->vsh.i_horizontal_size);
            p_yuv += p->vsh.i_horizontal_size;
            p_bufimg += p_cur->i_stride[0];
        }

        /* U */
        p_yuv = p_yuv_in[1];
        p_bufimg = p_cur->p_data[1];
        for (j = 0; j < p->vsh.i_vertical_size/2; j++)
        {
            memcpy(p_yuv, p_bufimg, p->vsh.i_horizontal_size/2); 
            p_yuv += p->vsh.i_horizontal_size/2;
            p_bufimg += p_cur->i_stride[1];
        }

        /* V */
        p_yuv = p_yuv_in[2];
        p_bufimg = p_cur->p_data[2];
        for ( j = 0; j < p->vsh.i_vertical_size/2; j++ )
        {
            memcpy(p_yuv, p_bufimg, p->vsh.i_horizontal_size/2);      
            p_yuv += p->vsh.i_horizontal_size/2;
            p_bufimg += p_cur->i_stride[2];
        }
    }
    else
    {
        int i_offset_y = 0;
        int i_offset_uv = 0;
        int i_bufimg_y = 0;
        int i_bufimg_uv = 0;

        if( b_interlaced && /*p->b_bottom*/b_bottom )
        {
            i_offset_y = ((p_cur->i_stride[0])>>1) - 32;
            i_offset_uv = ((p_cur->i_stride[1])>>1) - 16;
            i_bufimg_y =  (p_cur->i_stride[0]>>1);
            i_bufimg_uv =  (p_cur->i_stride[1]>>1);
        }

        /* Y */
        p_yuv = p_yuv_in[0] + i_offset_y;
        p_bufimg = p_cur->p_data[0] + i_bufimg_y;
        for ( j = 0; j < (p->vsh.i_vertical_size>>b_interlaced); j++ )
        {
            memcpy( p_yuv, p_bufimg, p->vsh.i_horizontal_size);
            p_yuv += (p->vsh.i_horizontal_size<<b_interlaced);
            p_bufimg += p_cur->i_stride[0];
        }

        /* U */
        p_yuv = p_yuv_in[1] + i_offset_uv;
        p_bufimg = p_cur->p_data[1] + i_bufimg_uv;
        for ( j = 0; j < ((p->vsh.i_vertical_size/2)>>b_interlaced); j++ )
        {
            memcpy( p_yuv, p_bufimg, p->vsh.i_horizontal_size/2 );
            p_yuv += ((p->vsh.i_horizontal_size/2)<<b_interlaced);
            p_bufimg += p_cur->i_stride[1];
        }

        /* V */
        p_yuv = p_yuv_in[2] + i_offset_uv;
        p_bufimg = p_cur->p_data[2] + i_bufimg_uv;
        for ( j = 0; j < ((p->vsh.i_vertical_size/2)>>b_interlaced); j++ )
        {  
            memcpy(p_yuv, p_bufimg, p->vsh.i_horizontal_size/2); 
            p_yuv += ((p->vsh.i_horizontal_size/2)<<b_interlaced);
            p_bufimg += p_cur->i_stride[2];
        }
    }
}

static int cavs_get_extension(cavs_bitstream *s, cavs_decoder *p)
{
	int extension_id = cavs_bitstream_get_bits(s, 4);
	if (extension_id == 2)	//sequence display extension
	{
		return cavs_get_sequence_display_extension(s, &p->sde);
	}
	else if (extension_id == 4) //copyright extension
	{
		return cavs_get_copyright_extension(s, &p->ce);
	}
	else if (extension_id == 11) //camera extension
	{
		return cavs_get_camera_parameters_extension(s, &p->cpe);
	}
	else
	{
		while (((*(uint32_t*)(s->p)) & 0x00ffffff) != 0x000001/*0x00010000*/ && s->p < s->p_end )
			cavs_bitstream_get_bits(s, 8);
	}
	return 0;
}

int cavs_macroblock_cache_init( cavs_decoder *p );

static void cavs_bitstream_save( frame_pack *frame, InputStream*p )
{
    int64_t min_length;
    
    frame->bits.p_ori = p->f;
    frame->bits.iBitsCount = p->iBitsCount;
    frame->bits.iBufBytesNum = p->iBufBytesNum;
    frame->bits.iBytePosition = p->iBytePosition;
    frame->bits.iClearBitsNum = p->iClearBitsNum;
    frame->bits.iStuffBitsNum = p->iStuffBitsNum;
    frame->bits.uClearBits = p->uClearBits;
    frame->bits.uPre3Bytes = p->uPre3Bytes;
    frame->bits.demulate_enable = p->demulate_enable;

    min_length = p->f_end - p->f;
    min_length = MIN2(SVA_STREAM_BUF_SIZE, min_length );
    frame->bits.min_length = min_length;
    memcpy(frame->bits.buf, p->buf,  /*min_length*/SVA_STREAM_BUF_SIZE );
}

static void cavs_bitstream_restore( InputStream*p, frame_pack *frame )
{
    p->f = frame->bits.p_ori;
    p->iBitsCount = frame->bits.iBitsCount;
    p->iBufBytesNum = frame->bits.iBufBytesNum;
    p->iBytePosition = frame->bits.iBytePosition;
    p->iClearBitsNum = frame->bits.iClearBitsNum;
    p->iStuffBitsNum = frame->bits.iStuffBitsNum;
    p->uClearBits = frame->bits.uClearBits;
    p->uPre3Bytes = frame->bits.uPre3Bytes;
    p->demulate_enable = frame->bits.demulate_enable;    
    memcpy( p->buf, frame->bits.buf, /*frame->bits.min_length*/SVA_STREAM_BUF_SIZE );
}

uint32_t cavs_frame_pack( InputStream*p, frame_pack *frame )
{
    uint32_t i_startcode;

    int i_len;

    /* pic header */
    i_startcode = frame->slice[0].i_startcode ;
	
    while(i_startcode)
    {  
        cavs_bitstream_save( frame, p );
        i_startcode = cavs_get_one_nal( p, frame->m_buf, &i_len );

        switch( i_startcode )
        {	
            case CAVS_VIDEO_SEQUENCE_START_CODE:
                cavs_bitstream_restore( p, frame );
                return i_startcode;
            case CAVS_VIDEO_SEQUENCE_END_CODE:
                cavs_bitstream_restore( p, frame );
                return i_startcode;
            case CAVS_VIDEO_EDIT_CODE:
                cavs_bitstream_restore( p, frame );
                return i_startcode;
            case CAVS_I_PICUTRE_START_CODE:
                cavs_bitstream_restore( p, frame );
                return i_startcode;
            case CAVS_PB_PICUTRE_START_CODE:
                cavs_bitstream_restore( p, frame );
                return i_startcode;

            case CAVS_USER_DATA_CODE:
                break;
            case CAVS_EXTENSION_START_CODE:
                break;
            case 0x000001fe: // FIXIT
                cavs_bitstream_restore( p, frame );
                return i_startcode;
            default:
                memcpy(frame->p_cur, frame->m_buf, i_len ); /* pack slice into one frame */

                /* set slice info */
                frame->slice[frame->slice_num].i_startcode = i_startcode;
                frame->slice[frame->slice_num].p_start = frame->p_cur + 4; /* for skip current startcode */
                frame->slice[frame->slice_num].i_len = i_len-4;
        
                /*update frame info */
                frame->p_cur = frame->p_cur + i_len;
                frame->i_len += i_len;
                frame->slice_num++;
                break;
    	 }
    }

    return 0;
}

static void cavs_decoder_thread_init( cavs_decoder *p )
{

}

int cavs_decoder_process(void *p_decoder, unsigned char *p_in, int i_in_length)
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;
    uint32_t   i_startcode;
    uint8_t   *p_buf;
    int    i_len,i_result=0;

    p_buf = p_in;
    i_len = i_in_length;	

    i_startcode = ((p_buf[0]<<24))|((p_buf[1]<<16))|(p_buf[2]<<8)|p_buf[3];
    p_buf += 4; /* skip startcode */
    i_len -= 4;
    if(i_startcode==0)
    {
    	return CAVS_ERROR;
    }

    if(i_startcode!=CAVS_VIDEO_SEQUENCE_START_CODE&&!p->b_get_video_sequence_header)
    {
    	return  CAVS_ERROR;
    } 

    switch(i_startcode)
    {
    case CAVS_VIDEO_SEQUENCE_START_CODE:
    	cavs_bitstream_init(&p->s,p_buf,i_len);
    	if(cavs_get_video_sequence_header(&p->s,&p->vsh)==0)
    	{
    		if(cavs_alloc_resource(p)!=0)
    		{
    			return CAVS_ERROR;
    		}
    		i_result = CAVS_SEQ_HEADER;
    		p->b_get_video_sequence_header=1;

            if(!p->b_threadpool_flag)
            {
                if( p->param.i_thread_num >= 1 &&
                    cavs_threadpool_init( &p->threadpool, p->param.i_thread_num, (void*)cavs_decoder_thread_init, p ) )
                        goto fail;
                p->b_threadpool_flag = 1;
            }

    		if( !p->b_thread_flag &&  p->param.i_thread_num > 0 )
    		{
    			int i;

    			p->thread[0] = p;
				if( cavs_pthread_mutex_init( &p->thread[0]->mutex, NULL ) )
    					goto fail;
    			if( cavs_pthread_cond_init( &p->thread[0]->cv, NULL ) )
    					goto fail;

    			for( i = 1; i < p->param.i_thread_num; i++ )
    				CHECKED_MALLOC( p->thread[i], sizeof(cavs_decoder) );

    			for( i = 1; i < p->param.i_thread_num; i++ )
    			{
    				if( i > 0 )
    					*p->thread[i] = *p;
    				if( cavs_pthread_mutex_init( &p->thread[i]->mutex, NULL ) )
    					goto fail;
    				if( cavs_pthread_cond_init( &p->thread[i]->cv, NULL ) )
    					goto fail;

                    if(cavs_macroblock_cache_init( p->thread[i]))
                    	goto fail;    			
    			}
    			p->b_thread_flag = 1;
    		}

             if( !p->b_accelerate_flag && p->param.b_accelerate )
            {
                p->unused[0] = p;
                CHECKED_MALLOC( p->unused[1], sizeof(cavs_decoder) );

                *p->unused[1] = *p;

                if( cavs_pthread_mutex_init( &p->unused[1]->mutex, NULL ) )
                	goto fail;
                if( cavs_pthread_cond_init( &p->unused[1]->cv, NULL ) )
                	goto fail;

                if(cavs_macroblock_cache_init( p->unused[1] ))
                    goto fail;

                p->b_accelerate_flag = 1;

                p->unused_backup[0] = p->unused[0];
                p->unused_backup[1] = p->unused[1];
            }
    	}
       else
       {
           return CAVS_ERROR;          
       }
    	break;
    case CAVS_VIDEO_SEQUENCE_END_CODE:
      if( !p->param.b_accelerate )
        p->last_delayed_pframe = p->p_save[1];

    	i_result = CAVS_SEQ_END;
    	break;
    case CAVS_USER_DATA_CODE:
    	cavs_bitstream_init(&p->s,p_buf,i_len);
    	i_result = cavs_get_user_data(&p->s, &p->user_data);
		i_result = CAVS_USER_DATA;
    	break;
    case CAVS_EXTENSION_START_CODE:
    	cavs_bitstream_init(&p->s,p_buf,i_len);
    	i_result = cavs_get_extension(&p->s, p);
		i_result = CAVS_USER_DATA;
    	break;
    case CAVS_VIDEO_EDIT_CODE:
    	p->i_video_edit_code_flag=0;
		i_result = CAVS_USER_DATA;
    	break;
    default:
	    i_result = CAVS_ERROR;
	    break;

    }

    return i_result;

fail:
    return CAVS_ERROR;//-1;
}

int cavs_macroblock_cache_init( cavs_decoder *p )
{
    uint32_t i_mb_width, i_mb_height, i_edge;

    i_mb_width = (p->vsh.i_horizontal_size+15)>>4;
    if ( p->vsh.b_progressive_sequence )
        i_mb_height = (p->vsh.i_vertical_size+15)>>4;
    else
        i_mb_height = ((p->vsh.i_vertical_size+31) & 0xffffffe0)>>4;

    p->i_mb_width = i_mb_width;
    p->i_mb_height = i_mb_height;
    p->i_mb_num = i_mb_width*i_mb_height;
    p->i_mb_num_half = p->i_mb_num>>1;
    p->p_top_qp = (uint8_t *)cavs_malloc( p->i_mb_width);

    p->p_top_mv[0] = (cavs_vector *)cavs_malloc((p->i_mb_width*2+1)*sizeof(cavs_vector));
    p->p_top_mv[1] = (cavs_vector *)cavs_malloc((p->i_mb_width*2+1)*sizeof(cavs_vector));
    p->p_top_intra_pred_mode_y = (int *)cavs_malloc( p->i_mb_width*4*sizeof(*p->p_top_intra_pred_mode_y));
    p->p_mb_type_top = (int8_t *)cavs_malloc(p->i_mb_width*sizeof(int8_t));
    p->p_chroma_pred_mode_top = (int8_t *)cavs_malloc(p->i_mb_width*sizeof(int8_t));
    p->p_cbp_top = (int8_t *)cavs_malloc(p->i_mb_width*sizeof(int8_t));
    p->p_ref_top = (int8_t (*)[2][2])cavs_malloc(p->i_mb_width*2*2*sizeof(int8_t));
    p->p_top_border_y = (uint8_t *)cavs_malloc((p->i_mb_width+1)*CAVS_MB_SIZE);
    p->p_top_border_cb = (uint8_t *)cavs_malloc((p->i_mb_width)*10);
    p->p_top_border_cr = (uint8_t *)cavs_malloc((p->i_mb_width)*10);

    p->p_col_mv = (cavs_vector *)cavs_malloc( p->i_mb_width*(p->i_mb_height)*4*sizeof(cavs_vector));
    p->p_col_type_base = (uint8_t *)cavs_malloc(p->i_mb_width*(p->i_mb_height));
    p->p_block = (DCTELEM *)cavs_malloc(64*sizeof(DCTELEM));

    if( p->param.b_accelerate )
    {
        p->level_buf_tab = (int (*)[6][64])cavs_malloc(p->i_mb_num*6*64*sizeof(int));
        p->run_buf_tab = (int (*)[6][64])cavs_malloc(p->i_mb_num*6*64*sizeof(int));
        p->num_buf_tab = (int (*)[6])cavs_malloc(p->i_mb_num*6*sizeof(int));

        p->i_mb_type_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->i_qp_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->i_cbp_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->mv_tab = (cavs_vector (*)[24])cavs_malloc( p->i_mb_num*24*sizeof(cavs_vector));

        p->i_intra_pred_mode_y_tab = (int (*)[25])cavs_malloc(p->i_mb_num*25*sizeof(int));
        p->i_pred_mode_chroma_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
        p->p_mvd_tab = (int16_t (*)[2][6][2])cavs_malloc(p->i_mb_num*2*6*2*sizeof(int16_t));
        p->p_ref_tab = (int8_t (*)[2][9])cavs_malloc(p->i_mb_num*2*9*sizeof(int8_t));
#if B_MB_WEIGHTING
        p->weighting_prediction_tab = (int *)cavs_malloc(p->i_mb_num*sizeof(int));
#endif		
	}
    
    
    i_edge = (p->i_mb_width*CAVS_MB_SIZE + CAVS_EDGE*2)*2*17*2*2;
    p->p_edge = (uint8_t *)cavs_malloc(i_edge);
    if( p->p_edge )
    {
        memset(p->p_edge, 0, i_edge);
    }
    p->p_edge += i_edge/2;

    if(   !p->p_top_qp                 ||!p->p_top_mv[0]          ||!p->p_top_mv[1]
        ||!p->p_top_intra_pred_mode_y  ||!p->p_top_border_y       ||!p->p_top_border_cb
        ||!p->p_top_border_cr          ||!p->p_col_mv             ||!p->p_col_type_base
        ||!p->p_block                  ||!p->p_edge				  ||!p->p_mb_type_top
        ||!p->p_chroma_pred_mode_top   ||!p->p_cbp_top			  ||!p->p_ref_top)
    {
        return -1;
    }

    memset(p->p_block,0,64*sizeof(DCTELEM));

	return 0;
}

static int cavs_macroblock_cache_finit(cavs_decoder *p)
{
	uint32_t i_edge;

	CAVS_SAFE_FREE(p->p_top_qp);
	CAVS_SAFE_FREE(p->p_top_mv[0]);
	CAVS_SAFE_FREE(p->p_top_mv[1]);
	CAVS_SAFE_FREE(p->p_top_intra_pred_mode_y);
	CAVS_SAFE_FREE(p->p_mb_type_top);
	CAVS_SAFE_FREE(p->p_chroma_pred_mode_top);
	CAVS_SAFE_FREE(p->p_cbp_top);
	CAVS_SAFE_FREE(p->p_ref_top);
	CAVS_SAFE_FREE(p->p_top_border_y);
	CAVS_SAFE_FREE(p->p_top_border_cb);
	CAVS_SAFE_FREE(p->p_top_border_cr);

	CAVS_SAFE_FREE(p->p_col_mv);
	CAVS_SAFE_FREE(p->p_col_type_base);
	CAVS_SAFE_FREE(p->p_block);

	if (p->param.b_accelerate)
	{
		CAVS_SAFE_FREE(p->level_buf_tab);
		CAVS_SAFE_FREE(p->run_buf_tab);
		CAVS_SAFE_FREE(p->num_buf_tab);

		CAVS_SAFE_FREE(p->i_mb_type_tab);
		CAVS_SAFE_FREE(p->i_qp_tab);
		CAVS_SAFE_FREE(p->i_cbp_tab);
		CAVS_SAFE_FREE(p->mv_tab);

		CAVS_SAFE_FREE(p->i_intra_pred_mode_y_tab);
		CAVS_SAFE_FREE(p->i_pred_mode_chroma_tab);
		CAVS_SAFE_FREE(p->p_mvd_tab);
		CAVS_SAFE_FREE(p->p_ref_tab);

#if B_MB_WEIGHTING
		CAVS_SAFE_FREE(p->weighting_prediction_tab);
#endif		

	}

	i_edge = (p->i_mb_width*CAVS_MB_SIZE + CAVS_EDGE * 2) * 2 * 17 * 2 * 2;
	p->p_edge -= i_edge / 2;
	CAVS_SAFE_FREE(p->p_edge);

	return 0;
}

int cavs_decode_one_frame_delay( void *p_decoder, cavs_param *param )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    cavs_decoder *p_m = p;
    cavs_decoder *p_rec;
    int b_interlaced = p->param.b_interlaced;

    int i_result = 0;
    
    /* REC */
    /*init REC handle */
    p->current[0] = p->current[1];
    p_rec = p->current[0];
    
    p->i_frame_decoded++;
    if( p_rec != NULL )
    {
        if( !b_interlaced )
        {  
            cavs_init_picture_ref_list( p_rec );
            cavs_get_slice_rec( p_rec );

            /* this module will effect ref-list creature and update */
            if(p_rec->b_complete && p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE)
            {
                cavs_image *p_image = p_rec->p_save[0];
                if(p_rec->vsh.b_low_delay)
                {
                    i_result = CAVS_FRAME_OUT;

                    /* set for ffmpeg interface */
                    param->output_type = p_rec->p_save[0]->i_code_type;

                    /* when b_low_delay is 1, output current frame when finish decoded current */
                    cavs_out_image_yuv420(p, p_rec->p_save[0], param->p_out_yuv, 0);
                }
                if(!p_rec->p_save[1])
                {
                    p_rec->p_save[0] = &p_rec->image[1];
                }
                else
                {
                    if( p_rec->vsh.b_low_delay == 0 /*&& p_rec->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                    {
                        i_result = CAVS_FRAME_OUT;

                        /* set for ffmpeg interface */
                        param->output_type = p_rec->p_save[1]->i_code_type;
                          
                        /* when b_low_delay is 0, current frame should delay one frame to output */
                        cavs_out_image_yuv420(p, p_rec->p_save[1], param->p_out_yuv, 0);
                    }

                    if(!p_rec->p_save[2])
                    {
                        p_rec->p_save[0] = &p_rec->image[2];	 
                    }
                    else
                    {
                        p_rec->p_save[0] = p_rec->p_save[2];
                    }
                }
                p_rec->p_save[2] = p_rec->p_save[1];
                p_rec->p_save[1] = p_image;
                memset(&p_rec->cur, 0, sizeof(cavs_image));
                p_rec->b_complete = 0;

                /* copy ref-list to anther threads */
                p->current[1]->p_save[0] = p_rec->p_save[0];
                p->current[1]->p_save[1] = p_rec->p_save[1];
                p->current[1]->p_save[2] = p_rec->p_save[2];
            }
            else if(p_rec->b_complete)
            {
                i_result = CAVS_FRAME_OUT;

                /* set for ffmpeg interface */
                param->output_type = p_rec->p_save[0]->i_code_type;
                  
                /* ouput B-frame no delay */
                cavs_out_image_yuv420(p, p_rec->p_save[0], param->p_out_yuv, 0);
                memset(&p_rec->cur,0,sizeof(cavs_image));
                p_rec->b_complete=0;
            }

            /* NOTE : need this to output last frame */
            p->last_delayed_pframe = p_rec->p_save[1];
            p->p_save[1] = p_rec->p_save[1];
        }
        else /* field */
        {
            int field_count = 2;

            while(field_count--)
            {
                p_rec->b_bottom = !field_count; /* first-top second-botttom */
                cavs_init_picture_context_fld( p_rec );
                cavs_get_slice_rec_threads( p_rec );

                if(p_rec->b_complete && p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE)
                {
                    cavs_image *p_image = p_rec->p_save[0];
                    if(p_rec->vsh.b_low_delay)
                    {
                        i_result = CAVS_FRAME_OUT;
                        
                        /* set for ffmpeg interface */
                        param->output_type = p_rec->p_save[0]->i_code_type;
                          
                        cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv, p_rec->b_bottom);
                    }
                    if(!p_rec->p_save[1])
                    {
                        if( p_rec->b_bottom )
                            p_rec->p_save[0] = &p_rec->image[1];
                    }
                    else
                    {
                        if(p_rec->vsh.b_low_delay == 0 /*&& p_rec->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                        {
                            i_result = CAVS_FRAME_OUT;

                            /* set for ffmpeg interface */
                            param->output_type = p_rec->p_save[1]->i_code_type;
                              
                            cavs_out_image_yuv420(p_m, p_rec->p_save[1], param->p_out_yuv, p_rec->b_bottom);
                        }

                        if( p_rec->b_bottom )
                        {
                            if(!p_rec->p_save[2])
                            {
                                p_rec->p_save[0] = &p_rec->image[2];	 
                            }
                            else
                            {
                                p_rec->p_save[0] = p_rec->p_save[2];
                            }    
                        }
                    }

                    if( p_rec->b_bottom )
                    {
                        p_rec->p_save[2] = p_rec->p_save[1];
                        p_rec->p_save[1] = p_image;
                        memset(&p_rec->cur, 0, sizeof(cavs_image));

                        /* copy ref-list to anther threads */
                        p_m->current[1]->p_save[0] = p_rec->p_save[0];
                        p_m->current[1]->p_save[1] = p_rec->p_save[1];
                        p_m->current[1]->p_save[2] = p_rec->p_save[2];
                    }
                    p_rec->b_complete = 0;
                    p_rec->b_bottom = 0;
                }
                else if( p_rec->b_complete )
                {
                    i_result = CAVS_FRAME_OUT;
                    
                    /* set for ffmpeg interface */
                    param->output_type = p_rec->p_save[0]->i_code_type;
                      
                    cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv, p_rec->b_bottom);
                    memset(&p_rec->cur,0,sizeof(cavs_image));
                    p_rec->b_complete=0;
                    p_rec->b_bottom = 0;
                }	                         
            }

            /* NOTE : need this to output last frame */
            p->last_delayed_pframe = p_rec->p_save[1];
            p->p_save[1] = p_rec->p_save[1];
        }
    }
  
    return i_result;

}

int decode_one_frame(void *p_decoder, frame_pack *frame, cavs_param *param )
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;

    uint32_t  cur_startcode;
    uint8_t   *p_buf;
    int    i_len, i_result=0;
    int sliceNum, threadNum;
    int slice_count = 1; /* 0 is for pic header */

    sliceNum = frame->slice_num - 1; /* exclude pic header */
    threadNum = 0;

    cur_startcode = frame->slice[0].i_startcode;
    p_buf = frame->slice[0].p_start;
    i_len = frame->slice[0].i_len;
    frame->slice_num--;

    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf, i_len);
            cavs_get_i_picture_header(&p->s, &p->ph, &p->vsh);
			
			if( p->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{
				p->ph.b_aec_enable = p->vsh.b_aec_enable;
			}
			else if(p->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p->b_error_flag  = 1;

                return CAVS_ERROR;
			}
            
			p->ph.b_picture_structure = 1; /* NOTE : guarantee frame not change */
            cavs_init_picture(p);

            /* add for no-seq header before I-frame */
            p->last_delayed_pframe = p->p_save[1];
            break;

        case CAVS_PB_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf, i_len);
            cavs_get_pb_picture_header(&p->s, &p->ph, &p->vsh);

			if( p->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{	
				//p->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p->ph.b_aec_enable = p->vsh.b_aec_enable;
			}
			else if(p->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p->b_error_flag  = 1;

                return CAVS_ERROR;
			}

			p->ph.b_picture_structure = 1; /* NOTE : guarantee frame not change */
            cavs_init_picture(p);

            break;

        default:
            return CAVS_ERROR;
    }	
    
    /* weighting quant */
    cavs_init_frame_quant_param( p );

    cavs_frame_update_wq_matrix( p );

    /* decode slice */
    while( frame->slice_num--)
    {
        cur_startcode = frame->slice[slice_count].i_startcode;
        p_buf = frame->slice[slice_count].p_start;
        i_len = frame->slice[slice_count].i_len;
        slice_count++;

        switch( cur_startcode )
        {
            case CAVS_USER_DATA_CODE:
                cavs_bitstream_init(&p->s,p_buf,i_len);
                i_result = cavs_get_user_data(&p->s, &p->user_data);
                break;

            case CAVS_EXTENSION_START_CODE:
                cavs_bitstream_init(&p->s,p_buf,i_len);
                i_result = cavs_get_extension(&p->s, p);
                break;

            default:
                if( !p->cur.p_data[0] ) /* store current decoded frame */
                {
                    break;
                }

                if( p->ph.i_picture_coding_type == CAVS_B_PICTURE )
                {
                    if( !p->p_save[1] || !p->p_save[2] ) /* need L0 and L1 reference frame */
                    {
                        break;
                    }
                }
                
                if( p->ph.i_picture_coding_type == CAVS_P_PICTURE )
                {
                    if( !p->p_save[1] ) /* need reference frame */
                    {
                        break;
                    }
                }

                if( cur_startcode >= CAVS_SLICE_MIN_START_CODE 
                    && cur_startcode <= CAVS_SLICE_MAX_START_CODE )
                {
                    int i_next_mb_y;

                    cavs_bitstream_init(&p->s, p_buf, i_len);                        
                    p->i_mb_y = cur_startcode & 0x000000FF;                       
                        
                    if( frame->slice_num != 0 )
                        i_next_mb_y = frame->slice[slice_count].i_startcode & 0x000000FF; /*slice_count++ before*/
                    else
                        i_next_mb_y = 0xFF;
                    if (i_next_mb_y>=0 && i_next_mb_y<=0xAF)
                        p->i_slice_mb_end = i_next_mb_y * p->i_mb_width;
                    else
                        p->i_slice_mb_end = p->i_mb_num;
                        
                    if( cavs_get_slice(p) != 0 )
                    {
                        p->b_complete = 1; /* set for pic exchange */
                        //return CAVS_ERROR;
                    }

                    /* this module will effect ref-list creature and update */
                    if(p->b_complete && p->ph.i_picture_coding_type != CAVS_B_PICTURE)
                    {
                        cavs_image *p_image = p->p_save[0];
                        if(p->vsh.b_low_delay)
                        {
                            i_result = CAVS_FRAME_OUT;

                            /* set for ffmpeg interface */
                            param->output_type = p->p_save[0]->i_code_type;
                                  
                            /* when b_low_delay is 1, output current frame when finish decoded current */
                            cavs_out_image_yuv420(p, p->p_save[0], param->p_out_yuv, 0);
                        }
                        if(!p->p_save[1])
                        {
                            p->p_save[0] = &p->image[1];
                        }
                        else
                        {
                            if( p->vsh.b_low_delay == 0 /*&& p->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                            {
                                i_result = CAVS_FRAME_OUT;

                                /* set for ffmpeg interface */
                                param->output_type = p->p_save[1]->i_code_type;
                                    
                                /* when b_low_delay is 0, current frame should delay one frame to output */
                                cavs_out_image_yuv420(p, p->p_save[1], param->p_out_yuv, 0);
                            }

                            if(!p->p_save[2])
                            {
                                p->p_save[0] = &p->image[2];	 
                            }
                            else
                            {
                                p->p_save[0] = p->p_save[2];
                            }
                        }
                        p->p_save[2] = p->p_save[1];
                        p->p_save[1] = p_image;
                        memset(&p->cur, 0, sizeof(cavs_image));
                        p->b_complete = 0;
                    }
                    else if(p->b_complete)
                    {
                        i_result = CAVS_FRAME_OUT;

                        /* set for ffmpeg interface */
                        param->output_type = p->p_save[0]->i_code_type;
                              
                        /* ouput B-frame no delay */
                        cavs_out_image_yuv420(p, p->p_save[0], param->p_out_yuv, 0);
                        memset(&p->cur,0,sizeof(cavs_image));
                        p->b_complete=0;
                    }
                }
                else
                {
                    return CAVS_ERROR;
                }
        }
    }

    if( p->b_error_flag )
    {
    	i_result = CAVS_ERROR;
    }

    return i_result;
}

static int frame_decode_aec_threads_far( cavs_decoder *p_aec )
{
    cavs_decoder *p_m = p_aec->p_m;
    frame_pack *frame = p_aec->fld;
    uint8_t   *p_buf;
    int    i_len, i_result = 0;
    int slice_count = p_aec->slice_count;
    uint32_t cur_startcode;
    uint32_t i_ret = 0;

    /* pack  */
    i_ret = cavs_frame_pack( p_aec->p_stream, frame );
    if( i_ret == CAVS_ERROR )
    {
        p_aec->b_error_flag = 1;
        return CAVS_ERROR;
    }
    
    cur_startcode = frame->slice[0].i_startcode;
    p_buf = frame->slice[0].p_start;
    i_len = frame->slice[0].i_len;
    frame->slice_num--;
    
    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            cavs_bitstream_init(&p_aec->s, p_buf, i_len);
            cavs_get_i_picture_header(&p_aec->s, &p_aec->ph, &p_aec->vsh);
            
			if( p_aec->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{	
				//p_aec->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p_aec->ph.b_aec_enable = p_m->vsh.b_aec_enable;
			}
			else if(p_aec->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p_aec->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p_aec->b_error_flag  = 1;

                return CAVS_ERROR;
			}
			p_aec->ph.b_picture_structure = 1; /* NOTE : guarantee frame not change */

            i_ret = cavs_init_picture_context(p_aec);
            if(i_ret == CAVS_ERROR)
            {
                p_aec->b_error_flag  = 1;
                return CAVS_ERROR;
            }
            cavs_init_picture_ref_list_for_aec(p_aec);

            break;

        case CAVS_PB_PICUTRE_START_CODE:
            cavs_bitstream_init(&p_aec->s, p_buf, i_len);
            cavs_get_pb_picture_header(&p_aec->s, &p_aec->ph, &p_aec->vsh);

			if( p_aec->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{	
				//p_aec->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p_aec->ph.b_aec_enable = p_m->vsh.b_aec_enable;
			}
			else if(p_aec->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p_aec->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p_aec->b_error_flag  = 1;

                return CAVS_ERROR;
			}
			p_aec->ph.b_picture_structure = 1; /* NOTE : guarantee frame not change */

            i_ret = cavs_init_picture_context(p_aec);
            if( i_ret == CAVS_ERROR )
            {
                p_aec->b_error_flag  = 1;
                return CAVS_ERROR;
            }
            cavs_init_picture_ref_list_for_aec(p_aec);
         
            break;

        default:
            return CAVS_ERROR;
    }	
    
    /* weighting quant */
    cavs_init_frame_quant_param( p_aec );

    cavs_frame_update_wq_matrix( p_aec );
    
    while( frame->slice_num--)
    {
        cur_startcode = frame->slice[slice_count].i_startcode;
        p_buf = frame->slice[slice_count].p_start;
        i_len = frame->slice[slice_count].i_len;
        slice_count++;

        switch( cur_startcode )
        {
            case CAVS_USER_DATA_CODE:
                cavs_bitstream_init(&p_aec->s,p_buf,i_len);
                i_result = cavs_get_user_data(&p_aec->s, &p_aec->user_data);
                break;

            case CAVS_EXTENSION_START_CODE:
                cavs_bitstream_init(&p_aec->s,p_buf,i_len);
                i_result = cavs_get_extension(&p_aec->s, p_aec);
                break;

            default:

                if( cur_startcode >= CAVS_SLICE_MIN_START_CODE 
                    && cur_startcode <= CAVS_SLICE_MAX_START_CODE )
                {
                    int i_next_mb_y;

                    cavs_bitstream_init(&p_aec->s, p_buf, i_len); /* init handle */                        
                    p_aec->i_mb_y = cur_startcode & 0x000000FF;                       
                        
                    if( frame->slice_num != 0 )
                        i_next_mb_y = frame->slice[slice_count].i_startcode & 0x000000FF; /*slice_count++ before*/
                    else
                        i_next_mb_y = 0xFF;
                    if (i_next_mb_y>=0 && i_next_mb_y<=0xAF)
                        p_aec->i_slice_mb_end = i_next_mb_y * p_aec->i_mb_width;
                    else
                        p_aec->i_slice_mb_end = p_aec->i_mb_num;
                    //p_aec->b_bottom = 0;
                    //p_aec->i_slice_mb_end_fld[p_aec->b_bottom] =  p_aec->i_slice_mb_end;
                        
                    /* AEC */
                    cavs_get_slice_aec(p_aec);
                    if(p_aec->b_error_flag == 1)
                        return  CAVS_ERROR;

                    /* creat independent ref-list for AEC module */
                    if( p_aec->ph.i_picture_coding_type != CAVS_B_PICTURE )
                    {
                        cavs_image *p_image = p_aec->p_save_aec[0];

                        if(!p_aec->p_save_aec[1])
                        {
                            p_aec->p_save_aec[0] = &p_aec->image_aec[1];
                        }
                        else
                        {
                            if(!p_aec->p_save_aec[2])
                            {
                                p_aec->p_save_aec[0] = &p_aec->image_aec[2];	 
                            }
                            else
                            {
                                p_aec->p_save_aec[0] = p_aec->p_save_aec[2];
                            }
                        }
                        p_aec->p_save_aec[2] = p_aec->p_save_aec[1];
                        p_aec->p_save_aec[1] = p_image;
                        memset(&p_aec->cur_aec, 0, sizeof(cavs_image));
                    }
                }
                else
                {
                    p_aec->b_error_flag = 1;

                    return CAVS_ERROR;
                }
        }
    }

	return 0;
}

static int decode_one_frame_far(void *p_decoder, frame_pack *frame, cavs_param *param )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    cavs_decoder *p_m = p;/* backup main threads */
    cavs_decoder *p_aec, *p_rec;

    int i_result=0;
    int slice_count = 1; /* 0 is for pic header */

    /* FIXIT : should init handle here */
    p_aec = p_m->unused[0];
    p = p_aec; /* switch to aec threads */

    /* decode slice */
    p_aec->p_m = p_m;
    p_aec->fld = frame;
    p_aec->slice_count = slice_count;
    cavs_threadpool_run( p_m->threadpool, (void*)frame_decode_aec_threads_far, p_aec );
    p_aec->b_thread_active = 1;

    /* handle switch */
    p_m->unused[0] = p_m->unused[1];
    p_m->unused[1] = p_aec;

    p_m->current[0] = p_m->current[1];
    p_m->current[1] = p_aec;

    /* REC */
    /*init REC handle */
    p_rec = p_m->current[0];                        
    if( p_rec != NULL )
    {                 
        cavs_init_picture_context_frame(p_rec);
        i_result = cavs_get_slice_rec_threads( p_rec );
        if( i_result == -1 )
        {
            i_result = CAVS_ERROR;

            /* close AEC at same time */
            if( p_aec->b_thread_active == 1 )
            {
                if(cavs_threadpool_wait( p_m->threadpool, p_aec))
                    ;//return -1;
     
                p_aec->b_thread_active = 0;
            }
            return i_result;
        }

        if( p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE )
            p_m->last_delayed_pframe = p_rec->p_save[1];
       
        /* this module will effect ref-list creature and update */
        if(p_rec->b_complete && p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE)
        {
            cavs_image *p_image = p_rec->p_save[0];
            if(p_rec->vsh.b_low_delay)
            {
                i_result = CAVS_FRAME_OUT;

                /* set for ffmpeg interface */
                param->output_type = p_rec->p_save[0]->i_code_type;

                /* when b_low_delay is 1, output current frame when finish decoded current */
                cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv, 0);
            }
            if(!p_rec->p_save[1])
            {
                p_rec->p_save[0] = &p_rec->image[1];
            }
            else
            {
                if( p_rec->vsh.b_low_delay == 0 /*&& p_rec->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                {
                    i_result = CAVS_FRAME_OUT;

                    /* set for ffmpeg interface */
                    param->output_type = p_rec->p_save[1]->i_code_type;
                    
                    /* when b_low_delay is 0, current frame should delay one frame to output */
                    cavs_out_image_yuv420(p_m, p_rec->p_save[1], param->p_out_yuv, 0);
                }
                else
                {
                	param->output_type = -1;
                }

                if(!p_rec->p_save[2])
                {
                    p_rec->p_save[0] = &p_rec->image[2];	 
                }
                else
                {
                    p_rec->p_save[0] = p_rec->p_save[2];
                }
            }
            p_rec->p_save[2] = p_rec->p_save[1];
            p_rec->p_save[1] = p_image;
            memset(&p_rec->cur, 0, sizeof(cavs_image));
            p_rec->b_complete = 0;

            /* copy ref-list to anther threads */
            p_m->current[1]->p_save[0] = p_rec->p_save[0];
            p_m->current[1]->p_save[1] = p_rec->p_save[1];
            p_m->current[1]->p_save[2] = p_rec->p_save[2];
        }
        else if(p_rec->b_complete)
        {
            i_result = CAVS_FRAME_OUT;

            /* set for ffmpeg interface */
            param->output_type = p_rec->p_save[0]->i_code_type;
            
            /* ouput B-frame no delay */
            cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv, 0);
            memset(&p_rec->cur,0,sizeof(cavs_image));
            p_rec->b_complete=0;
        }
    }

    /* waiting for end of AEC threads */
    if( p_aec->b_thread_active == 1 )
    {  
       if(cavs_threadpool_wait( p_m->threadpool, p_aec))
           ;//return -1;
        p_aec->b_thread_active = 0;
    }
    
    if( p_aec->b_error_flag )
    {
        return CAVS_ERROR;
    }

    p_m->unused[0]->p_save_aec[0] = p_aec->p_save_aec[0];/* should proc like field */
    p_m->unused[0]->p_save_aec[1] = p_aec->p_save_aec[1];
    p_m->unused[0]->p_save_aec[2] = p_aec->p_save_aec[2];
    
    /* copy col type to next handle */
    memcpy( p_m->unused[0]->p_col_mv, p_aec->p_col_mv,p_m->i_mb_width*(p_m->i_mb_height)*4*sizeof(cavs_vector));
    memcpy( p_m->unused[0]->p_col_type_base, p_aec->p_col_type_base, p_m->i_mb_width*(p_m->i_mb_height));

    if( p->b_error_flag )
    {
    	i_result = CAVS_ERROR;
    }

    return i_result;
}


/* cavs_topfield_pack */
uint32_t cavs_topfield_pack( InputStream*p, frame_pack *field )
{
    uint32_t i_startcode;
    int i_len;
    //unsigned char *m_buf;

    //m_buf = (unsigned char *)cavs_malloc((MAX_CODED_FRAME_SIZE/2)*sizeof(unsigned char));
    //memset(m_buf, 0, (MAX_CODED_FRAME_SIZE/2)*sizeof(unsigned char));

    i_startcode = field->slice[0].i_startcode ; /* pic header */

    while( i_startcode )
    {  
        cavs_bitstream_save( field, p );
        i_startcode = cavs_get_one_nal( p, field->m_buf, &i_len );

        if( ((i_startcode & 0xff) > (uint32_t)field->i_mb_end[0])
			&& i_startcode != CAVS_USER_DATA_CODE
			&& i_startcode != CAVS_EXTENSION_START_CODE ) /* next slice start mb address exceed field limit */
        {
            cavs_bitstream_restore( p, field );
            //cavs_free(m_buf);
			 
            return i_startcode;
        }

        switch( i_startcode )
        {
            case CAVS_USER_DATA_CODE:
                break;
            case CAVS_EXTENSION_START_CODE:
                break;
            default:
                memcpy(field->p_cur, field->m_buf, i_len ); /* pack slice into one frame */

                /* set slice info */
                field->slice[field->slice_num].i_startcode = i_startcode;
                field->slice[field->slice_num].p_start = field->p_cur + 4; /* for skip current startcode */
                field->slice[field->slice_num].i_len = i_len-4;
        
                /*update frame info */
                field->p_cur = field->p_cur + i_len;
                field->i_len += i_len;
                field->slice_num++;               
                break;
    	 }
    }

    return 0;
}

/* decode_top_field */
int decode_top_field(void *p_decoder, frame_pack *field, cavs_param *param )
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;
    uint32_t  cur_startcode;
    uint8_t   *p_buf;
    int    i_len, i_result=0;
    int sliceNum, threadNum;
    int ret = 0;
    int slice_count = 1; /* 0 is for pic header */
    //int b_interlaced = p->param.b_interlaced;
    
    p->b_bottom = field->b_bottom;
    sliceNum = field->slice_num - 1; /* exclude pic header */
    threadNum = 0;

    cur_startcode = field->slice[0].i_startcode;
    p_buf = field->slice[0].p_start;
    i_len = field->slice[0].i_len;
    field->slice_num--;

    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf, i_len);
            cavs_get_i_picture_header(&p->s,&p->ph,&p->vsh);
			if( p->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{	
				//p->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p->ph.b_aec_enable = p->vsh.b_aec_enable;
			}
			else if(p->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p->b_error_flag  = 1;

                return CAVS_ERROR;
			}
			p->ph.b_picture_structure = 0; /* NOTE : guarantee field not change */
            cavs_init_picture(p);

            /* add for no-seq header before I-frame */
            p->last_delayed_pframe = p->p_save[1];
            break;

        case CAVS_PB_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf, i_len);
            cavs_get_pb_picture_header(&p->s,&p->ph,&p->vsh);
			if( p->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{	
				//p->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p->ph.b_aec_enable = p->vsh.b_aec_enable;
			}
			else if(p->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p->b_error_flag  = 1;

                return CAVS_ERROR;
			}
			p->ph.b_picture_structure = 0; /* NOTE : guarantee field not change */
            cavs_init_picture(p);
            break;

        default:
            return CAVS_ERROR;
    }	
    
    /*add for weighting quant */
    cavs_init_frame_quant_param( p );

    cavs_frame_update_wq_matrix( p );

    /* decode slice */
    while( field->slice_num--)
    {
        cur_startcode = field->slice[slice_count].i_startcode;
        p_buf = field->slice[slice_count].p_start;
        i_len = field->slice[slice_count].i_len;
        slice_count++;

        switch( cur_startcode )
        {
            case CAVS_USER_DATA_CODE:
                cavs_bitstream_init(&p->s,p_buf,i_len);
                i_result = cavs_get_user_data(&p->s, &p->user_data);
                break;

            case CAVS_EXTENSION_START_CODE:
                cavs_bitstream_init(&p->s,p_buf,i_len);
                i_result = cavs_get_extension(&p->s, p);
                break;

            default:
                if( !p->cur.p_data[0] ) /* store current decoded frame */
                {
                    break;
                }

                if( p->ph.i_picture_coding_type == CAVS_B_PICTURE )
                {
                    if( !p->p_save[1] || !p->p_save[2] ) /* need L0 and L1 reference frame */
                    {
                        break;
                    }
                }
                
                if( p->ph.i_picture_coding_type == CAVS_P_PICTURE )
                {
                    if( !p->p_save[1] ) /* need reference frame */
                    {
                        break;
                    }
                }

                if( cur_startcode >= CAVS_SLICE_MIN_START_CODE 
                    && cur_startcode <= CAVS_SLICE_MAX_START_CODE )
                {
                    int i_next_mb_y;

                    p->b_error_flag = 0;
                    cavs_bitstream_init(&p->s, p_buf, i_len);                        
                    p->i_mb_y = cur_startcode & 0x000000FF;                       
                        
                    if( field->slice_num != 0 )
                        i_next_mb_y = field->slice[slice_count].i_startcode & 0x000000FF; /*slice_count++ before*/
                    else
                        i_next_mb_y = 0xFF;

                    if((uint32_t)i_next_mb_y > (p->i_mb_height>>1)
                        && field->slice_num != 0 )
                    {
                        p->b_error_flag = 1;
                    }
                        
                    if (i_next_mb_y>=0 && i_next_mb_y<=0xAF)
                        p->i_slice_mb_end = i_next_mb_y * p->i_mb_width;
                    else
                    {
                        p->i_slice_mb_end = (p->i_mb_num>>1);
                    }

                    if( ! p->b_error_flag )
                    {
                        if( cavs_get_slice(p) != 0 )
                        {
                            p->b_complete = 1; /* set for pic exchange */
                            //return CAVS_ERROR;
                        }
                    }
                    else
                        p->b_complete = 1; /* set for pic exchange */

                    if(p->b_complete && p->ph.i_picture_coding_type != CAVS_B_PICTURE)
                    {
                        //cavs_image *p_image = p->p_save[0];
                        if(p->vsh.b_low_delay)
                        {
                            i_result = CAVS_FRAME_OUT;

                            /* set for ffmpeg interface */
                            param->output_type = p->p_save[0]->i_code_type;
                                  
                            cavs_out_image_yuv420(p, p->p_save[0], param->p_out_yuv, 0);
                        }
                        if(!p->p_save[1])
                        {
                            //p->p_save[0] = &p->image[1];
                        }
                        else
                        {
                            if(p->vsh.b_low_delay == 0 /*&& p->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                            {
                                i_result = CAVS_FRAME_OUT;

                                /* set for ffmpeg interface */
                                param->output_type = p->p_save[1]->i_code_type;
                          
                                cavs_out_image_yuv420(p, p->p_save[1], param->p_out_yuv, 0);
                            }

                            //if(!p->p_save[2])
                            //{
                                //p->p_save[0] = &p->image[2];	 
                            //}
                            //else
                            //{
                                // p->p_save[0] = p->p_save[2];
                            //}
                        }
                        //p->p_save[2] = p->p_save[1];
                        //p->p_save[1] = p_image;
                        //memset(&p->cur, 0, sizeof(cavs_image));
                        p->b_complete = 0;
                    }
                    else if( p->b_complete )
                    {
                        i_result = CAVS_FRAME_OUT;

                        /* set for ffmpeg interface */
                        param->output_type =  p->p_save[0]->i_code_type;
                              
                        cavs_out_image_yuv420(p, p->p_save[0], param->p_out_yuv, 0);
                        memset(&p->cur,0,sizeof(cavs_image));
                        p->b_complete=0;
                    }	
                }
                else
                {
                    return CAVS_ERROR;
                }
        }
    }

	if( p->b_error_flag )
	{
		i_result = CAVS_ERROR;
	}

    return i_result;
}

/* cavs_botfield_pack */
uint32_t cavs_botfield_pack( InputStream*p, frame_pack *field )
{
    uint32_t i_startcode;
    int i_len;
    //unsigned char *m_buf;

    //m_buf = (unsigned char *)cavs_malloc((MAX_CODED_FRAME_SIZE/2)*sizeof(unsigned char));
    //memset(m_buf, 0, (MAX_CODED_FRAME_SIZE/2)*sizeof(unsigned char));

    i_startcode = 1; /* use to start loop */
    while( i_startcode )
    {  
        cavs_bitstream_save( field, p );
        i_startcode = cavs_get_one_nal( p, field->m_buf, &i_len );

        switch( i_startcode )
        {	
            case CAVS_VIDEO_SEQUENCE_START_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            case CAVS_VIDEO_SEQUENCE_END_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            case CAVS_VIDEO_EDIT_CODE:
                cavs_bitstream_restore( p, field );
				//cavs_free(m_buf);
                return i_startcode;
            case CAVS_I_PICUTRE_START_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            case CAVS_PB_PICUTRE_START_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;

            case CAVS_USER_DATA_CODE:
                break;
            case CAVS_EXTENSION_START_CODE:
                break;
            case 0x000001fe: // FIXIT
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            default:
                memcpy(field->p_cur, field->m_buf, i_len ); /* pack slice into one frame */

                /* set slice info */
                field->slice[field->slice_num].i_startcode = i_startcode;
                field->slice[field->slice_num].p_start = field->p_cur + 4; /* for skip current startcode */
                field->slice[field->slice_num].i_len = i_len-4;
        
                /*update frame info */
                field->p_cur = field->p_cur + i_len;
                field->i_len += i_len;
                field->slice_num++;               
                break;
    	 }
    }

    //cavs_free(m_buf);

    return 0;
}

/* decode_bot_field */
int decode_bot_field(void *p_decoder, frame_pack *field, cavs_param *param )
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;
    uint32_t  cur_startcode;
    uint8_t   *p_buf;
    int    i_len, i_result=0;
    int sliceNum, threadNum;
    int ret = 0;
    int slice_count = 0; /* bot pack has no pic header */
    //int b_interlaced = p->param.b_interlaced;

    p->b_bottom = field->b_bottom;
    sliceNum = field->slice_num; /* bot pack has no pic header */
    threadNum = 0;

    cur_startcode = field->i_startcode_type;

    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            //cavs_bitstream_init(&p->s, p_buf, i_len);
            //cavs_get_i_picture_header(&p->s,&p->ph,&p->vsh);
            cavs_init_picture(p);
            break;

        case CAVS_PB_PICUTRE_START_CODE:
            //cavs_bitstream_init(&p->s, p_buf, i_len);
            //cavs_get_pb_picture_header(&p->s,&p->ph,&p->vsh);
            cavs_init_picture(p);
            break;

        default:
            return CAVS_ERROR;
    }	
    
    /* decode slice */
    while( field->slice_num--)
    {
        cur_startcode = field->slice[slice_count].i_startcode;
        p_buf = field->slice[slice_count].p_start;
        i_len = field->slice[slice_count].i_len;
        slice_count++;

        switch( cur_startcode )
        {
            case CAVS_USER_DATA_CODE:
                cavs_bitstream_init(&p->s,p_buf,i_len);
                i_result = cavs_get_user_data(&p->s, &p->user_data);
                break;

            case CAVS_EXTENSION_START_CODE:
                cavs_bitstream_init(&p->s,p_buf,i_len);
                i_result = cavs_get_extension(&p->s, p);
                break;

            default:
                if( !p->cur.p_data[0] ) /* store current decoded frame */
                {
                    break;
                }

                if( p->ph.i_picture_coding_type == CAVS_B_PICTURE )
                {
                    if( !p->p_save[1] || !p->p_save[2] ) /* need L0 and L1 reference frame */
                    {
                        break;
                    }
                }
                
                if( p->ph.i_picture_coding_type == CAVS_P_PICTURE )
                {
                    if( !p->p_save[1] ) /* need reference frame */
                    {
                        break;
                    }
                }

                if( cur_startcode >= CAVS_SLICE_MIN_START_CODE 
                    && cur_startcode <= CAVS_SLICE_MAX_START_CODE )
                {
                    int i_next_mb_y;

                    p->b_error_flag = 0;
                    cavs_bitstream_init(&p->s, p_buf, i_len);                        
                    p->i_mb_y = cur_startcode & 0x000000FF;                       
                        
                    if( field->slice_num != 0 )
                        i_next_mb_y = field->slice[slice_count].i_startcode & 0x000000FF; /*slice_count++ before*/
                    else
                        i_next_mb_y = 0xFF;

                        
                    if((uint32_t)i_next_mb_y < (p->i_mb_height>>1))
                    {
                        p->b_error_flag = 1;
                    }
                        
                    if (i_next_mb_y>=0 && i_next_mb_y<=0xAF)
                        p->i_slice_mb_end = i_next_mb_y * p->i_mb_width;
                    else
                    {  
                        p->i_slice_mb_end = p->i_mb_num;
                    }                 

                    if( !p->b_error_flag )
                    {
                        if( cavs_get_slice(p) != 0 )
                        {
                            p->b_complete = 1; /* set for pic exchange */
                        }
                    }
                    else
                            p->b_complete = 1; /* set for pic exchange */
                        
                    if(p->b_complete && p->ph.i_picture_coding_type != CAVS_B_PICTURE)
                    {
                        cavs_image *p_image = p->p_save[0];
                        if( p->vsh.b_low_delay )
                        {
                            i_result = CAVS_FRAME_OUT;

                            /* set for ffmpeg interface */
                            param->output_type = p->p_save[0]->i_code_type;
                                
                            cavs_out_image_yuv420(p, p->p_save[0], param->p_out_yuv, 1);
                        }
                        if(!p->p_save[1])
                        {
                            p->p_save[0] = &p->image[1];
                        }
                        else
                        {
                            if(p->vsh.b_low_delay == 0 /*&& p->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                            {
                                i_result = CAVS_FRAME_OUT;

                                /* set for ffmpeg interface */
                                param->output_type = p->p_save[1]->i_code_type;
                                                                           
                                cavs_out_image_yuv420(p, p->p_save[1], param->p_out_yuv, 1);
                            }

                            if(!p->p_save[2])
                            {
                                p->p_save[0] = &p->image[2];	 
                            }
                            else
                            {
                                p->p_save[0] = p->p_save[2];
                            }
                        }
                        p->p_save[2] = p->p_save[1];
                        p->p_save[1] = p_image;
                        memset(&p->cur, 0, sizeof(cavs_image));
                        p->b_complete = 0;
                    }
                    else if(p->b_complete)
                    {
                        i_result = CAVS_FRAME_OUT;

                        /* set for ffmpeg interface */
                        param->output_type = p->p_save[0]->i_code_type;
                            
                        cavs_out_image_yuv420(p, p->p_save[0], param->p_out_yuv, 1);
                        memset(&p->cur,0,sizeof(cavs_image));
                        p->b_complete=0;
                    }	
                }
                else
                {
                    return CAVS_ERROR;
                }
        }
    }
	
    if( p->b_error_flag )
    {
        i_result = CAVS_ERROR;
    }

    return i_result;
}

/* pack top-field and bot-field into one frame */
static uint32_t cavs_field_pack( InputStream*p, frame_pack *field )
{
    uint32_t i_startcode, pre_startcode = 0;
    int i_len;
    //unsigned char *m_buf;

    //m_buf = (unsigned char *)cavs_malloc((MAX_CODED_FRAME_SIZE)*sizeof(unsigned char));
    //memset(m_buf, 0, (MAX_CODED_FRAME_SIZE)*sizeof(unsigned char));

    i_startcode = 1; /* use to start loop */
    while( i_startcode )
    {  
        cavs_bitstream_save( field, p );
        i_startcode = cavs_get_one_nal( p, field->m_buf, &i_len );
		
        switch( i_startcode )
        {	
            case CAVS_VIDEO_SEQUENCE_START_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            case CAVS_VIDEO_SEQUENCE_END_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            case CAVS_VIDEO_EDIT_CODE:
                cavs_bitstream_restore( p, field );
				//cavs_free(m_buf);
                return i_startcode;
            case CAVS_I_PICUTRE_START_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            case CAVS_PB_PICUTRE_START_CODE:
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;

            case CAVS_USER_DATA_CODE:
                break;
            case CAVS_EXTENSION_START_CODE:
                break;
            case 0x000001fe: // FIXIT
                cavs_bitstream_restore( p, field );
                //cavs_free(m_buf);
                return i_startcode;
            default:
				if( i_startcode == pre_startcode ) /* NOTE : repeat slice should discard */
					break;
				if ((i_startcode & 0xff) > (uint32_t)field->i_mb_end[1]) 
				{
					cavs_bitstream_restore( p, field );
					//cavs_free(m_buf);

					return i_startcode;
				}
				
                memcpy(field->p_cur, field->m_buf, i_len ); /* pack slice into one frame */

                /* set slice info */
                field->slice[field->slice_num].i_startcode = i_startcode;
                field->slice[field->slice_num].p_start = field->p_cur + 4; /* for skip current startcode */
                field->slice[field->slice_num].i_len = i_len-4;
        
                /*update frame info */
                field->p_cur = field->p_cur + i_len;
                field->i_len += i_len;
                field->slice_num++;               
                break;
    	 }

		pre_startcode = i_startcode;
    }

    //cavs_free(m_buf);

    return 0;
}

static int field_decode_aec_threads( cavs_decoder *p_aec )
{
    //cavs_decoder *p_m = p_aec->p_m;
    frame_pack *field = p_aec->fld;
    uint8_t   *p_buf;
    int    i_len, i_result = 0;
    int slice_count = p_aec->slice_count;
    int field_count = p_aec->field_count;
    uint32_t  cur_startcode;
    uint32_t i_ret =0;

    while( field->slice_num--)
    {
        cur_startcode = field->slice[slice_count].i_startcode;
        p_buf = field->slice[slice_count].p_start;
        i_len = field->slice[slice_count].i_len;
        slice_count++;
        field_count++;
        
        switch( cur_startcode )
        {
            case CAVS_USER_DATA_CODE:
                cavs_bitstream_init(&p_aec->s,p_buf,i_len);
                i_result = cavs_get_user_data(&p_aec->s, &p_aec->user_data);
                break;

            case CAVS_EXTENSION_START_CODE:
                cavs_bitstream_init(&p_aec->s,p_buf,i_len);
                i_result = cavs_get_extension(&p_aec->s, p_aec);
                break;

            default:

                if( cur_startcode >= CAVS_SLICE_MIN_START_CODE 
                    && cur_startcode <= CAVS_SLICE_MAX_START_CODE )
                {
                    int i_next_mb_y;
                    uint32_t i_mb_y_ori = 0;

                    cavs_bitstream_init(&p_aec->s, p_buf, i_len);                        
                    p_aec->i_mb_y = cur_startcode & 0x000000FF;
                    i_mb_y_ori = p_aec->i_mb_y;                    
                        
                    /* set b_bottom */
                    p_aec->b_bottom = 0;
                    if( p_aec->i_mb_y >= (p_aec->i_mb_height>>1))
                    {
                        p_aec->b_bottom = 1;
                            
                        i_ret = cavs_init_picture_context(p_aec);
                        if( i_ret == CAVS_ERROR )
                        {
                            p_aec->b_error_flag  = 1;
                            return CAVS_ERROR;
                        }
                        cavs_init_picture_ref_list_for_aec(p_aec);
                    }
                    p_aec->i_mb_y = i_mb_y_ori;
             
                    if( field->slice_num != 0 )
                        i_next_mb_y = field->slice[slice_count].i_startcode & 0x000000FF; /*slice_count++ before*/
                    else
                        i_next_mb_y = 0xFF;

                    if( !p_aec->b_bottom )
                    {
                        if((uint32_t)i_next_mb_y<(p_aec->i_mb_height>>1))
                        {
                            p_aec->b_error_flag = 1;
                            return  CAVS_ERROR;
                        }
                    }                       
                        
                    if (i_next_mb_y>=0 && i_next_mb_y<=0xAF)
                        p_aec->i_slice_mb_end = i_next_mb_y * p_aec->i_mb_width;
                    else
                    {
                        p_aec->i_slice_mb_end = (p_aec->i_mb_num>>(!p_aec->b_bottom));
                    }
                    p_aec->i_slice_mb_end_fld[p_aec->b_bottom] =  p_aec->i_slice_mb_end;
                        
                    cavs_get_slice_aec(p_aec);
                    if(p_aec->b_error_flag == 1)
                        return  CAVS_ERROR;

                    /* creat independent ref-list for AEC module */
                    if( p_aec->ph.i_picture_coding_type != CAVS_B_PICTURE )
                    {
                        cavs_image *p_image = p_aec->p_save_aec[0];

                        if(!p_aec->p_save_aec[1])
                        {
                            if( p_aec->b_bottom )
                                p_aec->p_save_aec[0] = &p_aec->image_aec[1];
                        }
                        else
                        {
                            if( p_aec->b_bottom )
                            {
                                if(!p_aec->p_save_aec[2])
                                {
                                    p_aec->p_save_aec[0] = &p_aec->image_aec[2];	 
                                }
                                else
                                {
                                    p_aec->p_save_aec[0] = p_aec->p_save_aec[2];
                                }
                            }
                        }

                        if( p_aec->b_bottom )
                        {
                            p_aec->p_save_aec[2] = p_aec->p_save_aec[1];
                            p_aec->p_save_aec[1] = p_image;
                            memset(&p_aec->cur_aec, 0, sizeof(cavs_image));
                        }
                    }
                }
                else
                {
                    return CAVS_ERROR;
                }
        }
    }

    return 0;
}


/* decode top-field and bot-field orderly in one function */
static int cavs_field_decode(void *p_decoder, frame_pack *field, cavs_param *param )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    cavs_decoder *p_m = p;/* backup main threads */
    cavs_decoder *p_aec, *p_rec;
    int field_count = 0;
    
    uint32_t  cur_startcode;
    uint8_t   *p_buf;
    int    i_len, i_ret = 0, i_result=0;
    int sliceNum, threadNum;
    int slice_count = 1; /* 0 is for pic header */
    
    sliceNum = field->slice_num - 1; /* exclude pic header */
    threadNum = 0;

    cur_startcode = field->slice[0].i_startcode;
    p_buf = field->slice[0].p_start;
    i_len = field->slice[0].i_len;
    field->slice_num--;

    /* init aec handle */
    p_aec = p_m->unused[0];
    p = p_aec;
    p_aec->b_bottom = 0; /* init bottom flag */
    p_aec->i_frame_decoded = p_m->i_frame_decoded; //just for debug

    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf, i_len);
            cavs_get_i_picture_header(&p->s,&p->ph,&p->vsh);
			if( p->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{
				//p->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p->ph.b_aec_enable = p->vsh.b_aec_enable;
			}
			else if(p->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p->ph.b_aec_enable = 0; /* AVS */				
			}
			else
			{
				p->b_error_flag  = 1;

                return CAVS_ERROR;
			}
			p->ph.b_picture_structure = 0; /* NOTE : guarantee field not change */
            i_ret = cavs_init_picture_context(p);
            if( i_ret == CAVS_ERROR )
            {
                p->b_error_flag  = 1;
                return CAVS_ERROR;
            }
            cavs_init_picture_ref_list_for_aec(p);

            break;

        case CAVS_PB_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf, i_len);
            cavs_get_pb_picture_header(&p->s,&p->ph,&p->vsh);
			if( p->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{
				//p->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p->ph.b_aec_enable = p->vsh.b_aec_enable;
			}
			else if(p->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p->b_error_flag  = 1;

                return CAVS_ERROR;
			}
            p->ph.b_picture_structure = 0; /* NOTE : guarantee field not change */
            i_ret = cavs_init_picture_context(p);
            if( i_ret == CAVS_ERROR )
            {
                p->b_error_flag  = 1;
                return CAVS_ERROR;
            }
            cavs_init_picture_ref_list_for_aec(p);
            break;

        default:
            return CAVS_ERROR;
    }	
    
    /*add for weighting quant */
    cavs_init_frame_quant_param( p );

    cavs_frame_update_wq_matrix( p );

    p_aec->p_m = p_m;
    p_aec->fld = field;
    p_aec->slice_count = slice_count;
    p_aec->field_count = field_count;

    cavs_threadpool_run( p_m->threadpool, (void*)field_decode_aec_threads, p_aec );
    p_aec->b_thread_active = 1;

    p_m->unused[0] = p_m->unused[1];
    p_m->unused[1] = p_aec;
    
    p_m->current[0] = p_m->current[1];
    p_m->current[1] = p_aec;



    /* rec decode loop */
    p_rec = p_m->current[0];
    field_count =  2; /* only support two field */
    if( p_rec != NULL )
    {
        while( field_count-- )
        {
            p_rec->b_bottom = !field_count; /* first-top second-botttom */

            cavs_init_picture_context_fld( p_rec );
            cavs_get_slice_rec_threads( p_rec );
            
            if( p_rec->b_bottom )
            {
                if( p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE )
                    p_m->last_delayed_pframe = p_rec->p_save[1];
            }
            
            if(p_rec->b_complete && p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE)
            {
                cavs_image *p_image = p_rec->p_save[0];
                if(p_rec->vsh.b_low_delay)
                {
                    i_result = CAVS_FRAME_OUT;

                    /* set for ffmpeg interface */
                    param->output_type = p_rec->p_save[0]->i_code_type;
                    
                    cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv,  p_rec->b_bottom);
                }
                if(!p_rec->p_save[1])
                {
                    if( p_rec->b_bottom )
                        p_rec->p_save[0] = &p_rec->image[1];
                }
                else
                {
                    if(p_rec->vsh.b_low_delay == 0 /*&& p_rec->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                    {
                        i_result = CAVS_FRAME_OUT;

                        /* set for ffmpeg interface */
                        param->output_type = p_rec->p_save[1]->i_code_type;
                          
                        cavs_out_image_yuv420(p_m, p_rec->p_save[1], param->p_out_yuv,  p_rec->b_bottom);
                    }
                    else
                    {
                    	param->output_type = -1;
                    }
   
                    if( p_rec->b_bottom )
                    {
                        if(!p_rec->p_save[2])
                        {
                            p_rec->p_save[0] = &p_rec->image[2];	 
                        }
                        else
                        {
                            p_rec->p_save[0] = p_rec->p_save[2];
                        }    
                    }
                }

                if( p_rec->b_bottom )
                {
                    p_rec->p_save[2] = p_rec->p_save[1];
                    p_rec->p_save[1] = p_image;
                    memset(&p_rec->cur, 0, sizeof(cavs_image));

                    /* copy ref-list to anther threads */
                    p_m->current[1]->p_save[0] = p_rec->p_save[0];
                    p_m->current[1]->p_save[1] = p_rec->p_save[1];
                    p_m->current[1]->p_save[2] = p_rec->p_save[2];
                }
                p_rec->b_complete = 0;
                p_rec->b_bottom = 0;
            }
            else if( p_rec->b_complete )
            {
                i_result = CAVS_FRAME_OUT;

                /* set for ffmpeg interface */
                param->output_type =  p_rec->p_save[0]->i_code_type;
                 
                cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv,  p_rec->b_bottom);
                memset(&p_rec->cur,0,sizeof(cavs_image));
                p_rec->b_complete=0;
                p_rec->b_bottom = 0;
            }	                         
        }
    }

    /*wait for end of AEC handle */
    if( p_aec->b_thread_active == 1 )
    {  
        if(cavs_threadpool_wait( p_m->threadpool, p_aec))
            ;//return -1;

        p_aec->b_thread_active = 0;
    }
    
    p_m->unused[0]->p_save_aec[0] = p_aec->p_save_aec[0];
    p_m->unused[0]->p_save_aec[1] = p_aec->p_save_aec[1];
    p_m->unused[0]->p_save_aec[2] = p_aec->p_save_aec[2];

    /* copy col type to next handle */
    memcpy( p_m->unused[0]->p_col_mv, p_aec->p_col_mv,p_m->i_mb_width*(p_m->i_mb_height)*4*sizeof(cavs_vector));
    memcpy( p_m->unused[0]->p_col_type_base, p_aec->p_col_type_base, p_m->i_mb_width*(p_m->i_mb_height));

#if B_MB_WEIGHTING
	{
		int i, j;
		for( j = 0; j < 2; j++ )
		{
            p_m->unused[0]->b_slice_weighting_flag[j] = p_aec->b_slice_weighting_flag[j];
			p_m->unused[0]->b_mb_weighting_flag[j] = p_aec->b_mb_weighting_flag[j];
			for( i = 0; i < 4; i++ )
			{
				p_m->unused[0]->i_luma_scale[j][i] = p_aec->i_luma_scale[j][i];
				p_m->unused[0]->i_luma_shift[j][i] = p_aec->i_luma_shift[j][i];
				p_m->unused[0]->i_chroma_scale[j][i] = p_aec->i_chroma_scale[j][i];
				p_m->unused[0]->i_chroma_shift[j][i] = p_aec->i_chroma_shift[j][i];
			}
		}
	}
#endif	

    if( p->b_error_flag )
    {
        i_result = CAVS_ERROR;
    }

    return i_result;
}

static int field_decode_aec_threads_far( cavs_decoder *p_aec )
{
    cavs_decoder *p_m = p_aec->p_m;
    frame_pack *field = p_aec->fld;
    uint8_t   *p_buf;
    int    i_len, i_result = 0;
    int slice_count = p_aec->slice_count;
    int field_count = p_aec->field_count;
    uint32_t   cur_startcode;
    uint32_t i_ret = 0;

    /* pack slice */
    cavs_field_pack( p_aec->p_stream, field );

    cur_startcode = field->slice[0].i_startcode;
    p_buf = field->slice[0].p_start;
    i_len = field->slice[0].i_len;
    field->slice_num--;

    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            cavs_bitstream_init(&p_aec->s, p_buf, i_len);
            cavs_get_i_picture_header(&p_aec->s,&p_aec->ph,&p_aec->vsh);
			
			if( p_aec->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{
				//p_aec->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p_aec->ph.b_aec_enable = p_m->vsh.b_aec_enable;
			}
			else if(p_aec->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p_aec->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p_aec->b_error_flag  = 1;

                return CAVS_ERROR;
			}

			p_aec->ph.b_picture_structure = 0; /* NOTE : guarantee field or frame not change */

            i_ret = cavs_init_picture_context(p_aec);
            if( i_ret == CAVS_ERROR )
            {
                p_aec->b_error_flag  = 1;
                return CAVS_ERROR;
            }
            cavs_init_picture_ref_list_for_aec(p_aec);

            break;

        case CAVS_PB_PICUTRE_START_CODE:
            cavs_bitstream_init(&p_aec->s, p_buf, i_len);
            cavs_get_pb_picture_header(&p_aec->s, &p_aec->ph, &p_aec->vsh);

			if( p_aec->vsh.i_profile_id == PROFILE_GUANGDIAN )
			{
				//p_aec->ph.b_aec_enable = 1; /* AEC only for AVS+ */
				p_aec->ph.b_aec_enable = p_m->vsh.b_aec_enable;
			}
			else if(p_aec->vsh.i_profile_id == PROFILE_JIZHUN)
			{
				p_aec->ph.b_aec_enable = 0; /* AVS */
			}
			else
			{
				p_aec->b_error_flag  = 1;

                return CAVS_ERROR;
			}
            p_aec->ph.b_picture_structure = 0; /* NOTE : guarantee field not change */

            i_ret = cavs_init_picture_context(p_aec);
            if( i_ret == CAVS_ERROR )
            {
                p_aec->b_error_flag  = 1;
                return CAVS_ERROR;
            }
            cavs_init_picture_ref_list_for_aec(p_aec);

            break;

        default:
            return CAVS_ERROR;
    }	
    
    /*add for weighting quant */
    cavs_init_frame_quant_param( p_aec );

    cavs_frame_update_wq_matrix( p_aec );
    while( field->slice_num--)
    {
        cur_startcode = field->slice[slice_count].i_startcode;
        p_buf = field->slice[slice_count].p_start;
        i_len = field->slice[slice_count].i_len;
        slice_count++;
        field_count++;
        
        switch( cur_startcode )
        {
            case CAVS_USER_DATA_CODE:
                cavs_bitstream_init(&p_aec->s,p_buf,i_len);
                i_result = cavs_get_user_data(&p_aec->s, &p_aec->user_data);
                break;

            case CAVS_EXTENSION_START_CODE:
                cavs_bitstream_init(&p_aec->s,p_buf,i_len);
                i_result = cavs_get_extension(&p_aec->s, p_aec);
                break;

            default:

                if( cur_startcode >= CAVS_SLICE_MIN_START_CODE 
                    && cur_startcode <= CAVS_SLICE_MAX_START_CODE )
                {
                    int i_next_mb_y;
                    uint32_t i_mb_y_ori = 0;

                    cavs_bitstream_init(&p_aec->s, p_buf, i_len);                        
                    p_aec->i_mb_y = cur_startcode & 0x000000FF;
                    i_mb_y_ori = p_aec->i_mb_y;                    
                        
                    /* set b_bottom */
                    p_aec->b_bottom = 0;
                    if( p_aec->i_mb_y >= (p_aec->i_mb_height>>1))
                    {
                        p_aec->b_bottom = 1;
                            
                        i_ret = cavs_init_picture_context(p_aec);
                        if( i_ret == CAVS_ERROR )
                        {
                            p_aec->b_error_flag  = 1;
                            return CAVS_ERROR;
                        }
                        cavs_init_picture_ref_list_for_aec(p_aec);
                    }
                    p_aec->i_mb_y = i_mb_y_ori;
             
                    if( field->slice_num != 0 )
                        i_next_mb_y = field->slice[slice_count].i_startcode & 0x000000FF; /*slice_count++ before*/
                    else
                        i_next_mb_y = 0xFF;/* bot-field */
                    if (i_next_mb_y>=0 && i_next_mb_y<=0xAF)
                        p_aec->i_slice_mb_end = i_next_mb_y * p_aec->i_mb_width;
                    else
                    {
                        p_aec->i_slice_mb_end = (p_aec->i_mb_num>>(!p_aec->b_bottom));
                    }
                    p_aec->i_slice_mb_end_fld[p_aec->b_bottom] =  p_aec->i_slice_mb_end;

                    cavs_get_slice_aec(p_aec);
                    if(p_aec->b_error_flag == 1)
                        return  CAVS_ERROR;

                    /* creat independent ref-list for AEC module */
                    if( p_aec->ph.i_picture_coding_type != CAVS_B_PICTURE )
                    {
                        cavs_image *p_image = p_aec->p_save_aec[0];

                        if(!p_aec->p_save_aec[1])
                        {
                            if( p_aec->b_bottom )
                                p_aec->p_save_aec[0] = &p_aec->image_aec[1];
                        }
                        else
                        {
                            if( p_aec->b_bottom )
                            {
                                if(!p_aec->p_save_aec[2])
                                {
                                    p_aec->p_save_aec[0] = &p_aec->image_aec[2];	 
                                }
                                else
                                {
                                    p_aec->p_save_aec[0] = p_aec->p_save_aec[2];
                                }
                            }
                        }

                        if( p_aec->b_bottom )
                        {
                            p_aec->p_save_aec[2] = p_aec->p_save_aec[1];
                            p_aec->p_save_aec[1] = p_image;
                            memset(&p_aec->cur_aec, 0, sizeof(cavs_image));
                        }
                    }
                }
                else
                {
					p_aec->b_error_flag = 1;
                    return CAVS_ERROR;
                }
        }
    }

    return 0;
}

static int cavs_field_decode_far(void *p_decoder, frame_pack *field, cavs_param *param )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    cavs_decoder *p_m = p;/* backup main threads */
    cavs_decoder *p_aec, *p_rec;
    int field_count = 0;
    
    int i_result = 0;
    int sliceNum, threadNum;
    int slice_count = 1; /* 0 is for pic header */
    
    sliceNum = field->slice_num - 1; /* exclude pic header */
    threadNum = 0;

    /* init aec handle */
    p_aec = p_m->unused[0];
    p = p_aec;
    p_aec->b_bottom = 0; /* init bottom flag */
    p_aec->i_frame_decoded = p_m->i_frame_decoded; //just for debug

    /* init p_aec*/
    p_aec->p_m = p_m;
    p_aec->fld = field;
    p_aec->slice_count = slice_count;
    p_aec->field_count = field_count; 

    cavs_threadpool_run( p_m->threadpool, (void*)field_decode_aec_threads_far, p_aec );
    p_aec->b_thread_active = 1;

    p_m->unused[0] = p_m->unused[1];
    p_m->unused[1] = p_aec;
    
    p_m->current[0] = p_m->current[1];
    p_m->current[1] = p_aec;

    /* rec decode loop */
    p_rec = p_m->current[0];
    field_count =  2; /* only support two field */
    if( p_rec != NULL )
    {
        while( field_count-- )
        {
            p_rec->b_bottom = !field_count; /* first-top second-botttom */  

            cavs_init_picture_context_fld( p_rec );
            i_result = cavs_get_slice_rec_threads( p_rec );
            if( i_result == -1 )
            {
                /* close AEC at same time */
                if( p_aec->b_thread_active == 1 )
                {
                	if(cavs_threadpool_wait( p_m->threadpool, p_aec ) )
                		;//return -1;
                    p_aec->b_thread_active = 0;
                }
                
                return CAVS_ERROR;
            }
            
            if( p_rec->b_bottom )
            {
                if( p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE )
                    p_m->last_delayed_pframe = p_rec->p_save[1];
            }
            
            if(p_rec->b_complete && p_rec->ph.i_picture_coding_type != CAVS_B_PICTURE)
            {
                cavs_image *p_image = p_rec->p_save[0];
                if(p_rec->vsh.b_low_delay)
                {
                    i_result = CAVS_FRAME_OUT;

                    /* set for ffmpeg interface */
                    param->output_type = p_rec->p_save[0]->i_code_type;
                    
                    cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv,  p_rec->b_bottom);
                }
                if(!p_rec->p_save[1])
                {
                    if( p_rec->b_bottom )
                        p_rec->p_save[0] = &p_rec->image[1];
                }
                else
                {
                    if(p_rec->vsh.b_low_delay == 0 /*&& p_rec->ph.i_picture_coding_type != CAVS_I_PICTURE*/ )
                    {
                        i_result = CAVS_FRAME_OUT;

                        /* set for ffmpeg interface */
                        param->output_type = p_rec->p_save[1]->i_code_type;
                          
                        cavs_out_image_yuv420(p_m, p_rec->p_save[1], param->p_out_yuv,  p_rec->b_bottom);
                    }
                    else
                    {
                    	param->output_type = -1;
                    }

                    if( p_rec->b_bottom )
                    {
                        if(!p_rec->p_save[2])
                        {
                            p_rec->p_save[0] = &p_rec->image[2];	 
                        }
                        else
                        {
                            p_rec->p_save[0] = p_rec->p_save[2];
                        }    
                    }
                }

                if( p_rec->b_bottom )
                {
                    p_rec->p_save[2] = p_rec->p_save[1];
                    p_rec->p_save[1] = p_image;
                    memset(&p_rec->cur, 0, sizeof(cavs_image));

                    /* copy ref-list to anther threads */
                    p_m->current[1]->p_save[0] = p_rec->p_save[0];
                    p_m->current[1]->p_save[1] = p_rec->p_save[1];
                    p_m->current[1]->p_save[2] = p_rec->p_save[2];
                }
                p_rec->b_complete = 0;
                p_rec->b_bottom = 0;
            }
            else if( p_rec->b_complete )
            {
                i_result = CAVS_FRAME_OUT;

                /* set for ffmpeg interface */
                param->output_type =  p_rec->p_save[0]->i_code_type;
                 
                cavs_out_image_yuv420(p_m, p_rec->p_save[0], param->p_out_yuv,  p_rec->b_bottom);
                memset(&p_rec->cur,0,sizeof(cavs_image));
                p_rec->b_complete=0;
                p_rec->b_bottom = 0;
            }	                         
        }
    }

    /*wait for end of AEC handle */
    if( p_aec->b_thread_active == 1 )
    {  
	if(cavs_threadpool_wait( p_m->threadpool, p_aec ) )
        		;//return -1;

        p_aec->b_thread_active = 0;
    }
    
    if( p_aec->b_error_flag )
    {
        return CAVS_ERROR;
    }
    
    p_m->unused[0]->p_save_aec[0] = p_aec->p_save_aec[0];
    p_m->unused[0]->p_save_aec[1] = p_aec->p_save_aec[1];
    p_m->unused[0]->p_save_aec[2] = p_aec->p_save_aec[2];

    /* copy col type to next handle */
    memcpy( p_m->unused[0]->p_col_mv, p_aec->p_col_mv,p_m->i_mb_width*(p_m->i_mb_height)*4*sizeof(cavs_vector));
    memcpy( p_m->unused[0]->p_col_type_base, p_aec->p_col_type_base, p_m->i_mb_width*(p_m->i_mb_height));

#if B_MB_WEIGHTING
	{
		int i, j;
		for( j = 0; j < 2; j++ )
		{
            p_m->unused[0]->b_slice_weighting_flag[j] = p_aec->b_slice_weighting_flag[j];
			p_m->unused[0]->b_mb_weighting_flag[j] = p_aec->b_mb_weighting_flag[j];
			for( i = 0; i < 4; i++ )
			{
				p_m->unused[0]->i_luma_scale[j][i] = p_aec->i_luma_scale[j][i];
				p_m->unused[0]->i_luma_shift[j][i] = p_aec->i_luma_shift[j][i];
				p_m->unused[0]->i_chroma_scale[j][i] = p_aec->i_chroma_scale[j][i];
				p_m->unused[0]->i_chroma_shift[j][i] = p_aec->i_chroma_shift[j][i];
			}
		}
	}
#endif

    if( p->b_error_flag )
    {
        i_result = CAVS_ERROR;
    }

    return i_result;
}


int cavs_decoder_create(void **pp_decoder, cavs_param *param)
{
    cavs_decoder *p;
    int counter = 0;
	int loop_cnt = 0;
    unsigned int i_feature_value = 0;
    int i_result = 0;

    p = (cavs_decoder *)cavs_malloc(sizeof(cavs_decoder));
    if( !p )
    {
        return -1;
    }
    memset( p, 0, sizeof(cavs_decoder) );
    memcpy (&p->param, param, sizeof (cavs_param));
    p->i_video_edit_code_flag=0;
    *pp_decoder = p;

    p->param.b_accelerate = param->b_accelerate; 
    p->param.output_type = param->output_type = -1;

    cavs_decoder_init( p );

    p->p_stream = (InputStream*)cavs_malloc(sizeof(InputStream));;
	
    /* threading */
    /* use threadpool to optimize */
	cavs_decoder_thread_param_init(p);

    return 0;
    
}

int cavs_decoder_get_seq(void *p_decoder, cavs_seq_info *p_si)
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;

    if(!p)
    {
    	return -1;
    }

    if(!p->b_get_video_sequence_header)
    {
    	return -1;
    }
    
    p_si->lWidth=p->vsh.i_horizontal_size;
    p_si->lHeight=p->vsh.i_vertical_size;

    p_si->i_frame_rate_den = frame_rate_tab[p->vsh.i_frame_rate_code][0];
    p_si->i_frame_rate_num = frame_rate_tab[p->vsh.i_frame_rate_code][1];
    p_si->b_interlaced = !p->vsh.b_progressive_sequence;

    p_si->profile = p->vsh.i_profile_id;
    p_si->level = p->vsh.i_level_id;
    p_si->aspect_ratio = p->vsh.i_aspect_ratio;
    p_si->low_delay = p->vsh.b_low_delay;
    p_si->frame_rate_code = p->vsh.i_frame_rate_code;
    
    return 0;
}

int cavs_decoder_buffer_init(cavs_param *param)
{
    int w = param->seqsize.lWidth;
    int h = param->seqsize.lHeight;
    
	if (w > 1920 || h > 1080)
	{
		return -1;
	}

    param->p_out_yuv[0]=(unsigned char *)cavs_malloc(w*h*3/2);
    param->p_out_yuv[1] = param->p_out_yuv[0] + w*h;
    param->p_out_yuv[2] = param->p_out_yuv[1] + w*h/4;
    
    if ( param->p_out_yuv[0] == NULL )
    	return -1;
    return 0;
}

int cavs_decoder_buffer_end(cavs_param *param)
{
    cavs_free(param->p_out_yuv[0]);
    
    return 0;
}

void cavs_decoder_destroy( void *p_decoder )
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;
    if(!p)
    {
    	return ;
    }

    if( p->param.i_thread_num >= 1 )
	{
        cavs_threadpool_delete( p->threadpool );		
	}

 	cavs_decoder_seq_end(p_decoder);
}

void cavs_decoder_slice_destroy( void *p_decoder )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    if(!p)
    {
    	return;
    }

	for (int i = 1; i < p->param.i_thread_num; i++)
	{
		if (p->thread[i])
		{
			/* destroy threads */
			cavs_pthread_mutex_destroy(&p->thread[i]->mutex);
			cavs_pthread_cond_destroy(&p->thread[i]->cv);

			cavs_macroblock_cache_finit(p->thread[i]);

			CAVS_SAFE_FREE(p->thread[i]);
		}
	}
}

int cavs_decoder_reset(void *p_decoder)
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

    if( !p )
    {
        return -1;
    }
    p->i_video_edit_code_flag = 0;
    p->b_get_i_picture_header = 0;
    
    if( p->param.b_accelerate )
    {
        memset(&p->ref_aec[0], 0, sizeof(cavs_image));
        memset(&p->ref_aec[1], 0, sizeof(cavs_image));
        memset(&p->ref_aec[2], 0, sizeof(cavs_image));
        memset(&p->ref_aec[3], 0, sizeof(cavs_image));
        p->image_aec[0].i_distance_index = 0;
        p->image_aec[1].i_distance_index = 0;
        p->image_aec[2].i_distance_index = 0;

        p->p_save_aec[0] = &p->image_aec[0];
        //p->last_delayed_pframe = p->p_save[1];
        p->p_save_aec[1] = 0; // FIXIT : can't shut down this line for open gop
        p->p_save_aec[2] = 0;
    }   

    memset(&p->ref[0], 0, sizeof(cavs_image));
    memset(&p->ref[1], 0, sizeof(cavs_image));
    memset(&p->ref[2], 0, sizeof(cavs_image));
    memset(&p->ref[3], 0, sizeof(cavs_image));
    p->image[0].i_distance_index = 0;
    p->image[1].i_distance_index = 0;
    p->image[2].i_distance_index = 0;

    p->p_save[0] = &p->image[0];
    p->last_delayed_pframe = p->p_save[1];
    p->p_save[1] = 0; // FIXIT : can't shut down this line for open gop
    p->p_save[2] = 0;

    memset(p->p_block, 0, 64*sizeof(DCTELEM));

    return 0;
}

int cavs_out_delay_frame(void *p_decoder, unsigned char *p_out_yuv[3])
{
    cavs_decoder * p = (cavs_decoder *)p_decoder;
    int b_interlaced = p->param.b_interlaced;

	 /* B frame exists */
    if ( !p->b_low_delay 
		&& ((cavs_decoder *)p_decoder)->last_delayed_pframe /* delay frame exists */
		)
    {
        if( !b_interlaced )
            cavs_out_image_yuv420((cavs_decoder *)p_decoder, ((cavs_decoder *)p_decoder)->last_delayed_pframe, p_out_yuv, 0);
        else
        {
            //p->b_bottom = 0;
            cavs_out_image_yuv420((cavs_decoder *)p_decoder, ((cavs_decoder *)p_decoder)->last_delayed_pframe, p_out_yuv,  0);
            //p->b_bottom = 1;
            cavs_out_image_yuv420((cavs_decoder *)p_decoder, ((cavs_decoder *)p_decoder)->last_delayed_pframe, p_out_yuv, 1);
        }

        /* reset for next delay frame */
        ((cavs_decoder *)p_decoder)->last_delayed_pframe = NULL;

        return 1;
    }

    return 0;
}

int cavs_set_last_delay_frame( void *p_decoder )
{
    cavs_decoder * p = (cavs_decoder *)p_decoder;
    
    p->last_delayed_pframe = p->p_save[1];
    
    return 0;
}

int cavs_out_delay_frame_end(void *p_decoder, unsigned char *p_out_yuv[3])
{
    cavs_decoder * p = (cavs_decoder *)p_decoder;
    int b_interlaced = p->param.b_interlaced;
    
	if ( !/*p->vsh.b_low_delay*/p->b_low_delay ) /* B frame exists */
    {
        p->last_delayed_pframe = p->p_save[1];

		if(p->last_delayed_pframe == NULL)
		{
			return 0;
		}

        if( !b_interlaced )
            cavs_out_image_yuv420((cavs_decoder *)p_decoder, ((cavs_decoder *)p_decoder)->last_delayed_pframe, p_out_yuv, 0);
        else
        {
            //p->b_bottom = 0;
            cavs_out_image_yuv420((cavs_decoder *)p_decoder, ((cavs_decoder *)p_decoder)->last_delayed_pframe, p_out_yuv, 0);
            //p->b_bottom = 1;
            cavs_out_image_yuv420((cavs_decoder *)p_decoder, ((cavs_decoder *)p_decoder)->last_delayed_pframe, p_out_yuv, 1);
        }

        return 1;
    }

    return 0;
}

int cavs_decoder_seq_init( void *p_decoder , cavs_param *param )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

    /* NOTE : we can't distinguish frame or field coding frame seq header */
    p->Frame = (frame_pack* )cavs_malloc(sizeof(frame_pack));
	memset(p->Frame, 0, sizeof(frame_pack));
    p->Frame->p_start = (uint8_t* )cavs_malloc(MAX_CODED_FRAME_SIZE*sizeof(uint8_t));
    memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));  
	
    param->fld_mb_end[0] = (((param->seqsize.lHeight+31)& 0xffffffe0)>>5) - 1; /* top field mb end */
    param->fld_mb_end[1] = (((param->seqsize.lHeight+31)& 0xffffffe0)>>4) - 1; /* bot field mb end */

    return 0;
}

int cavs_decoder_seq_end( void *p_decoder )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

	if(p && p->Frame)
	{
		if(p->Frame->p_start)
			cavs_free( p->Frame->p_start );
		cavs_free( p->Frame );
		p->Frame = NULL;
	}

	if (p)
	{
#if B_MB_WEIGHTING
		if (p->mb_line_buffer[0])
		{
			p->mb_line_buffer[0] -= p->image[0].i_stride[0] * CAVS_EDGE + CAVS_EDGE;
			CAVS_SAFE_FREE(p->mb_line_buffer[0]);
		}

		if (p->mb_line_buffer[1])
		{
			p->mb_line_buffer[1] -= p->image[0].i_stride[1] * CAVS_EDGE / 2 + CAVS_EDGE / 2;
			CAVS_SAFE_FREE(p->mb_line_buffer[1]);
		}

		if (p->mb_line_buffer[2])
		{
			p->mb_line_buffer[2] -= p->image[0].i_stride[2] * CAVS_EDGE / 2 + CAVS_EDGE / 2;
			CAVS_SAFE_FREE(p->mb_line_buffer[2]);
		}
#endif

		//if (p->mutex)
		{
			cavs_pthread_mutex_destroy(&p->mutex);
		}
		//if (p->cv)
		{
			cavs_pthread_cond_destroy(&p->cv);
		}

		for (int i = 0; i < 2; i++)
		{
			if (p->unused[i] && p->unused[i] != p)
			{
				//if (p->unused[i]->mutex)
				{
					cavs_pthread_mutex_destroy(&p->unused[i]->mutex);
				}
				//if (p->unused[i]->cv)
				{
					cavs_pthread_cond_destroy(&p->unused[i]->cv);
				}

				cavs_macroblock_cache_finit(p->unused[i]);

				CAVS_SAFE_FREE(p->unused[i]);
			}
		}

		cavs_decoder_slice_destroy(p_decoder);
		cavs_free_resource(p);

		CAVS_SAFE_FREE(p->p_stream);
		CAVS_SAFE_FREE(p);
	}

    return 0;
}


int cavs_decode_one_frame( void *p_decoder, int i_startcode, cavs_param *param, unsigned char* buf, int length )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    InputStream *p_stream = p->p_stream;
    int b_interlaced = p->param.b_interlaced;
    int i_result;
	
	if (!param->seq_header_flag)
	{
		return 0;
	}

    p->i_frame_decoded++;
    if( !b_interlaced ) /* decode one frame or one field */
    {
        /* init frame pack */
        //memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));

        /* init */
        p->Frame->p_cur = p->Frame->p_start;
        p->Frame->i_len = 0;
        p->Frame->slice_num = 0;
        p->Frame->i_startcode_type = i_startcode;
        memcpy(p->Frame->p_cur, buf, length ); /* pack header of pic into one frame */

        /* set pic header info, use position of slice_num = 0 */
        p->Frame->slice[p->Frame->slice_num].i_startcode = i_startcode;
        p->Frame->slice[p->Frame->slice_num].p_start = p->Frame->p_cur + 4; /* for skip current startcode */
        p->Frame->slice[p->Frame->slice_num].i_len = length-4;

        /* update frame info */
        p->Frame->p_cur = p->Frame->p_cur + length;
        p->Frame->i_len += length;
        p->Frame->slice_num++;

        if( !p->param.b_accelerate )
        {
            i_startcode = cavs_frame_pack(p_stream, p->Frame );
            if(i_startcode == CAVS_ERROR)
                return CAVS_ERROR;

            i_result = decode_one_frame( p_decoder, p->Frame, param );
        }
        else
        {
            i_result = decode_one_frame_far( p_decoder, p->Frame, param );
        }
    }
    else
    {
        uint32_t frame_type = i_startcode;
        
        /* init field pack of top */
        //memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));
        p->Frame->b_bottom = 0;
        p->Frame->i_mb_end[0] = param->fld_mb_end[0];
        p->Frame->p_cur = p->Frame->p_start;
        p->Frame->i_len = 0;
        p->Frame->slice_num = 0;
        p->Frame->i_startcode_type = i_startcode;
        memcpy( p->Frame->p_cur, buf, length ); /* pack header of pic into one frame */

        /* set pic header info , use position of slice_num = 0 */
        p->Frame->slice[p->Frame->slice_num].i_startcode = i_startcode;
        p->Frame->slice[p->Frame->slice_num].p_start = p->Frame->p_cur + 4; /* for skip current startcode */
        p->Frame->slice[p->Frame->slice_num].i_len = length-4;
        
        /* update field info of top */
        p->Frame->p_cur = p->Frame->p_cur + length;
        p->Frame->i_len += length;
        p->Frame->slice_num++;

        if( p->param.b_accelerate ) /* entropy decoded parallel */
        {
            p->Frame->i_mb_end[0] = param->fld_mb_end[0];
            p->Frame->i_mb_end[1] = param->fld_mb_end[1];
            p->b_bottom = 0;

            /* decode one image as two field */
            i_result = cavs_field_decode_far( p_decoder, p->Frame, param );
      
        }
        else
        {
            /* pack top-field */
            i_startcode = cavs_topfield_pack(p_stream, p->Frame );

            /* decode top-field */
            i_result = decode_top_field( p_decoder, p->Frame, param );               
            if( i_result == CAVS_ERROR )
            {
                return i_result;
            }

            /* init field pack of bot */
            /* NOTE : bot-field has no pic header */
            memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));
            p->Frame->b_bottom = 1;
            p->Frame->i_mb_end[0] = param->fld_mb_end[0];
            p->Frame->i_mb_end[1] = param->fld_mb_end[1];
            p->Frame->p_cur = p->Frame->p_start;
            p->Frame->i_len = 0;
            p->Frame->slice_num = 0;
            p->Frame->i_startcode_type = frame_type;

            /* pack bot-field */
            i_startcode = cavs_botfield_pack(p_stream, p->Frame );

            /* decode bot-field */
            i_result = decode_bot_field( p_decoder, p->Frame, param );    
        }
    }

    return i_result;
}

int cavs_decoder_init_stream( void *p_decoder, unsigned char *rawstream, unsigned int i_len)
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

    cavs_init_bitstream( p->p_stream , rawstream, i_len );

    return 0;
}

int cavs_decoder_get_one_nal( void *p_decoder, unsigned char *buf, int *length )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    int i_ret;
    
    i_ret = cavs_get_one_nal( p->p_stream, buf, length );

    return i_ret;
}

int cavs_decoder_cur_frame_type( void* p_decoder )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

    if( p->param.b_accelerate )
    {
        return p->unused[0]->ph.i_picture_coding_type;
    } 
    else
        return p->ph.i_picture_coding_type;
}

int cavs_decoder_thread_param_init( void* p_decoder )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

	p->param.i_thread_num = cavs_cpu_num_processors();

    return 0;    
}




int cavs_decoder_probe_seq(void *p_decoder,  unsigned char  *p_in, int i_in_length)
{
    cavs_decoder *p=(cavs_decoder *)p_decoder;
    uint32_t   i_startcode;
    uint8_t   *p_buf;
    int    i_len, i_result=0;

    p_buf = p_in;
    i_len = i_in_length;	

    i_startcode = ((p_buf[0]<<24))|((p_buf[1]<<16))|(p_buf[2]<<8)|p_buf[3];
    p_buf += 4; /* skip startcode */
    i_len -= 4;
    if(i_startcode==0)
    {
    	return CAVS_ERROR;
    }

    if(i_startcode!=CAVS_VIDEO_SEQUENCE_START_CODE&&!p->b_get_video_sequence_header)
    {
    	return  CAVS_ERROR;
    } 

    switch(i_startcode)
    {
    case CAVS_VIDEO_SEQUENCE_START_CODE:
    	cavs_bitstream_init(&p->s,p_buf,i_len);
    	if(cavs_get_video_sequence_header(&p->s,&p->vsh)==0)
    	{
            p->b_get_video_sequence_header = 1;

            return CAVS_SEQ_HEADER;
    	}
		else
		{
			return CAVS_ERROR;
		}

    	break;
    case CAVS_VIDEO_SEQUENCE_END_CODE:
    	p->last_delayed_pframe = p->p_save[1];
    	i_result = CAVS_SEQ_END;
    	break;
    case CAVS_USER_DATA_CODE:
    	cavs_bitstream_init(&p->s,p_buf,i_len);
    	i_result = cavs_get_user_data(&p->s, &p->user_data);
    	break;
    case CAVS_EXTENSION_START_CODE:
    	cavs_bitstream_init(&p->s,p_buf,i_len);
    	i_result = cavs_get_extension(&p->s, p);
    	break;
    case CAVS_VIDEO_EDIT_CODE:
    	p->i_video_edit_code_flag=0;
    	break;

    }

    return i_result;

//fail:
//    return -1;
}


int cavs_decoder_pic_header( void* p_decoder,  unsigned char  *p_buf,  int i_len, cavs_param* param, unsigned int cur_startcode )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    int i_ret = 0;

    /* decode header of pic */
    switch( cur_startcode )
    {
        case CAVS_I_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf+4, i_len-4);
            cavs_get_i_picture_header(&p->s,&p->ph,&p->vsh);
            break;

        case CAVS_PB_PICUTRE_START_CODE:
            cavs_bitstream_init(&p->s, p_buf+4, i_len-4);
       
			i_ret = cavs_get_pb_picture_header(&p->s,&p->ph,&p->vsh);
            if(i_ret == -1)
                return CAVS_ERROR;
				
            break;

        default:
            return CAVS_ERROR;
    }

    /* decide frame or field */
    param->b_interlaced = !p->ph.b_picture_structure;
    p->b_get_video_sequence_header = 0;

    return 0;
}

int cavs_decoder_set_format_type( void* p_decoder, cavs_param *param )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

	p->param.b_interlaced = param->b_interlaced;

    return 0;
}

static int cavs_decoder_reset_pipeline( cavs_decoder *p_decoder )
{
    cavs_decoder *p = p_decoder;

    if( !p )
    {
        return -1;
    }
    p->i_video_edit_code_flag = 0;
    p->b_get_i_picture_header = 0;
    
    if( p->param.b_accelerate )
    {
        memset(&p->ref_aec[0], 0, sizeof(cavs_image));
        memset(&p->ref_aec[1], 0, sizeof(cavs_image));
        memset(&p->ref_aec[2], 0, sizeof(cavs_image));
        memset(&p->ref_aec[3], 0, sizeof(cavs_image));
        p->image_aec[0].i_distance_index = 0;
        p->image_aec[1].i_distance_index = 0;
        p->image_aec[2].i_distance_index = 0;

        p->p_save_aec[0] = &p->image_aec[0];
        //p->last_delayed_pframe = p->p_save[1];
        p->p_save_aec[1] = NULL; // FIXIT : can't shut down this line for open gop
        p->p_save_aec[2] = NULL;
    }  

    memset(&p->ref[0], 0, sizeof(cavs_image));
    memset(&p->ref[1], 0, sizeof(cavs_image));
    memset(&p->ref[2], 0, sizeof(cavs_image));
    memset(&p->ref[3], 0, sizeof(cavs_image));
    p->image[0].i_distance_index = 0;
    p->image[1].i_distance_index = 0;
    p->image[2].i_distance_index = 0;

    p->p_save[0] = &p->image[0];
    p->last_delayed_pframe = p->p_save[1];
    p->p_save[1] = NULL; // FIXIT : can't shut down this line for open gop
    p->p_save[2] = NULL;

    memset(p->p_block, 0, 64*sizeof(DCTELEM));

    return 0;
}

int cavs_decoder_seq_header_reset( void* p_decoder )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;
    
    if(p->b_low_delay != p->vsh.b_low_delay)
    {
        p->b_low_delay = p->vsh.b_low_delay;
    }
    
    return 0;
}

int cavs_decoder_seq_header_reset_pipeline( void* p_decoder )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

    /* reset for pipeline */
    cavs_decoder_reset_pipeline( p->unused_backup[0] );
    cavs_decoder_reset_pipeline( p->unused_backup[1] );

	if (p->unused_backup[1])
	{
		p->unused_backup[1]->p_save_aec[0] = &p->image_aec[0];
	}
	if (p->unused_backup[1])
	{
		p->unused_backup[1]->p_save[0] = &p->image[0];
	}

    p->current[0] = NULL;
    p->current[1] = NULL;
    
    return 0;
}

int cavs_decoder_low_delay_value( void* p_decoder )
{
	cavs_decoder *p = (cavs_decoder *)p_decoder;

	return  p->vsh.b_low_delay;
}

int cavs_decoder_slice_num_probe( void* p_decoder,  int i_startcode, cavs_param *param, unsigned char  *buf,  int length )
{
    cavs_decoder *p = (cavs_decoder *)p_decoder;

    int b_interlaced = p->param.b_interlaced;

    p->Frame = (frame_pack* )cavs_malloc(sizeof(frame_pack));
    p->Frame->p_start = (uint8_t* )cavs_malloc(MAX_CODED_FRAME_SIZE*sizeof(uint8_t));
    memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));  

    if( !b_interlaced )
    {
        /* init frame pack */
        memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));

        /* init */
        p->Frame->p_cur = p->Frame->p_start;
        p->Frame->i_len = 0;
        p->Frame->slice_num = 0;
        p->Frame->i_startcode_type = i_startcode;
        memcpy(p->Frame->p_cur, buf, length ); /* pack header of pic into one frame */

        /* set pic header info, use position of slice_num = 0 */
        p->Frame->slice[p->Frame->slice_num].i_startcode = i_startcode;
        p->Frame->slice[p->Frame->slice_num].p_start = p->Frame->p_cur + 4; /* for skip current startcode */
        p->Frame->slice[p->Frame->slice_num].i_len = length-4;

        /* update frame info */
        p->Frame->p_cur = p->Frame->p_cur + length;
        p->Frame->i_len += length;
        p->Frame->slice_num++;

        /* pack  */
        cavs_frame_pack( p->p_stream,  p->Frame );

		p->param.b_accelerate = 0; /* accelerate version don't support multi-slice */
		p->param.i_thread_num = 1;// cavs_cpu_num_processors(); //64;
		param->b_accelerate = 0;
    }
    else
    {
        /* init field pack of top */
        memset(p->Frame->p_start, 0, MAX_CODED_FRAME_SIZE*sizeof(unsigned char));
        p->Frame->b_bottom = 0;
        //p->Frame->i_mb_end[0] = param->fld_mb_end[0];
        p->Frame->p_cur = p->Frame->p_start;
        p->Frame->i_len = 0;
        p->Frame->slice_num = 0;
        p->Frame->i_startcode_type = i_startcode;
        memcpy( p->Frame->p_cur, buf, length ); /* pack header of pic into one frame */

        /* set pic header info , use position of slice_num = 0 */
        p->Frame->slice[p->Frame->slice_num].i_startcode = i_startcode;
        p->Frame->slice[p->Frame->slice_num].p_start = p->Frame->p_cur + 4; /* for skip current startcode */
        p->Frame->slice[p->Frame->slice_num].i_len = length-4;

        /* update field info of top */
        p->Frame->p_cur = p->Frame->p_cur + length;
        p->Frame->i_len += length;
        p->Frame->slice_num++;

		p->Frame->i_mb_end[0] = (((p->vsh.i_vertical_size+31)& 0xffffffe0)>>5) - 1; /* top field mb end */
		p->Frame->i_mb_end[1] = (((p->vsh.i_vertical_size+31)& 0xffffffe0)>>4) - 1; /* bot field mb end */

        /* pack  */
        cavs_field_pack( p->p_stream, p->Frame );

		p->param.b_accelerate = 0; /* accelerate version don't support multi-slice */
		p->param.i_thread_num = 1;// cavs_cpu_num_processors();
		param->b_accelerate = 0;
    }

    cavs_free( p->Frame->p_start );
	p->Frame->p_start = NULL;
    cavs_free( p->Frame );
	p->Frame = NULL;
    
    return 0;
}

/* Opt for multi-thread of AEC module */
/* AEC module : aec or 2d-vlc */
/* REC module : reconstructed processdure */

/*================================= MB level ========================================================*/

/* read coeffs */
static int get_residual_block_aec_opt( cavs_decoder *p, const cavs_vlc *p_vlc_table, 
	int i_escape_order, int b_chroma, int i_block )
{
    int *level_buf = p->level_buf;
    int *run_buf = p->run_buf;
    int i;
    int pos = -1;
    //DCTELEM *block = p->p_block;

    i = p->bs_read_coeffs(p, p_vlc_table, i_escape_order, b_chroma);
    if(p->b_error_flag )
    {
        return -1;
    }
      
    /* copy buf to tab */
    memcpy( &(p->num_buf_tab[p->i_mb_index][i_block]), &i, 1*sizeof(int));
    memcpy( p->run_buf_tab[p->i_mb_index][i_block], run_buf, 64*sizeof(int));
    memcpy( p->level_buf_tab[p->i_mb_index][i_block], level_buf, 64*sizeof(int));

	if (i == -1 || i > 64)
	{
		p->b_error_flag = 1;
		return -1;
    }

    /* add for detecting pos error */
    while(--i >= 0)
    {
        pos += /*run_buf[i]*/p->run_buf_tab[p->i_mb_index][i_block][i];
        if(pos > 63 || pos < 0) 
        {
        	p->b_error_flag = 1;

        	return -1;
        }

        //block[zigzag[pos]] = (/*level_buf[i]*/p->level_buf_tab[p->i_mb_index][i_block][i]*dqm + dqa) >> dqs;
    }
        
    return 0;
}

static int get_residual_block_rec_opt(cavs_decoder *p,const cavs_vlc *p_vlc_table, 
	int i_escape_order,int i_qp, uint8_t *p_dest, int i_stride, int b_chroma, int i_block )
{
    int i, pos = -1;
    //int *level_buf = p->level_buf;
    //int *run_buf = p->run_buf;
    int dqm = dequant_mul[i_qp];
    int dqs = dequant_shift[i_qp];
    int dqa = 1 << (dqs - 1);
    const uint8_t *zigzag = p->ph.b_picture_structure==0 ? zigzag_field : zigzag_progressive;
    DCTELEM *block = p->p_block;
    
    /* read level */
    i= p->num_buf_tab[p->i_mb_index][i_block];

    if( i == -1 || i > 64)
    {
        p->b_error_flag = 1;
        return -1;
    }

    if( !p->b_weight_quant_enable )
    {
     	while(--i >= 0)
    	{
    		pos += p->run_buf_tab[p->i_mb_index][i_block][i];
    		if(pos > 63 || pos < 0) 
    		{
    			p->b_error_flag = 1;
            
    			return -1;
    		}

    		block[zigzag[pos]] = (p->level_buf_tab[p->i_mb_index][i_block][i]*dqm + dqa) >> dqs;
    	}
    }
    else
    {
    	int m, n;

        while(--i >= 0)
    	{
    		pos += p->run_buf_tab[p->i_mb_index][i_block][i];
    		if(pos > 63 || pos < 0 ) 
    		{
    			p->b_error_flag = 1;
            
    			return -1;
    		}

    		m = SCAN[p->ph.b_picture_structure][pos][0];
    		n = SCAN[p->ph.b_picture_structure][pos][1];

             /* NOTE : bug will conflict with idct asm when Win32-Release, don't why now */
    		block[zigzag[pos]] = (((((int)(p->level_buf_tab[p->i_mb_index][i_block][i]*p->cur_wq_matrix[n*8+m])>>3)*dqm)>>4) + dqa) >> dqs;
    	}
    }

	cavs_idct8_add(p_dest, block, i_stride);

    return 0;
}

static inline int get_residual_inter_aec_opt(cavs_decoder *p,int i_mb_type) 
{      
    int block;
    uint8_t *p_d;
    int i_cbp;
    int i_ret = 0;

    i_cbp = p->i_cbp_tab[p->i_mb_index];   

    /* luma */
    for(block = 0; block < 4; block++)
    {
        if( i_cbp & (1<<block))
        {
            p_d = p->p_y + p->i_luma_offset[block];               
            
            i_ret = get_residual_block_aec_opt( p, inter_2dvlc, 0, 0, block);
            if(i_ret == -1)
                return -1;
        }
    }

    /* cb */
    if( i_cbp & /*(1<<4)*/16)
    {
        i_ret = get_residual_block_aec_opt( p, chroma_2dvlc, 0, 1, 4);
        if(i_ret == -1)
            return -1;

    }
    
    /* cr */
    if( i_cbp & /*(1<<5)*/32)
    {
        i_ret = get_residual_block_aec_opt( p, chroma_2dvlc, 0, 1, 5);
        if(i_ret == -1)
            return -1;
    }

    return 0;
}

static inline int get_residual_inter_rec_opt(cavs_decoder *p,int i_mb_type) 
{      
    int block;
    int i_stride = p->cur.i_stride[0];
    uint8_t *p_d;
    int i_qp_cb = 0, i_qp_cr = 0;
    int i_cbp;
    int i_ret = 0;

    i_cbp = p->i_cbp_tab[p->i_mb_index];

	if(i_cbp <0 || i_cbp >63 || p->i_qp_tab[p->i_mb_index] < 0
		|| p->i_qp_tab[p->i_mb_index] > 63 )
	{
		return -1;
	}

    /* luma */
    for(block = 0; block < 4; block++)
    {
        if( i_cbp & (1<<block))
        {
            p_d = p->p_y + p->i_luma_offset[block];
               
            i_ret = get_residual_block_rec_opt( p, inter_2dvlc, 0, p->i_qp_tab[p->i_mb_index], p_d, i_stride, 0, block );
            if( i_ret == -1 )
                return -1;
        }
    }

    /* FIXIT : weighting quant qp for chroma
        NOTE : not modify p->i_qp directly
    */
    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
    { 
        i_qp_cb = clip3_int( p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_u, 0, 63  );
        i_qp_cr = clip3_int( p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_v, 0, 63 );
    }

    /* cr */
    if( i_cbp & /*(1<<4)*/16)
    {
        i_ret = get_residual_block_rec_opt(p, chroma_2dvlc, 0, 
                              chroma_qp[p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable ? i_qp_cb : p->i_qp_tab[p->i_mb_index]],
                              p->p_cb, p->cur.i_stride[1], 1, 4);
        if( i_ret == -1 )
            return -1;
    }

    /* cr */
    if( i_cbp & /*(1<<5)*/32)
    {
        i_ret = get_residual_block_rec_opt(p, chroma_2dvlc, 0, 
                              chroma_qp[ p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable ? i_qp_cr : p->i_qp_tab[p->i_mb_index]],
                              p->p_cr, p->cur.i_stride[2], 1, 5);
        if( i_ret == -1 )
            return -1;
    }

    return 0;
}

static inline void rec_init_mb( cavs_decoder *p )
{
    //int i = 0;
    int i_offset = p->i_mb_x<<1;
    int i_offset_b4 = p->i_mb_x<<2;

    if(!(p->i_mb_flags & A_AVAIL))
    {
        p->mv[MV_FWD_A1] = MV_NOT_AVAIL;
        p->mv[MV_FWD_A3] = MV_NOT_AVAIL;
        p->mv[MV_BWD_A1] = MV_NOT_AVAIL;
        p->mv[MV_BWD_A3] = MV_NOT_AVAIL;

        p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
        p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;
    }

    if(p->i_mb_flags & B_AVAIL)
    {
        p->mv[MV_FWD_B2] = p->p_top_mv[0][i_offset];
        p->mv[MV_BWD_B2] = p->p_top_mv[1][i_offset];
        p->mv[MV_FWD_B3] = p->p_top_mv[0][i_offset+1];
        p->mv[MV_BWD_B3] = p->p_top_mv[1][i_offset+1];

        p->i_intra_pred_mode_y[1] = p->p_top_intra_pred_mode_y[i_offset_b4];
        p->i_intra_pred_mode_y[2] = p->p_top_intra_pred_mode_y[i_offset_b4+1];
        p->i_intra_pred_mode_y[3] = p->p_top_intra_pred_mode_y[i_offset_b4+2];
        p->i_intra_pred_mode_y[4] = p->p_top_intra_pred_mode_y[i_offset_b4+3];
    }
    else
    {
        p->mv[MV_FWD_B2] = MV_NOT_AVAIL;
        p->mv[MV_FWD_B3] = MV_NOT_AVAIL;
        p->mv[MV_BWD_B2] = MV_NOT_AVAIL;
        p->mv[MV_BWD_B3] = MV_NOT_AVAIL;
        
        p->i_intra_pred_mode_y[1] = p->i_intra_pred_mode_y[2] = 
        p->i_intra_pred_mode_y[3] = p->i_intra_pred_mode_y[4] = NOT_AVAIL;
    }

    if(p->i_mb_flags & C_AVAIL)
    {
        p->mv[MV_FWD_C2] = p->p_top_mv[0][i_offset+2];
        p->mv[MV_BWD_C2] = p->p_top_mv[1][i_offset+2];
    }
    else
    {
        p->mv[MV_FWD_C2] = MV_NOT_AVAIL;
        p->mv[MV_BWD_C2] = MV_NOT_AVAIL;
    }

    if(!(p->i_mb_flags & D_AVAIL))
    {
        p->mv[MV_FWD_D3] = MV_NOT_AVAIL;
        p->mv[MV_BWD_D3] = MV_NOT_AVAIL;	
    }

    p->p_col_type = &p->p_col_type_base[p->i_mb_y*p->i_mb_width + p->i_mb_x];

    if (p->ph.b_aec_enable)
    {
        if (!p->i_mb_x)
        {
            p->i_chroma_pred_mode_left = 0;
            p->i_cbp_left = -1;

            M32(p->p_mvd[0][0]) 
                = M32(p->p_mvd[0][3]) 
                = M32(p->p_mvd[1][0]) 
                = M32(p->p_mvd[1][3]) = 0;
            
            p->p_ref[0][3] 
                = p->p_ref[0][6] 
                = p->p_ref[1][3] 
                = p->p_ref[1][6] = -1;                
        }
        else
        {
            M32(p->p_mvd[0][0]) = M32(p->p_mvd[0][2]);
            M32(p->p_mvd[0][3]) = M32(p->p_mvd[0][5]);
            M32(p->p_mvd[1][0]) = M32(p->p_mvd[1][2]);
            M32(p->p_mvd[1][3]) = M32(p->p_mvd[1][5]);

            p->p_ref[0][3] = p->p_ref[0][5];
            p->p_ref[0][6] = p->p_ref[0][8];
            p->p_ref[1][3] = p->p_ref[1][5];
            p->p_ref[1][6] = p->p_ref[1][8];
        
        }

        CP16(&p->p_ref[0][1], p->p_ref_top[p->i_mb_x][0]);
        CP16(&p->p_ref[1][1], p->p_ref_top[p->i_mb_x][1]);

        M64(p->p_mvd[0][1]) 
            = M64(p->p_mvd[1][1]) 
            = M64(p->p_mvd[0][4]) 
            = M64(p->p_mvd[1][4]) = 0;

        M16(&p->p_ref[0][4]) 
            = M16(&p->p_ref[0][7]) 
            = M16(&p->p_ref[1][4]) 
            = M16(&p->p_ref[1][7]) = -1;
    }    
}    

static inline int aec_next_mb( cavs_decoder *p )
{
    int i, i_y;
    int i_offset = p->i_mb_x<<1;

    p->i_mb_flags |= A_AVAIL;

    for( i = 0; i <= 20; i += 4 )
    {
        p->mv[i] = p->mv[i+2];
    }
   
    p->mv[MV_FWD_D3] = p->p_top_mv[0][i_offset+1];
    p->mv[MV_BWD_D3] = p->p_top_mv[1][i_offset+1];
    p->p_top_mv[0][i_offset+0] = p->mv[MV_FWD_X2];
    p->p_top_mv[0][i_offset+1] = p->mv[MV_FWD_X3];
    p->p_top_mv[1][i_offset+0] = p->mv[MV_BWD_X2];
    p->p_top_mv[1][i_offset+1] = p->mv[MV_BWD_X3];

    /* for AEC */
    if (p->ph.b_aec_enable)
    {
        p->p_mb_type_top[p->i_mb_x] = p->i_mb_type_tab[p->i_mb_index];

		if( p->ph.i_picture_coding_type == 0 && p->b_bottom == 0)
        {
            p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = p->i_pred_mode_chroma_tab[p->i_mb_index];
            p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = p->i_cbp;
        }
        else
        {
            if( p->i_mb_type_tab[p->i_mb_index] != P_SKIP && p->i_mb_type_tab[p->i_mb_index] != B_SKIP )
            {
                p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = p->i_pred_mode_chroma_tab[p->i_mb_index];
                p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = p->i_cbp;
            } 
            else
            {   
                p->i_chroma_pred_mode_left = p->p_chroma_pred_mode_top[p->i_mb_x] = 0;
                p->i_cbp_left = p->p_cbp_top[p->i_mb_x] = 0; /* skip has not cbp */
                p->i_last_dqp = 0;
            }
        }

        CP16(p->p_ref_top[p->i_mb_x][0], &p->p_ref[0][7]);
        CP16(p->p_ref_top[p->i_mb_x][1], &p->p_ref[1][7]);
    }

    /* Move to next MB */
    p->i_mb_x++;
    if( p->i_mb_x == p->i_mb_width ) 
    { 
        p->i_mb_flags = B_AVAIL|C_AVAIL;

        p->i_intra_pred_mode_y[5] = p->i_intra_pred_mode_y[10] = 
        p->i_intra_pred_mode_y[15] = p->i_intra_pred_mode_y[20] = NOT_AVAIL;
          
        for( i = 0; i <= 20; i += 4 )
        {
            p->mv[i] = MV_NOT_AVAIL;
        }
        p->i_mb_x = 0;
        p->i_mb_y++;
        i_y = p->i_mb_y - p->i_mb_offset;
        p->b_first_line = 0;

        /* for AEC */
        p->i_mb_type_left = -1;
    }
    else
    {
        /* for AEC */
        p->i_mb_type_left = p->i_mb_type_tab[p->i_mb_index];
    }

    if( p->i_mb_x == p->i_mb_width-1 )
    {
        p->i_mb_flags &= ~C_AVAIL;
    }

    if( p->i_mb_x && p->i_mb_y && p->b_first_line==0 ) 
    {
        p->i_mb_flags |= D_AVAIL;
    }

    /* set for AEC decode of frame level */
    if( (p->i_mb_type_tab[p->i_mb_index] != I_8X8 )
		&& (p->ph.i_picture_coding_type != 0 
		|| (p->ph.i_picture_coding_type == CAVS_I_PICTURE && p->b_bottom) ) )
    {
        /* set intra prediction modes to default values */
        p->i_intra_pred_mode_y[5] =  p->i_intra_pred_mode_y[10] = 
        p->i_intra_pred_mode_y[15] =  p->i_intra_pred_mode_y[20] = NOT_AVAIL;

        p->p_top_intra_pred_mode_y[(i_offset<<1)+0] = p->p_top_intra_pred_mode_y[(i_offset<<1)+1] = 
        p->p_top_intra_pred_mode_y[(i_offset<<1)+2] = p->p_top_intra_pred_mode_y[(i_offset<<1)+3] = NOT_AVAIL;    
    }
    
    return 0;
}

static inline int rec_next_mb( cavs_decoder *p )
{
    int i, i_y;

    p->i_mb_flags |= A_AVAIL;
    p->p_y += 16;
    p->p_cb += 8;
    p->p_cr += 8;

    /* Move to next MB */
    p->i_mb_x++;
    if(p->i_mb_x == p->i_mb_width) 
    { 
        p->i_mb_flags = B_AVAIL|C_AVAIL;

        for( i = 0; i <= 20; i += 4 )
        {
            p->mv[i] = MV_NOT_AVAIL;
        }
        p->i_mb_x = 0;
        p->i_mb_y++;
        i_y = p->i_mb_y - p->i_mb_offset;
        p->p_y = p->cur.p_data[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
        p->p_cb = p->cur.p_data[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
        p->p_cr = p->cur.p_data[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];

#if B_MB_WEIGHTING
        p->p_back_y = p->mb_line_buffer[0] + i_y*CAVS_MB_SIZE*p->cur.i_stride[0];
        p->p_back_cb = p->mb_line_buffer[1] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[1];
        p->p_back_cr = p->mb_line_buffer[2] + i_y*(CAVS_MB_SIZE>>1)*p->cur.i_stride[2];
#endif

        p->b_first_line = 0;

        /* for AEC */
        p->i_mb_type_left = -1;
    }
    else
    {
        /* for AEC */
        p->i_mb_type_left = p->i_mb_type_tab[p->i_mb_index];
    }

    if( p->i_mb_x == p->i_mb_width-1 )
    {
        p->i_mb_flags &= ~C_AVAIL;
    }

    if( p->i_mb_x && p->i_mb_y && p->b_first_line==0 ) 
    {
        p->i_mb_flags |= D_AVAIL;
    }
    
    return 0;
}    


static int get_mb_i_aec_opt( cavs_decoder *p )
{
    static const uint8_t scan5x5[4][4] = {
    	{6, 7, 11, 12},
    	{8, 9, 13, 14},
    	{16, 17, 21, 22},
    	{18, 19, 23, 24}
    };

    int i;
    int i_offset = p->i_mb_x<<2;
    int i_rem_mode;
    int i_pred_mode;
    
    int i_ret = 0;

    init_mb(p);

    /* luma predict mode */
    for( i = 0; i < 4; i++ ) 
    {
        {
            int i_pos = scan5x5[i][0];

            i_rem_mode = p->bs_read[SYNTAX_INTRA_LUMA_PRED_MODE](p);
            if( p->b_error_flag )
            {
                return -1;
            }
          
            i_pred_mode = cavs_mb_predict_intra8x8_mode(p, i_pos);
            if(!p->pred_mode_flag)
            {
                i_pred_mode = i_rem_mode + (i_rem_mode >= i_pred_mode);
            }
            if( i_pred_mode > 4) 
            {
                p->b_error_flag = 1;
                
                return -1;
            }

            p->i_intra_pred_mode_y[scan5x5[i][0]] =
            p->i_intra_pred_mode_y[scan5x5[i][1]] =
            p->i_intra_pred_mode_y[scan5x5[i][2]] =
            p->i_intra_pred_mode_y[scan5x5[i][3]] = i_pred_mode;
            p->i_intra_pred_mode_y_tab[p->i_mb_index][scan5x5[i][0]] =
            p->i_intra_pred_mode_y_tab[p->i_mb_index][scan5x5[i][1]] =
            p->i_intra_pred_mode_y_tab[p->i_mb_index][scan5x5[i][2]] =
            p->i_intra_pred_mode_y_tab[p->i_mb_index][scan5x5[i][3]] = i_pred_mode;
        }
    }

    p->i_pred_mode_chroma_tab[p->i_mb_index] = p->bs_read[SYNTAX_INTRA_CHROMA_PRED_MODE](p);
    if(p->i_pred_mode_chroma_tab[p->i_mb_index] > 6 || p->b_error_flag ) 
    {
        p->b_error_flag = 1;
                
        return -1;
    }
    
    p->i_intra_pred_mode_y[5] =  p->i_intra_pred_mode_y[9];
    p->i_intra_pred_mode_y[10] =  p->i_intra_pred_mode_y[14];
    p->i_intra_pred_mode_y[15] =  p->i_intra_pred_mode_y[19];
    p->i_intra_pred_mode_y[20] =  p->i_intra_pred_mode_y[24];

    p->p_top_intra_pred_mode_y[i_offset+0] = p->i_intra_pred_mode_y[21];
    p->p_top_intra_pred_mode_y[i_offset+1] = p->i_intra_pred_mode_y[22];
    p->p_top_intra_pred_mode_y[i_offset+2] = p->i_intra_pred_mode_y[23];
    p->p_top_intra_pred_mode_y[i_offset+3] = p->i_intra_pred_mode_y[24];

    if(!(p->i_mb_flags & A_AVAIL))
    {
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[6] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[11] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[16] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[21] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_C, &p->i_pred_mode_chroma );
    }
    if(!(p->i_mb_flags & B_AVAIL))
    {
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[6] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[7] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[8] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y[9] );
        adapt_pred_mode(TOP_ADAPT_INDEX_C, &p->i_pred_mode_chroma);
    }
    
    if(!(p->i_mb_flags & A_AVAIL))
    {
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][6] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][11] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][16] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][21] );
        adapt_pred_mode(LEFT_ADAPT_INDEX_C, &p->i_pred_mode_chroma_tab[p->i_mb_index] );
    }
    if(!(p->i_mb_flags & B_AVAIL))
    {
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][6] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][7] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][8] );
        adapt_pred_mode(TOP_ADAPT_INDEX_L, &p->i_intra_pred_mode_y_tab[p->i_mb_index][9] );
        adapt_pred_mode(TOP_ADAPT_INDEX_C, &p->i_pred_mode_chroma_tab[p->i_mb_index] );
    }

    /* cbp */
    p->i_cbp_tab[p->i_mb_index] = p->bs_read[SYNTAX_INTRA_CBP](p);
    if( p->b_error_flag )
    {
        return -1;
    }
    
    p->i_cbp = p->i_cbp_tab[p->i_mb_index];
    if( p->i_cbp_tab[p->i_mb_index] > 63 ) 
    {
        p->b_error_flag = 1;

        return -1;
    }

    /* qp */
    p->i_qp_tab[p->i_mb_index] = p->i_qp;
    if(p->i_cbp_tab[p->i_mb_index] && !p->b_fixed_qp)
    {
        int delta_qp =  p->bs_read[SYNTAX_DQP](p);
        
		p->i_qp_tab[p->i_mb_index] = (p->i_qp_tab[p->i_mb_index] -MIN_QP + delta_qp + (MAX_QP-MIN_QP+1))%(MAX_QP-MIN_QP+1)+MIN_QP;

        p->i_qp = p->i_qp_tab[p->i_mb_index];

        if( p->i_qp < 0 || p->i_qp > 63  || p->b_error_flag )
        {
			p->b_error_flag = 1;
            return -1;  
        }
    }
    else
        p->i_last_dqp  = 0; // for cabac only

    /* luma coeffs */
    for( i = 0; i < 4; i++ )
    {
        if(p->i_cbp_tab[p->i_mb_index]  & (1<<i))
        {
            /* read coeffs from stream */
            i_ret = get_residual_block_aec_opt( p, intra_2dvlc,  1, 0, i );
            if( i_ret == -1 )
                return -1;
        }
    }

    /* chroma coeffs */
    if(p->i_cbp_tab[p->i_mb_index]  & (1<<4))
    {
        /* read cb coeffs from stream */
        i_ret = get_residual_block_aec_opt( p, chroma_2dvlc,  0, 1, 4 );
        if( i_ret == -1 )
            return -1;
    }
    if(p->i_cbp_tab[p->i_mb_index]  & (1<<5))
    {
        /* read cr coeffs from stream */
        i_ret = get_residual_block_aec_opt( p, chroma_2dvlc,  0, 1, 5 );
        if( i_ret == -1 )
            return -1;
    }

    p->mv [MV_FWD_X0] = MV_INTRA;
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);
    p->mv[MV_BWD_X0] = MV_INTRA;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);

    if(p->ph.i_picture_coding_type != CAVS_B_PICTURE)
    	*p->p_col_type = I_8X8;

    /* need to update mb info */
    return aec_next_mb(p);
}

static int get_mb_p_aec_opt( cavs_decoder *p )
{
    int i_offset;
    int ref[4];
    int i_mb_type = p->i_mb_type;
    int16_t (*p_mvd)[6][2] = p->p_mvd;
    int8_t (*p_ref)[9] = p->p_ref;
    int i_ret = 0;

    init_mb(p);

#define FWD		0
#define MVD_X0  1
#define MVD_X1	2
#define MVD_X2  4
#define MVD_X3	5
#define REF_X0  4
#define REF_X1	5
#define REF_X2	7
#define REF_X3  8

    switch(i_mb_type)
    {
        case P_SKIP:
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_PSKIP, BLK_16X16, PSKIP_REF, MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            break;
        case P_16X16:
            ref[0] = p->bs_read_ref_p(p, REF_X0);            
            if( ref[0] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN,   BLK_16X16, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }

            M32(p_mvd[FWD][MVD_X1]) = M32(p_mvd[FWD][MVD_X3]) = M32(p_mvd[FWD][MVD_X0]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X2] = p_ref[FWD][REF_X3] = p_ref[FWD][REF_X0];
            break;
        case P_16X8:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if(  ref[0] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            ref[2] = p->bs_read_ref_p(p, REF_X2);
            if( ref[2] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_TOP,      BLK_16X8, ref[0], MVD_X0);
            if( p->b_error_flag )
            {                
                return -1;
            }
            mv_pred(p, MV_FWD_X2, MV_FWD_A1, MV_PRED_LEFT,     BLK_16X8, ref[2], MVD_X2);
            if( p->b_error_flag )
            {
                return -1;
            }
            
            CP32(p_mvd[FWD][MVD_X1], p_mvd[FWD][MVD_X0]);
            CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X2]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X0];
            p_ref[FWD][REF_X3] = p_ref[FWD][REF_X2];
            break;
        case P_8X16:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if( ref[0] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            ref[1] = p->bs_read_ref_p(p, REF_X1);
            if( ref[1] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            mv_pred(p, MV_FWD_X0, MV_FWD_B3, MV_PRED_LEFT,     BLK_8X16, ref[0], MVD_X0);
            if( p->b_error_flag )
            {                
                return -1;
            }
            mv_pred(p, MV_FWD_X1, MV_FWD_C2, MV_PRED_TOPRIGHT, BLK_8X16, ref[1], MVD_X1);
            if( p->b_error_flag )
            {
                return -1;
            }

            CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X1]);
            p_ref[FWD][REF_X2] = p_ref[FWD][REF_X0];
            p_ref[FWD][REF_X3] = p_ref[FWD][REF_X1];
            break;
        case P_8X8:
            ref[0] = p->bs_read_ref_p(p, REF_X0);
            if( ref[0] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
                    
            ref[1] = p->bs_read_ref_p(p, REF_X1);
            if( ref[1] > 3  || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            ref[2] = p->bs_read_ref_p(p, REF_X2);
            if( ref[2] > 3  || p->b_error_flag  )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            ref[3] = p->bs_read_ref_p(p, REF_X3);              
            if( ref[3] > 3 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            mv_pred(p, MV_FWD_X0, MV_FWD_B3, MV_PRED_MEDIAN,   BLK_8X8, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            mv_pred(p, MV_FWD_X1, MV_FWD_C2, MV_PRED_MEDIAN,   BLK_8X8, ref[1], MVD_X1);
            if( p->b_error_flag )
            {
                return -1;
            }
            mv_pred(p, MV_FWD_X2, MV_FWD_X1, MV_PRED_MEDIAN,   BLK_8X8, ref[2], MVD_X2);
            if( p->b_error_flag )
            {
                return -1;
            }
            mv_pred(p, MV_FWD_X3, MV_FWD_X0, MV_PRED_MEDIAN,   BLK_8X8, ref[3], MVD_X3);
            if( p->b_error_flag )
            {
                return -1;
            }

            break;

        default:
            p->b_error_flag = 1;
            return -1;
    }
 
    /* save current mv, mvd, refidx into a frame, ignore neighbor info */
    /* mv copy */
    memcpy(p->mv_tab[p->i_mb_index], p->mv, 24*sizeof(cavs_vector));

    /* mvd copy */
    memcpy(p->p_mvd_tab[p->i_mb_index], p->p_mvd, 2*6*2*sizeof(int16_t));
    
    /* refidx copy */
    memcpy(p->p_ref_tab[p->i_mb_index], p->p_ref, 2*9*sizeof(int8_t));
    
#undef FWD
#undef MVD_X3
#undef MVD_X2
#undef MVD_X1
#undef MVD_X0
#undef REF_X3
#undef REF_X2
#undef REF_X1
#undef REF_X0

#if B_MB_WEIGHTING
	p->weighting_prediction = 0;
#endif
    if (i_mb_type != P_SKIP)
    {
#if B_MB_WEIGHTING
        if ( p->sh.b_slice_weighting_flag && p->sh.b_mb_weighting_flag )
        {
            if( p->ph.b_aec_enable )
			{
				p->weighting_prediction = cavs_cabac_get_mb_weighting_prediction( p );
			}
			else
				p->weighting_prediction = cavs_bitstream_get_bit1(&p->s);	//weighting_prediction
        }
#else 
        if (p->sh.b_mb_weighting_flag)
        {
            cavs_bitstream_get_bit1(&p->s); /* weighting_prediction */
        }
#endif
    }

#if B_MB_WEIGHTING
    p->weighting_prediction_tab[p->i_mb_index] = p->weighting_prediction;
#endif

    i_offset = (p->i_mb_y*p->i_mb_width + p->i_mb_x)*4;
    p->p_col_mv[i_offset+ 0] = p->mv[MV_FWD_X0];
    p->p_col_mv[i_offset + 1] = p->mv[MV_FWD_X1];
    p->p_col_mv[i_offset + 2] = p->mv[MV_FWD_X2];
    p->p_col_mv[i_offset + 3] = p->mv[MV_FWD_X3];

    p->i_qp_tab[p->i_mb_index] = p->i_qp; /* init for skip */
    if (i_mb_type != P_SKIP)
    {
        /* get coded block pattern */
        p->i_cbp_tab[p->i_mb_index] = p->bs_read[SYNTAX_INTER_CBP](p);
        if( p->b_error_flag )
        {
            return -1;
        }
        p->i_cbp = p->i_cbp_tab[p->i_mb_index];
        if( p->i_cbp_tab[p->i_mb_index] > 63) 
        {
            p->b_error_flag = 1;

            return -1;
        }
                
        /* get quantizer */
        p->i_qp_tab[p->i_mb_index] = p->i_qp;
        if(p->i_cbp_tab[p->i_mb_index]  && !p->b_fixed_qp)
        {
            int delta_qp =  p->bs_read[SYNTAX_DQP](p);
            
			p->i_qp_tab[p->i_mb_index] = (p->i_qp_tab[p->i_mb_index] -MIN_QP + delta_qp + (MAX_QP-MIN_QP+1))%(MAX_QP-MIN_QP+1)+MIN_QP;

            p->i_qp = p->i_qp_tab[p->i_mb_index];

            if( p->i_qp < 0 || p->i_qp > 63  || p->b_error_flag )
            {
				p->b_error_flag = 1;
                return -1;  
            }
        }
        else
            p->i_last_dqp = 0;

        i_ret = get_residual_inter_aec_opt(p, i_mb_type);
        if( i_ret == -1 )
            return -1;
    }

    *p->p_col_type = i_mb_type;

    return aec_next_mb(p);
}

static int get_mb_b_aec_opt( cavs_decoder *p )
{   
    int kk = 0; 
    cavs_vector mv[24];
    int block;
    enum cavs_mb_sub_type sub_type[4];
    int flags;
	int ref[4] = { 0 };
    int i_ref_offset = p->ph.b_picture_structure == 0 ? 2 : 1;
    uint8_t i_col_type;
    cavs_vector *p_mv;
    int i_mb_type = p->i_mb_type_tab[p->i_mb_index];
    int16_t (*p_mvd)[6][2] = p->p_mvd;
    int8_t	(*p_ref)[9] = p->p_ref;
    int i_ret = 0;
    
    init_mb(p);

    p->mv[MV_FWD_X0] = MV_REF_DIR;
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);
    p->mv[MV_BWD_X0] = MV_REF_DIR;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);

#define FWD		0
#define BWD		1
#define MVD_X0	1
#define MVD_X1	2
#define MVD_X2	4
#define MVD_X3	5
#define REF_X0  4
#define REF_X1  5
#define REF_X2  7
#define REF_X3  8

    /* The MVD of pos X[0-3] have been initialized as 0
        The REF of pos X[0-3] have been initialized as -1 */
    switch(i_mb_type) 
    {
        case -1:
            return -1;
            break;
        
        case B_SKIP:
        case B_DIRECT:
            get_b_direct_skip_mb(p);
            if(p->b_error_flag)
            {
                return -1;
            }
        
            break;
            
        case B_FWD_16X16:
            ref[0] = p->bs_read_ref_b(p, FWD, REF_X0);            
            if( ref[0] > 1 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0]+i_ref_offset, MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
                                      
            M32(p_mvd[FWD][MVD_X1]) = M32(p_mvd[FWD][MVD_X3]) = M32(p_mvd[FWD][MVD_X0]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X2] = p_ref[FWD][REF_X3] = p_ref[FWD][REF_X0];
            break;
            
        case B_SYM_16X16:
            ref[0] = p->bs_read_ref_b(p, FWD, REF_X0);
            if( ref[0] > 1 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0]+i_ref_offset, MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
              
            mv_pred_sym(p, &p->mv[MV_FWD_X0], BLK_16X16,ref[0]);
            if(p->b_error_flag)
            {
                return -1;
            }
            M32(p_mvd[FWD][MVD_X1]) = M32(p_mvd[FWD][MVD_X3]) = M32(p_mvd[FWD][MVD_X0]);
            p_ref[FWD][REF_X1] = p_ref[FWD][REF_X2] = p_ref[FWD][REF_X3] = p_ref[FWD][REF_X0];
            break;
            
        case B_BWD_16X16:
            ref[0] = p->bs_read_ref_b(p, BWD, REF_X0);
            if( ref[0] > 1 || p->b_error_flag )
            {
                p->b_error_flag = 1;
                
                return -1;
            }
            mv_pred(p, MV_BWD_X0, MV_BWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0], MVD_X0);
            if( p->b_error_flag )
            {
                return -1;
            }
            
            M32(p_mvd[BWD][MVD_X1]) = M32(p_mvd[BWD][MVD_X3]) = M32(p_mvd[BWD][MVD_X0]);
            p_ref[BWD][REF_X1] = p_ref[BWD][REF_X2] = p_ref[BWD][REF_X3] = p_ref[BWD][REF_X0];
            break;
            
        case B_8X8:
            mv[MV_FWD_A1] = p->mv[MV_FWD_A1];
            mv[MV_FWD_B2] = p->mv[MV_FWD_B2];
            mv[MV_FWD_C2] = p->mv[MV_FWD_C2];
            mv[MV_FWD_D3] = p->mv[MV_FWD_D3];
            mv[MV_BWD_A1] = p->mv[MV_BWD_A1];
            mv[MV_BWD_B2] = p->mv[MV_BWD_B2];
            mv[MV_BWD_C2] = p->mv[MV_BWD_C2];
            mv[MV_BWD_D3] = p->mv[MV_BWD_D3];

            for(block = 0; block < 4; block++)
            {    
                sub_type[block] = (enum cavs_mb_sub_type)p->bs_read[SYNTAX_MB_PART_TYPE](p);
                if(sub_type[block] < 0 || sub_type[block] > 3 || p->b_error_flag )
                {
                    p->b_error_flag = 1;
                
                    return -1;
                }
            }
            
            for( block = 0; block < 4; block++)
            {
                if(sub_type[block] == B_SUB_DIRECT || sub_type[block] == B_SUB_BWD)
                    ref[block] = 0;
                else
                    ref[block] = p->bs_read_ref_b(p, FWD, ref_scan[block]);
                  
                if( ref[block] > 1 || p->b_error_flag )
                {
                    p->b_error_flag = 1;
                
                    return -1;
                }
            }
            
            for(block = 0; block < 4; block++)
            {
                if(sub_type[block] == B_SUB_BWD)
                {
                    ref[block] = p->bs_read_ref_b(p, BWD, ref_scan[block]);
                }
                
                if( ref[block] > 1 || p->b_error_flag )
                {
                    p->b_error_flag = 1;
                
                    return -1;
                }
            }
            
            for( block = 0; block < 4; block++ ) 
            {
                switch(sub_type[block])
                {
                    case B_SUB_DIRECT:
                        get_col_info(p, &i_col_type, &p_mv, block);
                        if(!i_col_type)
                        {
                            mv_pred_sub_direct(p, mv, 0, mv_scan[block], mv_scan[block]-3,
                                    MV_PRED_BSKIP, BLK_8X8, i_ref_offset);
							if(p->b_error_flag)
								return -1;
                            mv_pred_sub_direct(p, mv, MV_BWD_OFFS, mv_scan[block]+MV_BWD_OFFS,
                                    mv_scan[block]-3+MV_BWD_OFFS,
                                    MV_PRED_BSKIP, BLK_8X8, 0);
                        } 
                        else
                        {
                            get_b_direct_skip_sub_mb(p, block, p_mv);
                        }
						                            
                        if(p->b_error_flag)
                        {
                            return -1;
                        }

                        break;
                        
                    case B_SUB_FWD:
                        mv_pred(p, mv_scan[block], mv_scan[block]-3,
                                    MV_PRED_MEDIAN, BLK_8X8, ref[block]+i_ref_offset, mvd_scan[block]);
                        if( p->b_error_flag )
                        {
                            return -1;
                        }
                        break;
                        
                    case B_SUB_SYM:
                        mv_pred(p, mv_scan[block], mv_scan[block]-3,
                                        MV_PRED_MEDIAN, BLK_8X8, ref[block]+i_ref_offset, mvd_scan[block]);
                        if( p->b_error_flag )
                        {
                            return -1;
                        }
                        mv_pred_sym(p, &p->mv[mv_scan[block]], BLK_8X8,ref[block]);
                        if(p->b_error_flag)
                        {
                            return -1;
                        }
                        break;
                    default:
                    	break;
                }
            }
            for( block = 0; block < 4; block++ )
            {
                if(sub_type[block] == B_SUB_BWD)
                {
                    mv_pred(p, mv_scan[block]+MV_BWD_OFFS,
                            mv_scan[block]+MV_BWD_OFFS-3,
                            MV_PRED_MEDIAN, BLK_8X8, ref[block], mvd_scan[block]);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                }
            }
            break;
        default:
            flags = partition_flags[i_mb_type];
            if( i_mb_type <= B_SYM_16X16 )
            {
                p->b_error_flag = 1;

                return -1;
            }
            if(i_mb_type & 1) /* 16x8 macroblock types */
            { 
                int i = 0, k = 0;
                
                if (flags & FWD0)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X0);
                if (flags & FWD1)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X2);
                if (flags & BWD0)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X0);
                if (flags & BWD1)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X2);
                if( p->b_error_flag )
                {
                    return -1;
                }

                k = i;
                for( i = 0 ; i < k; i++ )
                {
                    if( ref[i] > 1 )
                    {
                        p->b_error_flag = 1;
                
                        return -1;
                    }    
                }
                
                if(flags & FWD0)
                {
                    mv_pred(p, MV_FWD_X0, MV_FWD_C2, MV_PRED_TOP,  BLK_16X8, ref[kk++]+i_ref_offset, MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                    CP32(p_mvd[FWD][MVD_X1], p_mvd[FWD][MVD_X0]);
                    p_ref[FWD][REF_X1] = p_ref[FWD][REF_X0];
                }
                
                if(flags & SYM0)
                {    
                    mv_pred_sym(p, &p->mv[MV_FWD_X0], BLK_16X8,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & FWD1)
                {
                    mv_pred(p, MV_FWD_X2, MV_FWD_A1, MV_PRED_LEFT, BLK_16X8, ref[kk++]+i_ref_offset, MVD_X2);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                      
                    CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X2]);
                    p_ref[FWD][REF_X3] = p_ref[FWD][REF_X2];
                }
                if(flags & SYM1)
                {    
                    mv_pred_sym(p, &p->mv[MV_FWD_X2], BLK_16X8,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & BWD0)
                {
                    mv_pred(p, MV_BWD_X0, MV_BWD_C2, MV_PRED_TOP,  BLK_16X8, ref[kk++], MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                      
                    CP32(p_mvd[BWD][MVD_X1], p_mvd[BWD][MVD_X0]);
                    p_ref[BWD][REF_X1] = p_ref[BWD][REF_X0];
                }
                if(flags & BWD1)
                {
                    mv_pred(p, MV_BWD_X2, MV_BWD_A1, MV_PRED_LEFT, BLK_16X8, ref[kk++], MVD_X2);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                      
                    CP32(p_mvd[BWD][MVD_X3], p_mvd[BWD][MVD_X2]);
                    p_ref[BWD][REF_X3] = p_ref[BWD][REF_X2];
                }
            } 
            else  /* 8x16 macroblock types */
            {
                int i = 0, k = 0;
                
                if (flags & FWD0)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X0);
                if (flags & FWD1)
                	ref[i++] = p->bs_read_ref_b(p, FWD, REF_X1);
                if (flags & BWD0)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X0);
                if (flags & BWD1)
                	ref[i++] = p->bs_read_ref_b(p, BWD, REF_X1);
                if( p->b_error_flag )
                {
                    return -1;
                }
                
                k = i;
                for( i = 0 ; i < k; i++ )
                {
                    if( ref[i] > 1 )
                    {
                        p->b_error_flag = 1;
                
                        return -1;
                    }    
                }
                           
                if(flags & FWD0)
                {
                    mv_pred(p, MV_FWD_X0, MV_FWD_B3, MV_PRED_LEFT, BLK_8X16, ref[kk++]+i_ref_offset, MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	 p_ref[FWD][REF_X2] = p_ref[FWD][REF_X0];
                }
                if(flags & SYM0)
                {    
                    mv_pred_sym(p, &p->mv[MV_FWD_X0], BLK_8X16,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                if(flags & FWD1)
                {
                    mv_pred(p, MV_FWD_X1, MV_FWD_C2, MV_PRED_TOPRIGHT,BLK_8X16, ref[kk++]+i_ref_offset, MVD_X1);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	CP32(p_mvd[FWD][MVD_X3], p_mvd[FWD][MVD_X1]);
                	p_ref[FWD][REF_X3] = p_ref[FWD][REF_X1];
                }
                if(flags & SYM1)
                {
                    mv_pred_sym(p, &p->mv[MV_FWD_X1], BLK_8X16,ref[kk-1]);
                    if(p->b_error_flag)
                    {
                        return -1;
                    }
                }
                
                if(flags & BWD0)
                {
                    mv_pred(p, MV_BWD_X0, MV_BWD_B3, MV_PRED_LEFT, BLK_8X16, ref[kk++], MVD_X0);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	p_ref[BWD][REF_X2] = p_ref[BWD][REF_X0];
                }
                if(flags & BWD1)
                {
                	mv_pred(p, MV_BWD_X1, MV_BWD_C2, MV_PRED_TOPRIGHT,BLK_8X16, ref[kk++], MVD_X1);
                    if( p->b_error_flag )
                    {
                        return -1;
                    }
                	CP32(p_mvd[BWD][MVD_X3], p_mvd[BWD][MVD_X1]);
                	p_ref[BWD][REF_X3] = p_ref[BWD][REF_X1];
                }
            }
    }

    /* save current mv, mvd, refidx into a frame, ignore neighbor info */
    /* mv copy */
    memcpy(p->mv_tab[p->i_mb_index], p->mv, 24*sizeof(cavs_vector));

    /* mvd copy */
    memcpy(p->p_mvd_tab[p->i_mb_index], p->p_mvd, 2*6*2*sizeof(int16_t));

    /* refidx copy */
    memcpy(p->p_ref_tab[p->i_mb_index], p->p_ref, 2*9*sizeof(int8_t));
    
#undef FWD
#undef BWD
#undef MVD_X3
#undef MVD_X2
#undef MVD_X1
#undef MVD_X0
#undef REF_X3
#undef REF_X2
#undef REF_X1
#undef REF_X0

#if B_MB_WEIGHTING
	p->weighting_prediction = 0;
#endif
    if (i_mb_type != B_SKIP)
    {
#if B_MB_WEIGHTING
        if ( p->sh.b_slice_weighting_flag && p->sh.b_mb_weighting_flag )
        {
            if( p->ph.b_aec_enable )
            {
                p->weighting_prediction = cavs_cabac_get_mb_weighting_prediction( p );
            }
            else
                p->weighting_prediction = cavs_bitstream_get_bit1(&p->s);   //weighting_prediction
        }
#else 
        if (p->sh.b_mb_weighting_flag)
        {
            cavs_bitstream_get_bit1(&p->s); /* weighting_prediction */
        }
#endif

    }

#if B_MB_WEIGHTING
	p->weighting_prediction_tab[p->i_mb_index] = p->weighting_prediction;
#endif
    
    p->i_qp_tab[p->i_mb_index] = p->i_qp;  /* init for skip */
    if(i_mb_type != B_SKIP)
    {
        /* get coded block pattern */
        p->i_cbp_tab[p->i_mb_index] = p->bs_read[SYNTAX_INTER_CBP](p);
        if( p->b_error_flag )
        {
            return -1;
        }
        p->i_cbp = p->i_cbp_tab[p->i_mb_index];
        if( p->i_cbp_tab[p->i_mb_index] > 63) 
        {
            p->b_error_flag = 1;

            return -1;
        }
        
        /* get quantizer */
        p->i_qp_tab[p->i_mb_index] = p->i_qp;
        if(p->i_cbp_tab[p->i_mb_index]  && !p->b_fixed_qp)
        {  
            int delta_qp =  p->bs_read[SYNTAX_DQP](p);
            
			p->i_qp_tab[p->i_mb_index] = (p->i_qp_tab[p->i_mb_index] -MIN_QP + delta_qp + (MAX_QP-MIN_QP+1))%(MAX_QP-MIN_QP+1)+MIN_QP;

            p->i_qp = p->i_qp_tab[p->i_mb_index];

            if( p->i_qp < 0 || p->i_qp > 63  || p->b_error_flag )
            {
				p->b_error_flag = 1;

                return -1;  
            }
        }
        else
            p->i_last_dqp = 0;
       
        i_ret = get_residual_inter_aec_opt(p, i_mb_type);
        if( i_ret == -1 )
            return -1;
    }
    
    return aec_next_mb(p);
}

static int get_mb_i_rec_opt( cavs_decoder *p )
{
    uint8_t i_top[18];
    uint8_t *p_left = NULL;
    uint8_t *p_d;

    static const uint8_t scan5x5[4][4] = {
    	{6, 7, 11, 12},
    	{8, 9, 13, 14},
    	{16, 17, 21, 22},
    	{18, 19, 23, 24}
    };

    int i;
    DECLARE_ALIGNED_16(uint8_t edge_8x8[33]);
    int i_qp_cb = 0, i_qp_cr = 0;
    int ret = 0;

	if(p->i_cbp_tab[p->i_mb_index] < 0 || p->i_cbp_tab[p->i_mb_index] > 63 
		|| p->i_qp_tab[p->i_mb_index] < 0
		|| p->i_qp_tab[p->i_mb_index] > 63 )
	{
		return -1;
	}

    for( i = 0; i < 4; i++ )
    {
        int i_mode;

        p_d = p->p_y + p->i_luma_offset[i];

        load_intra_pred_luma(p, i_top, &p_left, i);

        {
            i_mode = p->i_intra_pred_mode_y_tab[p->i_mb_index][scan5x5[i][0]];

            if( i_mode > 7 || i_mode < 0 )
            {
                return -1;
            }
            
            p->cavs_intra_luma[i_mode](p_d, edge_8x8, p->cur.i_stride[0], i_top, p_left);

            if(p->i_cbp_tab[p->i_mb_index] & (1<<i))
            {
                ret = get_residual_block_rec_opt( p, intra_2dvlc, 1, p->i_qp_tab[p->i_mb_index], p_d, p->cur.i_stride[0], 0, i);
                if(ret == -1)
                {
                	return -1;
                }
            }
        }
    }

    load_intra_pred_chroma(p);

    if(p->i_pred_mode_chroma_tab[p->i_mb_index]>6 || p->i_pred_mode_chroma_tab[p->i_mb_index] < 0)
    {
        return -1;
    }

    p->cavs_intra_chroma[p->i_pred_mode_chroma_tab[p->i_mb_index]](p->p_cb, p->mb.i_neighbour, p->cur.i_stride[1], &p->p_top_border_cb[p->i_mb_x*10], p->i_left_border_cb);
    p->cavs_intra_chroma[p->i_pred_mode_chroma_tab[p->i_mb_index]](p->p_cr, p->mb.i_neighbour, p->cur.i_stride[2], &p->p_top_border_cr[p->i_mb_x*10], p->i_left_border_cr);
    
    if( p->b_weight_quant_enable && !p->ph.chroma_quant_param_disable )
    { 
        i_qp_cb = clip3_int( p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_u, 0, 63 );
        i_qp_cr = clip3_int( p->i_qp_tab[p->i_mb_index] + p->ph.chroma_quant_param_delta_v, 0, 63 );
    }

    if(p->i_cbp_tab[p->i_mb_index] & (1<<4))
    {
        ret = get_residual_block_rec_opt( p, chroma_2dvlc, 0, 
                chroma_qp[ p->b_weight_quant_enable 
                && !p->ph.chroma_quant_param_disable? i_qp_cb : p->i_qp_tab[p->i_mb_index]],
                p->p_cb,p->cur.i_stride[1], 1, 4);
        if(ret == -1)
        {
            return -1;
        }
    }
    if(p->i_cbp_tab[p->i_mb_index] & (1<<5))
    {
        ret = get_residual_block_rec_opt( p, chroma_2dvlc, 0, 
                 chroma_qp[ p->b_weight_quant_enable 
                 && !p->ph.chroma_quant_param_disable? i_qp_cr : p->i_qp_tab[p->i_mb_index]],
                 p->p_cr, p->cur.i_stride[2], 1, 5);
        if(ret == -1)
        {
            return -1;
        }
    }

    filter_mb(p, I_8X8);
    p->mv[MV_FWD_X0] = MV_INTRA;
    copy_mvs(&p->mv[MV_FWD_X0], BLK_16X16);
    p->mv[MV_BWD_X0] = MV_INTRA;
    copy_mvs(&p->mv[MV_BWD_X0], BLK_16X16);

    return rec_next_mb(p); 
}

static int get_mb_p_rec_opt( cavs_decoder *p )
{
    int i_mb_type = p->i_mb_type_tab[p->i_mb_index];
    int ret = 0;

#define FWD		0
#define MVD_X0  1
#define MVD_X1	2
#define MVD_X2  4
#define MVD_X3	5
#define REF_X0  4
#define REF_X1	5
#define REF_X2	7
#define REF_X3  8

    p->mv[MV_FWD_B2] = p->mv_tab[p->i_mb_index][MV_FWD_B2];
    p->mv[MV_FWD_B3] = p->mv_tab[p->i_mb_index][MV_FWD_B3];
    p->mv[MV_FWD_A1] = p->mv_tab[p->i_mb_index][MV_FWD_A1];/* following four used for deblock */
    p->mv[MV_FWD_X0] = p->mv_tab[p->i_mb_index][MV_FWD_X0];
    p->mv[MV_FWD_X1] = p->mv_tab[p->i_mb_index][MV_FWD_X1];
    p->mv[MV_FWD_A3] = p->mv_tab[p->i_mb_index][MV_FWD_A3];
    p->mv[MV_FWD_X2] = p->mv_tab[p->i_mb_index][MV_FWD_X2];
    p->mv[MV_FWD_X3] = p->mv_tab[p->i_mb_index][MV_FWD_X3];

    /** mvd cache
    0:    A1  X0  X1
    3:    A3  X2  X3   */
    CP32(p->p_mvd[FWD][MVD_X0], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X0]);
    CP32(p->p_mvd[FWD][MVD_X1], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X1]);
    CP32(p->p_mvd[FWD][MVD_X2], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X2]);
    CP32(p->p_mvd[FWD][MVD_X3], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X3]);

    /** ref cache
    0:    D3  B2  B3
    3:    A1  X0  X1
    6:    A3  X2  X3   */
    p->p_ref[FWD][REF_X0] = p->p_ref_tab[p->i_mb_index][FWD][REF_X0];    
    p->p_ref[FWD][REF_X1] = p->p_ref_tab[p->i_mb_index][FWD][REF_X1];
    p->p_ref[FWD][REF_X2] = p->p_ref_tab[p->i_mb_index][FWD][REF_X2];
    p->p_ref[FWD][REF_X3] = p->p_ref_tab[p->i_mb_index][FWD][REF_X3];
 
#undef FWD
#undef MVD_X3
#undef MVD_X2
#undef MVD_X1
#undef MVD_X0
#undef REF_X3
#undef REF_X2
#undef REF_X1
#undef REF_X0

#if B_MB_WEIGHTING
	p->weighting_prediction = p->weighting_prediction_tab[p->i_mb_index];
#endif
    
    inter_pred(p, i_mb_type);

    if (i_mb_type != P_SKIP)
    {
        ret = get_residual_inter_rec_opt(p, i_mb_type);
        if(ret == -1)
        {
            return -1;
        }
    }

    filter_mb(p, i_mb_type);

    return rec_next_mb(p);   
}


static int get_mb_b_rec_opt( cavs_decoder *p )
{   
    int i_mb_type = p->i_mb_type_tab[p->i_mb_index];//p->i_mb_type;
    int ret = 0;

#define FWD		0
#define BWD		1
#define MVD_X0	1
#define MVD_X1	2
#define MVD_X2	4
#define MVD_X3	5
#define REF_X0  4
#define REF_X1  5
#define REF_X2  7
#define REF_X3  8

    p->mv[MV_FWD_B2] = p->mv_tab[p->i_mb_index][MV_FWD_B2];
    p->mv[MV_FWD_B3] = p->mv_tab[p->i_mb_index][MV_FWD_B3];
    p->mv[MV_FWD_A1] = p->mv_tab[p->i_mb_index][MV_FWD_A1];/* following four used for deblock */
    p->mv[MV_FWD_X0] = p->mv_tab[p->i_mb_index][MV_FWD_X0];
    p->mv[MV_FWD_X1] = p->mv_tab[p->i_mb_index][MV_FWD_X1];
    p->mv[MV_FWD_A3] = p->mv_tab[p->i_mb_index][MV_FWD_A3];
    p->mv[MV_FWD_X2] = p->mv_tab[p->i_mb_index][MV_FWD_X2];
    p->mv[MV_FWD_X3] = p->mv_tab[p->i_mb_index][MV_FWD_X3];

    p->mv[MV_BWD_B2] = p->mv_tab[p->i_mb_index][MV_BWD_B2];
    p->mv[MV_BWD_B3] = p->mv_tab[p->i_mb_index][MV_BWD_B3];
    p->mv[MV_BWD_A1] = p->mv_tab[p->i_mb_index][MV_BWD_A1];/* following four used for deblock */
    p->mv[MV_BWD_X0] = p->mv_tab[p->i_mb_index][MV_BWD_X0];
    p->mv[MV_BWD_X1] = p->mv_tab[p->i_mb_index][MV_BWD_X1];
    p->mv[MV_BWD_A3] = p->mv_tab[p->i_mb_index][MV_BWD_A3];
    p->mv[MV_BWD_X2] = p->mv_tab[p->i_mb_index][MV_BWD_X2];
    p->mv[MV_BWD_X3] = p->mv_tab[p->i_mb_index][MV_BWD_X3];

    /** mvd cache
    0:    A1  X0  X1
    3:    A3  X2  X3   */
    CP32(p->p_mvd[FWD][MVD_X0], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X0]);
    CP32(p->p_mvd[FWD][MVD_X1], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X1]);
    CP32(p->p_mvd[FWD][MVD_X2], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X2]);
    CP32(p->p_mvd[FWD][MVD_X3], p->p_mvd_tab[p->i_mb_index][FWD][MVD_X3]);
    CP32(p->p_mvd[BWD][MVD_X0], p->p_mvd_tab[p->i_mb_index][BWD][MVD_X0]);
    CP32(p->p_mvd[BWD][MVD_X1], p->p_mvd_tab[p->i_mb_index][BWD][MVD_X1]);
    CP32(p->p_mvd[BWD][MVD_X2], p->p_mvd_tab[p->i_mb_index][BWD][MVD_X2]);
    CP32(p->p_mvd[BWD][MVD_X3], p->p_mvd_tab[p->i_mb_index][BWD][MVD_X3]);

    /** ref cache
    0:    D3  B2  B3
    3:    A1  X0  X1
    6:    A3  X2  X3   */
    p->p_ref[FWD][REF_X0] = p->p_ref_tab[p->i_mb_index][FWD][REF_X0];    
    p->p_ref[FWD][REF_X1] = p->p_ref_tab[p->i_mb_index][FWD][REF_X1];
    p->p_ref[FWD][REF_X2] = p->p_ref_tab[p->i_mb_index][FWD][REF_X2];
    p->p_ref[FWD][REF_X3] = p->p_ref_tab[p->i_mb_index][FWD][REF_X3];
    p->p_ref[BWD][REF_X0] = p->p_ref_tab[p->i_mb_index][BWD][REF_X0];    
    p->p_ref[BWD][REF_X1] = p->p_ref_tab[p->i_mb_index][BWD][REF_X1];
    p->p_ref[BWD][REF_X2] = p->p_ref_tab[p->i_mb_index][BWD][REF_X2];
    p->p_ref[BWD][REF_X3] = p->p_ref_tab[p->i_mb_index][BWD][REF_X3];

#undef FWD
#undef BWD
#undef MVD_X3
#undef MVD_X2
#undef MVD_X1
#undef MVD_X0
#undef REF_X3
#undef REF_X2
#undef REF_X1
#undef REF_X0

#if B_MB_WEIGHTING
	p->weighting_prediction = p->weighting_prediction_tab[p->i_mb_index];
#endif

    inter_pred(p, i_mb_type);

    if(i_mb_type != B_SKIP)
    {
        ret = get_residual_inter_rec_opt(p, i_mb_type);
        if(ret == -1)
        {
            return -1;
        }
    }
    
    filter_mb(p, i_mb_type);
   
    return rec_next_mb(p);
}
