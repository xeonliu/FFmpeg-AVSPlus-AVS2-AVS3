/*
 * DRA audio decoder.
 * Copyright (c) 2021 maliwen2015/ffmpeg_cavs_dra 
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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <memory.h>
#include <time.h>
#include <math.h>
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "internal.h"

#include "dradec.h"

#define __min(a,b) (((a) < (b)) ? (a) : (b))

typedef struct DRAContext
{
	void *pDRADecoder;
}DRAContext;

static av_cold int dra_decode_init(AVCodecContext *avctx)
{
    struct DRAContext *s = avctx->priv_data;

	avctx->frame_size = 1024;
	av_channel_layout_default(&avctx->ch_layout, 2);
	avctx->sample_fmt = AV_SAMPLE_FMT_S16;

	s->pDRADecoder = DRADecCreate();
	if(s->pDRADecoder == NULL)
	{
		return 1;
	}

	return 0;
}

static int dra_decode_frame(AVCodecContext *avctx, AVFrame *data,
                                 int *got_frame_ptr, AVPacket *avpkt)
{
	int ret = 0;
	
    uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
	AVFrame *frame     = data;
    int dec_len = 0;
	DRAFrameInfo dra_frame_info;
	
    struct DRAContext *s = avctx->priv_data;
	
	unsigned char *pcm_data = NULL;
	int pcm_len = 0;

    *got_frame_ptr = 0;

	if(buf == NULL || buf_size == 0)
	{
		ret = DRADecRecvFrame(s->pDRADecoder, &pcm_data, &pcm_len);
		if(ret == 0)
		{
			DRADecGetFrameInfo(s->pDRADecoder, &dra_frame_info);
			avctx->sample_rate = dra_frame_info.nSampleRate;
			av_channel_layout_default(&avctx->ch_layout, dra_frame_info.nChannels);
			if(avctx->ch_layout.nb_channels != 0)
			{
				avctx->frame_size = pcm_len / (avctx->ch_layout.nb_channels * 2);
			}

			frame->ch_layout = avctx->ch_layout;
			frame->sample_rate = avctx->sample_rate;
			frame->nb_samples = avctx->frame_size;
			frame->format = avctx->sample_fmt;

		    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
		        return ret;

		    memcpy(frame->data[0], pcm_data, pcm_len);

		    *got_frame_ptr = 1;
		}
		dec_len = buf_size;
	}
	else
	{
		ret = DRADecRecvFrame(s->pDRADecoder, &pcm_data, &pcm_len);
		if(ret == 0)
		{
			DRADecGetFrameInfo(s->pDRADecoder, &dra_frame_info);
			avctx->sample_rate = dra_frame_info.nSampleRate;
			av_channel_layout_default(&avctx->ch_layout, dra_frame_info.nChannels);
			if(avctx->ch_layout.nb_channels != 0)
			{
				avctx->frame_size = pcm_len / (avctx->ch_layout.nb_channels * 2);
			}

			frame->ch_layout = avctx->ch_layout;
			frame->sample_rate = avctx->sample_rate;
			frame->nb_samples = avctx->frame_size;
			frame->format = avctx->sample_fmt;

		    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
		        return ret;

		    memcpy(frame->data[0], pcm_data, pcm_len);

		    *got_frame_ptr = 1;

		    dec_len = 0;
		}
		else
		{
			ret = DRADecSendData(s->pDRADecoder, buf, buf_size);
			if(ret != 0)
			{
				return AVERROR(AVERROR_BUG);
			}

			ret = DRADecRecvFrame(s->pDRADecoder, &pcm_data, &pcm_len);
			if(ret == 0)
			{
				DRADecGetFrameInfo(s->pDRADecoder, &dra_frame_info);
		    	avctx->sample_rate = dra_frame_info.nSampleRate;
				av_channel_layout_default(&avctx->ch_layout, dra_frame_info.nChannels);
				if(avctx->ch_layout.nb_channels != 0)
				{
					avctx->frame_size = pcm_len / (avctx->ch_layout.nb_channels * 2);
				}

				frame->ch_layout = avctx->ch_layout;

				frame->ch_layout = avctx->ch_layout;
				frame->sample_rate = avctx->sample_rate;
		    	frame->nb_samples = avctx->frame_size;
		    	frame->format = avctx->sample_fmt;

			    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
			        return ret;

			    memcpy(frame->data[0], pcm_data, pcm_len);

			    *got_frame_ptr = 1;
			}

			dec_len = buf_size;
		}
	}
	
	return dec_len;
}

static av_cold int dra_decode_end(AVCodecContext *avctx)
{
	struct DRAContext *s = avctx->priv_data;

	if(s->pDRADecoder)
	{
		DRADecDestroy(s->pDRADecoder);
		s->pDRADecoder = NULL;
	}
	
    return 0;
}

const FFCodec ff_libdra_decoder = {
    .p.name           = "libdra",    
    .p.long_name    = NULL_IF_CONFIG_SMALL("dra audio decoder"),
    .p.type           = AVMEDIA_TYPE_AUDIO,
    .p.id             = AV_CODEC_ID_DRA,
    .priv_data_size = sizeof(DRAContext),
    .init           = dra_decode_init,
    .close          = dra_decode_end,
    FF_CODEC_DECODE_CB(dra_decode_frame),
    .p.capabilities   = AV_CODEC_CAP_CHANNEL_CONF | AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .p.sample_fmts     = (const enum AVSampleFormat[]) { AV_SAMPLE_FMT_S16,
                                                     AV_SAMPLE_FMT_NONE },
};
