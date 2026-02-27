/*****************************************************************************
* dradec.h: the header file of dra decoder 
* Copyright (c) 2021 maliwen2015/ffmpeg_cavs_dra
*****************************************************************************
*/

#pragma once


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct DRAFrameInfo
{
	int nChannels;
	int nSampleRate;
	int nFrameSize;
}DRAFrameInfo;

void* DRADecCreate(void);
void DRADecDestroy(void* pDecoder);
int DRADecGetFrameInfo(void* pDecoder, DRAFrameInfo* pDRAFrameInfo);
int DRADecSendData(void* pDecoder, unsigned char* pData, int nLen);
int DRADecRecvFrame(void* pDecoder, unsigned char** ppPCMData, int* nPCMLen);

#ifdef __cplusplus
}
#endif
