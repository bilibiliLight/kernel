/* nuc970_jpeg.h
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __NUC970_JPEG_H
#define __NUC970_JPEG_H

#include <linux/videodev2.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <mach/regs-jpeg.h>

typedef enum jpeg_state {
    JPEG_IDLE               = 1,
    JPEG_ENCODING           = 2,
    JPEG_ENCODED_IMAGE      = 3,
    JPEG_DECODING           = 9,
    JPEG_DECODED_HEADER     = 10,
    JPEG_DECODED_IMAGE      = 11,
    JPEG_DECODE_ERROR       = 12,
    JPEG_MEM_SHORTAGE       = 15,
    JPEG_DECODE_PARAM_ERROR = 13,
    JPEG_ENCODE_PARAM_ERROR = 14,
    JPEG_CLOSED             = 20,
} jpeg_state_t;


#define IS_ENCODE(state)      (state >=JPEG_ENCODING && state <JPEG_DECODING)
#define IS_DECODE(state)      (state >=JPEG_DECODING)
#define IS_DECODED(state)     ((state == JPEG_DECODED_IMAGE) || (state==JPEG_DECODE_ERROR))
#define IS_ENCODED(state)     (state == JPEG_ENCODED_IMAGE)
#define IS_FINISHED(state)    (IS_DECODED(state) || IS_ENCODED(state))

#define JPEG_CMD_NONE                0
#define JPEG_CMD_YUV422_TO_RGB555    1
#define JPEG_CMD_YUV422_TO_RGB565    2
#define JPEG_CMD_EXIT                3

/* Define for Decode Output Format */
#define DRVJPEG_DEC_PRIMARY_PLANAR_YUV      0x8021    // (PLANAR_ON | PDHTAB | DHEND)
#define DRVJPEG_DEC_PRIMARY_PACKET_YUV422   0x0021    // (PDHTAB | DHEND)
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB555   0x004021  // (PDHTAB | DHEND | ORDER)
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB565   0x006021  // (PDHTAB | DHEND | RGB555_565 | ORDER )
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB555R1 0x404021  // (PDHTAB | DHEND | ORDER)
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB565R1 0x406021  // (PDHTAB | DHEND | RGB555_565 | ORDER )
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB565R2 0x806021  // (PDHTAB | DHEND | RGB555_565 | ORDER )
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB555R2 0x804021  // (PDHTAB | DHEND | ORDER)
#define DRVJPEG_DEC_PRIMARY_PACKET_RGB888   0x14021   // (PDHTAB | DHEND | ORDER | ARGB888)

#define DRVJPEG_DEC_SOFTWARE_RGB555         0xFFF0    // (PDHTAB | DHEND | ORDER)
#define DRVJPEG_DEC_SOFTWARE_RGB565         0xFFF1    // (PDHTAB | DHEND | ORDER)

#define DRVJPEG_DEC_THUMBNAIL_PLANAR_YUV    0x8011    // (PLANAR_ON | DTHB | PDHTAB)
#define DRVJPEG_DEC_THUMBNAIL_PACKET_YUV422 0x0031    // (DTHB | PDHTAB | DHEND)
#define DRVJPEG_DEC_THUMBNAIL_PACKET_RGB555 0x4031    // (DTHB | PDHTAB | DHEND | ORDER)

//Define for Encode Image Format
#define DRVJPEG_ENC_PRIMARY_YUV420     0xA0
#define DRVJPEG_ENC_PRIMARY_YUV422     0xA8
#define DRVJPEG_ENC_PRIMARY_GRAY       0xA1
#define DRVJPEG_ENC_THUMBNAIL_YUV420   0x90
#define DRVJPEG_ENC_THUMBNAIL_YUV422   0x98
#define DRVJPEG_ENC_THUMBNAIL_GRAY     0x91

//Define for Decode Image Format
#define DRVJPEG_DEC_YUV420             0x000
#define DRVJPEG_DEC_YUV422             0x100
#define DRVJPEG_DEC_YUV444             0x200
#define DRVJPEG_DEC_YUV411             0x300
#define DRVJPEG_DEC_GRAY               0x400
#define DRVJPEG_DEC_YUV422T            0x500

//Define for Encode input Format
#define DRVJPEG_ENC_SRC_PLANAR         0
#define DRVJPEG_ENC_SRC_PACKET         1

//Define for Encode Image Header
#define DRVJPEG_ENC_PRIMARY_DRI        P_DRI
#define DRVJPEG_ENC_PRIMARY_QTAB       P_QTAB
#define DRVJPEG_ENC_PRIMARY_HTAB       P_HTAB
#define DRVJPEG_ENC_PRIMARY_JFIF       P_JFIF
#define DRVJPEG_ENC_THUMBNAIL_DRI      T_DRI
#define DRVJPEG_ENC_THUMBNAIL_QTAB     T_QTAB
#define DRVJPEG_ENC_THUMBNAIL_HTAB     T_HTAB
#define DRVJPEG_ENC_THUMBNAIL_JFIF     T_JFIF

#define JPEG_DECIPW_BUFFER_EMPTY       1
#define JPEG_DECIPW_BUFFER_NOT_EMPTY   0

#define JPEG_TRIGGER                   _IOW('v',120, struct jpeg_param)
#define JPEG_S_PARAM                   _IOW('v',121, struct jpeg_param)
#define JPEG_G_PARAM                   _IOW('v',122, struct jpeg_param)
#define JPEG_STATE                     _IOR('v',123, struct jpeg_param)
#define JPEG_G_INFO                    _IOR('v',124, struct jpeg_info)
#define JPEG_DECIPW_BUFFER_STATE       _IOR('v',125, __u32)
#define JPEG_G_DECIPW_BUFFER_SIZE      _IOR('v',126, __u32)
#define JPEG_DECODE_RESUME             _IOR('v',127, __u32)        
#define JPEG_S_DECOPW                  _IOW('v',128,  __u32)   
#define JPEG_GET_JPEG_BUFFER           _IOR('v',129, __u32)
#define JPEG_SET_ENCOCDE_RESERVED      _IOW('v',130,  __u32)
//#define JPEG_SET_ENC_SRC_FROM_VIN _IOW('v',131,  __u32)
#define JPEG_FLUSH_CACHE               _IOW('v',132,  __u32)
#define JPEG_SET_ENC_THUMBNAIL         _IOW('v',133, __u32)
#define JPEG_GET_ENC_THUMBNAIL_SIZE    _IOW('v',134, __u32)
#define JPEG_GET_ENC_THUMBNAIL_OFFSET  _IOW('v',135, __u32)
#define JPEG_SET_ENC_STRIDE            _IOW('v',136, __u32)
#define JPEG_SET_ENC_USER_QTABLE0      _IOW('v',137, __u32)
#define JPEG_SET_ENC_USER_QTABLE1      _IOW('v',138, __u32)
#define JPEG_ACTIVE_ENC_DEFAULTQTABLE  _IOW('v',139, __u32)
#define JPEG_ACTIVE_ENC_USER_QTABLE    _IOW('v',140, __u32)
#define JPEG_SET_ENC_USER_YADDRESS     _IOW('v',141,  __u32)
#define JPEG_SET_ENC_USER_UADDRESS     _IOW('v',142,  __u32)
#define JPEG_SET_ENC_USER_VADDRESS     _IOW('v',143,  __u32)
#define JPEG_GET_JPEG_BUFFER_PADDR     _IOR('v',144, __u32)
#define JPEG_DECODE_TO_FRAME_BUFFER    _IOR('v',145, __u32)


#define DRVJPEG_ENC_PRIMARY                 0
#define DRVJPEG_ENC_THUMBNAIL               1

//Define for Scaling
#define DRVJPEG_ENC_UPSCALE_MODE            0
#define DRVJPEG_DEC_PACKET_DOWNSCALE_MODE   1
#define DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE   2
#define DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE   3
    
//Define for Encode input Format
#define DRVJPEG_ENC_SOURCE_PLANAR   0
#define DRVJPEG_ENC_SOURCE_PACKET   1

//Version definition


//Export functions
#define _DRVJPEG_SET_YADDR(u32Address)              writel(u32Address, REG_JYADDR0)
#define _DRVJPEG_SET_UADDR(u32Address)              writel(u32Address, REG_JUADDR0)
#define _DRVJPEG_SET_VADDR(u32Address)              writel(u32Address, REG_JVADDR0)
#define _DRVJPEG_GET_YADDR()                        readl(REG_JYADDR0)
#define _DRVJPEG_GET_UADDR()                        readl(REG_JUADDR0)
#define _DRVJPEG_GET_VADDR()                        readl(REG_JVADDR0)
#define _DRVJPEG_SET_YSTRIDE(u32Stride)             writel(u32Stride, REG_JYSTRIDE)
#define _DRVJPEG_SET_USTRIDE(u32Stride)             writel(u32Stride, REG_JUSTRIDE)
#define _DRVJPEG_SET_VSTRIDE(u32Stride)             writel(u32Stride, REG_JVSTRIDE)
#define _DRVJPEG_GET_YSTRIDE()                      readl(REG_JYSTRIDE) 
#define _DRVJPEG_GET_USTRIDE()                      readl(REG_JUSTRIDE)
#define _DRVJPEG_GET_VSTRIDE()                      readl(REG_JVSTRIDE)
#define _DRVJPEG_SET_BITSTREAM_ADDR(u32Address)     writel(u32Address, REG_JIOADDR0)
#define _DRVJPEG_GET_BITSTREAM_ADDR()               readl(REG_JIOADDR0)
#define _DRVJPEG_SET_ENC_DEC(u8Mode)                writel((readl(REG_JMCR) & ~ENC_DEC) | (u8Mode << 7), REG_JMCR)

//Encode
#define _DRVJPEG_GET_ENC_PRIMARY_BITSTREAM_SIZE()   readl(REG_JPRI_SIZE)
#define _DRVJPEG_GET_ENC_THUMBNAIL_BITSTREAM_SIZE() readl(REG_JTHB_SIZE)
#define _DRVJPEG_SET_SOURCE_IMAGE_HEIGHT(u16Size)   writel(u16Size, REG_JSRCH)
#define _DRVJPEG_GET_SOURCE_IMAGE_HEIGHT()          readl(REG_JSRCH)
#define _DRVJPEG_ENC_ENABLE_UPSCALING()             writel(readl(REG_JPSCALU) | JPSCALU_8X, REG_JPSCALU)
#define _DRVJPEG_ENC_DISABLE_UPSCALING()            writel(readl(REG_JPSCALU) & ~JPSCALU_8X, REG_JPSCALU)
#define _DRVJPEG_ENC_ISENABLE_UPSCALING()           ((readl(REG_JPSCALU) & JPSCALU_8X) >> 6)
#define _DRVJPEG_ENC_SET_HEADER_CONTROL(u8Control)  writel(u8Control, REG_JHEADER)
#define _DRVJPEG_ENC_GET_HEADER_CONTROL()           readl(REG_JHEADER)
#define _DRVJPEG_ENC_SET_RDI_VALUE(u8Value)         writel(u8Value, REG_JPRST)
#define _DRVJPEG_ENC_GET_RDI_VALUE()                readl(REG_JPRST)

//Decode
#define _DRVJPEG_DEC_ENABLE_DOWNSCALING()           writel(PSX_ON, REG_JPSCALD) 
#define _DRVJPEG_DEC_ISENABLE_DOWNSCALING()         ((readl(REG_JPSCALD) & PSX_ON) >> 15) 
#define _DRVJPEG_DEC_DISABLE_DOWNSCALING()          writel(~PSX_ON, REG_JPSCALD)
#define _DRVJPEG_DEC_GET_DECODED_IMAGE_FORMAT()     (readl(REG_JITCR) & DYUV_MODE)
#define _DRVJPEG_DEC_ENABLE_LOW_PASS_FILTER()       writel(readl(REG_JPSCALD) | PS_LPF_ON, REG_JPSCALD)
#define _DRVJPEG_DEC_DISABLE_LOW_PASS_FILTER()      writel(readl(REG_JPSCALD) & ~PS_LPF_ON, REG_JPSCALD)
#define _DRVJPEG_DEC_ISENABLE_LOW_PASS_FILTER()     ((readl(REG_JPSCALD) & PS_LPF_ON) >> 14)
#define _DRVJPEG_DEC_SET_INPUT_WAIT(u16Size)        writel(0x00400008 | ((u16Size & 0x3FF)<< 8), REG_JMACR)
#define _DRVJPEG_DEC_RESUME_INPUT_WAIT()            writel(readl(REG_JMCR)|RESUMEI, REG_JMCR)
#define _DRVJPEG_DEC_DISABLE_WINDOWDECODE()         writel(readl(REG_JMCR) & ~(WIN_DEC), REG_JMCR)
//Interrupt
#define _DRVJPEG_INT_ENABLE(u32Intflag)             writel(u32Intflag, REG_JINTCR)
#define _DRVJPEG_INT_DISABLE(u32Intflag)            writel(readl (REG_JINTCR) & ~(u32Intflag), REG_JINTCR)
#define _DRVJPEG_GET_INT_STATUS()                   readl(REG_JINTCR)
#define _DRVJPEG_CLEAR_INT(u32Intflag)              writel((readl (REG_JINTCR) & ~0xFF) | u32Intflag, REG_JINTCR)


typedef struct jpeg_param {
    /*for get/set parameters*/
    __u32   vaddr_src;
    __u32   decopw_vaddr;
    __u32   decopw_en;
    __u32   decopw_TargetBuffersize;
    __u32   vaddr_dst;
    __u32   paddr_src;
    __u32   paddr_dst;
    __u32   src_bufsize;
    __u32   dst_bufsize;
    __u32   dec_stride;
    __u32   windec_en;
    __u32   windec_mcux_start;
    __u32   windec_mcux_end;
    __u32   windec_mcuy_start;
    __u32   windec_mcuy_end;
    __u32   windec_stride;
    /*for set only*/
    __u32   decode_output_format;   // DRVJPEG_DEC_PRIMARY_PACKET_YUV422,
    __u32   encode;                 // 1 decode, 0 encode 
    __u32   scale;                  // 1 enable, 0 disable
    /* encode / decode scale */
    __u32   scaled_width;
    __u32   scaled_height;
    __u32   decInWait_buffer_size;  
    /*encode parameters for set only*/
    __u32   encode_width;           /* the width that will be encoded image raw data*/
    __u32   encode_height;          /* the height that will be encoded image raw data*/
    __u8    qadjust;                /* the larger the better quality[2-16](0.25Q, 0.5Q, 0.75Q, Q, 1.25Q, 1.5Q, 1.75Q, 2Q, 2.25Q, 2.5Q, 2.75Q, 3Q, 3.25Q, 3.5Q, 3.75Q)  */
    __u8    qscaling;               /* the smaller the better quality[1-16]  */
    __u32   encode_source_format;
    __u32   encode_image_format;
    __u32   buffersize;             /* each encode buffer size   */
    __u32   buffercount;            /* total encode buffer count */
} jpeg_param_t;


//Define for Encode application
#define DRVJPEG_ENC_THUMBNAIL_QVGA     0
#define DRVJPEG_ENC_THUMBNAIL_QQVGA    1

typedef struct jpeg_info {
    /*decode information*/
    __u32   yuvformat;              /* for decode */
    __u32   width;                  /* for decode */
    __u32   height;                 /* for decode */
    __u32   dec_stride;
    /*encode information*/  
    __u32   bufferend;
    __u32   state;
    __u32   image_size[]; /*image size after encoded*/
} jpeg_info_t;


typedef struct jpeg_priv {
	struct v4l2_device	 v4l2_dev;
    struct video_device  jdev;
    struct mutex         lock;
    struct file *file;
    struct work_struct   tqueue;
    __u32   irq_status;
    jpeg_state_t state;
    __u32   vaddr;                  /* if vaddr_src/vaddr_dst == 0, will using vaddr address */
    __u32   paddr;                  /* if paddr_src/paddr_dst == 0, will using paddr address */
    __u32   vaddr_src;
    __u32   vaddr_dst;
    __u32   paddr_src;
    __u32   paddr_dst;
    __u32   decopw_vaddr;
    __u32   decopw_en;
    __u32   decopw_tnum;
    __u32   decopw_tsize;
    __u32   decopw_tcount;
    __u32   decopw_mcux;
    __u32   decopw_mcuy;
    __u32   decopw_yaddr;
    __u32   decopw_bitaddr;
    __u32   decopw_tmcuynum;
    __u32   decopw_page_index;
    __u32   decopw_page_offset;
    __u32   decopw_TargetBuffersize;
    __u32   windec_mcux_start;
    __u32   windec_mcux_end;
    __u32   windec_mcuy_start;
    __u32   windec_mcuy_end;
    __u32   windec_stride;
    __u32   windec_en;
    struct  page **pages; 
    
    // TESTTEST
    __u32 nr_pages;
    
    __u32   decopw_end;
    __u32   decInWait_buffer_empty;
    __u32   decInWait_buffer_size;
    __u32   decInWait_counter;
    __u32   src_bufsize;            /* source buffer size */
    __u32   dst_bufsize;            /* dst buffer size */
    __u32   mmap_bufsize;           /* mmap buffer size, user can set buffer address or using mmap */
    __u32   src_datasize;           /* source image data size for encode/decode */
    __u32   yuvformat;              /* for decode */
    __u32   width;                  /* for decode */
    __u32   height;                 /* for decode */
  __u32 decode_output_format;
    __u32   scale;                  /* 1 enable, 0 disable */
    __u32   scaled_width;
    __u32   scaled_height;
    __u32   dec_stride;
    __u32   *image_size;            /* image size after encoded */
    __u32   byte2pixel;             /* is 2 times of actual byte */
    __u32   encode_width;           /* the width that will be encoded image raw data */
    __u32   encode_height;          /* the height that will be encoded image raw data */
    __u32   encode_image_format;
    __u32   encode_source_format;
    __u8    qadjust;                /* the larger the better quality[2-16](0.25Q, 0.5Q, 0.75Q, Q, 1.25Q, 1.5Q, 1.75Q, 2Q, 2.25Q, 2.5Q, 2.75Q, 3Q, 3.25Q, 3.5Q, 3.75Q)  */
    __u8    qscaling;               /* the smaller the better quality[1-16] */
    __u32   encode;                 /* 1 decode, 0 encode  */
    __u32   buffersize;             /* each encode buffer size */
    __u32   buffercount;            /* total encode buffer count */
    __u32   bufferend;              /* jpeg encode buffer end */
    __u32   bufferend_bak;          /* jpeg encode buffer end backup */
    __u32   convert;
} jpeg_priv_t;


extern void nuc970_jpeg_init(void);
extern void nuc970_jpeg_trigger(void);
extern int  nuc907_jpeg_set_enc_mode(__u8 u8SourceFormat, __u16 u16JpegFormat);
extern int  nuc970_jpeg_set_dec_mode(__u32 u32OutputFormat);

extern int nuc970_jpeg_CalScalingFactor(
    __u8    u8Mode,                     //Up / Down Scaling
    __u16   u16Height,                  //Original Height
    __u16   u16Width,                   //Original Width
    __u16   u16ScalingHeight,           //Scaled Height
    __u16   u16ScalingWidth,            //Scaled Width
    __u16*  pu16RatioH,                 //Horizontal Ratio
    __u16*  pu16RatioW                  //Vertical Ratio        
);

extern int nuc970_jpeg_SetScalingFactor(
    __u8    u8Mode,                 //Up / Down Scaling
    __u16   u16FactorH,             //Vertical Scaling Factor
    __u16   u16FactorW              //Horizontal Scaling Factor
);

extern void nuc970_jpeg_GetDecodedDimension(
    __u16* pu16Height,          //Decode/Encode Height
    __u16*  pu16Width           //Decode/Encode Width
);

extern void nuc970_jpeg_SetDimension(
    __u16 u16Height,            //Decode/Encode Height
    __u16 u16Width              //Decode/Encode Width
);

extern void nuc970_jpeg_GetDimension(
    __u16* pu16Height,          //Decoded Height from bit stream
    __u16*  pu16Width           //Decoded Width  from bit stream
);

extern void nuc970_jpeg_GetScalingFactor(
    __u8    u8Mode,             //Up / Down Scaling
    __u16* pu16FactorH,     //Vertical Scaling Factor
    __u16*  pu16FactorW         //Horizontal Scaling Factor
);

extern int  nuc970_jpeg_SetWindowDecode(   
    __u16   u16StartMCUX,   //Start X MCU
    __u16   u16StartMCUY,   //Horizontal Scaling Factor
    __u16   u16EndMCUX,     //Vertical Scaling Factor
    __u16   u16EndMCUY,     //Horizontal Scaling Factor 
    __u32   u32Stride       //Decode Output Stride
);

extern int  nuc970_Jpeg_AdjustQTAB(
    __u8 u8Mode,
    __u8 u8Qadjust,
    __u8 u8Qscaling
);

extern int  nuc970_jpeg_SetQTAB(
    __u8* puQTable0,
    __u8* puQTable1,
    __u8* puQTable2,
    __u8 u8num);


#endif  // __NUC970_JPEG_H
