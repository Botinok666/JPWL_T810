/*
 * The copyright in this software is being made available under the 2-clauses
 * BSD License, included below. This software may be subject to other third
 * party and contributor rights, including patent rights, and no such rights
 * are granted under this license.
 *
 * Copyright (c) 2002-2014, Universite catholique de Louvain (UCL), Belgium
 * Copyright (c) 2002-2014, Professor Benoit Macq
 * Copyright (c) 2001-2003, David Janssens
 * Copyright (c) 2002-2003, Yannick Verschueren
 * Copyright (c) 2003-2007, Francois-Olivier Devaux
 * Copyright (c) 2003-2014, Antonin Descampe
 * Copyright (c) 2005, Herve Drolon, FreeImage Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __J2K_CONVERT_H
#define __J2K_CONVERT_H

#include "openjpeg/openjpeg.h"

/**@name RAW component encoding parameters */
/*@{*/
typedef struct raw_comp_cparameters {
    /** subsampling in X direction */
    int dx;
    /** subsampling in Y direction */
    int dy;
    /*@}*/
} raw_comp_cparameters_t;

/**@name RAW image encoding parameters */
/*@{*/
typedef struct raw_cparameters {
    /** width of the raw image */
    int rawWidth;
    /** height of the raw image */
    int rawHeight;
    /** number of components of the raw image */
    int rawComp;
    /** bit depth of the raw image */
    int rawBitDepth;
    /** signed/unsigned raw image */
    OPJ_BOOL rawSigned;
    /** raw components parameters */
    raw_comp_cparameters_t *rawComps;
    /*@}*/
} raw_cparameters_t;

typedef struct {
	OPJ_UINT16 bfType;      /* 'BM' for Bitmap */
	OPJ_UINT32 bfSize;      /* Size of the file        */
	OPJ_UINT16 bfReserved1; /* Reserved : 0            */
	OPJ_UINT16 bfReserved2; /* Reserved : 0            */
	OPJ_UINT32 bfOffBits;   /* Offset                  */
} opj_bmp_header_t;

typedef struct {
	OPJ_UINT32 biSize;             /* Size of the structure in bytes */
	OPJ_UINT32 biWidth;            /* Width of the image in pixels */
	OPJ_UINT32 biHeight;           /* Height of the image in pixels */
	OPJ_UINT16 biPlanes;           /* 1 */
	OPJ_UINT16 biBitCount;         /* Number of color bits by pixels */
	OPJ_UINT32 biCompression;      /* Type of encoding 0: none 1: RLE8 2: RLE4 */
	OPJ_UINT32 biSizeImage;        /* Size of the image in bytes */
	OPJ_UINT32 biXpelsPerMeter;    /* Horizontal (X) resolution in pixels/meter */
	OPJ_UINT32 biYpelsPerMeter;    /* Vertical (Y) resolution in pixels/meter */
	OPJ_UINT32 biClrUsed;          /* Number of color used in the image (0: ALL) */
	OPJ_UINT32 biClrImportant;     /* Number of important color (0: ALL) */
	OPJ_UINT32 biRedMask;          /* Red channel bit mask */
	OPJ_UINT32 biGreenMask;        /* Green channel bit mask */
	OPJ_UINT32 biBlueMask;         /* Blue channel bit mask */
	OPJ_UINT32 biAlphaMask;        /* Alpha channel bit mask */
	OPJ_UINT32 biColorSpaceType;   /* Color space type */
	OPJ_UINT8  biColorSpaceEP[36]; /* Color space end points */
	OPJ_UINT32 biRedGamma;         /* Red channel gamma */
	OPJ_UINT32 biGreenGamma;       /* Green channel gamma */
	OPJ_UINT32 biBlueGamma;        /* Blue channel gamma */
	OPJ_UINT32 biIntent;           /* Intent */
	OPJ_UINT32 biIccProfileData;   /* ICC profile data */
	OPJ_UINT32 biIccProfileSize;   /* ICC profile size */
	OPJ_UINT32 biReserved;         /* Reserved */
} opj_bmp_info_t;

/* planar / interleaved conversions */
typedef void (* convert_32s_CXPX)(const OPJ_INT32* pSrc, OPJ_INT32* const* pDst,
                                  OPJ_SIZE_T length);
extern const convert_32s_CXPX convert_32s_CXPX_LUT[5];
typedef void (* convert_32s_PXCX)(OPJ_INT32 const* const* pSrc, OPJ_INT32* pDst,
                                  OPJ_SIZE_T length, OPJ_INT32 adjust);
extern const convert_32s_PXCX convert_32s_PXCX_LUT[5];
/* bit depth conversions */
typedef void (* convert_XXx32s_C1R)(const OPJ_BYTE* pSrc, OPJ_INT32* pDst,
                                    OPJ_SIZE_T length);
extern const convert_XXx32s_C1R convert_XXu32s_C1R_LUT[9]; /* up to 8bpp */
typedef void (* convert_32sXXx_C1R)(const OPJ_INT32* pSrc, OPJ_BYTE* pDst,
                                    OPJ_SIZE_T length);
extern const convert_32sXXx_C1R convert_32sXXu_C1R_LUT[9]; /* up to 8bpp */

/* BMP conversion */
opj_image_t* bmp_to_image(uint8_t** bmp, opj_cparameters_t* parameters, uint8_t** pix_data, int tiles_x, int tiles_y);
errno_t image_to_bmp(opj_image_t* image, uint8_t* bmp, size_t bmp_size, size_t* written_bytes);

/* BMP info */
OPJ_BOOL bmp_read_file_header(uint8_t** in, opj_bmp_header_t* header);
OPJ_BOOL bmp_read_info_header(uint8_t** in, opj_bmp_info_t* header);

#endif /* __J2K_CONVERT_H */
