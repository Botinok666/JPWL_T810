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
 * Copyright (c) 2006-2007, Parvatha Elangovan
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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "openjpeg/openjpeg.h"
#include "convert.h"

static void opj_applyLUT8u_8u32s_C1R(
	OPJ_UINT8 const* pSrc, OPJ_INT32 srcStride,
	OPJ_INT32* pDst, OPJ_INT32 dstStride,
	OPJ_UINT8 const* pLUT,
	OPJ_UINT32 width, OPJ_UINT32 height)
{
	OPJ_UINT32 y;

	for (y = height; y != 0U; --y) {
		OPJ_UINT32 x;

		for (x = 0; x < width; x++) {
			pDst[x] = (OPJ_INT32)pLUT[pSrc[x]];
		}
		pSrc += srcStride;
		pDst += dstStride;
	}
}

static void opj_applyLUT8u_8u32s_C1P3R(
	OPJ_UINT8 const* pSrc, OPJ_INT32 srcStride,
	OPJ_INT32* const* pDst, OPJ_INT32 const* pDstStride,
	OPJ_UINT8 const* const* pLUT,
	OPJ_UINT32 width, OPJ_UINT32 height)
{
	OPJ_UINT32 y;
	OPJ_INT32* pR = pDst[0];
	OPJ_INT32* pG = pDst[1];
	OPJ_INT32* pB = pDst[2];
	OPJ_UINT8 const* pLUT_R = pLUT[0];
	OPJ_UINT8 const* pLUT_G = pLUT[1];
	OPJ_UINT8 const* pLUT_B = pLUT[2];

	for (y = height; y != 0U; --y) {
		OPJ_UINT32 x;

		for (x = 0; x < width; x++) {
			OPJ_UINT8 idx = pSrc[x];
			pR[x] = (OPJ_INT32)pLUT_R[idx];
			pG[x] = (OPJ_INT32)pLUT_G[idx];
			pB[x] = (OPJ_INT32)pLUT_B[idx];
		}
		pSrc += srcStride;
		pR += pDstStride[0];
		pG += pDstStride[1];
		pB += pDstStride[2];
	}
}

static void bmp24toimage(const OPJ_UINT8* pData, OPJ_UINT32 stride,
	opj_image_t* image)
{
	int index;
	OPJ_UINT32 width, height;
	OPJ_UINT32 x, y;
	const OPJ_UINT8* pSrc = NULL;

	width = image->comps[0].w;
	height = image->comps[0].h;

	index = 0;
	pSrc = pData + (height - 1ULL) * stride;
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			image->comps[0].data[index] = (OPJ_INT32)pSrc[3 * x + 2]; /* R */
			image->comps[1].data[index] = (OPJ_INT32)pSrc[3 * x + 1]; /* G */
			image->comps[2].data[index] = (OPJ_INT32)pSrc[3 * x + 0]; /* B */
			index++;
		}
		pSrc -= stride;
	}
}

static void bmp_mask_get_shift_and_prec(OPJ_UINT32 mask, OPJ_UINT32* shift,
	OPJ_UINT32* prec)
{
	OPJ_UINT32 l_shift, l_prec;

	l_shift = l_prec = 0U;

	if (mask != 0U) {
		while ((mask & 1U) == 0U) {
			mask >>= 1;
			l_shift++;
		}
		while (mask & 1U) {
			mask >>= 1;
			l_prec++;
		}
	}
	*shift = l_shift;
	*prec = l_prec;
}

static void bmpmask32toimage(const OPJ_UINT8* pData, OPJ_UINT32 stride,
	opj_image_t* image, OPJ_UINT32 redMask, OPJ_UINT32 greenMask,
	OPJ_UINT32 blueMask, OPJ_UINT32 alphaMask)
{
	int index;
	OPJ_UINT32 width, height;
	OPJ_UINT32 x, y;
	const OPJ_UINT8* pSrc = NULL;
	OPJ_BOOL hasAlpha;
	OPJ_UINT32 redShift, redPrec;
	OPJ_UINT32 greenShift, greenPrec;
	OPJ_UINT32 blueShift, bluePrec;
	OPJ_UINT32 alphaShift, alphaPrec;

	width = image->comps[0].w;
	height = image->comps[0].h;

	hasAlpha = image->numcomps > 3U;

	bmp_mask_get_shift_and_prec(redMask, &redShift, &redPrec);
	bmp_mask_get_shift_and_prec(greenMask, &greenShift, &greenPrec);
	bmp_mask_get_shift_and_prec(blueMask, &blueShift, &bluePrec);
	bmp_mask_get_shift_and_prec(alphaMask, &alphaShift, &alphaPrec);

	image->comps[0].prec = redPrec;
	image->comps[1].prec = greenPrec;
	image->comps[2].prec = bluePrec;
	if (hasAlpha) {
		image->comps[3].prec = alphaPrec;
	}

	index = 0;
	pSrc = pData + (height - 1ULL) * stride;
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			OPJ_UINT32 value = 0U;

			value |= ((OPJ_UINT32)pSrc[4 * x + 0]) << 0;
			value |= ((OPJ_UINT32)pSrc[4 * x + 1]) << 8;
			value |= ((OPJ_UINT32)pSrc[4 * x + 2]) << 16;
			value |= ((OPJ_UINT32)pSrc[4 * x + 3]) << 24;

			image->comps[0].data[index] = (OPJ_INT32)((value & redMask) >>
				redShift);   /* R */
			image->comps[1].data[index] = (OPJ_INT32)((value & greenMask) >>
				greenShift); /* G */
			image->comps[2].data[index] = (OPJ_INT32)((value & blueMask) >>
				blueShift);  /* B */
			if (hasAlpha) {
				image->comps[3].data[index] = (OPJ_INT32)((value & alphaMask) >>
					alphaShift);  /* A */
			}
			index++;
		}
		pSrc -= stride;
	}
}

static void bmpmask16toimage(const OPJ_UINT8* pData, OPJ_UINT32 stride,
	opj_image_t* image, OPJ_UINT32 redMask, OPJ_UINT32 greenMask,
	OPJ_UINT32 blueMask, OPJ_UINT32 alphaMask)
{
	int index;
	OPJ_UINT32 width, height;
	OPJ_UINT32 x, y;
	const OPJ_UINT8* pSrc = NULL;
	OPJ_BOOL hasAlpha;
	OPJ_UINT32 redShift, redPrec;
	OPJ_UINT32 greenShift, greenPrec;
	OPJ_UINT32 blueShift, bluePrec;
	OPJ_UINT32 alphaShift, alphaPrec;

	width = image->comps[0].w;
	height = image->comps[0].h;

	hasAlpha = image->numcomps > 3U;

	bmp_mask_get_shift_and_prec(redMask, &redShift, &redPrec);
	bmp_mask_get_shift_and_prec(greenMask, &greenShift, &greenPrec);
	bmp_mask_get_shift_and_prec(blueMask, &blueShift, &bluePrec);
	bmp_mask_get_shift_and_prec(alphaMask, &alphaShift, &alphaPrec);

	image->comps[0].prec = redPrec;
	image->comps[1].prec = greenPrec;
	image->comps[2].prec = bluePrec;
	if (hasAlpha) {
		image->comps[3].prec = alphaPrec;
	}

	index = 0;
	pSrc = pData + (height - 1ULL) * stride;
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			OPJ_UINT32 value = 0U;

			value |= ((OPJ_UINT32)pSrc[2 * x + 0]) << 0;
			value |= ((OPJ_UINT32)pSrc[2 * x + 1]) << 8;

			image->comps[0].data[index] = (OPJ_INT32)((value & redMask) >>
				redShift);   /* R */
			image->comps[1].data[index] = (OPJ_INT32)((value & greenMask) >>
				greenShift); /* G */
			image->comps[2].data[index] = (OPJ_INT32)((value & blueMask) >>
				blueShift);  /* B */
			if (hasAlpha) {
				image->comps[3].data[index] = (OPJ_INT32)((value & alphaMask) >>
					alphaShift);  /* A */
			}
			index++;
		}
		pSrc -= stride;
	}
}

static opj_image_t* bmp8toimage(const OPJ_UINT8* pData, OPJ_UINT32 stride,
	opj_image_t* image, OPJ_UINT8 const* const* pLUT)
{
	OPJ_UINT32 width, height;
	const OPJ_UINT8* pSrc = NULL;

	width = image->comps[0].w;
	height = image->comps[0].h;

	pSrc = pData + (height - 1ULL) * stride;
	if (image->numcomps == 1U) {
		opj_applyLUT8u_8u32s_C1R(pSrc, -(OPJ_INT32)stride, image->comps[0].data,
			(OPJ_INT32)width, pLUT[0], width, height);
	}
	else {
		OPJ_INT32* pDst[3] = { image->comps[0].data, image->comps[1].data, image->comps[2].data };
		OPJ_INT32  pDstStride[3] = { (OPJ_INT32)width,(OPJ_INT32)width,(OPJ_INT32)width };

		opj_applyLUT8u_8u32s_C1P3R(pSrc, -(OPJ_INT32)stride, pDst, pDstStride, pLUT,
			width, height);
	}
	return image;
}

errno_t bmp_read_file_header(uint8_t** in, opj_bmp_header_t* header)
{
	header->bfType = *(uint16_t*)*in;

	if (header->bfType != 0x4d42) {
		return -1;
	}

	/* FILE HEADER */
	/* ------------- */
	header->bfSize = *(uint32_t*)(*in + 2);
	header->bfReserved1 = *(uint16_t*)(*in + 6);
	header->bfReserved2 = *(uint16_t*)(*in + 8);
	header->bfOffBits = *(uint32_t*)(*in + 10);

	*in += 14;
	return 0;
}

errno_t bmp_read_info_header(uint8_t** in, opj_bmp_info_t* header)
{
	memset(header, 0, sizeof(*header));
	/* INFO HEADER */
	/* ------------- */
	header->biSize = *(uint32_t*)*in;

	switch (header->biSize) {
	case 12U:  /* BITMAPCOREHEADER */
	case 40U:  /* BITMAPINFOHEADER */
	case 52U:  /* BITMAPV2INFOHEADER */
	case 56U:  /* BITMAPV3INFOHEADER */
	case 108U: /* BITMAPV4HEADER */
	case 124U: /* BITMAPV5HEADER */
		break;
	default:
		return -1;
	}

	header->biWidth = *(uint32_t*)(*in + 4);
	header->biHeight = *(uint32_t*)(*in + 8);

	header->biPlanes = *(uint16_t*)(*in + 12);
	header->biBitCount = *(uint16_t*)(*in + 14);
	if (header->biBitCount == 0) {
		return -2;
	}

	*in += 16;
	if (header->biSize >= 40U) {
		header->biCompression = *(uint32_t*)*in;
		header->biSizeImage = *(uint32_t*)(*in + 4);
		header->biXpelsPerMeter = *(uint32_t*)(*in + 8);
		header->biYpelsPerMeter = *(uint32_t*)(*in + 12);
		header->biClrUsed = *(uint32_t*)(*in + 16);
		header->biClrImportant = *(uint32_t*)(*in + 20);

		*in += 24;
	}

	if (header->biSize >= 56U) {
		header->biRedMask = *(uint32_t*)*in;
		if (!header->biRedMask) {
			return -3;
		}

		header->biGreenMask = *(uint32_t*)(*in + 4);
		if (!header->biGreenMask) {
			return -3;
		}

		header->biBlueMask = *(uint32_t*)(*in + 8);
		if (!header->biBlueMask) {
			return -3;
		}

		header->biAlphaMask = *(uint32_t*)(*in + 12);

		*in += 16;
	}

	if (header->biSize >= 108U) {
		header->biColorSpaceType = *(uint32_t*)*in;
		if (memcpy_s(&(header->biColorSpaceEP), sizeof(header->biColorSpaceEP),
			*in + 4, 36)) {
			return -4;
		}

		header->biRedGamma = *(uint32_t*)(*in + 40);
		header->biGreenGamma = *(uint32_t*)(*in + 44);
		header->biBlueGamma = *(uint32_t*)(*in + 48);

		*in += 52;
	}

	if (header->biSize >= 124U) {
		header->biIntent = *(uint32_t*)*in;
		header->biIccProfileData = *(uint32_t*)(*in + 4);
		header->biIccProfileSize = *(uint32_t*)(*in + 8);
		header->biReserved = *(uint32_t*)(*in + 12);

		*in += 16;
	}

	return 0;
}

static OPJ_BOOL bmp_read_raw_data(uint8_t** in, OPJ_UINT8* pData, OPJ_UINT32 stride,
	OPJ_UINT32 width, OPJ_UINT32 height)
{
	OPJ_ARG_NOT_USED(width);

	size_t sz = (size_t)stride * height;
	if (memcpy_s(pData, sz, *in, sz)) {
		fprintf(stderr,
			"\nError: memcpy_s failed\n");
		return OPJ_FALSE;
	}

	*in += sz;
	return OPJ_TRUE;
}

static OPJ_BOOL bmp_read_rle8_data(uint8_t** in, OPJ_UINT8* pData,
	size_t stride, OPJ_UINT32 width, OPJ_UINT32 height)
{
	OPJ_UINT32 x, y, written;
	OPJ_UINT8* pix;
	const OPJ_UINT8* beyond;

	beyond = pData + stride * height;
	pix = pData;

	x = y = written = 0U;
	while (y < height) {
		uint8_t c = **in;
		(*in)++;

		if (c) {
			int j;
			uint8_t c1 = **in;
			(*in)++;

			for (j = 0; (j < c) && (x < width) &&
				((OPJ_SIZE_T)pix < (OPJ_SIZE_T)beyond); j++, x++, pix++) {
				*pix = c1;
				written++;
			}
		}
		else {
			c = **in;
			(*in)++;

			if (c == 0x00) { /* EOL */
				x = 0;
				++y;
				pix = pData + y * stride + x;
			}
			else if (c == 0x01) { /* EOP */
				break;
			}
			else if (c == 0x02) { /* MOVE by dxdy */
				c = **in;
				(*in)++;
				x += (OPJ_UINT32)c;
				c = **in;
				(*in)++;
				y += (OPJ_UINT32)c;
				pix = pData + y * stride + x;
			}
			else { /* 03 .. 255 */
				int j;
				for (j = 0; (j < c) && (x < width) &&
					((OPJ_SIZE_T)pix < (OPJ_SIZE_T)beyond); j++, x++, pix++) {
					OPJ_UINT8 c1 = **in;
					(*in)++;
					*pix = c1;
					written++;
				}
				if ((OPJ_UINT32)c & 1U) { /* skip padding byte */
					c = **in;
					(*in)++;
				}
			}
		}
	}/* while() */

	if (written != width * height) {
		fprintf(stderr, "warning, image's actual size does not match advertized one\n");
		return OPJ_FALSE;
	}

	return OPJ_TRUE;
}

static OPJ_BOOL bmp_read_rle4_data(uint8_t** in, OPJ_UINT8* pData,
	size_t stride, OPJ_UINT32 width, OPJ_UINT32 height)
{
	OPJ_UINT32 x, y, written;
	OPJ_UINT8* pix;
	const OPJ_UINT8* beyond;

	beyond = pData + stride * height;
	pix = pData;
	x = y = written = 0U;
	while (y < height) {
		uint8_t c = **in;
		(*in)++;

		if (c) { /* encoded mode */
			int j;
			uint8_t c1 = **in;
			(*in)++;

			for (j = 0; (j < c) && (x < width) &&
				((OPJ_SIZE_T)pix < (OPJ_SIZE_T)beyond); j++, x++, pix++) {
				*pix = (OPJ_UINT8)((j & 1) ? (c1 & 0x0fU) : ((c1 >> 4) & 0x0fU));
				written++;
			}
		}
		else { /* absolute mode */
			c = **in;
			(*in)++;

			if (c == 0x00) { /* EOL */
				x = 0;
				y++;
				pix = pData + y * stride;
			}
			else if (c == 0x01) { /* EOP */
				break;
			}
			else if (c == 0x02) { /* MOVE by dxdy */
				c = **in;
				(*in)++;
				x += (OPJ_UINT32)c;
				c = **in;
				(*in)++;
				y += (OPJ_UINT32)c;
				pix = pData + y * stride + x;
			}
			else { /* 03 .. 255 : absolute mode */
				int j;
				OPJ_UINT8 c1 = 0U;

				for (j = 0; (j < c) && (x < width) &&
					((OPJ_SIZE_T)pix < (OPJ_SIZE_T)beyond); j++, x++, pix++) {
					if ((j & 1) == 0) {
						c1 = **in;
						(*in)++;
					}
					*pix = (OPJ_UINT8)((j & 1) ? (c1 & 0x0fU) : ((c1 >> 4) & 0x0fU));
					written++;
				}
				if (c & 3) { /* skip padding byte */
					c = **in;
					(*in)++;
				}
			}
		}
	}  /* while(y < height) */
	if (written != width * height) {
		fprintf(stderr, "warning, image's actual size does not match advertized one\n");
		return OPJ_FALSE;
	}
	return OPJ_TRUE;
}

void color_esycc_to_rgb(opj_image_t* image)
{
	int y, cb, cr, sign1, sign2, val;
	unsigned int w, h, max, i;
	int flip_value = (1 << (image->comps[0].prec - 1));
	int max_value = (1 << image->comps[0].prec) - 1;

	if ((image->numcomps < 3)
		|| (image->comps[0].dx != image->comps[1].dx) ||
		(image->comps[0].dx != image->comps[2].dx)
		|| (image->comps[0].dy != image->comps[1].dy) ||
		(image->comps[0].dy != image->comps[2].dy)
		) {
		fprintf(stderr, "%s:%d:color_esycc_to_rgb\n\tCAN NOT CONVERT\n", __FILE__,
			__LINE__);
		return;
	}

	w = image->comps[0].w;
	h = image->comps[0].h;

	sign1 = (int)image->comps[1].sgnd;
	sign2 = (int)image->comps[2].sgnd;

	max = w * h;

	for (i = 0; i < max; ++i) {
		y = image->comps[0].data[i];
		cb = image->comps[1].data[i];
		cr = image->comps[2].data[i];

		if (!sign1) {
			cb -= flip_value;
		}
		if (!sign2) {
			cr -= flip_value;
		}

		val = (int)(y - 0.0000368f * cb + 1.40199f * cr + 0.5f);

		if (val > max_value) {
			val = max_value;
		}
		else if (val < 0) {
			val = 0;
		}
		image->comps[0].data[i] = val;

		val = (int)(1.0003f * y - 0.344125f * cb - 0.7141128f * cr + 0.5f);

		if (val > max_value) {
			val = max_value;
		}
		else if (val < 0) {
			val = 0;
		}
		image->comps[1].data[i] = val;

		val = (int)(0.999823f * y + 1.77204f * cb - 0.000008f * cr + 0.5f);

		if (val > max_value) {
			val = max_value;
		}
		else if (val < 0) {
			val = 0;
		}
		image->comps[2].data[i] = val;
	}
	image->color_space = OPJ_CLRSPC_SRGB;
}

static void sycc_to_rgb(int offset, int upb, int y, int cb, int cr,
	int* out_r, int* out_g, int* out_b)
{
	int r, g, b;

	cb -= offset;
	cr -= offset;
	r = y + (int)(1.402f * cr);
	if (r < 0) {
		r = 0;
	}
	else if (r > upb) {
		r = upb;
	}
	*out_r = r;

	g = y - (int)(0.344f * cb + 0.714f * cr);
	if (g < 0) {
		g = 0;
	}
	else if (g > upb) {
		g = upb;
	}
	*out_g = g;

	b = y + (int)(1.772f * cb);
	if (b < 0) {
		b = 0;
	}
	else if (b > upb) {
		b = upb;
	}
	*out_b = b;
}

static void sycc444_to_rgb(opj_image_t* img)
{
	int* d0, * d1, * d2, * r, * g, * b;
	const int* y, * cb, * cr;
	size_t maxw, maxh, max, i;
	int offset, upb;

	upb = (int)img->comps[0].prec;
	offset = 1 << (upb - 1);
	upb = (1 << upb) - 1;

	maxw = (size_t)img->comps[0].w;
	maxh = (size_t)img->comps[0].h;
	max = maxw * maxh;

	y = img->comps[0].data;
	cb = img->comps[1].data;
	cr = img->comps[2].data;

	d0 = r = (int*)opj_image_data_alloc(sizeof(int) * max);
	d1 = g = (int*)opj_image_data_alloc(sizeof(int) * max);
	d2 = b = (int*)opj_image_data_alloc(sizeof(int) * max);

	if (r == NULL || g == NULL || b == NULL) {
		opj_image_data_free(r);
		opj_image_data_free(g);
		opj_image_data_free(b);
		return;
	}

	for (i = 0U; i < max; ++i) {
		sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
		++y;
		++cb;
		++cr;
		++r;
		++g;
		++b;
	}
	opj_image_data_free(img->comps[0].data);
	img->comps[0].data = d0;
	opj_image_data_free(img->comps[1].data);
	img->comps[1].data = d1;
	opj_image_data_free(img->comps[2].data);
	img->comps[2].data = d2;
	img->color_space = OPJ_CLRSPC_SRGB;
}

static void sycc422_to_rgb(opj_image_t* img)
{
	int* d0, * d1, * d2, * r, * g, * b;
	const int* y, * cb, * cr;
	size_t maxw, maxh, max, offx, loopmaxw;
	int offset, upb;
	size_t i;

	upb = (int)img->comps[0].prec;
	offset = 1 << (upb - 1);
	upb = (1 << upb) - 1;

	maxw = (size_t)img->comps[0].w;
	maxh = (size_t)img->comps[0].h;
	max = maxw * maxh;

	y = img->comps[0].data;
	cb = img->comps[1].data;
	cr = img->comps[2].data;

	d0 = r = (int*)opj_image_data_alloc(sizeof(int) * max);
	d1 = g = (int*)opj_image_data_alloc(sizeof(int) * max);
	d2 = b = (int*)opj_image_data_alloc(sizeof(int) * max);

	if (r == NULL || g == NULL || b == NULL) {
		opj_image_data_free(r);
		opj_image_data_free(g);
		opj_image_data_free(b);
		return;
	}

	/* if img->x0 is odd, then first column shall use Cb/Cr = 0 */
	offx = img->x0 & 1U;
	loopmaxw = maxw - offx;

	for (i = 0U; i < maxh; ++i) {
		size_t j;

		if (offx > 0U) {
			sycc_to_rgb(offset, upb, *y, 0, 0, r, g, b);
			++y;
			++r;
			++g;
			++b;
		}

		for (j = 0U; j < (loopmaxw & ~(size_t)1U); j += 2U) {
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
			++y;
			++r;
			++g;
			++b;
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
			++y;
			++r;
			++g;
			++b;
			++cb;
			++cr;
		}
		if (j < loopmaxw) {
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
			++y;
			++r;
			++g;
			++b;
			++cb;
			++cr;
		}
	}

	opj_image_data_free(img->comps[0].data);
	img->comps[0].data = d0;
	opj_image_data_free(img->comps[1].data);
	img->comps[1].data = d1;
	opj_image_data_free(img->comps[2].data);
	img->comps[2].data = d2;

	img->comps[1].w = img->comps[2].w = img->comps[0].w;
	img->comps[1].h = img->comps[2].h = img->comps[0].h;
	img->comps[1].dx = img->comps[2].dx = img->comps[0].dx;
	img->comps[1].dy = img->comps[2].dy = img->comps[0].dy;
	img->color_space = OPJ_CLRSPC_SRGB;
}

static void sycc420_to_rgb(opj_image_t* img)
{
	int* d0, * d1, * d2, * r, * g, * b, * nr, * ng, * nb;
	const int* y, * cb, * cr, * ny;
	size_t maxw, maxh, max, offx, loopmaxw, offy, loopmaxh;
	int offset, upb;
	size_t i;

	upb = (int)img->comps[0].prec;
	offset = 1 << (upb - 1);
	upb = (1 << upb) - 1;

	maxw = (size_t)img->comps[0].w;
	maxh = (size_t)img->comps[0].h;
	max = maxw * maxh;

	y = img->comps[0].data;
	cb = img->comps[1].data;
	cr = img->comps[2].data;

	d0 = r = (int*)opj_image_data_alloc(sizeof(int) * max);
	d1 = g = (int*)opj_image_data_alloc(sizeof(int) * max);
	d2 = b = (int*)opj_image_data_alloc(sizeof(int) * max);

	if (r == NULL || g == NULL || b == NULL) {
		opj_image_data_free(r);
		opj_image_data_free(g);
		opj_image_data_free(b);
	}

	/* if img->x0 is odd, then first column shall use Cb/Cr = 0 */
	offx = img->x0 & 1U;
	loopmaxw = maxw - offx;
	/* if img->y0 is odd, then first line shall use Cb/Cr = 0 */
	offy = img->y0 & 1U;
	loopmaxh = maxh - offy;

	if (offy > 0U) {
		size_t j;

		for (j = 0; j < maxw; ++j) {
			sycc_to_rgb(offset, upb, *y, 0, 0, r, g, b);
			++y;
			++r;
			++g;
			++b;
		}
	}

	for (i = 0U; i < (loopmaxh & ~(size_t)1U); i += 2U) {
		size_t j;

		ny = y + maxw;
		nr = r + maxw;
		ng = g + maxw;
		nb = b + maxw;

		if (offx > 0U) {
			sycc_to_rgb(offset, upb, *y, 0, 0, r, g, b);
			++y;
			++r;
			++g;
			++b;
			sycc_to_rgb(offset, upb, *ny, *cb, *cr, nr, ng, nb);
			++ny;
			++nr;
			++ng;
			++nb;
		}

		for (j = 0; j < (loopmaxw & ~(size_t)1U); j += 2U) {
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
			++y;
			++r;
			++g;
			++b;
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
			++y;
			++r;
			++g;
			++b;

			sycc_to_rgb(offset, upb, *ny, *cb, *cr, nr, ng, nb);
			++ny;
			++nr;
			++ng;
			++nb;
			sycc_to_rgb(offset, upb, *ny, *cb, *cr, nr, ng, nb);
			++ny;
			++nr;
			++ng;
			++nb;
			++cb;
			++cr;
		}
		if (j < loopmaxw) {
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
			++y;
			++r;
			++g;
			++b;

			sycc_to_rgb(offset, upb, *ny, *cb, *cr, nr, ng, nb);
			++ny;
			++nr;
			++ng;
			++nb;
			++cb;
			++cr;
		}
		y += maxw;
		r += maxw;
		g += maxw;
		b += maxw;
	}
	if (i < loopmaxh) {
		size_t j;

		for (j = 0U; j < (maxw & ~(size_t)1U); j += 2U) {
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);

			++y;
			++r;
			++g;
			++b;

			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);

			++y;
			++r;
			++g;
			++b;
			++cb;
			++cr;
		}
		if (j < maxw) {
			sycc_to_rgb(offset, upb, *y, *cb, *cr, r, g, b);
		}
	}

	opj_image_data_free(img->comps[0].data);
	img->comps[0].data = d0;
	opj_image_data_free(img->comps[1].data);
	img->comps[1].data = d1;
	opj_image_data_free(img->comps[2].data);
	img->comps[2].data = d2;

	img->comps[1].w = img->comps[2].w = img->comps[0].w;
	img->comps[1].h = img->comps[2].h = img->comps[0].h;
	img->comps[1].dx = img->comps[2].dx = img->comps[0].dx;
	img->comps[1].dy = img->comps[2].dy = img->comps[0].dy;
	img->color_space = OPJ_CLRSPC_SRGB;
}

void color_sycc_to_rgb(opj_image_t* img)
{
	if (img->numcomps < 3) {
		img->color_space = OPJ_CLRSPC_GRAY;
		return;
	}

	if ((img->comps[0].dx == 1)
		&& (img->comps[1].dx == 2)
		&& (img->comps[2].dx == 2)
		&& (img->comps[0].dy == 1)
		&& (img->comps[1].dy == 2)
		&& (img->comps[2].dy == 2)) { /* horizontal and vertical sub-sample */
		sycc420_to_rgb(img);
	}
	else if ((img->comps[0].dx == 1)
		&& (img->comps[1].dx == 2)
		&& (img->comps[2].dx == 2)
		&& (img->comps[0].dy == 1)
		&& (img->comps[1].dy == 1)
		&& (img->comps[2].dy == 1)) { /* horizontal sub-sample only */
		sycc422_to_rgb(img);
	}
	else if ((img->comps[0].dx == 1)
		&& (img->comps[1].dx == 1)
		&& (img->comps[2].dx == 1)
		&& (img->comps[0].dy == 1)
		&& (img->comps[1].dy == 1)
		&& (img->comps[2].dy == 1)) { /* no sub-sample */
		sycc444_to_rgb(img);
	}
	else {
		fprintf(stderr, "%s:%d:color_sycc_to_rgb\n\tCAN NOT CONVERT\n", __FILE__,
			__LINE__);
		return;
	}
}

opj_image_t* bmp_to_image(uint8_t** bmp, opj_cparameters_t* parameters, uint8_t** pix_data, int tiles_x, int tiles_y)
{
	opj_image_cmptparm_t cmptparm[4] = { 0 };   /* maximum of 4 components */
	OPJ_UINT8 lut_R[256] = { 0 }, lut_G[256] = { 0 }, lut_B[256] = { 0 };
	OPJ_UINT8 const* pLUT[3] = { lut_R, lut_G, lut_B };
	opj_image_t* image = NULL;
	opj_bmp_header_t File_h;
	opj_bmp_info_t Info_h;
	OPJ_UINT32 i, palette_len, numcmpts = 1U;
	OPJ_BOOL l_result = OPJ_FALSE;
	OPJ_UINT8* pData = NULL;
	OPJ_UINT32 stride;
	uint8_t* in = *bmp;

	if (bmp_read_file_header(bmp, &File_h)) {
		return NULL;
	}
	if (bmp_read_info_header(bmp, &Info_h)) {
		return NULL;
	}

	/* Load palette */
	if (Info_h.biBitCount <= 8U) {
		memset(&lut_R[0], 0, sizeof(lut_R));
		memset(&lut_G[0], 0, sizeof(lut_G));
		memset(&lut_B[0], 0, sizeof(lut_B));

		palette_len = Info_h.biClrUsed;
		if ((palette_len == 0U) && (Info_h.biBitCount <= 8U)) {
			palette_len = (1U << Info_h.biBitCount);
		}
		if (palette_len > 256U) {
			palette_len = 256U;
		}
		if (palette_len > 0U) {
			OPJ_UINT8 has_color = 0U;
			for (i = 0U; i < palette_len; i++) {
				lut_B[i] = **bmp;
				(*bmp)++;
				lut_G[i] = **bmp;
				(*bmp)++;
				lut_R[i] = **bmp;
				*bmp += 2; /* 1 byte padding */
				has_color |= (lut_B[i] ^ lut_G[i]) | (lut_G[i] ^ lut_R[i]);
			}
			if (has_color) {
				numcmpts = 3U;
			}
		}
	}
	else {
		numcmpts = 3U;
		if ((Info_h.biCompression == 3) && (Info_h.biAlphaMask != 0U)) {
			numcmpts++;
		}
	}

	if (Info_h.biWidth == 0 || Info_h.biHeight == 0) {
		return NULL;
	}

	if (Info_h.biBitCount > (((OPJ_UINT32)-1) - 31) / Info_h.biWidth) {
		return NULL;
	}
	stride = ((Info_h.biWidth * Info_h.biBitCount + 31U) / 32U) * 4U; /* rows are aligned on 32bits */
	if (Info_h.biBitCount == 4 &&
		Info_h.biCompression == 2) { /* RLE 4 gets decoded as 8 bits data for now... */
		if (8 > (((OPJ_UINT32)-1) - 31) / Info_h.biWidth) {
			return NULL;
		}
		stride = ((Info_h.biWidth * 8U + 31U) / 32U) * 4U;
	}

	if (stride > ((OPJ_UINT32)-1) / sizeof(OPJ_UINT8) / Info_h.biHeight) {
		return NULL;
	}
	pData = (OPJ_UINT8*)calloc(1, sizeof(OPJ_UINT8) * stride * Info_h.biHeight);
	if (pData == NULL) {
		return NULL;
	}
	/* Place the cursor at the beginning of the image information */
	*bmp = in + File_h.bfOffBits;

	switch (Info_h.biCompression) {
	case 0:
	case 3:
		/* read raw data */
		l_result = bmp_read_raw_data(bmp, pData, stride, Info_h.biWidth,
			Info_h.biHeight);
		break;
	case 1:
		/* read rle8 data */
		l_result = bmp_read_rle8_data(bmp, pData, stride, Info_h.biWidth,
			Info_h.biHeight);
		break;
	case 2:
		/* read rle4 data */
		l_result = bmp_read_rle4_data(bmp, pData, stride, Info_h.biWidth,
			Info_h.biHeight);
		break;
	default:
		fprintf(stderr, "Unsupported BMP compression\n");
		l_result = OPJ_FALSE;
		break;
	}
	if (!l_result) {
		free(pData);
		return NULL;
	}

	/* create the image */
	memset(&cmptparm[0], 0, sizeof(cmptparm));
	for (i = 0; i < 4U; i++) {
		cmptparm[i].prec = 8;
		cmptparm[i].sgnd = 0;
		cmptparm[i].dx = (OPJ_UINT32)parameters->subsampling_dx;
		cmptparm[i].dy = (OPJ_UINT32)parameters->subsampling_dy;
		cmptparm[i].w = Info_h.biWidth;
		cmptparm[i].h = Info_h.biHeight;
	}

	if (parameters->tile_size_on) {
		if (Info_h.biWidth % tiles_x || Info_h.biHeight % tiles_y)
			return NULL;
		parameters->cp_tdx = Info_h.biWidth / tiles_x;
		parameters->cp_tdy = Info_h.biHeight / tiles_y;
		image = opj_image_tile_create(numcmpts, cmptparm,
			(numcmpts == 1U) ? OPJ_CLRSPC_GRAY : OPJ_CLRSPC_SRGB);
	}
	else {
		image = opj_image_create(numcmpts, &cmptparm[0],
			(numcmpts == 1U) ? OPJ_CLRSPC_GRAY : OPJ_CLRSPC_SRGB);
	}
	if (!image) {
		free(pData);
		return NULL;
	}
	if (numcmpts == 4U) {
		image->comps[3].alpha = 1;
	}

	/* set image offset and reference grid */
	image->x0 = (OPJ_UINT32)parameters->image_offset_x0;
	image->y0 = (OPJ_UINT32)parameters->image_offset_y0;
	image->x1 = image->x0 + (Info_h.biWidth - 1U) * (OPJ_UINT32)
		parameters->subsampling_dx + 1U;
	image->y1 = image->y0 + (Info_h.biHeight - 1U) * (OPJ_UINT32)
		parameters->subsampling_dy + 1U;

	if (parameters->tile_size_on) {
		if (pix_data) {
			*pix_data = pData;
			return image;
		}
		return NULL;
	}

	/* Read the data */
	if (Info_h.biBitCount == 24 && Info_h.biCompression == 0) { /*RGB */
		bmp24toimage(pData, stride, image);
	}
	else if (Info_h.biBitCount == 8 &&
		Info_h.biCompression == 0) { /* RGB 8bpp Indexed */
		bmp8toimage(pData, stride, image, pLUT);
	}
	else if (Info_h.biBitCount == 8 && Info_h.biCompression == 1) { /*RLE8*/
		bmp8toimage(pData, stride, image, pLUT);
	}
	else if (Info_h.biBitCount == 4 && Info_h.biCompression == 2) { /*RLE4*/
		bmp8toimage(pData, stride, image,
			pLUT); /* RLE 4 gets decoded as 8 bits data for now */
	}
	else if (Info_h.biBitCount == 32 && Info_h.biCompression == 0) { /* RGBX */
		bmpmask32toimage(pData, stride, image, 0x00FF0000U, 0x0000FF00U, 0x000000FFU,
			0x00000000U);
	}
	else if (Info_h.biBitCount == 32 && Info_h.biCompression == 3) { /* bitmask */
		if ((Info_h.biRedMask == 0U) && (Info_h.biGreenMask == 0U) &&
			(Info_h.biBlueMask == 0U)) {
			Info_h.biRedMask = 0x00FF0000U;
			Info_h.biGreenMask = 0x0000FF00U;
			Info_h.biBlueMask = 0x000000FFU;
		}
		bmpmask32toimage(pData, stride, image, Info_h.biRedMask, Info_h.biGreenMask,
			Info_h.biBlueMask, Info_h.biAlphaMask);
	}
	else if (Info_h.biBitCount == 16 && Info_h.biCompression == 0) { /* RGBX */
		bmpmask16toimage(pData, stride, image, 0x7C00U, 0x03E0U, 0x001FU, 0x0000U);
	}
	else if (Info_h.biBitCount == 16 && Info_h.biCompression == 3) { /* bitmask */
		if ((Info_h.biRedMask == 0U) && (Info_h.biGreenMask == 0U) &&
			(Info_h.biBlueMask == 0U)) {
			Info_h.biRedMask = 0xF800U;
			Info_h.biGreenMask = 0x07E0U;
			Info_h.biBlueMask = 0x001FU;
		}
		bmpmask16toimage(pData, stride, image, Info_h.biRedMask, Info_h.biGreenMask,
			Info_h.biBlueMask, Info_h.biAlphaMask);
	}
	else {
		opj_image_destroy(image);
		image = NULL;
		fprintf(stderr,
			"Other system than 24 bits/pixels or 8 bits (no RLE coding) is not yet implemented [%d]\n",
			Info_h.biBitCount);
	}
	free(pData);
	return image;
}

errno_t image_to_bmp(opj_image_t* image, uint8_t* bmp, size_t bmp_size, size_t* written_bytes)
{
	int w, h, sz;
	int i;
	int adjustR, adjustG, adjustB;

	if (image->color_space != OPJ_CLRSPC_SYCC
		&& image->numcomps == 3 && image->comps[0].dx == image->comps[0].dy
		&& image->comps[1].dx != 1) {
		image->color_space = OPJ_CLRSPC_SYCC;
	}
	else if (image->numcomps <= 2) {
		image->color_space = OPJ_CLRSPC_GRAY;
	}

	if (image->color_space == OPJ_CLRSPC_SYCC) {
		color_sycc_to_rgb(image);
	}
	else if (image->color_space == OPJ_CLRSPC_EYCC) {
		color_esycc_to_rgb(image);
	}

	if (image->comps[0].prec < 8) {
		return -1;
	}
	if (image->numcomps >= 3 && image->comps[0].dx == image->comps[1].dx
		&& image->comps[1].dx == image->comps[2].dx
		&& image->comps[0].dy == image->comps[1].dy
		&& image->comps[1].dy == image->comps[2].dy
		&& image->comps[0].prec == image->comps[1].prec
		&& image->comps[1].prec == image->comps[2].prec
		&& image->comps[0].sgnd == image->comps[1].sgnd
		&& image->comps[1].sgnd == image->comps[2].sgnd) {

		/* -->> -->> -->> -->>
		24 bits color
		<<-- <<-- <<-- <<-- */

		w = (int)image->comps[0].w;
		h = (int)image->comps[0].h;
		sz = w * h;

		*(uint16_t*)bmp = 0x4d42;
		uint32_t len = 2;

		/* FILE HEADER */
		/* ------------- */
		*(int32_t*)(bmp + len) = sz * 3 + 3 * h * (w & 1) + 54;
		len += 4;
		*(uint32_t*)(bmp + len) = 0;
		len += 4;
		*(uint32_t*)(bmp + len) = 54;
		len += 4;

		/* INFO HEADER   */
		/* ------------- */
		*(uint32_t*)(bmp + len) = 40;
		len += 4;
		*(int32_t*)(bmp + len) = w;
		len += 4;
		*(int32_t*)(bmp + len) = h;
		len += 4;
		*(uint16_t*)(bmp + len) = 1;
		len += 2;
		*(uint16_t*)(bmp + len) = 24;
		len += 2;
		*(uint32_t*)(bmp + len) = 0;
		len += 4;
		*(uint32_t*)(bmp + len) = sz * 3 + 3 * h * (w & 1);
		len += 4;
		*(uint32_t*)(bmp + len) = 7834;
		len += 4;
		*(uint32_t*)(bmp + len) = 7834;
		len += 4;
		*(uint32_t*)(bmp + len) = 0;
		len += 4;
		*(uint32_t*)(bmp + len) = 0;
		len += 4;

		if (image->comps[0].prec > 8) {
			adjustR = (int)image->comps[0].prec - 8;
		}
		else {
			adjustR = 0;
		}
		if (image->comps[1].prec > 8) {
			adjustG = (int)image->comps[1].prec - 8;
		}
		else {
			adjustG = 0;
		}
		if (image->comps[2].prec > 8) {
			adjustB = (int)image->comps[2].prec - 8;
		}
		else {
			adjustB = 0;
		}

		int ra = image->comps[0].sgnd ? 1 << (image->comps[0].prec - 1) : 0;
		int ga = image->comps[1].sgnd ? 1 << (image->comps[1].prec - 1) : 0;
		int ba = image->comps[2].sgnd ? 1 << (image->comps[2].prec - 1) : 0;

		for (i = 0; i < sz; i++) {
			int r, g, b;
			int dataIdx = sz - (i / w + 1) * w + i % w;

			b = image->comps[2].data[dataIdx];
			b += ba;
			if (adjustB > 0) {
				b = ((b >> adjustB) + ((b >> (adjustB - 1)) & 1));
			}
			if (b > 255) {
				b = 255;
			}
			else if (b < 0) {
				b = 0;
			}
			*(bmp + len) = (OPJ_UINT8)b;
			len++;

			g = image->comps[1].data[dataIdx];
			g += ga;
			if (adjustG > 0) {
				g = ((g >> adjustG) + ((g >> (adjustG - 1)) & 1));
			}
			if (g > 255) {
				g = 255;
			}
			else if (g < 0) {
				g = 0;
			}
			*(bmp + len) = (OPJ_UINT8)g;
			len++;

			r = image->comps[0].data[dataIdx];
			r += ra;
			if (adjustR > 0) {
				r = ((r >> adjustR) + ((r >> (adjustR - 1)) & 1));
			}
			if (r > 255) {
				r = 255;
			}
			else if (r < 0) {
				r = 0;
			}
			*(bmp + len) = (OPJ_UINT8)r;
			len++;

			if ((i + 1) % w == 0) {
				len += 4 - (3 * w) & 3;
			}
		}
		*written_bytes = len;
	}
	else {            /* Gray-scale */

	 /* -->> -->> -->> -->>
	 8 bits non code (Gray scale)
	 <<-- <<-- <<-- <<-- */

		w = (int)image->comps[0].w;
		h = (int)image->comps[0].h;
		sz = w * h;

		*(uint16_t*)bmp = 0x424d;
		uint32_t len = 2;

		/* FILE HEADER */
		/* ------------- */
		*(int32_t*)(bmp + len) = sz + 54 + 1024 + h * (w & 1);
		len += 4;
		*(uint32_t*)(bmp + len) = 0;
		len += 4;
		*(uint32_t*)(bmp + len) = 54 + 1024;
		len += 4;

		/* INFO HEADER   */
		/* ------------- */
		*(uint32_t*)(bmp + len) = 40;
		len += 4;
		*(int32_t*)(bmp + len) = w;
		len += 4;
		*(int32_t*)(bmp + len) = h;
		len += 4;
		*(uint16_t*)(bmp + len) = 1;
		len += 2;
		*(uint16_t*)(bmp + len) = 8;
		len += 2;
		*(uint32_t*)(bmp + len) = 0;
		len += 4;
		*(uint32_t*)(bmp + len) = sz + h * (w & 1);
		len += 4;
		*(uint32_t*)(bmp + len) = 7834;
		len += 4;
		*(uint32_t*)(bmp + len) = 7834;
		len += 4;
		*(uint32_t*)(bmp + len) = 256;
		len += 4;
		*(uint32_t*)(bmp + len) = 256;
		len += 4;

		if (image->comps[0].prec > 8) {
			adjustR = (int)image->comps[0].prec - 8;
		}
		else {
			adjustR = 0;
		}

		int lut = 0;
		for (i = 0; i < 256; i++) {
			*(uint32_t*)(bmp + len) = lut;
			lut += 0x00010101;
			len += 4;
		}

		int ra = image->comps[0].sgnd ? 1 << (image->comps[0].prec - 1) : 0;
		for (i = 0; i < w * h; i++) {
			int r;

			r = image->comps[0].data[w * h - (i / w + 1) * w + i % w];
			r += ra;
			if (adjustR > 0) {
				r = ((r >> adjustR) + ((r >> (adjustR - 1)) & 1));
			}
			if (r > 255) {
				r = 255;
			}
			else if (r < 0) {
				r = 0;
			}

			*(bmp + len) = (uint8_t)r;
			len++;

			if ((i + 1) % w == 0) {
				len += 4 - w & 3;
			}
		}
		*written_bytes = len;
	}

	return 0;
}
