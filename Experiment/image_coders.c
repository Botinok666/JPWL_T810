#include <memory.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <Windows.h>

#include "experiment.h"
#include "convert.h"
#include "memstream.h"
#include "format_defs.h"
#include "image_coders.h"

static void error_callback(const char* msg, void* client_data)
{
	(void)client_data;
	fprintf(stdout, "[ERROR] %s", msg);
}

static void warning_callback(const char* msg, void* client_data)
{
	(void)client_data;
	fprintf(stdout, "[WARNING] %s", msg);
}

errno_t calculate_qf(uint8_t* original_bmp, uint8_t* decoded_bmp, quality_factors* qf_result) {
	opj_bmp_header_t orig_header, dec_header;
	opj_bmp_info_t orig_info, dec_info;
	errno_t err = 0;

	err = bmp_read_file_header(&original_bmp, &orig_header);
	if (err) return -1;
	err = bmp_read_file_header(&decoded_bmp, &dec_header);
	if (err) return -2;
	err = bmp_read_info_header(&original_bmp, &orig_info);
	if (err) return -3;
	err = bmp_read_info_header(&decoded_bmp, &dec_info);
	if (err) return -4;

	if (orig_header.bfSize != dec_header.bfSize
		|| orig_info.biWidth != dec_info.biWidth || orig_info.biHeight != dec_info.biHeight
		|| orig_info.biCompression != dec_info.biCompression)
		return -5;
	if (orig_info.biCompression != 0 || orig_info.biBitCount != 24)
		return -6;

	double mse = 0, rdiv = 1.0 / (3.0 * orig_info.biWidth * orig_info.biHeight);
	int padding = orig_info.biWidth & 3 ? 4 - (orig_info.biWidth & 3) : 0;
	for (uint32_t i = 0; i < orig_info.biHeight; i++) {
		for (uint32_t j = 0; j < orig_info.biWidth * 3; j++) {
			double d = (double)*original_bmp - (double)*decoded_bmp;
			mse += d * d * rdiv;
			original_bmp++;
			decoded_bmp++;
		}
		original_bmp += padding;
		decoded_bmp += padding;
	}
	qf_result->MSE = (float)mse;
	qf_result->PSNR = 10.0f * (float)log10(255.0 * 255.0 / mse);

	return 0;
}

void free_res3(opj_codec_t* codec, opj_image_t* image, void* parameters) {
	opj_destroy_codec(codec);
	opj_image_destroy(image);
	free(parameters);
}

void free_res2(opj_codec_t* codec, opj_image_t* image) {
	opj_destroy_codec(codec);
	opj_image_destroy(image);
}

errno_t encode_BMP_to_J2K(uint8_t* bmp, opj_memory_stream* out_stream, opj_cparameters_t* parameters,
	int tiles_x, int tiles_y) {
	int num_tiles = tiles_x * tiles_y;
	uint8_t* pix_data = NULL;
	opj_image_t* image = bmp_to_image(&bmp, parameters, &pix_data, tiles_x, tiles_y);
	if (!image || !pix_data)
		return -1;

	opj_codec_t* compressor = opj_create_compress(OPJ_CODEC_J2K);
	opj_set_warning_handler(compressor, warning_callback, 0);
	opj_set_error_handler(compressor, error_callback, 0);

	if (!opj_setup_encoder(compressor, parameters, image)) {
		free_res3(compressor, image, pix_data);
		return -2;
	}

	opj_stream_t* output = opj_stream_create_default_memory_stream(out_stream, OPJ_FALSE);
	OPJ_BOOL bSuccess = opj_start_compress(compressor, image, output);
	if (!bSuccess) {
		free_res3(compressor, image, pix_data);
		return -3;
	}

	size_t tile_size = (size_t)image->numcomps * parameters->cp_tdx * parameters->cp_tdy;
	uint8_t* l_data = (uint8_t*)malloc(tile_size);
	if (!l_data) {
		free_res3(compressor, image, pix_data);
		return -1;
	}

	for (int i = 0; i < num_tiles; i++) {
		uint32_t tile_x = i % tiles_x;
		uint32_t tile_y = i / tiles_x;
		uint8_t* data = l_data;
		uint32_t shift = image->x1 - tile_x * parameters->cp_tdx + tile_y * parameters->cp_tdy * image->x1 - 1;

		for (uint32_t c = 1; c <= image->numcomps; c++) {
			uint8_t* p_data = pix_data + num_tiles * tile_size - c - shift * (size_t)image->numcomps;
			for (int j = 0; j < parameters->cp_tdy; j++) {
				for (int k = 0; k < parameters->cp_tdx; k++) {
					*data++ = *p_data;
					p_data += image->numcomps;
				}
				p_data -= ((size_t)image->x1 + parameters->cp_tdx) * image->numcomps;
			}
		}

		if (!opj_write_tile(compressor, i, l_data, (uint32_t)tile_size, output)) {
			free_res3(compressor, image, pix_data);
			free(l_data);
			return -4;
		}
	}

	bSuccess = bSuccess & opj_end_compress(compressor, output);
	if (!bSuccess) {
		free_res2(compressor, image);
		return -5;
	}

	free_res2(compressor, image);
	return 0;
}

errno_t decode_J2K_to_BMP(opj_memory_stream* in_stream, opj_memory_stream* out_stream, int* tile_positions) {
	opj_dparameters_t* parameters = (opj_dparameters_t*)malloc(sizeof(opj_dparameters_t));
	if (!parameters)
		return -1;
	opj_set_default_decoder_parameters(parameters);
	parameters->decod_format = J2K_CFMT;

	opj_stream_t* input = opj_stream_create_default_memory_stream(in_stream, OPJ_TRUE);
	opj_codec_t* decompressor = opj_create_decompress(OPJ_CODEC_J2K);
	opj_image_t* image = NULL;

	opj_set_warning_handler(decompressor, warning_callback, 0);
	opj_set_error_handler(decompressor, error_callback, 0);

	if (!opj_setup_decoder(decompressor, parameters)) {
		opj_destroy_codec(decompressor);
		free(parameters);
		return -2;
	}

	/* Read the main header of the codestream and if necessary the JP2 boxes */
	if (!opj_read_header(input, decompressor, &image)) {
		free_res3(decompressor, image, parameters);
		return -3;
	}
	opj_decoder_set_strict_mode(decompressor, FALSE);
	uint8_t* p_data = NULL, * pix_data = out_stream->pData + 54;
	while (1) {
		uint32_t tile_idx, data_size, numcomps;
		int x0, y0, x1, y1, go_on;

		opj_read_tile_header(decompressor, input, &tile_idx, &data_size,
			&x0, &y0, &x1, &y1, &numcomps, &go_on);
		if (!go_on) break;

		if (!p_data) {
			out_stream->offset = TILES_X * TILES_Y * data_size + 54;
			p_data = malloc(data_size);
			if (!p_data) {
				return -1;
			}
		}
		opj_decode_tile_data(decompressor, tile_idx, p_data, data_size, input);

		uint8_t* data = p_data;
		uint32_t shift = image->x1 - x0 + y0 * image->x1 - 1;

		for (uint32_t c = 1; c <= numcomps; c++) {
			uint8_t* p_data = pix_data + TILES_X * TILES_Y * data_size - c - shift * (size_t)numcomps;
			for (int j = y0; j < y1; j++) {
				for (int k = x0; k < x1; k++) {
					*p_data = *data++;
					p_data += numcomps;
				}
				p_data -= ((size_t)image->x1 + x1 - x0) * numcomps;
			}
		}
	}

	errno_t err = image_to_bmp(image, out_stream->pData, out_stream->dataSize, &out_stream->offset);
	free_res3(decompressor, image, parameters);

	return err;
}
