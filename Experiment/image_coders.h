#pragma once
#include <stdint.h>
#include "memstream.h"

errno_t encode_BMP_to_J2K(uint8_t* bmp, opj_memory_stream* out_stream, opj_cparameters_t* parameters,
	int tiles_x, int tiles_y);

errno_t decode_J2K_to_BMP(opj_memory_stream* in_stream, opj_memory_stream* out_stream);

errno_t calculate_qf(uint8_t* original_bmp, uint8_t* decoded_bmp, quality_factors* qf_result);
