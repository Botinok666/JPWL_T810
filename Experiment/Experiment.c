#include "windows.h"
#include "winbase.h"
#include "math.h"
#include "memory.h"

#include "..\jpwl\jpwl_types.h"
#include "..\jpwl\jpwl_params.h"
#include "..\jpwl\jpwl_encoder.h"
#include "..\jpwl\jpwl_decoder.h"
#include "..\add_chaos\add_chaos.h"
#include "..\add_chaos\chaos_params.h"
#include "..\add_chaos\mt19937.h"

#include "openjpeg\openjpeg.h"
#include "convert.h"
#include "format_defs.h"
#include "memstream.h"
#include "experiment.h"

opj_cparameters_t parameters;
err_thresholds err_matrix[6] = {
	{.tiles = .98f,
	.thresholds = { .003f, .008f, .015f, .018f, .027f, .054f, .061f, .065f,
		.084f, .118f, .131f, .147f, .153f, .162f, .168f, .168f }
	},
	{.tiles = .9f,
	.thresholds = { .007f, .014f, .022f, .026f, .036f, .065f, .069f, .075f,
		.097f, .131f, .144f, .161f, .165f, .175f, .181f, .184f }
	},
	{.tiles = .8f,
	.thresholds = { .012f, .02f, .032f, .036f, .047f, .078f, .08f, .087f,
		.113f, .147f, .161f, .178f, .18f, .191f, .197f, .203f }
	},
	{.tiles = .7f,
	.thresholds = { .017f, .027f, .042f, .045f, .059f, .092f, .09f, .099f,
		.129f, .163f, .177f, .195f, .195f, .207f, .213f, .222f }
	},
	{.tiles = .6f,
	.thresholds = { .022f, .033f, .051f, .055f, .07f, .106f, .101f, .111f,
		.144f, .18f, .194f, .212f, .21f, .224f, .229f, .241f }
	},
	{.tiles = .5f,
	.thresholds = { .027f, .04f, .061f, .065f, .082f, .119f, .111f, .124f,
		.16f, .196f, .21f, .228f, .225f, .24f, .245f, .26f }
	}
};

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

void init_err_matrix(wchar_t const* filename) {
	FILE* in;
	if (_wfopen_s(&in, filename, L"rt, ccs=UTF-8"))
		return;
	FILE* out;
	if (_wfopen_s(&out, L"..\\Backup\\init_err_matrix.tsv", L"wt, ccs=UTF-8"))
		return;
	if (!in || !out)
		return;
	int rs_prev_idx = 0;
	float err_prev = 0, tiles_prev = 1.0f, min_tiles_prev = 1.0f;
	for (int e = 0; e < ERR_MATRIX_ROWS; e++) {
		while (getwc(in) != L'\n');
		fwprintf(out, L"%.2f\n", err_matrix[e].tiles);
		char idx_rdy = 0;
		do {
			int rs, rs_idx = 0;
			float err, tiles, _;
			fwscanf_s(in, L"%d\t%f\t%f\t%f\n", &rs, &err, &tiles, &_);
			tiles *= .01f;
			for (int i = 0; i < JPWL_CODES; i++) {
				if (jpwl_codes[i] == rs) {
					rs_idx = i;
					break;
				}
			}
			if (rs_prev_idx != rs_idx) {
				rs_prev_idx = rs_idx;
				tiles_prev = 1.0f;
				err_prev = err > 1.0f ? err - 1.0f : 0;
				idx_rdy = 0;
			}
			if (idx_rdy) continue;
			if (tiles == err_matrix[e].tiles) {
				err_matrix[e].thresholds[rs_idx] = err * .01f;
				fwprintf(out, L"%.1f\t", err);
				idx_rdy = 1;
			}
			else if (tiles_prev > err_matrix[e].tiles && err_matrix[e].tiles > tiles) {
				float deltaT = tiles_prev - tiles;
				float deltaE = err - err_prev;
				float k = (tiles_prev - err_matrix[e].tiles) / deltaT;
				err = err_prev + deltaE * k;
				err_matrix[e].thresholds[rs_idx] = err * .01f;
				fwprintf(out, L"%.1f\t", err);
				idx_rdy = 1;
			}

			tiles_prev = tiles;
			err_prev = err;
		} while (!feof(in));
		fwprintf(out, L"\n");
		rewind(in);
	}
	fclose(in);
	fclose(out);
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

errno_t decode_J2K_to_BMP(opj_memory_stream* in_stream, opj_memory_stream* out_stream) {
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

	if (!(opj_decode(decompressor, input, image) &&
		opj_end_decompress(decompressor, input))) {
		free_res3(decompressor, image, parameters);
		return -4;
	}

	errno_t err = image_to_bmp(image, out_stream->pData, out_stream->dataSize, &out_stream->offset);
	free_res3(decompressor, image, parameters);

	return err;
}

errno_t read_BMP_from_file(wchar_t const* filename, uint8_t** bmp, size_t* length) {
	FILE* in;
	if (_wfopen_s(&in, filename, L"rb"))
		return -1;
	if (!in)
		return -1;
	fseek(in, 0, SEEK_END);
	*length = ftell(in);
	*bmp = (uint8_t*)malloc(*length);
	if (!*bmp) {
		fclose(in);
		return -2;
	}
	rewind(in);
	fread_s(*bmp, *length * sizeof(uint8_t), sizeof(uint8_t), *length, in);
	fclose(in);

	return 0;
}

errno_t save_BMP_to_file(wchar_t const* filename, uint8_t* bmp, size_t length) {
	FILE* out;
	if (_wfopen_s(&out, filename, L"wb"))
		return -1;
	if (!out)
		return -1;
	fwrite(bmp, sizeof(uint8_t), length, out);
	fflush(out);
	fclose(out);

	return 0;
}

void print_stats(LARGE_INTEGER start, LARGE_INTEGER end, LARGE_INTEGER freq, size_t size)
{
	LARGE_INTEGER elapsed = {
		.QuadPart = (end.QuadPart - start.QuadPart) * 1000 / freq.QuadPart
	};
	int msecs = elapsed.LowPart;
	if (!msecs)
		msecs = 15;
	wprintf(L"time %.3f s, speed %zd Kb/s\n", msecs / 1000.0f, ((size >> 10) * 1000) / msecs);
}

float get_secs(LARGE_INTEGER start, LARGE_INTEGER end, LARGE_INTEGER freq) {
	LARGE_INTEGER elapsed = {
		.QuadPart = (end.QuadPart - start.QuadPart) * 1000 / freq.QuadPart
	};
	return elapsed.LowPart * .001f;
}

void test_full_cycle(wchar_t const* bmp_name, float compression, int protection, int err_probability) {
	uint8_t* bmp = NULL;
	size_t bmp_size = 0;
	wchar_t out_name[64], in_name[64];
	swprintf(in_name, 64, L"%s.bmp", bmp_name);
	errno_t err = read_BMP_from_file(in_name, &bmp, &bmp_size);
	if (!bmp || err) {
		wprintf(L"Something went wrong while reading bmp: code %d\n", err);
		return;
	}

	uint8_t* pack_sens = (uint8_t*)malloc(MAX_EPBSIZE);
	uint16_t* tile_packets = (uint16_t*)malloc(MAX_TILES);
	LARGE_INTEGER StartingTime, EndingTime, Frequency;
	opj_memory_stream in_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	opj_memory_stream out_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	opj_memory_stream jpwl_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	if (!pack_sens || !tile_packets || !in_stream.pData || !out_stream.pData || !jpwl_stream.pData) {
		wprintf(L"Memory allocation error, aborting\n");
		return;
	}

	opj_set_default_encoder_parameters(&parameters);
	parameters.decod_format = BMP_DFMT;
	parameters.tcp_numlayers = 1;
	parameters.tcp_rates[0] = compression;
	parameters.cp_disto_alloc = 1;
	parameters.irreversible = 1;
	parameters.tile_size_on = 1;

	if (jpwl_init())
	{
		wprintf(L"JPWL init failed");
		return;
	}
	jpwl_enc_bResults enc_bResults;
	jpwl_enc_params enc_params;
	jpwl_enc_set_default_params(&enc_params);
	enc_params.wcoder_data = protection;
	enc_params.wcoder_mh = 1;
	enc_params.wcoder_th = 1;
	jpwl_enc_init(&enc_params);

	jpwl_dec_bResults dec_bResults;
	jpwl_dec_init();

	quality_factors qf;

	chaos_init();
	in_stream.offset = 0;

	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	err = encode_BMP_to_J2K(bmp, &in_stream, &parameters, TILES_X, TILES_Y);
	if (err) {
		wprintf(L"Something went wrong while encoding: code %d\n", err);
		return;
	}
	QueryPerformanceCounter(&EndingTime);

	wprintf(L"BMP to %.2f Mb J2K: ", in_stream.offset / 1048576.0f);
	print_stats(StartingTime, EndingTime, Frequency, bmp_size);

	sens_create(in_stream.pData, tile_packets, pack_sens);

	jpwl_enc_bParams enc_bParams = {
		.stream_len = (uint32_t)in_stream.offset,
		.tile_packets = tile_packets,
		.pack_sens = pack_sens
	};

	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	if (jpwl_enc_run(in_stream.pData, jpwl_stream.pData, &enc_bParams, &enc_bResults))
		return;
	QueryPerformanceCounter(&EndingTime);

	wprintf(L"J2K to %.2f Mb JPWL: ", enc_bResults.wcoder_out_len / 1048576.0f);
	print_stats(StartingTime, EndingTime, Frequency, in_stream.offset);

	if (err_probability > 0) {
		memcpy(out_stream.pData, jpwl_stream.pData, enc_bResults.wcoder_mh_len);
		size_t packets = write_packets_with_interleave(
			jpwl_stream.pData, enc_bResults.wcoder_out_len, enc_params.wcoder_data);
		int errors = create_packet_errors((int)packets, err_probability * .01f, 1);
		size_t read_bytes = read_packets_with_deinterleave(
			jpwl_stream.pData, packets, enc_params.wcoder_data);
		memcpy(jpwl_stream.pData, out_stream.pData, enc_bResults.wcoder_mh_len);
		enc_bResults.wcoder_out_len = (uint32_t)read_bytes;
		wprintf(L"Tampered buffer with ~%.1f%% packet errors\n",
			errors * 100.0f / enc_bResults.wcoder_out_len);
	}

	jpwl_dec_bParams dec_bParams = {
		.inp_buffer = jpwl_stream.pData,
		.inp_length = enc_bResults.wcoder_out_len,
		.out_buffer = in_stream.pData
	};

	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	if (jpwl_dec_run(&dec_bParams, &dec_bResults))
		return;
	QueryPerformanceCounter(&EndingTime);

	wprintf(L"JPWL to %.2f Mb J2K: ", dec_bResults.out_length / 1048576.0f);
	print_stats(StartingTime, EndingTime, Frequency, enc_bResults.wcoder_out_len);
	wprintf(L"All bad: %d, partially restored: %d, fully restored: %d\n",
		dec_bResults.all_bad_length, dec_bResults.tile_part_rest_cnt, dec_bResults.tile_all_rest_cnt);
	restore_stats* stats = jpwl_dec_stats();
	wprintf(L"Corrected/uncorrected bytes: %.1f%%/%.1f%%\n",
		stats->corrected_rs_bytes * 100.0f / enc_bResults.wcoder_out_len,
		stats->uncorrected_rs_bytes * 100.0f / enc_bResults.wcoder_out_len);

	jpwl_stream.offset = 0;
	out_stream.offset = 0;
	in_stream.offset = 0;

	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	err = decode_J2K_to_BMP(&in_stream, &out_stream);
	if (err) {
		wprintf(L"Something went wrong while decoding: code %d\n", err);
		return;
	}
	QueryPerformanceCounter(&EndingTime);

	wprintf(L"J2K to %.2f Mb BMP: ", out_stream.offset / 1048576.0f);
	print_stats(StartingTime, EndingTime, Frequency, dec_bResults.out_length);
	
	err = calculate_qf(bmp, out_stream.pData, &qf);
	if (err) {
		wprintf(L"Something went wrong while comparing images: code %d\n", err);
		return;
	}
	wprintf(L"Comparison result: MSE = %.2f, PSNR = %.2fdB\n", qf.MSE, qf.PSNR);

	swprintf(out_name, 64, L"%s_%dx%d_%dp.bmp",
		bmp_name, TILES_X, TILES_Y, err_probability);
	save_BMP_to_file(out_name, out_stream.pData, out_stream.offset);
	if (err) {
		wprintf(L"Something went wrong while writing bmp: code %d\n", err);
		return;
	}
	wprintf(L"BMP file saved: %lld bytes\n\n", out_stream.offset);

	jpwl_destroy();
	free(pack_sens);
	free(jpwl_stream.pData);
	free(in_stream.pData);
	free(out_stream.pData);
	free(tile_packets);
}

void test_error_recovery(wchar_t const* bmp_name, float compression, int iterations) {
	uint8_t* bmp = NULL;
	size_t bmp_size = 0;
	wchar_t in_name[64];
	swprintf(in_name, 64, L"%s.bmp", bmp_name);
	errno_t err = read_BMP_from_file(in_name, &bmp, &bmp_size);
	if (!bmp || err) {
		wprintf(L"Something went wrong while reading bmp: code %d\n", err);
		return;
	}

	FILE* test_data;
	if (_wfopen_s(&test_data, L"..\\Backup\\test_error_recovery.tsv", L"wt, ccs=UTF-8"))
		return;
	uint8_t* pack_sens = (uint8_t*)malloc(MAX_EPBSIZE);
	uint16_t* tile_packets = (uint16_t*)malloc(MAX_TILES);
	LARGE_INTEGER StartingTime, EndingTime, Frequency;
	opj_memory_stream in_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	opj_memory_stream out_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	opj_memory_stream jpwl_stream = {
		.dataSize = BUFFER_SIZE >> 1,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE >> 1)
	};
	opj_memory_stream jpwl_copy_stream = {
		.dataSize = BUFFER_SIZE >> 1,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE >> 1)
	};
	if (!pack_sens || !tile_packets || !test_data || !in_stream.pData || !out_stream.pData 
		|| !jpwl_stream.pData || !jpwl_copy_stream.pData) {
		wprintf(L"Memory allocation error, aborting\n");
		return;
	}

	opj_set_default_encoder_parameters(&parameters);
	parameters.decod_format = BMP_DFMT;
	parameters.tcp_numlayers = 1;
	parameters.tcp_rates[0] = compression;
	parameters.cp_disto_alloc = 1;
	parameters.irreversible = 1;
	parameters.tile_size_on = 1;

	if (jpwl_init())
	{
		wprintf(L"JPWL init failed");
		return;
	}
	jpwl_enc_bResults enc_bResults;
	jpwl_enc_params enc_params;
	jpwl_enc_set_default_params(&enc_params);
	enc_params.wcoder_mh = 1;
	enc_params.wcoder_th = 1;

	jpwl_dec_bResults dec_bResults;
	jpwl_dec_init();

	err = encode_BMP_to_J2K(bmp, &in_stream, &parameters, TILES_X, TILES_Y);
	if (err) {
		wprintf(L"Something went wrong while encoding to J2K code %d\n", err);
		return;
	}

	sens_create(in_stream.pData, tile_packets, pack_sens);
	jpwl_enc_bParams enc_bParams = {
		.stream_len = (uint32_t)in_stream.offset,
		.tile_packets = tile_packets,
		.pack_sens = pack_sens
	};

	chaos_init();
	int err_probability = 0;
	fwprintf(test_data, L"RS code\tBuffer errors\tRecovered tiles\tTime\n");

	for (int i = 0; i < JPWL_CODES; i++) {
		enc_params.wcoder_data = jpwl_codes[i]; // protection
		jpwl_enc_init(&enc_params);

		in_stream.offset = 0;
		if (jpwl_enc_run(in_stream.pData, jpwl_stream.pData, &enc_bParams, &enc_bResults)) {
			wprintf(L"Something went wrong while encoding to jpwl %d\n", enc_params.wcoder_data);
			continue;
		}

		while (err_probability < 50) {
			jpwl_dec_bParams dec_bParams = {
				.inp_buffer = jpwl_copy_stream.pData,
				.inp_length = enc_bResults.wcoder_out_len,
				.out_buffer = out_stream.pData
			};
			int recovered_tiles = 0;
			float errors = 0;
			QueryPerformanceFrequency(&Frequency);
			QueryPerformanceCounter(&StartingTime);
			for (int j = 0; j < iterations; j++) {
				size_t packets = write_packets_with_interleave(
					jpwl_stream.pData, enc_bResults.wcoder_out_len, enc_params.wcoder_data);
				errors += create_packet_errors((int)packets, err_probability * .01f, 1)
					* 100.0f / enc_bResults.wcoder_out_len;
				(void)read_packets_with_deinterleave(
					jpwl_copy_stream.pData, packets, enc_params.wcoder_data);
				// Restore main header - we assume that it will be intact
				memcpy(jpwl_copy_stream.pData, jpwl_stream.pData, enc_bResults.wcoder_mh_len);
				if (jpwl_dec_run(&dec_bParams, &dec_bResults)) {
					wprintf(L"Something went wrong while decoding from jpwl %d\n", enc_params.wcoder_data);
					continue;
				}
				recovered_tiles += dec_bResults.tile_all_rest_cnt;
				jpwl_stream.offset = 0;
				jpwl_copy_stream.offset = 0;
				out_stream.offset = 0;
			}
			QueryPerformanceCounter(&EndingTime);
			errors /= iterations;
			recovered_tiles /= iterations;
			fwprintf(test_data, L"%d\t%.1f\t%d\t%.3f\n", enc_params.wcoder_data, errors, recovered_tiles,
				get_secs(StartingTime, EndingTime, Frequency) / iterations);

			if (recovered_tiles > ((TILES_X * TILES_Y) >> 1)) {
				err_probability++;
			}
			else {
				int sd = err_probability >> 1;
				sd = sd > 5 ? sd : 5;
				err_probability -= sd > err_probability ? err_probability : sd;
				break;
			}
		}
		wprintf(L"Tested with jpwl %d\n", enc_params.wcoder_data);
	};

	jpwl_destroy();
	free(pack_sens);
	free(jpwl_stream.pData);
	free(in_stream.pData);
	free(out_stream.pData);
	free(jpwl_copy_stream.pData);
	free(tile_packets);
	fflush(test_data);
	fclose(test_data);
}

void select_params_adaptive(float buffer_errors, float recovered_tiles, float min_tiles_percent, jpwl_enc_params* params) {
	static float prev_rec_errors[WINDOW_SIZE] = { 0, 0, 0, 0 };
	static float prev_rec_tiles[WINDOW_SIZE] = { 0, 0, 0, 0 };
	static int prev_idx = 0, window_cnt = 0;

	int jpwl_idx = 0;
	for (int i = 0; i < JPWL_CODES; i++) {
		if (jpwl_codes[i] == params->wcoder_data) {
			jpwl_idx = i;
			break;
		}
	}
	prev_rec_errors[prev_idx] = buffer_errors;
	prev_rec_tiles[prev_idx] = recovered_tiles;
	prev_idx++;
	if (prev_idx >= WINDOW_SIZE) {
		prev_idx = 0;
	}

	if (window_cnt >= WINDOW_SIZE) {
		float avg_errors = 0, avg_tiles = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			avg_errors += prev_rec_errors[i];
			avg_tiles += prev_rec_tiles[i];
		}
		avg_errors /= WINDOW_SIZE;
		avg_tiles /= WINDOW_SIZE;

		int matrix_idx = ERR_MATRIX_ROWS - 1;
		for (int i = 0; i < ERR_MATRIX_ROWS; i++) {
			if (err_matrix[i].tiles <= min_tiles_percent) {
				matrix_idx = i;
				break;
			}
		}
		float err_thresholds[JPWL_CODES];
		memcpy_s(err_thresholds, JPWL_CODES * sizeof(err_thresholds[0]),
			err_matrix[matrix_idx].thresholds, JPWL_CODES * sizeof(err_matrix[matrix_idx].thresholds[0]));
		if (matrix_idx > 0 && min_tiles_percent > err_matrix[ERR_MATRIX_ROWS - 1].tiles) {
			float linear_approx_k = (min_tiles_percent - err_matrix[matrix_idx].tiles) /
				(err_matrix[matrix_idx - 1].tiles - err_matrix[matrix_idx].tiles);
			for (int i = 0; i < JPWL_CODES; i++) {
				err_thresholds[i] += (err_matrix[matrix_idx - 1].thresholds[i] - err_thresholds[i]) * linear_approx_k;
			}
		}
		// This RS code selection based on detected errors in stream
		int guessed_code_idx = JPWL_CODES - 1;
		for (int i = 1; i < JPWL_CODES; i++) {
			if (err_thresholds[i] > avg_errors) {
				guessed_code_idx = i - 1;
				break;
			}
		}

		float max_tiles_percent = min_tiles_percent > .88f ? .99f : (min_tiles_percent + .1f);
		int selected_code_idx = jpwl_idx;
		// Now we need to check more valuable parameter - recovered tiles percentage
		if (avg_tiles < min_tiles_percent) {
			int step = max((int)((min_tiles_percent - avg_tiles) * JPWL_CODES), 1);
			selected_code_idx = min(jpwl_idx + step, JPWL_CODES - 1);
			selected_code_idx = max(guessed_code_idx, selected_code_idx);
			window_cnt = WINDOW_SIZE >> 1;
		}
		else if (avg_tiles > max_tiles_percent) {
			selected_code_idx = (jpwl_idx + guessed_code_idx) >> 1;
			window_cnt = WINDOW_SIZE >> 1;
		}

		params->wcoder_data = jpwl_codes[selected_code_idx];
	}
	else {
		window_cnt++;
	}
}

void test_adaptive_algorithm(wchar_t const* bmp_name, int max_error_percent, int min_tiles_percent, error_functions func) {
	uint8_t* bmp = NULL;
	size_t bmp_size = 0;
	wchar_t name[64];
	swprintf(name, 64, L"%s.bmp", bmp_name);
	errno_t err = read_BMP_from_file(name, &bmp, &bmp_size);
	if (!bmp || err) {
		wprintf(L"Something went wrong while reading bmp: code %d\n", err);
		return;
	}

	FILE* test_data;
	swprintf(name, 64, L"..\\Backup\\adaptive_%dt_%df.tsv", min_tiles_percent, func);
	if (_wfopen_s(&test_data, name, L"wt, ccs=UTF-8"))
		return;
	uint8_t* pack_sens = (uint8_t*)malloc(MAX_EPBSIZE);
	uint16_t* tile_packets = (uint16_t*)malloc(MAX_TILES);
	opj_memory_stream in_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	opj_memory_stream out_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};
	opj_memory_stream jpwl_stream = {
		.dataSize = BUFFER_SIZE >> 1,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE >> 1)
	};
	if (!pack_sens || !tile_packets || !test_data || !jpwl_stream.pData || !in_stream.pData || !out_stream.pData)
	{
		wprintf(L"Memory allocation error, aborting\n");
		return;
	}

	opj_set_default_encoder_parameters(&parameters);
	parameters.decod_format = BMP_DFMT;
	parameters.tcp_numlayers = 1;
	parameters.tcp_rates[0] = DEFAULT_COMPRESSION;
	parameters.cp_disto_alloc = 1;
	parameters.irreversible = 1;
	parameters.tile_size_on = 1;

	if (jpwl_init())
	{
		wprintf(L"JPWL init failed\n");
		return;
	}
	jpwl_enc_bResults enc_bResults;
	jpwl_enc_params enc_params;
	jpwl_enc_set_default_params(&enc_params);
	enc_params.wcoder_data = jpwl_codes[0];
	enc_params.wcoder_mh = 1;
	enc_params.wcoder_th = 1;

	jpwl_dec_bResults dec_bResults;
	jpwl_dec_init();

	err = encode_BMP_to_J2K(bmp, &in_stream, &parameters, TILES_X, TILES_Y);
	if (err) {
		wprintf(L"Something went wrong while encoding to J2K code %d\n", err);
		return;
	}
	sens_create(in_stream.pData, tile_packets, pack_sens);
	jpwl_enc_bParams enc_bParams = {
		.stream_len = (uint32_t)in_stream.offset,
		.tile_packets = tile_packets,
		.pack_sens = pack_sens
	};

	chaos_init();
	fwprintf(test_data, L"Iteration\tRS code\tBuffer errors\tRecovered tiles\n");

	float err_prob = 0;
	for (int i = 0; i < ADAPTIVE_ITERATIONS; i++) {
		jpwl_stream.offset = 0;
		out_stream.offset = 0;
		in_stream.offset = 0;

		jpwl_enc_init(&enc_params);
		if (jpwl_enc_run(in_stream.pData, jpwl_stream.pData, &enc_bParams, &enc_bResults)) {
			wprintf(L"Something went wrong while encoding to jpwl %d\n", enc_params.wcoder_data);
			continue;
		}

		size_t packets = write_packets_with_interleave(
			jpwl_stream.pData, enc_bResults.wcoder_out_len, enc_params.wcoder_data);
		memcpy(out_stream.pData, jpwl_stream.pData, enc_bResults.wcoder_mh_len);

		switch (func)
		{
		case SINEWAVE:
			err_prob = (cosf((float)i / 50 * 3.1415927f) + 1.0f) * max_error_percent * .005f;
			break;

		case STEPPER:
			if (!(i % 25)) {
				err_prob = get_rand_float() * max_error_percent * .01f;
			}
			break;

		case RISING:
			err_prob = (1.0f - cosf((float)i / ADAPTIVE_ITERATIONS * 3.1415927f)) * max_error_percent * .005f;
			break;

		case FALLING:
			err_prob = (cosf((float)i / ADAPTIVE_ITERATIONS * 3.1415927f) + 1.0f) * max_error_percent * .005f;
			break;

		default:
			break;
		}
		int errors = create_packet_errors((int)packets, err_prob, 1);
		memcpy(jpwl_stream.pData, out_stream.pData, enc_bResults.wcoder_mh_len);
		(void)read_packets_with_deinterleave(jpwl_stream.pData, packets, enc_params.wcoder_data);

		jpwl_dec_bParams dec_bParams = {
			.inp_buffer = jpwl_stream.pData,
			.inp_length = enc_bResults.wcoder_out_len,
			.out_buffer = out_stream.pData
		};
		if (jpwl_dec_run(&dec_bParams, &dec_bResults)) {
			wprintf(L"Something went wrong while decoding from jpwl %d\n", enc_params.wcoder_data);
			continue;
		}
		float buffer_errors = (float)errors / enc_bResults.wcoder_out_len;
		float recovered_tiles = (float)dec_bResults.tile_all_rest_cnt / (TILES_X * TILES_Y);

		fwprintf(test_data, L"%d\t%d\t%.1f\t%d\n", i, enc_params.wcoder_data, 
			buffer_errors * 100.0f, (int)(recovered_tiles * 100.0f));
		select_params_adaptive(buffer_errors, recovered_tiles, min_tiles_percent * .01f, &enc_params);

		if (!(i & 15)) {
			wprintf(L"\rTest iteration %d completed", i);
		}
	};

	jpwl_destroy();
	free(pack_sens);
	free(jpwl_stream.pData);
	free(in_stream.pData);
	free(out_stream.pData);
	free(tile_packets);
	fflush(test_data);
	fclose(test_data);
}

int main(int argc, wchar_t* argv[])
{
	wprintf(L"hello\n");
	init_err_matrix(L"test_error_recovery.tsv");

	int opt;
	wprintf(L"Select test\n");
	wprintf(L"0 - single image full cycle\n");
	wprintf(L"1 - error resilience\n");
	wprintf(L"2 - adaptive test\n");
	wprintf(L"3 - adaptive deep test\n");
	wscanf_s(L"%d", &opt);
	switch (opt)
	{
	case 0:
		test_full_cycle(in_files[4], DEFAULT_COMPRESSION, 80, 12);
		break;

	case 1:
		test_error_recovery(in_files[5], DEFAULT_COMPRESSION, 8);
		break;

	case 2:
		wprintf(L"Select error function:\n");
		wprintf(L"0 - sinewave\n");
		wprintf(L"1 - stepper\n");
		wprintf(L"2 - rising\n");
		wprintf(L"3 - falling\n");
		error_functions ef;
		wscanf_s(L"%d", &ef);
		test_adaptive_algorithm(in_files[6], 20, 50, ef);
		break;

	case 3:
		for (int i = 0; i < 4; i++) {
			int error_prob[6] = { 99, 90, 80, 70, 60, 50 };
			for (int j = 0; j < 6; j++) {
				wprintf(L"Testing with %d%% target tiles and with %d mode\n", error_prob[j], i);
				test_adaptive_algorithm(in_files[6], 20, error_prob[j], i);
			}
		}
		break;

	default:
		break;
	}

	return 0;
}
