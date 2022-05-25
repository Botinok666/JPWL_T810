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

#include "openjpeg\openjpeg.h"
#include "convert.h"
#include "format_defs.h"
#include "memstream.h"

#define WINDOW_SIZE 4
#define MIN_TILES_PERC .9f
#define ADAPTIVE_ITERATIONS 512
#define ADAPTIVE_SIN_WIDTH 256
#define MAX_ERRORS_PERC .2f

typedef struct {
	float MSE;		// Mean Square Error
	float PSNR;		// Peak Signal/Noise Ratio
} quality_factors;

int const BUFFER_SIZE = 1ULL << 25; // 32Mb
uint8_t const used_jpwl[] = { 38, 43, 48, 56, 64, 75, 85, 96, 112, 128 };
uint8_t const test_jpwl[] = { 38, 43, 48, 56, 64, 75, 85, 96 };
float const high_to_low[] = { .005f, .015f, .035f, .06f, .1f, .12f, .135f, .15f };
wchar_t const* test_tile_xy = L"test_tile_xy.tsv";
wchar_t const* test_err_perc = L"test_error_perc.tsv";
wchar_t const* in_files[] = { 
	L"..\\test_bmp\\Image1", L"..\\test_bmp\\Image2", L"..\\test_bmp\\Image3",
	L"..\\test_bmp\\Image4", L"..\\test_bmp\\Image5", L"..\\test_bmp\\Image6",
	L"..\\test_bmp\\Image7", L"..\\test_bmp\\Image8" };

opj_cparameters_t parameters;

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

void get_bmp_info(uint8_t* bmp, opj_bmp_info_t* info) {
	opj_bmp_header_t orig_header;
	if (bmp_read_file_header(&bmp, &orig_header))
		return;
	bmp_read_info_header(&bmp, info);
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

void test_tile_count(wchar_t const* bmp_name, opj_memory_stream* in_stream, opj_memory_stream* out_stream) {
	uint8_t* bmp = NULL;
	size_t bmp_size = 0;
	wchar_t out_name[64], in_name[64];
	swprintf(in_name, 64, L"%s.bmp", bmp_name);
	errno_t err = read_BMP_from_file(in_name, &bmp, &bmp_size);
	if (!bmp || err) {
		wprintf(L"Something went wrong while reading bmp: code %d\n", err);
		return;
	}

	FILE* test_data;
	if (_wfopen_s(&test_data, test_tile_xy, L"rt, ccs=UTF-8"))
		return;
	uint8_t* pack_sens = (uint8_t*)malloc(MAX_EPBSIZE);
	uint16_t* tile_packets = (uint16_t*)malloc(MAX_TILES);
	LARGE_INTEGER StartingTime, EndingTime, Frequency;
	if (!pack_sens || !tile_packets || !test_data)
		return;

	opj_set_default_encoder_parameters(&parameters);
	parameters.decod_format = BMP_DFMT;
	parameters.tcp_numlayers = 1;
	parameters.tcp_rates[0] = 5;
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
	enc_params.wcoder_data = 64; // protection
	enc_params.wcoder_mh = 1;
	enc_params.wcoder_th = 1;
	jpwl_enc_init(&enc_params);

	jpwl_dec_bResults dec_bResults;
	jpwl_dec_init();

	quality_factors qf;
	opj_memory_stream jpwl_stream = {
		.dataSize = BUFFER_SIZE,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE)
	};

	chaos_init();
	float err_probability = .12f;

	do {
		int tile_x, tile_y;
		in_stream->offset = 0;

		fwscanf_s(test_data, L"%d\t%d\n", &tile_x, &tile_y);
		wprintf(L"Testing with %d tiles\n", tile_x * tile_y);

		QueryPerformanceFrequency(&Frequency);
		QueryPerformanceCounter(&StartingTime);
		errno_t err = encode_BMP_to_J2K(bmp, in_stream, &parameters, tile_x, tile_y);
		if (err) {
			wprintf(L"Something went wrong while encoding: code %d\n", err);
			return;
		}
		QueryPerformanceCounter(&EndingTime);

		wprintf(L"BMP to %.2f Mb J2K: ", in_stream->offset / 1048576.0f);
		print_stats(StartingTime, EndingTime, Frequency, bmp_size);

		sens_create(in_stream->pData, tile_packets, pack_sens);

		jpwl_enc_bParams enc_bParams = {
			.stream_len = (uint32_t)in_stream->offset,
			.tile_packets = tile_packets,
			.pack_sens = pack_sens
		};

		QueryPerformanceFrequency(&Frequency);
		QueryPerformanceCounter(&StartingTime);
		if (jpwl_enc_run(in_stream->pData, jpwl_stream.pData, &enc_bParams, &enc_bResults))
			return;
		QueryPerformanceCounter(&EndingTime);

		wprintf(L"J2K to %.2f Mb JPWL: ", enc_bResults.wcoder_out_len / 1048576.0f);
		print_stats(StartingTime, EndingTime, Frequency, in_stream->offset);

		if (err_probability > 0) {
			memcpy(out_stream->pData, jpwl_stream.pData, enc_bResults.wcoder_mh_len);
			size_t packets = write_packets_with_interleave(
				jpwl_stream.pData, enc_bResults.wcoder_out_len, enc_params.wcoder_data);
			int errors = create_packet_errors((int)packets, err_probability, 1);
			size_t read_bytes = read_packets_with_deinterleave(
				jpwl_stream.pData, packets, enc_params.wcoder_data);
			memcpy(jpwl_stream.pData, out_stream->pData, enc_bResults.wcoder_mh_len);
			enc_bResults.wcoder_out_len = (uint32_t)read_bytes;
			wprintf(L"Tampered buffer with ~%.1f%% packet errors\n",
				errors * 100.0f / enc_bResults.wcoder_out_len);
		}

		jpwl_dec_bParams dec_bParams = {
			.inp_buffer = jpwl_stream.pData,
			.inp_length = enc_bResults.wcoder_out_len,
			.out_buffer = in_stream->pData
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
		out_stream->offset = 0;
		in_stream->offset = 0;

		QueryPerformanceFrequency(&Frequency);
		QueryPerformanceCounter(&StartingTime);
		err = decode_J2K_to_BMP(in_stream, out_stream);
		if (err) {
			wprintf(L"Something went wrong while decoding: code %d\n", err);
			return;
		}
		QueryPerformanceCounter(&EndingTime);

		wprintf(L"J2K to %.2f Mb BMP: ", out_stream->offset / 1048576.0f);
		print_stats(StartingTime, EndingTime, Frequency, dec_bResults.out_length);
	
		err = calculate_qf(bmp, out_stream->pData, &qf);
		if (err) {
			wprintf(L"Something went wrong while comparing images: code %d\n", err);
			return;
		}
		wprintf(L"Comparison result: MSE = %.2f, PSNR = %.2fdB\n", qf.MSE, qf.PSNR);

		swprintf(out_name, 64, L"%s_%dx%d_%dp.bmp",
			bmp_name, tile_x, tile_y, (uint32_t)(err_probability * 100));
		save_BMP_to_file(out_name, out_stream->pData, out_stream->offset);
		if (err) {
			wprintf(L"Something went wrong while writing bmp: code %d\n", err);
			return;
		}
		wprintf(L"BMP file saved: %lld bytes\n\n", out_stream->offset);
	} while (!feof(test_data));

	jpwl_destroy();
	free(pack_sens);
	free(jpwl_stream.pData);
	free(tile_packets);
	fclose(test_data);
}

void test_error_recovery(wchar_t const* bmp_name, opj_memory_stream* in_stream, opj_memory_stream* out_stream) {
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
	if (_wfopen_s(&test_data, L"test_error_recovery.tsv", L"wt, ccs=UTF-8"))
		return;
	uint8_t* pack_sens = (uint8_t*)malloc(MAX_EPBSIZE);
	uint16_t* tile_packets = (uint16_t*)malloc(MAX_TILES);
	LARGE_INTEGER StartingTime, EndingTime, Frequency;
	if (!pack_sens || !tile_packets || !test_data)
		return;

	opj_set_default_encoder_parameters(&parameters);
	parameters.decod_format = BMP_DFMT;
	parameters.tcp_numlayers = 1;
	parameters.tcp_rates[0] = 5;
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

	int tiles_x = 10, tiles_y = 10;
	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	err = encode_BMP_to_J2K(bmp, in_stream, &parameters, tiles_x, tiles_y);
	if (err) {
		wprintf(L"Something went wrong while encoding to J2K code %d\n", err);
		return;
	}
	QueryPerformanceCounter(&EndingTime);
	opj_bmp_info_t bmp_info;
	get_bmp_info(bmp, &bmp_info);
	fwprintf(test_data, L"%s.j2k %dx%d %zd %.2fs\n", bmp_name, bmp_info.biWidth, bmp_info.biHeight,
		in_stream->offset, get_secs(StartingTime, EndingTime, Frequency));

	sens_create(in_stream->pData, tile_packets, pack_sens);
	jpwl_enc_bParams enc_bParams = {
		.stream_len = (uint32_t)in_stream->offset,
		.tile_packets = tile_packets,
		.pack_sens = pack_sens
	};

	chaos_init();
	float err_probability = .0f;
	fwprintf(test_data, L"RS code\tBuffer errors\tRecovered tiles\tTime\n");

	for (int i = 0; i < sizeof(used_jpwl); i++) {
		enc_params.wcoder_data = used_jpwl[i]; // protection
		jpwl_enc_init(&enc_params);

		in_stream->offset = 0;
		if (jpwl_enc_run(in_stream->pData, jpwl_stream.pData, &enc_bParams, &enc_bResults)) {
			wprintf(L"Something went wrong while encoding to jpwl %d\n", enc_params.wcoder_data);
			continue;
		}

		while (err_probability < .5f) {
			jpwl_dec_bParams dec_bParams = {
				.inp_buffer = jpwl_copy_stream.pData,
				.inp_length = enc_bResults.wcoder_out_len,
				.out_buffer = out_stream->pData
			};
			int iterations = 8;
			int recovered_tiles = 0;
			float errors = 0;
			QueryPerformanceFrequency(&Frequency);
			QueryPerformanceCounter(&StartingTime);
			for (int j = 0; j < iterations; j++) {
				size_t packets = write_packets_with_interleave(
					jpwl_stream.pData, enc_bResults.wcoder_out_len, enc_params.wcoder_data);
				errors += create_packet_errors((int)packets, err_probability, 1) * 100.0f / enc_bResults.wcoder_out_len;
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
				out_stream->offset = 0;
			}
			QueryPerformanceCounter(&EndingTime);
			errors /= iterations;
			recovered_tiles /= iterations;
			fwprintf(test_data, L"%d\t%.1f\t%d\t%.3f\n", enc_params.wcoder_data, errors, recovered_tiles,
				get_secs(StartingTime, EndingTime, Frequency) / iterations);

			if (recovered_tiles > (tiles_x * tiles_y) * 9 / 10) {
				err_probability += .005f;
			}
			else {
				err_probability -= .03f;
				if (err_probability < 0)
					err_probability = 0;
				break;
			}
		}
		wprintf(L"Tested with jpwl %d\n", enc_params.wcoder_data);
	};

	jpwl_destroy();
	free(pack_sens);
	free(jpwl_stream.pData);
	free(jpwl_copy_stream.pData);
	free(tile_packets);
	fflush(test_data);
	fclose(test_data);
}

void select_params_adaptive(float buffer_errors, float recovered_tiles, jpwl_enc_params* params) {
	static float prev_rec_errors[WINDOW_SIZE] = { 0, 0, 0, 0 };
	static float prev_rec_tiles[WINDOW_SIZE] = { 0, 0, 0, 0 };
	static int prev_idx = 0;

	int jpwl_idx = 0, jpwl_codes = sizeof(test_jpwl);
	for (int i = 0; i < jpwl_codes; i++) {
		if (test_jpwl[i] == params->wcoder_data) {
			jpwl_idx = i;
			break;
		}
	}
	prev_rec_errors[prev_idx] = buffer_errors;
	prev_rec_tiles[prev_idx] = recovered_tiles;
	prev_idx++;
	if (prev_idx >= WINDOW_SIZE) {
		prev_idx = 0;

		float avg_errors = 0, avg_tiles = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			avg_errors += prev_rec_errors[i];
			avg_tiles += prev_rec_tiles[i];
		}
		avg_errors /= WINDOW_SIZE;
		avg_tiles /= WINDOW_SIZE;

		int step = 0;
		if (avg_tiles < MIN_TILES_PERC) {
			// step up in range 1..3
			avg_tiles = (MIN_TILES_PERC - avg_tiles) * (2.0f / MIN_TILES_PERC);
			step = (int)avg_tiles + 1;
			if (jpwl_idx + step >= jpwl_codes) {
				step = jpwl_codes - 1 - jpwl_idx;
			}
		}
		else if (avg_tiles > .98f) {
			if (jpwl_idx > 0 && high_to_low[jpwl_idx - 1] > avg_errors) {
				step = -1;
			}
		}

		params->wcoder_data = test_jpwl[jpwl_idx + step];
	}
}

void test_adaptive_algorithm(wchar_t const* bmp_name, opj_memory_stream* in_stream, opj_memory_stream* out_stream) {
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
	if (_wfopen_s(&test_data, L"test_adaptive.tsv", L"wt, ccs=UTF-8"))
		return;
	uint8_t* pack_sens = (uint8_t*)malloc(MAX_EPBSIZE);
	uint16_t* tile_packets = (uint16_t*)malloc(MAX_TILES);
	opj_memory_stream jpwl_stream = {
		.dataSize = BUFFER_SIZE >> 1,
		.offset = 0,
		.pData = (uint8_t*)malloc(BUFFER_SIZE >> 1)
	};
	if (!pack_sens || !tile_packets || !test_data || !jpwl_stream.pData)
		return;

	opj_set_default_encoder_parameters(&parameters);
	parameters.decod_format = BMP_DFMT;
	parameters.tcp_numlayers = 1;
	parameters.tcp_rates[0] = 5;
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
	enc_params.wcoder_data = test_jpwl[0];
	enc_params.wcoder_mh = 1;
	enc_params.wcoder_th = 1;

	jpwl_dec_bResults dec_bResults;
	jpwl_dec_init();

	int tiles_x = 10, tiles_y = 10;
	err = encode_BMP_to_J2K(bmp, in_stream, &parameters, tiles_x, tiles_y);
	if (err) {
		wprintf(L"Something went wrong while encoding to J2K code %d\n", err);
		return;
	}
	opj_bmp_info_t bmp_info;
	get_bmp_info(bmp, &bmp_info);
	fwprintf(test_data, L"%s.j2k %dx%d %zd\n", bmp_name, bmp_info.biWidth, bmp_info.biHeight, in_stream->offset);

	sens_create(in_stream->pData, tile_packets, pack_sens);
	jpwl_enc_bParams enc_bParams = {
		.stream_len = (uint32_t)in_stream->offset,
		.tile_packets = tile_packets,
		.pack_sens = pack_sens
	};

	chaos_init();
	fwprintf(test_data, L"Iteration\tRS code\tBuffer errors\tRecovered tiles\n");

	for (int i = 0; i < ADAPTIVE_ITERATIONS; i++) {
		jpwl_stream.offset = 0;
		out_stream->offset = 0;
		in_stream->offset = 0;

		jpwl_enc_init(&enc_params);
		if (jpwl_enc_run(in_stream->pData, jpwl_stream.pData, &enc_bParams, &enc_bResults)) {
			wprintf(L"Something went wrong while encoding to jpwl %d\n", enc_params.wcoder_data);
			continue;
		}

		size_t packets = write_packets_with_interleave(
			jpwl_stream.pData, enc_bResults.wcoder_out_len, enc_params.wcoder_data);
		memcpy(out_stream->pData, jpwl_stream.pData, enc_bResults.wcoder_mh_len);
		// error function
		float err_prob = (cosf((float)i / ADAPTIVE_SIN_WIDTH * 3.1415927f) + 1.0f) * .5f * MAX_ERRORS_PERC;
		int errors = create_packet_errors((int)packets, err_prob, 1);
		memcpy(jpwl_stream.pData, out_stream->pData, enc_bResults.wcoder_mh_len);
		(void)read_packets_with_deinterleave(jpwl_stream.pData, packets, enc_params.wcoder_data);

		jpwl_dec_bParams dec_bParams = {
			.inp_buffer = jpwl_stream.pData,
			.inp_length = enc_bResults.wcoder_out_len,
			.out_buffer = out_stream->pData
		};
		if (jpwl_dec_run(&dec_bParams, &dec_bResults)) {
			wprintf(L"Something went wrong while decoding from jpwl %d\n", enc_params.wcoder_data);
			continue;
		}
		float buffer_errors = (float)errors / enc_bResults.wcoder_out_len;
		float recovered_tiles = (float)dec_bResults.tile_all_rest_cnt / (tiles_x * tiles_y);

		fwprintf(test_data, L"%d\t%d\t%.3f\t%.2f\n", i, enc_params.wcoder_data, buffer_errors, recovered_tiles);
		select_params_adaptive(buffer_errors, recovered_tiles, &enc_params);

		if (!(i & 31)) {
			wprintf(L"\rTest iteration %d completed", i);
		}
	};

	jpwl_destroy();
	free(pack_sens);
	free(jpwl_stream.pData);
	free(tile_packets);
	fflush(test_data);
	fclose(test_data);
}

int main(int argc, wchar_t* argv[])
{
	wprintf(L"hello\n");

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
	int opt;
	wprintf(L"Select test\n");
	wprintf(L"1 - single image full cycle\n");
	wprintf(L"2 - error resilience\n");
	wprintf(L"3 - adaptive test\n");
	wscanf_s(L"%d", &opt);
	switch (opt)
	{
	case 1:
		test_tile_count(in_files[5], &in_stream, &out_stream);
		break;

	case 2:
		test_error_recovery(in_files[6], &in_stream, &out_stream);
		break;

	case 3:
		test_adaptive_algorithm(in_files[6], &in_stream, &out_stream);
		break;

	default:
		break;
	}

	free(in_stream.pData);
	free(out_stream.pData);

	return 0;
}
