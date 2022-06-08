#include <memory.h>
#include <stdio.h>
#include <stdint.h>
#include <Windows.h>
#include <WinBase.h>
#include <math.h>

#include "memstream.h"
#include "experiment.h"
#include "format_defs.h"
#include "tests.h"
#include "image_coders.h"

#include "..\jpwl\jpwl_types.h"
#include "..\jpwl\jpwl_params.h"
#include "..\jpwl\jpwl_encoder.h"
#include "..\jpwl\jpwl_decoder.h"
#include "../jpwl/adaptive.h"
#include "..\add_chaos\add_chaos.h"
#include "..\add_chaos\chaos_params.h"
#include "..\add_chaos\mt19937.h"

opj_cparameters_t parameters;
int tile_positions[TILES_X * TILES_Y];

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
	jpwl_enc_bResults* enc_bResults = malloc(sizeof(jpwl_enc_bResults));
	if (!pack_sens || !tile_packets || !in_stream.pData || !out_stream.pData 
		|| !jpwl_stream.pData || !enc_bResults) {
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
	if (jpwl_enc_run(in_stream.pData, jpwl_stream.pData, &enc_bParams, enc_bResults))
		return;
	QueryPerformanceCounter(&EndingTime);

	wprintf(L"J2K to %.2f Mb JPWL: ", enc_bResults->wcoder_out_len / 1048576.0f);
	print_stats(StartingTime, EndingTime, Frequency, in_stream.offset);

	if (err_probability > 0) {
		memcpy(out_stream.pData, jpwl_stream.pData, enc_bResults->wcoder_mh_len);
		size_t packets = write_packets_with_interleave(
			jpwl_stream.pData, enc_bResults->wcoder_out_len, enc_params.wcoder_data);
		int errors = create_packet_errors((int)packets, err_probability * .01f, 1);
		size_t read_bytes = read_packets_with_deinterleave(
			jpwl_stream.pData, packets, enc_params.wcoder_data);
		memcpy(jpwl_stream.pData, out_stream.pData, enc_bResults->wcoder_mh_len);
		enc_bResults->wcoder_out_len = (uint32_t)read_bytes;
		wprintf(L"Tampered buffer with ~%.1f%% packet errors\n",
			errors * 100.0f / enc_bResults->wcoder_out_len);
	}

	jpwl_dec_bParams dec_bParams = {
		.inp_buffer = jpwl_stream.pData,
		.inp_length = enc_bResults->wcoder_out_len,
		.out_buffer = in_stream.pData
	};
	memcpy(tile_positions, enc_bResults->tile_position, sizeof(tile_positions));
	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	if (jpwl_dec_run(&dec_bParams, &dec_bResults, tile_positions))
		return;
	QueryPerformanceCounter(&EndingTime);

	wprintf(L"JPWL to %.2f Mb J2K: ", dec_bResults.out_length / 1048576.0f);
	print_stats(StartingTime, EndingTime, Frequency, enc_bResults->wcoder_out_len);
	wprintf(L"All bad: %d, partially restored: %d, fully restored: %d\n",
		dec_bResults.all_bad_length, dec_bResults.tile_part_rest_cnt, dec_bResults.tile_all_rest_cnt);
	restore_stats* stats = jpwl_dec_stats();
	wprintf(L"Corrected/uncorrected bytes: %.1f%%/%.1f%%\n",
		stats->corrected_rs_bytes * 100.0f / enc_bResults->wcoder_out_len,
		stats->uncorrected_rs_bytes * 100.0f / enc_bResults->wcoder_out_len);

	jpwl_stream.offset = 0;
	out_stream.offset = 0;
	in_stream.offset = 0;
	for (int i = 0; i < TILES_X * TILES_Y; i++) {
		if (!tile_positions[i]) continue;
		memcpy(in_stream.pData + tile_positions[i], enc_bResults->tile_headers[i], 
			sizeof(enc_bResults->tile_headers[0]));
	}

	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	err = decode_J2K_to_BMP(&in_stream, &out_stream, enc_bResults->tile_position);
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
	free(enc_bResults);
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
	jpwl_enc_bResults* enc_bResults = malloc(sizeof(jpwl_enc_bResults));
	if (!pack_sens || !tile_packets || !test_data || !in_stream.pData || !out_stream.pData
		|| !jpwl_stream.pData || !jpwl_copy_stream.pData || !enc_bResults) {
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
		if (jpwl_enc_run(in_stream.pData, jpwl_stream.pData, &enc_bParams, enc_bResults)) {
			wprintf(L"Something went wrong while encoding to jpwl %d\n", enc_params.wcoder_data);
			continue;
		}

		while (err_probability < 50) {
			jpwl_dec_bParams dec_bParams = {
				.inp_buffer = jpwl_copy_stream.pData,
				.inp_length = enc_bResults->wcoder_out_len,
				.out_buffer = out_stream.pData
			};
			int recovered_tiles = 0;
			float errors = 0;
			QueryPerformanceFrequency(&Frequency);
			QueryPerformanceCounter(&StartingTime);
			for (int j = 0; j < iterations; j++) {
				size_t packets = write_packets_with_interleave(
					jpwl_stream.pData, enc_bResults->wcoder_out_len, enc_params.wcoder_data);
				errors += create_packet_errors((int)packets, err_probability * .01f, 1)
					* 100.0f / enc_bResults->wcoder_out_len;
				(void)read_packets_with_deinterleave(
					jpwl_copy_stream.pData, packets, enc_params.wcoder_data);
				// Restore main header - we assume that it will be intact
				memcpy(jpwl_copy_stream.pData, jpwl_stream.pData, enc_bResults->wcoder_mh_len);
				if (jpwl_dec_run(&dec_bParams, &dec_bResults, tile_positions)) {
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
	free(enc_bResults);
	fflush(test_data);
	fclose(test_data);
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
	jpwl_enc_bResults* enc_bResults = malloc(sizeof(jpwl_enc_bResults));
	if (!pack_sens || !tile_packets || !test_data || !jpwl_stream.pData 
		|| !in_stream.pData || !out_stream.pData || !enc_bResults)
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
		if (jpwl_enc_run(in_stream.pData, jpwl_stream.pData, &enc_bParams, enc_bResults)) {
			wprintf(L"Something went wrong while encoding to jpwl %d\n", enc_params.wcoder_data);
			continue;
		}

		size_t packets = write_packets_with_interleave(
			jpwl_stream.pData, enc_bResults->wcoder_out_len, enc_params.wcoder_data);
		memcpy(out_stream.pData, jpwl_stream.pData, enc_bResults->wcoder_mh_len);

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
		memcpy(jpwl_stream.pData, out_stream.pData, enc_bResults->wcoder_mh_len);
		(void)read_packets_with_deinterleave(jpwl_stream.pData, packets, enc_params.wcoder_data);

		jpwl_dec_bParams dec_bParams = {
			.inp_buffer = jpwl_stream.pData,
			.inp_length = enc_bResults->wcoder_out_len,
			.out_buffer = out_stream.pData
		};
		if (jpwl_dec_run(&dec_bParams, &dec_bResults, tile_positions)) {
			wprintf(L"Something went wrong while decoding from jpwl %d\n", enc_params.wcoder_data);
			continue;
		}
		float buffer_errors = (float)errors / enc_bResults->wcoder_out_len;
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
	free(enc_bResults);
	fflush(test_data);
	fclose(test_data);
}
