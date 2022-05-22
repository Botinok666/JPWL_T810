// Expiriment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
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

typedef struct {
	float MSE;		// Mean Square Error
	float PSNR;		// Peak Signal/Noise Ratio
} quality_factors;

int const BUFFER_SIZE = 1ULL << 25; // 32Mb
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

	chaos_params_t chaos_params;
	chaos_set_default_params(&chaos_params);
	chaos_params.err_probability = .12f;
	chaos_init(&chaos_params);

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

		if (chaos_params.err_probability > 0) {
			size_t packets = write_packets_with_interleave(jpwl_stream.pData, enc_bResults.wcoder_out_len);
			size_t errors = create_packet_errors(packets, chaos_params.err_probability);
			size_t read_bytes = read_packets_with_deinterleave(jpwl_stream.pData, (uint16_t)packets);
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
			bmp_name, tile_x, tile_y, (uint32_t)(chaos_params.err_probability * 100));
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

int _tmain(int argc, wchar_t* argv[])
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
	test_tile_count(in_files[5], &in_stream, &out_stream);

	free(in_stream.pData);
	free(out_stream.pData);

	return 0;
}

//BYTE *pSrcStream;				// буфер для исходного изображения
//BYTE *pCodeStream;				// буфер для кодового потока jpeg2000 часть1
//unsigned char out_b[MAX_OUTBUUFER_LN]; // буфер для кодового потока jpwl
//unsigned char out_p[MAX_OUTBUUFER_LN*2]; // буфер для rtp пакетов
//unsigned short tiles_b[MAX_TILES];		// буфер для данных о кол-ве пакетов по тайлам
//unsigned short p_lengthes[MAX_ALLPACCKETS];		// массив длин сфомированных пакетов
//unsigned char packsens_b[MAX_ALLPACCKETS]; // буфер для данных о чувствительности пакетов
//jpwl_encoder_BufferParams e_par;	// параметры кодера
//jpwl_encoder_BufferResults e_res;	// результаты кодера
	//int i, j, protact_val, amm_val, tile_ct, percent;
	//unsigned long l;
	//w_encoder_params_value p_val;	// Структура с параметрами кодера jpwl
	//FILE* f, * f_par, * f_kpack, * f_plen, * f_buf;
	//unsigned char* c;
	//jpwl_encoder_params j_e_p;
	//unsigned char* outbuf, * in_b;
	//int nBMPSize, nCodeSize;
	//BOOL bLossy;

//// инициализация кодера j2k
//// 1 - с потерями, 0 - без потерь
//	bLossy=1;
//	nCodeSize = CodeJ2K(pSrcStream, pCodeStream, &j2kresults, bLossy);
//// создание данных о чувствительности
//	in_b=(unsigned char *)pCodeStream;
//	sens_create(in_b,tiles_b,packsens_b);
////	ibuf_write();
////	printf("inbuf writed\n");
////	sens_create();
////	printf("Sens intervals created\n");
//// устанавливаем параметры кодера
////	set_encoder_params();
//	e_par.pack_sens_val=packsens_b;
//	e_par.tile_packets_val=tiles_b;
//	e_par.stream_len=nCodeSize;
///*	p_val.wcoder_mh_param_value=1;	// Стандартная защита основного заголовка
//	p_val.wcoder_th_param_value=1;	// Стандартная защита  заголовка тайла
//	p_val.wcoder_data_param_value=37;
//	p_val.esd_use_value=_false_;
//	p_val.esd_mode_value=ESD_PACKETS;
//	p_val.inbuf_value=in_b;
//	p_val.outbuf_value=out_b;
////	p_val.tile_packets_value=tiles_b;
////	p_val.pack_sens_value=packsens_b;
//	p_val.interleave_use_value=_false_;	// Avvtndment не используется
//	p_val.jpwl_encoder_mode_value=1;
//	p_val.jpwl_encoder_dump_value=0;
//	p_val.dump_dir=NULL; */
//	if((f_par=fopen("Params.txt","rt"))==NULL) {
//		printf("File Params.txt not found");
//		return;
//	};
//	f_kpack=fopen("Kpack.txt","wt");
//	f_plen=fopen("Plen.txt","wt");
//	f_buf=fopen("Buf.dat","wb");
//	fscanf(f_par,"%d\n", &tile_ct);
//	fprintf(f_kpack,"%d\n",tile_ct);
//	do {
//		fscanf(f_par,"%d %d %d\n", &protact_val, &amm_val, &percent); 
//		j_e_p.esd_use_val=0;
//		j_e_p.interleave_use_val=(unsigned char)amm_val;
//		j_e_p.jpwl_encoder_dump_val=0;
//		j_e_p.jpwl_encoder_mode_val=1;
//		j_e_p.wcoder_data_param_val=(unsigned char)protact_val;
//		j_e_p.wcoder_mh_param_val=1;
//		j_e_p.wcoder_th_param_val=1;
//		if (!Init_jpwl_encoder(&j_e_p)) {
//			printf("Init_jpwl_encoder error\n");
//			return 0;
//		};
//		c=ProcessBuffer_jpwl_encoder(in_b,out_b,&e_par,&e_res);
//		printf("Encoder return code=%d\n",c);
//		
///*		outbuf=out_b;
//		f=fopen("OutBuf_encoder.j2k","wb");
//		if(f==NULL) {
//			printf("File not created\n");
//			return;
//		};
//		for(i=0; i<e_res.wcoder_outlen; i++)
//			fputc(*outbuf++,f);
//		fclose(f);	
//		printf("Outfile created\n");
//		getchar();
//*/	
//		r_r_p_t.datagram_size_val=1024;
//		r_r_p_t.Is_rtp_rfc_t=1;
//		if (!rtp_rfc_init_t(&r_r_p_t)) {
//			printf("rtp_rfc_init error\n");
//			return 0;
//		};
//		in_params.instream_len_val=e_res.wcoder_outlen;
//		in_params.instream_mhlen_val=e_res.wcoder_mhlen;
//		in_params.interlace_mode_val=0;
//		in_params.packet_lengthes_val=p_lengthes;
//		in_params.ts_start_val=0;
//		if (!ProcessBuffer_rtp_rfc_t(out_b,out_p,&in_params,&out_params)) {
//			printf("ProcessBuffer_rtp_rfc_t error\n");
//			return 0;
//		};
//		fprintf(f_kpack,"%d %d %d %d\n",protact_val, amm_val,out_params.packet_count_val,percent);
//		for(i=0; i<out_params.packet_count_val; i++)
//			fprintf(f_plen,"%d\n",p_lengthes[i]);
//		
//		printf("ProcessBuffer_rtp_rfc_t done\n");
//