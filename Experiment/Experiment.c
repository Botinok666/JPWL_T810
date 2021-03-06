#include "windows.h"
#include "winbase.h"
#include "math.h"
#include "memory.h"

#include "openjpeg\openjpeg.h"
#include "convert.h"
#include "format_defs.h"
#include "memstream.h"
#include "experiment.h"
#include "tests.h"

wchar_t const* in_files[] = {
	L"..\\test_bmp\\Image1", L"..\\test_bmp\\Image2", L"..\\test_bmp\\Image3",
	L"..\\test_bmp\\Image4", L"..\\test_bmp\\Image5", L"..\\test_bmp\\Image6",
	L"..\\test_bmp\\Image7", L"..\\test_bmp\\Image8" };

int main(int argc, wchar_t* argv[])
{
	wprintf(L"hello\n");

	int opt, img;
	wprintf(L"Select test\n");
	wprintf(L"0 - single image full cycle\n");
	wprintf(L"1 - error resilience\n");
	wprintf(L"2 - adaptive test\n");
	wprintf(L"3 - adaptive deep test\n");
	wscanf_s(L"%d", &opt);
	switch (opt)
	{
	case 0:
		wprintf(L"Image index (1-8): ");
		wscanf_s(L"%d", &img);
		if (1 > img || img > 8) {
			wprintf(L"No image with such index\n");
			break;
		}
		int ep, prot, i;
		wprintf(L"Protection code: ");
		wscanf_s(L"%d", &prot);
		int codes[16] = { 37, 38, 40, 43, 45, 48, 51, 53, 56, 64, 75, 80, 85, 96, 112, 128 };
		for (i = 0; i < 16; i++) {
			if (codes[i] == prot) break;
		}
		if (i == 16) {
			wprintf(L"Code is not supported\n");
			break;
		}
		wprintf(L"Error probability: ");
		wscanf_s(L"%d", &ep);
		if (ep > 25 || ep < 0) {
			wprintf(L"Wrong value\n");
			break;
		}

		test_full_cycle(in_files[img - 1], DEFAULT_COMPRESSION, prot, ep);
		break;

	case 1:
		wprintf(L"Image index (1-8): ");
		wscanf_s(L"%d", &img);
		if (1 > img || img > 8) {
			wprintf(L"No image with such index\n");
			break;
		}
		test_error_recovery(in_files[img - 1], DEFAULT_COMPRESSION, 8);
		break;

	case 2:
		wprintf(L"Image index (1-8): ");
		wscanf_s(L"%d", &img);
		if (1 > img || img > 8) {
			wprintf(L"No image with such index\n");
			break;
		}
		wprintf(L"Select error function:\n");
		wprintf(L"0 - sinewave\n");
		wprintf(L"1 - stepper\n");
		wprintf(L"2 - rising\n");
		wprintf(L"3 - falling\n");
		error_functions ef;
		wscanf_s(L"%d", &ef);
		test_adaptive_algorithm(in_files[img - 1], 20, 50, ef);
		break;

	case 3:
		wprintf(L"Image index (1-8): ");
		wscanf_s(L"%d", &img);
		if (1 > img || img > 8) {
			wprintf(L"No image with such index\n");
			break;
		}
		for (int i = 0; i < 4; i++) {
			int error_prob[6] = { 99, 90, 80, 70, 60, 50 };
			for (int j = 0; j < 6; j++) {
				wprintf(L"Testing with %d%% target tiles and with %d mode\n", error_prob[j], i);
				test_adaptive_algorithm(in_files[img - 1], 20, error_prob[j], i);
			}
		}
		break;

	default:
		break;
	}

	return 0;
}
