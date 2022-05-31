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
