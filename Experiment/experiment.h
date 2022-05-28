#pragma once
#define JPWL_CODES 16
#define WINDOW_SIZE 4
#define ADAPTIVE_ITERATIONS 500
#define TILES_X 10
#define TILES_Y 10
#define BUFFER_SIZE (1ULL << 25)
#define DEFAULT_COMPRESSION 5
#define ERR_MATRIX_ROWS 6

typedef enum error_func { SINEWAVE, STEPPER, RISING, FALLING } error_functions;

typedef struct {
	float MSE;		// Mean Square Error
	float PSNR;		// Peak Signal/Noise Ratio
} quality_factors;

typedef struct {
	float tiles;
	float thresholds[JPWL_CODES];
} err_thresholds;

uint8_t const jpwl_codes[JPWL_CODES] = 
	{ 37, 38, 40, 43, 45, 48, 51, 53, 56, 64, 75, 80, 85, 96, 112, 128 };

wchar_t const* in_files[] = {
	L"..\\test_bmp\\Image1", L"..\\test_bmp\\Image2", L"..\\test_bmp\\Image3",
	L"..\\test_bmp\\Image4", L"..\\test_bmp\\Image5", L"..\\test_bmp\\Image6",
	L"..\\test_bmp\\Image7", L"..\\test_bmp\\Image8" };
