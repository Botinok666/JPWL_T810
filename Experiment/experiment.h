#pragma once
#define ADAPTIVE_ITERATIONS 500
#define TILES_X 10
#define TILES_Y 10
#define BUFFER_SIZE (1ULL << 25)
#define DEFAULT_COMPRESSION 5

typedef enum error_func { SINEWAVE, STEPPER, RISING, FALLING } error_functions;

typedef struct {
	float MSE;		// Mean Square Error
	float PSNR;		// Peak Signal/Noise Ratio
} quality_factors;
