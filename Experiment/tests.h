#pragma once
#include <stdint.h>
#include "experiment.h"

void test_full_cycle(wchar_t const* bmp_name, float compression, int protection, int err_probability);

void test_error_recovery(wchar_t const* bmp_name, float compression, int iterations);

void test_adaptive_algorithm(wchar_t const* bmp_name, int max_error_percent, int min_tiles_percent, error_functions func);
