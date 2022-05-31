#pragma once
#include "jpwl_types.h"

#define WINDOW_SIZE 4
#define ERR_MATRIX_ROWS 6

typedef struct {
	float tiles;
	float thresholds[JPWL_CODES];
} err_thresholds;

uint8_t const jpwl_codes[JPWL_CODES] =
{ 37, 38, 40, 43, 45, 48, 51, 53, 56, 64, 75, 80, 85, 96, 112, 128 };

err_thresholds const err_matrix[6] = {
	{.tiles = .98f,
	.thresholds = { .002f, .01f, .015f, .024f, .031f, .053f, .055f,
		.064f, .081f, .125f, .13f, .161f, .161f, .191f, .191f, .179f }
	},
	{.tiles = .9f,
	.thresholds = { .01f, .018f, .032f, .036f, .045f, .078f, .076f, 
		.08f, .11f, .15f, .162f, .19f, .186f, .203f, .199f, .207f }
	},
	{.tiles = .8f,
	.thresholds = { .014f, .024f, .038f, .045f, .054f, .09f, .086f, 
		.095f, .121f, .168f, .174f, .199f, .195f, .208f, .222f, .23f }
	},
	{.tiles = .7f,
	.thresholds = { .018f, .029f, .045f, .052f, .062f, .099f, .093f, 
		.104f, .135f, .173f, .183f, .214f, .203f, .228f, .232f, .233f }
	},
	{.tiles = .6f,
	.thresholds = { .022f, .035f, .052f, .057f, .07f, .108f, .1f, 
		.112f, .143f, .179f, .193f, .218f, .211f, .232f, .235f, .236f }
	},
	{.tiles = .5f,
	.thresholds = { .027f, .041f, .058f, .063f, .078f, .115f, .108f, 
		.119f, .149f, .194f, .198f, .222f, .217f, .236f, .238f, .239f }
	}
};

__declspec(dllimport)
void select_params_adaptive(float buffer_errors, float recovered_tiles, float min_tiles_percent, jpwl_enc_params* params);
