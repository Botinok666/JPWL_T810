#pragma once 
#include "jpwl_types.h"
#include "jpwl_params.h"
#include <corecrt.h>

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
void jpwl_dec_init();

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
errno_t jpwl_dec_run(jpwl_dec_bParams * bParams, jpwl_dec_bResults * bResult, int* tile_positions);

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
errno_t jpwl_init();

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
void jpwl_destroy();

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
restore_stats* jpwl_dec_stats();
