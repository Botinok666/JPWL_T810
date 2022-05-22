#pragma once 
/**
 * \file jpwl_decoder.h
 * \brief Файл описания программного интерфейса декодера JPWL
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12
*/

#include "jpwl_types.h"
#include "jpwl_params.h"
#include <corecrt.h>

/**
 * brief Инициализация декодера jpwl
 * param params  Адрес структуры с параметрами инициализации декодера jpwl
 */
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
errno_t jpwl_dec_run(jpwl_dec_bParams * bParams, jpwl_dec_bResults * bResult);

/**
 * brief Инициализация библиотеки jpwl
 */
#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
errno_t jpwl_init();

/**
 * brief Очистка библиотеки jpwl
 */
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

__declspec(dllimport) void markers_write();
