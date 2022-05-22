#pragma once 
/** 
 * \file jpwl_encoder.h 
 * \brief Файл описания программного интерфейса кодера JPWL
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12 
*/

#include "jpwl_params.h"

/**
 * brief  Установка значений параметров кодера jpwl по умолчанию
 * param  params Cсылка на структуру jpwl_enc_params со значениями параметров кодера jpwl
 */
#ifndef __cplusplus
__declspec(dllimport) 
#else
extern "C" __declspec(dllimport) 
#endif
void jpwl_enc_set_default_params(jpwl_enc_params *params);

/**
 * brief  Инициализация значений параметров кодера jpwl, переданных из ПО ПИИ
 * param  params Cсылка на структуру jpwl_enc_params со значениями параметров кодера jpwl
 */
#ifndef __cplusplus
__declspec(dllimport) 
#else
extern "C" __declspec(dllimport) 
#endif
void jpwl_enc_init(jpwl_enc_params *params);

/**
 * brief  Запуск кодера jpwl
 * param  inp_buffer Cсылка на входной буфер
 * param  out_buffer Cсылка на выходной буфер
 * param  bParams Cсылка на структуру jpwl_enc_bParams с дополнительными данными для кодера
 * param  bResult Cсылка на структуру jpwl_enc_bResults с дополнительными результатами кодера
 */
#ifndef __cplusplus
__declspec(dllimport) 
#else
extern "C" __declspec(dllimport)
#endif
errno_t jpwl_enc_run(uint8_t* inp_buffer, uint8_t* out_buffer, 
							   jpwl_enc_bParams *bParams,
							   jpwl_enc_bResults *bResult);

#ifndef _TEST
#define _TEST
#endif

#ifdef _TEST
#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C"__declspec(dllimport)
#endif
void sens_create(unsigned char* input, unsigned short* tile_packets, unsigned char* pack_sens);
#endif
