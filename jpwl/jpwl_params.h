﻿#pragma once 
/** 
 * \file  jpwl_enc_params.h
 * \brief Файл описания внешних переменных кодера JPWL, содержащих параметры работы кодера
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12 
*/
#include "jpwl_types.h"
#include <stdint.h>

#define RS_OPTIMIZED

#define ESD_PACKETS _true_		/**< пакетный режим данных о чувствительности */
#define ESD_BYTE_RANGE _false_	/**< режим байтового диапазона данных о чувствительности */

int (*encode_RS)(uint8_t* data, uint8_t* parity, int n, int k);
int (*decode_RS)(uint8_t* data, uint8_t* parity, int n, int k);

/*
* Здесь описаны переменные, значения которых задаются 
* в ПО ПИИ и определяют параметры (режимы) работы кодера jpwl
* предполагается, что ПО ПИИ обеспечивает присваивание только
* одного из описанных значений параметров, т.е. присваивание 
 * других значений блокируется ПО ПИИ
*/

/**
 * \brief Параметр защиты основного заголовка (возможные значения см. ниже)
 * \details Возможные значения:
 * 0 - защита отсутствует
 * 1 - предопределенная защита - RS(160,64)
 * 16 - 16-битная контрольная сумма СКС-16
 * 32 - 32-битная контрольная сумма СКС-32
 * 37 - RS-код RS(37,32)
 * 38 - RS-код RS(38,32)
 * 40 - RS-код RS(40,32)
 * 43 - RS-код RS(43,32)
 * 45 - RS-код RS(45,32)
 * 48 - RS-код RS(48,32)
 * 51 - RS-код RS(51,32)
 * 53 - RS-код RS(53,32)
 * 56 - RS-код RS(56,32)
 * 64 - RS-код RS(64,32)
 * 75 - RS-код RS(75,32)
 * 80 - RS-код RS(80,32)
 * 85 - RS-код RS(85,32)
 * 96 - RS-код RS(96,32)
 * 112 - RS-код RS(112,32)
 * 128 - RS-код RS(128,32)
 */
unsigned char wcoder_mh_param;

/**
 * \brief Параметр защиты  заголовка tile (возможные значения см. ниже)
 * \details Возможные значения:
 * 0 - защита отсутствует
 * 1 - предопределенная защита - RS(80,25)
 * Остальные - см. выше
 */
unsigned char wcoder_th_param;

/**
 * \brief Параметр защиты данных кодового потока jpeg2000 часть 1 (возможные значения см. ниже)
 * \details Возможные значения:
 * 0 - защита отсутствует
 * 1 - неравномерная защита от ошибок UEP - используются RS-коды
 * от 37 до 128 в зависимости от значений чувствительности пакетов,
 * которые передаются кодером jpeg2000 часть 1
 * Остальные - см. выше
 */
unsigned char wcoder_data_param;

/**
 * \brief Режим использования внутрикадрового чередования
 * \details Возможные значения:
 * _true_ - чередование используется
 * _false_ - чередование не используется
 * В данной версии возможность внутрикадрового чередования зарезервирована, но не реализована
 */
_bool_ interleave_use;
