#pragma once 
/** 
 * \file add_haos_import.h
 * \brief Файл описания программного интерфейса библиотеки искусственного зашумления кодового потока.
 * \author Скороход С.В.
 * \date Дата последней модификации - 28.02.13 
*/

#include <stdint.h>
#include "chaos_params.h"

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
void chaos_set_default_params(chaos_params_t* params);

/* 
 * brief Установка по умолчанию значений параметров библиотеки на стороне передатчика
 * param a_h_p_t Стркутура типа chaos_params_t с параметрами библиотеки на стороне передатчика
 */
#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport) 
#endif
void chaos_init(chaos_params_t* params);

/* 
 * brief Вызов действий библиотеки по зашумлению RTP-пакетов в реальном масштабе времени
 * details Зашумляет входной буфер и копирует во входной. Если адреса входного и выходного буфера совпадают, копирование не производится
 * param in_stream Входной буфер с подготовленными RTP-пакетами
 * param out_stream Выходной буфер с зашумленными RTP-пакетами
 * param buf_par Побочные входные данные - адрес массива, в котором записаны длины RTP пакетов, и количество пакетов
 */
#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
size_t create_packet_errors(size_t length, float probability);

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
size_t read_packets_with_deinterleave(uint8_t* out_buf, uint16_t count);

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
size_t write_packets_with_interleave(uint8_t* inp_buf, size_t length);
