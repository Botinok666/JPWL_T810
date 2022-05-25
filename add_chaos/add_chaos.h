#pragma once 
#include <stdint.h>
#include "chaos_params.h"

/* 
 * brief Установка по умолчанию значений параметров библиотеки на стороне передатчика
 * param a_h_p_t Стркутура типа chaos_params_t с параметрами библиотеки на стороне передатчика
 */
#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport) 
#endif
void chaos_init();

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
int create_packet_errors(int length, float probability, int burst_length);

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
size_t read_packets_with_deinterleave(uint8_t* out_buf, size_t count, size_t stripe);

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport)
#endif
size_t write_packets_with_interleave(uint8_t* inp_buf, size_t length, size_t stripe);
