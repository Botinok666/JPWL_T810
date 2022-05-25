/**
 * \file add_haos.cpp
 * \brief Подпрограммы библиотеки искусственного зашумления кодового потока.
 * \author Скороход С.В.
 * \date Дата последней модификации - 28.02.12
*/

#include "stdlib.h"
#include "stdio.h"
#include <memory.h>
#include <stdint.h>
#include "math.h"
#include <time.h>
#include "add_chaos.h"
#include "chaos_params.h"
#include "mt19937.h"

rtp_packet_t packets[MAX_PACKETS];

/**
 * \brief Создание битовой маски для зашумления данных в реальном масштабе времени.
 * \details В массив err_packet_mask вставляются единицы для моделирования битовых ошибок с заданной вероятностью и
 * пакетных байтовых ошибок с заданными параметрами нормальных распределений расстояния между ними и ширины пакетной ошибки
 * Используется на стороне передатчика.
 * \param params  ссылка на структуру с параметрами зашумления.
 */
__declspec(dllexport)
void chaos_init()
{
	initialize_mersenne((uint32_t)time(NULL));
}

/**
 * \brief Вызов действий библиотеки по зашумлению
 * \details Using model based on RTP https://datatracker.ietf.org/doc/html/rfc3550
 * \param length - packets count
 * \return errors count
 */
__declspec(dllexport)
int create_packet_errors(int length, float probability, int burst_length)
{
	int i, total_err = 0;

	if (burst_length == 1) {
		for (i = 0; i < length; i++) {
			if (probability > get_rand_float()) {
				memset(&packets[i], 0xff, sizeof(rtp_packet_t));
				total_err += PACKET_SIZE;
			}
		}
	}
	else {
		int error_target = (int)(length * probability);
		int bursts = error_target / burst_length;
		int burst_step = length / bursts;
		for (i = 0; i < bursts; i++) {
			int burst_start = (i + 1) * burst_step - (burst_step >> 1);
			burst_start += (int)(burst_step * (get_rand_float() - .5f));
			int burst_size = (int)(burst_length * (get_rand_float() + .5f));
			if (burst_start + burst_size > length)
				burst_size = length - burst_start;
			memset(&packets[burst_start], 0xff, sizeof(rtp_packet_t) * burst_size);
			total_err += burst_size * PACKET_SIZE;
		}
	}
	return total_err;
}

__declspec(dllexport)
size_t read_packets_with_deinterleave(uint8_t* out_buf, size_t count, size_t stripe) {
	size_t processed = 0;
	while (1)
	{
		for (size_t i = 0; i < PACKET_SIZE; i++) {
			for (size_t j = 0; j < stripe; j++) {
				*out_buf++ = packets[processed + j].payload[i];
			}
		}
		processed += stripe;
		if (processed >= count)
			return processed * PACKET_SIZE;
	}
}

__declspec(dllexport)
size_t write_packets_with_interleave(uint8_t* inp_buf, size_t buf_length, size_t stripe) {
	uint8_t* inp_end = inp_buf + buf_length;
	size_t processed = 0;
	for (size_t i = 0; i < MAX_PACKETS; i++)
		packets[i].header.sequence_number = (uint16_t)i;
	while (1)
	{
		for (size_t i = 0; i < PACKET_SIZE; i++) {
			for (size_t j = 0; j < stripe; j++) {
				packets[processed + j].payload[i] = *inp_buf++;
				if (inp_buf == inp_end)
					return processed + stripe;
			}
		}
		processed += stripe;
	}
}
