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
float byte_error_prob = 0;

#define MASK_LEN (1024*100)		/**< Количествово байт в массиве-маске. */

uint8_t err_byte_mask[MASK_LEN];
int mask_pos = 0;			///< Текущая позиция маски в массиве err_packet_mask

/**
 * \brief Установка по умолчанию значений параметров библиотеки на стороне передатчика
 * \param params Стркутура типа chaos_params_t с параметрами библиотеки на стороне передатчика
 * \return  Код завершения: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void chaos_set_default_params(chaos_params_t* params)
{
	params->err_probability = .1f;		// 10% вероятность
}

/**
 * \brief Вставка инверсий битов в массив-маску
 * \details Вставляет инверсии битов в массив err_packet_mask с вероятностью probability
 * Используется на стороне передатчика
 * \param probability - задает вероятность инверсии бита:
 * 0 <= probability <= 100
 * \return  кол-во инвертированных битов
 */
void create_byte_mask(float probability)
{
	memset(err_byte_mask, 0, MASK_LEN);
	for (int i = 0; i < MASK_LEN; i++) {
		float err = get_rand_float();
		if (probability > err) {
			err_byte_mask[i] = (uint8_t)(err * 254 + 1);
		}
	}
}

/**
 * \brief Создание битовой маски для зашумления данных в реальном масштабе времени.
 * \details В массив err_packet_mask вставляются единицы для моделирования битовых ошибок с заданной вероятностью и
 * пакетных байтовых ошибок с заданными параметрами нормальных распределений расстояния между ними и ширины пакетной ошибки
 * Используется на стороне передатчика.
 * \param params  ссылка на структуру с параметрами зашумления.
 */
__declspec(dllexport)
void chaos_init(chaos_params_t* params)
{
	initialize_mersenne((uint32_t)time(NULL));
	byte_error_prob = params->err_probability;

	create_byte_mask(params->err_probability);
}

size_t create_chaos(size_t bytes)
{
	uint8_t* data = (uint8_t*)packets;
	size_t i, total_err = 0;

	for (i = 0; i < bytes; i++) {
		uint8_t err = err_byte_mask[mask_pos++];
		if (err)
			total_err++;
		data[i] ^= err;
		if (mask_pos == MASK_LEN)
			mask_pos = 0;
	}
	return total_err;
}

/**
 * \brief Вызов действий библиотеки по зашумлению
 * \details Using model based on RTP https://datatracker.ietf.org/doc/html/rfc3550
 * \param length - packets count
 * \return errors count
 */
__declspec(dllexport)
size_t create_packet_errors(size_t length, float probability)
{
	size_t i, total_err = 0;

	for (i = 0; i < length; i++) {
		if (probability > get_rand_float()) {
			memset(&packets[i], 0xff, sizeof(rtp_packet_t));
			total_err += PACKET_SIZE;
		}
	}
	return total_err;
}

__declspec(dllexport)
size_t read_packets_with_deinterleave(uint8_t* out_buf, uint16_t count) {
	size_t processed = 0;
	while (1)
	{
		for (size_t i = 0; i < PACKET_SIZE; i++) {
			for (size_t j = 0; j < STRIPE_LENGTH; j++) {
				*out_buf++ = packets[processed + j].payload[i];
			}
		}
		processed += STRIPE_LENGTH;
		if (processed >= count)
			return processed * PACKET_SIZE;
	}
}

__declspec(dllexport)
size_t write_packets_with_interleave(uint8_t* inp_buf, size_t buf_length) {
	uint8_t* inp_end = inp_buf + buf_length;
	size_t processed = 0;
	for (size_t i = 0; i < MAX_PACKETS; i++)
		packets[i].header.sequence_number = (uint16_t)i;
	while (1)
	{
		for (size_t i = 0; i < PACKET_SIZE; i++) {
			for (size_t j = 0; j < STRIPE_LENGTH; j++) {
				packets[processed + j].payload[i] = *inp_buf++;
				if (inp_buf == inp_end)
					return processed + STRIPE_LENGTH;
			}
		}
		processed += STRIPE_LENGTH;
	}
}
