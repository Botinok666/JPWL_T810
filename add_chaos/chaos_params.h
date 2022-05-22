#pragma once 
/** 
 * \file add_haos_params.h
 * \brief Файл описания типов входных параметров процедур интерфейса библиотеки
 * \author Скороход С.В.
 * \date Дата последней модификации - 28.02.13 
*/
#include <stdint.h>

#define PACKET_SIZE 256
#define STRIPE_LENGTH 32
#define MAX_BUFFER_SIZE (1ULL << 23) // 8Mb
#define MAX_PACKETS (MAX_BUFFER_SIZE / PACKET_SIZE)

typedef struct {
	float err_probability;
} chaos_params_t;

typedef struct {
	uint8_t bit_fields1;
	uint8_t bit_fields2;
	uint16_t sequence_number;
	uint32_t timestamp;
	uint32_t ssrc;
} rtp_header_t;

typedef struct {
	rtp_header_t header;
	uint8_t payload[PACKET_SIZE];
} rtp_packet_t;
