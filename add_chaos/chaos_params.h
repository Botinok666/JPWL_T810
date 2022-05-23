#pragma once 
#include <stdint.h>

#define PACKET_SIZE 256
#define MAX_BUFFER_SIZE (1ULL << 24) // 16Mb
#define MAX_PACKETS (MAX_BUFFER_SIZE / PACKET_SIZE)

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
