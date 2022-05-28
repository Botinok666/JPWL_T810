#pragma once 
#include <stdint.h>

#ifndef __cplusplus
__declspec(dllimport)
#else
extern "C" __declspec(dllimport) 
#endif
void chaos_init();

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
