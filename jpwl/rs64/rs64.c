#include "string.h"
#include "windows.h"
#include "rs64.h"

#define CODES_COUNT 19

union u8u16
{
	struct {
		uint8_t k, n;
	} code;
	uint16_t nk;
};

struct rs_code {
	union u8u16 id;
	uint8_t* lut;
	uint8_t* coefs;
};

static char is_initialized = FALSE;
static struct rs_code codes[CODES_COUNT] =
{
	{.id = {.code.n = 37, .code.k = 32 }},
	{.id = {.code.n = 38, .code.k = 32 }},
	{.id = {.code.n = 40, .code.k = 13 }},
	{.id = {.code.n = 40, .code.k = 32 }},
	{.id = {.code.n = 43, .code.k = 32 }},
	{.id = {.code.n = 45, .code.k = 32 }},
	{.id = {.code.n = 48, .code.k = 32 }},
	{.id = {.code.n = 51, .code.k = 32 }},
	{.id = {.code.n = 53, .code.k = 32 }},
	{.id = {.code.n = 56, .code.k = 32 }},
	{.id = {.code.n = 64, .code.k = 32 }},
	{.id = {.code.n = 75, .code.k = 32 }},
	{.id = {.code.n = 80, .code.k = 25 }},
	{.id = {.code.n = 80, .code.k = 32 }},
	{.id = {.code.n = 85, .code.k = 32 }},
	{.id = {.code.n = 96, .code.k = 32 }},
	{.id = {.code.n = 112, .code.k = 32 }},
	{.id = {.code.n = 128, .code.k = 32 }},
	{.id = {.code.n = 160, .code.k = 64 }},
};

static int (*EncodeData)(uint8_t, uint8_t, uint8_t*, uint8_t*, uint8_t*);
static int (*DecodeData)(uint8_t, uint8_t, uint8_t*, uint8_t*);

errno_t rs_init_all()
{
	if (is_initialized)
		return 0;
	int ext = GetSupportedExtensions();
	for (int i = 0; i < CODES_COUNT; i++)
	{
		if (ext)
		{
			codes[i].coefs = (uint8_t*)malloc(SSE_COEFS_SIZE);
			codes[i].lut = (uint8_t*)malloc(SSE_LUT_SIZE);
			if (!codes[i].coefs || !codes[i].lut)
				return -1;
			InitSSSE3(codes[i].coefs, codes[i].id.code.n - codes[i].id.code.k, codes[i].lut);
		}
		else
		{
			codes[i].coefs = (uint8_t*)malloc(ALU_COEFS_SIZE);
			codes[i].lut = (uint8_t*)malloc(ALU_LUT_SIZE);
			if (!codes[i].coefs || !codes[i].lut)
				return -1;
			InitALU(codes[i].coefs, codes[i].id.code.n - codes[i].id.code.k, codes[i].lut);
		}
	}
	if (ext & SSSE3_SUPPORTED)
	{
		EncodeData = &EncodeSSSE3;
		DecodeData = &DecodeSSSE3;
	}
	else
	{
		EncodeData = &EncodeALU;
		DecodeData = &DecodeALU;
	}
	is_initialized = TRUE;

	return 0;
}

void rs_destroy() 
{
	if (!is_initialized)
		return;
	for (int i = 0; i < CODES_COUNT; i++)
	{
		if (codes[i].coefs != NULL)
			free(codes[i].coefs);
		if (codes[i].lut != NULL)
			free(codes[i].lut);
	}
	is_initialized = FALSE;
}

int rs_encode(uint8_t* data, uint8_t* parity, int n, int k)
{
	union u8u16 id = { .code.k = (uint8_t)k, .code.n = (uint8_t)n };
	uint8_t* lut = NULL, * coefs;
	for (int i = 0; i < CODES_COUNT; i++)
	{
		if (codes[i].id.nk == id.nk) 
		{
			lut = codes[i].lut;
			coefs = codes[i].coefs;
			break;
		}
	}
	if (!lut)
		return -1;
	if (!parity)
		return EncodeData(n, k, lut, coefs, data);
	uint8_t buffer[256];
	memcpy_s(buffer, k, data, k);
	int result = EncodeData(n, k, lut, coefs, buffer);
	memcpy_s(parity, (size_t)n - k, buffer + k, (size_t)n - k);
	return result;
}

int rs_decode(uint8_t* data, uint8_t* parity, int n, int k)
{
	union u8u16 id = { .code.k = (uint8_t)k, .code.n = (uint8_t)n };
	uint8_t* lut = NULL;
	for (int i = 0; i < CODES_COUNT; i++)
	{
		if (codes[i].id.nk == id.nk)
		{
			lut = codes[i].lut;
			break;
		}
	}
	if (!lut)
		return -1;
	if (!parity)
		return DecodeData(n, k, lut, data);
	uint8_t buffer[256];
	memcpy_s(buffer, k, data, k);
	memcpy_s(buffer + k, (size_t)n - k, parity, (size_t)n - k);
	int result = DecodeData(n, k, lut, buffer);
	memcpy_s(data, k, buffer, k);
	return result;
}
