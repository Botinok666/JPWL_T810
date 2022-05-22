#pragma once

#include "openjpeg/openjpeg.h"

typedef struct
{
	OPJ_UINT8* pData; //Our data.
	OPJ_SIZE_T dataSize; //How big is our data.
	OPJ_SIZE_T offset; //Where are we currently in our data.

} opj_memory_stream;

opj_stream_t* opj_stream_create_default_memory_stream(opj_memory_stream* p_memoryStream, OPJ_BOOL p_is_read_stream);
