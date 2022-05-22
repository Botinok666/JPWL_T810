﻿#pragma once
/** 
 * \file  jpwl_encoder_types.h
 * \brief Файл описания описания типов и констант для библиотеки кодера JPWL
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12 
*/

#include <stdint.h>

#define _true_ 0x01				///<  Логическая истина
#define _false_ 0x00				///<  Логическая ложь

#define EPC_MARKER 0xff68	///< Значение маркера EPC 
#define EPB_MARKER 0xff66	///< Значение маркера EPB   
#define ESD_MARKER 0xff67	///< Значение маркера ESD  
#define SOC_MARKER 0xff4f	///< Значение маркера SOC 
#define SOT_MARKER 0xff90	///< Значение маркера SOT 
#define SOD_MARKER 0xff93	///< Значение маркера SOD 
#define EOC_MARKER 0xffD9	///< Значение маркера EOC 
#define SIZ_MARKER 0xff51	///< Значение маркера SIZ 
#define SOP_MARKER 0xff91	///< Значение маркера SOP 
#define BAD_ID	0x0123		///< Идентификатор невосстановимого блока
#define EPC_LOW 0x68	///< Значение второго байта маркера EPC 
#define EPB_LOW 0x66	///< Значение второго байта маркера EPB   
#define ESD_LOW 0x67	///< Значение второго байта маркера ESD  
#define RED_LOW 0x69	///< Значение второго байта маркера RED 
#define SOC_LOW 0x4f	///< Значение второго байта маркера SOC 
#define SOT_LOW 0x90	///< Значение второго байта маркера SOT 
#define SOD_LOW 0x93	///< Значение второго байта маркера SOD 
#define EOC_LOW 0xD9	///< Значение второго байта маркера EOC 
#define SIZ_LOW 0x51	///< Значение второго байта маркера SIZ 
#define SOP_LOW 0x91	///< Значение второго байта маркера SOP 
#define EMPTY_LOW	0xFF		/**< отсутствующий маркер*/

#define COD_LOW 0x52	///< Значение второго байта маркера COD 
#define COC_LOW 0x53	///< Значение второго байта маркера COC
#define QCD_LOW 0x5c	///< Значение второго байта маркера QCD
#define QCC_LOW 0x5d	///< Значение второго байта маркера QCC
#define RGN_LOW 0x5e	///< Значение второго байта маркера RGN
#define POC_LOW 0x5f	///< Значение второго байта маркера POC
#define PPM_LOW 0x60	///< Значение второго байта маркера PPM
#define TLM_LOW 0x55	///< Значение второго байта маркера TLM
#define PLM_LOW 0x57	///< Значение второго байта маркера PLM
#define CRG_LOW 0x63	///< Значение второго байта маркера CRG
#define COM_LOW 0x64	///< Значение второго байта маркера COM

#define EPB_LN 11		/**< Длина постоянной части сегмента марокера EPB Lepb+Depb+LDPepb+Pepb */
#define EPC_LN 9		/**< Длина постоянной части сегмента марокера EPC Lepc+Pcrc+DL+Pepc */
#define ESD_LN 5		/**< Длина постоянной части сегмента марокера ESD Lesd+Cesd+Pesd */
#define SOT_LN 10		/**< Длина сегмента маркера SOT без самого маркера */

#define ESDINT_LN 9	/**< Длина записи об одном интервале ESD в режиме байтового диапазона: адрес начала (4 байта) + адрес конца (4 байта) + чувствительность (1 байт) */
#define MAX_EPBSIZE 65535	/**< Максимальная длина сегмента маркера EPB - определяется кол-вом байт, отводимых под длину сегмента в спецификации T.810 (2 байта беззнаковое число) */
#define PRE_RSCODE_SIZE (40-13) /**< Длина RS-кода для защиты заголовка не первого EPB в заголовке, т.е. EPB, используемого для защиты данных */
#define MAX_MARKERS 80000	/**< Максимальное количество маркеров */
#define MAX_TILES 3072		/**< Максимальное количество тайлов */

#define JPWL_USING	1		/**< Использовать кодер JPWL */
#define JPWL_NOTUSING	0	/**< Не использовать кодер JPWL */
#define MAX_OUT_SIZE	(1UL << 24)	/**< Максимальный размер выходного кодового потока */
#define FAST_TILE_SEARCH	// альтернатива - SLOW_TILE_SEARCH

#define MAX_BADPARTS 13500	///< Макс. кол-во некорректируемых фрагментов пост-данных блока EPB
#define MAX_INTERVALS 6553 ///< Макс. кол-во интервалов RED - 1 REd по 10 байт на 1 интервал
#define TILE_MINLENGTH 80	///< Минимальная длина тайла

typedef unsigned char _bool_;
typedef unsigned char* addr_char;

/**
 * \struct epb_ms
 * \brief Структура данных для маркера EPB
 */
typedef struct {
	_bool_ latest;	// последний EPB в заголовке?
					// этот маркер расположен (-1 - основной заголовок)
	unsigned char index; ///< порядковый номер (индекс) блока EPB в своем заголовке
	int hprot;		///< метод защиты от ошибок: 0 - никакой метод не применяется, 1 - предопределенные RS-коды, 16 - CRC-16, 32 - СКС -32, 37-128 - RS-коды
	int k_pre;		///< длина  блока пре-данных
	int n_pre;		///< длина кодового слова пре-данных
	int pre_len;	///< длина защищаемых пре-данных
	int k_post;		///< длина  блока пост-данных
	int n_post;		///< длина кодового слова пост-данных
	int post_len;	///< длина защищаемых пост-данных
	// поля сегмента маркера
	unsigned short Lepb;	///< 2 байта - длина сегмента маркера без учета маркера
	unsigned char Depb;		///< 1 байт - стиль EPB
	unsigned long LDPepb;	///< 4 байта - длина защищаемых блоком данных
	unsigned long Pepb;		///< 4 байта - метод защиты данных
} epb_ms;

/**
 * \struct epc_ms
 * \brief Структура данных для маркера EPC
 */
typedef struct {
	_bool_ epb_on;	///< EPB используется в кодовом потоке?
	// поля сегмента маркера
	unsigned short Lepc;	///< 2 байта - длина сегмента маркера без самого маркера
	unsigned short Pcrc;	///< 2 байта - CRC для EPC, исключая само поле Pcrc
	unsigned long DL;	///< 4 байта - длина выходного кодового потока от SOC до EOC
	unsigned char Pepc;	///< 1 байт - применение в кодовом потоке средств jpwl
	unsigned short ID;	// 2 байта - идентификатор информативного метода
	unsigned short Lid;	// 2 байта - длина данных Pid для информативных методов
} epc_ms;

/**
 * \struct esd_ms
 * \brief Структура данных для маркера ESD
 */
typedef struct {
	unsigned char addrm;	// режим адресации кодового потока
							// 0 - пакетный режим,
							// 1 - байтовый диапазон,
							// 2 - диапазон пакетов
							// 3 - зарезервировано
	unsigned short interv_start;	// номер первого элемента массива e_intervals
									// с которого начинаются записи этого ESD
	unsigned short interv_cnt;		// кол-во записей в массиве e_intervals
	unsigned short Lesd;	///< 2 байта - длина сегмента маркера без самого маркера
	unsigned short Cesd;	// 2 байта - определяет, какой компонент ESD указывается
	unsigned char Pesd;		// 1 байт - описывает опции ESD
} esd_ms;

/**
 * \struct bad_block
 * \brief Структура для описания невосстановимого блока данных
 */
typedef struct {
	unsigned long Lbad;	///< длина невосстановимого блока
} bad_block;

/**
 * \struct w_marker
 * \brief Структура данных для описания одного из маркеров jpwl или невосстановимого блока
 */
typedef struct {
	unsigned short id;		///< Значение маркера (EPC_MARKER, EPB_MARKER, ESD_MARKER или RED_MARKER
	union {
		epb_ms epb;
		epc_ms epc;
		esd_ms esd;
		bad_block bad;
	} m;				///< Описание маркера или блока
	int tile_num;		///< Порядковый номер (индекс) заголовка Tile , в котором этот маркер (блок) расположен (-1 - основной заголовок)
	unsigned long pos_in;	///< Позиция маркера (блока) во входном буфере
	unsigned long pos_out;	///< Позиция маркера в выходном буфере
	unsigned long len;		///< Длирна сегмента маркера  (блока) без самого маркера
} w_marker;

/**
* \struct int_struct
* \brief Структура  для описания одного интервала чувствительности
*/
typedef struct {
	unsigned long start;	///<  Смещение первого байта интервала чувствительности относительно начала tile
	unsigned long end;	///<  Смещение последнего байта интервала чувствительности относительно начала tile
	unsigned char sens;	///<  Относительная чувствительность интервала
	unsigned char code;	///<  RS-код для интервала
} int_struct;

/**
 * \struct Restore_Statistic
 * \brief Структура данных для выдачи статистики по декодированию видеокадров в ПО ПИИ
 */
typedef struct {
	uint32_t fully_restored;
	uint32_t partially_restored;
	uint32_t not_restored;
	uint32_t not_JPWL;
	uint32_t corrected_rs_bytes;
	uint32_t uncorrected_rs_bytes;
} restore_stats;

typedef struct {
	unsigned char* inp_buffer;	///<  Адрес входного буфера
	unsigned long inp_length;	///<  Длина входного буфера в байтах
	unsigned char* out_buffer;	///<  Адрес выходного буфера
	unsigned long out_length;		///< Длина данных записанных в выходной буфер
} w_dec_params;

/**
 * \struct jpwl_dec_bParams
 * \brief Структура с входными параметрами  декодера JPWL
 */
typedef struct {
	unsigned char* inp_buffer;		///<  Адрес входного буфера
	unsigned long inp_length;		///<  Длина входного буфера в байтах
	unsigned char* out_buffer;		///<  Адрес выходного буфера
} jpwl_dec_bParams;

/**
 * \struct jpwl_dec_bResults
 * \brief Структура с выходными параметрами  декодера JPWL
 */
typedef struct {
	unsigned short tile_all_rest_cnt;	///< Количество полностью восстановленных тайлов кадра
	unsigned short tile_part_rest_cnt;	///< Количество частично восстановленных тайлов кадра
	unsigned long out_length;			///< Длина данных записанных в выходной буфер
	unsigned long all_bad_length;			///< Общее количество недекодированных данных кадра
} jpwl_dec_bResults;

typedef struct {
	unsigned char wcoder_mh;		///< значение wcoder_mh_param (0,1,16,32,37,38,40,43,45,48,51,53,56,64,75,80,85,96,112,128)
	unsigned char wcoder_th;		///< значение wcoder_th_param (0,1,16,32,37,38,40,43,45,48,51,53,56,64,75,80,85,96,112,128) 
	unsigned char wcoder_data;		///< значение wcoder_data_param  (0,1,16,32,37,38,40,43,45,48,51,53,56,64,75,80,85,96,112,128)
	unsigned char interleave_used;	///< значение interleave_use (1- Ammendment используется? 0 - не используется)
	unsigned char* inp_buffer;		///< адрес входного буфера
	unsigned char* out_buffer;		///< адрес выходного буфера
	unsigned short* tile_packets;	///< указатель на первый эл-т массива с кол-вом пакетов по тайлам
	unsigned char* packet_sense;	///< указатель на первый эл-т массива со значениями чувствительности пакетов тайлов
	unsigned long wcoder_out_len;	///< длина выходного кодового потока
	unsigned long wcoder_mh_len;	///< длина основного заголовка в байтах
	unsigned char jpwl_enc_mode;	///< режим использования кодера jpwl: 1 - использовать, 0 - не использовать
} w_enc_params;

/**
* \struct jpwl_enc_params
* \brief Структура для передачи значений параметров кодера jpwl из ПО ПИИ
*/
typedef struct {
	unsigned char wcoder_mh;	///< значение wcoder_mh_param (0,1,16,32,37,38,40,43,45,48,51,53,56,64,75,80,85,96,112,128)
	unsigned char wcoder_th;	///< значение wcoder_th_param (0,1,16,32,37,38,40,43,45,48,51,53,56,64,75,80,85,96,112,128)
	unsigned char wcoder_data;	///< значение wcoder_data_param (0,1,16,32,37,38,40,43,45,48,51,53,56,64,75,80,85,96,112,128)
	unsigned char interleave_used;		///< значение interleave_use (1- Ammendment используется? 0 - не используется)
	unsigned char jpwl_enc_mode;	///< режим использования кодера jpwl: 1 - использовать, 0 - не использовать
} jpwl_enc_params;

/**
* \struct jpwl_enc_bParams
* \brief Структура для передачи побочных параметров кодеру jpwl из ПО ПИИ
*/
typedef struct {
	unsigned long stream_len;				///< длина входного кодового потока
	unsigned short* tile_packets;		///< указатель на первый эл-т массива с кол-вом пакетов по тайлам (этот массив с таким же именем формирует кодер jpeg2000)
	unsigned char* pack_sens;			///< указатель на первый эл-т массива со значениями чувствительности пакетов тайлов (этот массив с таким же именем формирует кодер jpeg2000)
} jpwl_enc_bParams;

/**
* \struct jpwl_enc_bResults
* \brief Структура для возврата побочных результатов из кодера jpwl в ПО ПИИ
*/
typedef struct {
	unsigned long wcoder_out_len;	///< длина выходного кодового потока jpwl
	unsigned long wcoder_mh_len;		///< длина основного заголовка в байтах после кодера jpwl
} jpwl_enc_bResults;