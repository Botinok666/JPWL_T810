#pragma once
#include <stdint.h>

#define _true_ 0x01
#define _false_ 0x00

#define EPC_MARKER 0xff68
#define EPB_MARKER 0xff66
#define ESD_MARKER 0xff67
#define SOC_MARKER 0xff4f
#define SOT_MARKER 0xff90
#define SOD_MARKER 0xff93
#define EOC_MARKER 0xffD9
#define SIZ_MARKER 0xff51
#define SOP_MARKER 0xff91
#define BAD_ID	0x0123
#define EPC_LOW 0x68
#define EPB_LOW 0x66
#define ESD_LOW 0x67
#define RED_LOW 0x69
#define SOC_LOW 0x4f
#define SOT_LOW 0x90
#define SOD_LOW 0x93
#define EOC_LOW 0xD9
#define SIZ_LOW 0x51
#define SOP_LOW 0x91
#define EMPTY_LOW	0xFF

#define COD_LOW 0x52
#define COC_LOW 0x53
#define QCD_LOW 0x5c
#define QCC_LOW 0x5d
#define RGN_LOW 0x5e
#define POC_LOW 0x5f
#define PPM_LOW 0x60
#define TLM_LOW 0x55
#define PLM_LOW 0x57
#define CRG_LOW 0x63
#define COM_LOW 0x64

#define EPB_LN 11		/**< Длина постоянной части сегмента марокера EPB Lepb+Depb+LDPepb+Pepb */
#define EPC_LN 9		/**< Длина постоянной части сегмента марокера EPC Lepc+Pcrc+DL+Pepc */
#define ESD_LN 5		/**< Длина постоянной части сегмента марокера ESD Lesd+Cesd+Pesd */
#define SOT_LN 10		/**< Длина сегмента маркера SOT без самого маркера */

#define ESDINT_LN 9	/**< Длина записи об одном интервале ESD в режиме байтового диапазона: адрес начала (4 байта) + адрес конца (4 байта) + чувствительность (1 байт) */
#define MAX_EPBSIZE 65535	/**< Максимальная длина сегмента маркера EPB - определяется кол-вом байт, отводимых под длину сегмента в спецификации T.810 (2 байта беззнаковое число) */
#define PRE_RSCODE_SIZE (40-13) /**< Длина RS-кода для защиты заголовка не первого EPB в заголовке, т.е. EPB, используемого для защиты данных */
#define MAX_MARKERS 8096	/**< Максимальное количество маркеров */
#define MAX_TILES 1024		/**< Максимальное количество тайлов */

#define MAX_OUT_SIZE	(1UL << 24)	/**< Максимальный размер выходного кодового потока */

#define MAX_BADPARTS 13500	/// Макс. кол-во некорректируемых фрагментов пост-данных блока EPB
#define MAX_INTERVALS 6553 /// Макс. кол-во интервалов RED - 1 REd по 10 байт на 1 интервал
#define TILE_MINLENGTH 80	/// Минимальная длина тайла
#define JPWL_CODES 16

typedef unsigned char _bool_;
typedef unsigned char* addr_char;

/**
 * \struct epb_ms
 * \brief Структура данных для маркера EPB
 */
typedef struct {
	_bool_ latest;	// последний EPB в заголовке?
					// этот маркер расположен (-1 - основной заголовок)
	unsigned char index; /// индекс блока EPB в своем заголовке
	int hprot;		/// метод защиты от ошибок: 0 - нет, 1 - предопределенные RS-коды, 16 - CRC-16, 32 - CRC-32, 37-128 - RS-коды
	int k_pre;		/// длина  блока пре-данных
	int n_pre;		/// длина кодового слова пре-данных
	int pre_len;	/// длина защищаемых пре-данных
	int k_post;		/// длина  блока пост-данных
	int n_post;		/// длина кодового слова пост-данных
	int post_len;	/// длина защищаемых пост-данных
	// поля сегмента маркера
	unsigned short Lepb;	/// длина сегмента маркера без учета маркера
	unsigned char Depb;		/// стиль EPB
	unsigned long LDPepb;	/// длина защищаемых блоком данных
	unsigned long Pepb;		/// метод защиты данных
} epb_ms;

/**
 * \struct epc_ms
 * \brief Структура данных для маркера EPC
 */
typedef struct {
	_bool_ epb_on;	///< EPB используется в кодовом потоке?
	// поля сегмента маркера
	unsigned short Lepc;	/// длина сегмента маркера без самого маркера
	unsigned short Pcrc;	/// CRC для EPC, исключая само поле Pcrc
	unsigned long DL;	/// длина выходного кодового потока от SOC до EOC
	unsigned char Pepc;	/// применение в кодовом потоке средств jpwl
	unsigned short ID;	// идентификатор информативного метода
	unsigned short Lid;	// длина данных Pid для информативных методов
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
	unsigned short Lesd;	// длина сегмента маркера без самого маркера
	unsigned short Cesd;	// определяет, какой компонент ESD указывается
	unsigned char Pesd;		// описывает опции ESD
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
	unsigned short id;		/// Значение маркера (EPC_MARKER, EPB_MARKER, ESD_MARKER или RED_MARKER
	union {
		epb_ms epb;
		epc_ms epc;
		esd_ms esd;
		bad_block bad;
	} m;				/// Описание маркера или блока
	int tile_num;		/// индекс заголовка Tile, в котором этот маркер расположен (-1 - основной заголовок)
	unsigned long pos_in;	/// Позиция маркера во входном буфере
	unsigned long pos_out;	/// Позиция маркера в выходном буфере
	unsigned long len;		/// Длирна сегмента маркера без самого маркера
} w_marker;

/**
* \struct int_struct
* \brief Структура  для описания одного интервала чувствительности
*/
typedef struct {
	unsigned long start;	/// Смещение первого байта интервала чувствительности относительно начала tile
	unsigned long end;	///  Смещение последнего байта интервала чувствительности относительно начала tile
	unsigned char sens;	///  Относительная чувствительность интервала
	unsigned char code;	///  RS-код для интервала
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
	unsigned char* inp_buffer;
	unsigned long inp_length;
	unsigned char* out_buffer;
	unsigned long out_length;	/// Длина данных записанных в выходной буфер
} w_dec_params;

/**
 * \struct jpwl_dec_bParams
 * \brief Структура с входными параметрами  декодера JPWL
 */
typedef struct {
	unsigned char* inp_buffer;
	unsigned long inp_length;
	unsigned char* out_buffer;
} jpwl_dec_bParams;

/**
 * \struct jpwl_dec_bResults
 * \brief Структура с выходными параметрами  декодера JPWL
 */
typedef struct {
	unsigned short tile_all_rest_cnt;	/// Количество полностью восстановленных тайлов кадра
	unsigned short tile_part_rest_cnt;	/// Количество частично восстановленных тайлов кадра
	unsigned long out_length;			/// Длина данных записанных в выходной буфер
	unsigned long all_bad_length;		/// Общее количество недекодированных данных кадра
} jpwl_dec_bResults;

typedef struct {
	unsigned char wcoder_mh;
	unsigned char wcoder_th;
	unsigned char wcoder_data;
	unsigned char interleave_used;	/// 1 - Ammendment используется, 0 - не используется
	unsigned char* inp_buffer;
	unsigned char* out_buffer;
	unsigned short* tile_packets;
	unsigned char* packet_sense;
	unsigned long wcoder_out_len;	/// длина выходного кодового потока
	unsigned long wcoder_mh_len;	/// длина основного заголовка в байтах
	unsigned char jpwl_enc_mode;	/// 1 - использовать, 0 - не использовать
} w_enc_params;

/**
* \struct jpwl_enc_params
* \brief Структура для передачи значений параметров кодера jpwl
*/
typedef struct {
	unsigned char wcoder_mh;
	unsigned char wcoder_th;
	unsigned char wcoder_data;
	unsigned char interleave_used;	/// 1 - используется, 0 - не используется
	unsigned char jpwl_enc_mode;	/// 1 - использовать, 0 - не использовать
} jpwl_enc_params;

/**
* \struct jpwl_enc_bParams
* \brief Структура для передачи побочных параметров кодеру jpwl
*/
typedef struct {
	unsigned long stream_len;		/// длина входного кодового потока
	unsigned short* tile_packets;	/// указатель на первый эл-т массива с кол-вом пакетов по тайлам
	unsigned char* pack_sens;		/// указатель на первый эл-т массива со значениями чувствительности пакетов тайлов
} jpwl_enc_bParams;

/**
* \struct jpwl_enc_bResults
* \brief Структура для возврата побочных результатов из кодера jpwl
*/
typedef struct {
	unsigned long wcoder_out_len;	/// длина выходного кодового потока jpwl
	unsigned long wcoder_mh_len;	/// длина основного заголовка в байтах после кодера jpwl
	int tile_position[MAX_TILES];
	uint8_t tile_headers[MAX_TILES][16];
} jpwl_enc_bResults;
