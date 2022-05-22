/*
 * \file jpwl_encoder.cpp
 * \brief Файл с текстами программ библиотеки кодера JPWL
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12
*/

#include "math.h"
#include "memory.h"
#include <stdio.h>
#include <stdlib.h>
#include "crc.h"
#include "jpwl_types.h"
#include "jpwl_params.h"

#ifdef RS_OPTIMIZED
#include "rs64/rs64.h"
#else
#include "..\rs_crc_lib\rs_crc_decl.h"
#include "..\rs_crc_lib\rs_crc_import.h"
#endif // RS_OPTIMIZED

#ifndef _TEST
#define _TEST
#endif

#define W_OPTIMIZED

#define MARKER_COUNT_CHECK if(enc_markers_cnt==MAX_MARKERS) return(-1);
#define INTERV_COUNT_CHECK if(enc_interv_count==MAX_INTERVALS) return(-4);

int_struct e_intervals[MAX_INTERVALS];	///< Буфер для интервалов чувствительности данных тайлов

unsigned long AllMarkers_len;	///< Длина всех созданных маркеров
unsigned long amm_len;			///< Длина выходного потока при применении Ammendment
unsigned char* cur_pack;	//ссылка на чувствительность тек. пакета для 
_bool_ empty_stream;			///< Флаг пустого потока, состоящего только из основного заголовка
unsigned short enc_interv_count;	///< Счетчик записей об интервалах чувствительности в массиве e_intervals
w_marker enc_markers[MAX_MARKERS]; ///< Массив маркеров jpwl
unsigned short enc_markers_cnt;		///< Счетчик маркеров в массиве enc_markers 
unsigned short epb_count;		///< Количество EPB блоков в тайлах
unsigned long enc_epc_dl;			///< Длина выходного кодового потока
unsigned char* epc_point;		///< Адрес для записи карты EPB блоков в сегмент EPC
unsigned long h_length[MAX_TILES + 1]; ///< Массив длин заголовков
unsigned char imatrix[MAX_OUT_SIZE];	///< Массив для выполнения внутрикадрового чередования выходного потока
unsigned short pack_count;		///< Счетчик пакетов в данных о чувствительности
unsigned long Psot_new[MAX_TILES]; ///< Массив обновленных значений длин Psot тайлов 
unsigned short tile_count;		///< Счетчик тайлов
w_enc_params w_params;	///< Структура с параметрами кодера jpwl

/**
 * \brief Поиск заданного маркера в буфере
 * \param buf  Адрес начала входного буфера
 * \param marker  Значение второго байта искомого маркера
 * \param terminated_marker  Значение второго байта маркера-ограничителя, на котором поиск заканчивается
 * \param t_adr Побочный эффект: eсли маркер не найден, присваивает t_adr адрес найденного маркера-ограничителя
 * \return Cсылка на первий найденный маркер или NULL, если маркера нет
 */
uint8_t* mark_search(uint8_t* buf, uint8_t marker, uint8_t terminated_marker, addr_char * t_adr)
{
	uint8_t* mark = NULL;

	while (1)
	{
		if (*buf == 0xff) {
			if (*(buf + 1) == terminated_marker) {
				*t_adr = buf;
				break;
			}
			else if (*(buf + 1) == marker) {
				mark = buf;
				break;
			};
		}
		buf++;
	}
	return mark;
}

/**
 * \brief  Инициализация переменных и массивов кодера и проверка корректности значений параметров кодера, полученных из ПО ПИИ
 * \param inp_buf Ссылка на буфер с входным кодовым потоком
 * \return Возвращает код завершения: 0 - параметры корректны
 */
int w_enc_init(uint8_t* inp_buf)
{
	if (enc_markers_cnt == 0)
		memset(enc_markers, 0, sizeof(w_marker) * MAX_MARKERS);
	else					// последующие обнуления - enc_markers_cnt элементов
		memset(enc_markers, 0, sizeof(w_marker) * enc_markers_cnt);
	enc_markers_cnt = 0;
	enc_interv_count = 0;
	pack_count = 0;
	AllMarkers_len = 0;
	epb_count = 0;
	empty_stream = _false_;
	// Определяем наличие в кодовом потоке маркеров SOP
	uint8_t* unused = NULL;
	uint8_t* marker = mark_search(inp_buf, SOD_LOW, EOC_LOW, &unused);
	if (marker == NULL) {
		empty_stream = _true_;
		w_params.interleave_used = 0;
	}
	return 0;
}

/**
 * \brief На основании параметра защиты protection формирует и возвращает значение Pepb для блока EPB
 * \param protection Код варианта защиты
 * \return Сформированное значение Pepb
*/
uint32_t get_Pepb(uint8_t protection)
{
	if (protection == 0)		// нет защиты
		return 0xFFFFFFFF;
	else if (protection == 1)	// предопределенные RS-коды
		return 0x00000000;
	else if (protection == 16)	// crc-16
		return 0x10000000;
	else if (protection == 32)	// crc-32
		return 0x10000001;
	else if (protection == 37)		// RS(37,32)
		return 0x20002520;
	else if (protection == 38)		// RS(38,32)
		return 0x20002620;
	else if (protection == 41)		// RS(40,32)
		return 0x20002820;
	else if (protection == 43)		// RS(43,32)
		return 0x20002B20;
	else if (protection == 45)		// RS(45,32)
		return 0x20002D20;
	else if (protection == 48)		// RS(48,32)
		return 0x20003020;
	else if (protection == 51)		// RS(51,32)
		return 0x20003320;
	else if (protection == 53)		// RS(53,32)
		return 0x20003520;
	else if (protection == 56)		// RS(56,32)
		return 0x20003820;
	else if (protection == 64)		// RS(64,32)
		return 0x20004020;
	else if (protection == 75)		// RS(75,32)
		return 0x20004B20;
	else if (protection == 81)		// RS(80,32)
		return 0x20005020;
	else if (protection == 85)		// RS(85,32)
		return 0x20005520;
	else if (protection == 96)		// RS(96,32)
		return 0x20006020;
	else if (protection == 112)	// RS(112,32)
		return 0x20007020;
	else if (protection == 128)	// RS(128,32)
		return 0x20008020;
	else if (protection == 144)	// RS(144,32)
		return 0x20009020;
	else if (protection == 161)	// RS(160,32)
		return 0x2000A020;
	else if (protection == 176)	// RS(176,32)
		return 0x2000B020;
	else if (protection == 192)	// RS(192,32)
		return 0x2000C020;
	else return 0;
}

/**
 * \brief  Создание маркеров jpwl в основном заголовке
 * \details Побочный эффект:
 * передвигает tile на начало первого тайла, расположенного непосредственно
 * за основным заголовком
 * \param tile  Cсылка на начало буфера, в котором находится кодовый поток jpeg2000 часть 1.
 * \return Код завершения:  
 *  0 - все нормально, 
 * -1 - недостаточно места в массиве для размещения всех маркеров, 
 * -2 - не найдено ни одного тайла, 
 * -3 - слишком длинный заголовок, мало одного EPB, 
 * -4 - недостаточно места в массиве для интервалов чувствительности
*/
int enc_mh_markers_create(addr_char* codestream)
{
	uint8_t* v = NULL, * buf;
	uint16_t l = 0, d;
	epb_ms* epb;
	epc_ms* epc;
	uint32_t l_rs;

	buf = *codestream;
	uint8_t* marker = mark_search(buf, SOT_LOW, EOC_LOW, &v);	// ищем маркер SOT, расположенный за MH 
	if (marker == NULL)	{	// нет SOT 
		if (!empty_stream)				
			return -2;
		marker = mark_search(buf, EOC_LOW, EMPTY_LOW, &v);	// Ищем EOC
		if (marker == NULL)	// Нет EOC
			return -2;
		enc_epc_dl = (uint32_t)(marker - buf + 2);			// Длина входного кодового потока из осн. заголовка + маркер EOC
	}
	h_length[0] = (uint32_t)(marker - buf) - 1;		// Запомнили смещение посл. байта заголовка отн. его начала
	// формируем в l длину сегмента маркера SIZ
	v = (uint8_t*)&l; 
	*v = *(buf + 5); 
	*(v + 1) = *(buf + 4);
	l = l + 4;	// длина вместе с маркерами SOC и SIZ

	// создаем EPB
	enc_markers[0].id = EPB_MARKER;			// значение маркера
	enc_markers[0].pos_in = l;				// после SOC и сегмента SIZ
	enc_markers[0].pos_out = l;				// для первого маркера совпадает с pos_in
	epb = &enc_markers[0].m.epb;
	enc_markers[0].tile_num = -1;					// основной заголовок
	epb->index = 0;						// индекс=0
	epb->hprot = wcoder_mh_param;		// параметр из ПО ПИИ
	epb->k_pre = 64;					// предопределенное значение
	epb->n_pre = 160;					// предопределенное значение
	epb->pre_len = l + EPB_LN + 2;			// SOC, сегмент SIZ + заголовок EPB

	// вычисляем длину пост-данных (без чередования)
	d = (unsigned short)(marker - buf) - l;			// длина осн. заголовка от окончания
										// сегмента SIZ до конца заголовка
	// прибавляем длину маркера EPC (без чередования) вместе с маркером
	d += EPC_LN + 2;
	epb->post_len = d;
	if (wcoder_mh_param == 1) {
		epb->k_post = 64;
		epb->n_post = 160;
	}
	else if (wcoder_mh_param >= 37) {
		epb->k_post = 32;
		epb->n_post = wcoder_mh_param;
	}
	else {
		epb->k_post = 0;
		epb->n_post = 0;
	};
	// вычисляем длину сегмента маркера для разных вариантов защиты
	if (wcoder_mh_param == 1 || wcoder_mh_param >= 37) //  RS-код
		l_rs = (uint16_t)(ceil((double)d / epb->k_post)) * (epb->n_post - epb->k_post);
	else if (wcoder_mh_param == 16)				// CRC-16
		l_rs = 2;
	else if (wcoder_mh_param == 32)				// CRC-32
		l_rs = 4;
	else										// нет защиты
		l_rs = 0;
	l_rs += EPB_LN + 96;			// +длина постоянной части + длина RS-кодов для пре данных
	if (l_rs > MAX_EPBSIZE)				
		return -3; // одного EPB мало

	epb->Lepb = (uint16_t)l_rs;	// Длина EPB без маркера
	enc_markers[0].len = epb->Lepb;	
	AllMarkers_len += l_rs + 2;	// длина сегмента + сам маркер
	h_length[0] += l_rs + 2;	
	epb->Depb = 0xC0;				// последний в заголовке, упакованный, индекс=0
	epb->LDPepb = epb->pre_len + epb->post_len;	// защищаемая длина пре-данных + пост-данных
	// формируем поле Pepb с описанием метода защиты данных табл. А.6-А.8
	epb->Pepb = get_Pepb(wcoder_mh_param);

	// Создаем маркер EPC без информативных методов
	// DL и контрольная сумма заполняются позднее
	enc_markers[1].id = EPC_MARKER;			// Идентификатор маркера
	enc_markers[1].pos_in = l;				// после SOC и сегмента SIZ 
	enc_markers[1].pos_out = l + AllMarkers_len;	// позиция в вых. буфере отличается от позиции во входном 
											// на длину ранее созданных сегментов
	enc_markers[1].len = EPC_LN;					// Длина сегмента без самого маркера
	enc_markers[1].tile_num = -1;				// основной заголовок
	epc = &enc_markers[1].m.epc;
	epc->Pepc = 0x40;		// Info-,EPB+, RED-, E
	epc->Lepc = EPC_LN;			// Длина сегмента без самого маркера
	AllMarkers_len += EPC_LN + 2;	// добавляем длину сегмента + сам маркер
	h_length[0] += EPC_LN + 2;		// Увеличиваем длину заголовка на длину EPC

	enc_markers_cnt += 2;
	*codestream = marker;	// устанавливаем буфер на первый тайл

	return 0;
}

/**
 * \brief  Создание маркеров jpwl в  заголовке тайла
 * \details Побочный эффект:
 * передвигает tile на начало следующего тайла или присваивает ему NULL,
 * если следующего тайла нет
 * \param tile Ссылка на тайл кодового потока jpeg2000 часть 1
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока  (маркера SOC) тайла
 * \param pack_sens Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 255). В массиве pack_sens сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов, затем второго и т.д.
 * \param buf_start Ссылка на начало всего кодового потока, т.е. на начало основного заголовка
 * \return Код завершения: 0 - все нормально, -1 - недостаточно места в массиве для размещения всех маркеров, -2 - ошибки в кодовом потоке jpeg2000 часть1, -3 - слишком большой заголовок, недостаточно одного EPB,-4 - недостаточно места в массиве для интервалов чувствительности
*/
int enc_th_markers_create(addr_char* tile, uint16_t* tile_packets, uint8_t* pack_sens,
	uint8_t* buf_start)
{
	uint8_t* p, * v, * g, * p_start, * buf_new, * buf;
	uint8_t epb_ind, data_p, rs, ses;
	uint16_t i_s, i_k;
	uint32_t l, l_rs;
	int i, d, intrv_max, intrv_ln, AllTileEpb_ln;
	double dd;
	epb_ms* epb;

	if (empty_stream) {		// Если поток не содержит тайлов
		*tile = NULL;
		return 0;
	};
	AllTileEpb_ln = 0;
	v = NULL;
	buf = *tile;
	p = mark_search(buf, SOD_LOW, EOC_LOW, &v);	// ищем  маркер SOD - конец заголовка тайла 
	if (p == NULL)								// нет маркера SOD - ошибочный кодовый поток
		return -2;
	h_length[tile_count + 1] = (uint32_t)(p - buf) + 1; // смещение последнего байта заголовка тайла относительно начала
	p += 2;								// устанавливаем p на начало первого пакета данных тайла
	p_start = p;							// p_start - начало первого пакета в тайле			
	g = mark_search(buf + 2, SOT_LOW, EOC_LOW, &v);	// ищем следующий маркер  SOT 
	buf_new = g;							// ссылка на следующий тайл или NULL, усли он отсутствует
	if (g == NULL) {						// нет маркера SOT - найден маркер конца EOC (т.е. тайл явл. последним)
		g = v + 2;							// устанавливаем g на адрес первого байта после конца данных последнего тайла
		enc_epc_dl = (uint32_t)(g - buf_start);	// длина входного кодового потока
	};
	i_s = enc_interv_count;					// индекс начального интервала данных о чувствительности пакетов тайла
										// в массиве e_intervals
	i_k = 0;								// начальное хначение кол-ва интервалов чувствительности

	
	// чувствительность задается одним интервалом от начала первого до конца последнего пакета
	dd = 0;
	uint8_t* cur_sens = pack_sens + pack_count;
	for (i = 0; i < tile_packets[tile_count]; i++, cur_sens++)
		dd += (double)*cur_sens;
	dd /= tile_packets[tile_count];
	ses = (uint8_t)dd;						// чувствительность интервала

	i_k++;
	e_intervals[enc_interv_count].start = (uint32_t)(p_start - buf);	// начало интервала - начало тайла

	if (wcoder_data_param >= 37) {
		e_intervals[enc_interv_count].sens = ses;		// чувствительность интервала
		rs = e_intervals[enc_interv_count].code = wcoder_data_param;
		// и вычисляем максимально возможную длину интервала
		// для одного EPB	
		intrv_max = (int)(floor((double)(MAX_EPBSIZE - EPB_LN - PRE_RSCODE_SIZE) 
			/ (e_intervals[enc_interv_count].code - 32)) * 32);
		intrv_ln = (int)(g - 1 - p_start);			// фактическая длина интервала
		// дробим  интервал на несколько, каждый из которых целиком может быть защищен одним EPB
		for (; intrv_ln >= intrv_max; intrv_ln -= intrv_max) {
			e_intervals[enc_interv_count].end = e_intervals[enc_interv_count].start + intrv_max - 1;
			// начало следующего интервала - через 1 байт после конца предыдущего
			if (intrv_ln > intrv_max) {			// не все байты вошли в созданный интервал
				i_k++;
				INTERV_COUNT_CHECK
					e_intervals[++enc_interv_count].start = e_intervals[enc_interv_count - 1].end + 1;
				e_intervals[enc_interv_count].code = rs;
				e_intervals[enc_interv_count].sens = ses;		// чувствительность интервала
			}
		};
		if (intrv_ln > 0)				// заканчиваем последний интервал
			e_intervals[enc_interv_count].end = e_intervals[enc_interv_count].start + intrv_ln;
	}
	else {
		i_k = 1;
		e_intervals[enc_interv_count].sens = (uint8_t)dd;			// чувствительность интервала
		e_intervals[enc_interv_count].end = (uint32_t)(g - 1 - buf); // конец  последнего интервала -
												// последний байт данных текущего тайла или
												// последний байт маркера EOC последнего тайла
	};
	INTERV_COUNT_CHECK
		enc_interv_count++;
	pack_count += tile_packets[tile_count];			// прибавляем в pack_count кол-во обработанных значений о чувствительности
				
	// создаем блоки EPB в заголовке тайла
	epb_count++;								// Подсчет количества EPB блоков для реализации Ammendment
	epb_ind = 0;									// индекс текущего EPB в заголовке
	MARKER_COUNT_CHECK
		enc_markers[enc_markers_cnt].id = EPB_MARKER;			// значение маркера
	l = (uint32_t)(buf - buf_start) + SOT_LN + 2;		// смещение относительно начала всего кодового потока
												// куда будет вставляться маркер EPB для защиты заголовка тайлша
												// туда же (т.е. после него) будут вставляться EPBдля защиты данных и ESD
	enc_markers[enc_markers_cnt].pos_in = l;				// после сегмента SOT
	enc_markers[enc_markers_cnt].pos_out = l + AllMarkers_len;	// поз. вых буфера = поз. входн. буфера + длина всех добавленных ранее сегментов
	epb = &enc_markers[enc_markers_cnt].m.epb;			// ссылка на EPB в Union и инкремент кол-ва созданных маркеров
	enc_markers[enc_markers_cnt].tile_num = tile_count;			// индекс текущего тайла
	epb->index = 0;
	epb->hprot = wcoder_th_param;
	epb->k_pre = 25;
	epb->n_pre = 80;
	epb->pre_len = SOT_LN + 2 + EPB_LN + 2;	// сегмент SOT + маркер SOT + заголовок EPB + маркер EPB

	// вычисляем длину пост-данных 
	d = (uint32_t)(p_start - buf) - SOT_LN - 2; // длина  заголовка от окончания сегмента SOT до конца заголовка
	epb->post_len = d;
	if (wcoder_th_param == 1) {
		epb->k_post = 25;
		epb->n_post = 80;
	}
	else if (wcoder_th_param >= 37) {
		epb->k_post = 32;
		epb->n_post = wcoder_th_param;
	}
	else {
		epb->k_post = 0;
		epb->n_post = 0;
	};
	// вычисляем длину сегмента маркера для разных вариантов защиты
	if (wcoder_th_param == 1 || wcoder_th_param >= 37) //  RS-код
		l_rs = (uint16_t)(ceil((double)d / epb->k_post)) * (epb->n_post - epb->k_post);
	else if (wcoder_th_param == 16)				// CRC-16
		l_rs = 2;
	else if (wcoder_th_param == 32)				// CRC-32
		l_rs = 4;
	else										// нет защиты
		l_rs = 0;

	l_rs += EPB_LN + 55;						// +длина постоянной части + длина RS-кодов для
												// пре данных
	if (l_rs > MAX_EPBSIZE)				// одного EPB мало
		return -3;
	epb->Lepb = (uint16_t)l_rs; // Длина EPB без маркера
	AllMarkers_len += l_rs + 2;
	enc_markers[enc_markers_cnt++].len = epb->Lepb; // Длина EPB без маркера
	AllTileEpb_ln = epb->Lepb + 2;	// длина всего EPB вместе с маркером
	if (wcoder_data_param == 0)	// нет защиты данных, больше EPB не будет
		epb->Depb = 0xC0;			// последний в заголовке, упакованный, индекс=0
	else
		epb->Depb = 0x80;			// не последний в заголовке, упакованный, индекс=0
	epb->LDPepb = epb->pre_len + epb->post_len +	// защищаемая длина пре-данных + пост-данных
											// + длина всех данных тайла, если пост-данные заголовка и данные тайла не защищаются
		((wcoder_th_param == 0 && wcoder_data_param == 0) ? e_intervals[enc_interv_count - 1].end - e_intervals[enc_interv_count - 1].start + 1 : 0);
	// формируем поле Pepb с описанием метода защиты данных табл. А.6-А.8
	epb->Pepb = get_Pepb(wcoder_th_param);
		// создаем переменное количество блоков для защиты данных 
		// по 1 блоку на каждый интервал чувствительности
	if (wcoder_data_param != 0) {
		for (i = 0; i < i_k; i++) {
			MARKER_COUNT_CHECK
				epb_count++;								// Подсчет количества EPB блоков для реализации Ammendment
			enc_markers[enc_markers_cnt].id = EPB_MARKER;			// значение маркера
			enc_markers[enc_markers_cnt].pos_in = l;				// после сегмента SOT
			enc_markers[enc_markers_cnt].pos_out = l + AllMarkers_len;	// позиция в вых. буфере 
			epb = &enc_markers[enc_markers_cnt].m.epb;			// ссылка на EPB в Union и инкремент кол-ва созданных маркеров
			//			epb->latest=(i==i_k-1?_true_:_false_);	// последний в заголовке, если обрабатывается последний интервал
															// и не последний, если не последний интервал
			//			epb->packed=_true_;					// упакованный
			enc_markers[enc_markers_cnt].tile_num = tile_count;			// индекс текущего тайла
			epb->index = ++epb_ind;				// индекс EPB в заголовке
			epb->hprot = wcoder_data_param;
			epb->k_pre = 13;
			epb->n_pre = 40;
			epb->pre_len = EPB_LN + 2;			// заголовок EPB + маркер EPB
			// вычисляем длину пост-данных 
			epb->post_len = d = (int)(e_intervals[i_s + i].end - e_intervals[i_s + i].start + 1); // длина  интервала чувствительности		
			if (wcoder_data_param >= 37) {
				epb->k_post = 32;
				epb->n_post = wcoder_data_param;
			}
			else {
				epb->k_post = 0;
				epb->n_post = 0;
			};
			// вычисляем длину сегмента маркера для разных вариантов защиты
			if (wcoder_data_param >= 37) //  RS-код
				l_rs = (uint16_t)(ceil((double)d / epb->k_post)) * (epb->n_post - epb->k_post);
			else if (wcoder_data_param == 16)				// CRC-16
				l_rs = 2;
			else if (wcoder_data_param == 32)				// CRC-32
				l_rs = 4;
			else										// нет защиты
				l_rs = 0;
			l_rs += EPB_LN + 27;			// +длина постоянной части + длина RS-кодов для пре данных
			if (l_rs > MAX_EPBSIZE)				// одного EPB мало
				return -3;
			epb->Lepb = (uint16_t)l_rs; 		// Длина EPB без маркера
			AllMarkers_len += l_rs + 2;				// наращиваем длину созданных сегментов
			enc_markers[enc_markers_cnt++].len = epb->Lepb; // Длина EPB без маркера
			AllTileEpb_ln += epb->Lepb + 2;		// вычисляем общую длину всех EPB тайла вместе с их маркерами
			if (i == i_k - 1)			// это последний интервал, больше EPB не будет
				epb->Depb = 0xC0 | (uint8_t)(epb->index & 0x3f); // последний в заголовке, упакованный
			else
				epb->Depb = 0x80 | (uint8_t)(epb->index & 0x3f);	// не последний в заголовке, упакованный
			epb->LDPepb = epb->pre_len + epb->post_len;	// защищаемая длина пре-данных + пост-данных
			// формируем поле Pepb с описанием метода защиты данных табл. А.6-А.8
			data_p = wcoder_data_param;
			if (data_p == 1)
				data_p = epb->n_post;
			epb->Pepb = get_Pepb(data_p);
		}
	}

	for (i = 0; i < i_k; i++) {
		e_intervals[i + i_s].start += AllTileEpb_ln;
		e_intervals[i + i_s].end += AllTileEpb_ln;
	}
	// Увеличиваем длину заголовка на ту же величину (все EPB + все ESD)
	h_length[tile_count + 1] += AllTileEpb_ln;
	// Корректируем длину тайла в сегменте SOT
	l = _byteswap_ulong(*(uint32_t*)(*tile + 6));
	Psot_new[tile_count] = l + AllTileEpb_ln; // увеличиваем l на сумму длин внедряемых данных
	*tile = buf_new;
	return 0;
}

/**
 * \brief Создание маркеров jpwl в массиве enc_markers
 * \details Вызывает функции создания маркеров в основном заголовке и заголовках тайлов
 * \param inp_buf Ссылка на начало буфера, в котором находится кодовый поток jpeg200 часть 1
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 * \param pack_sens Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 255). 
 * В массиве pack_sens сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов
 */
errno_t enc_w_markers_create(uint8_t* inp_buf, uint8_t* out_buf, uint16_t* tile_packets, uint8_t* pack_sens)
{
	uint8_t* p = inp_buf;
	int exit_code, i;
	uint32_t epc_plus_size, epb0_plus_size, l_rs;
	double f;

	exit_code = enc_mh_markers_create(&p);
	if (exit_code) {
		switch (exit_code)
		{
		case -1: return -4;
		case -2: return -2;
		case -3: return -3;
		case -4: return -5;
		};
	}
	// цикл, перебирающий тайлы
	pack_count = 0;
	for (tile_count = 0; p != NULL; tile_count++) {
		exit_code = enc_th_markers_create(&p, tile_packets, pack_sens, inp_buf);
		if (exit_code) { // создаем маркеры в заголовке тайла
			switch (exit_code)
			{
			case -1: return -4;
			case -2: return -2;
			case -3: return -3;
			case -4: return -5;
			};
		}
		if (tile_count == (MAX_TILES - 1) && p != NULL)
			return -1;
	};
	// Здесь в случае использования внутрикадрового интерлейсинга отводится 
	// место под карту EPB в маркере EPC и изменяются размеры маркеров EPC и EPB основного заголовка
	epc_plus_size = w_params.interleave_used ? 6 + 10 * epb_count : 0;		// Увеличение размера EPC при использовании Ammendment
	epb0_plus_size = 0;				// Увеличение размера первого EPB при использовании Ammendment
	if (w_params.interleave_used) {	// Коррекция длин и позиций маркеров при использовании Ammendment
		enc_markers[1].len += (uint16_t)epc_plus_size;	// Коррекция длины EPC
		enc_markers[1].m.epc.Lepc = (uint16_t)enc_markers[1].len;
		enc_markers[1].m.epc.Pepc |= 0x80;	// Установка в EPC признака использования информативных методов
		enc_markers[0].m.epb.LDPepb += epc_plus_size;	// Увеличиваем длину защищаемых данных для первого EPB
		enc_markers[0].len = (uint16_t)enc_markers[0].m.epb.LDPepb;
		enc_markers[0].m.epb.post_len += epc_plus_size;	// Увеличиваем длину пост-данных для первого EPB

		f = (double)enc_markers[0].m.epb.post_len;
		if (wcoder_mh_param == 1 || wcoder_mh_param >= 37) // RS-код
			l_rs = (uint16_t)(ceil(f / enc_markers[0].m.epb.k_post)) * 
				(enc_markers[0].m.epb.n_post - enc_markers[0].m.epb.k_post);
		else if (wcoder_mh_param == 16)				// CRC-16
			l_rs = 2;
		else if (wcoder_mh_param == 32)				// CRC-32
			l_rs = 4;
		else										// нет защиты
			l_rs = 0;
		l_rs += EPB_LN + 96;			// + длина постоянной части + длина RS-кодов для пре данных

		epb0_plus_size = (uint16_t)(l_rs - enc_markers[0].m.epb.Lepb);	// вычисляем добавку к длине сегмента первого EPB при использовании Ammendment
		enc_markers[0].m.epb.Lepb = (uint16_t)enc_markers[0].len = (uint16_t)l_rs;	// Обновляем длину сегмента первого EPB
		enc_markers[1].pos_out += epb0_plus_size;			// Корректируем позицию EPC в вых. буфере на величину увеличения первого EPB
		h_length[0] += epc_plus_size + epb0_plus_size;		// Коррекция длины основного заголовка
		for (i = 2; i < enc_markers_cnt; i++) {		// Коррекция позиции в выходном буфере всех маркеров после EPC на величину увеличения первого EPB и EPC
			enc_markers[i].pos_out += epb0_plus_size + epc_plus_size;
		};
	};
	// длина выходного потока = длина входного + добавленных сегментов
	enc_epc_dl += AllMarkers_len + epc_plus_size + epb0_plus_size;
	enc_markers[1].m.epc.DL = enc_epc_dl;	// заносим DL в EPC

	return 0;
}

/**
 * \brief  Копирование данных из входного буфера (jpeg2000 часть1) на свои места в выходном буфере (jpeg2000 частьII)
 * \details Данные копируются на свои места, пропуская места, которые займут
 *		сегменты маркеров jpwl
 * \param inbuf Входной буфер, содержащий кодовый поток jpeg2000 часть1
 * \param outbuf  Выходной буфер, в который будут скопированы данные из входного буфера
 */
void enc_data_copy(uint8_t* inp_buf, uint8_t* out_buf) {
	uint8_t* o_b = out_buf;
	uint32_t i;
	size_t len, j = 0;

	for (i = 0; i < enc_markers_cnt; i++) {
		len = out_buf + enc_markers[i].pos_out - o_b;
		memcpy(o_b, inp_buf, len);
		o_b += len + enc_markers[i].len + 2;// пропускаем в вых. буфере место под сегмент маркера + маркер
		inp_buf += len;
		j += len + enc_markers[i].len + 2;
	}
	if (enc_epc_dl > j)
		memcpy(o_b, inp_buf, (size_t)enc_epc_dl - j);
}

/**
 * \brief Копирование в выходной буфер маркера и сегмента маркера EPB кроме входящих в сегмент кодов четности
 * \param epb Ссылка на структуру w_marker с параметрами маркера EPB
 * \param outbuf Выходной буфер, в который выполняется копирование
 */
void enc_epb_copy(w_marker* marker, uint8_t* out_buf) {
	uint8_t* c = out_buf + marker->pos_out;
	uint16_t us = 0;
	epb_ms* e = &marker->m.epb;

	// первый EPB в заголовке - следует скорректировать в сегменте маркера SOT адрес старшего байта Psot
	if (marker->tile_num >= 0 && e->index == 0) {
		uint32_t* p = (uint32_t*)(Psot_new + marker->tile_num); // адрес мл. байта нового значения Psot
		*(uint32_t*)(c - 6) = _byteswap_ulong(*p);
	};
	*(uint16_t*)c = _byteswap_ushort(marker->id);
	c += 2;
	*(uint16_t*)c = _byteswap_ushort(e->Lepb);
	c += 2;
	*c++ = e->Depb;
	*(uint32_t*)c = _byteswap_ulong(e->LDPepb);
	c += 4;
	*(uint32_t*)c = _byteswap_ulong(e->Pepb);
	c += 4;
	if (w_params.interleave_used && marker->tile_num >= 0) {	// Используем Ammendment EPB в тайлах
		*(uint32_t*)epc_point = _byteswap_ulong(e->Pepb); // Запись RSepb = Pepb 4 байта
		epc_point += 4;		
		*(uint16_t*)epc_point = _byteswap_ushort(e->Lepb); // Запись Lebp 2 байта
		epc_point += 2;		
		*(uint32_t*)epc_point = _byteswap_ulong(marker->pos_out); // Запись Oepb
		epc_point += 4;
	}
}

/**
 * \brief Копирование в выходной буфер маркера и сегмента маркера EPC
 * \details Копирование выполняется по принципу "big endian" - старшие байты вперед
 * Входные параметры:
 * \param epb  Ссылка на структуру w_marker с параметрами маркера EPC
 * \param outbuf  Выходной буфер, в который выполняется копирование
 */
void enc_epc_copy(w_marker* marker, unsigned char* out_buf) {
	uint8_t* c = out_buf + marker->pos_out;
	uint16_t Lid;
	epc_ms* e = &marker->m.epc;

	*(uint16_t*)c = _byteswap_ushort(marker->id);
	c += 2;
	*(uint16_t*)c = _byteswap_ushort(e->Lepc);
	c += 2;
	*(uint16_t*)c = _byteswap_ushort(e->Pcrc);
	c += 2;
	*(uint32_t*)c = _byteswap_ulong(e->DL);
	c += 4;
	*c++ = e->Pepc;
	if (w_params.interleave_used) {	// Используем Ammendment
		*c++ = 0x02;				// ID=0x0200 - внутрикадровое чередование
		*c++ = 0x00;
		Lid = 2 + epb_count * 10;
		*(uint16_t*)c = _byteswap_ushort(Lid);
		c += 2;
		*(uint16_t*)c = _byteswap_ushort(epb_count);
		c += 2;
		epc_point = c;
	}
}

/**
 * \brief Копирование в выходной буфер маркера и сегмента маркера ESD
 * Входные параметры:
 * \param epb  Ссылка на структуру w_marker с параметрами маркера ESD
 * \param outbuf  Выходной буфер, в который выполняется копирование
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 */
void enc_esd_copy(w_marker* marker, uint8_t* out_buf, uint16_t* tile_packets) {
	uint8_t* c = out_buf + marker->pos_out;
	int i;
	esd_ms* e = &marker->m.esd;

	*(uint16_t*)c = _byteswap_ushort(marker->id);
	c += 2;
	*(uint16_t*)c = _byteswap_ushort(e->Lesd);
	c += 2;
	*(uint16_t*)c = _byteswap_ushort(e->Cesd);
	c += 2;
	*c++ = e->Pesd;
	if (e->addrm == 1 && e->interv_cnt == 0) { // байтовый диапазон без интервалов
		*(uint32_t*)c = 0;	// смещение начала заголовка 
		c += 4;
		*(uint32_t*)c = _byteswap_ulong(h_length[marker->tile_num + 1]); // смещение последнего байта заголовка
		c += 4;
		*c++ = 0xff;
	}
	else if (e->addrm == 1 && e->interv_cnt != 0) // байтовый диапазон с интервалами
		for (i = e->interv_start; i < e->interv_start + e->interv_cnt; i++) {
			*(uint32_t*)c = _byteswap_ulong(e_intervals[i].start);
			c += 4;		
			*(uint32_t*)c = _byteswap_ulong(e_intervals[i].end);
			c += 4;
			*c++ = e_intervals[i].sens;
		}
	else {				// пакетный режим - данные об отн. чувствительности пакетов тайла
		for (i = 0; i < tile_packets[marker->tile_num]; i++) // копируем чувствительности пакетов
			*c++ = *cur_pack++;
	};
}

/**
 * \brief  Копирование маркеров в выходной буфер
 * \details Копирование маркеров, параметров сегментов маркеров и данных сегментов ESD
 * в выходной буфер. Смещение первого байта, начиная с которого выполняется копирование маркера,
 * задается предварительно вычисленным значением enc_markers[i].pos_out
 * \param outbuf  Выходной буфер, в который выполняется копирование
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 * \param pack_sens Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 255). В массиве pack_sens сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов, затем второго и т.д.
 */
void enc_markers_copy(uint8_t* out_buf, uint16_t* tile_packets, uint8_t* pack_sens) {
	int i;

	cur_pack = pack_sens;
	for (i = 0; i < enc_markers_cnt; i++)	// перебор маркеров из массива enc_markers
		switch (enc_markers[i].id) {
		case EPB_MARKER:					// это маркер EPB
			enc_epb_copy(enc_markers + i, out_buf);	// копирование маркера EPB
			break;
		case EPC_MARKER:					// это маркер EPC
			enc_epc_copy(enc_markers + i, out_buf);	// копирование маркера EPC
			break;
		case ESD_MARKER:					// это маркер ESD
			enc_esd_copy(enc_markers + i, out_buf, tile_packets);	// копирование маркера ESD
		};
}

/**
 * \brief  Заполнение блоков EPB
 * \details Заполнение блоков EPB, расположенных в выходном буфере, кодами четности
 * в соответствии с предварительно установленными параметрами защиты в этих блоках.
 * \param  outbuf Адрес выходного буфера, который заполнен всеми данными и сегменнтами маркеров jpwl кроме кодов четности блоков EPB
 */
void enc_fill_epb(uint8_t* out_buf)
{
	uint8_t* postrs_start;			// начало кодов четности для пост-данных в выходном буфере
	uint8_t* postdata_start;		// адрес начала пост-данных в вых. буфере 
	uint8_t data_buf[64];
	uint8_t* tile_adr = out_buf;	// адрес текущего тайла
	uint16_t crc16_buf;
	int i, j, l = 0, n_rs_old, k_rs_old, n_rs, k_rs;
	uint32_t crc32_buf;
	int_struct* cur_int;
	epb_ms* e;

	cur_int = e_intervals;				// ссылка на первый интервал чувствительности
	n_rs_old = k_rs_old = 0;
	for (i = 0; i < enc_markers_cnt; i++) {
		if (enc_markers[i].id == EPB_MARKER) {
			e = &enc_markers[i].m.epb;			// ссылка на данные о EPB в массиве маркеров
			if (e->index == 0) {				// первый EPB в заголовке
				if (enc_markers[i].tile_num < 0) {				// основной заголовок
#ifndef RS_OPTIMIZED
					if (n_rs_old != 160 || k_rs_old != 64)
						init_rs(160, 64);
					n_rs_old = 160; 
					k_rs_old = 64;
#endif // !RS_OPTIMIZED
					if (e->pre_len != e->k_pre) {
						memset(data_buf, 0, 64);
						memcpy(data_buf, out_buf, e->pre_len); // копируем кодируемые данные в начало буфера
						encode_RS(data_buf, out_buf + enc_markers[i].pos_out + EPB_LN + 2, 160, 64);
					}
					else {
						encode_RS(out_buf, out_buf + enc_markers[i].pos_out + EPB_LN + 2, 160, 64);
					};
					postrs_start = out_buf + enc_markers[i].pos_out + EPB_LN + 2 + 96; // адрес начала кодов четности для пост-данных
					// адрес начала пост-данных: вых. буфер + смещение последнего байта осн. заголовка - длина пост-данных + 1
					postdata_start = out_buf + h_length[0] - e->post_len + 1;
				}
				else {	// заголовок тайла
#ifndef RS_OPTIMIZED
					if (n_rs_old != 80 || k_rs_old != 25)
						init_rs(80, 25);
					n_rs_old = 80; 
					k_rs_old = 25;
#endif // !RS_OPTIMIZED
					encode_RS(out_buf + enc_markers[i].pos_out - SOT_LN - 2, out_buf + enc_markers[i].pos_out + EPB_LN + 2, 80, 25);
					postrs_start = out_buf + enc_markers[i].pos_out + EPB_LN + 2 + 55;	// позиция начала RS-кодов в вых. буфере
					// начало тайла = начало первого EPB в заголовке тайла - длина сегмента SOT - длина маркера SOT
					tile_adr = out_buf + enc_markers[i].pos_out - SOT_LN - 2;
					// адрес начала пост-данных: началo тайла + смещение последнего байта заголовка тайла - длина пост-данных + 1
					postdata_start = tile_adr + h_length[enc_markers[i].tile_num + 1] - e->post_len + 1;
				}
			}
			else {	// не первый EPB в заголовке (защита данных тайла)
#ifndef RS_OPTIMIZED
				if (n_rs_old != 40 || k_rs_old != 13)
					init_rs(40, 13);
				n_rs_old = 40; 
				k_rs_old = 13;
#endif // !RS_OPTIMIZED
				encode_RS(out_buf + enc_markers[i].pos_out, out_buf + enc_markers[i].pos_out + EPB_LN + 2, 40, 13);
				postrs_start = out_buf + enc_markers[i].pos_out + EPB_LN + 2 + 27;
				postdata_start = tile_adr + cur_int++->start; // адрес пост данных = адрес тайла + смещение тек.интервала
			};
			// кодируем пост-данные
			if (e->hprot == 16) {
				crc16_buf = CRC16(postdata_start, e->post_len);	
				*(uint16_t*)postrs_start = _byteswap_ushort(crc16_buf);
				postrs_start += 2;
			}
			else if (e->hprot == 32) {	// CRC-32
				crc32_buf = CRC32(postdata_start, e->post_len);
				*(uint32_t*)postrs_start = _byteswap_ulong(crc32_buf);
				postrs_start += 4;
			}
			else if (e->hprot != 0) {	// RS-код
				if (e->hprot == 1)
					if (enc_markers[i].tile_num < 0) {
						n_rs = 160;
						k_rs = 64;
					}
					else {
						n_rs = 80;
						k_rs = 25;
					}
				else {
					n_rs = e->hprot;
					k_rs = 32;
				};
#ifndef RS_OPTIMIZED
				if (n_rs != n_rs_old || k_rs != k_rs_old) {
					init_rs(n_rs, k_rs);		// инициализируем кодер RS на новые параметры
					n_rs_old = n_rs;			// запоминаем параметры последней инициализации
					k_rs_old = k_rs;
				};
#endif // !RS_OPTIMIZED
				for (j = e->post_len; j >= k_rs; j -= k_rs) { // цикл по блокам из k_rs байт для вычисл. RS-кодов
					encode_RS(postdata_start, postrs_start, n_rs, k_rs); // RS-кодирование
					postdata_start += k_rs;				// на начало след. блока из k_rs байт
					postrs_start += (size_t)n_rs - k_rs;			// на начало след. блока кодов четности
				};
				if (j > 0) {					// остался фрагмент менее k_rs байт данных
					memset(data_buf, 0, 64);	// обнуляем кодируемый буфер
					memcpy(data_buf, postdata_start, j); // копируем кодируемые данные в начало буфера
					encode_RS(data_buf, postrs_start, n_rs, k_rs); // кодируем данные из буфера
				}
			};
		};
	}
}

/**
 * \brief Вычисление контрольной суммы для сегмента EPC и занесение ее в выходной буфер
 * \return Нет возвращаемого значения
 */
void enc_epc_crc()
{
	uint8_t* c;
	uint16_t l_epc, crc;

	c = w_params.out_buffer + enc_markers[1].pos_out;	// Адрес начала EPC в выходном буфере
	l_epc = (uint16_t)(enc_markers[1].len + 2);		// Длина сегмента вместе с маркером
	memcpy(imatrix, c, 4);
	memcpy(imatrix + 4, c + 6, l_epc - 6ULL);
	crc = CRC16(imatrix, l_epc - 2);
	*(uint16_t*)(c + 4) = _byteswap_ushort(crc);
}

/**
 * \brief Внетрикадровая перестановка выходного потока согласно Ammendment
 * \return Нет возвращаемого значения
 */
void interleave_outstream()
{
	uint8_t* c;
	uint32_t Nc, Nr, Len, i, j, k = 0;

	Len = enc_epc_dl - (h_length[0] + 1);		// Длина переставляемых данных: общая длина минус основной заголовок
	Nc = (uint32_t)ceil(sqrt((double)Len));	// Количество столбцов
	Nr = (uint32_t)ceil(((double)Len / Nc));			// Количество строк
	c = w_params.out_buffer + h_length[0] + 1;
	for (j = 0; j < Nc; j++) {
		for (i = 0; i < Nr; i++) {
			imatrix[i * Nc + j] = *c++;
			if (++k == Len)
				goto mcop;			// Все переставлено? переход к копированию
		}
	}
mcop:
	memcpy(w_params.out_buffer + h_length[0] + 1, imatrix, (size_t)Nc * Nr);
	amm_len = h_length[0] + 1 + Nc * Nr;
}

/**
 * \brief Кодер jpwl
 * \param  inbuf Ссылка на начало буфера, в котором находится кодовый поток jpeg200 часть 1
 * \param  outbuf  Ссылка на начало буфера, в который будет помещен кодовый поток jpeg200 часть 2 с внедренными в него средствами защиты от ошибок jpwl
 * \param  tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 * \param  pack_sens Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 254). В массиве pack_sens сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов, затем второго и т.д.
 * \param out_len  Адрес переменной в которую заносится длина выходного кодированного потока (количество байт, записанных в outbuf)
 */
errno_t w_encoder(uint8_t* inp_buf, uint8_t* out_buf, uint16_t* tile_packets,
	uint8_t* pack_sens, uint32_t* out_len)
{
	int exit_code;

	if (w_enc_init(inp_buf)) {
		return -6;		// если неверные значеня параметров - выход
	};
	exit_code = enc_w_markers_create(inp_buf, out_buf, tile_packets, pack_sens);
	if (exit_code) {
		return exit_code;
	};
	enc_data_copy(inp_buf, out_buf);
	enc_markers_copy(out_buf, tile_packets, pack_sens); // копирование маркеров в вых. буфер
	enc_epc_crc();					// Вычисление контрольной суммы для сегмента EPC
	enc_fill_epb(out_buf);			// заполнение блоков EPB кодами четности
	if (w_params.interleave_used) { // Используем Ammendment
		interleave_outstream();
		*out_len = amm_len;			// Длина при использовании Ammendment
	}
	else
		*out_len = enc_epc_dl;
	return 0;
}

/**
 * \brief  Yстановкa параметров кодера
 * \param  params Cсылка на структуру w_enc_params со значениями параметров кодера
 * \return Код завершения: 
 *  0 - все нормально
 * -1 - недостаточно места в буферах для тайлов
 * -2 - неверная структура кодового потока jpeg2000 часть 1
 * -3 - слишком большой заголовок, недостаточно одного EPB
 * -4 - недостаточно места в массиве для размещения всех маркеров
 * -5 - недостаточно места в массиве для интервалов чувствительности
 * -6 - недопустимая комбинация исходных параметров
 */
errno_t w_encoder_call(w_enc_params* params)
{
	int res;

	wcoder_mh_param = params->wcoder_mh;
	wcoder_th_param = params->wcoder_th;
	wcoder_data_param = params->wcoder_data;
	interleave_use = params->interleave_used;
	res = w_encoder(params->inp_buffer, params->out_buffer, params->tile_packets,
		params->packet_sense, &(params->wcoder_out_len));
	params->wcoder_mh_len = h_length[0] + 1; // Записываем длину основного заголовка
	return res;
}

/**
 * \brief  Установка значений параметров кодера jpwl по умолчанию
 * \param  params Cсылка на структуру jpwl_enc_params со значениями параметров кодера jpwl
 */
__declspec(dllexport)
void jpwl_enc_set_default_params(jpwl_enc_params* params)
{
	params->wcoder_mh = 1;		// предопределенная защита основного заголовка
	params->wcoder_th = 1;		// предопределенная защита заголовка тайла
	params->wcoder_data = 64;
	params->jpwl_enc_mode = 1;		// Использовать jpwl
	params->interleave_used = 0;	// Использовать Ammendment
}

/**
 * \brief  Инициализация значений параметров кодера jpwl, переданных из ПО ПИИ
 * \param  params Cсылка на структуру jpwl_enc_params со значениями параметров кодера jpwl
 */
__declspec(dllexport)
void jpwl_enc_init(jpwl_enc_params* params)
{
	w_params.wcoder_mh = params->wcoder_mh;
	w_params.wcoder_th = params->wcoder_th;
	w_params.wcoder_data = params->wcoder_data;
	w_params.interleave_used = params->interleave_used;
	w_params.jpwl_enc_mode = params->jpwl_enc_mode;
}

/**
 * \brief  Запуск кодера jpwl
 * \param  inp_buf Cсылка на входной буфер
 * \param  out_buf Cсылка на выходной буфер
 * \param  bParams Cсылка на структуру jpwl_enc_bParams с дополнительными данными для кодера
 * \param  bResults Cсылка на структуру jpwl_enc_bResults с дополнительными результатами кодера
 */
__declspec(dllexport)
errno_t jpwl_enc_run(uint8_t* inp_buf, uint8_t* out_buf,
	jpwl_enc_bParams* bParams, jpwl_enc_bResults* bResults)
{
	uint8_t* v;
	int res;
	addr_char ac;

	w_params.inp_buffer = inp_buf;
	w_params.out_buffer = out_buf;
	w_params.tile_packets = bParams->tile_packets;
	w_params.packet_sense = bParams->pack_sens;
	if (w_params.jpwl_enc_mode) {		// кодирование при использовании jpwl
		res = w_encoder_call(&w_params);
		if (res)
			return -1;
		bResults->wcoder_out_len = w_params.wcoder_out_len;
		bResults->wcoder_mh_len = w_params.wcoder_mh_len;
	}
	else {
		memcpy(out_buf, inp_buf, bParams->stream_len);
		bResults->wcoder_out_len = bParams->stream_len;
		v = mark_search(inp_buf, SOT_LOW, EOC_LOW, &ac); // поиск первого тайла
		if (v == NULL)
			return -2;
		bResults->wcoder_mh_len = (uint32_t)(v - inp_buf);	// длина основного заголовка
	};
	return 0;
}

__declspec(dllexport)
errno_t jpwl_init()
{
#ifdef RS_OPTIMIZED
	encode_RS = &rs_encode;
	decode_RS = &rs_decode;
	return rs_init_all();
#else
	encode_RS = &encode_rs;
	decode_RS = &decode_rs;
	generate_gf();
	return 0;
#endif // RS_OPTIMIZED
}
/**
 * \brief  Заключительная очистка библиотеки
 */
__declspec(dllexport)
void jpwl_destroy()
{
#ifdef RS_OPTIMIZED
	rs_destroy();
#endif // RS_OPTIMIZED
}

__declspec(dllexport)
void sens_create(unsigned char* input, unsigned short* tile_packets, unsigned char* pack_sens)
{
	unsigned char* g, * p, * v;
	int tile_count, j, k, p_no;
	int l;

	p_no = 0;
	v = NULL;
	g = input;
	for (tile_count = 0; (g = mark_search(g + 2, SOT_LOW, EOC_LOW, &v)) != NULL; tile_count++);
	p = input;
	for (j = 0; j < tile_count; j++) {			// цикл по тайлам
		p = mark_search(p, SOT_LOW, EOC_LOW, &v);
		for (k = 0; p != NULL; k++) {		// вычисляем кол-во пакетов в тайле по маркерам SOP
			p = mark_search(p + 2, SOP_LOW, j != tile_count - 1 ? SOT_LOW : EOC_LOW, &v);
		};
		p = v;
		tile_packets[j] = k - 1;
		if (tile_packets[j] == 0)
			tile_packets[j] = 31;		// заносим кол-во пакетов тайла в tiles_b[j]
		v = pack_sens + p_no;
		for (l = tile_packets[j] - 1; l >= 0; l--) // присваиваем чувствительности пакетов по методу OpenJpej
			*(v++) = l;
		p_no += tile_packets[j];				// наращиваем тек. номер пакета на кол-во записанных пакетов
	}
}
