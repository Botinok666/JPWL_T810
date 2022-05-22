
/** pa
 * \file jpwl_encoder.cpp
 * \brief Файл с текстами программ библиотеки кодера JPWL
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12
*/

#include "math.h"
#include "memory.h"
#include "stdio.h"
#include <string.h>
#include "jpwl_encoder.h"
#include "..\rs_crc_lib\rs_crc_decl.h"
#include "..\rs_crc_lib\rs_crc_import.h"
#include "jpwl_params.h"

#ifndef _TEST
#define _TEST
#endif

#define W_OPTIMIZED

#define MARKER_COUNT_CHECK if(mark_count==MAX_MARKERS) return(-1);
#define INTERV_COUNT_CHECK if(interv_count==MAX_INTERVALS) return(-4);



int_struct e_intervals[MAX_INTERVALS];	///< Буфер для интервалов чувствительности данных тайлов

unsigned char e_table[256] =	///< Таблица RS-кодов для каждого значения чувствительности
{ 37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,
38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,
43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,
45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,
48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,
51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,
53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,
56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,
64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,
80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,
85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,
96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,
112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,
128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128 };


/**
 * \brief Присутствуют ли в пакетах маркеры SOP
 * \details Возможные значения:
 * _true_ -присутствуют,
 * _false_ - не присутствуют.
 * Если маркеры SOP присутствуют, данные о чувствительности в ESD
 * формируются по пакетам, если нет - данные тайла рассматриваются
 * как один пакет
 */
_bool_ sop_enable;				///< Флаг присутствия маркеров SOP
_bool_ empty_stream;			///< Флаг пустого потока, состоящего только из основного заголовка
w_marker mark_mas[MAX_MARKERS]; ///< Массив маркеров jpwl
unsigned long h_length[MAX_TILES + 1]; ///< Массив длин заголовков - используется для cоздания интервала чувствительности заголовка в блоке ESD. Длина - смещение последнего байта отн. начала заголовка
unsigned long Psot_new[MAX_TILES]; ///< Массив обновленных значений длин Psot тайлов 
unsigned short mark_count;		///< Счетчик (количество) маркеров в массиве mark_mas 
unsigned short esd_count;		///< Счетчик (количество) маркеров ESD
unsigned short interv_count;	///< Счетчик (количество) записей об интервалах чувствительности в массиве e_intervals
unsigned short tile_count;		///< Счетчик тайлов
unsigned short pack_count;		///< Счетчик пакетов в данных о чувствительности
unsigned char* cur_sens;		///< Ссылка на текущее значение в массиве чувствительности  пакетов packet_sense
unsigned long DL_value;			///< Длина выходного кодового потока
unsigned long AllMarkers_len;	///< Длина всех созданных маркеров (требуется для вычисления позиции сегмента текущего маркера в выходном буфере)
unsigned short epb_count;		///< Количество EPB блоков в тайлах
w_enc_params par_val;	///< Структура с параметрами кодера jpwl
unsigned short encoder_dump_no; ///< Порядковый номер выводимого в файла дампа выходного буфера
unsigned long file_len;			///< Длина выводимого дампа выходного потока
unsigned char dump_path[50];	///< Массив для записи пути к папке выгрузки дампа выходного буфера
unsigned char* file_data;		///< Буфер для вывода дампа в файл
unsigned char* epc_point;		///< Адрес для записи карты EPB блоков в сегмент EPC
unsigned char imatrix[MAX_OUT_SIZE];	///< Массив для выполнения внутрикадрового чередования выходного потока по Ammendment	
unsigned long amm_len;			///< Длина выходного потока при применении Ammendment

#ifdef W_OPTIMIZED

/**
 * \brief Поиск заданного маркера в буфере
 * \param buf  Адрес начала входного буфера
 * \param marker  Значение второго байта искомого маркера
 * \param terminated_marker  Значение второго байта маркера-ограничителя, на котором поиск заканчивается
 * \param t_adr Побочный эффект: eсли маркер не найден, присваивает t_adr адрес найденного маркера-ограничителя
 * \return Cсылка на первий найденный маркер или NULL, если маркера нет
 */
unsigned char* mark_search(unsigned char* buf, unsigned char marker, unsigned char terminated_marker, addr_char* t_adr)
{
	unsigned char* b, * p;

	for (b = buf, p = NULL;; b++)
		if (*b == 0xff)
			if (*(b + 1) == terminated_marker) {
				*t_adr = b;
				break;
			}
			else if (*(b + 1) == marker) {
				p = b;
				break;
			};
	return(p);
}

/**
 * \brief  Инициализация переменных и массивов кодера и проверка корректности значений параметров кодера, полученных из ПО ПИИ
 * \param inbuf Ссылка на буфер с входным кодовым потоком
 * \return Возвращает код завершения: 0 - параметры корректны
 */
int w_enc_init(unsigned char* inbuf)
{
	unsigned char* p, * v;

	v = NULL;
	// обнуляем массив маркеров
	if (mark_count == 0)		// первое обнуление массива
		memset(mark_mas, 0, sizeof(w_marker) * MAX_MARKERS);
	else					// последующие обнуления - mark_count элементов
		memset(mark_mas, 0, sizeof(w_marker) * mark_count);
	mark_count = 0;			// обнуление счетчиков
	esd_count = 0;
	interv_count = 0;
	pack_count = 0;
	AllMarkers_len = 0;
	epb_count = 0;
	empty_stream = _false_;
	//	tile_count=0;
		// Определяем наличие в кодовом потоке маркеров SOP
	p = mark_search(inbuf, SOD_LOW, EOC_LOW, &v);
	if (p == NULL) {
		empty_stream = _true_;
		sop_enable = _false_;
		par_val.interleave_used = 0;
	}
	else if (*(p + 2) == 0xff && *(p + 3) == SOP_LOW)
		sop_enable = _true_;
	else
		sop_enable = _false_;
	return(0);
}

/**
 * \brief На основании параметра защиты prot_par формирует и возвращает значение Pepb для блока EPB согласно спецификации jpwl табл. А.6-А.8
 * \details Значения prot_par:
 * 0 - защита отсутствует
 * 1 - предопределенная защита - RS(80,25)
 * 16 - 16-битная контрольная сумма СКС-16
 * 32 - 32-битная контрольная сумма СКС-32
 * 37 - RS-код RS(37,32)
 * 38 - RS-код RS(38,32)
 * 40 - RS-код RS(40,32)
 * 43 - RS-код RS(43,32)
 * 45 - RS-код RS(45,32)
 * 48 - RS-код RS(48,32)
 * 51 - RS-код RS(51,32)
 * 53 - RS-код RS(53,32)
 * 56 - RS-код RS(56,32)
 * 64 - RS-код RS(64,32)
 * 75 - RS-код RS(75,32)
 * 80 - RS-код RS(80,32)
 * 85 - RS-код RS(85,32)
 * 96 - RS-код RS(96,32)
 * 112 - RS-код RS(112,32)
 * 128 - RS-код RS(128,32
 * \param prot_par Код варианта защиты
 * \return Сформированное значение Pepb
*/
unsigned long get_Pepb(unsigned char prot_par)
{
	unsigned long l;

	if (prot_par == 0)		// нет защиты
		l = 0xFFFFFFFF;
	else if (prot_par == 1)	// предопределенные RS-коды
		l = 0x00000000;
	else if (prot_par == 16)	// crc-16
		l = 0x10000000;
	else if (prot_par == 32)	// crc-32
		l = 0x10000001;
	else if (prot_par == 37)		// RS(37,32)
		l = 0x20002520;
	else if (prot_par == 38)		// RS(38,32)
		l = 0x20002620;
	else if (prot_par == 41)		// RS(40,32)
		l = 0x20002820;
	else if (prot_par == 43)		// RS(43,32)
		l = 0x20002B20;
	else if (prot_par == 45)		// RS(45,32)
		l = 0x20002D20;
	else if (prot_par == 48)		// RS(48,32)
		l = 0x20003020;
	else if (prot_par == 51)		// RS(51,32)
		l = 0x20003320;
	else if (prot_par == 53)		// RS(53,32)
		l = 0x20003520;
	else if (prot_par == 56)		// RS(56,32)
		l = 0x20003820;
	else if (prot_par == 64)		// RS(64,32)
		l = 0x20004020;
	else if (prot_par == 75)		// RS(75,32)
		l = 0x20004B20;
	else if (prot_par == 81)		// RS(80,32)
		l = 0x20005020;
	else if (prot_par == 85)		// RS(85,32)
		l = 0x20005520;
	else if (prot_par == 96)		// RS(96,32)
		l = 0x20006020;
	else if (prot_par == 112)	// RS(112,32)
		l = 0x20007020;
	else if (prot_par == 128)	// RS(128,32)
		l = 0x20008020;
	else if (prot_par == 144)	// RS(144,32)
		l = 0x20009020;
	else if (prot_par == 161)	// RS(1160,32)
		l = 0x2000A020;
	else if (prot_par == 176)	// RS(176,32)
		l = 0x2000B020;
	else if (prot_par == 192)	// RS(192,32)
		l = 0x2000C020;
	return(l);
}

/**
 * \brief  Создание маркеров jpwl в основном заголовке
 * \details Побочный эффект:
 * передвигает bb на начало первого тайла, расположенного непосредственно
 * за основным заголовком
 * \param bb  Cсылка на начало буфера, в котором находится кодовый поток jpeg2000 часть 1.
 * \return Код завершения:  0 - все нормально, -1 - недостаточно места в массиве для размещения всех маркеров, * -2 - не найдено ни одного тайла, * -3 - слишком длинный заголовок, мало одного EPB, * -4 - недостаточно места в массиве для интервалов чувствительности
*/
int mh_markers_create(addr_char* bb)
{
	unsigned char* p, * v, * buf;
	unsigned short l, d;
	double f;
	epb_ms* e;
	epc_ms* c;
	esd_ms* es;
	unsigned int l_rs;

	v = NULL;
	buf = *bb;
	p = mark_search(buf, SOT_LOW, EOC_LOW, &v);	// ищем маркер SOT, расположенный за MH 
	if (p == NULL)						// нет маркера SOT 
		if (!empty_stream)				// и поток не пустой - неверный кодовый поток
			return(-2);
		else {
			p = mark_search(buf, EOC_LOW, EMPTY_LOW, &v);	// Ищем EOC
			if (p == NULL)				// Нет EOC - неверный кодовый поток
				return(-2);
			DL_value = p - buf + 2;			// Длина входного кодового потока из осн. заголовка + маркер EOC
		};
	h_length[0] = (unsigned long)(p - buf) - 1;		// Запомнили смещение посл. байта заголовка отн. его начала
	if (MAX_MARKERS < (esd_use == _true_ ? 3 : 2))					// нет места для маркеров в MH
		return(-1);
	// формируем в l длину сегмента маркера SIZ
	v = (unsigned char*)&l; *v = *(buf + 5); *(v + 1) = *(buf + 4);
	l = l + 4;								// длина вместе с маркерами SOC и SIZ

	// ============ создаем маркер EPB
	mark_mas[0].id = EPB_MARKER;			// значение маркера
	mark_mas[0].pos_in = l;				// после SOC и сегмента SIZ
	mark_mas[0].pos_out = l;				// для первого маркера совпадает с pos_in
	//	mark_mas[0].posin_ready=_true_;		// позиция вх. буфера готова
	e = &mark_mas[0].m.epb;
	//	e->latest=_true_;					// последний в заголовке
	//	e->packed=_true_;					// упакованный
	mark_mas[0].tile_num = -1;					// основной заголовок
	e->index = 0;						// индекс=0
	e->hprot = wcoder_mh_param;		// параметр из ПО ПИИ
	e->k_pre = 64;					// предопределенное значение
	e->n_pre = 160;					// предопределенное значение
	e->pre_len = l + EPB_LN + 2;			// SOC, сегмент SIZ + заголовок EPB

	// вычисляем длину пост-данных (без чередования)
	d = (unsigned short)(p - buf) - l;			// длина осн. заголовка от окончания
	// сегмента SIZ до конца заголовка
// прибавляем длину маркера EPC (без чередования) вместе с маркером
	d += EPC_LN + 2;
	if (esd_use == _true_) // прибавляем длину маркера ESD из 1 байтового диапазона вмсте с маркером
		d += ESD_LN + 2 + ESDINT_LN;
	e->post_len = d;
	if (wcoder_mh_param == 1) {
		e->k_post = 64;
		e->n_post = 160;
	}
	else if (wcoder_mh_param >= 37) {
		e->k_post = 32;
		e->n_post = wcoder_mh_param;
	}
	else {
		e->k_post = 0;
		e->n_post = 0;
	};
	// вычисляем длину сегмента маркера для разных вариантов защиты
	f = (double)d;			// вещественное значение длины пост-данных
	if (wcoder_mh_param == 1 || wcoder_mh_param >= 37) //  RS-код
		l_rs =
		(unsigned short)(ceil(f / e->k_post)) * (e->n_post - e->k_post);
	else if (wcoder_mh_param == 16)				// CRC-16
		//		l_rs=(unsigned short)(ceil(f/crc16_blocksize))*2;
		l_rs = 2;
	else if (wcoder_mh_param == 32)				// CRC-32
		l_rs = 4;
	else										// нет защиты
		l_rs = 0;
	l_rs += EPB_LN + 96;			// +длина постоянной части + длина RS-кодов для пре данных
	if (l_rs > MAX_EPBSIZE)				// одного EPB мало
		return(-3);

	e->Lepb = (unsigned short)l_rs;	// Длина EPB без маркера
	mark_mas[0].len = e->Lepb;		// Длина EPB без маркера
	AllMarkers_len += l_rs + 2;			// длина сегмента + сам маркер
	//	mark_mas[0].len_ready=_true_;		// Длина готова
	h_length[0] += l_rs + 2;			// Увеличиваем длину заголовка на длину EPB с маркером
	e->Depb = 0xC0;				// последний в заголовке, упакованный, индекс=0
	e->LDPepb = e->pre_len + e->post_len;	// защищаемая длина пре-данных + пост-данных
	// формируем поле Pepb с описанием метода защиты данных табл. А.6-А.8
	e->Pepb = get_Pepb(wcoder_mh_param);
	//	mark_mas[0].params_ready=_true_;		// параметры готовы

		//============= Создаем маркер EPC без информативных методов
		// DL и контрольная сумма заполняются позднее
	mark_mas[1].id = EPC_MARKER;			// Идентификатор маркера
	mark_mas[1].pos_in = l;				// после SOC и сегмента SIZ 
	mark_mas[1].pos_out = l + AllMarkers_len;	// позиция в вых. буфере отличается от позиции во входном 
	// на длину ранее созданных сегментов
	mark_mas[1].len = EPC_LN;					// Длина сегмента без самого маркера
	//	mark_mas[1].len_ready=_true_;			// Длина готова
	//	mark_mas[1].posin_ready=_true_;		// позиция во вх. буфере готова
	mark_mas[1].tile_num = -1;				// основной заголовок
	c = &mark_mas[1].m.epc;
	//	c->epb_on=_true_;		// EPB используется
	if (esd_use == _true_) {
		//		c->esd_on=_true_;		// ESD используется
		c->Pepc = 0x50;		// Info-,EPB+, RED-, ESD+
	}
	else {
		//		c->esd_on=_false_;		// ESD не используется
		c->Pepc = 0x40;		// Info-,EPB+, RED-, ESD-
	};
	//	c->red_on=_false_;		// RED не используется
	//	c->info_on=_false_;	// Info методы не используются
	c->Lepc = EPC_LN;			// Длина сегмента без самого маркера
	AllMarkers_len += EPC_LN + 2;	// добавляем длину сегмента + сам маркер
	h_length[0] += EPC_LN + 2;		// Увеличиваем длину заголовка на длину EPC c маркером
	//	mark_mas[1].params_ready=_false_;		// параметры не готовы (DL неизвестно)

		//=========== Создаем маркер ESD с описанием чувствительности основного заголовка
		// в режиме байтового диапазона (1 диапазон от начала до конца осн. заголовка)
	if (esd_use == _true_) {
		mark_mas[2].id = ESD_MARKER;			// Идентификатор маркера
		mark_mas[2].pos_in = l;					// после SOC и сегмента SIZ 
		mark_mas[2].pos_out = l + AllMarkers_len;	// позиция в вых. буфере отличается от позиции во входном 
		// на длину ранее созданных сегментов
		mark_mas[2].len = ESD_LN + ESDINT_LN;		// Длина сегмента без самого маркера
		//		mark_mas[2].len_ready=_true_;				// Длина готова
		//		mark_mas[2].posin_ready=_true_;			// позиция во вх. буфере готова
		es = &mark_mas[2].m.esd;
		mark_mas[2].tile_num = -1;							// основной заголовок
		es->addrm = 1;							// байтогвый диапазон
		//		es->ad_size=4;							// на адрес байта - 4 байта
		//		es->senst=0;							// относительная чувствительность
		//		es->se_size=1;							// значение чувствительности - 1 байт
		es->Lesd = mark_mas[2].len;				// длина сегмента маркера без самого маркера
		AllMarkers_len += mark_mas[2].len + 2;		// добавляем длину сегмента + сам маркер
		h_length[0] += es->Lesd + 2;				// Увеличиваем длину заголовка на длину ESD с маркером
		es->Cesd = esd_count++;					// компонент ESD
		es->Pesd = 0x42;					// параметры ESD: байтовый диапазон, относительная чувствительность
		// 1 байт на чувствительность, 4 байта на адрес байта, неусредненная 
		// чувствительность
//		mark_mas[2].params_ready=_true_;	// парметры готовы
	};
	mark_count += (esd_use == _true_ ? 3 : 2);// увеличили счетчик маркеров
	*bb = p;							// устанавливаем буфер на первый тайл

	return(0);
}

/**
 * \brief  Создание маркеров jpwl в  заголовке тайла
 * \details Побочный эффект:
 * передвигает bb на начало следующего тайла или присваивает ему NULL,
 * если следующего тайла нет
 * \param bb Ссылка на тайл кодового потока jpeg2000 часть 1
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока  (маркера SOC) тайла
 * \param packet_sense Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 255). В массиве packet_sense сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов, затем второго и т.д.
 * \param allbuf Ссылка на начало всего кодового потока, т.е. на начало основного заголовка
 * \return Код завершения: 0 - все нормально, -1 - недостаточно места в массиве для размещения всех маркеров, -2 - ошибки в кодовом потоке jpeg2000 часть1, -3 - слишком большой заголовок, недостаточно одного EPB,-4 - недостаточно места в массиве для интервалов чувствительности
*/
int th_markers_create(addr_char* bb, unsigned short* tile_packets, unsigned char* packet_sense,
	unsigned char* allbuf)
{
	unsigned char* p, * v, * g, * p_start, * buf_new, epb_ind, data_p, rs, * buf, ses;
	unsigned short i_s, i_k;
	unsigned long l;
	unsigned int l_rs;
	int i, l_es, d, intrv_max, intrv_ln, AllTileEpb_ln;
	double dd, f;
	epb_ms* e;
	esd_ms* es;

	if (empty_stream) {		// Если поток не содержит тайлов
		*bb = NULL;
		return (0);
	};
	AllTileEpb_ln = 0;
	v = NULL;
	buf = *bb;
	p = mark_search(buf, SOD_LOW, EOC_LOW, &v);	// ищем  маркер SOD - конец заголовка тайла 
	if (p == NULL)								// нет маркера SOD - ошибочный кодовый поток
		return(-2);
	h_length[tile_count + 1] = (unsigned long)(p - buf) + 1;		// Запомнили смещение последнего байта заголовка тайла
	// относительно начала тайла
	p += 2;								// устанавливаем p на начало первого пакета данных тайла
	p_start = p;							// p_start - начало первого пакета в тайле			
	g = mark_search(buf + 2, SOT_LOW, EOC_LOW, &v);	// ищем следующий маркер  SOT 
	buf_new = g;							// ссылка на следующий тайл или NULL, усли он отсутствует
	if (g == NULL) {						// нет маркера SOT - найден маркер конца EOC (т.е. тайл явл. последним)
		g = v + 2;							// устанавливаем g на адрес первого байта после конца данных последнего 
		// тайла
		DL_value = (unsigned long)(g - allbuf);	// длина входного кодового потока
	};
	i_s = interv_count;					// индекс начального интервала данных о чувствительности пакетов тайла
	// в массиве e_intervals
	i_k = 0;								// начальное хначение кол-ва интервалов чувствительности
	// 	if(wcoder_data_param!=0 || (esd_use==_true_ && esd_mode==ESD_BYTE_RANGE)) { // защита используется или используется ESD
																				  // в режиме байтового диапазона, 
																				  // составляем карту интервалов чувствительности
	if (sop_enable == _true_ && wcoder_data_param == 1) { // присутствует маркер заголовка пакетов SOP и 
		// используется UEP, можно составлять попакетную
		// карту интервалов чувствительности
		for (i = 0, cur_sens = packet_sense + pack_count; i < tile_packets[tile_count]; i++, cur_sens++) {
			if (i == 0) {	// устанавливаем начало первого интервала как смещение отн.
				// начала заголовка тайла во входном буфере						
				e_intervals[interv_count].start = (unsigned long)(p_start - buf);
				ses = e_intervals[interv_count].sens = *cur_sens;    // значение чувствительности
				rs = e_intervals[interv_count].code = e_table[*cur_sens]; // присваиваем RS-код для этого интервала
				// вычисляем максимально возможную длину интервала,
				// который может быть защищен одним EPB
				intrv_max = floor(((double)(MAX_EPBSIZE - EPB_LN - PRE_RSCODE_SIZE)) / (e_intervals[interv_count].code - 32)) * 32;
				i_k++;								// наращиваем кол-во интервалов
			}
			else if (e_table[*cur_sens] != e_intervals[interv_count].code || // в очередном пакете другой RS-код!
				i == tile_packets[tile_count] - 1) {			// или последний пакет тайла
				// предварительно проверим не нужно ли разбить интервал на 2 из-за большой длины
				if (i != tile_packets[tile_count] - 1) // вычисляем длину интервала
					// при смене RS-кода между пакетами
					intrv_ln = (unsigned long)(p - buf) - e_intervals[interv_count].start;
				else							// для случая последнего пакета
					intrv_ln = (unsigned long)(g - buf) - e_intervals[interv_count].start;
				for (; intrv_ln >= intrv_max; intrv_ln -= intrv_max) {	// дробим интервал на несколько
					// длина интервала - intrv_max позволяет защитить его 
					// одним EPB
					e_intervals[interv_count].end = e_intervals[interv_count].start + intrv_max - 1;
					if (intrv_ln > intrv_max) { // не все вошло в предыд. интервал					
						// создаем следующий интервал  со след. байта
						i_k++;
						INTERV_COUNT_CHECK
							e_intervals[++interv_count].start = e_intervals[interv_count - 1].end + 1;
						e_intervals[interv_count].sens = ses;	// чувствительность такая же
						e_intervals[interv_count].code = rs;	// RS-код такой же
					}
				};
				if (intrv_ln != 0)				// Завершаем последний дробленный интервал
					e_intervals[interv_count].end = e_intervals[interv_count].start + intrv_ln - 1;
				if (i != tile_packets[tile_count] - 1) { // при смене RS-кода между пакетами создаем след. интервал
					i_k++;								// наращиваем кол-во интервалов
					INTERV_COUNT_CHECK
						e_intervals[++interv_count].start = (unsigned long)(p - buf); //начало нового интервала
					ses = e_intervals[interv_count].sens = *cur_sens;
					rs = e_intervals[interv_count].code = e_table[*cur_sens]; // RS-код нового интервала
				}
			};
			if (i != tile_packets[tile_count] - 1) { // не последний пакет тайла
				p = mark_search(p + 2, SOP_LOW, EOC_LOW, &v); // ищем следующий пакет
				if (p == NULL)						// нет следующего пакета - ошибка
					return(-2);
			}
		};
		INTERV_COUNT_CHECK
			interv_count++;
	}
	else {						// чувствительность задается одним интервалом от начала первого пакета
		// до конца последнего пакета
//			if(wcoder_data_param==1) { // задан режим UEP
							// вычисляем среднее арифметическое чувствительности всех пакетов
		for (i = 0, dd = 0, cur_sens = packet_sense + pack_count; i < tile_packets[tile_count]; i++, cur_sens++)
			dd += (double)*cur_sens;
		dd /= tile_packets[tile_count];

		//			}
		//			else if(wcoder_data_param>=37)
		//				dd=wcoder_data_param;
		ses = (unsigned char)dd;						// чувствительность интервала

		i_k++;
		e_intervals[interv_count].start = (unsigned long)(p_start - buf);	// начало интервала - начало тайла

		if (wcoder_data_param == 1 || wcoder_data_param >= 37) {
			e_intervals[interv_count].sens = ses;		// чувствительность интервала
			rs = e_intervals[interv_count].code =		// присваиваем RS-код для этого интервала
				wcoder_data_param == 1 ? e_table[(unsigned char)dd] : wcoder_data_param;
			// и вычисляем максимально возможную длину интервала
			// для одного EPB	
			intrv_max = floor((double)(MAX_EPBSIZE - EPB_LN - PRE_RSCODE_SIZE) / (e_intervals[interv_count].code - 32)) * 32;
			intrv_ln = (int)(g - 1 - p_start);			// фактическая длина интервала
			for (; intrv_ln >= intrv_max; intrv_ln -= intrv_max) { // дробим  интервал на несколько, 
				// каждый из которых
				// целиком может быть защищен одним EPB
				e_intervals[interv_count].end = e_intervals[interv_count].start + intrv_max - 1;
				// начало следующего интервала - через 1 байт
				// после конца предыдущего
				if (intrv_ln > intrv_max) {			// не все байты вошли в созданный интервал
					// создаем начало следующего интервала
					i_k++;
					INTERV_COUNT_CHECK
						e_intervals[++interv_count].start = e_intervals[interv_count - 1].end + 1;

					e_intervals[interv_count].code = rs;
					e_intervals[interv_count].sens = ses;		// чувствительность интервала

				}
			};
			if (intrv_ln > 0)				// заканчиваем последний интервал
				e_intervals[interv_count].end = e_intervals[interv_count].start + intrv_ln;
		}
		else {
			i_k = 1;
			e_intervals[interv_count].sens = (unsigned char)dd;			// чувствительность интервала
			e_intervals[interv_count].end = (unsigned long)(g - 1 - buf); // конец  последнего интервала -
			// последний байт данных текущего тайла или
			// последний байт маркера EOC последнего тайла
		};
		INTERV_COUNT_CHECK
			interv_count++;
	};
	pack_count += tile_packets[tile_count];			// прибавляем в pack_count кол-во обработанных значений о чувствительности
	// пакетов тайла, т.е. переводим счетчик на первое значение след. тайла
//	};
	// Вычисляем длину ESD
	if (esd_mode == ESD_BYTE_RANGE)			// в режиме байтового диапазона 1 ESD и на заголовок и на данные
		l_es = esd_use == _true_ ? ESD_LN + 2 + ESDINT_LN * (i_k + 1) : 0; // 1 доп. интервал на заголовок тайла
	else									// в пакетном режиме 2 ESD: один (байтовый диапазон) на заголовок тайла,
											// второй (пакетный режим) - на данные
		l_es = esd_use == _true_ ? ESD_LN + 2 + ESDINT_LN + ESD_LN + 2 + tile_packets[tile_count] : 0;

	//================= создаем блоки EPB в заголовке тайла
	// 1 блок для защиты заголовка 
	// + по 1 блоку на каждый интервал чувствительности

	// создаем EPB для защиты заголовка
	epb_count++;								// Подсчет количества EPB блоков для реализации Ammendment
	epb_ind = 0;									// индекс текущего EPB в заголовке
	MARKER_COUNT_CHECK
		mark_mas[mark_count].id = EPB_MARKER;			// значение маркера
	l = (unsigned long)(buf - allbuf) + SOT_LN + 2;		// смещение относительно начала всего кодового потока
	// куда будет вставляться маркер EPB для защиты заголовка тайлша
	// туда же (т.е. после него) будут вставляться EPBдля защиты данных
	// и ESD
	mark_mas[mark_count].pos_in = l;				// после сегмента SOT
	mark_mas[mark_count].pos_out = l + AllMarkers_len;	// поз. вых буфера = поз. входн. буфера + длина всех добавленных ранее сегментов
	//	mark_mas[mark_count].posin_ready=_true_;		// позиция вх. буфера готова
	e = &mark_mas[mark_count].m.epb;			// ссылка на EPB в Union и инкремент кол-ва созданных маркеров
	//	e->latest=wcoder_data_param==0?_true_:_false_;	// последний в заголовке, если данные не защищены
													// и не последний, если есть какая-либо защита данных
	//	e->packed=_true_;					// упакованный
	mark_mas[mark_count].tile_num = tile_count;			// индекс текущего тайла
	e->index = 0;						// индекс=0
	e->hprot = wcoder_th_param;		// параметр из ПО ПИИ
	e->k_pre = 25;					// предопределенное значение
	e->n_pre = 80;					// предопределенное значение
	e->pre_len = SOT_LN + 2 + EPB_LN + 2;	// сегмент SOT + маркер SOT + заголовок EPB+маркер EPB

	// вычисляем длину пост-данных 
	d = (unsigned long)(p_start - buf) - SOT_LN - 2; // длина  заголовка от окончания сегмента SOT до конца заголовка
	if (esd_use == _true_) // прибавляем длину маркера ESD если он используется
		d += l_es;
	e->post_len = d;
	if (wcoder_th_param == 1) {
		e->k_post = 25;
		e->n_post = 80;
	}
	else if (wcoder_th_param >= 37) {
		e->k_post = 32;
		e->n_post = wcoder_th_param;
	}
	else {
		e->k_post = 0;
		e->n_post = 0;
	};
	// вычисляем длину сегмента маркера для разных вариантов защиты
	f = (double)d;			// вещественное значение длины пост-данных
	if (wcoder_th_param == 1 || wcoder_th_param >= 37) //  RS-код
		l_rs =
		(unsigned short)(ceil(f / e->k_post)) * (e->n_post - e->k_post);
	else if (wcoder_th_param == 16)				// CRC-16
		//		l_rs=(unsigned short)(ceil(f/crc16_blocksize))*2;
		l_rs = 2;
	else if (wcoder_th_param == 32)				// CRC-32
		l_rs = 4;
	else										// нет защиты
		l_rs = 0;

	l_rs += EPB_LN + 55;						// +длина постоянной части + длина RS-кодов для
	// пре данных
	if (l_rs > MAX_EPBSIZE)				// одного EPB мало
		return(-3);
	e->Lepb = (unsigned short)l_rs; // Длина EPB без маркера
	AllMarkers_len += l_rs + 2;		// наращиваем длину созданных сегментов
	mark_mas[mark_count++].len = e->Lepb; // Длина EPB без маркера
	//	mark_mas[mark_count].len_ready=_true_; // Длина готова
	AllTileEpb_ln = e->Lepb + 2;	// длина всего EPB вместе с маркером
	if (wcoder_data_param == 0)	// нет защиты данных, больше EPB не будет
		e->Depb = 0xC0;			// последний в заголовке, упакованный, индекс=0
	else
		e->Depb = 0x80;			// не последний в заголовке, упакованный, индекс=0
	e->LDPepb = e->pre_len + e->post_len +	// защищаемая длина пре-данных + пост-данных
		// + длина всех данных тайла, если пост-данные заголовка и данные тайла не защищаются
		((wcoder_th_param == 0 && wcoder_data_param == 0) ? e_intervals[interv_count - 1].end - e_intervals[interv_count - 1].start + 1 : 0);
	// формируем поле Pepb с описанием метода защиты данных табл. А.6-А.8
	e->Pepb = get_Pepb(wcoder_th_param);
	//	mark_mas[mark_count].params_ready=_true_; // параметры готовы

		// создаем переменное количество блоков для защиты данных 
		// по 1 блоку на каждый интервал чувствительности
	if (wcoder_data_param != 0)
		for (i = 0; i < i_k; i++) {
			MARKER_COUNT_CHECK
				epb_count++;								// Подсчет количества EPB блоков для реализации Ammendment
			mark_mas[mark_count].id = EPB_MARKER;			// значение маркера
			mark_mas[mark_count].pos_in = l;				// после сегмента SOT
			mark_mas[mark_count].pos_out = l + AllMarkers_len;	// позиция в вых. буфере 
			//			mark_mas[mark_count].posin_ready=_true_;		// позиция вх. буфера готова
			e = &mark_mas[mark_count].m.epb;			// ссылка на EPB в Union и инкремент кол-ва созданных маркеров
			//			e->latest=(i==i_k-1?_true_:_false_);	// последний в заголовке, если обрабатывается последний интервал
															// и не последний, если не последний интервал
			//			e->packed=_true_;					// упакованный
			mark_mas[mark_count].tile_num = tile_count;			// индекс текущего тайла
			e->index = ++epb_ind;				// индекс EPB в заголовке
			if (wcoder_data_param != 1)		// не используется UEP
				e->hprot = wcoder_data_param;		// параметр из ПО ПИИ: предопределенный RS-код, CRC-16 или CRC-32
			else
				e->hprot = e_intervals[i_s + i].code;
			e->k_pre = 13;					// предопределенное значение
			e->n_pre = 40;					// предопределенное значение
			e->pre_len = EPB_LN + 2;			// заголовок EPB + маркер EPB

			// вычисляем длину пост-данных 
			e->post_len = d = (int)(e_intervals[i_s + i].end - e_intervals[i_s + i].start + 1); // длина  интервала чувствительности		
			// параметры RS-кодов для пост-данных
			if (wcoder_data_param == 1) {
				e->k_post = 32;
				e->n_post = e_intervals[i_s + i].code;
			}
			else if (wcoder_data_param >= 37) {
				e->k_post = 32;
				e->n_post = wcoder_data_param;
			}
			else {
				e->k_post = 0;
				e->n_post = 0;
			};
			// вычисляем длину сегмента маркера для разных вариантов защиты
			f = (double)d;			// вещественное значение длины пост-данных
			if (wcoder_data_param == 1 || wcoder_data_param >= 37) //  RS-код
				l_rs = (unsigned short)(ceil(f / e->k_post)) * (e->n_post - e->k_post);
			else if (wcoder_data_param == 16)				// CRC-16
				//				l_rs=(unsigned short)(ceil(f/crc16_blocksize))*2;
				l_rs = 2;
			else if (wcoder_data_param == 32)				// CRC-32
				l_rs = 4;
			else										// нет защиты
				l_rs = 0;
			l_rs += EPB_LN + 27;			// +длина постоянной части + длина RS-кодов для пре данных
			if (l_rs > MAX_EPBSIZE)				// одного EPB мало
				return(-3);
			e->Lepb = (unsigned short)l_rs; 		// Длина EPB без маркера
			AllMarkers_len += l_rs + 2;				// наращиваем длину созданных сегментов
			mark_mas[mark_count++].len = e->Lepb; // Длина EPB без маркера
			//			mark_mas[mark_count].len_ready=_true_; // Длина готова
			AllTileEpb_ln += e->Lepb + 2;		// вычисляем общую длину всех EPB тайла
			// вместе с их маркерами. Это нужно для создания правильных
			// адресов интервалов чувствительности массива e_intervals
			// относительно начала тайла
			if (i == i_k - 1)			// это последний интервал, больше EPB не будет
				e->Depb = 0xC0 | (unsigned char)(e->index % 64); // последний в заголовке, упакованный, индекс=e->index
			else
				e->Depb = 0x80 | (unsigned char)(e->index % 64);	// не последний в заголовке, упакованный, индекс=e->index
			e->LDPepb = e->pre_len + e->post_len;	// защищаемая длина пре-данных + пост-данных
			// формируем поле Pepb с описанием метода защиты данных табл. А.6-А.8
			data_p = wcoder_data_param;
			if (data_p == 1)
				data_p = e->n_post;
			e->Pepb = get_Pepb(data_p);
			//			mark_mas[mark_count++].params_ready=_true_; // параметры готовы
		};

	//================ создаем блоки ESD =====================
	if (esd_use == _true_) {
		// Создаем сегмент маркера ESD
		MARKER_COUNT_CHECK
			mark_mas[mark_count].id = ESD_MARKER;			// значение маркера
		mark_mas[mark_count].pos_in = l;				// после сегмента SOT за всеми EPB
		mark_mas[mark_count].pos_out = l + AllMarkers_len;	// позиция вых. буфера
		//		mark_mas[mark_count].posin_ready=_true_;		// позиция вх. буфера готова
		es = &mark_mas[mark_count].m.esd;			// ссылка на ESD в Union и инкремент кол-ва созданных маркеров
		es->addrm = 1;								// байтовый диапазон
		//		es->ad_size=4;								// 4 байта на адрес в байтовом диапазоне
		//		es->senst=0;								// относительная чувствительность
		//		es->se_size=1;								// значение чувствительности - 1 байт
		mark_mas[mark_count].tile_num = tile_count;						// индекс тайла
		if (esd_mode == ESD_BYTE_RANGE) { // в этом режиме создаваемый ESD описывает чувствительность всего тайла
			es->interv_start = i_s;					// нач. элемент массива e_intervals для этого ESD
			es->interv_cnt = i_k;						// кол-во элементов в массиве e_intervals для этого ESD
			es->Lesd = l_es - 2;						// длина ESD без маркера
		}
		else {		// в режиме ESD_PACKETS создаваемый ESD описывает чувствительность только заголовка тайла
			es->interv_start = 0;						// нач. элемент массива e_intervals для этого ESD
			es->interv_cnt = 0;							// кол-во элементов в массиве e_intervals для этого ESD
			es->Lesd = ESD_LN + ESDINT_LN;					// длина ESD без маркера
		};
		mark_mas[mark_count++].len = es->Lesd;			// длина ESD без маркера
		AllMarkers_len += es->Lesd + 2;					// наращиваем длину созданных сегментов
		//		mark_mas[mark_count].len_ready=_true_;		// Длина готова

		es->Cesd = esd_count++;					// индекс компонента ESD
		es->Pesd = 0x42;						// байтовый диапазон, относит. чувствительность, 1 байт на значение 
		// чувствительности, 4 байта адрес, не усредненные значения
//		mark_mas[mark_count].params_ready=_true_; // параметры готовы

		if (esd_mode == ESD_PACKETS) {			// в этом режиме нужен дополнительный ESD для чувствительности пакетов
			MARKER_COUNT_CHECK
				mark_mas[mark_count].id = ESD_MARKER;			// значение маркера
			mark_mas[mark_count].pos_in = l;				// после сегмента SOT за всеми EPB
			mark_mas[mark_count].pos_out = AllMarkers_len + l;	// позиция в вых. буфере
			//			mark_mas[mark_count].posin_ready=_true_;		// позиция вх. буфера готова
			es = &mark_mas[mark_count].m.esd;			// ссылка на ESD в Union и инкремент кол-ва созданных маркеров
			es->addrm = 0;								// пакетный режим
			//			es->ad_size=0;								// адрес не используется
			//			es->senst=0;								// относительная чувствительность
			//			es->se_size=1;								// значение чувствительности - 1 байт
			mark_mas[mark_count].tile_num = tile_count;						// индекс тайла
			es->interv_start = i_s;					// нач. элемент массива e_intervals для этого ESD
			es->interv_cnt = i_k;						// кол-во элементов в массиве e_intervals для этого ESD
			es->Lesd = ESD_LN + tile_packets[tile_count]; // длина ESD без маркера
			mark_mas[mark_count++].len = es->Lesd;		// длина ESD без маркера
			AllMarkers_len += es->Lesd + 2;					// наращиваем длину созданных сегментов
			//			mark_mas[mark_count].len_ready=_true_;	// Длина готова
			es->Cesd = esd_count++;					// индекс компонента ESD
			es->Pesd = 0x00;					// пакетный режим, относит. чувствительность, 1 байт на значение 
			// чувствительности, адрес не используется, не усредненные значения
//			mark_mas[mark_count++].params_ready=_true_; // параметры готовы

		};
	};
	// Увеличиваем адреса начала и конца интервалов чувствительности в массиве e_intervals на AllTileEpb_ln+l_es
	AllTileEpb_ln += l_es;			// учитываем длину сегментов ESD
	for (i = 0; i < i_k; i++) {
		e_intervals[i + i_s].start += AllTileEpb_ln;
		e_intervals[i + i_s].end += AllTileEpb_ln;
	};
	// Увеличиваем длину заголовка на ту же величину (все EPB + все ESD)
	h_length[tile_count + 1] += AllTileEpb_ln;
	// Корректируем длину тайла в сегменте SOT
	v = (unsigned char*)&l;				// адрес младшего байта l
	p = *bb;								// адрес маркера SOT
	p += 9;								// адрес содержимого младшего байта длины в SOT
	*v++ = *p--;							// копирование 4 байтов длины в переменную l
	*v++ = *p--;
	*v++ = *p--;
	*v = *p;
	l += AllTileEpb_ln;					// увеличиваем l на сумму длин внедряемых данных
	Psot_new[tile_count] = l;				// новое значение Psot, которое нужно будет записать
	// в сегмент маркера SOT

	*bb = buf_new;				// переводим указатель буфкра на след. тайл
	return(0);
}

/**
 * \brief Создание маркеров jpwl в массиве mark_mas
 * \details Вызывает функции создания маркеров в основном заголовке и заголовках тайлов
 * \param inbuf Ссылка на начало буфера, в котором находится кодовый поток jpeg200 часть 1
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 * \param packet_sense Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 255). В массиве packet_sense сначала идут данные о пакетах первого по порядку тайла в порядке 	расположения пакетов, затем второго и т.д.
 * \return Код завершения: 0 - все нормально, -1 - недостаточно места в буферах для тайлов, -2 - неверная структура кодового потока jpeg2000 часть 1, -3 - слишком большой заголовок, недостаточно одного EPB, -4 - недостаточно места в массиве для размещения всех маркеров, -5 - недостаточно места в массиве для интервалов чувствительности
 *
 */
int w_markers_create(unsigned char* inbuf, unsigned char* outbuf, unsigned short* tile_packets,
	unsigned char* packet_sense)
{
	unsigned char* p;
	int exit_code, i;
	unsigned long epc_plus_size, epb0_plus_size, l_rs;
	double f;

	p = inbuf;
	if (exit_code = mh_markers_create(&p) != 0) 	// создаем маркеры в основном заголовке
		switch (exit_code)
		{
		case -1: return (-4);
		case -2: return (-2);
		case -3: return (-3);
		case -4: return (-5);
		};
	// цикл, перебирающий тайлы
	for (tile_count = pack_count = 0, cur_sens = packet_sense; p != NULL; tile_count++) {
		if (exit_code = th_markers_create(&p, tile_packets, packet_sense, inbuf) != 0) // создаем маркеры в заголовке тайла
			switch (exit_code)
			{
			case -1: return (-4);
			case -2: return (-2);
			case -3: return (-3);
			case -4: return (-5);
			};
		if (tile_count == (MAX_TILES - 1) && p != NULL)
			return(-1);
	};
	// Здесь в случае использования внутрикадрового интерлейсинга отводится 
	//  место под карту EPB в маркере EPC и изменяются размеры маркеров EPC и EPB основного заголовка
	epc_plus_size = par_val.interleave_used ? 6 + 10 * epb_count : 0;		// Увеличение размера EPC при использовании Ammendment
	epb0_plus_size = 0;				// Увеличение размера первого EPB при использовании Ammendment
	if (par_val.interleave_used) {	// Коррекция длин и позиций маркеров при использовании Ammendment
		mark_mas[1].len += epc_plus_size;	// Коррекция длины EPC
		mark_mas[1].m.epc.Lepc = mark_mas[1].len;
		mark_mas[1].m.epc.Pepc |= 0x80;	// Установка в EPC признака использования информативных методов
		mark_mas[0].m.epb.LDPepb += epc_plus_size;	// Увеличиваем длину защищаемых данных для первого EPB
		mark_mas[0].len = mark_mas[0].m.epb.LDPepb;
		mark_mas[0].m.epb.post_len += epc_plus_size;	// Увеличиваем длину пост-данных для первого EPB
		// вычисляем длину сегмента первого EPB для разных вариантов защиты
		f = (double)mark_mas[0].m.epb.post_len;			// вещественное значение длины пост-данных
		if (wcoder_mh_param == 1 || wcoder_mh_param >= 37) //  RS-код
			l_rs =
			(unsigned short)(ceil(f / mark_mas[0].m.epb.k_post)) * (mark_mas[0].m.epb.n_post - mark_mas[0].m.epb.k_post);
		else if (wcoder_mh_param == 16)				// CRC-16
			l_rs = 2;
		else if (wcoder_mh_param == 32)				// CRC-32
			l_rs = 4;
		else										// нет защиты
			l_rs = 0;
		l_rs += EPB_LN + 96;			// +длина постоянной части + длина RS-кодов для пре данных

		epb0_plus_size = l_rs - mark_mas[0].m.epb.Lepb;	// вычисляем добавку к длине сегмента первого EPB при использовании Ammendment
		mark_mas[0].m.epb.Lepb = mark_mas[0].len = l_rs;	// Обновляем длину сегмента первого EPB
		mark_mas[1].pos_out += epb0_plus_size;			// Корректируем позицию EPC в вых. буфере на величину увеличения первого EPB
		h_length[0] += epc_plus_size + epb0_plus_size;		// Коррекция длины основного заголовка
		for (i = 2; i < mark_count; i++) {			// Коррекция позиции в выходном буфере всех маркеров после EPC на величину увеличения первого EPB и EPC
			mark_mas[i].pos_out += epb0_plus_size + epc_plus_size;
			/*			if(mark_mas[i].id==EPB_MARKER)
							mark_mas[i].pos_out+=epb0_plus_size+epc_plus_size;
						if(mark_mas[i].id==ESD_MARKER)
							mark_mas[i].pos_out+=epb0_plus_size+epc_plus_size;
			*/
		};
	};
	DL_value += AllMarkers_len + epc_plus_size + epb0_plus_size;	// длина выходного кодового потока = длина входного + длина добавленных
	// сегментов
	mark_mas[1].m.epc.DL = DL_value;	// заносим DL в EPC

	return(0);
}

/*
// привязка маркеров jpwl к позиции в выходном буфере,
// вычисление длины выходного кодового потока и
// коррекция длин тайлов в сегментах маркера SOT.
// К длине SOT добаляется длина всех данных jpwl, внедряемых в заголовок тайла.
// Коррекция выполняется во входном буфере.
// Входные параметры:
// inbuf - ссылка на входной буфер.
void locate_markers(unsigned char *inbuf)
{
	int i, t_no;
	unsigned long m_len, tplus_len;
	unsigned char *p;
	unsigned char *v;
	unsigned long d;

	t_no=-1;						// начальный индекс  тайла - основной заголовок
	m_len=0;						// обнуляем суммарный размер сегментов jpwl
	p=inbuf;						// ссылка на входной буфер
	tplus_len=0;					// нач. значение длины основного заголовка
	for(i=0;i<mark_count;i++) {		// цикл по таблице маркеров
		if(t_no!=mark_mas[i].tile_num)  // встретился первый маркер из нового заголовка
			if(mark_mas[i].tile_num==0) {  // встретился маркер самого первого тайла
				tplus_len=mark_mas[i].len+2;	// подсчет суммы длин данных, внедряемых в заголовок
				t_no=0;					// индекс текущего тайла
			}
			else {						// встретили не первый тайл - нужно корректировать длину в
										// SOT предыдущего тайла на подсчитанную длину внедренных данных
				p=mark_search(p,SOT_LOW,EOC_LOW,&v);	// ищем маркер SOT (v не используется)
				v=(unsigned char *)&d;				// адрес младшего байта d
				p+=9;								// адрес содержимого младшего байта длины в SOT
				*v++=*p--;							// копирование 4 байтов длины в переменную d
				*v++=*p--;
				*v++=*p--;
				*v=*p;
				d+=tplus_len;						// увеличиваем d на сумму длин внедряемых данных
				*p++=*v--;							// копирование 4 байтов длины из переменной d в SOT
				*p++=*v--;							// в прямом порядке байт (big endian)
				*p++=*v--;
				*p=*v;
				tplus_len=mark_mas[i].len+2;		// начинаем суммирование длин для нового тайла
				t_no=mark_mas[i].tile_num;			// индекс текущего тайла
			}
		else
			tplus_len+=mark_mas[i].len+2;			// добавляем длину очередного маркера из этого же заголовка
		mark_mas[i].pos_out=mark_mas[i].pos_in+m_len;	// позиция в выходном буфере
									// смещена от позиции во входном буфере на сумму длин
									// всех предшествующих сегментов + маркеры
		m_len+=mark_mas[i].len+2;	// добавляем длину текущего скгмента + маркер
//		mark_mas[i].posout_ready=_true_;	// позиция в вых. буфере готова
	};
	p=mark_search(p,SOT_LOW,EOC_LOW,&v);	// ищем маркер SOT (v не используется) последнего тайла
	v=(unsigned char *)&d;				// адрес младшего байта d
	p+=9;								// адрес содержимого младшего байта длины в SOT
	*v++=*p--;							// копирование 4 байтов длины в переменную d
	*v++=*p--;
	*v++=*p--;
	*v=*p;
	d+=tplus_len;						// увеличиваем d на сумму длин внедряемых данных
	*p++=*v--;							// копирование 4 байтов длины из переменной d в SOT
	*p++=*v--;							// в прямом порядке байт (big endian)
	*p++=*v--;
	*p=*v;

	DL_value+=m_len;			// наращиваем DL (для EPC)  на длину маркера
}
*/

//
 // brief  Завершение установки параметров маркеров jpwl
 // details Установка длины кодового потока DL в маркере EPC и расчет контрольной суммы EPC.
 // Здесь же нужно составить карту EPB при использовании внутрикадрового
 // чередования
 //
/* void finalize_marker_params() {
	unsigned char epc_image[EPC_LN];

	mark_mas[1].m.epc.DL=DL_value;	// заносим DL в EPC
	epc_image[0]=0xff;		// формируем образ маркера EPC с прямым порядком байт
	epc_image[1]=EPC_LOW;
	epc_image[2]=(unsigned char)(mark_mas[1].m.epc.Lepc>>8);
	epc_image[3]=(unsigned char)mark_mas[1].m.epc.Lepc;
	epc_image[4]=(unsigned char)(DL_value>>24);
	epc_image[5]=(unsigned char)(DL_value>>16);
	epc_image[6]=(unsigned char)(DL_value>>8);
	epc_image[7]=(unsigned char)DL_value;
	epc_image[8]=mark_mas[1].m.epc.Pepc;
	mark_mas[1].m.epc.Pcrc=Crc16(epc_image, EPC_LN);	// вычисляем контрольную сумму EPC
}
*/

/**
 * \brief  Копирование данных из входного буфера (jpeg2000 часть1) на свои места в выходном буфере (jpeg2000 частьII)
 * \details Данные копируются на свои места, пропуская места, которые займут
 *		сегменты маркеров jpwl
 * \param inbuf Входной буфер, содержащий кодовый поток jpeg2000 часть1
 * \param outbuf  Выходной буфер, в который будут скопированы данные из входного буфера
 */
void enc_data_copy(unsigned char* inbuf, unsigned char* outbuf) {
	int i, j;
	unsigned char* o_b;

	o_b = outbuf;
	j = 0;
	for (i = 0; i < mark_count; i++) { // цикл по таблице маркеров
		while (o_b != outbuf + mark_mas[i].pos_out) { // копируем в вых. буфер данные
			*o_b++ = *inbuf++;				// между последним маркером (или началом буфера)
			// и текущим маркером
			j++;
		};
		o_b += mark_mas[i].len + 2;		// пропускаем в вых. буфере место под сегмент маркера + маркер
		j += mark_mas[i].len + 2;			// наращиваем длину скопированных/пропущенных данных
	};
	for (; j < DL_value; j++)				// копируем оставшиеся данные, расположенные
		*o_b++ = *inbuf++;				// после последнего маркера
}

/**
 * \brief Копирование в выходной буфер маркера и сегмента маркера EPB кроме входящих в сегмент кодов четности (они будут вставляться позже).
 * \details Копирование выполняется по принципу "big endian" - старшие байты вперед
 * \param e Ссылка на структуру w_marker с параметрами маркера EPB
 * \param outbuf Выходной буфер, в который выполняется копирование
 */
void epb_copy(w_marker* w_m, unsigned char* outbuf) {
	unsigned char* c, * v, * p;
	epb_ms* e;
	unsigned short us;

	c = outbuf + w_m->pos_out;				// адрес в выходном буфере
	e = &w_m->m.epb;						// для сокращения ссылок
	if (w_m->tile_num >= 0 && e->index == 0) {	// первый EPB в заголовке тайла - здесь следует скорректировать
		// Psot в сегменте маркера SOT
		v = c - 3;							// адрес старшего байта Psot
		p = (unsigned char*)(Psot_new + w_m->tile_num); // адрес мл. байта нового значения Psot
		*v-- = *p++;
		*v-- = *p++;
		*v-- = *p++;
		*v = *p;
	};
	*c++ = (unsigned char)(w_m->id >> 8);	// собственно копирование
	*c++ = (unsigned char)w_m->id;
	*c++ = (unsigned char)(e->Lepb >> 8);
	*c++ = (unsigned char)e->Lepb;
	*c++ = (unsigned char)e->Depb;
	*c++ = (unsigned char)(e->LDPepb >> 24);
	*c++ = (unsigned char)(e->LDPepb >> 16);
	*c++ = (unsigned char)(e->LDPepb >> 8);
	*c++ = (unsigned char)e->LDPepb;
	*c++ = (unsigned char)(e->Pepb >> 24);
	*c++ = (unsigned char)(e->Pepb >> 16);
	*c++ = (unsigned char)(e->Pepb >> 8);
	*c++ = (unsigned char)e->Pepb;
	if (par_val.interleave_used && w_m->tile_num >= 0) {	// Используем Ammendment EPB в тайлах
		// Заносим информацию о EPB блоке в EPC
		*epc_point++ = (unsigned char)(e->Pepb >> 24);	// Запись RSepb = Pepb 4 байта ! ( в Ammendment ошибка)
		*epc_point++ = (unsigned char)(e->Pepb >> 16);
		*epc_point++ = (unsigned char)(e->Pepb >> 8);
		*epc_point++ = (unsigned char)e->Pepb;
		*epc_point++ = (unsigned char)(e->Lepb >> 8);	// Запись Lebp 2 байта ! ( в Ammendment ошибка)
		*epc_point++ = (unsigned char)e->Lepb;
		*epc_point++ = (unsigned char)(w_m->pos_out >> 24);	// Запись Oepb
		*epc_point++ = (unsigned char)(w_m->pos_out >> 16);
		*epc_point++ = (unsigned char)(w_m->pos_out >> 8);
		*epc_point++ = (unsigned char)w_m->pos_out;
	}
}

/**
 * \brief Копирование в выходной буфер маркера и сегмента маркера EPC
 * \details Копирование выполняется по принципу "big endian" - старшие байты вперед
 * Входные параметры:
 * \param e  Ссылка на структуру w_marker с параметрами маркера EPC
 * \param outbuf  Выходной буфер, в который выполняется копирование
 */
void epc_copy(w_marker* w_m, unsigned char* outbuf) {
	unsigned char* c;
	epc_ms* e;
	unsigned short Lid;

	c = outbuf + w_m->pos_out;				// адрес в выходном буфере
	e = &w_m->m.epc;						// для сокращения ссылок
	*c++ = (unsigned char)(w_m->id >> 8);	// собственно копирование
	*c++ = (unsigned char)w_m->id;
	*c++ = (unsigned char)(e->Lepc >> 8);
	*c++ = (unsigned char)e->Lepc;
	*c++ = (unsigned char)(e->Pcrc >> 8);
	*c++ = (unsigned char)e->Pcrc;
	*c++ = (unsigned char)(e->DL >> 24);
	*c++ = (unsigned char)(e->DL >> 16);
	*c++ = (unsigned char)(e->DL >> 8);
	*c++ = (unsigned char)e->DL;
	*c++ = (unsigned char)e->Pepc;
	if (par_val.interleave_used) {	// Используем Ammendment
		*c++ = 0x02;				// ID=0x0200 - внутрикадровое чередование
		*c++ = 0x00;
		Lid = 2 + epb_count * 10;		// Вычисляем Lid
		*c++ = (unsigned char)(Lid >> 8); // Заносим Lid
		*c++ = (unsigned char)Lid;
		*c++ = (unsigned char)(epb_count >> 8); // Заносим Nepb
		*c++ = (unsigned char)epb_count;
		epc_point = c;
	}
}

unsigned char* cur_pack;	//ссылка на чувствительность тек. пакета для 
// копирования ESD в пакетном режиме

/**
 * \brief Копирование в выходной буфер маркера и сегмента маркера ESD
 * \details Копирование выполняется по принципу "big endian" - старшие байты вперед
 * Входные параметры:
 * \param e  Ссылка на структуру w_marker с параметрами маркера ESD
 * \param outbuf  Выходной буфер, в который выполняется копирование
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 */
void esd_copy(w_marker* w_m, unsigned char* outbuf, unsigned short* tile_packets) {
	unsigned char* c;
	esd_ms* e;
	int i;

	c = outbuf + w_m->pos_out;				// адрес в выходном буфере
	e = &w_m->m.esd;						// для сокращения ссылок
	*c++ = (unsigned char)(w_m->id >> 8);	// собственно копирование
	*c++ = (unsigned char)w_m->id;
	*c++ = (unsigned char)(e->Lesd >> 8);
	*c++ = (unsigned char)e->Lesd;
	*c++ = (unsigned char)(e->Cesd >> 8);
	*c++ = (unsigned char)e->Cesd;
	*c++ = (unsigned char)e->Pesd;
	if (e->addrm == 1 && e->interv_cnt == 0) { // байтовый диапазон без интервалов - это
		// ESD с чувствительностью заголовка
		*(unsigned long*)c = 0;			// 4-байтное смещение начала заголовка 
		// отн. начала заголовка
		c += 4;
		i = w_m->tile_num + 1;
		*c++ = (unsigned char)(h_length[i] >> 24); // смещение последнего байта заголовка
		*c++ = (unsigned char)(h_length[i] >> 16);
		*c++ = (unsigned char)(h_length[i] >> 8);
		*c++ = (unsigned char)h_length[i];
		*c++ = 0xff;										// относительная чувствительность заголовка
		// максимальная
	}
	else if (e->addrm == 1 && e->interv_cnt != 0) // байтовый диапазон с интервалами - 
		// это чувствительность данных тайла
		for (i = e->interv_start; i < e->interv_start + e->interv_cnt; i++) {
			*c++ = (unsigned char)(e_intervals[i].start >> 24);
			*c++ = (unsigned char)(e_intervals[i].start >> 16);
			*c++ = (unsigned char)(e_intervals[i].start >> 8);
			*c++ = (unsigned char)e_intervals[i].start;
			*c++ = (unsigned char)(e_intervals[i].end >> 24);
			*c++ = (unsigned char)(e_intervals[i].end >> 16);
			*c++ = (unsigned char)(e_intervals[i].end >> 8);
			*c++ = (unsigned char)e_intervals[i].end;
			*c++ = (unsigned char)e_intervals[i].sens;
		}
	else {				// пакетный режим - данные об отн. чувствительности пакетов тайла
		for (i = 0; i < tile_packets[w_m->tile_num]; i++) // копируем чувствительности пакетов
			*c++ = *cur_pack++;
	};
}

/**
 * \brief  Копирование маркеров в выходной буфер
 * \details Копирование маркеров, параметров сегментов маркеров и данных сегментов ESD
 * в выходной буфер. Копирование выполняется по правилу "big endian", т.е.
 * от старшего байта к младшему (согласно спецификациям T.800 и T.810).
 * Смещение первого байта, начиная с которого выполняется копирование маркера,
 * задается предварительно вычисленным значением mark_mas[i].pos_out
 * \param outbuf  Выходной буфер, в который выполняется копирование
 * \param tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 * \param packet_sense Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 255). В массиве packet_sense сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов, затем второго и т.д.
 */
void markers_copy(unsigned char* outbuf, unsigned short* tile_packets, unsigned char* packet_sense) {
	int i;

	cur_pack = packet_sense;
	for (i = 0; i < mark_count; i++)	// перебор маркеров из массива mark_mas
		switch (mark_mas[i].id) {
		case EPB_MARKER:					// это маркер EPB
			epb_copy(mark_mas + i, outbuf);	// копирование маркера EPB
			break;
		case EPC_MARKER:					// это маркер EPC
			epc_copy(mark_mas + i, outbuf);	// копирование маркера EPC
			//			mark_mas[i].data_ready=_true_;	// данные готовы
			break;
		case ESD_MARKER:					// это маркер ESD
			esd_copy(mark_mas + i, outbuf, tile_packets);	// копирование маркера ESD
			//			mark_mas[i].data_ready=_true_;	// данные готовы
		};
}

/**
 * \brief  Заполнение блоков EPB
 * \details Заполнение блоков EPB, расположенных в выходном буфере, кодами четности
 * в соответствии с предварительно установленными параметрами защиты в этих блоках.
 * \param  outbuf Адрес выходного буфера, который заполнен всеми данными и сегменнтами маркеров jpwl кроме кодов четности блоков EPB
 */
void fill_epb(unsigned char* outbuf)
{
	int i, j, l, n_rs_old, k_rs_old, n_rs, k_rs;
	unsigned char* postrs_start;			// начало кодов четности для пост-данных в выходном буфере
	unsigned char* postdata_start;			// адрес начала пост-данных в вых. буфере 
	epb_ms* e;
	unsigned char data_buf[64];			// буфер для неполностью заполняемых кодируемых данных
	unsigned char* tile_adr;			// адрес текущего тайла
	int_struct* cur_int;				// ссылка на текущий интервал в массиве e-intervals
	unsigned short crc16_buf;			// буфер для вычисления crc16
	unsigned long crc32_buf;			// буфер для вычисления crc32

	cur_int = e_intervals;				// ссылка на первый интервал чувствительности
	n_rs_old = k_rs_old = 0;				// последние параметры RS-кодов
	for (i = 0; i < mark_count; i++) 		// цикл по таблице маркеров с целью перебрать все EPB
		if (mark_mas[i].id == EPB_MARKER) {	// нашли очередной EPB
			// кодируем пре-данные
			e = &mark_mas[i].m.epb;			// ссылка на данные о EPB в массиве маркеров
			if (e->index == 0)					// первый EPB в заголовке
				if (mark_mas[i].tile_num < 0) {				// основной заголовок
					if (e->pre_len != e->k_pre) {
						memset(data_buf, 0, 64);	// обнуляем кодируемый буфер
						memcpy(data_buf, outbuf, e->pre_len); // копируем кодируемые данные в начало буфера
						if (n_rs_old != 160 || k_rs_old != 64)
							init_rs(160, 64);		// инициализация предопределенного кода RS(164,60)
						encode_rs(data_buf, outbuf + mark_mas[i].pos_out + EPB_LN + 2, 160, 64); // RS-кодирование
					}
					else {
						if (n_rs_old != 160 || k_rs_old != 64)
							init_rs(160, 64);		// инициализация предопределенного кода RS(164,60)
						encode_rs(outbuf, outbuf + mark_mas[i].pos_out + EPB_LN + 2, 160, 64);	// RS-кодирование
					};
					postrs_start = outbuf + mark_mas[i].pos_out + EPB_LN + 2 + 96; // запоминаем адрес начала
					// кодов четности для пост-данных
					n_rs_old = 160; k_rs_old = 64;		// запоминаем последние параметры RS-кодов
					postdata_start = outbuf + h_length[0] - e->post_len + 1; // вычисляем адрес начала пост-данных
					// адрес вых. буфера + смещение последнего байта осн. заголовка - длина
					// пост-данных + 1
				}
				else {						// заголовок тайла
					if (n_rs_old != 80 || k_rs_old != 25)	// переход на новый RS-код
						init_rs(80, 25);		// инициализация предопределенного кода RS(80,25)
					encode_rs(outbuf + mark_mas[i].pos_out - SOT_LN - 2, outbuf + mark_mas[i].pos_out + EPB_LN + 2, 80, 25);
					postrs_start = outbuf + mark_mas[i].pos_out + EPB_LN + 2 + 55;	// позиция начала RS-кодов в вых. буфере
					n_rs_old = 80; k_rs_old = 25;		// запоминаем последние параметры RS-кодов

					tile_adr = outbuf + mark_mas[i].pos_out - SOT_LN - 2;	// начало тайла = начало первого EPB в заголовке тайла - длина
					// сегмента SOT - длина маркера SOT
					postdata_start = tile_adr + h_length[mark_mas[i].tile_num + 1] - e->post_len + 1; // вычисляем адрес начала пост-данных
					// адрес начала тайла + смещение последнего байта заголовка тайла - длина
					// пост-данных + 1

				}
			else {							// не первый EPB в заголовке (защита данных тайла)
				if (n_rs_old != 40 || k_rs_old != 13)
					init_rs(40, 13);		// инициализация предопределенного кода RS(80,25)
				encode_rs(outbuf + mark_mas[i].pos_out, outbuf + mark_mas[i].pos_out + EPB_LN + 2, 40, 13);
				postrs_start = outbuf + mark_mas[i].pos_out + EPB_LN + 2 + 27;
				n_rs_old = 40; k_rs_old = 13;		// запоминаем последние параметры RS-кодов
				postdata_start = tile_adr + cur_int++->start; // адрес пост данных = адрес тайла + смещение тек. 
				// интервала отн. начала тайла
			};
			// кодируем пост-данные
			if (e->hprot == 16) {					// двухбайтная контрольная сумма CRC-16
				//				for(j=e->post_len; j>0; j-=l) {	// цикл по блокам данных, в кот. вычисл. контрольная сумма
				//					l= j>=crc16_blocksize? crc16_blocksize: j;		// длина блока для контрольной суммы
																// crc16_blocksize - максимум, если блок данных больше crc16_blocksize байт
																// или все j байт, если блок меньше crc16_blocksize байт
				//					crc16_buf=Crc16(postdata_start,l); // вычисление контр. суммы
				//					*postrs_start++=(unsigned char)(crc16_buf>>8); // запись контрольной суммы в прямом 
				//					*postrs_start++=(unsigned char)crc16_buf;		// порядке байт
				//					postdata_start+=l;			// переходим к следующему блоку данных для контрольной суммы
				//				}
				crc16_buf = Crc16(postdata_start, e->post_len); // вычисл. конт. суммы
				*postrs_start++ = (unsigned char)(crc16_buf >> 8);
				*postrs_start++ = (unsigned char)crc16_buf;

			}
			else if (e->hprot == 32) {					// 4-байтная контрольная сумма CRC-32
				crc32_buf = Crc32(postdata_start, e->post_len); // вычисл. конт. суммы
				*postrs_start++ = (unsigned char)(crc32_buf >> 24); // запись контрольной суммы в прямом
				*postrs_start++ = (unsigned char)(crc32_buf >> 16); // порядке байт
				*postrs_start++ = (unsigned char)(crc32_buf >> 8);
				*postrs_start++ = (unsigned char)crc32_buf;
			}
			else if (e->hprot != 0) {				// RS-код
				if (e->hprot == 1)					// предопределенный RS-код для заголовка
					if (mark_mas[i].tile_num < 0) {			// осн. заголовок - RS(160,64)
						n_rs = 160;				// параметры RS-кода
						k_rs = 64;
					}
					else {						// заголовок тайла - RS(80,25)
						n_rs = 80;				// параметры RS-кода
						k_rs = 25;
					}
				else {						// RS-код от 27 до 128
					n_rs = e->hprot;			// параметры RS-кода
					k_rs = 32;
				};
				if (n_rs != n_rs_old || k_rs != k_rs_old) {
					init_rs(n_rs, k_rs);		// инициализируем кодер RS на новые параметры
					n_rs_old = n_rs;			// запоминаем параметры последней инициализации
					k_rs_old = k_rs;
				};
				for (j = e->post_len; j >= k_rs; j -= k_rs) { // цикл по блокам из k_rs байт для вычисл. RS-кодов
					encode_rs(postdata_start, postrs_start, n_rs, k_rs); // RS-кодирование
					postdata_start += k_rs;				// на начало след. блока из k_rs байт
					postrs_start += n_rs - k_rs;			// на начало след. блока кодов четности
				};
				if (j > 0) {					// остался фрагмент менее k_rs байт данных
					memset(data_buf, 0, 64);	// обнуляем кодируемый буфер
					memcpy(data_buf, postdata_start, j); // копируем кодируемые данные в начало буфера
					encode_rs(data_buf, postrs_start, n_rs, k_rs); // кодируем данные из буфера
				}
			};
			//			mark_mas[i].data_ready=_true_;
		};

}

/**
 * \brief Вычисление контрольной суммы для сегмента EPC и занесение ее в выходной буфер
 * \return Нет возвращаемого значения
 */
void epc_crc()
{
	unsigned char* c;
	unsigned short l_epc, crc;

	c = par_val.out_buffer + mark_mas[1].pos_out;	// Адрес начала EPC в выходном буфере
	l_epc = mark_mas[1].len + 2;		// Длина сегмента вместе с маркером
	memcpy(imatrix, c, 4);			// Копируем первые 4 байта EPC
	memcpy(imatrix + 4, c + 6, l_epc - 6);	// Копируем остальную часть EPC за исключением контрольной суммы
	crc = Crc16(imatrix, l_epc - 2);		// Вычисляем контрольную сумму
	c += 4;			// Указатель на Crc выходном буферен
	*c++ = (unsigned char)(crc >> 8);	// Запись контрольной суммы в выходной буфер
	*c++ = (unsigned char)crc;
}

/**
 * \brief Внетрикадровая перестановка выходного потока согласно Ammendment
 * \return Нет возвращаемого значения
 */
void interleave_outstream()
{
	unsigned long Nc, Nr, i, j, k, Len;
	unsigned char* c;

	Len = DL_value - (h_length[0] + 1);		// Длина переставляемых данных: общая длина минус основной заголовок
	Nc = (unsigned long)ceil(sqrt((double)Len));	// Количество столбцов
	Nr = (unsigned long)ceil(((double)Len / Nc));			// Количество строк
	for (k = 0, c = par_val.out_buffer + h_length[0] + 1, j = 0; j < Nc; j++)
		for (i = 0; i < Nr; i++) {
			imatrix[i * Nc + j] = *c++;
			if (++k == Len)
				goto mcop;			// Все переставлено переход к копированию
		};
mcop:
	memcpy(par_val.out_buffer + h_length[0] + 1, imatrix, Nc * Nr);
	amm_len = h_length[0] + 1 + Nc * Nr;
}

/**
 * \brief Кодер jpwl
 * \param  inbuf Ссылка на начало буфера, в котором находится кодовый поток jpeg200 часть 1
 * \param  outbuf  Ссылка на начало буфера, в который будет помещен кодовый поток jpeg200 часть 2 с внедренными в него средствами защиты от ошибок jpwl
 * \param  tile_packets  Массив, содержащий количество пакетов в каждом тайле потока: tile_packets[i] - количество пакетов i-го по порядку от начала кодового потока тайла
 * \param  packet_sense Массив данных об относительной чувствительности пакетов к ошибках (значения 0 - 254). В массиве packet_sense сначала идут данные о пакетах первого по порядку тайла в порядке расположения пакетов, затем второго и т.д.
 * \param out_len  Адрес переменной в которую заносится длина выходного кодированного потока (количество байт, записанных в outbuf)
 * \return Код завершения: 0 - все нормально, -1 - недостаточно места в буферах для тайлов, -2 - неверная структура кодового потока jpeg2000 часть 1, -3 - слишком большой заголовок, недостаточно одного EPB, -4 - недостаточно места в массиве для размещения всех маркеров, -5 - недостаточно места в массиве для интервалов чувствительности, -6 - недопустимая комбинация исходных параметров
 */
int w_encoder(unsigned char* inbuf, unsigned char* outbuf, unsigned short* tile_packets,
	unsigned char* packet_sense, unsigned long* out_len)
{
	int exit_code;

	if (w_enc_init(inbuf) != 0) {	// инициализация кодера, проверка параметров
#ifdef _DEBUG
		printf("w_encoder aborted exit_code=-6");
#endif
		return (-6);		// если неверные значеня параметров - выход
		// создание маркеров в массиве mark_mas
	};
	exit_code = w_markers_create(inbuf, outbuf, tile_packets, packet_sense);
	if (exit_code != 0) {
#ifdef _DEBUG
		printf("w_encoder aborted exit_code=%d", exit_code);
#endif		
		return(exit_code);
	};
	//	locate_markers(inbuf);		// привязка маркеров jpwl к позиции в выходном буфере
	//	finalize_marker_params();	// завершение создания параметров маркеров jpwl - 
									// доделываем EPC
	enc_data_copy(inbuf, outbuf);	// копирование данных из входного буфера 
	// на свои места в выходной буфер
	markers_copy(outbuf, tile_packets, packet_sense); // копирование маркеров в вых. буфер
	epc_crc();					// Вычисление контрольной суммы для сегмента EPC
	fill_epb(outbuf);			// заполнение блоков EPB кодами четности
	if (par_val.interleave_used) 	// Используем Ammendment
		interleave_outstream();		// Перестановка выходного потока
	if (par_val.interleave_used)	// записываем длину кодового потока в выходном буфере		
		*out_len = amm_len;			// Длина при использовании Ammendment
	else
		*out_len = DL_value;			// Длина без использования Ammendment
	return(0);
}

/**
 * \brief  Вызов кодера jpwl с установкой параметров кодера, переданных из ПО ПИИ
 * \details Побочный эффект:
 * По адресу out_len записывается длина в байтах сформированного кодового потока.
 * В структуру по адресу w_params записывается длина сформированного кодового потока и
 * длина основного заголовка.
 * \param  w_params Cсылка на структуру w_encoder_params_value со значениями параметров кодера
 * \return Код завершения, который возвращает кодер: 0 - все нормально, -1 - недостаточно места в буферах для тайлов, -2 - неверная структура кодового потока jpeg2000 часть 1, -3 - слишком большой заголовок, недостаточно одного EPB, -4 - недостаточно места в массиве для размещения всех маркеров, -5 - недостаточно места в массиве для интервалов чувствительности, -6 - недопустимая комбинация исходных параметров
 *
 */
int w_encoder_call(w_enc_params* w_params)
{
	int i;

	wcoder_mh_param = w_params->wcoder_mh;	// присваивание значений параметров кодера внешним переменным
	wcoder_th_param = w_params->wcoder_th;
	wcoder_data_param = w_params->wcoder_data;
	esd_use = w_params->esd_used;
	esd_mode = w_params->esd_mode;
	interleave_use = w_params->interleave_used;
	// вызов кодера и возврат возвращааемого им значения
	i = w_encoder(w_params->inp_buffer, w_params->out_buffer, w_params->tile_packets,
		w_params->packet_sense, &(w_params->wcoder_out_len));
	w_params->wcoder_mh_len = h_length[0] + 1;					// Записываем длину основного заголовка
	//	w_params->wcoder_outlen=i;
	return(i);
}

#endif

/**
 * \brief  Установка значений параметров кодера jpwl по умолчанию
 * \param  j_e_p Cсылка на структуру jpwl_encoder_params со значениями параметров кодера jpwl
 * \return Код завершения, который возвращает кодер: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void jpwl_enc_set_default_params(jpwl_enc_params* j_e_p)
{
	j_e_p->wcoder_mh = 1;		// предопределенная защита основного заголовка
	j_e_p->wcoder_th = 1;		// предопределенная защита заголовка тайла
	j_e_p->wcoder_data = 64;	// код RS(37,32) для защиты данных
	j_e_p->esd_used = _false_;			// Маркер ESD не вставляется в кодовый поток
	//	j_e_p->esd_use_val=0;				// ESD не используется
	j_e_p->jpwl_enc_mode = 1;		// Использовать jpwl
	j_e_p->interleave_used = _true_;	// Использовать Ammendment
	//	j_e_p->tile_packets_val=NULL;		// Этот параметр должен быть получен из ПО ПИИ
	//	j_e_p->packet_sense_val=NULL;			// Этот параметр должен быть получен из ПО ПИИ	
}

/**
 * \brief  Инициализация значений параметров кодера jpwl, переданных из ПО ПИИ
 * \param  j_e_p Cсылка на структуру jpwl_encoder_params со значениями параметров кодера jpwl
 * \return Код завершения, который возвращает кодер: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void jpwl_enc_init(jpwl_enc_params* j_e_p)
{
	par_val.wcoder_mh = j_e_p->wcoder_mh;
	par_val.wcoder_th = j_e_p->wcoder_th;
	par_val.wcoder_data = j_e_p->wcoder_data;
	par_val.esd_used = j_e_p->esd_used;
	par_val.esd_mode = ESD_PACKETS;
	par_val.interleave_used = j_e_p->interleave_used;
	//	par_val.tile_packets_value=j_e_p->tile_packets_val;
	//	par_val.packet_sense_value=j_e_p->packet_sense_val;
	par_val.jpwl_enc_mode = j_e_p->jpwl_enc_mode;
	encoder_dump_no = 0;
}

/**
 * \brief  Запуск кодера jpwl
 * \param  BufForIn Cсылка на входной буфер
 * \param  BufForOut Cсылка на выходной буфер
 * \param  j_e_b_p Cсылка на структуру jpwl_encoder_BufferParams с дополнительными данными для кодера
 * \param  j_e_b_r Cсылка на структуру jpwl_encoder_BufferResults с дополнительными результатами кодера
 * \return Код завершения, который возвращает кодер: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
errno_t jpwl_enc_run(unsigned char* BufForIn, unsigned char* BufForOut,
	jpwl_enc_bParams* j_e_b_p,
	jpwl_enc_bResults* j_e_b_r)
{
	int i;
	unsigned char* v;
	addr_char ac;

	par_val.inp_buffer = BufForIn;
	par_val.out_buffer = BufForOut;
	par_val.tile_packets = j_e_b_p->tile_packets;
	par_val.packet_sense = j_e_b_p->pack_sens;
	if (par_val.jpwl_enc_mode) {		// кодирование при использовании jpwl
		i = w_encoder_call(&par_val);
		if (i == 0) {
			j_e_b_r->wcoder_out_len = par_val.wcoder_out_len;
			j_e_b_r->wcoder_mh_len = par_val.wcoder_mh_len;
		}
		else
			return 1;
	}
	else {
		memcpy(BufForOut, BufForIn, j_e_b_p->stream_len);		// копирование в выходной буфер при неиспользовании jpwl
		j_e_b_r->wcoder_out_len = j_e_b_p->stream_len;			// длина такая же как у входного потока
		v = mark_search(BufForIn, SOT_LOW, EOC_LOW, &ac);			// поиск первого тайла
		if (v == NULL)				// тайл не найден
			return 1;
		else
			j_e_b_r->wcoder_mh_len = v - BufForIn;				// длина основного заголовка
	};
	return 0;
}

#ifdef _TEST

__declspec(dllexport)
void sens_create(unsigned char* i_buf, unsigned short* t_buf, unsigned char* p_buf)
{
	unsigned char* g, * p, * v;
	int i, j, k, p_no;
	int l;


	//	mark_search(buf, marker, terminated_marker, t_adr)
	p_no = 0;				// номер тек. эл-та в массиве packsens_b
	// вычисляем кол-во тайлов
	v = NULL;
	//	for(i=1,g=mark_search(in_b, SOT_LOW, EOC_LOW, &v); g!=NULL; g=mark_search(g+2, SOT_LOW, EOC_LOW, &v),i++);
	g = i_buf;
	for (i = 0; (g = mark_search(g + 2, SOT_LOW, EOC_LOW, &v)) != NULL; i++)
		;
	p = i_buf;
	for (j = 0; j < i; j++) {			// цикл по тайлам
		p = mark_search(p, SOT_LOW, EOC_LOW, &v);
		for (k = 0; p != NULL; k++) {		// вычисляем кол-во пакетов в тайле по маркерам SOP
			if (j != i - 1)
				p = mark_search(p + 2, SOP_LOW, SOT_LOW, &v);
			else
				p = mark_search(p + 2, SOP_LOW, EOC_LOW, &v);
		};
		p = v;
		t_buf[j] = k - 1;			// заносим кол-во пакетов тайла в tiles_b[j]
		//17.06 Имитация отсутствия маркеров SOP
		if (t_buf[j] == 0)
			t_buf[j] = 31;
		//17.06 
		for (l = t_buf[j] - 1, v = p_buf + p_no; l >= 0; l--) // присваиваем чувствительности пакетов по методу
			// OpenJpej
			*(v++) = l;
		// *(v++)=0xfe;
		p_no += t_buf[j];				// наращиваем тек. номер пакета на кол-во записанных пакетов
	}

	//	tiles_b[0]=18;
	//	for(l=tiles_b[0], v=packsens_b; l>=0; l--) // присваиваем чувствительности пакетов по методу
													 // OpenJpej
	//			 *(v++)=l;
	//			*(v++)=0xfe;

}

#endif
