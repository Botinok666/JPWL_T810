
/**
 * \file jpwl_decoder.cpp
 * \brief Файл библиотеки подпрограмм декодера JPWL
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12
*/
#include "stdio.h"
#include <crtdbg.h>
#include "math.h"
#include <string.h>
#include "jpwl_params.h"
#include "..\rs_crc_lib\rs_crc_decl.h"
#include "..\rs_crc_lib\rs_crc_import.h"

#ifndef _TEST
#define _TEST
#endif

unsigned char* in_buf;		///< Адрес входного буфера
unsigned long in_len;		///< Количествово байт во входном буфере
unsigned char* out_buf;		///< Адрес выходного буфера
unsigned long out_len;		///< Количество байт, записанных в выходной буфер
unsigned short tile_count;	///< Счетчик успешно скорректированных тайлов
//unsigned char rs_data[10000];	///< Буфер для неполно заполненных корректируемых RS-кодами данных и для проверки контрольной суммы EPC (для случая внутрикадрового чередования длина буфера должна позволить поместить всю карту EPB)
unsigned char rs_data[256];	///< Буфер для неполно заполненных корректируемых RS-кодами данных и для проверки контрольной суммы EPC (для случая внутрикадрового чередования длина буфера должна позволить поместить всю карту EPB)
w_marker mark_mas[MAX_MARKERS]; ///< Массив маркеров jpwl и некорректируемых участков, обнаруженных при декодировании
unsigned short mark_count;	///< Счетчик обнаруженных и записанных в mark_mas маркеров
unsigned short badparts[MAX_BADPARTS];	///< Массив индексов некорректируемых фрагментов пост-данных блока EPB
unsigned short badparts_count;	///< Счетчик некорректируемых фрагментов пост-данных блока EPB
_bool_ esd_yes;	///< ESD используется в кодовом потоке?
_bool_ epb_yes;	///< EPB используется в кодовом потоке?
_bool_ red_yes;	///< RED используются в кодовом потоке?
unsigned long DL_value;		///< Значение длины из EPC 
unsigned char* end_buf;		///< Ссылка на конец входного буфера
r_intervals r_int[MAX_INTERVALS * MAX_TILES]; ///< Массив интервалов для RED - задаются смещениями отн. начала тайла
unsigned long interv_count;		///< Счетчик интервалов
unsigned short old_rs_mode;	///< RS-код, который был проинициализирован последним
w_dec_params dec_par;	///< Параметры для запуска декодера jpwl
// 18.04
unsigned long MH_Tile_Len;		// Сумма длин основного заголовка и тайлов, копируемых в выходной буфер
// 18.04
								// Для статистики
unsigned long FullRestoredCadrs;	///<  Количество полностью восстановленных видеокадров
unsigned long PartiallyRestoredCadrs;	///< Количество частично восстановленных видеокадров
unsigned long NonRestoredCadrs;		///< Количество невосстановленных видеокадров
unsigned long NonJPWLCadrs;			///< Количество видеокадров, в которых отсутствуют средства JPWL
_bool_ IsBadBlocks;					///< Обнаружены ли невосстанавливаемые тайлы( _true_, _false_)
_bool_ IsAmmendment;				///< Используется ли Ammendment в кодовом потоке ( _true_, _false_)
unsigned short tepb_count;			///< Количество записей в таблице EPB при использовании Ammendment
unsigned char* tepb_adr;			///< Адрес таблицы EPB при использовании Ammendment
unsigned long mh_len;				///< Длина основного заголовка во входном буфере 
unsigned short tile_all_restored_count;	///< Количество полностью восстановленных тайлов кадра
unsigned short tile_red_restored_count;	///< Количество частично восстановленных тайлов кадра, в которых присутствуют маркеры RED
unsigned long bad_block_size;		 ///< Количество нераспознанных как тайл байт данных


/**
 * \brief определение способа защиты пост-данных блока EPB
 * \details Возможные коды защиты пост-данных:
 * 0 - защита пост-данных отсутствует
 *	16 - контрольная свумма crc-16
 *	32 - контрольная свумма crc-32
 *	37 - RS(37,32)
 *	38 - RS(38,32)
 *	40 - RS(40,32)
 *	43 - RS(43,32)
 *	45 - RS(45,32)
 *	48 - RS(48,32)
 *	51 - RS(51,32)
 *	53 - RS(53,32)
 *	56 - RS(56,32)
 *	64 - RS(64,32)
 *	75 - RS(75,32)
 *	80 - RS(80,32)
 *	85 - RS(85,32)
 *	96 - RS(96,32)
 *	112 - RS(112,32)
 *	128 - RS(128,32)
 *	40 - RS(40,13)
 *	80 - RS(80,25)
 *	160 - RS(160,64)
 * \param Pepb_value  Fдрес параметра Pepb во входном буфере
 * \param epb_type  Тип блока EPB: 0 - первый в основном заголовке, 1 - первый в заголовке тайла, 2 - не первый в заголовке.
 * \return Код защиты пост-данных (см. детали)
 */
unsigned short decode_Pepb(unsigned long* Pepb_value, unsigned char epb_type)
{
	switch (*Pepb_value) {
	case 0xffffffff: return(0);	// нет защиты
	case 0x00000000: if (epb_type == 0)	// предопределенная в первом EPB основного заголовка
		return(160);
				   else if (epb_type == 1)	// предопределенная в первом EPB заголовка тайла
		return(80);
				   else					// предопределенная в непервом EPB заголовка
		return(40);
	case 0x10: return(16);			// crc-16
	case 0x01000010: return(32);			// crc-32
	case 0x20250020: return(37);			// RS(37,32)
	case 0x20260020: return(38);			// RS(38,32)
	case 0x20280020: return(41);			// RS(41,32)
	case 0x202b0020: return(43);			// RS(43,32)
	case 0x202d0020: return(45);			// RS(45,32)
	case 0x20300020: return(48);			// RS(48,32)
	case 0x20330020: return(51);			// RS(51,32)
	case 0x20350020: return(53);			// RS(53,32)
	case 0x20380020: return(56);			// RS(56,32)
	case 0x20400020: return(64);			// RS(64,32)
	case 0x204b0020: return(75);			// RS(75,32)
	case 0x20500020: return(81);			// RS(81,32)
	case 0x20550020: return(85);			// RS(85,32)
	case 0x20600020: return(96);			// RS(96,32)
	case 0x20700020: return(112);			// RS(112,32)
	case 0x20800020: return(128);			// RS(128,32)
	case 0x20900020: return(144);			// RS(144,32)
	case 0x20A00020: return(161);			// RS(160,32)
	case 0x20B00020: return(176);			// RS(176,32)
	case 0x20C00020: return(192);			// RS(192,32)
	};
	return(1234);		// невозможный случай - для обнаружения ошибок
}

/**
 * \brief коррекция пост-данных блока EPB
 * \details  * Выполняет пофрагментную коррекцию пост-данных блока EPB.
 *	Заполняет массив badparts номерами фрагментов данных, которые не подлежат коррекции,
 *	присваивает badparts_count количество фрагментов данных, не подлежащих коррекции.
 *	Все фрагиенты, которые могут быть скорректированы, корректирует на месте
 *	\param epb_start  Адрес начала блока EPB во входном буфере, т.е. первого байта маркера
 *	\param postdata_start  Адрес начала пост-данных во входном буфере
 *	\param p_len  Длина пре-данных в байтах
 */
void postEPB_correct(unsigned char* epb_start, unsigned char* postdata_start, int p_len)
{
	unsigned short prot_mode, c16_value1, c16_value2;
	unsigned char epb_type;
	unsigned long data_len, c32_value1, c32_value2;
	unsigned char* parity_start;
	int n_p, k_p, l, i;

	badparts_count = 0;			// обнуляем счетчик некорректируемых фрагментов
	epb_type = p_len == 13 ? 2 : (p_len == 25 ? 1 : 0);	// тип EPB: 0 - первый в осн. заголовке, 1 - первый в заголовке тайла
	// 2 - не первый в заголовке 
	prot_mode = decode_Pepb((unsigned long*)(epb_start + 9), epb_type); // определяем способ защиты пост-данных
	if (prot_mode == 0)
		return;
	data_len = ((unsigned long)*(epb_start + 5)) << 24 | ((unsigned long)*(epb_start + 6)) << 16 | ((unsigned long)*(epb_start + 7)) << 8 |
		((unsigned long)*(epb_start + 8));	// значение LDPepb в обратном порядке байт
	data_len -= p_len;			// длина пост-данных
	if (epb_type == 0)				// первый EPB в осн. заголовке
		parity_start = epb_start + EPB_LN + 2 + 96;	// начало кодов четности смещено от начала epb на размер сегмента+маркер+коды для пре данных
	else if (epb_type == 1)		// первый EPB в заголовке тайла
		parity_start = epb_start + EPB_LN + 2 + 55;	// начало кодов четности смещено от начала epb на размер сегмента+маркер+коды для пре данных
	else						// не первый EPB
		parity_start = epb_start + EPB_LN + 2 + 27;	// начало кодов четности смещено от начала epb на размер сегмента+маркер+коды для пре данных
	if (prot_mode == 16) {			// защита - crc16
		c16_value1 = Crc16(postdata_start, data_len);	// вычисляем контрольную сумму пост-данных
		c16_value2 = ((unsigned short)*parity_start) << 8 | (unsigned short)*(parity_start + 1); // контрольная сумма из потока
		if (c16_value1 != c16_value2) {		// есть ошибки в пост-данных
			badparts_count = 1;				// один некорректируемый фрагмент
			badparts[0] = 0;					// некорректируемый фрагмент первый и единственный
		};
		return;
	};
	if (prot_mode == 32) {			// защита - crc32
		c32_value1 = Crc32(postdata_start, data_len);	// вычисляем контрольную сумму пост-данных
		c32_value2 = ((unsigned long)*parity_start) << 24 | ((unsigned long)*(parity_start + 1)) << 16 |
			((unsigned long)*(parity_start + 2)) << 8 | (unsigned long)*(parity_start + 3); // контрольная сумма из потока
		if (c32_value1 != c32_value2) {		// есть ошибки в пост-данных
			badparts_count = 1;				// один некорректируемый фрагмент
			badparts[0] = 0;					// некорректируемый фрагмент первый и единственный
		};
		return;
	};
	if (prot_mode == 160) {		// защита - RS(160,64)		
		n_p = 160;
		k_p = 64;
	}
	else if (prot_mode == 80) {		// защита - RS(80,25)		
		n_p = 80;
		k_p = 25;
	}
	else if (prot_mode == 40) {		// защита - RS(40,13)		
		n_p = 40;
		k_p = 13;
	}
	else {							// защита - RS(prot_mode,32)		
		n_p = prot_mode;
		k_p = 32;
	};
	if (old_rs_mode != n_p) {		// RS-код нужно инициализировать
		init_rs(n_p, k_p);		// инициализация RS-кода
		old_rs_mode = n_p;		// запомнили, что проинициализировали
	};
	for (i = 0, l = data_len; l >= k_p; i++, l -= k_p) {
		if (decode_rs(postdata_start, parity_start, n_p, k_p) < 0) // коррекция неудачна
			badparts[badparts_count++] = i;	// создаем элемент массива badparts, указывающий на 
		// нескорректированный фрагмент
		postdata_start += k_p;			// переходим к следующему блоку данных
		parity_start += (n_p - k_p);				// переходим к след. блоку RS-кодов
	};
	if (l > 0) {			// остался последний блок данных длиной менее k_p байт
		memcpy(rs_data, postdata_start, l);	// копируем оставшиеся данные в rs_data
		memset(rs_data + l, 0, 64 - l);			// оставшуюся часть rs_data до 64 байт заполняем нулями
		if (decode_rs(rs_data, parity_start, n_p, k_p) < 0) // последний уч-ток не корректируется
			badparts[badparts_count++] = i;	// создаем элемент массива badparts, указывающий на 
		// нескорректированный фрагмент
		else
			memcpy(postdata_start, rs_data, l);	// если данные скорректировались, изменяем их
		// во входном буфере
	}
}

#ifdef FAST_TILE_SEARCH
/**
 * \brief Быстрый поиск очередного тайла в случае потери структуры кодового потока
 * \details   Начиная с указанного места и до конца буфера ищет маркер SOT, предполагая, что он не был изменен,
 * предполагая наличие EPB после сегмента SOT корректируем его пре-данные, если скорректировано еще раз
 * проверяем маркер SOT и возвращаем адрес его начала. Если нескорректировано или маркер SOT после
 * коррекции стал другим маркером, продолжаем поиск
 * \param p  Адрес байта входного буфера, с которого нужно начать поиск
 * \return Fдрес байта во входном буфере, с которого начинается найденный тайл, у которого корректируются пре-данные первого EPB, или NULL, если такового нет до конца буфера.
 */
unsigned char* tile_search(unsigned char* p)
{
	unsigned char* v;
	unsigned short SOT_big_endian;

	SOT_big_endian = 0x00FF | ((unsigned short)SOT_LOW) << 8;
	for (v = p; in_len - (v - in_buf) >= 81; v++)	// ищем от заданного места до конца буфера минус 80 байт - защита 
		// пре-данных  первого EPB + 1 байт на пост-данные
		if (*(unsigned short*)v == SOT_big_endian) {  // найден SOT
			if (old_rs_mode != 80) {		// последний код не RS(80,25)
				init_rs(80, 25);			// инициализируем RS(80,25)
				old_rs_mode = 80;			// запомнили, что проинициализировали
			};
			if (decode_rs(v, v + 25, 80, 25) >= 0) // скорректировано !
				if (*(unsigned short*)v == SOT_big_endian) // после коррекции SOT остался!
					return(v);
		};
	return(NULL);
}
#endif

#ifdef SLOW_TILE_SEARCH
/*------ медленный и надежный поиск очередного тайла в случае потери структуры кодового потока -------------
 * Входные параметры:
 * p - адрес байта входного буфера, с которого нужно начать поиск.
 * Возвращаемое значение:
 * адрес байта во входном буфере, с которого начинается найденный тайл,
 * у которого корректируются пре-данные первого EPB, или NULL, если такового нет до конца буфера.
 * Действия:
 * начиная с указанного места и до конца буфера перебираем каждый байт, предполагая, что это маркер SOT b
 * он был изменен.
 * Предполагая наличие EPB после сегмента SOT корректируем его пре-данные, если скорректировано
 * проверяем маркер SOT и возвращаем адрес его начала. Если нескорректировано, переходим к след. байту.
 */
unsigned char* tile_search(unsigned char* p)
{
	unsigned char* v;
	unsigned short SOT_big_endian;

	SOT_big_endian = 0x00FF | ((unsigned short)SOT_LOW) << 8;
	for (v = p; v <= end_buf - 41; v++) {	// ищем от заданного места до конца буфера минус 40 байт - защита 
		// пре-данных не первого EPB - 1 байт на пост-данные
		if (old_rs_mode != 80) {		// последний код не RS(80,25)
			init_rs(80, 25);			// инициализируем RS(80,25)
			old_rs_mode = 80;			// запомнили, что проинициализировали
		};
		if (decode_rs(v, v + 25, 80, 25) >= 0) // скорректировано !
			if (*(unsigned short*)v == SOT_big_endian) // после коррекции SOT на свем месте!
				return(v);
	};
	return(NULL);
}
#endif

/**
 * \brief Попытка распознать тайл
 * \param v Предполагаемый адрес тайла во входном буфере. Тайл должен распознаваться с этого адреса
 * \return  Адрес первого распознанного тайла во входном буфере, т.е. тайла, у которого корректируются пре-данные первого EPB в заголовке тайла. Или NULL, если до конца буфера не удалось распознать ни один тайл.
 */
unsigned char* tile_detect(unsigned char* v)
{
	unsigned char* t;

	if (in_len - (v - in_buf) < TILE_MINLENGTH)	// С точки обнаружения тайла недостаточно места для тайла
		return(NULL);
	t = v;						// адрес предполагаемого начала тайла
	if (old_rs_mode != 80) {		// последний код не RS(80,25)
		init_rs(80, 25);			// инициализируем RS(80,25)
		old_rs_mode = 80;			// запомнили, что проинициализировали
	};
	if (decode_rs(v, v + 25, 80, 25) < 0) { // пре-данные первого EPB заголовка тайла не корректируются!
		_ASSERT(v - in_buf < in_len);
		v = tile_search(v + 80);				// ищем тайл
		if (v == NULL) // не найден ни один тайл
			return(NULL);
		else {						// тайл найден, пропущенный фрагмент заносим в mark_mas как BAD_ID
			IsBadBlocks = _true_;
			mark_mas[mark_count].id = BAD_ID;
			bad_block_size += (v - t);
			mark_mas[mark_count].m.bad.Lbad = mark_mas[mark_count].len = (v - t) - 2;
			mark_mas[mark_count].tile_num = tile_count++;
			_ASSERT(mark_count < MAX_MARKERS);
			mark_mas[mark_count++].pos_in = t - in_buf;
		}
	}
	else if (*v != 0xff || *(v + 1) != SOT_LOW) { // после коррекции на месте нет маркера SOT (невероятно, но все же..)
		_ASSERT(v + 80 - in_buf < in_len);
		v = tile_search(v + 80);
		if (v == NULL) // ищем первый "нормальный" тайл, но он не найден 
			return(NULL);
		else {							// тайл найден, пропущенный фрагмент заносим в mark_mas как BAD_ID
			IsBadBlocks = _true_;
			mark_mas[mark_count].id = BAD_ID;
			bad_block_size += (v - t);
			mark_mas[mark_count].m.bad.Lbad = mark_mas[mark_count].len = (v - t) - 2;
			mark_mas[mark_count].tile_num = tile_count++;
			_ASSERT(mark_count < MAX_MARKERS);
			mark_mas[mark_count++].pos_in = t - in_buf;
		}
	};
	return(v);
}

/**
 * \brief Обратная перестановка входного кодового потока
 * \details  Используется в случае применения внутрикадрового чередования на стороне кодера jpwl
 * \return Нет возвращаемого значения
 */
void deinterleave_instream()
{
	unsigned long Nc, Nr, i, j, k, Len, off;
	unsigned char* c;
	unsigned long wait_epb, sot_start, sot_start_old, PSot_val;
	unsigned short Lepb_val;

	Len = DL_value - mh_len;		// Длина переставляемых данных: общая длина минус основной заголовок
	Nc = (unsigned long)ceil(sqrt((double)Len));	// Количество столбцов
	Nr = (unsigned long)ceil(((double)Len / Nc));			// Количество строк
	for (k = 0, c = out_buf, j = 0; j < Nc; j++)
		for (i = 0; i < Nr; i++) {
			*c++ = in_buf[mh_len + i * Nc + j];
			if (++k == Len)
				goto mcop;			// Все переставлено переход к копированию
		};

mcop:
	memcpy(in_buf + mh_len, out_buf, Len);
	in_len = DL_value;
	// Восстановление маркеров EPB и SOT и фрагментов их сегментов на основе таблицы EPB
	wait_epb = sot_start = 0;
	for (i = 0; i < tepb_count; i++, tepb_adr += 10) {
		off = ((unsigned long)*(tepb_adr + 6)) << 24 | ((unsigned long)*(tepb_adr + 7)) << 16 | ((unsigned long)*(tepb_adr + 8)) << 8 |
			((unsigned long)*(tepb_adr + 9));			// Смещение EPB относительно начала потока
		in_buf[off] = 0xFF;				// Восстанавливаенм маркер EPB
		in_buf[off + 1] = EPB_LOW;
		memcpy(in_buf + off + 2, tepb_adr + 4, 2);	// Восстанавливаем Lepb
		Lepb_val = ((unsigned short)*(tepb_adr + 4)) << 8 | ((unsigned short)*(tepb_adr + 5));	// Значение Lepb
		memcpy(in_buf + off + 9, tepb_adr, 4);	// Восстанавливаем Pepb

		if (off != wait_epb) {					// Это первый EPB в заголовке тайла
			sot_start_old = sot_start;		// Сохраняем начало предыдущего SOT
			sot_start = off - 12;				// Смещение маркера SOT
			in_buf[sot_start] = 0xFF;			// Восстанавливаем маркер SOT
			in_buf[sot_start + 1] = SOT_LOW;
			in_buf[sot_start + 2] = 0;			// Восстанавливаем длину сегмента маркера SOT
			in_buf[sot_start + 3] = 10;
			if (sot_start_old != 0) {			// Обработка не первого SOT- можно вычислить PSot
				PSot_val = sot_start - sot_start_old;	// Вычисляем значение PSot
				in_buf[sot_start_old + 6] = (unsigned char)(PSot_val >> 24);	// Восстанавливаем PSot в кодовом потоке
				in_buf[sot_start_old + 7] = (unsigned char)(PSot_val >> 16);
				in_buf[sot_start_old + 8] = (unsigned char)(PSot_val >> 8);
				in_buf[sot_start_old + 9] = (unsigned char)PSot_val;
			}
		};
		wait_epb = off + Lepb_val + 2;			// Смещение следующего ожидаемого EPB в том же заголовке тайла 
	};
	PSot_val = in_len - sot_start - 2;			// PSot для последнего SOT
	in_buf[sot_start + 6] = (unsigned char)(PSot_val >> 24);	// Восстанавливаем PSot в кодовом потоке для последнего SOT
	in_buf[sot_start + 7] = (unsigned char)(PSot_val >> 16);
	in_buf[sot_start + 8] = (unsigned char)(PSot_val >> 8);
	in_buf[sot_start + 9] = (unsigned char)PSot_val;

}


/**
 * \brief Коррекция основного заголовка
 * \details  Корректирует основной заголовок,
 * ищет первый тайл, в котором корректируются пре-данные первого EPB заголовка тайла,
 * передвигает указатель *bb на первый  байт найденного тайла.
 * Указателю по адресу bb присваивается адрес первого тайла, у которого удалось
 * скорректировать пре-данные первого EPB в заголовке тайла или NULL, усли такого тайла
 * не обнаружено.
 * Коды завершения:
 * 1 - нет средств jpwl, коррекция потока не производится
 * 0 - заголовок скорректирован,
 * -1 - заголовок не корректируется
 * -2 - кодовый поток неправильный
 * -3 - используются не поддерживаемые информативные методы
 * -4 - есть RED, которых не должно быть
 * -5 - несоответствие информации о ESD в EPC и присутствием/отсутствием ESD
 * -7 - основной заголовок длиннее входного буфера
 * -8 - неверная контрольная сумма в EPC
 * \param bb  Адрес указателя на основной заголовок
 * \return Код завершения (смю детали)
 */

int mh_correct(addr_char* bb)
{
	unsigned char* p, * epb_start, * data_start, * epc_start, * esd_start, * v, Pepc, * inf_met;
	int  rs_ret, i;
	unsigned short siz_len, epb_len, epc_len, esd_len, Pcrc, id;
	unsigned long pre_l, prot_l;
	epb_ms* e;
	epc_ms* ec;
	unsigned long cur_len;
	/*	FILE *f;
		unsigned char *cc;
		int ii;
	*/	unsigned char p_data[96];

	p = in_buf;				// адрес основоного заголовка

	esd_yes = _false_;		// нач. значения флагов присутствия
	epb_yes = _false_;		// esd, epb и red
	red_yes = _false_;
	IsAmmendment = _false_;	// Ammendment не используется
	// пытаемся распознать случай, когда во входном буфере нет средств jpwl и
	// нет ошибок - требование совместимости назад T.810
/*	if(p[0]==0xff && p[1]==SOC_LOW) // есть маркер начала кодового потока
		if(p[2]==0xff && p[3]==SIZ_LOW) { // есть сегмент SIZ
			siz_len=((unsigned short)p[4])<<8 | (unsigned short)p[5]; // длина сегмента SIZ
			if((unsigned long)siz_len+5<in_len)
				if(p[4+siz_len]==0xff)
					switch(p[5+siz_len]) {	// есть ли допустимый маркер после сегмента SIZ?
					case COD_LOW:
					case COC_LOW:
					case QCD_LOW:
					case QCC_LOW:
					case RGN_LOW:
					case POC_LOW:
					case PPM_LOW:
					case TLM_LOW:
					case PLM_LOW:
					case CRG_LOW:
					case COM_LOW: return(1);	// в кодовом потоке нет средств jpwl
					}
		};
*/
// пытаемся скорректировать пре-данные  блока EPB заголовка в предположении,
// что имеется 1 цветовая компонента
/*	memcpy(rs_data,p,62);	// копируем 62 байта пре-данных в rs_data
	rs_data[62]=rs_data[63]=0;	// обнуляем 63-й и 64-й байты массива
	init_rs(160,64);			// Инициализация RS(160,64)
	old_rs_mode=160;			// Запомнили последний код
	rs_ret=decode_rs(rs_data,p+62,160,64);	// попытка коррекции
*/
	memset(rs_data, 0, 64);	// Заполняем нулями rs_data
	memcpy(rs_data, p, 58);	// копируем 58 байта пре-данных в rs_data
	memcpy(p_data, p + 58, 96);
	init_rs(160, 64);			// Инициализация RS(160,64)
	old_rs_mode = 160;			// Запомнили последний код
	rs_ret = decode_rs(rs_data, p_data, 160, 64);	// попытка коррекции
	if (rs_ret < 0 || rs_data[0] != 0xff || rs_data[1] != SOC_LOW || rs_data[2] != 0xff || rs_data[3] != SIZ_LOW ||
		rs_data[45] != 0xff || rs_data[46] != EPB_LOW) {			// коррекция неудачна
		// пытаемся скорректировать пре-данные первого EPB заголовка в предположении,
		// что имеются 3 цветовых компоненты
		memcpy(rs_data, p, 64);	// копируем 58 байта пре-данных в rs_data
		memcpy(p_data, p + 64, 96);
		rs_ret = decode_rs(rs_data, p_data, 160, 64);	// попытка коррекции
		if (rs_ret < 0 || p[0] != 0xff || p[1] != SOC_LOW || p[2] != 0xff || p[3] != SIZ_LOW ||
			p[51] != 0xff || p[52] != EPB_LOW) {
			if (p[0] == 0xff && p[1] == SOC_LOW) // есть маркер начала кодового потока
				if (p[2] == 0xff && p[3] == SIZ_LOW) { // есть сегмент SIZ
					siz_len = ((unsigned short)p[4]) << 8 | (unsigned short)p[5]; // длина сегмента SIZ
					if ((unsigned long)siz_len + 5 < in_len)
						if (p[4 + siz_len] == 0xff)
							switch (p[5 + siz_len]) {	// есть ли допустимый маркер после сегмента SIZ?
							case COD_LOW:
							case COC_LOW:
							case QCD_LOW:
							case QCC_LOW:
							case RGN_LOW:
							case POC_LOW:
							case PPM_LOW:
							case TLM_LOW:
							case PLM_LOW:
							case CRG_LOW:
							case COM_LOW: return(1);	// в кодовом потоке нет средств jpwl
							}
				};
			return(-1);		// заголовок не может быть восстановлен
		}
		else {				// удача - скорректировано и 3 компоненты
			pre_l = 64;		// пре-данные - 64 байт
			memcpy(p, rs_data, 64);	// копируем скорректированные данные во входной буфер
			memcpy(p + 64, p_data, 96);
		}
	}
	else {					// удача - скорректировано и 1 компонента
		memcpy(p, rs_data, 58);	// копируем скорректированные данные во входной буфер
		memcpy(p + 58, p_data, 96);
		pre_l = 58;			// пре-данные - 62 байта
	};

	// проверяем, не выходит ли блок EPB с данными за границу входного буфера
	epb_start = p + pre_l - EPB_LN - 2; // адрес начала блока EPB
	epb_len = ((unsigned short)*(epb_start + 2)) << 8 | (unsigned short)*(epb_start + 3); // длина сегмента EPB
	prot_l = ((unsigned long)*(epb_start + 5)) << 24 | ((unsigned long)*(epb_start + 6)) << 16 | ((unsigned long)*(epb_start + 7)) << 8 |
		((unsigned long)*(epb_start + 8));		// длина защищенных данных
	if (prot_l + epb_len - EPB_LN > in_len) // длина защищенных данных + сегмента EPB больше длины входного буфера
		// входной буфер содержит не весь заголовок
		return(-7);

	// пытаемся скорректировать пост-данные блока EPB заголовка
	data_start = epb_start + epb_len + 2; // адрес начала данных: адрес начала EPB + длина EPB + маркер EPB
	postEPB_correct(epb_start, data_start, pre_l); // попытка коррекции всех фрагментов данных
	if (badparts_count > 0)				// один или несколько фрагментов не корректируются
		return(-1);						// осн. заголовок невосстановим

	// основной заголовок восстановлен - собираем данные о сегментах маркеров jpwl и заносим
	// их в массив mark_mas
	// сегмент EPB
	mark_mas[0].id = EPB_MARKER;		// значение маркера
	e = &mark_mas[0].m.epb;
	e->latest = _true_;					// последний в заголовке
	e->index = 0;						// индекс = 0
	e->hprot = decode_Pepb((unsigned long*)(epb_start + 9), 0);	// метод защиты пост-данных
	e->k_pre = 64;				// параметры защиты пре-данных
	e->n_pre = 160;
	e->pre_len = pre_l;			// длина пре-данных
	e->post_len = prot_l - pre_l;	// длина пост-данных
	// параметры защиты пост-данных
	if (e->hprot == 160) {			// RS(160,64)
		e->k_post = 64;
		e->n_post = 160;
	}
	else if (e->hprot == 80) {		// RS(80,25)
		e->k_post = 25;
		e->n_post = 80;
	}
	else if (e->hprot == 40) {		// RS(40,13)
		e->k_post = 13;
		e->n_post = 40;
	}
	else if (e->hprot >= 32) {			// RS(e->hprot,32)
		e->k_post = 32;
		e->n_post = e->hprot;
	}
	else {				// crc16 или crc 32
		e->k_post = 0;
		e->n_post = 0;
	};
	e->Lepb = epb_len;			// длина сегмента
	mark_mas[0].tile_num = -1;		// основной заголовок
	mark_mas[0].pos_in = pre_l - EPB_LN - 2;	// позиция во входном буфере
	mark_mas[0].len = epb_len;	// длина сегмента
	_ASSERT(mark_count < MAX_MARKERS);
	mark_count++;				// наращиваем счетчик записанных маркеров

	// вычисляем длину основного заголовка
	mh_len = (pre_l - EPB_LN - 2) + (epb_len + 2) + e->post_len;	// длина до EPB + сегмент EPB с маркером + защищенные EPB данные
	// 18.04															// длина пост-данных
	MH_Tile_Len += mh_len - (epb_len + 2);		// Вычисляем длину основного занголовка - сейчас с EPC и ESD
	// 18.04														
	cur_len = mh_len - e->post_len;	// длина разобранной части осн. заголовка
	// проверяем наличие сегмента EPC
	epc_start = epb_start + epb_len + 2;	// EPC может начинаться только непосредственно после EPB
	if (*epc_start != 0xff || *(epc_start + 1) != EPC_LOW) // нет EPC, кодовый поток неправильный!
		return(-2);
	// EPC есть - проверяем контроьную сумму, используя rs_data
	epc_len = ((unsigned short)*(epc_start + 2)) << 8 | (unsigned short)*(epc_start + 3);
	if (((unsigned long)epc_len + 2) >= (mh_len - cur_len))	// неправдоподобная длина EPC
		return(-2);
	// готовим массив для вычисления контр. суммы
/*	*(unsigned long *)rs_data=*(unsigned long *)epc_start; // первые 4 байта
	for(i=6,v=epc_start+6; i<epc_len+2; i++, v++) // копируем остальную часть EPC без Pcrc
		rs_data[i-2]=*v;
*/
//	*(unsigned long *)out_buf=*(unsigned long *)epc_start; // первые 4 байта
	memcpy(out_buf, epc_start, 4);							// первые 4 байта
	memcpy(out_buf + 4, epc_start + 6, epc_len - 4);				// копируем остальную часть EPC без Pcrc
	Pcrc = Crc16(out_buf, epc_len);
	if (Pcrc == (((unsigned short)*(epc_start + 4)) << 8 | ((unsigned short)*(epc_start + 5)))) { // crc сошлась
		DL_value = ((unsigned long)*(epc_start + 6)) << 24 | ((unsigned long)*(epc_start + 7)) << 16 |
			((unsigned long)*(epc_start + 8)) << 8 | (unsigned long)*(epc_start + 9);	// длина код. потока
		// 19.04	if(in_len>DL_value)				// если длина входного буфера больше, чем длина потока, устанавливаем ее равной
		in_len = DL_value;			// правильная длина кодового потока
		Pepc = *(epc_start + 10);				// Pepc
		if ((Pepc & 0x10) != 0)					// есть ESD
			esd_yes = _true_;
		else
			esd_yes = _false_;
		if ((Pepc & 0x40) != 0)				// есть  EPB
			epb_yes = _true_;
		else
			epb_yes = _false_;
		if ((Pepc & 0x20) != 0)				// есть RED
			return(-4);
		if ((Pepc & 0x80) != 0) {				// есть Ammendment
			inf_met = epc_start + EPC_LN + 2;
			id = ((unsigned short)*inf_met) << 8 | (unsigned short)*(inf_met + 1);
			if (id != 0x0200)
				return(-3);		// Не поддерживаемый информативный метод
			else {
				IsAmmendment = _true_;		// есть Ammendment
				tepb_count = ((unsigned short)*(inf_met + 4)) << 8 | (unsigned short)*(inf_met + 5);
				//				tepb_count=(tepb_count-2)/10;		// Кол-во записей о блоках EPB
				tepb_adr = inf_met + 6;					// Адрес записи о первом EPB
			}
		}
	}
	else
		return (-8);
	mark_mas[1].id = EPC_MARKER;		// значение маркера
	ec = &mark_mas[1].m.epc;			// адрес для сокращения
	ec->Lepc = epc_len;				// длина сегмента без маркера
	ec->epb_on = epb_yes;				// параметры EPC
	ec->esd_on = esd_yes;
	ec->red_on = _false_;
	ec->DL = DL_value;
	mark_mas[1].tile_num = -1;			// осн. заголовок
	mark_mas[1].pos_in = epc_start - p;	// смещение отн. начала входного буфера
	mark_mas[1].len = epc_len;		// длина сегмента без маркера
	// 18.04															// длина пост-данных
	MH_Tile_Len -= mark_mas[1].len + 2;		// Вычисляем длину основного занголовка - отнимаем длину EPC
	// 18.04														

	_ASSERT(mark_count < MAX_MARKERS);
	mark_count++;				// наращиваем счетчик записанных маркеров
	cur_len += epc_len + 2;			// длина разобранного участка

	// обработка сегмента ESD ( в осн. заголовке может быть только один)
	esd_start = epc_start + epc_len + 2;	// может находиться только непосредственно за EPC
	if (esd_yes == _true_) {				// ESD должен присутствовать
		if (*esd_start != 0xff || *(esd_start + 1) != ESD_LOW) // но его нет
			return(-5);
	}
	else if (esd_yes == _false_ && DL_value > 0) // ESD не должно быть
		if (*esd_start == 0xff && *(esd_start + 1) == ESD_LOW) // а он есть
			return(-5);
	if (*esd_start == 0xff && *(esd_start + 1) == ESD_LOW) { // ESD есть
		esd_yes = _true_;
		esd_len = ((unsigned short)*(esd_start + 2)) << 8 | (unsigned short)*(esd_start + 3); // длина ESD
		if ((unsigned long)esd_len + 2 > mh_len - cur_len)	// неправдоподобная длина ESD - ESD занимает больше оставшейся части осн. заголовка 
			return(-5);
		// создаем элемент массива mark_mas
		mark_mas[2].id = ESD_MARKER;		// идентификатор
		mark_mas[2].len = mark_mas[2].m.esd.Lesd = esd_len;	// длина сегмента
		mark_mas[2].tile_num = -1;		// осн. заголовок
		mark_mas[2].pos_in = esd_start - p;	// смещение начала ESD отн. начала входного буфера
		// 18.04															// длина пост-данных
		MH_Tile_Len -= mark_mas[2].len + 2;		// Вычисляем длину основного занголовка - отнимаем длину ESD
		// 18.04														

		_ASSERT(mark_count < MAX_MARKERS);
		mark_count++;
	};
	// Если использован Ammendment, выполняем обратную перестановку
	if (IsAmmendment)
		deinterleave_instream();

	/*	if((f=fopen("e:/ResFiles/jdec_out.amd","wb"))==NULL) {
			printf("dec dump not created");
			return;
		};
		for(cc=in_buf,ii=0; ii<in_len; cc++,ii++)
			fputc(*cc,f);
		fclose(f);
	*/
	// поиск первого тайла
	v = p + mh_len;			// адрес первого тайла после осн. заголовка
	_ASSERT(v - in_buf < in_len);
	v = tile_detect(v);	// распознаем очередной тайл
	//	if(v==NULL)
	//		return(-6);			// не распознано ни одного тайла
	//	else
	*bb = v;				// адрес первого тайла, у которого корректируютя пре-данные первого EPB или NULL при отсутствии тайлов
	return(0);
}

/**
 * \brief Коррекция пре-данных всей группы EPB в заголовке тайла
 * \param p  Адрес первого байта тайла (у него пре-данные первого EPB уже скорректированы)
 * \param  data_offset В него заносится смещение первого байта пост-данных относительно начала тайла
 * \return 0 - все нормально скорректировалось, -1 - есть некорректируемые EPB
 */
int tile_preEPB_correct(unsigned char* p, unsigned long* data_offset)
{
	unsigned char* v;
	epb_ms* e;
	unsigned short epb_l;
	unsigned short i;
	unsigned long LDPepb;

	*data_offset = SOT_LN + 2;				// длина сегмента SOT + маркер
	v = p + *data_offset;				// адрес первого EPB
	for (i = 0; i >= 0; i++) { // цикл по блокам EPB в заголовке тайла
		// поскольку заголовок EPB уже скорректирован, создаем запись о нем в mark_mas
		mark_mas[mark_count].id = EPB_MARKER; // идентификатор маркера
		e = &mark_mas[mark_count].m.epb;			// адрес для сокращения записи
		epb_l = (((unsigned short)*(v + 2)) << 8) | ((unsigned short)*(v + 3)); // длина сегмента EPB
		mark_mas[mark_count].len = e->Lepb = epb_l;
		*data_offset += epb_l + 2;				// наращиваем на длину сегмента EPB + маркер
		mark_mas[mark_count].pos_in = v - in_buf;	// смещение отн. начала входного буфера
		_ASSERT(mark_count < MAX_MARKERS);
		mark_mas[mark_count++].tile_num = tile_count;	// индекс тайла
		e->index = (unsigned char)(i % 64);				// индекс блока EPB в заголовке
		e->hprot = decode_Pepb((unsigned long*)(v + 9), i == 0 ? 1 : 2);			// метод защиты пост-данных
		e->n_pre = (i == 0 ? 80 : 40);				// параметры RS-кодов для пре-данных
		e->k_pre = (i == 0 ? 25 : 13);
		e->pre_len = (i == 0 ? SOT_LN + 2 : 0) + EPB_LN + 2;	// длина пре-данных: сегменты SOT и EPB + 2 маркера
		LDPepb = ((unsigned long)*(v + 5)) << 24 | ((unsigned long)*(v + 6)) << 16 | ((unsigned long)*(v + 7)) << 8 |
			(unsigned long)*(v + 8);			// длина защищаемых данных
		e->post_len = LDPepb - e->pre_len;		// длина пост-данных
		switch (e->hprot) {
		case 16:							// для crc нет параметров RS-кодов
		case 32: 	e->n_post = e->k_post = 0;
			break;
		case 160:	e->n_post = 160; e->k_post = 64;	// параметры RS-кодов для пост-данных
			break;
		case 80:	e->n_post = 80; e->k_post = 25;
			break;
		case 40:	e->n_post = 40; e->k_post = 13;
			break;
		default:	e->n_post = e->hprot; e->k_post = 32;
		};
		e->latest = (*(v + 4) & 0x40) == 0 ? _false_ : _true_;	// последний или нет в заголовке
		if (e->latest == _true_)					// обработан последний EPB
			break;							// выход из цикла
		v += epb_l + 2;							// переходим к адресу следующего EPB
		if (old_rs_mode != 40) {				// Последник код не RS(40,13)
			init_rs(40, 13);					// Инициализируем RS(40,13)
			old_rs_mode = 40;					// запоминаем его
		};
		if (decode_rs(v, v + 13, 40, 13) < 0)		// заголовок EPB не корректируется
			return(-1);
	};
	return(0);								// все скорректировалось
}



/**
 * \brief Коррекция тайла
 * \param p  Адрес первого байта тайла во входном буфере (у этого тайла уже скорректированы пре-данные первого EPB, т.е. сегмент SOT - правильный
 * \return Адрес первого байта следующего тайла, у которого скорректировались пре-данные первого EPB, или NULL, если такого тайла нет
 */
unsigned char* tile_correct(unsigned char* p)
{
	unsigned long sot_l, d_off, first_interval, j, l, red_posin, r, tilemark_ln, sot_l_new;
	unsigned short mark_count_old, i;
	unsigned char* v, * u, * w;
	_bool_ tile_red, lastblock_corrected, lastblock_red;
	red_ms* rs;
	int err_c, rr;

	tilemark_ln = 0;						// длина всех удаляемых из заголовка тайла сегментов
	first_interval = interv_count;		// запоминаем номер первого RED интервала
	tile_red = _false_;						// пока нет RED интервалов
	sot_l = ((unsigned long)*(p + 6)) << 24 | ((unsigned long)*(p + 7)) << 16 | ((unsigned long)*(p + 8)) << 8 |
		(unsigned long)*(p + 9);			// извлекаем длину тайла
	mark_count_old = mark_count;			// запоминаем значение счетчика маркеров для возможного отката массива mark_mas
	err_c = tile_preEPB_correct(p, &d_off); // коррекция пре-данных группы EPB блоков заголовка тайла
	red_posin = d_off;					// позиция RED во входном буфере - начало данных
	if (err_c < 0) {		// в группе EPB в заголовке тайла есть некорретируемые пре-данные - отбрасываем тайл
		// и создаем для него bad блок
		IsBadBlocks = _true_;
		mark_count = mark_count_old;		// откат счетчика маркеров
		mark_mas[mark_count].id = BAD_ID;	// создаем bad блок размером с тайл
		bad_block_size += sot_l;
		mark_mas[mark_count].len = mark_mas[mark_count].m.bad.Lbad = sot_l - 2; // длина bad блока
		mark_mas[mark_count].tile_num = tile_count++;	// BAD-блок нумеруется как тайл	
		_ASSERT(mark_count < MAX_MARKERS);
		mark_mas[mark_count++].pos_in = p - in_buf;		// позиция блока - начало тайла
	}
	else {								// пре-данные всех EPB заголовка тайла скорректировались - можно разбирать пост-данные
		w = u = p + d_off;						// адрес первого байта пост-данных
		postEPB_correct(in_buf + mark_mas[mark_count_old].pos_in, u, 25); // коррекция пост-данных первого EPB
		if (badparts_count > 0) {		// пост-данные первого EPB не скорректировались,  есть неустранимые ошибки в заголовке тайла
			// отбрасываем тайл и создаем для него bad блок
			IsBadBlocks = _true_;
			mark_count = mark_count_old;		// откат счетчика маркеров
			mark_mas[mark_count].id = BAD_ID;	// создаем bad блок размером с тайл
			bad_block_size += sot_l;
			mark_mas[mark_count].len = mark_mas[mark_count].m.bad.Lbad = sot_l - 2; // длина bad блока
			mark_mas[mark_count].tile_num = tile_count++;	// BAD-блок нумеруется как тайл	
			_ASSERT(mark_count < MAX_MARKERS);
			mark_mas[mark_count++].pos_in = p - in_buf;	// позиция блока - начало тайла
		}
		else {					// весь заголовок тайла скорректирован, приступаем к данным
			v = p + SOT_LN + 2 + 5;		// адрес LDPepb первого EPB: сегмент SOT+маркер SOT+смещение LDPepb отн. маркера EPB
			tilemark_ln += mark_mas[mark_count_old].len + 2;	// добавляем длину сегмента первого EPB
			//			LDPepb=((unsigned long)*v)<<24 | ((unsigned long)*(v+1))<<16 | ((unsigned long)*(v+2))<<8 |
			//				(unsigned long)*(v+3)-25;	// извлекаем LDPepb и вычисляем размер по
			u += mark_mas[mark_count_old].m.epb.post_len;	// вычисляем адрес начала защищенных данных следующего EPB
			lastblock_corrected = _false_;	// последний блок данных не был безошибочным (т.к. для первого блока рнет предыдущего)
			lastblock_red = _false_;		// последний блок данных не был RED-блоком
			for (i = mark_count_old + 1; i < mark_count; i++) { // обработка всех последующих EPB, защищающих данные
				tilemark_ln += mark_mas[i].len + 2;		// добавляем длину сегмента очередного EPB
				postEPB_correct(in_buf + mark_mas[i].pos_in, u, 13); // коррекция данных
				switch (mark_mas[i].m.epb.hprot) {			// вычисляем длину блока пост-данных
				case 16:									// crc-16 или crc-32 - все данные
				case 32:	l = mark_mas[i].m.epb.post_len;
					break;
				case 160:	l = 64;						// для RS-кодов - длина кодируемого фрагмента
					break;
				case 80:	l = 25;
					break;
				case 40:	l = 13;
					break;
				default:	l = 32;
				};
				d_off = u - p;						// смещение начала пост-данных EPB отн. начала тайла
				if (badparts_count > 0) { 	// есть некорректируемые фрагменты - создание таблицы для иаркера RED 
					tile_red = _true_;					// устанавливаем флаг наличия RED интервалов
					if (badparts[0] > 0) {			// первый интервал без ошибок - создаем его и начало RED-интервала
						if (lastblock_corrected == _true_) // последний интервал предыдущего EPB был безошибочным - объединяем интервалы
							r_int[interv_count - 1].end = d_off + badparts[0] * l - 1; // смещение конца безошиб. интервала
						else {							// создаем новый безошибочный интервал
							r_int[interv_count].start = d_off;				// смещение начала безошиб. интервала
							r_int[interv_count].end = d_off + badparts[0] * l - 1; // смещение конца безошиб. интервала
							r_int[interv_count++].errors = 0x0000;				// нет ошибок
						};
						r_int[interv_count].start = r_int[interv_count - 1].end + 1;	// начало RED-интервала
					}
					else						// первый интервал ошибочный - создаем начало RED-интервала
						if (lastblock_red == _false_)		// последний интервал предыдущего EPB был безошибочным- создаем новый интервал
							r_int[interv_count].start = d_off + badparts[0] * l; // смещение начала red-интервала отн. начала тайла
						else							// в противном случае RED-интервал продолжает последний
							interv_count--;				// устанавливаем индекс на последний записанный интервал
					for (j = 1; j < badparts_count; j++)		// цикл по red-блокам
						if (badparts[j] != badparts[j - 1] + 1) { // не подряд идущий участок - завершаем предыдущий RED- интервал,
							// создаем безошибочный интервал и начинаем новый RED-интервал
							r_int[interv_count].errors = 0xFFFF;	// неопределенное кол-во ошибок
							r_int[interv_count++].end = d_off + (badparts[j - 1] + 1) * l - 1; // конец предыдущего интервала
							r_int[interv_count].start = r_int[interv_count - 1].end + 1;	// начало безошибочного
							r_int[interv_count].end = d_off + badparts[j] * l - 1;			// конец безошибочного
							r_int[interv_count++].errors = 0x0000;			// нет ошибок
							r_int[interv_count].start = r_int[interv_count - 1].end + 1;	// начало след. RED-интервала
						};
					// обработка конца последнего RED-интервала
					r_int[interv_count].errors = 0xFFFF;				// неопределенное кол-во ошибок
					r = (badparts[j - 1] + 1) * l;				// предполагаемый конец RED-интервала отн. начала пост-данных
					if (r >= mark_mas[i].m.epb.post_len) { // предполагаемый конец больше длины пост-данных
						// это возможно, если RED-интервал последний длиной меньше l байт
						r_int[interv_count++].end = d_off + mark_mas[i].m.epb.post_len - 1; // конец интервала
						// равен концу пост-данных
						lastblock_corrected = _false_;	// последний блок данных не был безошибочным 
						lastblock_red = _true_;		// последний блок данных не RED-блоком
					}
					else {						// имеется последний безошибочный интервал 
						r_int[interv_count++].end = d_off + (badparts[j - 1] + 1) * l - 1; // конец последнего RED-интервала
						// равен концу badparts[j-1]+1-го участка по l байт
		// создаем безошибочный интервал
						r_int[interv_count].start = r_int[interv_count - 1].end + 1;	// начало
						r_int[interv_count].end = d_off + mark_mas[i].m.epb.post_len - 1; // конец - последний байт пост-данных EPB 
						r_int[interv_count++].errors = 0x0000;			// ошибок нет
						lastblock_corrected = _true_;	// последний блок данных был безошибочным 
						lastblock_red = _false_;		// последний блок данных не был RED-блоком
					}
				}
				else {			// все фрагменты скорректировались - создаем безошибочный интервал
					if (lastblock_corrected == _true_) // до этого уже был безошибочный, объединяем их
						r_int[interv_count - 1].end = d_off + mark_mas[i].m.epb.post_len - 1; // смещение конца интервала
					else {						// создаем новый
						r_int[interv_count].start = d_off;				// смещение начала интервала
						r_int[interv_count].end = d_off + mark_mas[i].m.epb.post_len - 1; // смещение конца интервала
						r_int[interv_count++].errors = 0x0000;				// нет ошибок
					};
					lastblock_corrected = _true_;	// последний блок данных был безошибочным 
					lastblock_red = _false_;		// последний блок данных не был RED-блоком
				};
				//				v=in_buf+mark_mas[i].pos_in+5;						// адрес LDPepb
				u += mark_mas[i].m.epb.post_len;	// вычисляем адрес начала защищенных данных следующего EPB
			};
			// разбор маркеров ESD
			while (*w == 0xff && *(w + 1) == ESD_LOW) { // обработка очередного маркера ESD
				mark_mas[mark_count].id = ESD_MARKER;	// ид. маркера
				mark_mas[mark_count].len = mark_mas[mark_count].m.esd.Lesd = ((unsigned short)*(w + 2)) << 8 |
					(unsigned short)*(w + 3);			// длина сегмента без маркера
				tilemark_ln += mark_mas[mark_count].len + 2;	// добавляем длину сегмента ESD
				mark_mas[mark_count].tile_num = tile_count; //  индекс разобранного тайла
				mark_mas[mark_count].pos_in = w - in_buf;		// позиция во входном буфере
				_ASSERT(mark_count < MAX_MARKERS);
				w += mark_mas[mark_count++].len + 2;		// переводим адрес на потенциально следующий ESD
			};

			if (tile_red == _true_)				// были RED-интервалы в тайле
				tile_red_restored_count++;		// Инкремент частично восстановленных тайлов
			else
				tile_all_restored_count++;		// Инкремент полностью восстановленных тайлов
			// обработка маркера RED
			if (tile_red == _true_ && use_red == _true_) {		// были ошибки данных и установлен режим использовать RED,
				// вставка маркера RED в mark_mas
				red_yes = _true_;							// в потоке будут маркеры RED
				if (interv_count - first_interval > MAX_INTERVALS) {	// кол-во интервалов больше, чем можно поместить в сегмент RED
					// сливаем все интервалы с 6553 по последний в один RED интервал
					r_int[first_interval + MAX_INTERVALS].end = r_int[interv_count - 1].end; // конец последнего интервала
					r_int[interv_count - 1].errors = 0xFFFF;					// есть остаточные ошибки
					interv_count = first_interval + MAX_INTERVALS;				// уменьшаем кол-во интервалов до 6553
				};
				mark_mas[mark_count].id = RED_MARKER;		// идентификатор маркера
				rs = &mark_mas[mark_count].m.red;			// адрес для сокращения записи
				mark_mas[mark_count].len = rs->Lred = (unsigned short)(interv_count - first_interval) * 10 + RED_LN; // длина интервалов + параметры сегмента
				tilemark_ln -= mark_mas[mark_count].len + 2;		// вычитаем длину добавляемого сегмента RED
				rs->start = first_interval;							// номер первого интервала
				rs->count = interv_count - first_interval;				// кол-во интервалов
				rs->Pred = 0x7B;				// байтовый диапазон, уровень ошибки не определен (111), 4 байта на адрес, есть ошибки
				mark_mas[mark_count].tile_num = tile_count;	// индекс тайла
				_ASSERT(mark_count < MAX_MARKERS);
				mark_mas[mark_count++].pos_in = red_posin;	// позиция во вх. буфере - начало данных тайла
				for (i = first_interval; i < interv_count; i++) { // цикл по созданным интервалам

					r_int[i].start -= tilemark_ln;			// уменьшаем начала и концы интервалов на кол-во удаляемых из заголовка
					r_int[i].end -= tilemark_ln;			// тайла байт
				};
			}
			else		// REd не используется или не было ошибок, т.е. имеется только безошибочный интервал
				interv_count = first_interval;			// выполняем откат счетчика интервалов и не создаем маркер RED
			tile_count++;
			sot_l_new = sot_l - tilemark_ln;				// вычисляем новую длину тайла, которая будет после удаления сегментов
			// 18.04															// длина пост-данных
			MH_Tile_Len += sot_l_new;
			// 18.04
														// EPB и ESD и добавления сегмента RED
			*(p + 6) = (unsigned char)(sot_l_new >> 24);	// заносим новую длину в сегмент SOT во входной буфер в прямом порядке байт
			*(p + 7) = (unsigned char)(sot_l_new >> 16);
			*(p + 8) = (unsigned char)(sot_l_new >> 8);
			*(p + 9) = (unsigned char)sot_l_new;
		}
	};
	// ищем следующий тайл, у которого корректируютя пре-данные первого EPB
	p += sot_l;				// адрес начала следующего тайла
	rr = in_len - (p - in_buf);	// кол-во байт до конца входного буфера
	//	_ASSERT(p-in_buf<in_len);
	if (rr < 81) {	// в оставшейся части вх. буфера не может поместиться тайл
		/*		if(rr==2)	// остался только маркер EOC
					return(NULL);
				else if(rr>2) {		// остался неопределенный участок
					IsBadBlocks=_true_;
					mark_mas[mark_count].id=BAD_ID;	// создаем bad блок размером с тайл
					mark_mas[mark_count].len=rr-4; // длина bad блока
					mark_mas[mark_count].tile_num=tile_count++;	// BAD-блок нумеруется как тайл
					mark_mas[mark_count++].pos_in=p-in_buf;	// позиция блока - начало тайла
				}
				else			// невообразимая ситуация - указатель за позицией EOC
					return(NULL);
		*/
		if (rr > 2) {		// остался неопределенный участок
			IsBadBlocks = _true_;
			mark_mas[mark_count].id = BAD_ID;	// создаем bad блок размером с тайл
			bad_block_size += rr - 2;
			mark_mas[mark_count].len = rr - 4; // длина bad блока
			mark_mas[mark_count].tile_num = tile_count++;	// BAD-блок нумеруется как тайл	
			_ASSERT(mark_count < MAX_MARKERS);
			mark_mas[mark_count++].pos_in = p - in_buf;	// позиция блока - начало тайла
		};
		return(NULL);
	}
	else {			// в оставшейся части есть место для тайла
		u = p;					// адрес начала поиска
		_ASSERT(p - in_buf < in_len);
		p = tile_detect(p);		// распознаем следующий тайл
		if (p == NULL) {			// но тайл не найден - создаем bad-блок
			IsBadBlocks = _true_;
			mark_mas[mark_count].id = BAD_ID;		// создаем bad блок размером с тайл
			bad_block_size += rr - 2;
			mark_mas[mark_count].len = rr - 4;		// длина bad блока
			mark_mas[mark_count].tile_num = tile_count++;	// BAD-блок нумеруется как тайл	
			_ASSERT(mark_count < MAX_MARKERS);
			mark_mas[mark_count++].pos_in = u - in_buf;		// позиция блока - начало тайла
		};
		return(p);
	}
}

/**
 * \brief Копирование данных в выходной буфер
 * \details Выполняет копирование скорректированных данных в выходной буфер.
 * При этом из кодового потока удаляются сегменты маркеров EPB, EPC и ESD.
 * Если есть остаточные ошибки и установлен режим использования сегмента RED,
 * в поток добавляются сегменты EPC и RED. Сегмент RED добавляется в заголовок тайлов.
 * \return Длина выходного буфера
 */
unsigned long data_copy()
{
	int i, j, k, epc_pos, t_no, intr_no;
	unsigned char epc_data[EPC_LN], * p;
	unsigned short epc_crc;

	out_len = 0;		// обнуляем длину выходного буфера
	t_no = -2;		// начальный индекс заголовка
	for (i = j = 0; i < mark_count; i++) {	// цикл по таблице маркеров
		if (mark_mas[i].tile_num != t_no) {	// первый маркер очередного заголовка
			t_no = mark_mas[i].tile_num;	// запомним индекс этого заголовка
			for (; j < mark_mas[i].pos_in; j++) // копируем данные, предшествующие найденному маркеру
				out_buf[out_len++] = in_buf[j]; // в выходной буфер
		};
		if (mark_mas[i].id == EPC_MARKER && red_yes == _true_) {	// готовим к выводу в поток сегмент EPC
			epc_pos = out_len;				// позиция EPC в выходном буфере
			out_len += EPC_LN + 2;				// пропускаем место для EPC в вых. буфере
			// 18.04															// длина пост-данных
			MH_Tile_Len += EPC_LN + 2;			// Добавляем к общей длине длину EPC
			// 18.04														

			j += mark_mas[i].len + 2;			// пропускаем сегмент во входном буфере
			mark_mas[i].len = EPC_LN;			// длина EPC
			// crc и DL будут добавлены последними
//			mark_mas[i].m.epc.Pepc=0x20;	// присутствуют RED
		}
		else if (mark_mas[i].id == RED_MARKER && red_yes == _true_) { // выводим в поток маркер RED
			out_buf[out_len++] = 0xFF;		// сам маркер
			out_buf[out_len++] = RED_LOW;
			out_buf[out_len++] = (unsigned char)(mark_mas[i].len >> 8);	// длина сегмента
			out_buf[out_len++] = (unsigned char)mark_mas[i].len;
			out_buf[out_len++] = mark_mas[i].m.red.Pred;			// параметры RED
			for (k = 0; k < mark_mas[i].m.red.count; k++) {			// вывод данных RED
				intr_no = mark_mas[i].m.red.start + k;			// номер интервала
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].start >> 24);	// начало интервала
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].start >> 16);
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].start >> 8);
				out_buf[out_len++] = (unsigned char)r_int[intr_no].start;
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].end >> 24);	// конец интервала
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].end >> 16);
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].end >> 8);
				out_buf[out_len++] = (unsigned char)r_int[intr_no].end;
				out_buf[out_len++] = (unsigned char)(r_int[intr_no].errors >> 8);	// уровень ошибки
				out_buf[out_len++] = (unsigned char)r_int[intr_no].errors;
			}
		}
		else				// во всех остальных случаях пропускаем сегмент во входногм буфере
			j += mark_mas[i].len + 2;
	};
	if (j < in_len)			// остались нескопированные данные во вхолдном буфере
		for (; j < in_len; j++)
			out_buf[out_len++] = in_buf[j];
	// 18.04
	if (MH_Tile_Len > out_len) {	// Сумма длин основного заголовка и всех тайлов больше расчетной длины кодового потока
		// Это возможно при обрезанном в конце кодовом потоке
		out_len = MH_Tile_Len + 2;	// Увеличиваем длину выходного потока
		out_buf[out_len - 2] = 0xFF;	// Вставляем потерянный маркер конца кодового потока
		out_buf[out_len - 1] = EOC_LOW;
	};
	if (!(out_buf[out_len - 2] == 0xFF && out_buf[out_len - 1] == EOC_LOW)) {	// Пропущен маркер конца EOC
		// Это возможно при обрезанном входном потоке
		out_buf[out_len - 2] = 0xFF;							// Добавляем к концу EOC
		out_buf[out_len - 1] = EOC_LOW;
	};
	// 18.04
	if (red_yes == _true_) {			// окончательно формируем и выводим EPC
		// сначала данные для вычисления контрольной суммы
		epc_data[0] = 0xff;
		epc_data[1] = EPC_LOW;
		epc_data[2] = (unsigned char)(mark_mas[1].len << 8);
		epc_data[3] = (unsigned char)mark_mas[1].len;
		epc_data[4] = (unsigned char)(out_len << 24);
		epc_data[5] = (unsigned char)(out_len << 16);
		epc_data[6] = (unsigned char)(out_len << 8);
		epc_data[7] = (unsigned char)out_len;
		epc_data[8] = 0x20;						// присутствуют RED
		epc_crc = Crc16(epc_data, EPC_LN);			// контрольная сумма
		p = out_buf + epc_pos;						// вывод EPC в выходной буфер
		*(unsigned long*)p = *(unsigned long*)&epc_data[0];
		p += 4;
		*p++ = (unsigned char)(epc_crc << 8);
		*p++ = (unsigned char)out_len;
		*(unsigned long*)p = *(unsigned long*)(epc_data + 4);
		p += 4;
		*p = epc_data[8];
	};
	return(out_len);
}

/**
 * \brief Декодер jpwl
 * \details Выполняет коррекцию данных входного буфера в соответствии со средствами защиты,
 * предусмотренными во входном потоке.
 * Коды завершения:
 * 1 - в кодовом потоке нет средств jpwl, коррекция не нужна
 * 0 - основной заголовок восстановлен, кодовый поток несет данные об изображении, кадр следует отобразить
 * -1 - основной заголовок не восстановлен, кадр изображения следует отбросить
 * -2 - нет ни одного тайла с данными, кадр изображения следует отбросить
 * \param i_b  Адрес входного буфера, в котором расположен кодовый поток jpeg2000 часть 2, подвергшийся воздействию ошибок в канале передачи данных
 * \param i_l  Длина данных во входном буфере в байтах
 * \param o_b  Адрес выходного буфера, в который следует записать скорректированный кодовый поток jpeg2000, возможно с внедренными маркерами EPC и RED(остаточная ошибка)
 * \param o_l  Адрес переменной, в которую будет записана длина данных (в байтах) выходного буфера
 * \return Код завершения (см. детали)
 */

int w_decoder(unsigned char* i_b, unsigned long i_l, unsigned char* o_b,
	unsigned long* o_l)
{
	unsigned char* p;
	int i;

	in_buf = p = i_b;				// присваиваем значения входных параметров внешним переменным
	in_len = i_l;					// чтобы не пользоваться лишний раз формальными параметрами, а
	end_buf = in_buf - in_len;		// адрес последнего байта кодового потока во входном буфере
	out_buf = o_b;				// p-движущийся указатель устанавлмваем на начало буфера
	tile_count = 0;				// обнуляем счетчик успешно скорректированных тайлов
	mark_count = 0;				// обнуляем счетчик обнаруженных маркеров
	interv_count = 0;				// обнуляем счетчик интервалов
	old_rs_mode = 0;				// RS-код еще не инициализирован
	// 18.04
	MH_Tile_Len = 0;				// Обнуление суммы длин основного заголовка и тайлов, копируемых в выходной буфер
	// 18.04
	bad_block_size = tile_all_restored_count = tile_red_restored_count = 0;	// Обнуление статистики корекции тайлов
	i = mh_correct(&p);			// коррекция основного заголовка
	red_yes = _false_;			// нет маркеров RED
	IsBadBlocks = _false_;		// нет BAD блоков
	if (i < 0) {					// основной заголовок не корректируется
#ifdef _TEST
		//		printf("mh_correct_code=%d\n",i);
#endif
		return(-1);
	}
	else if (i > 0) {				// средства jpwl не обнаружены в кодовом потоке - коррекция не производится
		*o_l = i_l;				// длина вых. буфера равна длине входного
		_ASSERT(i_l > 0 && i_l <= 16000000l);
		memcpy(o_b, i_b, i_l);	// копируем входной буфер в выходной
		return(1);
	};
	/*	if(p==NULL)					// все тайлы отброшены
			return(-2);
		while((p=tile_correct(p))!=NULL);	//последовательная коррекция тайлов
		if(tile_count==0)			// если  не был разобран ни один тайл
			return(-2);
		else {						// были разобраны тайлы
			*o_l=data_copy();
			return(0);
		}
	*/
	//	if(p!=NULL)					// не все тайлы отброшены
	//		while((p=tile_correct(p))!=NULL);	//последовательная коррекция тайлов
	while (p != NULL) {
		p = tile_correct(p);
	};
	*o_l = data_copy();
	return(0);
}

/**
 * \brief  Вызов декодера jpwl
 * \details Коды завершения:
 * 1 - в кодовом потоке нет средств jpwl, коррекция не нужна
 * 0 - основной заголовок восстановлен, кодовый поток несет данные об изображении, кадр следует отобразить
 * -1 - основной заголовок не восстановлен, кадр изображения следует отбросить
 * -2 - нет ни одного тайла с данными, кадр изображения следует отбросить
 * \param w_par  Структура типа w_dec_params, содержащая параметры работы декодера
 * \param o_l  По адресу, содержащемуся в o_l записывается длина сформированного кодового потока
 * \return Код завершения, возвращаемый декодером (см. детали)
 */
int w_decoder_call(w_dec_params* w_par)
{
	int i;
	unsigned long o_l;

	use_red = w_par->use_red;
	i = w_decoder(w_par->inp_buffer, w_par->inp_length, w_par->out_buffer, &o_l);
	if (i == 0)
		w_par->out_length = o_l;
	return i;
}

/**
 * \brief  Задание параметров по умолчанию для декодера jpwl
 * \param w_par  Адрес структуры с параметрами инициализации декодера jpwl
 * \return Код завершения: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void jpwl_dec_set_default_params(jpwl_dec_params* w_par)
{
	w_par->use_red = _true_;
}

/**
 * \brief Инициализация декодера jpwl
 * \param w_par  Адрес структуры с параметрами инициализации декодера jpwl
 * \return Код завершения: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void jpwl_dec_init(jpwl_dec_params* w_par)
{

	dec_par.use_red = w_par->use_red;
	FullRestoredCadrs = PartiallyRestoredCadrs = NonRestoredCadrs = NonJPWLCadrs = 0;	// Обнуление статистики коррекции кадров
}

/**
 * \brief Запуск декодера jpwl
 * \param w_par  Адрес структуры с параметрами инициализации декодера jpwl
 * \return Код завершения: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
errno_t jpwl_dec_run(jpwl_dec_bParams* b_par, jpwl_dec_bResults* b_res)
{
	int i_res;

	if (b_par->inp_length == 0) {
		b_res->out_length = 0;
		return 0;
	};
	dec_par.inp_buffer = b_par->inp_buffer;		// Формируем структуру dec_par
	dec_par.inp_length = b_par->inp_length;
	dec_par.out_buffer = b_par->out_buffer;
	// Обнуляем статистику
	bad_block_size = 0;
	tile_all_restored_count = 0;
	tile_red_restored_count = 0;
	i_res = w_decoder_call(&dec_par);			// Запуск декодера
	if (i_res == 1) {
		//		memcpy(dec_par.out_buffer,dec_par.inp_buffer,dec_par.inp_length);
		NonJPWLCadrs++;
		b_res->out_length = dec_par.inp_length;
	}
	else if (i_res == 0) {
		if (IsBadBlocks == _true_ || red_yes == _true_)
			PartiallyRestoredCadrs++;
		else
			FullRestoredCadrs++;
		b_res->out_length = dec_par.out_length;
	}
	else {
		NonRestoredCadrs++;
		b_res->out_length = 0;
		bad_block_size = b_par->inp_length;
	};
	b_res->all_bad_length = bad_block_size;		// Заполнение статистики декодирования тайлов
	b_res->tile_all_rest_cnt = tile_all_restored_count;
	b_res->tile_part_rest_cnt = tile_red_restored_count;
	return 0;
}

/**
 * \brief Очистка библиотеки декодера jpwl
 * \return Код завершения: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void jpwl_destroy()
{
}
__declspec(dllexport)
errno_t jpwl_init() { return 0; }
/**
 * \brief Запуск декодера jpwl
 * \param w_par  Адрес структуры с параметрами инициализации декодера jpwl
 * \return Код завершения: 1 - все нормально, 0 - критическая ошибка, стоп
 */
__declspec(dllexport)
void jpwl_dec_stats(restore_stats* Stat)
{
	Stat->fully_restored = FullRestoredCadrs;
	Stat->not_JPWL = NonJPWLCadrs;
	Stat->not_restored = NonRestoredCadrs;
	Stat->partially_restored = PartiallyRestoredCadrs;
}
