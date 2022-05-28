#include <memory.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "crc.h"
#include "jpwl_params.h"
#include "jpwl_types.h"

#ifdef RS_OPTIMIZED
#include "rs64/rs64.h"
#else
#include "..\rs_crc_lib\rs_crc_decl.h"
#include "..\rs_crc_lib\rs_crc_import.h"
#endif // RS_OPTIMIZED

unsigned char* in_buf;		///< Адрес входного буфера
unsigned long in_len;		///< Количествово байт во входном буфере
unsigned char* out_buf;		///< Адрес выходного буфера
unsigned short tile_count;	///< Счетчик успешно скорректированных тайлов
unsigned char rs_data[256];	///< Буфер для неполно заполненных корректируемых RS-кодами данных и для проверки контрольной суммы EPC (для случая внутрикадрового чередования длина буфера должна позволить поместить всю карту EPB)
w_marker dec_markers[MAX_MARKERS]; ///< Массив маркеров jpwl и некорректируемых участков, обнаруженных при декодировании
unsigned short markers_cnt;	///< Счетчик обнаруженных и записанных в dec_markers маркеров
_bool_ esd_used;	///< ESD используется в кодовом потоке?
_bool_ epb_used;	///< EPB используется в кодовом потоке?
unsigned long dec_epc_dl;		///< Значение длины из EPC 
unsigned short old_rs_mode;	///< RS-код, который был проинициализирован последним
unsigned long mh_tile_len;		// Сумма длин основного заголовка и тайлов, копируемых в выходной буфер
_bool_ has_bad_blocks;					///< Обнаружены ли невосстанавливаемые тайлы( _true_, _false_)
_bool_ is_ammendment;				///< Используется ли Ammendment в кодовом потоке ( _true_, _false_)
unsigned short tepb_count;			///< Количество записей в таблице EPB при использовании Ammendment
unsigned char* tepb_adr;			///< Адрес таблицы EPB при использовании Ammendment
unsigned long mh_len;				///< Длина основного заголовка во входном буфере 
unsigned short tile_all_rest_cnt;	///< Количество полностью восстановленных тайлов кадра
unsigned short tile_red_rest_cnt;	///< Количество частично восстановленных тайлов кадра, в которых присутствуют маркеры RED
unsigned long bad_block_length;		 ///< Количество нераспознанных как тайл байт данных
restore_stats stats;

/**
 * \brief определение способа защиты пост-данных блока EPB
 * \param Pepb_value  Fдрес параметра Pepb во входном буфере
 * \param epb_type  Тип блока EPB: 0 - первый в основном заголовке, 1 - первый в заголовке тайла, 2 - не первый в заголовке.
 * \return Код защиты пост-данных
 */
uint16_t decode_Pepb(uint32_t* Pepb_value, uint8_t epb_type)
{
	switch (*Pepb_value) {
	case 0xffffffff: return 0;	// нет защиты
	case 0x00000000: 
		if (epb_type == 0)	// предопределенная в первом EPB основного заголовка
			return 1;
		else if (epb_type == 1)	// предопределенная в первом EPB заголовка тайла
			return 2;
		else					// предопределенная в не первом EPB заголовка
			return 3;
	case 0x00000010: return 16;			// crc-16
	case 0x01000010: return 32;			// crc-32
	case 0x20250020: return 37;			// RS(37,32)
	case 0x20260020: return 38;			// RS(38,32)
	case 0x20280020: return 40;			// RS(40,32)
	case 0x202b0020: return 43;			// RS(43,32)
	case 0x202d0020: return 45;			// RS(45,32)
	case 0x20300020: return 48;			// RS(48,32)
	case 0x20330020: return 51;			// RS(51,32)
	case 0x20350020: return 53;			// RS(53,32)
	case 0x20380020: return 56;			// RS(56,32)
	case 0x20400020: return 64;			// RS(64,32)
	case 0x204b0020: return 75;			// RS(75,32)
	case 0x20500020: return 80;			// RS(80,32)
	case 0x20550020: return 85;			// RS(85,32)
	case 0x20600020: return 96;			// RS(96,32)
	case 0x20700020: return 112;		// RS(112,32)
	case 0x20800020: return 128;		// RS(128,32)
	case 0x20900020: return 144;		// RS(144,32)
	case 0x20A00020: return 161;		// RS(160,32)
	case 0x20B00020: return 176;		// RS(176,32)
	case 0x20C00020: return 192;		// RS(192,32)
	};
	return 666;		// невозможный случай - для обнаружения ошибок
}

/**
 * \brief коррекция пост-данных блока EPB
 * \details  * Выполняет пофрагментную коррекцию пост-данных блока EPB.
 *	присваивает badparts_count количество фрагментов данных, не подлежащих коррекции.
 *	Все фрагиенты, которые могут быть скорректированы, корректирует на месте
 *	\param epb_start  Адрес начала блока EPB во входном буфере, т.е. первого байта маркера
 *	\param postdata_start  Адрес начала пост-данных во входном буфере
 *	\param p_len  Длина пре-данных в байтах
 */
uint32_t postEPB_correct(uint8_t* epb_start, uint8_t* postdata_start, uint32_t p_len)
{
	uint8_t epb_type;
	uint8_t* parity_start;
	uint16_t prot_mode, c16_calculated, c16_expected;
	int n_p, k_p, l, i;
	uint32_t data_len, c32_calculated, c32_expected, badparts_count = 0;

	
	// тип EPB: 0 - первый в осн. заголовке, 1 - первый в заголовке тайла, 2 - не первый в заголовке 
	epb_type = p_len == 13 ? 2 : (p_len == 25 ? 1 : 0);
	prot_mode = decode_Pepb((uint32_t*)(epb_start + 9), epb_type);
	if (prot_mode == 0)
		return 0;
	data_len = _byteswap_ulong(*(uint32_t*)(epb_start + 5));	// LDPepb
	data_len -= p_len;	// длина пост-данных
	if (epb_type == 0)	// начало кодов четности смещено от начала epb на размер сегмента + маркер + коды для пре данных
		parity_start = epb_start + EPB_LN + 2 + 96;
	else if (epb_type == 1)
		parity_start = epb_start + EPB_LN + 2 + 55;
	else
		parity_start = epb_start + EPB_LN + 2 + 27;
	if (prot_mode == 16) {			// crc16
		c16_calculated = CRC16(postdata_start, data_len);
		c16_expected = _byteswap_ushort(*(uint16_t*)parity_start);
		if (c16_calculated != c16_expected)
			return 1;
		return 0;
	};
	if (prot_mode == 32) {	// crc32
		c32_calculated = CRC32(postdata_start, data_len);
		c32_expected = _byteswap_ulong(*(uint32_t*)parity_start);
		if (c32_calculated != c32_expected)
			return 1;
		return 0;
	};
	if (prot_mode == 1) {
		n_p = 160;
		k_p = 64;
	}
	else if (prot_mode == 2) {
		n_p = 80;
		k_p = 25;
	}
	else if (prot_mode == 3) {	
		n_p = 40;
		k_p = 13;
	}
	else {
		n_p = prot_mode;
		k_p = 32;
	};
#ifndef RS_OPTIMIZED
	if (old_rs_mode != n_p) {		// RS-код нужно инициализировать
		init_rs(n_p, k_p);
		old_rs_mode = n_p;
	};
#endif // RS_OPTIMIZED
	for (i = 0, l = data_len; l >= k_p; i++, l -= k_p) {
		int x = decode_RS(postdata_start, parity_start, n_p, k_p);
		if (x < 0) {
			badparts_count++;
			stats.uncorrected_rs_bytes += n_p;
		}
		else
			stats.corrected_rs_bytes += x;
		postdata_start += k_p;			// переходим к следующему блоку данных
		parity_start += ((size_t)n_p - k_p);	// переходим к след. блоку RS-кодов
	};
	if (l > 0) {			// остался последний блок данных длиной менее k_p байт
		memcpy(rs_data, postdata_start, l);
		memset(rs_data + l, 0, 64ULL - l);
		int x = decode_RS(rs_data, parity_start, n_p, k_p);
		if (x < 0) {
			badparts_count++;
			stats.uncorrected_rs_bytes += l;
		}
		else {
			stats.corrected_rs_bytes += x;
			memcpy(postdata_start, rs_data, l);	// если данные скорректировались, изменяем их во входном буфере
		}
	}
	return badparts_count;
}

/**
 * \brief Быстрый поиск очередного тайла в случае потери структуры кодового потока
 * \details   Начиная с указанного места и до конца буфера ищет маркер SOT, предполагая, что он не был изменен,
 * предполагая наличие EPB после сегмента SOT корректируем его пре-данные, если скорректировано еще раз
 * проверяем маркер SOT и возвращаем адрес его начала. Если нескорректировано или маркер SOT после
 * коррекции стал другим маркером, продолжаем поиск
 * \param tile  Адрес байта, с которого нужно начать поиск
 * \return Fдрес байта, с которого начинается найденный тайл, у которого корректируются пре-данные первого EPB, или NULL
 */
uint8_t* dec_tile_search(uint8_t* p)
{
	uint8_t* v;
	uint16_t SOT_be;

	SOT_be = ((uint16_t)SOT_LOW << 8) | 0xFF;
	for (v = p; in_len - (v - in_buf) >= 81; v++) {	// ищем от заданного места до конца буфера минус 80 байт
		// защита пре-данных первого EPB + 1 байт на пост-данные
		if (*(uint16_t*)v == SOT_be) {  // найден SOT
#ifndef RS_OPTIMIZED
			if (old_rs_mode != 80) {
				init_rs(80, 25);
				old_rs_mode = 80;
			};
			if (decode_RS(v, v + 25, 80, 25) >= 0)
#else
			if (decode_RS(v, NULL, 80, 25) >= 0)
#endif // !RS_OPTIMIZED
			{
				if (*(uint16_t*)v == SOT_be) // после коррекции SOT остался
					return v;
			}
		};
	}
	return NULL;
}

/**
 * \brief Попытка распознать тайл
 * \param v Предполагаемый адрес тайла во входном буфере
 * \return  Адрес первого тайла, у которого корректируются пре-данные первого EPB в заголовке тайла. Или NULL
 */
uint8_t* dec_tile_detect(uint8_t* v)
{
	uint8_t* t;

	if (in_len - (v - in_buf) < TILE_MINLENGTH)	// С точки обнаружения тайла недостаточно места для тайла
		return NULL;
	t = v;						// адрес предполагаемого начала тайла
#ifndef RS_OPTIMIZED
	if (old_rs_mode != 80) {	// последний код не RS(80,25)
		init_rs(80, 25);
		old_rs_mode = 80;
	}; 
	// пре-данные первого EPB заголовка тайла не корректируются!
	// or после коррекции на месте нет маркера SOT (невероятно, но все же..)
	if (decode_RS(v, v + 25, 80, 25) < 0 || *v != 0xff || *(v + 1) != SOT_LOW)
#else
	if (decode_RS(v, NULL, 80, 25) < 0 || *v != 0xff || *(v + 1) != SOT_LOW)
#endif // !RS_OPTIMIZED 
	{
		if (v + 80 - in_buf < in_len)
			return NULL;

		v = dec_tile_search(v + 80);
		if (v == NULL || markers_cnt >= MAX_MARKERS)
			return NULL;
		// тайл найден, пропущенный фрагмент заносим в dec_markers как BAD_ID
		has_bad_blocks = _true_;
		dec_markers[markers_cnt].id = BAD_ID;
		bad_block_length += (uint32_t)(v - t);
		dec_markers[markers_cnt].m.bad.Lbad = dec_markers[markers_cnt].len = (uint32_t)(v - t) - 2;
		dec_markers[markers_cnt].tile_num = tile_count++;
		dec_markers[markers_cnt++].pos_in = (uint32_t)(t - in_buf);
	}
	return v;
}

/**
 * \brief Обратная перестановка входного кодового потока
 * \details  Используется в случае применения внутрикадрового чередования на стороне кодера jpwl
 * \return Нет возвращаемого значения
 */
void deinterleave_instream()
{
	uint8_t* c;
	uint16_t Lepb;
	uint32_t Nc, Nr, i, j, k, Len, off;
	uint32_t wait_epb, sot_start, sot_start_old, PSot;

	Len = dec_epc_dl - mh_len;		// Длина переставляемых данных: общая длина минус основной заголовок
	Nc = (uint32_t)ceil(sqrt((double)Len));	// Количество столбцов
	Nr = (uint32_t)ceil(((double)Len / Nc));			// Количество строк
	k = 0;
	c = out_buf;
	for (j = 0; j < Nc; j++) {
		for (i = 0; i < Nr; i++) {
			*c++ = in_buf[mh_len + i * Nc + j];
			if (++k == Len)
				goto mcop;			// Все переставлено переход к копированию
		}
	}

mcop:
	memcpy(in_buf + mh_len, out_buf, Len);
	in_len = dec_epc_dl;
	// Восстановление маркеров EPB и SOT и фрагментов их сегментов на основе таблицы EPB
	wait_epb = sot_start = 0;
	for (i = 0; i < tepb_count; i++, tepb_adr += 10) {
		off = _byteswap_ulong(*(uint32_t*)(tepb_adr + 6));	// Смещение EPB относительно начала потока
		in_buf[off] = 0xFF;				// Восстанавливаенм маркер EPB
		in_buf[off + 1] = EPB_LOW;
		memcpy(in_buf + off + 2, tepb_adr + 4, 2);	// Восстанавливаем Lepb
		Lepb = _byteswap_ushort(*(uint16_t*)(tepb_adr + 4));
		memcpy(in_buf + off + 9, tepb_adr, 4);	// Восстанавливаем Pepb

		if (off != wait_epb) {				// Это первый EPB в заголовке тайла
			sot_start_old = sot_start;		// Сохраняем начало предыдущего SOT
			sot_start = off - 12;			// Смещение маркера SOT
			*(uint32_t*)(in_buf + sot_start) = 0xFF | SOT_LOW << 8 | 0 << 16 | 10 << 24;
			if (sot_start_old != 0) {			// Обработка не первого SOT - можно вычислить PSot
				PSot = sot_start - sot_start_old;
				*(uint32_t*)(in_buf + sot_start_old + 6) = _byteswap_ulong(PSot); // Восстанавливаем PSot
			}
		};
		wait_epb = off + Lepb + 2;	// Смещение следующего ожидаемого EPB в том же заголовке тайла 
	};
	PSot = in_len - sot_start - 2;
	*(uint32_t*)(in_buf + sot_start + 6) = _byteswap_ulong(PSot); // Восстанавливаем PSot для последнего SOT
}


/**
 * \brief Коррекция основного заголовка
 * \details ищет первый тайл, в котором корректируются пре-данные первого EPB заголовка тайла,
 * передвигает указатель *header на первый  байт найденного тайла.
 * Указателю по адресу header присваивается адрес первого тайла, у которого удалось
 * скорректировать пре-данные первого EPB в заголовке тайла или NULL, if такого тайла
 * не обнаружено.
 * Коды завершения:
 * 1 - нет средств jpwl, коррекция потока не производится
 * 0 - заголовок скорректирован
 * -1 - заголовок не корректируется
 * -2 - кодовый поток неправильный
 * -3 - используются не поддерживаемые информативные методы
 * -4 - есть RED, которых не должно быть
 * -5 - несоответствие информации о ESD в EPC и присутствием/отсутствием ESD
 * -7 - основной заголовок длиннее входного буфера
 * -8 - неверная контрольная сумма в EPC
 * \param header  Адрес указателя на основной заголовок
 * \return Код завершения
 */

errno_t dec_mh_correct(addr_char* header)
{
	uint8_t* p, * epb_start, * data_start, * epc_start, * esd_start, * v, Pepc, * inf_met;
	uint8_t p_data[96];
	uint16_t epb_len, epc_len, esd_len, Pcrc, id;
	int rs_ret;
	uint32_t siz_len, pre_l, prot_l, cur_len;
	epb_ms* e;
	epc_ms* ec;

	p = in_buf;				// адрес основоного заголовка
	esd_used = _false_;
	epb_used = _false_;
	is_ammendment = _false_;

	memcpy(rs_data, p, 58);
	memset(rs_data + 58, 0, 6);
	memcpy(p_data, p + 58, 96);
#ifndef RS_OPTIMIZED
	init_rs(160, 64);
	old_rs_mode = 160;			// Запомнили последний код
#endif // !RS_OPTIMIZED
	rs_ret = decode_RS(rs_data, p_data, 160, 64);
	if (rs_ret < 0 
		|| rs_data[0] != 0xff || rs_data[1] != SOC_LOW 
		|| rs_data[2] != 0xff || rs_data[3] != SIZ_LOW 
		|| rs_data[45] != 0xff || rs_data[46] != EPB_LOW) {
		// пытаемся скорректировать пре-данные первого EPB заголовка for 3 цветовых компоненты
		memcpy(rs_data, p, 64);
		memcpy(p_data, p + 64, 96);
		rs_ret = decode_RS(rs_data, p_data, 160, 64);	// попытка коррекции
		if (rs_ret < 0
			|| rs_data[0] != 0xff || rs_data[1] != SOC_LOW 
			|| rs_data[2] != 0xff || rs_data[3] != SIZ_LOW 
			|| rs_data[51] != 0xff || rs_data[52] != EPB_LOW) {
			if (rs_data[0] == 0xff && rs_data[1] == SOC_LOW) { // есть маркер начала кодового потока
				if (rs_data[2] == 0xff && rs_data[3] == SIZ_LOW) { // есть сегмент SIZ
					siz_len = _byteswap_ushort(*(uint16_t*)(rs_data + 4));
					if (siz_len + 5 < in_len) {
						if (rs_data[4 + siz_len] == 0xff) {
							switch (rs_data[5 + siz_len]) {	// есть ли допустимый маркер после сегмента SIZ?
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
							case COM_LOW: return 1;	// в кодовом потоке нет средств jpwl
							}
						}
					}
				}
			};
			return -1;		// заголовок не может быть восстановлен
		}
		else {
			pre_l = 64;
			memcpy(p, rs_data, 64);
		}
	}
	else {
		memcpy(p, rs_data, 58);
		pre_l = 58;
	};

	// проверяем, не выходит ли блок EPB с данными за границу входного буфера
	epb_start = p + pre_l - EPB_LN - 2; // адрес начала блока EPB
	epb_len = _byteswap_ushort(*(uint16_t*)(epb_start + 2));
	prot_l = _byteswap_ulong(*(uint32_t*)(epb_start + 5));
	if (prot_l + epb_len - EPB_LN > in_len) // длина защищенных данных + сегмента EPB больше длины входного буфера
		return -7; // входной буфер содержит не весь заголовок

	// пытаемся скорректировать пост-данные блока EPB заголовка
	data_start = epb_start + epb_len + 2; // адрес начала данных: адрес начала EPB + длина EPB + маркер EPB
	uint32_t badparts = postEPB_correct(epb_start, data_start, pre_l);
	if (badparts > 0)
		return -1;	// осн. заголовок невосстановим

	// основной заголовок восстановлен - собираем данные о сегментах маркеров jpwl
	dec_markers[0].id = EPB_MARKER;
	e = &dec_markers[0].m.epb;
	e->latest = _true_;					// последний в заголовке
	e->index = 0;
	e->hprot = decode_Pepb((uint32_t*)(epb_start + 9), 0);	// метод защиты пост-данных
	e->k_pre = 64;				// параметры защиты пре-данных
	e->n_pre = 160;
	e->pre_len = pre_l;			// длина пре-данных
	e->post_len = prot_l - pre_l;	// длина пост-данных
	// параметры защиты пост-данных
	if (e->hprot == 1) {
		e->k_post = 64;
		e->n_post = 160;
	}
	else if (e->hprot == 2) {
		e->k_post = 25;
		e->n_post = 80;
	}
	else if (e->hprot == 3) {
		e->k_post = 13;
		e->n_post = 40;
	}
	else if (e->hprot >= 32) {
		e->k_post = 32;
		e->n_post = e->hprot;
	}
	else {				// crc16 или crc32
		e->k_post = 0;
		e->n_post = 0;
	};
	e->Lepb = epb_len;			// длина сегмента
	dec_markers[0].tile_num = -1;		// основной заголовок
	dec_markers[0].pos_in = pre_l - EPB_LN - 2;	// позиция во входном буфере
	dec_markers[0].len = epb_len;	// длина сегмента
	if (markers_cnt >= MAX_MARKERS)
		return -2;
	markers_cnt++;

	// вычисляем длину основного заголовка
	mh_len = (pre_l - EPB_LN - 2) + (epb_len + 2) + e->post_len;	// длина до EPB + сегмент EPB с маркером + защищенные EPB данные
	mh_tile_len += mh_len - (epb_len + 2);		// Вычисляем длину основного занголовка - сейчас с EPC и ESD
	cur_len = mh_len - e->post_len;	// длина разобранной части осн. заголовка
	// проверяем наличие сегмента EPC
	epc_start = epb_start + epb_len + 2;	// EPC может начинаться только непосредственно после EPB
	if (*epc_start != 0xff || *(epc_start + 1) != EPC_LOW) // нет EPC, кодовый поток неправильный!
		return -2;
	epc_len = _byteswap_ushort(*(uint16_t*)(epc_start + 2));
	if (epc_len + 2UL >= mh_len - cur_len)	// неправдоподобная длина EPC
		return -2;

	memcpy(out_buf, epc_start, 4);
	memcpy(out_buf + 4, epc_start + 6, epc_len - 4ULL);
	Pcrc = CRC16(out_buf, epc_len);
	if (Pcrc == _byteswap_ushort(*(uint16_t*)(epc_start + 4))) {
		dec_epc_dl = _byteswap_ulong(*(uint32_t*)(epc_start + 6));
		in_len = dec_epc_dl;			// длина кодового потока
		Pepc = *(epc_start + 10);
		if (Pepc & 0x10)	// есть ESD
			esd_used = _true_;
		else
			esd_used = _false_;
		if (Pepc & 0x40)	// есть  EPB
			epb_used = _true_;
		else
			epb_used = _false_;
		if (Pepc & 0x20)	// есть RED
			return -4;
		if (Pepc & 0x80) {	// есть Ammendment
			inf_met = epc_start + EPC_LN + 2;
			id = _byteswap_ushort(*(uint16_t*)inf_met);
			if (id != 0x0200)
				return -3;
			is_ammendment = _true_;
			tepb_count = _byteswap_ushort(*(uint16_t*)(inf_met + 4));
			tepb_adr = inf_met + 6;		// Адрес записи о первом EPB
		}
	}
	else
		return -8;
	dec_markers[1].id = EPC_MARKER;
	ec = &dec_markers[1].m.epc;
	ec->Lepc = epc_len;				// длина сегмента без маркера
	ec->epb_on = epb_used;
	ec->DL = dec_epc_dl;
	dec_markers[1].tile_num = -1;			// осн. заголовок
	dec_markers[1].pos_in = (uint32_t)(epc_start - p);	// смещение отн. начала входного буфера
	dec_markers[1].len = epc_len;		// длина сегмента без маркера
	mh_tile_len -= dec_markers[1].len + 2;		// длинa основного занголовка - отнимаем длину EPC

	if (markers_cnt >= MAX_MARKERS)
		return -2;
	markers_cnt++;
	cur_len += epc_len + 2;	// длина разобранного участка

	// обработка сегмента ESD ( в осн. заголовке может быть только один)
	esd_start = epc_start + epc_len + 2;	// может находиться только непосредственно за EPC
	if (esd_used == _true_) {				// ESD должен присутствовать
		if (*esd_start != 0xff || *(esd_start + 1) != ESD_LOW) // но его нет
			return -5;
	}
	else if (esd_used == _false_ && dec_epc_dl > 0) // ESD не должно быть
		if (*esd_start == 0xff && *(esd_start + 1) == ESD_LOW) // а он есть
			return -5;
	if (*esd_start == 0xff && *(esd_start + 1) == ESD_LOW) { // ESD есть
		esd_used = _true_;
		esd_len = _byteswap_ushort(*(uint16_t*)(esd_start + 2));
		if (esd_len + 2UL > mh_len - cur_len)	// неправдоподобная длина ESD
			return -5;
		// создаем элемент массива dec_markers
		dec_markers[2].id = ESD_MARKER;		// идентификатор
		dec_markers[2].len = dec_markers[2].m.esd.Lesd = esd_len;	// длина сегмента
		dec_markers[2].tile_num = -1;		// осн. заголовок
		dec_markers[2].pos_in = (uint32_t)(esd_start - p);	// смещение начала ESD отн. начала входного буфера
		mh_tile_len -= dec_markers[2].len + 2;		// Вычисляем длину основного занголовка - отнимаем длину ESD
		if (markers_cnt >= MAX_MARKERS)
			return -2;
		markers_cnt++;
	};
	// Если использован Ammendment, выполняем обратную перестановку
	if (is_ammendment)
		deinterleave_instream();

	// поиск первого тайла
	v = p + mh_len;			// адрес первого тайла после осн. заголовка
	if (in_len <= v - in_buf)
		return -1;
	v = dec_tile_detect(v);
	*header = v;	// адрес первого тайла или NULL при отсутствии
	return 0;
}

/**
 * \brief Коррекция пре-данных всей группы EPB в заголовке тайла
 * \param tile  Адрес первого байта тайла (у него пре-данные первого EPB уже скорректированы)
 * \param  data_offset В него заносится смещение первого байта пост-данных относительно начала тайла
 * \return 0 - все нормально скорректировалось, -1 - есть некорректируемые EPB
 */
errno_t tile_preEPB_correct(uint8_t* tile, uint32_t* data_offset)
{
	uint8_t* v;
	uint16_t epb_l, i = 0;
	uint32_t LDPepb;
	epb_ms* e;

	*data_offset = SOT_LN + 2;		// длина сегмента SOT + маркер
	v = tile + *data_offset;		// адрес первого EPB
	while (1) { // цикл по блокам EPB в заголовке тайла
		if (markers_cnt >= MAX_MARKERS)
			return -1;
		// поскольку заголовок EPB уже скорректирован, создаем запись о нем в dec_markers
		dec_markers[markers_cnt].id = EPB_MARKER;
		e = &dec_markers[markers_cnt].m.epb;
		epb_l = _byteswap_ushort(*(uint16_t*)(v + 2)); // длина сегмента EPB
		*data_offset += epb_l + 2;				// длинa сегмента EPB + маркер
		dec_markers[markers_cnt].len = e->Lepb = epb_l;
		dec_markers[markers_cnt].pos_in = (uint32_t)(v - in_buf);	// смещение отн. начала входного буфера
		dec_markers[markers_cnt++].tile_num = tile_count;	// индекс тайла

		e->index = (uint8_t)(i & 0x3f);				// индекс блока EPB в заголовке
		e->hprot = decode_Pepb((uint32_t*)(v + 9), i == 0 ? 1 : 2);			// метод защиты пост-данных
		e->n_pre = (i == 0 ? 80 : 40);				// параметры RS-кодов для пре-данных
		e->k_pre = (i == 0 ? 25 : 13);
		e->pre_len = (i == 0 ? SOT_LN + 2 : 0) + EPB_LN + 2;	// длина пре-данных: сегменты SOT и EPB + 2 маркера
		LDPepb = _byteswap_ulong(*(uint32_t*)(v + 5));	// длина защищаемых данных
		e->post_len = LDPepb - e->pre_len;		// длина пост-данных

		switch (e->hprot) {
		case 16:							// для crc нет параметров RS-кодов
		case 32: 	e->n_post = e->k_post = 0;
			break;
		case 1:	e->n_post = 160; e->k_post = 64;	// параметры RS-кодов для пост-данных
			break;
		case 2:	e->n_post = 80; e->k_post = 25;
			break;
		case 3:	e->n_post = 40; e->k_post = 13;
			break;
		default:	e->n_post = e->hprot; e->k_post = 32;
		};
		e->latest = (*(v + 4) & 0x40) == 0 ? _false_ : _true_;	// последний или нет в заголовке
		if (e->latest == _true_)					// обработан последний EPB
			break;
		v += epb_l + 2ULL;							// переходим к адресу следующего EPB
#ifndef RS_OPTIMIZED
		if (old_rs_mode != 40) {				// Последник код не RS(40,13)
			init_rs(40, 13);
			old_rs_mode = 40;
		};
		if (decode_RS(v, v + 13, 40, 13) < 0)		// заголовок EPB не корректируется
#else
		if (decode_RS(v, NULL, 40, 13) < 0)
#endif // !RS_OPTIMIZED
			return -1;
		i++;
	};
	return 0;
}

/**
 * \brief Коррекция тайла
 * \param tile  Адрес первого байта тайла where скорректированы пре-данные первого EPB, т.е. сегмент SOT - правильный
 * \return Адрес первого байта следующего тайла, у которого скорректировались пре-данные первого EPB, или NULL
 */
uint8_t* dec_tile_correct(uint8_t* tile)
{
	uint8_t* v, * u, * w;
	uint16_t mark_count_old, i;
	uint32_t sot_l, d_off, l, tilemark_ln, sot_l_new, rr, badparts = 0;

	tilemark_ln = 0;						// длина всех удаляемых из заголовка тайла сегментов
	sot_l = _byteswap_ulong(*(uint32_t*)(tile + 6)); // извлекаем длину тайла
	mark_count_old = markers_cnt;			// запоминаем счетчик маркеров для возможного отката массива dec_markers
	errno_t err_c = tile_preEPB_correct(tile, &d_off);

	if (!err_c) {
		w = u = tile + d_off;	// адрес первого байта пост-данных
		badparts = postEPB_correct(in_buf + dec_markers[mark_count_old].pos_in, u, 25);
	}
	if (err_c || badparts > 0) {
		if (markers_cnt >= MAX_MARKERS)
			return NULL;
		has_bad_blocks = _true_;
		markers_cnt = mark_count_old;		// откат счетчика маркеров
		dec_markers[markers_cnt].id = BAD_ID;	// создаем bad блок размером с тайл
		bad_block_length += sot_l;
		dec_markers[markers_cnt].len = dec_markers[markers_cnt].m.bad.Lbad = sot_l - 2; // длина bad блока
		dec_markers[markers_cnt].tile_num = tile_count++;	// BAD-блок нумеруется как тайл
		dec_markers[markers_cnt++].pos_in = (uint32_t)(tile - in_buf);	// позиция блока - начало тайла
	}
	else {					
		// заголовок тайла скорректирован, приступаем к данным
		v = tile + SOT_LN + 2 + 5;		// адрес LDPepb первого EPB: сегмент SOT + маркер SOT + смещение LDPepb
		tilemark_ln += dec_markers[mark_count_old].len + 2;	// добавляем длину сегмента первого EPB
		u += dec_markers[mark_count_old].m.epb.post_len;	// адрес начала защищенных данных следующего EPB
		for (i = mark_count_old + 1; i < markers_cnt; i++) { // обработка всех последующих EPB, защищающих данные
			tilemark_ln += dec_markers[i].len + 2;
			badparts = postEPB_correct(in_buf + dec_markers[i].pos_in, u, 13);
			switch (dec_markers[i].m.epb.hprot) {			// вычисляем длину блока пост-данных
			case 16:									
			case 32:	l = dec_markers[i].m.epb.post_len; // crc-16 или crc-32 - все данные
				break;
			case 160:	l = 64;	// для RS-кодов - длина кодируемого фрагмента
				break;
			case 80:	l = 25;
				break;
			case 40:	l = 13;
				break;
			default:	l = 32;
			};

			d_off = (uint32_t)(u - tile);	// смещение начала пост-данных EPB отн. начала тайла
			u += dec_markers[i].m.epb.post_len;	// вычисляем адрес начала защищенных данных следующего EPB
		};
		// разбор маркеров ESD
		while (*w == 0xff && *(w + 1) == ESD_LOW) { // обработка очередного маркера ESD
			if (markers_cnt >= MAX_MARKERS)
				return NULL;
			dec_markers[markers_cnt].id = ESD_MARKER;	// ид. маркера
			dec_markers[markers_cnt].len = dec_markers[markers_cnt].m.esd.Lesd = _byteswap_ushort(*(uint16_t*)(w + 2));
			tilemark_ln += dec_markers[markers_cnt].len + 2;	// добавляем длину сегмента ESD
			dec_markers[markers_cnt].tile_num = tile_count; //  индекс разобранного тайла
			dec_markers[markers_cnt].pos_in = (uint32_t)(w - in_buf);		// позиция во входном буфере
			w += dec_markers[markers_cnt++].len + 2ULL;		// переводим адрес на потенциально следующий ESD
		};

		if (badparts > 0)
			tile_red_rest_cnt++;		// Инкремент частично восстановленных тайлов
		else
			tile_all_rest_cnt++;		// Инкремент полностью восстановленных тайлов
		
		tile_count++;
		sot_l_new = sot_l - tilemark_ln;	// вычисляем новую длину тайла, которая будет после удаления сегментов
		mh_tile_len += sot_l_new;
		*(uint32_t*)(tile + 6) = _byteswap_ulong(sot_l_new); // заносим новую длину в сегмент SOT во входной буфер
	}

	// ищем следующий тайл, у которого корректируютcя пре-данные первого EPB
	tile += sot_l;				// адрес начала следующего тайла
	if (tile - in_buf > in_len)
		return NULL;
	rr = in_len - (uint32_t)(tile - in_buf);	// кол-во байт до конца входного буфера

	u = tile; // началo поиска
	if (rr > 80)
		tile = dec_tile_detect(tile);		// распознаем следующий тайл
	else if (rr <= 2)
		return NULL;
	if (tile == NULL || (rr < 81 && rr > 2)) {
		if (markers_cnt >= MAX_MARKERS)
			return NULL;
		has_bad_blocks = _true_;
		dec_markers[markers_cnt].id = BAD_ID;	// создаем bad блок размером с тайл
		bad_block_length += rr - 2;
		dec_markers[markers_cnt].len = rr - 4; // длина bad блока
		dec_markers[markers_cnt].tile_num = tile_count++;	
		dec_markers[markers_cnt++].pos_in = (uint32_t)(u - in_buf);	// позиция блока - начало тайла
		return NULL;
	}
	return tile;
}

/**
 * \brief Копирование данных в выходной буфер
 * \details Выполняет копирование скорректированных данных в выходной буфер.
 * При этом из кодового потока удаляются сегменты маркеров EPB, EPC и ESD.
 * \return Длина выходного буфера
 */
uint32_t dec_data_copy()
{
	int epc_pos = 0, t_no = -2; // начальный индекс заголовка
	uint32_t i, j, out_len = 0;

	for (i = j = 0; i < markers_cnt; i++) {
		if (dec_markers[i].tile_num != t_no) {	// первый маркер очередного заголовка
			t_no = dec_markers[i].tile_num;	// запомним индекс этого заголовка
			for (; j < dec_markers[i].pos_in; j++) // копируем данные, предшествующие найденному маркеру
				out_buf[out_len++] = in_buf[j]; // в выходной буфер
		}
		j += dec_markers[i].len + 2;
	}
	if (j < in_len)
		memcpy_s(out_buf + out_len, (size_t)in_len - j, in_buf + j, (size_t)in_len - j);

	if (mh_tile_len > out_len) {	// Сумма длин основного заголовка и всех тайлов больше расчетной
		out_len = mh_tile_len + 2;	// Увеличиваем длину выходного потока
		out_buf[out_len - 2] = 0xFF;	// Вставляем потерянный маркер конца кодового потока
		out_buf[out_len - 1] = EOC_LOW;
	}
	if (!(out_buf[out_len - 2] == 0xFF && out_buf[out_len - 1] == EOC_LOW)) {	// Пропущен маркер конца EOC
		out_buf[out_len - 2] = 0xFF;
		out_buf[out_len - 1] = EOC_LOW;
	}
	return out_len;
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
 * \param inp_buf  Адрес входного буфера, в котором расположен кодовый поток jpeg2000 часть 2, подвергшийся воздействию ошибок в канале передачи данных
 * \param inp_len  Длина данных во входном буфере в байтах
 * \param out_buf  Адрес выходного буфера, в который следует записать скорректированный кодовый поток jpeg2000, возможно с внедренными маркерами EPC и RED(остаточная ошибка)
 * \param out_len  Адрес переменной, в которую будет записана длина данных (в байтах) выходного буфера
 * \return Код завершения
 */

int w_decoder(uint8_t* inp_buffer, uint32_t inp_len, uint8_t* out_buffer, uint32_t* out_len)
{
	uint8_t* p;
	int i;

	in_buf = p = inp_buffer;
	in_len = inp_len;
	out_buf = out_buffer;
	tile_count = 0;
	markers_cnt = 0;
	old_rs_mode = 0;				// RS-код еще не инициализирован
	mh_tile_len = 0;				// Обнуление суммы длин основного заголовка и тайлов, копируемых в выходной буфер
	bad_block_length = tile_all_rest_cnt = tile_red_rest_cnt = 0;	// Обнуление статистики корекции тайлов
	i = dec_mh_correct(&p);			// коррекция основного заголовка
	has_bad_blocks = _false_;
	if (i < 0) {					// основной заголовок не корректируется
		return -1;
	}
	else if (i > 0) {				// средства jpwl не обнаружены в кодовом потоке - коррекция не производится
		if (!inp_len)
			return -1;
		*out_len = inp_len;
		memcpy(out_buffer, inp_buffer, inp_len);
		return 1;
	};
	while (p != NULL) {
		p = dec_tile_correct(p);
	};
	*out_len = dec_data_copy();
	return 0;
}

/**
 * \brief  Вызов декодера jpwl
 * \details Коды завершения:
 * 1 - в кодовом потоке нет средств jpwl, коррекция не нужна
 * 0 - основной заголовок восстановлен, кодовый поток несет данные об изображении, кадр следует отобразить
 * -1 - основной заголовок не восстановлен, кадр изображения следует отбросить
 * -2 - нет ни одного тайла с данными, кадр изображения следует отбросить
 * \param params  Структура типа w_dec_params, содержащая параметры работы декодера
 * \param out_len  По адресу, содержащемуся в out_len записывается длина сформированного кодового потока
 * \return Код завершения, возвращаемый декодером (см. детали)
 */
uint32_t w_decoder_call(w_dec_params* params)
{
	int i;
	uint32_t o_l;

	i = w_decoder(params->inp_buffer, params->inp_length, params->out_buffer, &o_l);
	if (i == 0)
		params->out_length = o_l;
	return i;
}

/**
 * \brief Инициализация декодера jpwl
 * \param params  Адрес структуры с параметрами инициализации декодера jpwl
 */
__declspec(dllexport)
void jpwl_dec_init()
{
}

/**
 * \brief Запуск декодера jpwl
 * \param params  Адрес структуры с параметрами инициализации декодера jpwl
 */
__declspec(dllexport)
errno_t jpwl_dec_run(jpwl_dec_bParams* bParams, jpwl_dec_bResults* bResults)
{
	int i_res;
	w_dec_params dec_par = {
		.inp_buffer = bParams->inp_buffer,
		.inp_length = bParams->inp_length,
		.out_buffer = bParams->out_buffer
	};
	memset(&stats, 0, sizeof(stats));

	if (bParams->inp_length == 0) {
		bResults->out_length = 0;
		return -1;
	};

	bad_block_length = 0;
	tile_all_rest_cnt = 0;
	tile_red_rest_cnt = 0;
	i_res = w_decoder_call(&dec_par);
	if (i_res == 1) {
		stats.not_JPWL++;
		bResults->out_length = dec_par.inp_length;
	}
	else if (i_res == 0) {
		if (has_bad_blocks == _true_)
			stats.partially_restored++;
		else
			stats.fully_restored++;
		bResults->out_length = dec_par.out_length;
	}
	else {
		stats.not_restored++;
		bResults->out_length = 0;
		bad_block_length = bParams->inp_length;
	};
	bResults->all_bad_length = bad_block_length;
	bResults->tile_all_rest_cnt = tile_all_rest_cnt;
	bResults->tile_part_rest_cnt = tile_red_rest_cnt;

	return 0;
}

/**
 * \brief Запуск декодера jpwl
 * \param params  Адрес структуры с параметрами инициализации декодера jpwl
 */
__declspec(dllexport)
restore_stats* jpwl_dec_stats()
{
	return &stats;
}
