// rs_crc_lib.cpp : Defines the entry point for the DLL application.
//
/**
 * \file rs_crc_lib.cpp
 * \brief Файл текстов программ библиотеки вычисления кодов четности
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12
*/

#include "stdio.h"
#include <crtdbg.h>
#include <math.h>
#include <memory.h>
#include "stdlib.h"
#include "rs_crc_decl.h"

/**
 * \brief Тип для значений элементов поля Галуа, используемых RS-кодом
 */
typedef int gf;

static int	KK;	///< Количество кодируемых информационных символов


/* 1+x^2+x^3+x^4+x^8 */
int Pp[MM + 1] = { 1, 0, 1, 1, 1, 0, 0, 0, 1 }; ///< Примитивный полином для символа из 8 бит 

#define B0	0 ///< Альфа-экспонента для первого корня полинома генератора 
/* Отличается от значения по умолчанию 1 */

gf Alpha_to[NN + 1]; ///< index.polynomial из таблицы преобразования

/* */
gf Index_of[NN + 1]; ///< Polynomial.index из таблицы преобразования 

/* Нет допустимого значения в индексной форме, которое представляет 0
 * поэтому требуется специальное значение для этой цели
 */
#define A0	(NN) ///< Ноль в индексной форме

 /* Генератор полинома g(x)
  * Степень g(x) = 2*TT
  * имеет корни @**B0, @**(B0+1), ... ,@^(B0+2*TT-1)
  */

gf		Gg[NN + 1]; ///< Генерирующий (образующий) полином

/**
 * \brief Вычисление x % NN, где NN = 2**MM - 1, без использования медленного деления
 */
static  gf modnn(int x)
{
	while (x >= NN) {
		x -= NN;
		x = (x >> MM) + (x & NN);
	}
	return x;
}

#define	CLEAR(a,n) {\
	int ci;\
	for(ci=(n)-1;ci >=0;ci--)\
		(a)[ci] = 0;\
	}

#define	COPY(a,b,n) {\
	int ci;\
	for(ci=(n)-1;ci >=0;ci--)\
		(a)[ci] = (b)[ci];\
	}
#define	COPYDOWN(a,b,n) {\
	int ci;\
	for(ci=(n)-1;ci >=0;ci--)\
		(a)[ci] = (b)[ci];\
	}



/* Генерирует GF(2**m) из неприводимого полинома p(X) в p[0]..p[m]
   Вычисляемые таблицы:
   index->polynomial form   alpha_to[] содержит j=alpha**i;
   polynomial form -> index из  index_of[j=alpha**i] = i
   alpha=2 это примитивный элемент GF(2**m)
*/
/**
 * \brief Генерация таблицы вычислений в поле Галуа на основе примитивного полинома
 * \details Вызывается один раз перед началом кодирования или декодирования. Результат заносится
 * в массивы Alpha_to и Index_of
 */
__declspec(dllexport)
void generate_gf(void)
{
	register int i, mask;

	mask = 1;
	Alpha_to[MM] = 0;
	for (i = 0; i < MM; i++) {
		Alpha_to[i] = mask;
		Index_of[Alpha_to[i]] = i;
		if (Pp[i] != 0)
			Alpha_to[MM] ^= mask;
		mask <<= 1;
	}
	Index_of[Alpha_to[MM]] = MM;

	mask >>= 1;
	for (i = MM + 1; i < NN; i++) {
		if (Alpha_to[i - 1] >= mask)
			Alpha_to[i] = Alpha_to[MM] ^ ((Alpha_to[i - 1] ^ mask) << 1);
		else
			Alpha_to[i] = Alpha_to[i - 1] << 1;
		Index_of[Alpha_to[i]] = i;
	}
	Index_of[0] = A0;
	Alpha_to[NN] = 0;
}

/*
 * Генерирует полином для коррекции TT ошибок, длина
 * NN=(2**MM -1) RS-кода как продукт (X+@**(B0+i)), i = 0,
 * ... ,(2*TT-1)
 *
 * Примеры:
 *
 * Если B0 = 1, TT = 1. deg(g(x)) = 2*TT = 2.
 * g(x) = (x+@) (x+@**2)
 *
 * Если B0 = 0, TT = 2. deg(g(x)) = 2*TT = 4.
 * g(x) = (x+1) (x+@) (x+@**2) (x+@**3)
 */
 /**
  * \brief Генерирует полином для коррекции ошибок
  * \details Результат заносится в массив Gg
  */
void gen_poly(void)
{
	register int i, j;

	Gg[0] = Alpha_to[B0];
	Gg[1] = 1;		/* g(x) = (X+@**B0) в начале */
	for (i = 2; i <= NN - KK; i++) {
		Gg[i] = 1;
		/*
		 * Дале умножаем (Gg[0]+Gg[1]*x + ... +Gg[i]x^i) на
		 * (@**(B0+i-1) + x)
		 */
		for (j = i - 1; j > 0; j--)
			if (Gg[j] != 0)
				Gg[j] = Gg[j - 1] ^ Alpha_to[modnn((Index_of[Gg[j]]) + B0 + i - 1)];
			else
				Gg[j] = Gg[j - 1];
		/* Gg[0] никогда не может быть 0 */
		Gg[0] = Alpha_to[modnn((Index_of[Gg[0]]) + B0 + i - 1)];
	}
	/* конвертируем Gg[] в индексную форму для быстрого кодирования */
	for (i = 0; i <= NN - KK; i++)
		Gg[i] = Index_of[Gg[i]];
}

// инициализация кодера/декодера Рида-Соломона на длину кодируемого слова
// Входные параметры:
// k = NN - (n_rs - k_rs), т.е.
// k = 255 - кол-во символов четности в кодовом слове Рида-Соломона
// Результат: строится полином в массиве Gg
// Функция должна быть вызвана перед первым кодированием/декодированием кода RS(n_rs,k_rs) 

/**
 * \brief Инициализация кодера/декодера кодов Рида-Соломона
 * \details Инициализация проводится один раз перед серией кодирований/декодирований с одинаковыми значениями
 * n_rs и k_rs. Декодирование позволяет исправить (n_rs - k_rs)/2 поврежденных байт
 * \param n_rs Общая длина кодового слова
 * \param k_rs Количество информационных байт в кодовом слове
 */
__declspec(dllexport)
void init_rs(int n_rs, int k_rs)
{
	KK = NN - (n_rs - k_rs);
	if (KK >= NN) {
		printf("KK must be less than 2**MM - 1\n");
		exit(1);
	}
	gen_poly();
}

/*
 *берет строку символов из CodingData[i], i=0..(k_rs-1) и кодирует
 * систематически чтобы получить n_rs-k_rs символов четности в bb[0]..bb[n_rs-k_rs-1]
 * CodingData[] - это входные данные
 * bb[] это результат в полиномиальной форме. Кодирование выполняется с использованием
 * сдвигового регистра с соответствующими связями, определяемыми элементами Gg[],
 * который был сгенерирован выше. Кодовое слово  c(X) =
 * data(X)*X**(NN-KK)+ b(X)
 * k_rs - количество информационных символов
 * n_rs - длина кодового слова, т.е. n_rs-k_rs - кол-во символов четности
 * Перед первым кодированием кода RS(n_rs,k_rs) должна быть вызвана функция Init_rs(n_rs,k_rs)
 * Для всех последующих вызовов для кодирования этим же кодом инициализация не нужна.
 * При смене кода производится новая инициализация
 */
 /**
  * \brief Кодирование RS-кодов
  * \param CodingData Адрес буфера с кодируемыми данными
  * \param bb Адрес буфера для записи кодов четности
  * \param n_rs Длина кодового слова (информационные байты + байты кодов четности)
  * \param k_rs Количество информационных байт
  * \return 0
  */
__declspec(dllexport)
int encode_rs(dtype* CodingData, dtype* bb, int n_rs, int k_rs)
{
	register int i, j;
	gf feedback;
	dtype data[NN];
	//	unsigned short P, NN_P;

	//	P = n_rs - k_rs;
	//	NN_P = NN - P;
	memset(data, 0, NN);
	memcpy(data, CodingData, k_rs);
	//	init_rs(NN_P);

	CLEAR(bb, NN - KK);
	for (i = KK - 1; i >= 0; i--) {
		feedback = Index_of[data[i] ^ bb[NN - KK - 1]];
		if (feedback != A0) {	/* значение feedback не ноль */
			for (j = NN - KK - 1; j > 0; j--)
				if (Gg[j] != A0)
					bb[j] = bb[j - 1] ^ Alpha_to[modnn(Gg[j] + feedback)];
				else
					bb[j] = bb[j - 1];
			bb[0] = Alpha_to[modnn(Gg[0] + feedback)];
		}
		else {	/* feedback =0. кодер выполняет
			  * однобайтовый сдвиг */
			for (j = NN - KK - 1; j > 0; j--)
				bb[j] = bb[j - 1];
			bb[0] = 0;
		}
	}
	return 0;
}

/*
 * Выполняет декодирование RS-кодов. Если декодирование успешно,
 * записывает кодовое слово в data[] на то же место. Иначе data[] не изменяются.
 *
 * Возвращает количество скорректированых символов, или -1 если кодовое слово
 * недопустимо или некорректабельно.
 *
 * Изначально "no_eras" объявлено в вызывающей программе. Далее,
 * максимум # исправимых ошибок -  t_after_eras = floor((NN-KK-no_eras)/2).
 * Если кол-во канальных ошибок не превышает "t_after_eras"
 * измененное кодовое слово может быть восстановлено.
 * CodedData - восстанавливаемые данные длиной k_rs символов (байт)
 * ParityBuf - символы четности длиной n_rs - k_rs байт
 * n_rs - длина кодового rs - слова
 * ? Перед первым декодированием кода RS(n_rs,k_rs) должна быть вызвана функция Init_rs(n_rs,k_rs)
 * ? Для всех последующих вызовов для декодирования этим же кодом инициализация не нужна.
 * ? При смене кода производится новая инициализация

 */
 //__declspec(dllexport) int decode_rs(dtype *CodedData, dtype *ParityBuf,  int n_rs, int k_rs)

 /**
  * \brief Декодер RS-кодов
  * \details Возвращает код возврата:
  * 0 - в данных нет искажений,
  * >0 - данные успешно скорректированы, возвращается количество исправленных байт,
  * -1 - данные не корректируются.
  * \param CodedData Адрес буфера с декодируемыми данными
  * \param ParityBuf Адрес буфера с кодами четности
  * \param n_rs Длина кодового слова (информационные байты + байты кодов четности)
  * \param k_rs Количество информационных байт
  * \return Код возврата (см. детали)
  */
int dec_rs(dtype* CodedData, dtype* ParityBuf, int n_rs, int k_rs)
{
	int deg_lambda, el, deg_omega;
	int i, j, r;
	gf q, tmp, num1, num2, den, discr_r;
	gf recd[NN];
	/* полином локатора и полином синдрома */
	/*gf lambda[NN-KK + 1], s[NN-KK + 1];
	gf b[NN-KK + 1], t[NN-KK + 1], omega[NN-KK + 1];
	gf root[NN-KK], reg[NN-KK + 1], loc[NN-KK];*/
	gf lambda[NN + 1], s[NN + 1];
	gf b[NN + 1], t[NN + 1], omega[NN + 1];
	gf root[NN], reg[NN + 1], loc[NN];
	int syn_error, count, no_eras;
	dtype data[NN + 1];
	unsigned short P, NN_P;

	P = n_rs - k_rs;		// подготовка данных для декодирования
	NN_P = NN - P;
	memset(data, 0, NN + 1);
	_ASSERT(k_rs > 0 && k_rs <= NN);
	memcpy(data, CodedData, k_rs);
	_ASSERT(P > 0 && P <= NN);
	memcpy(data + NN_P, ParityBuf, P);
	//	init_rs(NN_P);
	no_eras = 0;


	/* data[] в полиномиальной форме, копируем и преобразуем в индексную форму */
	for (i = NN - 1; i >= 0; i--) {
		recd[i] = Index_of[(int)data[i]];
	}
	/* первая форма синдромов;  сравниваем recd(x) с корнями g(x)
	 * обозначаемыми @**(B0+i), i = 0, ... ,(NN-KK-1)
	 */
	syn_error = 0;
	for (i = 1; i <= NN - KK; i++) {
		tmp = 0;
		for (j = 0; j < NN; j++)
			if (recd[j] != A0)	/* recd[j] в индексной форме */
				tmp ^= Alpha_to[modnn(recd[j] + (B0 + i - 1) * j)];
		syn_error |= tmp;	/* уст. флаг в ненулевой синдром  =>
					 * ошибка */
					 /* сохраняем синдром в индексной форме  */
		s[i] = Index_of[tmp];
	}
	if (!syn_error) {
		/*
		 * если синдром ноль, data[] это кодовое слово и в нем нет
		 * ошибок. Возвращаем data[] неизмененным
		 */
		return 0;
	}
	CLEAR(&lambda[1], NN - KK);
	lambda[0] = 1;

	/*	if (no_eras > 0) {
			// Инициализируем lambda как полином локатора ошибок
			lambda[1] = Alpha_to[eras_pos[0]];
			for (i = 1; i < no_eras; i++) {
				u = eras_pos[i];
				for (j = i+1; j > 0; j--) {
					tmp = Index_of[lambda[j - 1]];
					if(tmp != A0)
						lambda[j] ^= Alpha_to[modnn(u + tmp)];
				}
			}
		}
	*/

	for (i = 0; i < NN - KK + 1; i++)
		b[i] = Index_of[lambda[i]];

	/*
	 * Начало алгоритма Берлекемпа-Месси для определения
	 * полинома локатора ошибок+стираний
	 */
	r = no_eras;
	el = no_eras;
	while (++r <= NN - KK) {	/* r is the step number */
		/* Вычисление несоответствия в r-том шаге в форме полинома */
		discr_r = 0;
		for (i = 0; i < r; i++) {
			if ((lambda[i] != 0) && (s[r - i] != A0)) {
				discr_r ^= Alpha_to[modnn(Index_of[lambda[i]] + s[r - i])];
			}
		}
		discr_r = Index_of[discr_r];	/* Index form */
		if (discr_r == A0) {
			/* 2 строки ниже: B(x) <-- x*B(x) */
			COPYDOWN(&b[1], b, NN - KK);
			b[0] = A0;
		}
		else {
			/* 7 строк ниже: T(x) <-- lambda(x) - discr_r*x*b(x) */
			t[0] = lambda[0];
			for (i = 0; i < NN - KK; i++) {
				if (b[i] != A0)
					t[i + 1] = lambda[i + 1] ^ Alpha_to[modnn(discr_r + b[i])];
				else
					t[i + 1] = lambda[i + 1];
			}
			if (2 * el <= r + no_eras - 1) {
				el = r + no_eras - el;
				/*
				 * 2 строки ниже: B(x) <-- inv(discr_r) *
				 * lambda(x)
				 */
				for (i = 0; i <= NN - KK; i++)
					b[i] = (lambda[i] == 0) ? A0 : modnn(Index_of[lambda[i]] - discr_r + NN);
			}
			else {
				/* 2 строки ниже: B(x) <-- x*B(x) */
				COPYDOWN(&b[1], b, NN - KK);
				b[0] = A0;
			}
			COPY(lambda, t, NN - KK + 1);
		}
	}

	/* Преобраз. lambda в индексную форму и вычисление deg(lambda(x)) */
	deg_lambda = 0;
	for (i = 0; i < NN - KK + 1; i++) {
		lambda[i] = Index_of[lambda[i]];
		if (lambda[i] != A0)
			deg_lambda = i;
	}
	/*
	 * Находим корни полинома локатора ошибок. Методом Ченя
	 */
	COPY(&reg[1], &lambda[1], NN - KK);
	count = 0;		/* Кол-во корней lambda(x) */
	for (i = 1; i <= NN; i++) {
		q = 1;
		for (j = deg_lambda; j > 0; j--)
			if (reg[j] != A0) {
				reg[j] = modnn(reg[j] + j);
				q ^= Alpha_to[reg[j]];
			}
		if (!q) {
			/* сохраняем корень (index-form) и номер обнаруженной ошибки */
			root[count] = i;
			loc[count] = NN - i;
			count++;
		}
	}

	if (deg_lambda != count) {
		/*
		 * deg(lambda) не равен кол-ву корней => обнаружена
		 * неисправимая ошибка
		 */
		return -1;
	}
	/*
	 * Вычисление err+eras оценочного полинома omega(x) = s(x)*lambda(x) (modulo
	 * x**(NN-KK)). в индексной форме. Находим deg(omega).
	 */
	deg_omega = 0;
	for (i = 0; i < NN - KK; i++) {
		tmp = 0;
		j = (deg_lambda < i) ? deg_lambda : i;
		for (; j >= 0; j--) {
			if ((s[i + 1 - j] != A0) && (lambda[j] != A0))
				tmp ^= Alpha_to[modnn(s[i + 1 - j] + lambda[j])];
		}
		if (tmp != 0)
			deg_omega = i;
		omega[i] = Index_of[tmp];
	}
	omega[NN - KK] = A0;

	/*
	 * Вычисляем значения ошибок в полин. форме. num1 = omega(inv(X(l))), num2 =
	 * inv(X(l))**(B0-1) и den = lambda_pr(inv(X(l))) все в полин. форме
	 */
	for (j = count - 1; j >= 0; j--) {
		num1 = 0;
		for (i = deg_omega; i >= 0; i--) {
			if (omega[i] != A0)
				num1 ^= Alpha_to[modnn(omega[i] + i * root[j])];
		}
		num2 = Alpha_to[modnn(root[j] * (B0 - 1) + NN)];
		den = 0;

		/* lambda[i+1] для четного i  - формальная производная lambda_pr of lambda[i] */
		for (i = min(deg_lambda, NN - KK - 1) & ~1; i >= 0; i -= 2) {
			if (lambda[i + 1] != A0)
				den ^= Alpha_to[modnn(lambda[i + 1] + i * root[j])];
		}
		if (den == 0) {
			return -1;
		}
		/* Исправление ошибки */
		if (num1 != 0) {
			data[loc[j]] ^= Alpha_to[modnn(Index_of[num1] + Index_of[num2] + NN - Index_of[den])];
		}
	};
	//	if(count>tt)		// исправлено больше, чем разрешающая способность кода
	//		return(-1);
	_ASSERT(k_rs > 0 && k_rs <= NN);
	memcpy(CodedData, data, k_rs);
	_ASSERT(NN_P > 0 || NN_P <= NN);
	memcpy(ParityBuf, data + NN_P, P);
	return count;
}

//unsigned char ZeroMas[NN];		///< Массив нулей для контроля полных стираний данных

/* декодирование RS-кодов с обнаружением неисправимой ошибки путем повторного декодирования
*/
/**
 * \brief Декодирование RS-кодов с обнаружением неисправимой ошибки путем повторного декодирования
 * \details Декодер RS-кодов при невозможности скорректировать данные может "ошибиться" и выдать
 * пороговое значение (n_rs - k_rs)/2 как количество успешно скорректированных байт. В этом случае требуется повторное
 * декодироване тех же самых данных. Если оно завершается с кодом, отличным от 0, данные не могут быть скорректированы.
 * Возвращает код возврата:
 * 0 - в данных нет искажений,
 * >0 - данные успешно скорректированы, возвращается количество исправленных байт,
 * -1 - данные не корректируются.
 * \param CodedData Адрес буфера с декодируемыми данными
 * \param ParityBuf Адрес буфера с кодами четности
 * \param n_rs Длина кодового слова (информационные байты + байты кодов четности)
 * \param k_rs Количество информационных байт
 * \return Код возврата (см. детали)
 */
__declspec(dllexport)
int decode_rs(dtype* CodedData, dtype* ParityBuf, int n_rs, int k_rs)
{
	int rs_res, tt;

	tt = (n_rs - k_rs) >> 1; // разрешающая способность RS-кода
	rs_res = dec_rs(CodedData, ParityBuf, n_rs, k_rs);		// декодируем
	if (rs_res > tt)											// исправлено больше ошибок, чем позволяют коды - неисправимая ошибка
		return(-1);
	if (rs_res == tt) {											// исправлены ошибки на границе способности кодов
		if (dec_rs(CodedData, ParityBuf, n_rs, k_rs) != 0)		// провторно декодируем и снова исправлены ошибки
			return(-1);										// неустранимая ошибка
	};
	/*	if(rs_res==0)
			if(!(memcmp(CodedData,ZeroMas,k_rs) | memcmp(ParityBuf,ZeroMas,n_rs-k_rs)))	// Полное стирание
				return(-1);
	*/
	return(rs_res);
}
