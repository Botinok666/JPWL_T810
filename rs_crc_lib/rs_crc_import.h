#pragma once 
/** 
 * \file rs_crc_import.h 
 * \brief Файл программного интерфейса библиотеки вычисления кодов четности
 * \author Скороход С.В.
 * \date Дата последней модификации - 9.11.12 
*/

#include "..\rs_crc_lib\rs_crc_decl.h"

/* Генерирует GF(2**m) из неприводимого полинома p(X) в p[0]..p[m]
   Вычисляемые таблицы:  
   index->polynomial form   alpha_to[] содержит j=alpha**i;
   polynomial form -> index из  index_of[j=alpha**i] = i
   alpha=2 это примитивный элемент GF(2**m)
*/
#ifndef __cplusplus
__declspec(dllimport) 
void generate_gf(void);
#else
extern "C" 
__declspec(dllimport) 
void generate_gf(void);
#endif

/* функция генерации таблиц для вычисления в поле Галуа
 * должна вызываться один раз перед первым вызовом init_rs
 */
#ifndef __cplusplus
__declspec(dllimport) 
void generate_gf(void);
#else
extern "C" 
__declspec(dllimport) 
void generate_gf(void);
#endif

/** функция инициализации 
 * должна вызываться для каждого нового RS-кода RS(n_rs,k_rs)
 */
#ifndef __cplusplus
__declspec(dllimport) 
void init_rs(int n_rs, int k_rs);
#else
extern "C" 
__declspec(dllimport) 
void init_rs(int n_rs, int k_rs);
#endif

/** RS- кодер
 * data[] - входной блок данных, символы четности помещаются в bb[]
 * k_rs -кол-во символов кодируемой информации
 * n_rs - длина кодового слова
 * n_rs - k_rs - кол-во символов четности
 */
#ifndef __cplusplus
__declspec(dllimport) 
int encode_rs(dtype *CodingData, dtype *bb,int n_rs,int k_rs);
#else
extern "C" 
__declspec(dllimport) 
int encode_rs(dtype *CodingData, dtype *bb,int n_rs,int k_rs);
#endif

/** RS- декодер
 * Декодируемый блок передается в data[] и может содержать ошибки 
 * данных. 
 *
 * Декодер исправляет символы на месте, если это возможно, и 
 * возвращает количество исправленных символов.
 * Если исправление невозможно, данные не изменяются и возвращается 
 * -1.
 *
 * CodedData - восстанавливаемые данные длиной k_rs символов (байт)
 * ParityBuf - символы четности длиной n_rs - k_rs байт
 * n_rs - длина кодового rs - слова 
 */
#ifndef __cplusplus
__declspec(dllimport) 
int decode_rs(dtype *CodedData, dtype *ParityBuf,  int n_rs, int k_rs);
#else
extern "C" 
__declspec(dllimport) 
int decode_rs(dtype *CodedData, dtype *ParityBuf,  int n_rs, int k_rs);
#endif
