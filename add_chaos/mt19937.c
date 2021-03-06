/* A C-program for MT19937: Real number version (1999/10/28)    */
/*   genrand() generates one pseudorandom real number (double)  */
/* which is uniformly distributed on [0,1]-interval, for each   */
/* call. sgenrand(seed) sets initial values to the working area */
/* of 624 words. Before genrand(), sgenrand(seed) must be       */
/* called once. (seed is any 32-bit integer.)                   */
/* Integer generator is obtained by modifying two lines.        */
/*   Coded by Takuji Nishimura, considering the suggestions by  */
/* Topher Cooper and Marc Rieffel in July-Aug. 1997.            */

/* This library is free software; you can redistribute it and/or   */
/* modify it under the terms of the GNU Library General Public     */
/* License as published by the Free Software Foundation; either    */
/* version 2 of the License, or (at your option) any later         */
/* version.                                                        */
/* This library is distributed in the hope that it will be useful, */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of  */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.            */
/* See the GNU Library General Public License for more details.    */
/* You should have received a copy of the GNU Library General      */
/* Public License along with this library; if not, write to the    */
/* Free Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA   */
/* 02111-1307  USA                                                 */

/* Copyright (C) 1997, 1999 Makoto Matsumoto and Takuji Nishimura. */
/* Any feedback is very welcome. For any question, comments,       */
/* see http://www.math.keio.ac.jp/matumoto/emt.html or email       */
/* matumoto@math.keio.ac.jp                                        */

/* REFERENCE                                                       */
/* M. Matsumoto and T. Nishimura,                                  */
/* "Mersenne Twister: A 623-Dimensionally Equidistributed Uniform  */
/* Pseudo-Random Number Generator",                                */
/* ACM Transactions on Modeling and Computer Simulation,           */
/* Vol. 8, No. 1, January 1998, pp 3--30.                          */

#ifndef MT_19937_C
#define MT_19937_C

#include "mt19937.h"

void initialize_mersenne(unsigned long seed)
{
    int i;
    mt19937_state_t* m = &mt19937_state;
    for (i = 0; i < __N__; i++)
    {
        m->mt[i] = seed & 0xffff0000;
        seed = 69069 * seed + 1;
        m->mt[i] |= (seed & 0xffff0000) >> 16;
        seed = 69069 * seed + 1;
    }
    m->mti = __N__;
}

unsigned long get_rand_uint()
{
    unsigned long y;
    unsigned long mag01[2] =  {0, MATRIX_A };
    mt19937_state_t* m = &mt19937_state;

    /* mag01[x] = x * MATRIX_A  for x=0,1 */

    if (m->mti >= __N__)
    { /* generate N words at one time */
        int kk = 0;
        while (kk < __N__ - __M__)
        {
            y = (m->mt[kk] & UPPER_MASK) | (m->mt[kk + 1] & LOWER_MASK);
            m->mt[kk] = m->mt[kk + __M__] ^ (y >> 1) ^ mag01[y & 0x1];
            ++kk;
        }
        while (kk < __N__ - 1)
        {
            y = (m->mt[kk] & UPPER_MASK) | (m->mt[kk + 1] & LOWER_MASK);
            m->mt[kk] = m->mt[kk + (__M__ - __N__)] ^ (y >> 1) ^ mag01[y & 0x1];
            ++kk;
        }

        y = (m->mt[__N__ - 1] & UPPER_MASK) | (m->mt[0] & LOWER_MASK);
        m->mt[__N__ - 1] = m->mt[__M__ - 1] ^ (y >> 1) ^ mag01[y & 0x1];

        m->mti = 0;
    }

    y = m->mt[m->mti++];
    y ^= TEMPERING_SHIFT_U(y);
    y ^= TEMPERING_SHIFT_S(y) & TEMPERING_MASK_B;
    y ^= TEMPERING_SHIFT_T(y) & TEMPERING_MASK_C;
    y ^= TEMPERING_SHIFT_L(y);
    return y;
}

__declspec(dllexport)
float get_rand_float()
{
    mt19937_state_t* m = &mt19937_state;
    return get_rand_uint(m) * 2.3283064e-10f;
}

double get_rand_double()
{
    mt19937_state_t* m = &mt19937_state;
    return get_rand_uint(m) * 2.3283064370807974e-10;
}

#endif
