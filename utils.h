
/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/


/*
 *      PROJECT:   NFCC firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file
 *
 *  \author Ulrich Herrmann
 *
 *  \brief Common and helpful macros
 *
 */

#ifndef UTILS_H
#define UTILS_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <string.h>

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
/*!
 * this macro evaluates an error variable \a ERR against an error code \a EC.
 * in case it is not equal it jumps to the given label \a LABEL.
 */
#define EVAL_ERR_NE_GOTO(EC, ERR, LABEL)                                   \
    if (EC != ERR) goto LABEL;

/*!
 * this macro evaluates an error variable \a ERR against an error code \a EC.
 * in case it is equal it jumps to the given label \a LABEL.
 */
#define EVAL_ERR_EQ_GOTO(EC, ERR, LABEL)                                   \
    if (EC == ERR) goto LABEL;

#define SIZEOF_ARRAY(a)     (sizeof(a) / sizeof(a[0]))    /*!< Compute the size of an array           */
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))    /*!< Return the maximum of the 2 values     */
#define MIN(a, b)           (((a) < (b)) ? (a) : (b))    /*!< Return the minimum of the 2 values     */
#define BITMASK_1           (0x01)                        /*!< Bit mask for lsb bit                   */
#define BITMASK_2           (0x03)                        /*!< Bit mask for two lsb bits              */
#define BITMASK_3           (0x07)                        /*!< Bit mask for three lsb bits            */
#define BITMASK_4           (0x0F)                        /*!< Bit mask for four lsb bits             */
#define U16TOU8(a)          ((a) & 0x00FF)                /*!< Cast 16-bit unsigned to 8-bit unsigned */
#define GETU16(a)           (uint16_t)((a[0] << 8) | a[1])/*!< Cast two Big Endian 8-bits byte array to 16-bits unsigned */


#define REVERSE_BYTES(pData, nDataSize) \
  unsigned char swap, *lo = pData, *hi = pData + nDataSize - 1; \
  while (lo < hi) { swap = *lo; *lo++ = *hi; *hi-- = swap; }


#define ST_MEMMOVE          memmove     /*!< map memmove to string library code */
#define ST_MEMCPY           memcpy      /*!< map memcpy to string library code  */
#define ST_MEMSET           memset      /*!< map memset to string library code  */
#define ST_BYTECMP          memcmp      /*!< map bytecmp to string library code */

#define NO_WARNING(v)      ((void) (v)) /*!< Macro to suppress compiler warning */


#ifndef NULL
  #define NULL (void*)0                 /*!< represents a NULL pointer */
#endif /* !NULL */

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

#endif /* UTILS_H */

