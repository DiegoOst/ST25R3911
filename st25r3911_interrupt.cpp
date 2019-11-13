
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
 *      PROJECT:   ST25R3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file
 *
 *  \author Ulrich Herrmann
 *
 *  \brief ST25R3911 Interrupt handling
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "st25r3911_interrupt.h"
#include "st25r3911_com.h"
#include "ST25R3911.h"
#include "st_errno.h"
#include "utils.h"
#include "rfal_rf.h"


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/*! Length of the interrupt registers       */
#define ST25R3911_INT_REGS_LEN          ( (ST25R3911_REG_IRQ_ERROR_WUP - ST25R3911_REG_IRQ_MAIN) + 1 )

/*
 ******************************************************************************
 * LOCAL DATA TYPES
 ******************************************************************************
 */

/*! Holds current and previous interrupt callback pointer as well as current Interrupt status and mask */
typedef struct s_st25r3911Interrupt
{
    void      (*prevCallback)(); /*!< call back function for 3911 interrupt               */
    void      (*callback)();     /*!< call back function for 3911 interrupt               */
    uint32_t  status;            /*!< latest interrupt status                             */
    uint32_t  mask;              /*!< Interrupt mask. Negative mask = ST25R3911 mask regs */
}t_st25r3911Interrupt;

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/

volatile t_st25r3911Interrupt st25r3911interrupt; /*!< Instance of ST25R3911 interrupt*/

extern volatile bool wakeupFlag;
/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void st25r3911InitInterrupts( DigitalOut* fieldLED_06 )
{

    
    st25r3911interrupt.callback     = NULL;
    st25r3911interrupt.prevCallback = NULL;
    st25r3911interrupt.status       = 0;
    st25r3911interrupt.mask         = 0;
    
   fieldLED_06 -> write(0);

}

void st25r3911Isr( SPI * mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911CheckForReceivedInterrupts(mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    if (NULL != st25r3911interrupt.callback)
        st25r3911interrupt.callback;
}

void st25r3911CheckForReceivedInterrupts( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t  iregs[ST25R3911_INT_REGS_LEN];
    uint32_t irqStatus;

    ST_MEMSET( iregs, (uint8_t)ST25R3911_IRQ_MASK_ALL, ST25R3911_INT_REGS_LEN );
        


       mST25 -> readMultipleRegisters(ST25R3911_REG_IRQ_MAIN, iregs, sizeof(iregs), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

       if (iregs[0] & ST25R3911_IRQ_MASK_TXE)
       {
    	   fieldLED_06 -> write(1);
       }



       irqStatus  = (uint32_t)iregs[0];
       irqStatus |= (uint32_t)iregs[1]<<8;
       irqStatus |= (uint32_t)iregs[2]<<16;
       /* forward all interrupts, even masked ones to application. */
       st25r3911interrupt.status |= irqStatus;

}


void st25r3911ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask,SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    int i;
    uint32_t old_mask;
    uint32_t new_mask;

    old_mask = st25r3911interrupt.mask;
    new_mask = (~old_mask & set_mask) | (old_mask & clr_mask);
    st25r3911interrupt.mask &= ~clr_mask;
    st25r3911interrupt.mask |= set_mask;
    for (i=0; i<3 ; i++)
    { 
        if (! ((new_mask >> (8*i)) & 0xff)) continue;
        mST25 -> writeRegister(ST25R3911_REG_IRQ_MASK_MAIN + i, (st25r3911interrupt.mask>>(8*i))&0xff, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }
    return;
}


uint32_t st25r3911WaitForInterruptsTimed(uint32_t mask, uint16_t tmo, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint32_t tmr;
    uint32_t status;
   
    tmr = platformTimerCreate(tmo);
    do 
    {
    	st25r3911Isr(mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        status = st25r3911interrupt.status & mask;
    } while ((!status) && !platformTimerIsExpired(tmr));

    status = st25r3911interrupt.status & mask;
    
    st25r3911interrupt.status &= ~status;

    
    return status;
}


uint32_t st25r3911GetInterrupt(uint32_t mask, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
	st25r3911Isr(mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    mask &= st25r3911interrupt.status;

    if (mask)
    {
        st25r3911interrupt.status &= ~mask;

    }
    return mask;
}

void st25r3911EnableInterrupts(uint32_t mask,SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ModifyInterrupts(mask,0, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911DisableInterrupts(uint32_t mask,SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ModifyInterrupts(0,mask, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911ClearInterrupts( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t iregs[3];

    mST25 -> readMultipleRegisters(ST25R3911_REG_IRQ_MAIN, iregs, 3, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    st25r3911interrupt.status = 0;


    return;
}

void st25r3911IRQCallbackSet( void (*cb)() )
{
    st25r3911interrupt.prevCallback = st25r3911interrupt.callback;
    st25r3911interrupt.callback     = cb;
}

void st25r3911IRQCallbackRestore( void )
{
    st25r3911interrupt.callback     = st25r3911interrupt.prevCallback;
    st25r3911interrupt.prevCallback = NULL;
}

uint8_t st25r3911ChipHasWoke( void )
{
	uint8_t flagWake = 0;

	 if( (st25r3911interrupt.status & 0x0001) == 1 )
	 {
		flagWake = 1;
	 }
	 return flagWake;
}


