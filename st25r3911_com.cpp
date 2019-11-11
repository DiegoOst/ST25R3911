
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
 *  \brief Implementation of ST25R3911 communication.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "st25r3911_com.h"






/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

void st25r3911ReadTestRegister(uint8_t reg, uint8_t* val, ST25R3911* mST25, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{


//	uint8_t rx;
  // val rx
    mST25 -> executeCommand(ST25R3911_CMD_TEST_ACCESS, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    mST25 -> readRegister( reg, val, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;


    return;
}

void st25r3911WriteTestRegister(uint8_t reg, uint8_t val, ST25R3911* mST25, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{


    mST25 -> executeCommand(ST25R3911_CMD_TEST_ACCESS, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    mST25 -> writeRegister( reg,  val, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 );


    return;
}


void st25r3911ClrRegisterBits( uint8_t reg, uint8_t clr_mask, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t tmp;

    mST25 -> readRegister( reg,  &tmp, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    tmp &= ~clr_mask;
    mST25 -> writeRegister( reg,  tmp, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    
    return;
}


void st25r3911SetRegisterBits( uint8_t reg, uint8_t set_mask, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t tmp;

    mST25 -> readRegister( reg,  &tmp, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    tmp |= set_mask;
    mST25 -> writeRegister( reg,  tmp, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    
    return;
}

void st25r3911ChangeRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ModifyRegister(reg, valueMask, (valueMask & value), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911ModifyRegister(uint8_t reg, uint8_t clr_mask, uint8_t set_mask, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t tmp;

    mST25 -> readRegister( reg,  &tmp, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* mask out the bits we don't want to change */
    tmp &= ~clr_mask;
    /* set the new value */
    tmp |= set_mask;
    mST25 -> writeRegister( reg,  tmp, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return;
}

void st25r3911ChangeTestRegisterBits( uint8_t reg, uint8_t valueMask, uint8_t value, ST25R3911* mST25, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t    rdVal;
    uint8_t    wrVal;
    
    /* Read current reg value */
    st25r3911ReadTestRegister(reg, &rdVal, mST25, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    
    /* Compute new value */
    wrVal  = (rdVal & ~valueMask);
    wrVal |= (value & valueMask);
    
    /* Write new reg value */
    st25r3911WriteTestRegister(reg, wrVal, mST25, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    
    return;
}



bool st25r3911IsRegValid( uint8_t reg )
{
    if( !(( (int8_t)reg >= ST25R3911_REG_IO_CONF1) && (reg <= ST25R3911_REG_CAPACITANCE_MEASURE_RESULT)) &&
        (reg != ST25R3911_REG_IC_IDENTITY)                                                         )
    {
        return false;
    }
    return true;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

