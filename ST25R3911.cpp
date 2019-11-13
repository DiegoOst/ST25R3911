
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
 *  \brief ST25R3911 high level interface
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "ST25R3911.h"
#include "st25r3911_com.h"
#include "st25r3911_interrupt.h"



#include "stdio.h"
#include <limits.h>




/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static uint32_t st25r3911NoResponseTime_64fcs;
typedef uint16_t      ReturnCode; //eeeee.... st_errno.h
#ifdef ST25R391X_COM_SINGLETXRX
static uint8_t comBuf[ST25R3911_BUF_LEN];
#endif /* ST25R391X_COM_SINGLETXRX */

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static ReturnCode st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );


static inline void st25r3911CheckFieldSetLED(uint8_t val, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    if (ST25R3911_REG_OP_CONTROL_tx_en & val)
    {
    	fieldLED_06 -> write(1);

    }
    else
    {
    	fieldLED_06 -> write(0);
    }

}
/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*!
 *****************************************************************************
 *  \brief  Returns the content of a register within the ST25R3911
 *
 *  This function is used to read out the content of ST25R3911 registers.
 *
 *  \param[in]  reg: Address of register to read.
 *  \param[out] val: Returned value.
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *
 *****************************************************************************
 */

void ST25R3911::readRegister(uint8_t reg, uint8_t* val, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    uint8_t buf[2];
    uint8_t buf_rx[2];
    buf_rx[0] = 0;
    buf_rx[1] = 0;


    buf[0] = (reg | ST25R3911_READ_MODE);
    buf[1] = 0;

    IRQ->disable_irq();
    *gpio_cs = 0;
    wait_us(50);
    mspiChannel -> write((const char *)(buf) ,2 , (char*)(buf_rx) , 2);
    *gpio_cs = 1;
    IRQ->enable_irq();



    if(val != NULL)
    {
      *val = buf_rx[1];
    }

}

/*!
 *****************************************************************************
 *  \brief  Writes a given value to a register within the ST25R3911
 *
 *  This function is used to write \a val to address \a reg within the ST25R3911.
 *
 *  \param[in]  reg: Address of the register to write.
 *  \param[in]  val: Value to be written.
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */


void ST25R3911::writeRegister(uint8_t reg, uint8_t val, SPI * mspiChannel,  DigitalOut * gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    uint8_t buf[2];

    buf[0] = reg | ST25R3911_WRITE_MODE;
    buf[1] = val;

    if (ST25R3911_REG_OP_CONTROL == reg)
    {
        st25r3911CheckFieldSetLED(val, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }

    IRQ->disable_irq();
    *gpio_cs = 0;
    wait_us(50);
    mspiChannel -> write((const char *)(buf) ,2, NULL , NULL);
    *gpio_cs = 1;
    IRQ->enable_irq();


}



/*!
 *****************************************************************************
 *  \brief  Reads from multiple ST25R3911 registers
 *
 *  This function is used to read from multiple registers using the
 *  auto-increment feature. That is, after each read the address pointer
 *  inside the ST25R3911 gets incremented automatically.
 *
 *  \param[in]  reg: Address of the frist register to read from.
 *  \param[in]  values: pointer to a buffer where the result shall be written to.
 *  \param[in]  length: Number of registers to be read out.
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  mST25: Object of the ST25 chip
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */


void ST25R3911::readMultipleRegisters(uint8_t reg, uint8_t* val, uint8_t length, SPI * mspiChannel, ST25R3911* mST25,  DigitalOut * gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    uint8_t cmd = (reg | ST25R3911_READ_MODE);
    uint8_t index = 0;


    if (length > 0)
    {
        /* make this operation atomic */
    	for(index = 0; index < length; index ++)
    	{

        	mST25 -> readRegister(cmd + index, (val + index), mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    	}
    }


}

/*!
 *****************************************************************************
 *  \brief  Writes multiple values to ST25R3911 registers
 *
 *  This function is used to write multiple values to the ST25R3911 using the
 *  auto-increment feature. That is, after each write the address pointer
 *  inside the ST25R3911 gets incremented automatically.
 *
 *  \param[in]  reg: Address of the frist register to write.
 *  \param[in]  values: pointer to a buffer containing the values to be written.
 *  \param[in]  length: Number of values to be written.
 *  \param[in]  mST25: Object of the ST25 chip
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */

void ST25R3911::writeMultipleRegisters(uint8_t reg, const uint8_t* values, uint8_t length, SPI * mspiChannel, ST25R3911* mST25,  DigitalOut * gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    uint8_t cmd = (reg | ST25R3911_WRITE_MODE);
    uint8_t index = 0;

    if (reg <= ST25R3911_REG_OP_CONTROL && reg+length >= ST25R3911_REG_OP_CONTROL)
    {
        st25r3911CheckFieldSetLED(values[ST25R3911_REG_OP_CONTROL-reg], fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }

    if (length > 0)
    {
        /* make this operation atomic */
    	for(index = 0; index < length; index ++)
    	{
        	mST25 -> writeRegister(cmd + index, *(values + index), mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    	}
    }

}

/*!
 *****************************************************************************
 *  \brief  Execute a direct command
 *
 *  This function is used to start so-called direct command. These commands
 *  are implemented inside the chip and each command has unique code (see
 *  datasheet).
 *
 *  \param[in]  cmd : code of the direct command to be executed.
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */

void ST25R3911::executeCommand(uint8_t cmd, SPI* mspiChannel,  DigitalOut * gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    uint8_t reg[1];
    reg[0] = cmd;

    if ( cmd >= ST25R3911_CMD_TRANSMIT_WITH_CRC && cmd <= ST25R3911_CMD_RESPONSE_RF_COLLISION_0)
    {
    	fieldLED_06 -> write(0);
    }

    IRQ->disable_irq();
    *gpio_cs = 0;
    wait_us(50);
    mspiChannel -> write(( const char *)(reg), ST25R3911_CMD_LEN, NULL , NULL);
    *gpio_cs = 1;
    IRQ->enable_irq();


    return;
}

/*!
 *****************************************************************************
 *  \brief  Execute several direct commands
 *
 *  This function is used to start so-called direct command. These commands
 *  are implemented inside the chip and each command has unique code (see
 *  datasheet).
 *
 *  \param[in]  cmds   : codes of the direct command to be executed.
 *  \param[in]  length : number of commands to be executed
 *  \param[in]  mST25: Object of the ST25 chip
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */


void ST25R3911::executeCommands(uint8_t *cmds, uint8_t length, ST25R3911* mST25, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    uint8_t index;

 	for(index = 0; index < length; index++)
 	{
    	mST25 -> executeCommand(*cmds, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    	*cmds++;
 	}

    return;
}





/*!
 *****************************************************************************
 *  \brief  Writes values to ST25R3911 FIFO
 *
 *  This function needs to be called in order to write to the ST25R3911 FIFO.
 *
 *  \param[in]  values: pointer to a buffer containing the values to be written
 *                      to the FIFO.
 *  \param[in]  length: Number of values to be written.
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */



void ST25R3911::writeFifo(const uint8_t* val, uint8_t length, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    if (length > 0)
    {

        uint8_t cmd = ST25R3911_FIFO_LOAD;
        uint8_t values[length + 1];
        values[0] = cmd;
        for(int i = 0 ; i < length; i++)
        	values[i + 1] = *(val + i);

        IRQ->disable_irq();
        *gpio_cs = 0;
        wait_us(50);
        mspiChannel -> write( (const char *)values , length + 1, NULL , NULL);
        *gpio_cs = 1;
        IRQ->enable_irq();

    }

    return;
}

/*!
 *****************************************************************************
 *  \brief  Read values from ST25R3911 FIFO
 *
 *  This function needs to be called in order to read from ST25R3911 FIFO.
 *
 *  \param[out]  buf: pointer to a buffer where the FIFO content shall be
 *                       written to.
 *  \param[in]  length: Number of bytes to read. (= size of \a buf)
 *  \note: This function doesn't check whether \a length is really the
 *  number of available bytes in FIFO
 *  \param[in]  mspiChannel: Channel of the SPI
 *  \param[in]  gpio_cs: Chip Select of the SPI
 *  \param[in]  IRQ: Interrupt line. Not implemented, It is a polling application.
 *  \param[out]  fieldLED_01: Digital Output of the LED1
 *  \param[out]  fieldLED_02: Digital Output of the LED2
 *  \param[out]  fieldLED_03: Digital Output of the LED3
 *  \param[out]  fieldLED_04: Digital Output of the LED4
 *  \param[out]  fieldLED_05: Digital Output of the LED5
 *  \param[out]  fieldLED_06: Digital Output of the LED6
 *
 *****************************************************************************
 */

void ST25R3911::readFifo(uint8_t* buf, uint8_t length, SPI* mspiChannel, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    if (length > 0)
    {

        uint8_t cmd[2];
        cmd[0] = ST25R3911_FIFO_READ;
        cmd[1] = 0;

        IRQ->disable_irq();
        *gpio_cs = 0;
        wait_us(50);
        mspiChannel -> write((const char *)cmd, ST25R3911_CMD_LEN, NULL, NULL);
        mspiChannel -> write(NULL, NULL, (char *)buf, length);
        *gpio_cs = 1;
        IRQ->enable_irq();

    }



    return ;
}



////////////////////////////////
void st25r3911TxRxOn( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911SetRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911TxRxOff( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ClrRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}


void st25r3911OscOn( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t reg = 0;

    /* Make sure that oscillator is turned on and stable */
    mST25 -> readRegister( ST25R3911_REG_OP_CONTROL, &reg, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Use ST25R3911_REG_OP_CONTROL_en instead of ST25R3911_REG_AUX_DISPLAY_osc_ok to be on the safe side */
    if (!(ST25R3911_REG_OP_CONTROL_en & reg))
    {
        /* enable oscillator frequency stable interrupt */
        st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_OSC, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

        /* enable oscillator and regulator output */
        st25r3911ModifyRegister(ST25R3911_REG_OP_CONTROL, 0x00, ST25R3911_REG_OP_CONTROL_en, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

        /* wait for the oscillator interrupt */
        st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_OSC, ST25R3911_OSC_STABLE_TIMEOUT, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_OSC, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }

} /* st25r3911OscOn() */


void st25r3911Initialize(SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{


	uint16_t vdd_mV;



    /* first, reset the st25r3911 */
    mST25 -> executeCommand(ST25R3911_CMD_SET_DEFAULT, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;


    /* enable pull downs on miso line */
    st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2, 0,
            ST25R3911_REG_IO_CONF2_miso_pd1 |
            ST25R3911_REG_IO_CONF2_miso_pd2, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* after reset all interrupts are enabled. so disable them at first */
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    /* and clear them, just to be sure... */
    st25r3911ClearInterrupts( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* trim settings for VHBR board, will anyway changed later on */
    mST25 -> writeRegister(ST25R3911_REG_ANT_CAL_TARGET, 0x80, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    st25r3911OscOn( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Measure vdd and set sup3V bit accordingly */
    vdd_mV = st25r3911MeasureVoltage(ST25R3911_REG_REGULATOR_CONTROL_mpsv_vdd, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2,
                         ST25R3911_REG_IO_CONF2_sup3V,
                         (vdd_mV < 3600)?ST25R3911_REG_IO_CONF2_sup3V:0, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Make sure Transmitter and Receiver are disabled */
    st25r3911TxRxOff( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return;
}

void st25r3911Deinitialize(SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Disabe Tx and Rx, Keep OSC */
    st25r3911TxRxOff( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return;
}

ReturnCode st25r3911AdjustRegulators(uint16_t* result_mV, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t result;
    uint8_t io_conf2;
    ReturnCode err = ERR_NONE;

    /* first check the status of the reg_s bit in ST25R3911_REG_VREG_DEF register.
       if this bit is set adjusting the regulators is not allowed */
    mST25 -> readRegister(ST25R3911_REG_REGULATOR_CONTROL, &result, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    if (result & ST25R3911_REG_REGULATOR_CONTROL_reg_s)
    {
        return ERR_REQUEST;
    }

    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_ADJUST_REGULATORS,
                                    ST25R3911_REG_REGULATOR_RESULT,
                                    5,
                                    &result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    mST25 -> readRegister(ST25R3911_REG_IO_CONF2, &io_conf2, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    result >>= ST25R3911_REG_REGULATOR_RESULT_shift_reg;
    result -= 5;
    if (result_mV)
    {
        if(io_conf2 & ST25R3911_REG_IO_CONF2_sup3V)
        {
            *result_mV = 2400;
            *result_mV += result * 100;
        }
        else
        {
            *result_mV = 3900;
            *result_mV += result * 120;
        }
    }
    return err;
}

void st25r3911MeasureRF(uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_AMPLITUDE,
                                    ST25R3911_REG_AD_RESULT,
                                    10,
                                    result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911MeasureCapacitance(uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_CAPACITANCE,
                                    ST25R3911_REG_AD_RESULT,
                                    10,
                                    result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911MeasureAntennaResonance(uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_PHASE,
                                    ST25R3911_REG_AD_RESULT,
                                    10,
                                    result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911CalibrateAntenna(uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_ANTENNA,
                                    ST25R3911_REG_ANT_CAL_RESULT,
                                    10,
                                    result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}

void st25r3911CalibrateModulationDepth(uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_MODULATION,
                                    ST25R3911_REG_AM_MOD_DEPTH_RESULT,
                                    10,
                                    result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}


void st25r3911CalibrateCapacitiveSensor(uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_C_SENSOR,
                                    ST25R3911_REG_CAP_SENSOR_RESULT,
                                    10,
                                    result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}


ReturnCode st25r3911SetBitrate(uint8_t txRate, uint8_t rxRate, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t reg;

       mST25 -> readRegister(ST25R3911_REG_BIT_RATE, &reg, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    if (rxRate != ST25R3911_BR_DO_NOT_SET)
    {
        if(rxRate > ST25R3911_BR_3390)
        {
            return ERR_PARAM;
        }
        else
        {
            reg &= ~ST25R3911_REG_BIT_RATE_mask_rxrate;
            reg |= rxRate << ST25R3911_REG_BIT_RATE_shift_rxrate;
        }
    }
    if (txRate != ST25R3911_BR_DO_NOT_SET)
    {
        if(txRate > ST25R3911_BR_6780)
        {
            return ERR_PARAM;
        }
        else
        {
            reg &= ~ST25R3911_REG_BIT_RATE_mask_txrate;
            reg |= txRate<<ST25R3911_REG_BIT_RATE_shift_txrate;
        }
    }
    mST25 -> writeRegister(ST25R3911_REG_BIT_RATE, reg, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return ERR_NONE;
}


uint16_t st25r3911MeasureVoltage(uint8_t mpsv,SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t result;
    uint16_t mV;

    mpsv &= ST25R3911_REG_REGULATOR_CONTROL_mask_mpsv;

    st25r3911ModifyRegister(ST25R3911_REG_REGULATOR_CONTROL,
                         ST25R3911_REG_REGULATOR_CONTROL_mask_mpsv,
                         mpsv, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_VDD,
            ST25R3911_REG_AD_RESULT,
            100,
            &result, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 );
    mV = ((uint16_t)result) * 23;
    mV += ((((uint16_t)result) * 438) + 500) / 1000;

    return mV;
}


uint8_t st25r3911GetNumFIFOLastBits(  SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t  reg;
    
    mST25 ->  readRegister( ST25R3911_REG_FIFO_RX_STATUS2, &reg, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    
    return ((reg & ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb) >> ST25R3911_REG_FIFO_RX_STATUS2_shift_fifo_lb);
}

uint32_t st25r3911GetNoResponseTime_64fcs()
{
    return st25r3911NoResponseTime_64fcs;
}

void st25r3911StartGPTimer_8fcs(uint16_t gpt_8fcs, uint8_t trigger_source, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    st25r3911SetGPTime_8fcs(gpt_8fcs, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL,
            ST25R3911_REG_GPT_CONTROL_gptc_mask,
            trigger_source, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    if (!trigger_source)
    	mST25 -> executeCommand(ST25R3911_CMD_START_GP_TIMER, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return;
}

void st25r3911SetGPTime_8fcs(uint16_t gpt_8fcs, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
	mST25 -> writeRegister(ST25R3911_REG_GPT1, gpt_8fcs >> 8, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
	mST25 -> writeRegister(ST25R3911_REG_GPT2, gpt_8fcs & 0xff, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return;
}

bool st25r3911CheckReg( uint8_t reg, uint8_t mask, uint8_t val, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t regVal;

    regVal = 0;
    mST25 -> readRegister( reg, &regVal, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return ((regVal & mask) == val );
}


bool st25r3911CheckChipID( uint8_t *rev, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t ID;


    mST25 -> readRegister(ST25R3911_REG_IC_IDENTITY, &ID, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Check if IC Identity Register contains ST25R3911's IC type code */
    if( (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_type)  != (ST25R3911_REG_IC_IDENTITY_ic_type) )
    {
        return false;
    }


    if(rev != NULL)
    {
        *rev = (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_rev);
    }
    
    return true;
}

ReturnCode st25r3911SetNoResponseTime_64fcs(uint32_t nrt_64fcs, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    ReturnCode err = ERR_NONE;
    uint8_t nrt_step = 0;

    st25r3911NoResponseTime_64fcs = nrt_64fcs;
    if (nrt_64fcs > USHRT_MAX)
    {
        nrt_step = ST25R3911_REG_GPT_CONTROL_nrt_step;
        nrt_64fcs = (nrt_64fcs + 63) / 64;
        if (nrt_64fcs > USHRT_MAX)
        {
            nrt_64fcs = USHRT_MAX;
            err = ERR_PARAM;
        }
        st25r3911NoResponseTime_64fcs = 64 * nrt_64fcs;
    }

    st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_step, nrt_step, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    mST25 -> writeRegister(ST25R3911_REG_NO_RESPONSE_TIMER1, nrt_64fcs >> 8, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    mST25 -> writeRegister(ST25R3911_REG_NO_RESPONSE_TIMER2, nrt_64fcs & 0xff, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return err;
}

ReturnCode st25r3911SetStartNoResponseTime_64fcs(uint32_t nrt_64fcs, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    ReturnCode err;

    err = st25r3911SetNoResponseTime_64fcs( nrt_64fcs, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    if(err == ERR_NONE)
    {
    	  mST25 -> executeCommand(ST25R3911_CMD_START_NO_RESPONSE_TIMER, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }

    return err;
}

ReturnCode st25r3911PerformCollisionAvoidance( uint8_t FieldONCmd, uint8_t pdThreshold, uint8_t caThreshold, uint8_t nTRFW, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t  treMask;
    uint32_t irqs;

    if( (FieldONCmd != ST25R3911_CMD_INITIAL_RF_COLLISION)    &&
        (FieldONCmd != ST25R3911_CMD_RESPONSE_RF_COLLISION_0) &&
        (FieldONCmd != ST25R3911_CMD_RESPONSE_RF_COLLISION_N)   )
    {
        return ERR_PARAM;
    }

    /* Check if new thresholds are to be applied */
    if( (pdThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) || (caThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) )
    {
        treMask = 0;

        if(pdThreshold != ST25R3911_THRESHOLD_DO_NOT_SET)
        {
            treMask |= ST25R3911_REG_FIELD_THRESHOLD_mask_trg;
        }

        if(caThreshold != ST25R3911_THRESHOLD_DO_NOT_SET)
        {
            treMask |= ST25R3911_REG_FIELD_THRESHOLD_mask_rfe;
        }

        /* Set Detection Threshold and|or Collision Avoidance Threshold */
        st25r3911ChangeRegisterBits( ST25R3911_REG_FIELD_THRESHOLD, treMask, (pdThreshold & ST25R3911_REG_FIELD_THRESHOLD_mask_trg) | (caThreshold & ST25R3911_REG_FIELD_THRESHOLD_mask_rfe ), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }
    
    /* Set n x TRFW */
    st25r3911ModifyRegister(ST25R3911_REG_AUX, ST25R3911_REG_AUX_mask_nfc_n, (nTRFW & ST25R3911_REG_AUX_mask_nfc_n), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Enable and clear CA specific interrupts and execute command */
    st25r3911EnableInterrupts( (ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    mST25 -> executeCommand(FieldONCmd, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    irqs = st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT, ST25R3911_CA_TIMEOUT, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Clear any previous External Field events and disable CA specific interrupts */
    st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_EOF | ST25R3911_IRQ_MASK_EON), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    st25r3911DisableInterrupts((ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 );


    if(ST25R3911_IRQ_MASK_CAC & irqs)                             /* Collision occurred */
    {
        return ERR_RF_COLLISION;
    }

    if(ST25R3911_IRQ_MASK_CAT & irqs)                             /* No Collision detected, Field On */
    {
        return ERR_NONE;
    }

    /* No interrupt detected */
    return ERR_INTERNAL;
}

ReturnCode st25r3911GetRegsDump(uint8_t* resRegDump, uint8_t* sizeRegDump, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t regIt;
    uint8_t regDump[ST25R3911_REG_IC_IDENTITY+1];
    
    if(!sizeRegDump || !resRegDump)
    {
        return ERR_PARAM;
    }
    
    for( regIt = ST25R3911_REG_IO_CONF1; regIt < SIZEOF_ARRAY(regDump); regIt++ )
    {
          mST25 -> readRegister(regIt, &regDump[regIt], mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }
    
    *sizeRegDump = MIN(*sizeRegDump, regIt);
    ST_MEMCPY(resRegDump, regDump, *sizeRegDump );

    return ERR_NONE;
}


void st25r3911SetNumTxBits( uint32_t nBits, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
	mST25 -> writeRegister(ST25R3911_REG_NUM_TX_BYTES2, (uint8_t)((nBits >> 0) & 0xff), mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
	mST25 -> writeRegister(ST25R3911_REG_NUM_TX_BYTES1, (uint8_t)((nBits >> 8) & 0xff), mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}


bool st25r3911IsCmdValid( uint8_t cmd )
{
    if( !((cmd >= ST25R3911_CMD_SET_DEFAULT)       && (cmd <= ST25R3911_CMD_ANALOG_PRESET))           &&
        !((cmd >= ST25R3911_CMD_MASK_RECEIVE_DATA) && (cmd <= ST25R3911_CMD_CLEAR_RSSI))              &&
        !((cmd >= ST25R3911_CMD_TRANSPARENT_MODE)  && (cmd <= ST25R3911_CMD_START_NO_RESPONSE_TIMER)) &&
        !((cmd >= ST25R3911_CMD_TEST_CLEARA)       && (cmd <= ST25R3911_CMD_FUSE_PPROM))               )
    {
        return false;
    }
    return true;
}

ReturnCode st25r3911StreamConfigure(const struct st25r3911StreamConfig *config, SPI* mspiChannel, ST25R3911* mST25,  DigitalOut * gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
    uint8_t smd = 0;
    uint8_t mode;

    if (config->useBPSK)
    {
        mode = ST25R3911_REG_MODE_om_bpsk_stream;
        if (config->din<2 || config->din>4) /* not in fc/4 .. fc/16 */
        {
            return ERR_PARAM;
        }
        smd |= (4 - config->din) << ST25R3911_REG_STREAM_MODE_shift_scf;

    }
    else
    {
        mode = ST25R3911_REG_MODE_om_subcarrier_stream;
        if (config->din<3 || config->din>6) /* not in fc/8 .. fc/64 */
        {
            return ERR_PARAM;
        }
        smd |= (6 - config->din) << ST25R3911_REG_STREAM_MODE_shift_scf;
        if (config->report_period_length == 0)
        {
            return ERR_PARAM;
        }
    }

    if (config->dout<1 || config->dout>7) /* not in fc/2 .. fc/128 */
    {
        return ERR_PARAM;
    }
    smd |= (7 - config->dout) << ST25R3911_REG_STREAM_MODE_shift_stx;

    if (config->report_period_length > 3)
    {
        return ERR_PARAM;
    }
    smd |= config->report_period_length << ST25R3911_REG_STREAM_MODE_shift_scp;

    mST25 -> writeRegister(ST25R3911_REG_STREAM_MODE, smd, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    st25r3911ChangeRegisterBits(ST25R3911_REG_MODE, ST25R3911_REG_MODE_mask_om, mode, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return ERR_NONE;
}

bool st25r3911IrqIsWakeUpCap( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  return ( st25r3911GetInterrupt( ST25R3911_IRQ_MASK_WCAP, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 )  ? true : false );
}


bool st25r3911IrqIsWakeUpPhase( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  return ( st25r3911GetInterrupt( ST25R3911_IRQ_MASK_WPH, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 )  ? true : false );
}


bool st25r3911IrqIsWakeUpAmplitude( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  return ( st25r3911GetInterrupt( ST25R3911_IRQ_MASK_WAM, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 )  ? true : false );
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
/*!
 *****************************************************************************
 *  \brief  Executes a direct command and returns the result
 *
 *  This function executes the direct command given by \a cmd waits for
 *  \a sleeptime and returns the result read from register \a resreg.
 *
 *  \param[in] cmd: direct command to execute.
 *  \param[in] resreg: Address of the register containing the result.
 *  \param[in] sleeptime: time in milliseconds to wait before reading the result.
 *  \param[out] result: 8 bit long result
 *
 *****************************************************************************
 */
static ReturnCode st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t* result, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{

    if (   (cmd >= ST25R3911_CMD_INITIAL_RF_COLLISION && cmd <= ST25R3911_CMD_RESPONSE_RF_COLLISION_0)
            || (cmd == ST25R3911_CMD_MEASURE_AMPLITUDE)
            || (cmd >= ST25R3911_CMD_ADJUST_REGULATORS && cmd <= ST25R3911_CMD_MEASURE_PHASE)
            || (cmd >= ST25R3911_CMD_CALIBRATE_C_SENSOR && cmd <= ST25R3911_CMD_MEASURE_VDD)
            || (cmd >= 0xFD && cmd <= 0xFE )
       )
    {
        st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_DCT, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        st25r3911GetInterrupt(ST25R3911_IRQ_MASK_DCT, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        mST25 -> executeCommand(cmd, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_DCT, sleeptime, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_DCT, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }
    else
    {
        return ERR_PARAM;
    }

    /* read out the result if the pointer is not NULL */
    if (result)
    	      mST25 -> readRegister(resreg, result, mspiChannel,  gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    return ERR_NONE;

}
