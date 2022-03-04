/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "math.h"

// #include "MAX31865_lib.c"            // I want to make own library MAX31865_lib.c for using with nRF52
#include "MAX31865_lib.h"

/*------The characteristics of the PT100 sensor type and the reference resistor connected to the MAX31865------*/
#define PT100_R0 (double)100.0 		//Resistance of the PT100 sensor, at 0 °C
#define Rref (double)400.0		//Resistance of the reference resistor connected to the MAX31865
/*------The characteristics of the PT100 sensor type and the reference resistor connected to the MAX31865------*/


#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)           // From SPI example
{
    spi_xfer_done = true;
    //NRF_LOG_INFO("Transfer completed.");
    //if (m_rx_buf[0] != 0)
    //{
    //    NRF_LOG_INFO(" Received:");
    //    NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    //}
}

void spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.ss_pin     = SPI_SS_PIN;               // 31   P0.31   (CS on MAX31865)
    spi_config.miso_pin   = SPI_MISO_PIN;             // 30   P0.30   (SDO on MAX)
    spi_config.mosi_pin   = SPI_MOSI_PIN;             // 29   P0.29   (SDI on MAX)
    spi_config.sck_pin    = SPI_SCK_PIN;              // 26   P0.26   (CLK on MAX)

    spi_config.mode       = NRF_DRV_SPI_MODE_1;       // set up from datasheet (it is able to use spi mode 1 or 3)
    spi_config.frequency  = NRF_DRV_SPI_FREQ_1M;      // max frequency is 5 MHz
    spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}


/**************************Function declaration****************************/
void spi_init(void);
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);

bool begin(uint8_t wires);

uint8_t readFault(void);
void clearFault(void);
uint16_t readRTD();

void setWires(uint8_t wires);
void autoConvert(bool b);
void enable50Hz(bool b);
void enableBias(bool b);

float temperature(float RTDnominal, float refResistor);

void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
uint8_t readRegister8(uint8_t addr);
uint16_t readRegister16(uint8_t addr);
void writeRegister8(uint8_t addr, uint8_t reg);
/**************************Function declaration****************************/


float     PT100_Temperature = 0.0f;   // actual temperature from sensor
uint16_t  RTD = 0;        // RTD value
uint8_t   fault_state;    // variable for fault code

uint8_t test;             // variable for testing

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);          // Initialization of board LEDs

    
    spi_init();                             // Initialization of SPI

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\n\n\n");
    NRF_LOG_INFO("SPI example started.");

    //begin(0);                               // Initialization of MAX31865 -> 0 means 2 wire measuring with PT100

    

    while (1)
    {
        //for (uint8_t i = 0; i <= 0xFF; i++)
        //{   
        //    NRF_LOG_INFO("Transfered: %d", i)
        //    NRF_LOG_HEXDUMP_INFO(i, strlen((const char *)i));
        //    fault_state = readRegister16(i);

        //    NRF_LOG_INFO("Received: %d", fault_state);
        //    NRF_LOG_HEXDUMP_INFO(fault_state, strlen((const char *)fault_state));
        //    nrf_delay_ms(1);
        //    NRF_LOG_INFO("")
        //}

        bsp_board_led_invert(BSP_BOARD_LED_0);      // change state of LED 1 

        NRF_LOG_INFO("Fault: ");
        fault_state = readFault();                  // read the fault of MAX31865
        //TODO //NRF_LOG_HEXDUMP_INFO(fault_state, 1);  // TODO fce -> itoa int to string
        
        RTD = readRTD();                            // read the RTD value from MAX31865

        NRF_LOG_INFO("RTD value: %d", RTD);
        
        PT100_Temperature = temperature(Rref, PT100_R0);    // read the temperature from MAX31865
        NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(PT100_Temperature));

        nrf_delay_ms(1000);
    }
}



/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool begin(uint8_t wires)
{
    setWires(wires);

    //uint8_t test;

    //test = readRegister8(0x00);
    //NRF_LOG_INFO("Output value: %d", test);

    //test = readRegister8(0x03);
    //NRF_LOG_INFO("Output value: %d", test);

    enableBias(false);
    autoConvert(false);
    clearFault();

    NRF_LOG_INFO("Initialization of MAX31865 was succesful");     // Without testing so, it could not be true... 
    return true;
}


/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void setWires(uint8_t wires) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (wires == MAX31865_3WIRE) {
      t |= MAX31865_CONFIG_3WIRE;
    } else {
      // 2 or 4 wire
      t &= ~MAX31865_CONFIG_3WIRE;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void enableBias(bool b) 
{
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void autoConvert(bool b) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
      t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
    } else {
      t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void clearFault(void) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t &= ~0x2C;
    t |= MAX31865_CONFIG_FAULTSTAT;
    writeRegister8(MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t readFault(void) {
    return readRegister8(MAX31865_FAULTSTAT_REG);
}


/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/
void enable50Hz(bool b) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
      t |= MAX31865_CONFIG_FILT50HZ;
    } else {
      t &= ~MAX31865_CONFIG_FILT50HZ;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float temperature(float RTDnominal,
                  float refResistor) 
{
    float Z1, Z2, Z3, Z4, Rt, temp;

    Rt = readRTD();
    Rt /= 32768;
    Rt *= refResistor;

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / RTDnominal;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;

    if (temp >= 0)
      return temp;

    // ugh.
    Rt /= RTDnominal;
    Rt *= 100; // normalize to 100 ohm

    float rpoly = Rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt; // ^5
    temp += 1.5243e-10 * rpoly;

    return temp;
}


/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t readRTD(void) 
{
    clearFault();
    enableBias(true);
    nrf_delay_ms(10);
    uint8_t t;
    t = readRegister8(MAX31865_CONFIG_REG);
    t |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, t);
    nrf_delay_ms(65);

    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);

    enableBias(false); // Disable bias current again to reduce selfheating of PT100.

    // remove fault
    rtd >>= 1;

    return rtd;
}



void readRegisterN( uint8_t addr, 
                    uint8_t *buffer,
                    uint8_t n)
{
    addr &= 0x7F;                                   // make sure top bit is not set

    //nrf_drv_spi_xfer_desc_t   xfer_desc;
    //xfer_desc.p_tx_buffer = &addr;
    //xfer_desc.tx_length   = sizeof(addr);
    //xfer_desc.p_rx_buffer = buffer;
    //xfer_desc.rx_length   = n;

    spi_xfer_done = false;

    //APP_ERROR_CHECK(nrf_drv_spi_xfer(&spi, &xfer_desc, 0));     // I tried also this SPI_XFER function but withou success..

    //nrf_gpio_pin_set(31);
    //nrf_delay_ms(1);
    
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &addr, 1, buffer, n));
    
    //nrf_delay_ms(1);
    //nrf_gpio_pin_clear(31);

    while (!spi_xfer_done)
    {
        __WFE();
    }

    NRF_LOG_FLUSH();
}


uint8_t readRegister8(uint8_t addr) 
{
    uint8_t ret = 0;
    readRegisterN(addr, &ret, 1);

    return ret;
}


uint16_t readRegister16(uint8_t addr) 
{
    uint8_t buffer[2] = {0, 0};
    readRegisterN(addr, buffer, 2);

    uint16_t ret = buffer[0];
    ret <<= 8;
    ret |= buffer[1];

    return ret;
}


void writeRegister8(uint8_t addr, 
                    uint8_t data) 
{
    addr |= 0x80; // make sure top bit is set

    uint8_t buffer[2] = {addr, data};

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buffer, 2, NULL, 0));

    while (!spi_xfer_done)
    {
        __WFE();
    }

    NRF_LOG_FLUSH();
}
