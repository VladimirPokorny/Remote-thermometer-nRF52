#include "MAX31865.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stdint.h"

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
//#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "math.h"



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
}

void spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.ss_pin     = NRF_SPI_PIN_NOT_CONNECTED;  // 31   P0.31   (CS on MAX31865)
    spi_config.miso_pin   = SPI_MISO_PIN;               // 30   P0.30   (SDO on MAX)
    spi_config.mosi_pin   = SPI_MOSI_PIN;               // 29   P0.29   (SDI on MAX)
    spi_config.sck_pin    = SPI_SCK_PIN;                // 26   P0.26   (CLK on MAX)

    spi_config.mode       = NRF_DRV_SPI_MODE_1;         // set up from datasheet (it is able to use spi mode 1 or 3)
    spi_config.frequency  = NRF_DRV_SPI_FREQ_4M;        // max frequency is 5 MHz
    spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}


/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool MAX31865_init(max31865_numwires_t wires,
           slave_selects_t slave_select)
{
    setWires(wires, slave_select);
    enableBias(false, slave_select);
    autoConvert(false, slave_select);
    clearFault(slave_select);

    NRF_LOG_INFO("Initialization of MAX31865 was succesful");
    return true;
}


/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void setWires(max31865_numwires_t wires,
              slave_selects_t slave_select) 
{
     uint8_t t = readRegister8(MAX31865_CONFIG_REG, slave_select);
     if (wires == MAX31865_3WIRE)
     {
        t |= MAX31865_CONFIG_3WIRE;
     }

     else 
     {
        // 2 or 4 wire
        t &= ~MAX31865_CONFIG_3WIRE;
     }

     writeRegister8(MAX31865_CONFIG_REG, t, slave_select);
}


/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void enableBias(bool b,
                slave_selects_t slave_select) 
{
  uint8_t t = readRegister8(MAX31865_CONFIG_REG, slave_select);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31865_CONFIG_REG, t, slave_select);
}


/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void autoConvert( bool b,
                  slave_selects_t slave_select) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG, slave_select);
    if (b) {
      t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
    } else {
      t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
    }
    writeRegister8(MAX31865_CONFIG_REG, t, slave_select);
}


/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void clearFault(slave_selects_t slave_select) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG, slave_select);
    t &= ~0x2C;
    t |= MAX31865_CONFIG_FAULTSTAT;
    writeRegister8(MAX31865_CONFIG_REG, t, slave_select);
}


/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t readFault(slave_selects_t slave_select) 
{
    return readRegister8(MAX31865_FAULTSTAT_REG, slave_select);
}


/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/
void enable50Hz(bool b,
                slave_selects_t slave_select) 
{
    uint8_t t = readRegister8(MAX31865_CONFIG_REG, slave_select);
    if (b) {
      t |= MAX31865_CONFIG_FILT50HZ;
    } else {
      t &= ~MAX31865_CONFIG_FILT50HZ;
    }
    writeRegister8(MAX31865_CONFIG_REG, t, slave_select);
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
float readTemperature(float RTDnominal,
                      float refResistor,
                      slave_selects_t slave_select) 
{
    float Z1, Z2, Z3, Z4, Rt, temp;

    Rt = 0;
    Rt = readRTD(slave_select);
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
float temperature_new(float RTDnominal,
                      float refResistor,
                      slave_selects_t slave_select) 
{
    float a, b, c, Rt, temp;

    Rt = 0;
    Rt = readRTD(slave_select);
    Rt = Rt / 32768;
    Rt = Rt * refResistor;

    if (Rt >= RTDnominal)
    {
      a = RTDnominal * RTD_B;
      b = RTDnominal * RTD_A;
      c = RTDnominal - Rt;
          
      temp = (- b + sqrt(b * b - 4 * a * c))/(2 * a);

      return temp;
    }

    else
    {
      temp = (Rt/RTDnominal - 1)/(RTD_A + 100 * RTD_B);

      for (int i; i < 3; i++)
      {
        temp = temp + (1 + RTD_A * temp + RTD_B * pow(temp, 2) + RTD_C * pow(temp, 3) * (temp - 100) - Rt/RTDnominal)/(RTD_A + 2 * RTD_B * temp - 300 * RTD_C * pow(temp, 2) + 4 * RTD_C * pow(temp, 3));
      }
    }

    return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t readRTD(slave_selects_t slave_select) 
{
    clearFault(slave_select);
    enableBias(true, slave_select);

    nrf_delay_ms(10);

    uint8_t t;
    t = readRegister8(MAX31865_CONFIG_REG, slave_select);
    t |= MAX31865_CONFIG_1SHOT;

    writeRegister8(MAX31865_CONFIG_REG, t, slave_select);

    nrf_delay_ms(65);

    uint16_t rtd; 
    rtd = readRegister16(MAX31865_RTDMSB_REG, slave_select);

    // Disable bias current again to reduce selfheating of PT100.
    enableBias(false, slave_select); 

    rtd >>= 1;

    return rtd;
}


void readRegisterN( uint8_t addr, 
                    uint8_t *buffer,
                    uint8_t n,
                    slave_selects_t slave_select)
{
    addr &= 0x7F;                                   // make sure top bit is not set
    
    nrf_gpio_pin_clear(slave_select);
    spi_xfer_done = false;
    
    
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &addr, 1, buffer, n));

    while (!spi_xfer_done)
    {
        __WFE();
    }
    nrf_gpio_pin_set(slave_select);
}


uint8_t readRegister8(uint8_t addr,
                      slave_selects_t slave_select) 
{
    uint8_t ret[2] = {0x00, 0x00};
    readRegisterN(addr, ret, 2, slave_select);

    return ret[1];
}


uint16_t readRegister16(uint8_t addr,
                        slave_selects_t slave_select) 
{
    uint8_t buffer[3] = {0x00, 0x00, 0x00};
    readRegisterN(addr, buffer, 3, slave_select);

    uint16_t ret = buffer[1];
    ret <<= 8;
    ret |= buffer[2];

    return ret;
}


void writeRegister8(uint8_t addr, 
                    uint8_t data,
                    slave_selects_t slave_select) 
{
    addr |= 0x80; // make sure top bit is set

    uint8_t buffer[2] = {addr, data};

    nrf_gpio_pin_clear(slave_select);

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buffer, 2, NULL, 0));

    while (!spi_xfer_done)
    {
        __WFE();
    }
    nrf_gpio_pin_set(slave_select);
}




