/***************************************************
  This is a library for the RTD Sensor MAX31865

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef MAX31865_LIB_H
#define MAX31865_LIB_H

#include <stdio.h>
#include <stdbool.h>
#include "stdint.h"

#define MAX31865_CONFIG_REG       0x00
#define MAX31865_CONFIG_BIAS      0x80
#define MAX31865_CONFIG_MODEAUTO  0x40
#define MAX31865_CONFIG_MODEOFF   0x00
#define MAX31865_CONFIG_1SHOT     0x20
#define MAX31865_CONFIG_3WIRE     0x10
#define MAX31865_CONFIG_24WIRE    0x00
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ  0x01
#define MAX31865_CONFIG_FILT60HZ  0x00

#define MAX31865_RTDMSB_REG       0x01
#define MAX31865_RTDLSB_REG       0x02
#define MAX31865_HFAULTMSB_REG    0x03
#define MAX31865_HFAULTLSB_REG    0x04
#define MAX31865_LFAULTMSB_REG    0x05
#define MAX31865_LFAULTLSB_REG    0x06
#define MAX31865_FAULTSTAT_REG    0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH  0x40
#define MAX31865_FAULT_REFINLOW   0x20
#define MAX31865_FAULT_REFINHIGH  0x10
#define MAX31865_FAULT_RTDINLOW   0x08
#define MAX31865_FAULT_OVUV       0x04

#define RTD_A   3.9083e-3
#define RTD_B   -5.775e-7
#define RTD_C   -4.183e-12

typedef enum {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

typedef enum {
  SLAVE_1 = 23,
  SLAVE_2 = 24,
  SLAVE_3 = 17,
  SLAVE_4 = 31
} slave_selects_t;

typedef enum {
  EN_1 = 12,
  EN_2 = 3,
  EN_3 = 8,
  EN_4 = 14,
} pins_t;




/**************************Function declaration****************************/


bool MAX31865_init(uint8_t wires, slave_selects_t slave_select);

uint8_t readFault(slave_selects_t slave_select);

void clearFault(slave_selects_t slave_select);

uint16_t readRTD(slave_selects_t slave_select);

void setWires(uint8_t wires, slave_selects_t slave_select);

void autoConvert(bool b, slave_selects_t slave_select);

void enable50Hz(bool b, slave_selects_t slave_select);

void enableBias(bool b, slave_selects_t slave_select);

float readTemperature(float RTDnominal, float refResistor, slave_selects_t slave_select);

float temperature_new(float RTDnominal, float refResistor, slave_selects_t slave_select);

void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n, slave_selects_t slave_select);

uint8_t readRegister8(uint8_t addr, slave_selects_t slave_select);

uint16_t readRegister16(uint8_t addr, slave_selects_t slave_select);

void writeRegister8(uint8_t addr, uint8_t reg, slave_selects_t slave_select);



/**************************Function declaration****************************/

#endif /* MAX31865_LIB_H */
