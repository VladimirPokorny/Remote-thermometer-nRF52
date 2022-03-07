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

//#ifndef MAX31865_H
//#define MAX31865_H

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

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;







#ifndef INC_MAX31865_LIB_H_
#define INC_MAX31865_LIB_H_

//#include <main.h>

#include <stdio.h>
#include <stdbool.h>

void MAX31865_Init(uint8_t num_wires);
uint8_t MAX31865_Configuration_info(void);
//double MAX31865_Get_Temperature(void);
//double MAX31865_Get_Temperature_math(double PT100_Resistance);

#endif /* INC_MAX31865_LIB_H_ */
