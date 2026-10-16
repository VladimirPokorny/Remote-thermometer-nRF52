/***************************************************
  This is a library for the RTD Sensor MAX31865

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  Ported to the nRF5 SDK by Vladimir Pokorny, then to Zephyr / nRF Connect SDK.
 ****************************************************/

#ifndef MAX31865_LIB_H
#define MAX31865_LIB_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

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

#define RTD_A   3.9083e-3f
#define RTD_B   -5.775e-7f
#define RTD_C   -4.183e-12f

typedef enum {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

/**
 * @brief One MAX31865 chip with its RTD sensor.
 *
 * Replaces the slave_select pin of the nRF5 SDK version: the SPI bus, chip select, sensor and
 * reference resistances all come from a "maxim,max31865" devicetree node.
 */
struct max31865 {
  struct spi_dt_spec spi;
  float rtd_nominal;      /* Resistance of the RTD sensor at 0 C (100 for PT100, 1000 for PT1000) */
  float ref_resistor;     /* Resistance of the reference resistor (430 or 4300) */
  max31865_numwires_t wires;
};

/* SPI mode 1 (CPOL = 0, CPHA = 1), MSB first, 8-bit words. The chip supports modes 1 and 3. */
#define MAX31865_SPI_OPERATION (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

/** Initializer for struct max31865 from a "maxim,max31865" devicetree node. */
#define MAX31865_DT_SPEC_GET(node_id)                                              \
  {                                                                                \
    .spi = SPI_DT_SPEC_GET(node_id, MAX31865_SPI_OPERATION),                        \
    .rtd_nominal = DT_PROP(node_id, resistance_at_zero),                           \
    .ref_resistor = DT_PROP(node_id, resistance_reference),                        \
    .wires = DT_PROP(node_id, maxim_3_wire) ? MAX31865_3WIRE : MAX31865_4WIRE,     \
  }


/**************************Function declaration****************************/
/* Functions returning int return 0 on success or a negative errno from the SPI driver. */

int MAX31865_init(const struct max31865 *dev);

int readFault(const struct max31865 *dev, uint8_t *fault);

int clearFault(const struct max31865 *dev);

int readRTD(const struct max31865 *dev, uint16_t *rtd);

int setWires(const struct max31865 *dev, max31865_numwires_t wires);

int autoConvert(const struct max31865 *dev, bool b);

int enable50Hz(const struct max31865 *dev, bool b);

int enableBias(const struct max31865 *dev, bool b);

int readTemperature(const struct max31865 *dev, float *temperature);

int readTemperatureAfterPowerUp(const struct max31865 *dev, float *temperature);

float rtdToTemperature(const struct max31865 *dev, uint16_t rtd);

int temperature_new(const struct max31865 *dev, float *temperature);

int readRegisterN(const struct max31865 *dev, uint8_t addr, uint8_t buffer[], uint8_t n);

int readRegister8(const struct max31865 *dev, uint8_t addr, uint8_t *value);

int readRegister16(const struct max31865 *dev, uint8_t addr, uint16_t *value);

int writeRegister8(const struct max31865 *dev, uint8_t addr, uint8_t data);

/**************************Function declaration****************************/

#endif /* MAX31865_LIB_H */
