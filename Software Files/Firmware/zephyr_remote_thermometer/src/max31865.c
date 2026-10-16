#include "max31865.h"

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(max31865, LOG_LEVEL_INF);


/**************************************************************************/
/*!
    @brief Check the SPI bus and set the number of RTD wires used
    @param dev The chip, its wiring comes from devicetree
    @return 0 or a negative errno
*/
/**************************************************************************/
int MAX31865_init(const struct max31865 *dev)
{
    int err;

    if (!spi_is_ready_dt(&dev->spi))
    {
        LOG_ERR("SPI bus %s is not ready", dev->spi.bus->name);
        return -ENODEV;
    }

    err = setWires(dev, dev->wires);
    if (!err) err = enableBias(dev, false);
    if (!err) err = autoConvert(dev, false);
    if (!err) err = clearFault(dev);

    if (err)
    {
        LOG_ERR("Initialization of MAX31865 failed (%d)", err);
        return err;
    }

    LOG_DBG("Initialization of MAX31865 was succesful");    // after every power-up, so debug only
    return 0;
}


/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
int setWires(const struct max31865 *dev, max31865_numwires_t wires)
{
     uint8_t t;
     int err = readRegister8(dev, MAX31865_CONFIG_REG, &t);
     if (err) return err;

     if (wires == MAX31865_3WIRE)
     {
        t |= MAX31865_CONFIG_3WIRE;
     }

     else
     {
        // 2 or 4 wire
        t &= ~MAX31865_CONFIG_3WIRE;
     }

     return writeRegister8(dev, MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
int enableBias(const struct max31865 *dev, bool b)
{
  uint8_t t;
  int err = readRegister8(dev, MAX31865_CONFIG_REG, &t);
  if (err) return err;

  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  return writeRegister8(dev, MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
int autoConvert(const struct max31865 *dev, bool b)
{
    uint8_t t;
    int err = readRegister8(dev, MAX31865_CONFIG_REG, &t);
    if (err) return err;

    if (b) {
      t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
    } else {
      t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
    }
    return writeRegister8(dev, MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
int clearFault(const struct max31865 *dev)
{
    uint8_t t;
    int err = readRegister8(dev, MAX31865_CONFIG_REG, &t);
    if (err) return err;

    t &= ~0x2C;
    t |= MAX31865_CONFIG_FAULTSTAT;
    return writeRegister8(dev, MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @param fault The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
int readFault(const struct max31865 *dev, uint8_t *fault)
{
    return readRegister8(dev, MAX31865_FAULTSTAT_REG, fault);
}


/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/
int enable50Hz(const struct max31865 *dev, bool b)
{
    uint8_t t;
    int err = readRegister8(dev, MAX31865_CONFIG_REG, &t);
    if (err) return err;

    if (b) {
      t |= MAX31865_CONFIG_FILT50HZ;
    } else {
      t &= ~MAX31865_CONFIG_FILT50HZ;
    }
    return writeRegister8(dev, MAX31865_CONFIG_REG, t);
}


/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param temperature Temperature in C
*/
/**************************************************************************/
int readTemperature(const struct max31865 *dev, float *temperature)
{
    uint16_t rtd;

    int err = readRTD(dev, &rtd);
    if (err) return err;

    *temperature = rtdToTemperature(dev, rtd);
    return 0;
}


/**************************************************************************/
/*!
    @brief One-shot temperature measurement on a chip that was just powered
    up (its registers are at their reset values). Writes the configuration
    instead of read-modify-write and leaves the bias on, because the caller
    cuts the power right after: 3 SPI transfers instead of 17 for
    MAX31865_init() + readTemperature().
    @param temperature Temperature in C
*/
/**************************************************************************/
int readTemperatureAfterPowerUp(const struct max31865 *dev, float *temperature)
{
    uint8_t config = (dev->wires == MAX31865_3WIRE ? MAX31865_CONFIG_3WIRE : MAX31865_CONFIG_24WIRE)
                     | MAX31865_CONFIG_FILT60HZ;

    // Bias on, clear faults, then let the RTD input filter settle
    int err = writeRegister8(dev, MAX31865_CONFIG_REG,
                             config | MAX31865_CONFIG_BIAS | MAX31865_CONFIG_FAULTSTAT);
    if (err) return err;
    k_msleep(10);

    err = writeRegister8(dev, MAX31865_CONFIG_REG,
                         config | MAX31865_CONFIG_BIAS | MAX31865_CONFIG_1SHOT);
    if (err) return err;
    k_msleep(65);

    uint16_t rtd;
    err = readRegister16(dev, MAX31865_RTDMSB_REG, &rtd);
    if (err) return err;

    *temperature = rtdToTemperature(dev, rtd >> 1);
    return 0;
}


/**************************************************************************/
/*!
    @brief Convert the 15-bit RTD code to a temperature in C (AN709 method,
    polynomial below 0 C). Shared by readTemperature() and
    readTemperatureAfterPowerUp().
*/
/**************************************************************************/
float rtdToTemperature(const struct max31865 *dev, uint16_t rtd)
{
    float Z1, Z2, Z3, Z4, Rt, temp;

    Rt = rtd;
    Rt /= 32768;
    Rt *= dev->ref_resistor;

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / dev->rtd_nominal;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrtf(temp) + Z1) / Z4;

    if (temp >= 0)
      return temp;

    // ugh.
    Rt /= dev->rtd_nominal;
    Rt *= 100; // normalize to 100 ohm

    float rpoly = Rt;

    temp = -242.02f;
    temp += 2.2228f * rpoly;
    rpoly *= Rt; // square
    temp += 2.5859e-3f * rpoly;
    rpoly *= Rt; // ^3
    temp -= 4.8260e-6f * rpoly;
    rpoly *= Rt; // ^4
    temp -= 2.8183e-8f * rpoly;
    rpoly *= Rt; // ^5
    temp += 1.5243e-10f * rpoly;

    return temp;
}



/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses the Callendar-Van Dusen equation, solved directly above
    0 C and by Newton's method below.
    @param temperature Temperature in C
*/
/**************************************************************************/
int temperature_new(const struct max31865 *dev, float *temperature)
{
    float a, b, c, Rt, temp;
    float RTDnominal = dev->rtd_nominal;
    uint16_t rtd;

    int err = readRTD(dev, &rtd);
    if (err) return err;

    Rt = rtd;
    Rt = Rt / 32768;
    Rt = Rt * dev->ref_resistor;

    if (Rt >= RTDnominal)
    {
      a = RTDnominal * RTD_B;
      b = RTDnominal * RTD_A;
      c = RTDnominal - Rt;

      temp = (- b + sqrtf(b * b - 4 * a * c))/(2 * a);
    }

    else
    {
      temp = (Rt/RTDnominal - 1)/(RTD_A + 100 * RTD_B);

      for (int i = 0; i < 3; i++)
      {
        temp = temp + (1 + RTD_A * temp + RTD_B * powf(temp, 2) + RTD_C * powf(temp, 3) * (temp - 100) - Rt/RTDnominal)/(RTD_A + 2 * RTD_B * temp - 300 * RTD_C * powf(temp, 2) + 4 * RTD_C * powf(temp, 3));
      }
    }

    *temperature = temp;
    return 0;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @param rtd The raw unsigned 15-bit value, NOT temperature!
*/
/**************************************************************************/
int readRTD(const struct max31865 *dev, uint16_t *rtd)
{
    int err = clearFault(dev);
    if (!err) err = enableBias(dev, true);
    if (err) return err;

    k_msleep(10);

    uint8_t t;
    err = readRegister8(dev, MAX31865_CONFIG_REG, &t);
    if (err) return err;
    t |= MAX31865_CONFIG_1SHOT;

    err = writeRegister8(dev, MAX31865_CONFIG_REG, t);
    if (err) return err;

    k_msleep(65);

    uint16_t value;
    err = readRegister16(dev, MAX31865_RTDMSB_REG, &value);

    // Disable bias current again to reduce selfheating of PT100.
    int bias_err = enableBias(dev, false);
    if (err) return err;
    if (bias_err) return bias_err;

    *rtd = value >> 1;
    return 0;
}


int readRegisterN(const struct max31865 *dev, uint8_t addr, uint8_t buffer[], uint8_t n)
{
    addr &= 0x7F;                                   // make sure top bit is not set

    // Address byte out, then n bytes clocked in. buffer[0] is received while the address
    // goes out and holds no data, as with nrf_drv_spi_transfer(&addr, 1, buffer, n).
    const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
    const struct spi_buf rx_buf = { .buf = buffer, .len = n };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    return spi_transceive_dt(&dev->spi, &tx, &rx);
}


int readRegister8(const struct max31865 *dev, uint8_t addr, uint8_t *value)
{
    uint8_t ret[2] = {0x00, 0x00};
    int err = readRegisterN(dev, addr, ret, 2);

    *value = ret[1];
    return err;
}


int readRegister16(const struct max31865 *dev, uint8_t addr, uint16_t *value)
{
    uint8_t buffer[3] = {0x00, 0x00, 0x00};
    int err = readRegisterN(dev, addr, buffer, 3);

    uint16_t ret = buffer[1];
    ret <<= 8;
    ret |= buffer[2];

    *value = ret;
    return err;
}


int writeRegister8(const struct max31865 *dev, uint8_t addr, uint8_t data)
{
    addr |= 0x80; // make sure top bit is set

    uint8_t buffer[2] = {addr, data};
    const struct spi_buf tx_buf = { .buf = buffer, .len = sizeof(buffer) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    return spi_write_dt(&dev->spi, &tx);
}
