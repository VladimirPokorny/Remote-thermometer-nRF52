#include "Ada_MAX31865.h"

#include <stdlib.h>


/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool Adafruit_MAX31865::begin(max31865_numwires_t wires) {
  spi_dev.begin();

  setWires(wires);
  enableBias(false);
  autoConvert(false);
  clearFault();

  // Serial.print("config: ");
  // Serial.println(readRegister8(MAX31865_CONFIG_REG), HEX);
  return true;
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



/**********************************************/

readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  spi_dev.write_then_read(&addr, 1, buffer, n);
}

void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  addr |= 0x80; // make sure top bit is set

  uint8_t buffer[2] = {addr, data};
  spi_dev.write(buffer, 2);
}







