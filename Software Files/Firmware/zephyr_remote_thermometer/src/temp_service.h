/**
 * Remote thermometer GATT service.
 *
 * Same UUIDs and value format as the nRF5 SDK firmware (our_service.c), plus one characteristic that
 * carries all four temperatures:
 *
 *   Service:   0000ABCD-5468-616E-6B73-4D6F6D446164
 *   Sensor 1:  00001000-5468-616E-6B73-4D6F6D446164   read
 *   Sensor 2:  00002000-5468-616E-6B73-4D6F6D446164   read
 *   Sensor 3:  00003000-5468-616E-6B73-4D6F6D446164   read
 *   Sensor 4:  00004000-5468-616E-6B73-4D6F6D446164   read
 *   All:       00005000-5468-616E-6B73-4D6F6D446164   read, notify (once per measurement round)
 *   Config:    00006000-5468-616E-6B73-4D6F6D446164   read, write (with response)
 *
 * Sensor value: int32, little endian, temperature in milli-degrees Celsius. "All" is the four sensor
 * values in a row (16 bytes). Before the first measurement, and for disabled sensors: DE AD BE EF.
 *
 * Config, 8 bytes, little endian:
 *   [0]    version, 1
 *   [1]    sensor mask: bit n = sensor n+1 enabled
 *   [2]    flags: bit 0 = measure only while a client is connected
 *   [3]    reserved, 0
 *   [4..7] sampling interval in ms, 100 .. 3600000
 * A write takes effect at once (a measurement round starts with the new settings). Invalid values are
 * rejected with ATT error 0x13 (value not allowed), a wrong length with 0x0D.
 */

#ifndef TEMP_SERVICE_H__
#define TEMP_SERVICE_H__

#include <stdint.h>

#define TEMP_SERVICE_SENSOR_COUNT 4

/**
 * @brief Store a new temperature. Nothing is sent until temp_service_publish().
 *
 * @param sensor         Sensor index, 0..TEMP_SERVICE_SENSOR_COUNT-1.
 * @param milli_celsius  Temperature in thousandths of a degree Celsius.
 */
void temp_service_update(uint8_t sensor, int32_t milli_celsius);

/** @brief Mark a sensor as not measured (DE AD BE EF), e.g. after it was disabled. */
void temp_service_clear(uint8_t sensor);

/**
 * @brief Notify subscribed clients of all stored temperatures in one packet.
 *
 * Called once per measurement round, after all sensors were read.
 */
void temp_service_publish(void);

#endif /* TEMP_SERVICE_H__ */
