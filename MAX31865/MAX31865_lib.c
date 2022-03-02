#include "MAX31865_lib.h"

/*------The characteristics of the PT100 sensor type and the reference resistor connected to the MAX31865------*/
#define MAX31865_PT100_R0 (double)  100.0 		//Resistance of the PT100 sensor, at 0 °C
#define MAX31865_Rref (double)      400.0		//Resistance of the reference resistor connected to the MAX31865
/*------The characteristics of the PT100 sensor type and the reference resistor connected to the MAX31865------*/

/*-----------Coefficients from GOST 6651-2009 for sensor type PT100(Platinum TS & PE, 0.00385°C^-1)------------*/
#define MAX31865_A (double)         0.0039083
#define MAX31865_B (double)         0.0000005775
/*-----------Coefficients from GOST 6651-2009 for sensor type PT100(Platinum TS & PE, 0.00385°C^-1)------------*/


/*------------------------------------------Global variables---------------------------------------------------*/
double MAX31865_PT100_R     = 0.0; 				//Global variable defining PT100 sensor resistance
double MAX31865_PT100_T     = 0.0; 				//Global variable defining the temperature of the PT100 sensor
bool MAX31865_Sensor_Error  = 0; 				//Global variable defining the failure of the PT100 sensor
/*------------------------------------------Global variables---------------------------------------------------*/




/*=======================INITIALIZING THE MAX31865 MODULE=========================*/
void MAX31865_Init(uint8_t num_wires) {
	///Initialization function of the MAX31865 module
	///I don't see much sense in displaying the full module setting, so let's make
	///small simplification for the end user
	///all that the user can configure is to select the type of connection
	///2,3 or 4-wire
	/// \param num_wires - sensor connection type 2,3 or 4 wire

	uint8_t MAX31865_Reinitialization_cnt = 0;
	MAX31865_Sensor_Error = 0;
	uint8_t MAX31865_Configuration_register_write[] = { 0x80, 0x00 };
	if (num_wires == 2 || num_wires == 4) {
		MAX31865_Configuration_register_write[1] = 0xC3; //0xC3
	} else if (num_wires == 3) {
		MAX31865_Configuration_register_write[1] = 0xD3; //0xD3
	}

        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, MAX31865_Configuration_register_write, 2, NULL, 0));

        while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);

	//In order to reach the sensor after power-up, as initialization may not take place the first time, we start a loop.
	while (MAX31865_Configuration_info() != 0xD1 || MAX31865_Configuration_info() != 0xC1) {
		MAX31865_Reinitialization_cnt++;

		cs_set();
		HAL_SPI_Transmit(&hspi3, MAX31865_Configuration_register_write, 2, 100);
		cs_reset();

		if (MAX31865_Reinitialization_cnt == 100) {
			// printf("Initialization MAX31865 != OK");
			break;
		}

	}

}
/*=======================INITIALIZING THE MAX31865 MODULE=========================*/


/*====================MAX31865 CONFIGURATION INFORMATION===================*/
uint8_t MAX31865_Configuration_info(void) {
	//Function for retrieving configuration information of the MAX31865 module
	///retrieves configuration value.
	/// Don't be surprised if you send 0xC3 during initialization and get 0xC1
	///(see datasheet MAX31865 p.14 "The fault status clear bit D1, self-clears to 0.")

	uint8_t read_data = 0x00;
	uint8_t MAX31865_Configuration = 0x00;

        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, read_data, 1, MAX31865_Configuration, 1));

        while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);

	NRF_LOG_INFO("Configuration Received:");
        NRF_LOG_HEXDUMP_INFO(MAX31865_Configuration, strlen((const char *)MAX31865_Configuration));
	return MAX31865_Configuration;
}
/*====================MAX31865 CONFIGURATION INFORMATION===================*/