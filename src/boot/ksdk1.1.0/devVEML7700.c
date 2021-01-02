
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceVEML7700State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initVEML7700(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	return;
}

WarpStatus
writeSensorRegisterVEML7700(uint8_t deviceRegister, uint8_t payload1, uint8_t payload0, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceVEML7700State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload0;
	payloadByte[1] = payload1;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorVEML7700(uint16_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus0, i2cWriteStatus1, i2cWriteStatus2;
	i2cWriteStatus0 = writeSensorRegisterVEML7700(0x00, 0x00, 0x01, menuI2cPullupValue); // shutdown
	i2cWriteStatus1 = writeSensorRegisterVEML7700(0x00, 0x00, 0b11000001, menuI2cPullupValue); // set gain = 0 and integration time = 800ms
        i2cWriteStatus2 = writeSensorRegisterVEML7700(0x00, 0x00, 0b11000000, menuI2cPullupValue); // power on
	//i2cWriteStatus1 = writeSensorRegisterVEML7700(0x01, 0x4E, 0x20, menuI2cPullupValue);
        //i2cWriteStatus2 = writeSensorRegisterVEML7700(0x02, 0x27, 0x10, menuI2cPullupValue);
	

	return (i2cWriteStatus0 | i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterVEML7700(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceVEML7700State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceVEML7700State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataVEML7700(bool hexModeFlag)
{
	uint16_t		readSensorRegisterValueLSB;
	uint16_t		readSensorRegisterValueMSB;
	int32_t	readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	i2cReadStatus = readSensorRegisterVEML7700(0x04, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceVEML7700State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceVEML7700State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined/139);
		}
	}
}




void
updateSensorDataVEML7700(uint16_t * current_lux)
{
        uint16_t                readSensorRegisterValueLSB;
        uint16_t                readSensorRegisterValueMSB;
        int32_t readSensorRegisterValueCombined;
        WarpStatus      i2cReadStatus;

        i2cReadStatus = readSensorRegisterVEML7700(0x04, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB = deviceVEML7700State.i2cBuffer[1];
        readSensorRegisterValueLSB = deviceVEML7700State.i2cBuffer[0];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        if (i2cReadStatus != kWarpStatusOK)
        {
                SEGGER_RTT_WriteString(0, " ----,");
        }
        else
        {
		*current_lux = readSensorRegisterValueCombined/139;
        }
}


