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

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;
	        
        switch (deviceRegister)
        {       
                case 0x01: case 0x02:
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
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


WarpStatus
readSensorRegisterINA219(int numberOfBytes)
{
	// read address is set by a previous write command (a pointer is written to a register), so no need for deviceRegister or cmdBuf

	i2c_status_t	status;
	USED(numberOfBytes);

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							NULL,    //send a NULL instead of cmdBuf
							0,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219()
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int16_t		readBusVoltageValue;
	int16_t		readShuntVoltageValue;	

/*
 *    Since the data registers on the INA219 are 16-bits but I2C 
 *    transfers data in 8-bit packets, we do 2-byte read transactions 
 *    for each of the registers. 
 */

	//Write 0x02 to the register pointer, then the subsequent read operation will read from the bus voltage register
	writeSensorRegisterINA219(0x02,0xff);
	readSensorRegisterINA219(2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];

	//Bus voltage value stored in upper 12 bits
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 5) | ((readSensorRegisterValueLSB & 0xF8) >> 2);

	//Convert to millivolts by multiplying by 4mV (LSB) and store in readBusVoltageValue
	readBusVoltageValue = readSensorRegisterValueCombined * 4;	
	
	//Write 0x01 to the register pointer, then the subsequent read operation will read from the shunt voltage register
	writeSensorRegisterINA219(0x01,0xff);
	readSensorRegisterINA219(2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];

	//Concatenate the two bytes together
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;
	
	//Convert to uV by multiplying by 10uV (LSB) and store in readShuntVoltageValue
	readShuntVoltageValue = readSensorRegisterValueCombined * 10;	
	
	//Multiply shunt voltage (uV) by 1/shunt resistance (=10) to get current in uA
	//Print current (uA) and bus voltage (mV) as comma-separated variables
	SEGGER_RTT_printf(0, "\n %d, %d", readShuntVoltageValue * 10, readBusVoltageValue);
}
