void		initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
WarpStatus	readSensorRegisterINA219(int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister,
					uint8_t payloadBtye,
					uint16_t menuI2cPullupValue);
void		printSensorDataINA219();
