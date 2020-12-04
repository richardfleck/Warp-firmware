

void		initVEML7700(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
WarpStatus	readSensorRegisterVEML7700(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterVEML7700(uint8_t deviceRegister,
					uint8_t payloadBtye,
					uint16_t menuI2cPullupValue);
WarpStatus	configureSensorVEML7700(uint16_t menuI2cPullupValue);
WarpStatus	readSensorSignalVEML7700(WarpTypeMask signal,
					WarpSignalPrecision precision,
					WarpSignalAccuracy accuracy,
					WarpSignalReliability reliability,
					WarpSignalNoise noise);
void		printSensorDataVEML7700(bool hexModeFlag);
void            updateSensorDataVEML7700(uint16_t *current_lux);
