#include "BME280.h"

BME280::BME280(void)
  : I2C_ADDR(0x77),
  tStandby(0),
  filter(0),
  tempOverSample(1),
  pressOverSample(1),
  humidOverSample(1),
  referencePressure(101325.0)
{
  fd = open("/dev/i2c-8", O_RDWR);

  if (fd < 0) {
    printf("Error opening file: %s\n", strerror(errno));
  }

  if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) {
    printf("ioctl error: %s\n", strerror(errno));
  }

  this->readCalibrationData();

	setStandbyTime(tStandby); //0.5ms
	setFilter(filter); //Filter off
	setPressureOverSample(pressOverSample); //Default of 1x oversample
	setHumidityOverSample(humidOverSample); //Default of 1x oversample
	setTempOverSample(tempOverSample); //Default of 1x oversample

  this->setMode(MODE_NORMAL);
}

void BME280::writeRegister(uint8_t reg_addr_, uint8_t dataToWrite_) {
  uint8_t buffer[2];
  buffer[0] = reg_addr_;
  buffer[1] = dataToWrite_;
  write(fd, buffer, 2);
  return;
}

uint8_t BME280::readRegister(uint8_t reg_addr_) {
  uint8_t buffer;
  write(fd, &reg_addr_, 1);
  read(fd, &buffer, 1);
  return buffer;
}

int16_t BME280::readRegisterInt16(uint8_t reg_addr_) {
  uint8_t buffer[2];
  write(fd, &reg_addr_, 1);
  read(fd, &buffer, 2);
  return (int16_t)buffer[0] | int16_t(buffer[1] << 8);
}

void BME280::readRegisterRegion(uint8_t* buffer_, uint8_t reg_addr_, uint8_t length_) {
  write(fd, &reg_addr_, 1);
  for (int i = 0; i<length_; i++) {
    buffer_[i] = readRegister(reg_addr_+i);
  }
  return;
}

//Gets the current mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
uint8_t BME280::getMode()
{
	uint8_t controlData = readRegister(BME280_REG::BME280_CTRL_MEAS_REG);
	return(controlData & 0b00000011); //Clear bits 7 through 2
}

//Set the mode bits in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BME280::setMode(uint8_t mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = readRegister(BME280_REG::BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	writeRegister(BME280_REG::BME280_CTRL_MEAS_REG, controlData);
}

float BME280::getTempC(void)
{
  return tempC;
}
float BME280::getTempF(void)
{
  return tempF;
}
float BME280::getPressure(void)
{
  return pressure;
}
float BME280::getAltMeters(void)
{
  return altMeters;
}
float BME280::getAltFeet(void)
{
  return altFeet;
}
float BME280::getHumidity(void)
{
  return humidity;
}

//Set the standby bits in the config register
//tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
void BME280::setStandbyTime(uint8_t timeSetting)
{
	if(timeSetting > 0b111) timeSetting = 0; //Error check. Default to 0.5ms
	
	uint8_t controlData = readRegister(BME280_REG::BME280_CONFIG_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	controlData |= (timeSetting << 5); //Align with bits 7/6/5
	writeRegister(BME280_REG::BME280_CONFIG_REG, controlData);
}

//Set the filter bits in the config register
//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
void BME280::setFilter(uint8_t filterSetting)
{
	if(filterSetting > 0b111) filterSetting = 0; //Error check. Default to filter off
	
	uint8_t controlData = readRegister(BME280_REG::BME280_CONFIG_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2); //Align with bits 4/3/2
	writeRegister(BME280_REG::BME280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void BME280::setTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = readRegister(BME280_REG::BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	writeRegister(BME280_REG::BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid over sampling values
void BME280::setPressureOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = readRegister(BME280_REG::BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	writeRegister(BME280_REG::BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid over sampling values
void BME280::setHumidityOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData = readRegister(BME280_REG::BME280_CTRL_HUMIDITY_REG);
	controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	writeRegister(BME280_REG::BME280_CTRL_HUMIDITY_REG, controlData);

	setMode(originalMode); //Return to the original user's choice
}

//Return the local reference pressure
float BME280::getReferencePressure()
{
	return(referencePressure);
}

// Sets the internal variable _referencePressure so the altitude is calculated properly.
// This is also known as "sea level pressure" and is in Pascals. The value is probably
// within 10% of 101325. This varies based on the weather:
// https://en.wikipedia.org/wiki/Atmospheric_pressure#Mean_sea-level_pressure
//
// if you are concerned about accuracy or precision, make sure to pull the
// "sea level pressure"value from a trusted source like NOAA.
void BME280::setReferencePressure(float refPressure)
{
	referencePressure = refPressure;
}

void BME280::reset(void)
{
	writeRegister(BME280_REG::BME280_RST_REG, 0xB6);
}

//Check the measuring bit and return true while device is taking measurement
bool BME280::isMeasuring(void)
{
	uint8_t stat = readRegister(BME280_REG::BME280_STAT_REG);
	return(stat & (1<<3)); //If the measuring bit (3) is set, return true
}

void BME280::readAllMeasurements(void){
	
	uint8_t dataBurst[8];
	readRegisterRegion(dataBurst, BME280_REG::BME280_MEASUREMENTS_REG, 8);
	
	readFloatPressureFromBurst(dataBurst);
	readFloatHumidityFromBurst(dataBurst);

  readTempCFromBurst(dataBurst);
  altMeters = readFloatAltitudeMeters(pressure);
  readTempFFromBurst(dataBurst);
  altFeet = readFloatAltitudeFeet(pressure); // TODO: the first value of feet is too large
}

void BME280::readCalibrationData(void)
{
	dig_T1 = (uint16_t)(readRegisterInt16(BME280_REG::BME280_DIG_T1_LSB_REG));
	dig_T2 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_T2_LSB_REG));
	dig_T3 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_T3_LSB_REG));

	dig_P1 = (uint16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P1_LSB_REG));
	dig_P2 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P2_LSB_REG));
	dig_P3 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P3_LSB_REG));
	dig_P4 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P4_LSB_REG));
	dig_P5 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P5_LSB_REG));
	dig_P6 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P6_LSB_REG));
	dig_P7 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P7_LSB_REG));
	dig_P8 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P8_LSB_REG));
	dig_P9 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_P9_LSB_REG));

	dig_H1 = (uint8_t)(readRegister(BME280_REG::BME280_DIG_H1_REG));
	dig_H2 = (int16_t)(readRegisterInt16(BME280_REG::BME280_DIG_H2_LSB_REG));
	dig_H3 = (uint8_t)(readRegister(BME280_REG::BME280_DIG_H3_REG));
	dig_H4 = (int16_t)((readRegister(BME280_REG::BME280_DIG_H4_MSB_REG) << 4) + \
    (readRegister(BME280_REG::BME280_DIG_H4_LSB_REG) & 0x0F));
	dig_H5 = (int16_t)((readRegister(BME280_REG::BME280_DIG_H5_MSB_REG) << 4) + \
    ((readRegister(BME280_REG::BME280_DIG_H4_LSB_REG) >> 4) & 0x0F));
	dig_H6 = (int8_t)(readRegister(BME280_REG::BME280_DIG_H6_REG));
}

float BME280::readFloatPressure(void)
{

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_REG::BME280_PRESSURE_MSB_REG, 3);
  int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = ((static_cast<int64_t>(dig_P8)) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	
	return (float)p_acc / 256.0;
}

float BME280::readFloatAltitudeMeters(float pressure_)
{
	float heightOutput = 0;
	
	heightOutput = ((float)-44330.77)*(pow((pressure_/(float)referencePressure), 0.190263) - (float)1);
	return heightOutput;
	
}

float BME280::readFloatAltitudeFeet(float pressure_)
{
	float heightOutput = 0;
	
	heightOutput = readFloatAltitudeMeters(pressure_) * 3.28084;
	return heightOutput;
	
}

float BME280::readTempC(void)
{
	//get the reading (adc_T);
  uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_REG::BME280_TEMPERATURE_MSB_REG, 3);
  int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *	((int32_t)dig_T3)) >> 14;

	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;
	output = output / 100.0f;
  return output;
}

float BME280::readTempF(void)
{
	float output = readTempC();
	output = (output * 9) / 5 + 32;
	return output;
}

float BME280::readFloatHumidity(void)
{
	
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    uint8_t buffer[2];
	readRegisterRegion(buffer, BME280_REG::BME280_HUMIDITY_MSB_REG, 2);
    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;
}
double BME280::dewPointC(void)
{
  double celsius = readTempC(); 
  double humidity = readFloatHumidity();
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);
         // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;
         // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

double BME280::dewPointF(void)
{
	return(dewPointC() * 1.8 + 32); //Convert C to F
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the humidty, pressure, and temp oversample functions
uint8_t BME280::checkSampleValue(uint8_t userValue)
{
	switch(userValue) 
	{
		case(0): 
			return 0;
			break; //Valid
		case(1): 
			return 1;
			break; //Valid
		case(2): 
			return 2;
			break; //Valid
		case(4): 
			return 3;
			break; //Valid
		case(8): 
			return 4;
			break; //Valid
		case(16): 
			return 5;
			break; //Valid
		default: 
			return 1; //Default to 1x
			break; //Good
	}
}

// TODO: It needs to be merged to original function (w/o burst) -> It is doubled
float BME280::readTempFromBurst(uint8_t buffer[])
{
  int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100;
	
 	return output;
}

void BME280::readTempCFromBurst(uint8_t buffer[])
{
  tempC = readTempFromBurst(buffer);
}

void BME280::readTempFFromBurst(uint8_t buffer[])
{
  float output = readTempFromBurst(buffer);
	output = (output * 9) / 5 + 32;
	tempF = output;
}

void BME280::readFloatPressureFromBurst(uint8_t buffer[])
{
	// Set pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  
  int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		pressure = 0; // avoid exception caused by division by zero
	}
	else
	{
		p_acc = 1048576 - adc_P;
		p_acc = (((p_acc<<31) - var2)*3125)/var1;
		var1 = (((int64_t)dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
		var2 = (((int64_t)dig_P8) * p_acc) >> 19;
		p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
		
		pressure = (float)p_acc / 256.0;
	}
}

void BME280::readFloatHumidityFromBurst(uint8_t buffer[])
{	
	// Set humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
  int32_t adc_H = ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7]);
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	humidity = (float)(var1>>12) / 1024.0;
}
