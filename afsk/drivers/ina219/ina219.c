/**
 * @file ina219.c
 * @author Philip Wig (github.com/philipwig)
 * @brief Driver for the TI INA219 current/power monitor
 * @version 1.0
 * @date 2021-04-04
 * 
 * @copyright Copyright (c) 2021 under the GNU General Public License v3.0
 * 
 */

#include <stdint.h>
#include <stdlib.h>
#include <wiringPiI2C.h>

#include "ina219.h"


/**
 * @brief Initilizes the specified ina219 sensor. Need to specify devId, maxInputCurrent, and Rshunt in the ina219 struct before using.
 *        Everything else will get overwritten when ina219Init() is called. Defaults to a bus voltage range of 16V, 12 bit ADC resolution 
 *        for the bus and shunt voltage, and continuous shunt and bus measurement mode. This configuration can be changed with 
 *        ina219SetConfiguration_All() or ina219SetConfiguration_Select()
 * 
 * @param sensor The ina219 sensor to use
 * @return int Returns 1 if the function there is an error, 0 for success
 */
int ina219Init(struct ina219 *sensor) {
    // Initializes the device with wiringpi
    sensor->fd = wiringPiI2CSetup(sensor->devId);

    // Reset INA219 (set to default values)
    ina219WriteRegister16(sensor, INA219_REG_CONFIG, INA219_CONFIG_RESET);

    // Calculate the 16 bit calibration value. Equations from datasheet  
    double calibration = 0.04096 / (sensor->maxInputCurrent / 32768 * sensor->Rshunt);
  
    // Convert the calculated calibration value to a 16 bit value. If the calculated calibration is higher than possible it stays at the max 16 bit value
    uint16_t calibration16bit;
    if (calibration < 65535) calibration16bit = (uint16_t) calibration;
    else return 1;  // ERROR: Calculated calibration value overflows 16 bit int. Check maxInputCurrent and Rshunt values

    // Set Calibration register to the calibration calculated above	
    ina219WriteRegister16(sensor, INA219_REG_CALIBRATION, calibration16bit);

    // Calculate the current and power LSB values from the 16 bit calibration. Equations from datasheet
    sensor->ina219_current_LSB = 0.04096 / (calibration16bit * sensor->Rshunt);
    sensor->ina219_power_LSB = sensor->ina219_current_LSB * 20;

    // Change the value of the users maxInputCurrent so they have the actual max input current value
    sensor->maxInputCurrent = sensor->ina219_current_LSB * 32768;

    // Calculate the max shunt voltage using the max input current and the value of the shunt resistor
    double maxVshunt = sensor->maxInputCurrent * sensor->Rshunt;

    // Calculate the necessary value of the PGA gain
    uint16_t PGAGain;
    if(maxVshunt < 0.04) PGAGain = INA219_CONFIG_GAIN_1_40MV;
    else if (maxVshunt < 0.08) PGAGain = INA219_CONFIG_GAIN_2_80MV;
    else if (maxVshunt < 0.16) PGAGain = INA219_CONFIG_GAIN_4_160MV;
    else if (maxVshunt < 0.32) PGAGain = INA219_CONFIG_GAIN_8_320MV;
    else return 1;  //ERROR: Could not calculate suitable PGAGain. Check maxInputCurrent and Rshunt values

    // Set Config register to take using the PGA gain from above and some defaults
    sensor->config  =   INA219_CONFIG_BVOLTAGERANGE_16V |
                        PGAGain |
                        INA219_CONFIG_BADCRES_12BIT_1S |
                        INA219_CONFIG_SADCRES_12BIT_1S |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ina219WriteRegister16(sensor, INA219_REG_CONFIG, sensor->config);

    return 0;
}


/**
 * @brief Sets the configuration register of the ina219 sensor. Have to provide all of the configuration bits 
 * 
 * @param sensor The ina219 sensor to edit the configuration of
 * @param BRNG The bus voltage range setting
 * @param PG The PGA gain setting
 * @param BADC The bus ADC setting
 * @param SADC The shunt ADC setting
 * @param MODE The operating mode
 */
void ina219SetConfiguration_All(struct ina219 *sensor, uint16_t RST, uint16_t BRNG, uint16_t PG, uint16_t BADC, uint16_t SADC, uint16_t MODE) {
    sensor->config = RST | BRNG | PG | BADC | SADC | MODE;

    ina219WriteRegister16(sensor, INA219_REG_CONFIG, sensor->config);
}


/**
 * @brief Sets a section of the ina219 configuration register
 * 
 * @param sensor The ina219 sensor to edit the configuration of
 * @param MASK The mask of the configuration register bits to change
 * @param CONFIG The bits to change the configuration register to
 */
void ina219SetConfiguration_Selection(struct ina219 *sensor, uint16_t MASK, uint16_t CONFIG) {
    sensor->config = (sensor->config & ~MASK) | CONFIG;
    ina219WriteRegister16(sensor, INA219_REG_CONFIG, sensor->config);
}


/**
 * @brief Gets the raw shunt voltage register value
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The raw shunt voltage register value (16-bit signed integer, so +-32767)
 */
int16_t ina219GetShuntVoltage_Raw(struct ina219 *sensor) {
    return ina219Read16(sensor, INA219_REG_SHUNTVOLTAGE);
}


/**
 * @brief Gets the value of the shunt voltage in V
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The shunt voltage in V 
 */
double ina219GetShuntVoltage_V(struct ina219 *sensor) {
    return ina219GetShuntVoltage_Raw(sensor) * 0.00001;
}


/**
 * @brief Gets the value of the shunt voltage in mV
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The shunt voltage in mV
 */
double ina219GetShuntVoltage_mV(struct ina219 *sensor) {
    return ina219GetShuntVoltage_V(sensor) * 1000;
}


/**
 * @brief Gets the raw bus voltage register value
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The raw bus voltage register value (16-bit signed integer, so +-32767)
 */
int16_t ina219GetBusVoltage_Raw(struct ina219 *sensor) {
    uint16_t value = ina219Read16(sensor, INA219_REG_BUSVOLTAGE);

    // Shift to the right 3 to drop CNVR and OVF
    return (value >> 3);
}


/**
 * @brief Gets the value of the bus voltage in V
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The bus voltage in V 
 */
double ina219GetBusVoltage_V(struct ina219 *sensor) {
    return ina219GetBusVoltage_Raw(sensor) * 0.004;
}


/**
 * @brief Gets the value of the bus voltage in mV
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The bus voltage in mV
 */
double ina219GetBusVoltage_mV(struct ina219 *sensor) {
    return ina219GetBusVoltage_V(sensor) * 1000;
}


/**
 * @brief Gets the raw current register value
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The raw power register value (16-bit signed integer, so +-32767)
 */
int16_t ina219GetCurrent_Raw(struct ina219 *sensor) {
    return ina219Read16(sensor, INA219_REG_CURRENT);
}


/**
 * @brief Gets the measured current in A
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The current in A
 */
double ina219GetCurrent_A(struct ina219 *sensor) {
    return ina219GetCurrent_Raw(sensor) * sensor->ina219_current_LSB;
}


/**
 * @brief Gets the measured current in mA
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The current in mA
 */
double ina219GetCurrent_mA(struct ina219 *sensor) {
    return ina219GetCurrent_A(sensor) * 1000;
}
/**
 * @brief Gets the raw power register value
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The raw power register value (16-bit signed integer, so +-32767)
 */
int16_t ina219GetPower_Raw(struct ina219 *sensor) {
    return ina219Read16(sensor, INA219_REG_POWER);
}


/**
 * @brief Gets the measured power in W
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The power in W
 */
double ina219GetPower_W(struct ina219 *sensor) {
    return  ina219GetPower_Raw(sensor) * sensor->ina219_power_LSB;
}


/**
 * @brief Gets the measured power in mW
 * 
 * @param sensor The ina219 sensor to read from
 * @return int16_t The power in mW
 */
double ina219GetPower_mW(struct ina219 *sensor) {
    return  ina219GetPower_W(sensor) * 1000;
}


/**
 * @brief Writes the input value to the register of the specifed sensor
 * 
 * @param sensor The ina219 sensor to write to
 * @param reg The 8 bit address of the register to write to
 * @param value The 16 bit value to write to the register
 */
void ina219WriteRegister16(struct ina219 *sensor, uint8_t reg, uint16_t value) {
    // Flips the upper 8 bits with the lower 8 bits
    uint16_t data = (value >> 8 | 0xFF00) & (value << 8 | 0xFF);

    // Writes the 16 bit value to the specified register
    wiringPiI2CWriteReg16(sensor->fd, reg, data);
}


/**
 * @brief Reads a value from the register of the specifed sensor
 * 
 * @param sensor The ina219 sensor to read from
 * @param reg The 8 bit address of the register to read from
 * @return uint16_t The 16 bit value that was read from the register
 */
uint16_t ina219Read16(struct ina219 *sensor, uint8_t reg) {
    // Reads a 16 bit integer from the specified register
    uint16_t data = wiringPiI2CReadReg16(sensor->fd, reg);

    // Flips the upper 8 bits with the lower 8 bits and returns the result
    return (data << 8 | 0x00FF) & (data >> 8 | 0xFF00);
}
