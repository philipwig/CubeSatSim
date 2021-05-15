/**
 * @file ina219.h
 * @author Philip Wig (github.com/philipwig)
 * @brief Driver for the TI INA219 current/power monitor
 * @version 1.0
 * @date 2021-04-04
 * 
 * @copyright Copyright (c) 2021 under the GNU General Public License v3.0
 * 
 */

/*
    TODO:
    Need to add errors for when the devices are not able to be accessed.

*/

#ifndef INA219_H
#define INS219_H

#include <stdint.h>


#define INA219_REG_CONFIG                       0x00    // Configuration register
#define INA219_REG_SHUNTVOLTAGE                 0x01    // Shunt voltage register
#define INA219_REG_BUSVOLTAGE                   0x02    // Bus voltage register
#define INA219_REG_POWER                        0x03    // Power register
#define INA219_REG_CURRENT                      0x04    // Current register
#define INA219_REG_CALIBRATION                  0x05    // Calibration register

#define INA219_CONFIG_RESET                     0x8000  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK        0x4000  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V         0x0000  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V         0x4000  // 0-32V Range
	
#define INA219_CONFIG_GAIN_MASK                 0x1800  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV               0x0000  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV               0x0800  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV              0x1000  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV              0x1800  // Gain 8, 320mV Range
	
#define INA219_CONFIG_BADCRES_MASK              0x0780  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_BADCRES_9BIT_1S           0x0000  // 1 x 9-bit shunt sample, 84us conversion time
#define INA219_CONFIG_BADCRES_10BIT_1S          0x0080  // 1 x 10-bit shunt sample, 148us conversion time
#define INA219_CONFIG_BADCRES_11BIT_1S          0x0100  // 1 x 11-bit shunt sample, 276us conversion time
#define INA219_CONFIG_BADCRES_12BIT_1S          0x0180  // 1 x 12-bit shunt sample, 532us conversion time
#define INA219_CONFIG_BADCRES_12BIT_2S          0x0480	// 2 x 12-bit shunt samples averaged together, 1.06ms conversion time
#define INA219_CONFIG_BADCRES_12BIT_4S          0x0500  // 4 x 12-bit shunt samples averaged together, 2.13ms conversion time
#define INA219_CONFIG_BADCRES_12BIT_8S          0x0580  // 8 x 12-bit shunt samples averaged together, 4.26ms conversion time
#define INA219_CONFIG_BADCRES_12BIT_16S         0x0600  // 16 x 12-bit shunt samples averaged together, 8.51ms conversion time
#define INA219_CONFIG_BADCRES_12BIT_32S         0x0680  // 32 x 12-bit shunt samples averaged together, 17.02ms conversion time
#define INA219_CONFIG_BADCRES_12BIT_64S         0x0700  // 64 x 12-bit shunt samples averaged together, 34.05ms conversion time
#define INA219_CONFIG_BADCRES_12BIT_128S        0x0780  // 128 x 12-bit shunt samples averaged together, 68.10ms conversion time
	
#define INA219_CONFIG_SADCRES_MASK              0x0078  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S           0x0000  // 1 x 9-bit shunt sample, 84us conversion time
#define INA219_CONFIG_SADCRES_10BIT_1S          0x0008  // 1 x 10-bit shunt sample, 148us conversion time
#define INA219_CONFIG_SADCRES_11BIT_1S          0x0010  // 1 x 11-bit shunt sample, 276us conversion time
#define INA219_CONFIG_SADCRES_12BIT_1S          0x0018  // 1 x 12-bit shunt sample, 532us conversion time
#define INA219_CONFIG_SADCRES_12BIT_2S          0x0048	// 2 x 12-bit shunt samples averaged together, 1.06ms conversion time
#define INA219_CONFIG_SADCRES_12BIT_4S          0x0050  // 4 x 12-bit shunt samples averaged together, 2.13ms conversion time
#define INA219_CONFIG_SADCRES_12BIT_8S          0x0058  // 8 x 12-bit shunt samples averaged together, 4.26ms conversion time
#define INA219_CONFIG_SADCRES_12BIT_16S         0x0060  // 16 x 12-bit shunt samples averaged together, 8.51ms conversion time
#define INA219_CONFIG_SADCRES_12BIT_32S         0x0068  // 32 x 12-bit shunt samples averaged together, 17.02ms conversion time
#define INA219_CONFIG_SADCRES_12BIT_64S         0x0070  // 64 x 12-bit shunt samples averaged together, 34.05ms conversion time
#define INA219_CONFIG_SADCRES_12BIT_128S        0x0078  // 128 x 12-bit shunt samples averaged together, 68.10ms conversion time
	
#define INA219_CONFIG_MODE_MASK                 0x0007  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN            0x0000  // Power-down
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED      0x0001  // Shunt voltage, triggered
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED      0x0002  // Bus voltage, triggered
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED  0x0003  // Shunt and bus, triggered
#define INA219_CONFIG_MODE_ADCOFF               0x0004  // ADC off (disabled)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS     0x0005  // Shunt voltage, continous
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS     0x0006  // Bus voltage, continous
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007	// Shunt and bus, continous


/**
 * @brief Structure holding useful data about the ina219 sensor. Should only be changing the devId, maxInputCurrent, and Rshunt
 * 
 */
struct ina219 {
    // User configurable
    const char *device; // The i2c bus ("/dev/i2c-x") the device is connected to
    int devId; // The ID of the ina219 current sensor
    double maxInputCurrent; // The max expected input current to measure
    double Rshunt; // The value of the shunt resistor

    // Non user configurable
    int fd; // The file descriptor of the ina219 from wiringPi
    double ina219_current_LSB; // The current LSB, used in current measurements
    double ina219_power_LSB; // The power LSB, used in power measurments
    uint16_t config; // The value of the configuration register on the ina219
};

void     ina219Init(struct ina219 *sensor);
void     ina219SetConfiguration_All(struct ina219 *sensor, uint16_t RST, uint16_t BRNG, uint16_t PG, uint16_t BADC, uint16_t SADC, uint16_t MODE);
void     ina219SetConfiguration_Selection(struct ina219 *sensor, uint16_t MASK, uint16_t CONFIG);


int16_t  ina219GetShuntVoltage_Raw(struct ina219 *sensor);
double   ina219GetShuntVoltage_V(struct ina219 *sensor);
double   ina219GetShuntVoltage_mV(struct ina219 *sensor);

int16_t  ina219GetBusVoltage_Raw(struct ina219 *sensor);
double   ina219GetBusVoltage_V(struct ina219 *sensor);
double   ina219GetBusVoltage_mV(struct ina219 *sensor);

int16_t  ina219GetPower_Raw(struct ina219 *sensor);
double   ina219GetPower_W(struct ina219 *sensor);
double   ina219GetPower_mW(struct ina219 *sensor);

int16_t  ina219GetCurrent_Raw(struct ina219 *sensor);
double   ina219GetCurrent_A(struct ina219 *sensor);
double   ina219GetCurrent_mA(struct ina219 *sensor);

void     ina219WriteRegister16(struct ina219 *sensor, uint8_t reg, uint16_t value);
uint16_t ina219Read16(struct ina219 *sensor, uint8_t reg);

#endif
