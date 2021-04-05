#include "ina219.h"
#include <stdio.h>
#include <unistd.h>

// Assumes little endian
void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
}

int main() {
    struct ina219 sensor1;
    sensor1.devId = 0x40;
    sensor1.Rshunt = 0.1;
    sensor1.maxInputCurrent = 0.5;

    struct ina219 sensor2;
    sensor2.devId = 0x41;
    sensor2.Rshunt = 0.1;
    sensor2.maxInputCurrent = 0.5;

    ina219Init(&sensor1);
    printf("\nsensor1 config register: ");
    printBits(sizeof(sensor1.config), &sensor1.config);

    ina219Init(&sensor2);
    ina219SetConfiguration_Selection(&sensor2, INA219_CONFIG_SADCRES_MASK, INA219_CONFIG_SADCRES_12BIT_128S);
    printf("\nsensor2 config register: ");
    printBits(sizeof(sensor2.config), &sensor2.config);
    double measurement;

    while(1) {
        measurement = ina219GetCurrent_mA(&sensor1);
        printf("\nSensor 1 current: %f mA", measurement);

        measurement = ina219GetCurrent_mA(&sensor2);
        printf("\tSensor 2 current: %f mA", measurement);
        usleep(100000);
    }
  
    // printf("\ninput maxInputCurrent: %f", sensor1.maxInputCurrent);
    // printf("\nInput Rshunt: %f", sensor1.Rshunt);

    // printf("\nina219_current_LSB: %f", sensor1.ina219_current_LSB);
    // printf("\nina219_power_LSB: %f", sensor1.ina219_power_LSB);
    // printf("\nnew maxInputCurrent: %f", sensor1.maxInputCurrent);

    // printf("\nconfig: %i  ", sensor1.config);
    // printBits(sizeof(sensor1.config), &sensor1.config);
    // printf("\n");

    // for (int i = 0; i < 10; i++) {
    //     int16_t data;
    //     double measurement;

    //     data = ina219GetShuntVoltage_Raw(&sensor1);
    //     printf("\nina219GetShuntVoltage_Raw: %i ", data);
    //     printf("\tRaw bits: ");
    //     printBits(sizeof(data), &data);

    //     measurement = ina219GetShuntVoltage_V(&sensor1);
    //     printf("\nina219GetShuntVoltage_V: %f ", measurement);

    //     measurement = ina219GetShuntVoltage_mV(&sensor1);
    //     printf("\nina219GetShuntVoltage_mV: %f ", measurement);



    //     data = ina219GetBusVoltage_Raw(&sensor1);
    //     printf("\nina219GetBusVoltage_Raw: %i ", data);
    //     printf("\tRaw bits: ");
    //     printBits(sizeof(data), &data);

    //     measurement = ina219GetBusVoltage_V(&sensor1);
    //     printf("\nina219GetBusVoltage_V: %f ", measurement);

    //     measurement = ina219GetBusVoltage_mV(&sensor1);
    //     printf("\nina219GetBusVoltage_mV: %f ", measurement);



    //     data = ina219GetPower_Raw(&sensor1);
    //     printf("\nina219GetPower_Raw: %i ", data);
    //     printf("\tRaw bits: ");
    //     printBits(sizeof(data), &data);

    //     measurement = ina219GetPower_W(&sensor1);
    //     printf("\nina219GetPower_W: %f ", measurement);

    //     measurement = ina219GetPower_mW(&sensor1);
    //     printf("\nina219GetPower_mW: %f ", measurement);



    //     data = ina219GetCurrent_Raw(&sensor1);
    //     printf("\nina219GetCurrent_Raw: %i ", data);
    //     printf("\tRaw bits: ");
    //     printBits(sizeof(data), &data);

    //     measurement = ina219GetCurrent_A(&sensor1);
    //     printf("\nina219GetCurrent_A: %f ", measurement);

    //     measurement = ina219GetCurrent_mA(&sensor1);
    //     printf("\nina219GetCurrent_mA: %f ", measurement);


    //     printf("\n");


    //     usleep(10000); // Cannot go lower than 600uS as that is max speed of ina219 at 12bit resolution
    // }


    return 1;
}
