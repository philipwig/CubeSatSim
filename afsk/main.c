/*
 *  Transmits CubeSat Telemetry at 434.9MHz in AFSK, FSK, or BPSK format
 *
 *  Copyright Alan B. Johnston
 *
 *  Portions Copyright (C) 2018 Jonathan Brandenburg
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Helpful C libraries
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <limits.h>

// Wiring Pi Library
#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>

#include "utils.h"

#include "drivers/ina219/ina219.h"
#include "constants.h"

#define DEBUG_LOGGING


void get_telemetry_str(char *tlm_str, struct ina219 *sensors, int len);
int upper_digit(int number);
int lower_digit(int number);

int loop = -1, loop_count = 0;

int uart_fd;

int reset_count;
char call[5];

int mode, frameCnt;
int frames_sent = 0;
int cw_id = ON;

int vB4 = FALSE, vB5 = FALSE, vB3 = FALSE, ax5043 = FALSE, transmit = TRUE, onLed, onLedOn, onLedOff, txLed, txLedOn, txLedOff, payload = OFF;

float batteryThreshold = 3.0, batteryVoltage;
float latitude = 41.462399f, longitude = -87.038309f;
float lat_file, long_file;

int i2c_bus0 = OFF, i2c_bus1 = OFF, i2c_bus3 = OFF, camera = OFF, sim_mode = TRUE, rxAntennaDeployed = 0, txAntennaDeployed = 0;

char pythonStr[100], pythonConfigStr[100], busStr[10];
int map[8] = {0, 1, 2, 3, 4, 5, 6, 7};








int main(int argc, char * argv[]) {
    int num_resets;
    char callsign[10];
    char latlong_str[20];
    char tlm_str[1000];
    char command_str[1000];



    mode = AFSK;
    frameCnt = 1;





    if (argc > 1) {
        // Sets the transmit modulation type
        if ( * argv[1] == 'b') {
            mode = BPSK;
            printf("Mode BPSK\n");
        } 
        else if ( * argv[1] == 'a') {
            mode = AFSK;
            printf("Mode AFSK\n");
        } 
        else if ( * argv[1] == 'c') {
            mode = CW;
            printf("Mode CW\n");
        } 
        else {
            printf("Mode FSK\n");
        }

        if (argc > 2) {
            //		  printf("String is %s %s\n", *argv[2], argv[2]);
            loop = atoi(argv[2]);
            loop_count = loop;
        }
        
        printf("Looping %d times \n", loop);

        if (argc > 3) {
            if ( * argv[3] == 'n') {
                cw_id = OFF;
                printf("No CW id\n");
            }
        }
    }






    // Get the current working directory where the program was run
    char cwd[PATH_MAX]; // Working directory path
    char file_path[PATH_MAX]; // Able to store path to files as needed. Max length of 500 more than filepath max

    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        printf("\nCurrent working dir: %s", cwd);
    } 
    else {
        strcpy(cwd, "/home/pi/CubeSatSim");
        printf("\nCurrent working dir not found. Using default: %s", cwd);
    }

  

    // Read config file function
    /* ===================================================================================================== */
    sprintf(file_path, "%s%s", cwd, "/sim.cfg");
    read_config_file(file_path, mode, callsign, latlong_str, &num_resets);
    /* ===================================================================================================== */




    // Setup the wiringpi library
    wiringPiSetup();

    // Initilizes the ina219 sensors
    struct ina219 current_sensors[8];

    current_sensors[0].device = "/dev/i2c-1"; current_sensors[0].devId = 0x40;
    current_sensors[1].device = "/dev/i2c-1"; current_sensors[1].devId = 0x41;
    current_sensors[2].device = "/dev/i2c-1"; current_sensors[2].devId = 0x44;
    current_sensors[3].device = "/dev/i2c-1"; current_sensors[3].devId = 0x45;

    current_sensors[4].device = "/dev/i2c-3"; current_sensors[4].devId = 0x40;
    current_sensors[5].device = "/dev/i2c-3"; current_sensors[5].devId = 0x41;
    current_sensors[6].device = "/dev/i2c-3"; current_sensors[6].devId = 0x44;
    current_sensors[7].device = "/dev/i2c-3"; current_sensors[7].devId = 0x45;

    for(int i = 0; i < 8; i++) {
        current_sensors[i].Rshunt = 0.1;
        current_sensors[i].maxInputCurrent = 0.5;
        ina219SetConfiguration_Selection(&current_sensors[i], INA219_CONFIG_SADCRES_MASK, INA219_CONFIG_SADCRES_12BIT_128S);
        ina219Init(&current_sensors[i]);
    }


    // Make a function, setupLEDS or just leave. Only has support for vB5 board
    /* ===================================================================================================== */
    pinMode(26, INPUT);
    pullUpDnControl(26, PUD_UP);

    printf("\nvB5 Present\n");
    txLed = 2;
    txLedOn = HIGH;
    txLedOff = LOW;
    vB5 = TRUE;
    onLed = 27;
    onLedOn = HIGH;
    onLedOff = LOW;
    transmit = TRUE;


    pinMode(txLed, OUTPUT);
    digitalWrite(txLed, txLedOff);
    #ifdef DEBUG_LOGGING
    printf("Tx LED Off\n");
    #endif
    pinMode(onLed, OUTPUT);
    digitalWrite(onLed, onLedOn);
    #ifdef DEBUG_LOGGING
    printf("Power LED On\n");
    #endif
    /* ===================================================================================================== */


    // Edit and change so the pin assignments are correct
    /* ===================================================================================================== */
    // Changes map values and tests i2c buses
    // if (vB4) {
    //     map[BAT] = BUS;
    //     map[BUS] = BAT;
    //     snprintf(busStr, sizeof(busStr), "%d %d", test_i2c_bus(1), test_i2c_bus(0));
    //     printf("\n0 %d %d", test_i2c_bus(1), test_i2c_bus(0));
    // } 
    // else if (vB5) {
    map[MINUS_X] = MINUS_Y;
    map[PLUS_Z] = MINUS_X;
    map[MINUS_Y] = PLUS_Z;

        // if (access("/dev/i2c-11", W_OK | R_OK) >= 0) { // Test if I2C Bus 11 is present			
        //     printf("/dev/i2c-11 is present\n\n");
        //     snprintf(busStr, sizeof(busStr), "%d %d", test_i2c_bus(1), test_i2c_bus(11));
        //     printf("\nCase1 %d %d", test_i2c_bus(1), test_i2c_bus(11));
        // } 
        // else {

    // snprintf(busStr, 10, "%d %d", test_i2c_bus(1), test_i2c_bus(3));

        // }
    // } 
    // else {
    //     map[BUS] = MINUS_Z;
    //     map[BAT] = BUS;
    //     map[PLUS_Z] = BAT;
    //     map[MINUS_Z] = PLUS_Z;
    //     snprintf(busStr, 10, "%d %d", test_i2c_bus(1), test_i2c_bus(0));
    //     printf("\n%d %d", test_i2c_bus(1), test_i2c_bus(0));
    //     batteryThreshold = 8.0;
    // }
    /* ===================================================================================================== */

    // test i2c buses
    printf("Testing I2C Buses\n");
    // i2c_bus0 = (test_i2c_bus(0) != -1) ? ON : OFF;
    i2c_bus1 = (test_i2c_bus(1) != -1) ? ON : OFF;
    i2c_bus3 = (test_i2c_bus(3) != -1) ? ON : OFF;
    printf("All i2c buses tested!\n");





    // Wrap into function connectArdunio
    /* ===================================================================================================== */
    // Try connecting to Arduino payload using UART
    if (!ax5043 && !vB3) { // don't test if AX5043 is present
        payload_init(&uart_fd, &payload);
    }
    /* ===================================================================================================== */






    // Check is a camera is present
    // char cmdbuffer1[1000];
    // FILE * file4 = popen("vcgencmd get_camera", "r");
    // fgets(cmdbuffer1, 1000, file4);
    // char camera_present[] = "supported=1 detected=1";
    // // printf("strstr: %s \n", strstr( & cmdbuffer1, camera_present));
    // camera = (strstr( (const char *)& cmdbuffer1, camera_present) != NULL) ? ON : OFF;
    // printf("Camera result:%s camera: %d \n", & cmdbuffer1, camera);
    // pclose(file4);



    #ifdef DEBUG_LOGGING
    printf("INFO: I2C bus status 0: %d 1: %d 3: %d camera: %d\n", i2c_bus0, i2c_bus1, i2c_bus3, camera);
    #endif




    if ((i2c_bus1 == OFF) && (i2c_bus3 == OFF)) { // i2c bus 13 can be turned off manually by editing /boot/config.txt
        sim_mode = TRUE;
        printf("Simulated telemetry mode!\n");
        srand((unsigned int)time(0));

        // #ifdef DEBUG_LOGGING
        // for (int i = 0; i < 3; i++) printf("axis: %f angle: %f v: %f i: %f \n", axis[i], angle[i], volts_max[i], amps_max[i]);
        // printf("batt: %f speed: %f eclipse_time: %f eclipse: %f period: %f temp: %f max: %f min: %f\n", batt, speed, eclipse_time, eclipse, period, tempS, temp_max, temp_min);
        // #endif
    }
  
    // delay awaiting CW ID completion
    //if (mode == AFSK) sleep(10); 

    printf("\n\nStarted transmission!\n\n");

    // Main loop
    while (loop-- != 0) {
        frames_sent++;

        #ifdef DEBUG_LOGGING
        fprintf(stderr, "INFO: Battery voltage: %f V  Battery Threshold %f V\n", batteryVoltage, batteryThreshold);
        #endif

    // Check battery voltage, if low -> shutdown
    /* ===================================================================================================== */
        if ((batteryVoltage > 1.0) && (batteryVoltage < batteryThreshold)) { // no battery INA219 will give 0V, no battery plugged into INA219 will read < 1V
            fprintf(stderr, "Battery voltage too low: %f V - shutting down!\n", batteryVoltage);
            digitalWrite(txLed, TXLED_OFF);
            digitalWrite(onLed, onLedOff);
            sleep(1);
            digitalWrite(onLed, onLedOn);
            sleep(1);
            digitalWrite(onLed, onLedOff);
            sleep(1);
            digitalWrite(onLed, onLedOn);
            sleep(1);
            digitalWrite(onLed, onLedOff);

            popen("sudo shutdown -h now > /dev/null 2>&1", "r");
            sleep(10);
        }
    /* ===================================================================================================== */

        //  sleep(1);  // Delay 1 second
        #ifdef DEBUG_LOGGING
        fprintf(stderr, "INFO: Getting TLM Data\n");
        #endif

        // Gets the telemetry data and transmits it
        if ((mode == AFSK) || (mode == CW)) {
            get_telemetry_str(tlm_str, current_sensors, 8);


            strcpy(command_str, "");

            strcat(command_str, "echo '");
            strcat(command_str, callsign);
            strcat(command_str, ">CQ:");
            strcat(command_str, latlong_str);
            strcat(command_str, " hi hi ");
            strcat(command_str, tlm_str);
            strcat(command_str, "\' > t.txt && echo \'");
            strcat(command_str, callsign);
            strcat(command_str, ">CQ:010101/hi hi ' >> t.txt && gen_packets -o telem.wav t.txt -r 48000 -b 1200 > /dev/null 2>&1 && cat telem.wav | csdr convert_i16_f | csdr gain_ff 7000 | csdr convert_f_samplerf 20833 | sudo /home/pi/rpitx/rpitx -i- -m RF -f 434.9e3 > /dev/null 2>&1");
            fprintf(stderr, "\nString to execute: %s\n", command_str);


            digitalWrite(txLed, TXLED_ON);

            // Run the command string to transmit the data using csdr and rpitx
            if (transmit) {
                FILE * file2 = popen(command_str, "r");
                pclose(file2);
            } 
            else {
                fprintf(stderr, "\nNo CubeSatSim Band Pass Filter detected.  No transmissions after the CW ID.\n");
                fprintf(stderr, " See http://cubesatsim.org/wiki for info about building a CubeSatSim\n\n");
            }

            digitalWrite(txLed, TXLED_OFF);

            // Sleep for 3 seconds before next transmission
            sleep(3);
        } 
        else { // FSK or BPSK
            printf("\nFSK or BPSK not enabled right now!");
            break;   
        }
    /* ===================================================================================================== */

    }

    // Cleaning up after main loop is finished when cubesatsim is not in continous mode
    /* ===================================================================================================== */

    if (mode == BPSK) {
        digitalWrite(txLed, TXLED_ON);

        #ifdef DEBUG_LOGGING
        printf("Tx LED On\n");
        #endif

        printf("\nSleeping to allow BPSK transmission to finish.");
        sleep((unsigned int)(loop_count * 5));
        printf("\nDone sleeping");
        digitalWrite(txLed, TXLED_OFF);

        #ifdef DEBUG_LOGGING
        printf("Tx LED Off\n");
        #endif

    } 
    else if (mode == FSK) {
        printf("\nSleeping to allow FSK transmission to finish.");
        sleep((unsigned int)loop_count);
        printf("\nDone sleeping");
    }
    /* ===================================================================================================== */


    printf("\n");
    return 0;
}




//  Returns lower digit of a number which must be less than 99
//
int lower_digit(int number) {
    int digit = 0;
    
    if (number < 100)
        digit = number - ((int)(number / 10) * 10);
    else
        fprintf(stderr, "ERROR: Not a digit in lower_digit!\n");
    
    return digit;
}






// Returns upper digit of a number which must be less than 99
//
int upper_digit(int number) {
    int digit = 0;
  
    if (number < 100)
        digit = (int)(number / 10);
    else
        fprintf(stderr, "ERROR: Not a digit in upper_digit!\n");
  
    return digit;
}



void get_telemetry_str(char *tlm_str, struct ina219 *sensors, int len) {
    double voltage[9], current[9];

    // Sets arrays to 0
    memset(voltage, 0, sizeof(voltage));
    memset(current, 0, sizeof(current));

    // Creates tlm array and sets it all to 0
    int tlm[7][5];
    memset(tlm, 0, sizeof tlm);


    // Need to add gathering of data from i2c bus
    /* ===================================================================================================== */
    for (int i = 0; i < len; i++) {
      // fprintf(stderr, "\nRead voltage: %s", sensors[i].device);
      // fprintf(stderr, "\tRead voltage: %f", ina219GetCurrent_mA(&sensors[i]));

      voltage[i] = ina219GetBusVoltage_mV(&sensors[i]);
      current[i] = ina219GetCurrent_mA(&sensors[i]);
      
    }


    sprintf(tlm_str, "%05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f ",
            current[0],
            current[1],
            current[2],
            current[3],
            current[4],
            current[5],
            current[6],
            current[7],
            voltage[0],
            voltage[1],
            voltage[2],
            voltage[3],
            voltage[4],
            voltage[5],
            voltage[6],
            voltage[7]);


    // batteryVoltage = voltage[map[BAT]];

    // double cpuTemp = get_cpu_temp();

    // if (sim_mode) {
    //     gen_sim_telemetry(current, voltage, map);
    // }
    // /* ===================================================================================================== */


    // tlm[1][A] = (int)(voltage[map[BUS]] / 15.0 + 0.5) % 100; // Current of 5V supply to Pi
    // tlm[1][B] = (int)(99.5 - current[map[PLUS_X]] / 10.0) % 100; // +X current [4]
    // tlm[1][C] = (int)(99.5 - current[map[MINUS_X]] / 10.0) % 100; // X- current [10] 
    // tlm[1][D] = (int)(99.5 - current[map[PLUS_Y]] / 10.0) % 100; // +Y current [7]

    // tlm[2][A] = (int)(99.5 - current[map[MINUS_Y]] / 10.0) % 100; // -Y current [10] 
    // tlm[2][B] = (int)(99.5 - current[map[PLUS_Z]] / 10.0) % 100; // +Z current [10] // was 70/2m transponder power, AO-7 didn't have a Z panel
    // tlm[2][C] = (int)(99.5 - current[map[MINUS_Z]] / 10.0) % 100; // -Z current (was timestamp)
    // tlm[2][D] = (int)(50.5 + current[map[BAT]] / 10.0) % 100; // NiMH Battery current

    // tlm[3][A] = abs((int)((voltage[map[BAT]] * 10.0) - 65.5) % 100);
    // tlm[3][B] = (int)(voltage[map[BUS]] * 10.0) % 100; // 5V supply to Pi

    // tlm[4][B] = (int)((95.8 - cpuTemp) / 1.48 + 0.5) % 100;

    // tlm[6][B] = 0;
    // tlm[6][D] = 49 + rand() % 3;

    // #ifdef DEBUG_LOGGING
    // // Display tlm
    // int k, j;
    // for (k = 1; k < 7; k++) {
    //     for (j = 1; j < 5; j++) {
    //     printf(" %2d ", tlm[k][j]);
    //     }
    //     printf("\n");
    // }
    // #endif

    // // Create the formatted telemetry string
    // for (int channel = 1; channel < 7; channel++) {
    //     sprintf(tlm_str, "%d%d%d %d%d%d %d%d%d %d%d%d ",
    //             channel, upper_digit(tlm[channel][1]), lower_digit(tlm[channel][1]),
    //             channel, upper_digit(tlm[channel][2]), lower_digit(tlm[channel][2]),
    //             channel, upper_digit(tlm[channel][3]), lower_digit(tlm[channel][3]),
    //             channel, upper_digit(tlm[channel][4]), lower_digit(tlm[channel][4]));
        
    //     // printf("%s",tlm_str);
    // }

}
