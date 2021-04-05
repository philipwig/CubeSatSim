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

#define A 1
#define B 2
#define C 3
#define D 4

#define PLUS_X 0
#define PLUS_Y 1
#define BAT 2
#define BUS 3
#define MINUS_X 4
#define MINUS_Y 5
#define PLUS_Z 6
#define MINUS_Z 7

#define OFF - 1
#define ON 1

#define AFSK 1
#define FSK 2
#define BPSK 3
#define CW 4

void get_tlm();

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

int i2c_bus0 = OFF, i2c_bus1 = OFF, i2c_bus3 = OFF, camera = OFF, sim_mode = FALSE, rxAntennaDeployed = 0, txAntennaDeployed = 0;

char pythonStr[100], pythonConfigStr[100], busStr[10];
int map[8] = {0, 1, 2, 3, 4, 5, 6, 7};
float voltage_min[9], current_min[9], voltage_max[9], current_max[9], sensor_max[17], sensor_min[17], other_max[3], other_min[3];

int main(int argc, char * argv[]) {

  mode = FSK;
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
  char filePath[PATH_MAX + 500]; // Able to store path to files as needed. Max length of 500 more than filepath max

  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    printf("Current working dir: %s\n", cwd);
  } 
  else {
    strcpy(cwd, "/home/pi/CubeSatSim");
    printf("Current working dir not found. Using default: %s\n", cwd);
  }
  
  // Open configuration file with callsign and reset count	

  sprintf(filePath, "%s%s", cwd, "/sim.cfg");
  FILE * config_file = fopen(filePath, "r");

  // If there is no config file, create a blank one and open it
  if (config_file == NULL) {
    printf("Creating config file.");
    
    sprintf(filePath, "%s%s", cwd, "/sim.cfg");
    config_file = fopen(filePath, "w");
    
    fprintf(config_file, "%s %d", " ", 100);
    fclose(config_file);

    config_file = fopen(filePath, "r");
  }

  // Read in the callsign, reset count, latitude and longitude from the config file
//  char * cfg_buf[100];
  fscanf(config_file, "%s %d %f %f", call, & reset_count, & lat_file, & long_file);
  fclose(config_file);

  printf("Config file %s/sim.cfg contains %s %d %f %f\n", cwd, call, reset_count, lat_file, long_file);
  reset_count = (reset_count + 1) % 0xffff;

  if ((fabs(lat_file) > 0) && (fabs(lat_file) < 90.0) && (fabs(long_file) > 0) && (fabs(long_file) < 180.0)) {
    printf("Valid latitude and longitude in config file\n");
    latitude = lat_file;
    longitude = long_file;
  }

  // Setup the wiringpi library
  wiringPiSetup();

  txLed = 3; // defaults for vB3 board without TFB
  txLedOn = LOW;
  txLedOff = HIGH;

  // pinMode (0, OUTPUT);
  // pinMode (2, OUTPUT);
  // pinMode (3, INPUT);
  // pullUpDnControl (3, PUD_UP);

  // Set txLed to the correct pin
  pinMode(txLed, OUTPUT);
  digitalWrite(txLed, txLedOff);
  
  #ifdef DEBUG_LOGGING
  printf("Tx LED Off\n");
  #endif

  // Set power led to the correct pin
  pinMode(onLed, OUTPUT);
  digitalWrite(onLed, onLedOn);

  #ifdef DEBUG_LOGGING
  printf("Power LED On\n");
  #endif

  // Writes callsign, reset count, latitude and longitude to the sim.cfg file
  config_file = fopen("sim.cfg", "w");
  fprintf(config_file, "%s %d %8.4f %8.4f", call, reset_count, lat_file, long_file);
  //    fprintf(config_file, "%s %d", call, reset_count);
  fclose(config_file);
  config_file = fopen("sim.cfg", "r");

  /*
    Edit and change so the pin assignments are correct
  */
  // Changes map values and tests i2c buses
  if (vB4) {
    map[BAT] = BUS;
    map[BUS] = BAT;
    snprintf(busStr, sizeof(busStr), "%d %d", test_i2c_bus(1), test_i2c_bus(0));
  } 
  else if (vB5) {
    map[MINUS_X] = MINUS_Y;
    map[PLUS_Z] = MINUS_X;	
    map[MINUS_Y] = PLUS_Z;		  

    if (access("/dev/i2c-11", W_OK | R_OK) >= 0) { // Test if I2C Bus 11 is present			
      printf("/dev/i2c-11 is present\n\n");
      snprintf(busStr, sizeof(busStr), "%d %d", test_i2c_bus(1), test_i2c_bus(11));
    } 
    else {
      snprintf(busStr, 10, "%d %d", test_i2c_bus(1), test_i2c_bus(3));
    }
  } 
  else {
    map[BUS] = MINUS_Z;
    map[BAT] = BUS;
    map[PLUS_Z] = BAT;
    map[MINUS_Z] = PLUS_Z;
    snprintf(busStr, 10, "%d %d", test_i2c_bus(1), test_i2c_bus(0));
    batteryThreshold = 8.0;
  }

  // Create the python command string using the correct i2c buses
  // Creats the python command to read the i2c sensors. Example: "python3 /home/pi/CubeSatSim/python/voltcurrent.py 1 11"
  snprintf(pythonStr, sizeof(pythonStr), "%s %s%s %s", "python3", cwd, "/python/voltcurrent.py", busStr);

  // strcpy(pythonStr, pythonCmd);
  // strcat(pythonStr, busStr);
  strcpy(pythonConfigStr, pythonStr);
  strcat(pythonConfigStr, " c");

  //   FILE* file1 = popen("python3 /home/pi/CubeSatSim/python/voltcurrent.py 1 11 c", "r");
  printf("\n\nRunning voltcurrent.py in configure mode");
  FILE * file1 = popen(pythonConfigStr, "r");

  // Read the results of the python script
  char cmdbuffer[1000];
  fgets(cmdbuffer, 1000, file1);
  printf("\nvoltcurrent.py result: %s\n", cmdbuffer);

  pclose(file1);

  // Try connecting to Arduino payload using UART
  if (!ax5043 && !vB3) { // don't test if AX5043 is present
    printf("\nTrying to connect to Ardunio payload\n");
    payload = OFF;

    if ((uart_fd = serialOpen("/dev/ttyAMA0", 9600)) >= 0) {
      char c;
      int charss = (char) serialDataAvail(uart_fd);
      if (charss != 0) printf("Clearing buffer of %d chars \n", charss);
      while ((charss--> 0)) c = (char) serialGetchar(uart_fd); // clear buffer

      unsigned int waitTime;

      for (int i = 0; i < 2; i++) {
        serialPutchar(uart_fd, 'R');
        printf("Querying payload with R to reset\n");
        waitTime = millis() + 500;

        while ((millis() < waitTime) && (payload != ON)) {
          if (serialDataAvail(uart_fd)) {
            printf("%c", c = (char) serialGetchar(uart_fd));
            fflush(stdout);
            
            if (c == 'O') {
              printf("%c", c = (char) serialGetchar(uart_fd));
              fflush(stdout);

              if (c == 'K') payload = ON;
            }
          }
          //        sleep(0.75);
        }
      }
      if (payload == ON)
        printf("\nPayload is present!\n");
      else
        printf("\nPayload not present!\n");
    } 
    else {
      fprintf(stderr, "Unable to open UART: %s\n", strerror(errno));
    }
  }

  // test i2c buses	
  // i2c_bus0 = (test_i2c_bus(0) != -1) ? ON : OFF;
  // i2c_bus1 = (test_i2c_bus(1) != -1) ? ON : OFF;
  // i2c_bus3 = (test_i2c_bus(3) != -1) ? ON : OFF;
  // printf("All i2c buses tested!");

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

  // if ((i2c_bus1 == OFF) && (i2c_bus3 == OFF)) {
  if (i2c_bus3 == OFF) {  // i2c bus 13 can be turned off manually by editing /boot/config.txt

    sim_mode = TRUE;

    printf("Simulated telemetry mode!\n");

    srand((unsigned int)time(0));

    
    #ifdef DEBUG_LOGGING
    for (int i = 0; i < 3; i++)
      printf("axis: %f angle: %f v: %f i: %f \n", axis[i], angle[i], volts_max[i], amps_max[i]);
    printf("batt: %f speed: %f eclipse_time: %f eclipse: %f period: %f temp: %f max: %f min: %f\n", batt, speed, eclipse_time, eclipse, period, tempS, temp_max, temp_min);
    #endif


  }
  
  // delay awaiting CW ID completion
  //if (mode == AFSK) sleep(10); 

  // if (transmit == FALSE) {
  //   fprintf(stderr, "\nNo CubeSatSim Band Pass Filter detected.  No transmissions after the CW ID.\n");
  //   fprintf(stderr, " See http://cubesatsim.org/wiki for info about building a CubeSatSim\n\n");
  // }

  for (int i = 0; i < 9; i++) {
    voltage_min[i] = 1000.0;
    current_min[i] = 1000.0;
    voltage_max[i] = -1000.0;
    current_max[i] = -1000.0;
  }
  for (int i = 0; i < 17; i++) {
    sensor_min[i] = 1000.0;
    sensor_max[i] = -1000.0;
  }

  printf("Sensor min and max initialized!");

  for (int i = 0; i < 3; i++) {
    other_min[i] = 1000.0;
    other_max[i] = -1000.0;
  }

  printf("\n\nStarted transmission!\n\n");

  // Main loop
  while (loop-- != 0) {
    frames_sent++;

    #ifdef DEBUG_LOGGING
    fprintf(stderr, "INFO: Battery voltage: %f V  Battery Threshold %f V\n", batteryVoltage, batteryThreshold);
    #endif

    if ((batteryVoltage > 1.0) && (batteryVoltage < batteryThreshold)) { // no battery INA219 will give 0V, no battery plugged into INA219 will read < 1V
      fprintf(stderr, "Battery voltage too low: %f V - shutting down!\n", batteryVoltage);
      digitalWrite(txLed, txLedOff);
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

    //  sleep(1);  // Delay 1 second
    #ifdef DEBUG_LOGGING
    fprintf(stderr, "INFO: Getting TLM Data\n");
    #endif

    // Gets the telemetry data
    if ((mode == AFSK) || (mode == CW)) {
      get_tlm();
    } 
    else { // FSK or BPSK
      printf("\nFSK or BPSK not enabled right now!");
      break;   
    }

    #ifdef DEBUG_LOGGING
    fprintf(stderr, "INFO: Getting ready to send\n");
    #endif
  }

  if (mode == BPSK) {
    digitalWrite(txLed, txLedOn);

    #ifdef DEBUG_LOGGING
    printf("Tx LED On\n");
    #endif

    printf("\nSleeping to allow BPSK transmission to finish.");
    sleep((unsigned int)(loop_count * 5));
    printf("\nDone sleeping");
    digitalWrite(txLed, txLedOff);

    #ifdef DEBUG_LOGGING
    printf("Tx LED Off\n");
    #endif

  } 
  else if (mode == FSK) {
    printf("\nSleeping to allow FSK transmission to finish.");
    sleep((unsigned int)loop_count);
    printf("\nDone sleeping");
  }

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

void get_tlm(void) {

  //FILE * txResult;

  for (int j = 0; j < frameCnt; j++) {
    digitalWrite(txLed, txLedOn);
    
    #ifdef DEBUG_LOGGING
    printf("Tx LED On\n");
    #endif

    // Creates tlm array and sets it all to 0
    int tlm[7][5];
    memset(tlm, 0, sizeof tlm);

    //  Reading I2C voltage and current sensors

    int count1;
    char * token;
    char cmdbuffer[1000];

    // Calls voltcurrent.py with I2C buses
    // FILE * file = popen(pythonStr, "r");
    // fgets(cmdbuffer, 1000, file);
    // //   printf("result: %s\n", cmdbuffer);
    // pclose(file);

    const char space[2] = " ";
    token = strtok(cmdbuffer, space);

    float voltage[9], current[9];

    memset(voltage, 0, sizeof(voltage));
    memset(current, 0, sizeof(current));

    // Stores the voltage and current data read by the python script
    for (count1 = 0; count1 < 8; count1++) {
      if (token != NULL) {
        voltage[count1] = (float) atof(token);

        #ifdef DEBUG_LOGGING
        //		 printf("voltage: %f ", voltage[count1]);
        #endif

        token = strtok(NULL, space);
        if (token != NULL) {
          current[count1] = (float) atof(token);
          if ((current[count1] < 0) && (current[count1] > -0.5))
            current[count1] *= (-1);

          #ifdef DEBUG_LOGGING
          //		    printf("current: %f\n", current[count1]);
          #endif

          token = strtok(NULL, space);
        }
      }
    }

    batteryVoltage = voltage[map[BAT]];

    double cpuTemp;

    FILE * cpuTempSensor = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (cpuTempSensor) {
      fscanf(cpuTempSensor, "%lf", & cpuTemp);
      cpuTemp /= 1000;

      #ifdef DEBUG_LOGGING
      printf("CPU Temp Read: %6.1f\n", cpuTemp);
      #endif

    }
    fclose(cpuTempSensor);

    if (sim_mode) {
      gen_tlm(current, voltage, map);
    }

    tlm[1][A] = (int)(voltage[map[BUS]] / 15.0 + 0.5) % 100; // Current of 5V supply to Pi
    tlm[1][B] = (int)(99.5 - current[map[PLUS_X]] / 10.0) % 100; // +X current [4]
    tlm[1][C] = (int)(99.5 - current[map[MINUS_X]] / 10.0) % 100; // X- current [10] 
    tlm[1][D] = (int)(99.5 - current[map[PLUS_Y]] / 10.0) % 100; // +Y current [7]

    tlm[2][A] = (int)(99.5 - current[map[MINUS_Y]] / 10.0) % 100; // -Y current [10] 
    tlm[2][B] = (int)(99.5 - current[map[PLUS_Z]] / 10.0) % 100; // +Z current [10] // was 70/2m transponder power, AO-7 didn't have a Z panel
    tlm[2][C] = (int)(99.5 - current[map[MINUS_Z]] / 10.0) % 100; // -Z current (was timestamp)
    tlm[2][D] = (int)(50.5 + current[map[BAT]] / 10.0) % 100; // NiMH Battery current

    tlm[3][A] = abs((int)((voltage[map[BAT]] * 10.0) - 65.5) % 100);
    tlm[3][B] = (int)(voltage[map[BUS]] * 10.0) % 100; // 5V supply to Pi

    tlm[4][B] = (int)((95.8 - cpuTemp) / 1.48 + 0.5) % 100;

    tlm[6][B] = 0;
    tlm[6][D] = 49 + rand() % 3;

    #ifdef DEBUG_LOGGING
    // Display tlm
    int k, j;
    for (k = 1; k < 7; k++) {
      for (j = 1; j < 5; j++) {
        printf(" %2d ", tlm[k][j]);
      }
      printf("\n");
    }
    #endif

    char str[1000];
    char tlm_str[1000];
    //char header_str[] = "\x03\xf0hi hi ";
    char header_str3[] = "echo '";
    //char header_str2[] = ">CQ:>041440zhi hi ";
    //char header_str2[] = ">CQ:=4003.79N\\07534.33WShi hi ";
    char header_str2[] = ">CQ:";
    char header_str2b[30]; // for APRS coordinates
    char header_lat[10];
    char header_long[10];
    char header_str4[] = "hi hi ";
    char footer_str1[] = "\' > t.txt && echo \'";
    char footer_str[] = ">CQ:010101/hi hi ' >> t.txt && gen_packets -o telem.wav t.txt -r 48000 > /dev/null 2>&1 && cat telem.wav | csdr convert_i16_f | csdr gain_ff 7000 | csdr convert_f_samplerf 20833 | sudo /home/pi/rpitx/rpitx -i- -m RF -f 434.9e3 > /dev/null 2>&1";

    strcpy(str, header_str3);

    if (mode != CW) {
      strcat(str, call);
      strcat(str, header_str2);
      //	sprintf(header_str2b, "=%7.2f%c%c%c%08.2f%cShi hi ",4003.79,'N',0x5c,0x5c,07534.33,'W');  // add APRS lat and long
      
      if (latitude > 0) sprintf(header_lat, "%7.2f%c", latitude * 100.0, 'N'); // lat
      else sprintf(header_lat, "%7.2f%c", latitude * (-100.0), 'S'); // lat
      
      if (longitude > 0) sprintf(header_long, "%08.2f%c", longitude * 100.0, 'E'); // long
      else sprintf(header_long, "%08.2f%c", longitude * (-100.0), 'W'); // long
      
      sprintf(header_str2b, "=%s%c%c%sShi hi ", header_lat, 0x5c, 0x5c, header_long); // add APRS lat and long	    
      //printf("\n\nString is %s \n\n", header_str2b);
      strcat(str, header_str2b);
    } 
    else {
      strcat(str, header_str4);
    }

    int channel;
    for (channel = 1; channel < 7; channel++) {
      sprintf(tlm_str, "%d%d%d %d%d%d %d%d%d %d%d%d ",
        channel, upper_digit(tlm[channel][1]), lower_digit(tlm[channel][1]),
        channel, upper_digit(tlm[channel][2]), lower_digit(tlm[channel][2]),
        channel, upper_digit(tlm[channel][3]), lower_digit(tlm[channel][3]),
        channel, upper_digit(tlm[channel][4]), lower_digit(tlm[channel][4]));
      //        printf("%s",tlm_str);
      strcat(str, tlm_str);
    }
    // CW

    char cw_str2[500];
    char cw_header2[] = "echo '";
    char cw_footer2[] = "' > id.txt && gen_packets -M 20 id.txt -o morse.wav -r 48000 > /dev/null 2>&1 && cat morse.wav | csdr convert_i16_f | csdr gain_ff 7000 | csdr convert_f_samplerf 20833 | sudo /home/pi/rpitx/rpitx -i- -m RF -f 434.897e3";

    strcpy(cw_str2, cw_header2);
    //printf("Before 1st strcpy\n");
    strcat(cw_str2, str);
    //printf("Before 1st strcpy\n");
    strcat(cw_str2, cw_footer2);
    //printf("Before 1st strcpy\n");

    // read payload sensor if available
    char sensor_payload[500];
    if (payload == ON) {
      char c;
      int charss = (char) serialDataAvail(uart_fd);
      if (charss != 0)
        printf("Clearing buffer of %d chars \n", charss);
      while ((charss--> 0))
        c = (char) serialGetchar(uart_fd); // clear buffer

      unsigned int waitTime;
      int i = 0;

      serialPutchar(uart_fd, '?');
      printf("Querying payload with ?\n");
      waitTime = millis() + 500;
      int end = FALSE;
      while ((millis() < waitTime) && !end) {
        int chars = (char) serialDataAvail(uart_fd);
        while ((chars--> 0) && !end) {
          c = (char) serialGetchar(uart_fd);
          //	  printf ("%c", c);
          //	  fflush(stdout);
          if (c != '\n') {
            sensor_payload[i++] = c;
          } else {
            end = TRUE;
          }
        }
      }
      //    sensor_payload[i++] = '\n';
      sensor_payload[i] = '\0';
      printf("Payload string: %s", sensor_payload);

      strcat(str, sensor_payload); // append to telemetry string for transmission
    }

    #ifdef DEBUG_LOGGING
    printf("Tx LED On\n");
    #endif

    if (mode == CW) system(cw_str2);
    
    digitalWrite(txLed, txLedOn);

    #ifdef DEBUG_LOGGING
    printf("Tx LED On\n");
    #endif

    strcat(str, footer_str1);
    strcat(str, call);
    strcat(str, footer_str);
    fprintf(stderr, "String to execute: %s\n", str);

    if (transmit) {
      FILE * file2 = popen(str, "r");
      pclose(file2);
    } 
    else {
      fprintf(stderr, "\nNo CubeSatSim Band Pass Filter detected.  No transmissions after the CW ID.\n");
      fprintf(stderr, " See http://cubesatsim.org/wiki for info about building a CubeSatSim\n\n");
    }

    digitalWrite(txLed, txLedOff);

    #ifdef DEBUG_LOGGING
    printf("Tx LED Off\n");
    #endif

    sleep(3);
    digitalWrite(txLed, txLedOn);

    #ifdef DEBUG_LOGGING
    printf("Tx LED On\n");
    #endif
  }

  digitalWrite(txLed, txLedOff);
  #ifdef DEBUG_LOGGING
  printf("Tx LED Off\n");
  #endif

  return;
}