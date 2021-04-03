
#ifndef UTILS_C
#define UTILS_C

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>

#define PORT 8080

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

#define TEMP 2
#define PRES 3
#define ALT 4
#define HUMI 5
#define GYRO_X 7
#define GYRO_Y 8
#define GYRO_Z 9
#define ACCEL_X 10
#define ACCEL_Y 11
#define ACCEL_Z 12
#define XS1 14
#define XS2 15
#define XS3 16

#define RSSI 0
#define IHU_TEMP 2
#define SPIN 1

#define OFF - 1
#define ON 1

float rnd_float(double min, double max) {   // returns 2 decimal point random number
	
	int val = (rand() % ((int)(max*100) - (int)(min*100) + 1)) + (int)(min*100);
	float ret = ((float)(val)/100);
	
      return(ret);
}

int test_i2c_bus(int bus) {
	int output = bus; // return bus number if OK, otherwise return -1
	char busDev[20] = "/dev/i2c-";
	char busS[5];
	snprintf(busS, 5, "%d", bus);  
	strcat (busDev, busS);	
	printf("I2C Bus Tested: %s \n", busDev);
	
	if (access(busDev, W_OK | R_OK) >= 0)  {   // Test if I2C Bus is present			
//	  	printf("bus is present\n\n");	    
    	  	char result[128];		
    	  	const char command_start[] = "timeout 10 i2cdetect -y ";
		char command[50];
		strcpy (command, command_start);
    	 	strcat (command, busS);
//     	 	printf("Command: %s \n", command);
    	 	FILE *i2cdetect = popen(command, "r");
	
    	 	while (fgets(result, 128, i2cdetect) != NULL) {
    	 		//;
//       	 	printf("result: %s", result);
    	 	}	
    	 	int error = pclose(i2cdetect)/256;
//      	 	printf("%s error: %d \n", &command, error);
    	 	if (error != 0) 
    	 	{	
    	 		printf("ERROR: %sd bus has a problem \n  Check I2C wiring and pullup resistors \n", busDev);
			output = -1;
    		}													
	} 
    else {
    	printf("ERROR: %s bus has a problem \n  Check software to see if I2C enabled \n", busDev);
		output = -1; 
	}

	return(output);	// return bus number or -1 if there is a problem with the bus
}

void gen_tlm(float *current, float *voltage, int *map) {
    static float axis[3], angle[3], volts_max[3], amps_max[3], batt = 0, speed = 0, period = 0, tempS = 0, temp_max = 0, temp_min = 0, eclipse = 0;
    static float Xi = 0, Yi = 0, Zi = 0, Xv = 0, Yv = 0, Zv = 0;
    static double eclipse_time = 0, time = 0;
    static long time_start = 0;
    static float amps_avg = 0;
    static double cpuTemp = 0;
    static float charging = 0;

    amps_avg = rnd_float(150, 300);
    
    batt = rnd_float(3.8, 4.3);
    speed = rnd_float(1.0, 2.5);
    eclipse = (rnd_float(-1, +4) > 0) ? 1.0 : 0.0;
    period = rnd_float(150, 300);
    tempS = rnd_float(20, 55);
    temp_max = rnd_float(50, 70);
    temp_min = rnd_float(10, 20);

    time_start = (long int) millis();
    eclipse = 0.0;
    eclipse_time = (long int)(millis() / 1000.0);
    if (eclipse == 0.0) eclipse_time -= period / 2; // if starting in eclipse, shorten interval	
    
    // simulated telemetry 

    axis[0] = rnd_float(-0.2, 0.2);
    if (axis[0] == 0) axis[0] = rnd_float(-0.2, 0.2);
    axis[1] = rnd_float(-0.2, 0.2);
    axis[2] = (rnd_float(-0.2, 0.2) > 0) ? 1.0 : -1.0;

    angle[0] = (float) atan(axis[1] / axis[2]);
    angle[1] = (float) atan(axis[2] / axis[0]);
    angle[2] = (float) atan(axis[1] / axis[0]);

    volts_max[0] = rnd_float(4.5, 5.5) * (float) sin(angle[1]);
    volts_max[1] = rnd_float(4.5, 5.5) * (float) cos(angle[0]);
    volts_max[2] = rnd_float(4.5, 5.5) * (float) cos(angle[1] - angle[0]);

    amps_max[0] = (amps_avg + rnd_float(-25.0, 25.0)) * (float) sin(angle[1]);
    amps_max[1] = (amps_avg + rnd_float(-25.0, 25.0)) * (float) cos(angle[0]);
    amps_max[2] = (amps_avg + rnd_float(-25.0, 25.0)) * (float) cos(angle[1] - angle[0]);


    time = ((long int) millis() - time_start) / 1000.0;
    
    if ((time - eclipse_time) > period) {
        eclipse = (eclipse == 1) ? 0.0 : 1.0;
        eclipse_time = time;
        printf("\n\nSwitching eclipse mode! \n\n");
    }

    /*
      double Xi = eclipse * amps_max[0] * sin(2.0 * 3.14 * time / (46.0 * speed)) * fabs(sin(2.0 * 3.14 * time / (46.0 * speed))) + rnd_float(-2, 2);	  
      double Yi = eclipse * amps_max[1] * sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14/2.0)) * fabs(sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14/2.0))) + rnd_float(-2, 2);	  
      double Zi = eclipse * amps_max[2] * sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2])  * fabs(sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2])) + rnd_float(-2, 2);
    */
    
    /*	    
    double Xi = eclipse * amps_max[0] * sin(2.0 * 3.14 * time / (46.0 * speed)) + rnd_float(-2, 2);
    double Yi = eclipse * amps_max[1] * sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14 / 2.0)) + rnd_float(-2, 2);
    double Zi = eclipse * amps_max[2] * sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2]) + rnd_float(-2, 2);
    double Xv = eclipse * volts_max[0] * sin(2.0 * 3.14 * time / (46.0 * speed)) + rnd_float(-0.2, 0.2);
    double Yv = eclipse * volts_max[1] * sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14 / 2.0)) + rnd_float(-0.2, 0.2);
    double Zv = 2.0 * eclipse * volts_max[2] * sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2]) + rnd_float(-0.2, 0.2);
    */
    Xi = eclipse * amps_max[0] * (float) sin(2.0 * 3.14 * time / (46.0 * speed)) + rnd_float(-2, 2);
    Yi = eclipse * amps_max[1] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14 / 2.0)) + rnd_float(-2, 2);
    Zi = eclipse * amps_max[2] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2]) + rnd_float(-2, 2);
    Xv = eclipse * volts_max[0] * (float) sin(2.0 * 3.14 * time / (46.0 * speed)) + rnd_float(-0.2, 0.2);
    Yv = eclipse * volts_max[1] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14 / 2.0)) + rnd_float(-0.2, 0.2);
    Zv = 2.0 * eclipse * volts_max[2] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2]) + rnd_float(-0.2, 0.2);
    
    // printf("Yi: %f Zi: %f %f %f Zv: %f \n", Yi, Zi, amps_max[2], angle[2], Zv);
    current[map[PLUS_X]] = (Xi >= 0) ? Xi : 0;
    current[map[MINUS_X]] = (Xi >= 0) ? 0 : ((-1.0f) * Xi);
    current[map[PLUS_Y]] = (Yi >= 0) ? Yi : 0;
    current[map[MINUS_Y]] = (Yi >= 0) ? 0 : ((-1.0f) * Yi);
    current[map[PLUS_Z]] = (Zi >= 0) ? Zi : 0;
    current[map[MINUS_Z]] = (Zi >= 0) ? 0 : ((-1.0f) * Zi);
    voltage[map[PLUS_X]] = (Xv >= 1) ? Xv : rnd_float(0.9, 1.1);
    voltage[map[MINUS_X]] = (Xv <= -1) ? ((-1.0f) * Xv) : rnd_float(0.9, 1.1);
    voltage[map[PLUS_Y]] = (Yv >= 1) ? Yv : rnd_float(0.9, 1.1);
    voltage[map[MINUS_Y]] = (Yv <= -1) ? ((-1.0f) * Yv) : rnd_float(0.9, 1.1);
    voltage[map[PLUS_Z]] = (Zv >= 1) ? Zv : rnd_float(0.9, 1.1);
    voltage[map[MINUS_Z]] = (Zv <= -1) ? ((-1.0f) * Zv) : rnd_float(0.9, 1.1);
    
    // printf("temp: %f Time: %f Eclipse: %d : %f %f | %f %f | %f %f\n",tempS, time, eclipse, voltage[map[PLUS_X]], voltage[map[MINUS_X]], voltage[map[PLUS_Y]], voltage[map[MINUS_Y]], current[map[PLUS_Z]], current[map[MINUS_Z]]);
    tempS += (eclipse > 0) ? ((temp_max - tempS) / 50.0) : ((temp_min - tempS) / 50.0f);
    

    FILE * cpuTempSensor = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (cpuTempSensor) {
        fscanf(cpuTempSensor, "%lf", & cpuTemp);
        cpuTemp /= 1000;
        
        #ifdef DEBUG_LOGGING
        printf("CPU Temp Read: %6.1f\n", cpuTemp);
        #endif
    }
    fclose(cpuTempSensor);

    cpuTemp = tempS + rnd_float(-1.0, 1.0);

    voltage[map[BUS]] = rnd_float(5.0, 5.005);
    current[map[BUS]] = rnd_float(158, 171);

    //  float charging = current[map[PLUS_X]] + current[map[MINUS_X]] + current[map[PLUS_Y]] + current[map[MINUS_Y]] + current[map[PLUS_Z]] + current[map[MINUS_Z]];
    charging = eclipse * (fabs(amps_max[0] * 0.707) + fabs(amps_max[1] * 0.707) + rnd_float(-4.0, 4.0));
    current[map[BAT]] = ((current[map[BUS]] * voltage[map[BUS]]) / batt) - charging;
    
    //  printf("charging: %f bat curr: %f bus curr: %f bat volt: %f bus volt: %f \n",charging, current[map[BAT]], current[map[BUS]], batt, voltage[map[BUS]]);
    batt -= (batt > 3.5) ? current[map[BAT]] / 30000 : current[map[BAT]] / 3000;
    
    if (batt < 3.0) {
        batt = 3.0;
        printf("Safe Mode!\n");
    }
    
    if (batt > 4.5) batt = 4.5;
    voltage[map[BAT]] = batt + rnd_float(-0.01, 0.01);
    // end of simulated telemetry	

}

#endif