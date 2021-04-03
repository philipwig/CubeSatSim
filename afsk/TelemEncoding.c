/*
 * TelemEncoding.c
 *
   Fox-1 telemetry encoder
   January 2014 Phil Karn KA9Q

   This file has two external functions:
      void update_rs(unsigned char parity[32],unsigned char data);
      int encode_8b10b(int *state,int data).

   update_rs() is the Reed-Solomon encoder. Its first argument is the 32-byte
   encoder shift register, the second is the 8-bit data byte being encoded. It updates
   the shift register in place and returns void. At the end of each frame, it contains
   the parities ready for transmission, starting with parity[0].
   Be sure to zero this array before each new frame!

   encode_8b10b() is the 8b10b encoder. Its first argument is a pointer to a single integer
   with the 1-bit encoder state (the current run disparity, or RD). Initialize it to 0
   JUST ONCE at startup (not between frames).
   The second argument is the data byte being encoded. It updates the state and returns
   an integer containing the 10-bit encoded word, right justified.
   Transmit this word from left to right.

   The data argument is an int so it can hold the special value -1 to indicate end of frame;
   it generates the 8b10b control word K.28.5, which is used as an inter-frame flag.

   Some assert() calls are made to verify legality of arguments. These can be turned off in
   production code.


   sample frame transmission code:

   unsigned char data[64]; // Data block to be sent
   unsigned char parity[32]; // RS parities
   void transmit_word(int);  // User provided transmit function: 10 bits of data in bits 9....0
   int state,i;

   state = 0; // Only once at startup, not between frames
   memset(parity,0,sizeof(parity); // Do this before every frame
   // Transmit the data, updating the RS encoder
   for(i=0;i<64;i++){
     update_rs(parity,data[i]);
     transmit_word(encode_8b10b(&state,data[i]);
   }
   // get the RS parities
   for(i=0;i<32;i++)
     transmit_word(encode_8b10b(&state,parity[i]);

   transmit_word(encode_8b10b(&state,-1); // Transmit end-of-frame flag
*/


#include <string.h>
//#include "Fox.h"
//#include "TelemEncoding.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

#define NN (0xff) // Frame size in symbols
#define A0 (NN)   // special value for log(0)


// GF Antilog lookup table table
static unsigned char CCSDS_alpha_to[NN+1] = {
0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x87,0x89,0x95,0xad,0xdd,0x3d,0x7a,0xf4,
0x6f,0xde,0x3b,0x76,0xec,0x5f,0xbe,0xfb,0x71,0xe2,0x43,0x86,0x8b,0x91,0xa5,0xcd,
0x1d,0x3a,0x74,0xe8,0x57,0xae,0xdb,0x31,0x62,0xc4,0x0f,0x1e,0x3c,0x78,0xf0,0x67,
0xce,0x1b,0x36,0x6c,0xd8,0x37,0x6e,0xdc,0x3f,0x7e,0xfc,0x7f,0xfe,0x7b,0xf6,0x6b,
0xd6,0x2b,0x56,0xac,0xdf,0x39,0x72,0xe4,0x4f,0x9e,0xbb,0xf1,0x65,0xca,0x13,0x26,
0x4c,0x98,0xb7,0xe9,0x55,0xaa,0xd3,0x21,0x42,0x84,0x8f,0x99,0xb5,0xed,0x5d,0xba,
0xf3,0x61,0xc2,0x03,0x06,0x0c,0x18,0x30,0x60,0xc0,0x07,0x0e,0x1c,0x38,0x70,0xe0,
0x47,0x8e,0x9b,0xb1,0xe5,0x4d,0x9a,0xb3,0xe1,0x45,0x8a,0x93,0xa1,0xc5,0x0d,0x1a,
0x34,0x68,0xd0,0x27,0x4e,0x9c,0xbf,0xf9,0x75,0xea,0x53,0xa6,0xcb,0x11,0x22,0x44,
0x88,0x97,0xa9,0xd5,0x2d,0x5a,0xb4,0xef,0x59,0xb2,0xe3,0x41,0x82,0x83,0x81,0x85,
0x8d,0x9d,0xbd,0xfd,0x7d,0xfa,0x73,0xe6,0x4b,0x96,0xab,0xd1,0x25,0x4a,0x94,0xaf,
0xd9,0x35,0x6a,0xd4,0x2f,0x5e,0xbc,0xff,0x79,0xf2,0x63,0xc6,0x0b,0x16,0x2c,0x58,
0xb0,0xe7,0x49,0x92,0xa3,0xc1,0x05,0x0a,0x14,0x28,0x50,0xa0,0xc7,0x09,0x12,0x24,
0x48,0x90,0xa7,0xc9,0x15,0x2a,0x54,0xa8,0xd7,0x29,0x52,0xa4,0xcf,0x19,0x32,0x64,
0xc8,0x17,0x2e,0x5c,0xb8,0xf7,0x69,0xd2,0x23,0x46,0x8c,0x9f,0xb9,0xf5,0x6d,0xda,
0x33,0x66,0xcc,0x1f,0x3e,0x7c,0xf8,0x77,0xee,0x5b,0xb6,0xeb,0x51,0xa2,0xc3,0x00,
};

// GF log lookup table. Special value represents log(0)
static unsigned char CCSDS_index_of[NN+1] = {
 A0,  0,  1, 99,  2,198,100,106,  3,205,199,188,101,126,107, 42,
  4,141,206, 78,200,212,189,225,102,221,127, 49,108, 32, 43,243,
  5, 87,142,232,207,172, 79,131,201,217,213, 65,190,148,226,180,
103, 39,222,240,128,177, 50, 53,109, 69, 33, 18, 44, 13,244, 56,
  6,155, 88, 26,143,121,233,112,208,194,173,168, 80,117,132, 72,
202,252,218,138,214, 84, 66, 36,191,152,149,249,227, 94,181, 21,
104, 97, 40,186,223, 76,241, 47,129,230,178, 63, 51,238, 54, 16,
110, 24, 70,166, 34,136, 19,247, 45,184, 14, 61,245,164, 57, 59,
  7,158,156,157, 89,159, 27,  8,144,  9,122, 28,234,160,113, 90,
209, 29,195,123,174, 10,169,145, 81, 91,118,114,133,161, 73,235,
203,124,253,196,219, 30,139,210,215,146, 85,170, 67, 11, 37,175,
192,115,153,119,150, 92,250, 82,228,236, 95, 74,182,162, 22,134,
105,197, 98,254, 41,125,187,204,224,211, 77,140,242, 31, 48,220,
130,171,231, 86,179,147, 64,216, 52,176,239, 38, 55, 12, 17, 68,
111,120, 25,154, 71,116,167,193, 35, 83,137,251, 20, 93,248,151,
 46, 75,185, 96, 15,237, 62,229,246,135,165, 23, 58,163, 60,183,
};

// Only half the coefficients are given here because the
// generator polynomial is palindromic; G0 = G32, G1 = G31, etc.
// Only G16 is unique
static unsigned char CCSDS_poly[] = {
  0,249,  59, 66,  4,  43,126,251, 97,  30,   3,213, 50, 66,170,   5,
  24,
};

static inline int modnn(int x){
  while (x >= NN) {
    x -= NN;
    x = (x >> 8) + (x & NN);
  }
  return x;
}

// Update Reed-Solomon encoder
// parity -> 32-byte reed-solomon encoder state; clear this to zero before each frame
void update_rs(
   unsigned char parity[32], // 32-byte encoder state; zero before each frame
   unsigned char c)          // Current data byte to update
{
  unsigned char feedback;
  int j,t;

  assert(parity != NULL);
  feedback = CCSDS_index_of[c ^ parity[0]];
  if(feedback != A0){ // only if feedback is non-zero
    // Take advantage of palindromic polynomial to halve the multiplies
    // Do G1...G15, which is the same as G17...G31
    for(j=1;j<NP/2;j++){
      t = CCSDS_alpha_to[modnn(feedback + CCSDS_poly[j])];
      parity[j] ^= t;
      parity[NP-j] ^= t;
    }
    // Do G16, which is used in only parity[16]
    t = CCSDS_alpha_to[modnn(feedback + CCSDS_poly[j])];
    parity[j] ^= t;
  }
  // shift left
  memmove(&parity[0],&parity[1],NP-1);
  // G0 is 1 in alpha form, 0 in index form; don't need to multiply by it
  parity[NP-1] = CCSDS_alpha_to[feedback];
  //taskYIELD();
}

#define SYNC  (0x0fa) // K.28.5, RD=-1 
 
void write_little_endian(unsigned int word, int num_bytes, FILE *wav_file)
{
	unsigned buf;
	while(num_bytes>0)
	{   buf = word & 0xff;
		fwrite(&buf, 1,1, wav_file);
		num_bytes--;
	word >>= 8;
	}
}


void write_wave(int i, short int *buffer)
{
		if (mode == FSK)
		{
			if ((ctr - flip_ctr) < smaller)
				buffer[ctr++] = (short int)(0.1 * phase * (ctr - flip_ctr) / smaller);
			else
				buffer[ctr++] = (short int)(0.25 * amplitude * phase);
		}
		else
		{
			if ((ctr - flip_ctr) < smaller)
  		 		buffer[ctr++] = (short int)(amplitude * 0.4 * phase * sin((float)(2*M_PI*i*freq_Hz/S_RATE))); 					
 			else
 		 		buffer[ctr++] = (short int)(amplitude * phase * sin((float)(2*M_PI*i*freq_Hz/S_RATE)));
 		 } 			
//		printf("%d %d \n", i, buffer[ctr - 1]);

}

int encodeA(short int  *b, int index, int val) {
//    printf("Encoding A\n");
    b[index] = val & 0xff;
    b[index + 1] = (short int) ((b[index + 1] & 0xf0) | ((val >> 8) & 0x0f));
    return 0;	
}

int encodeB(short int  *b, int index, int val) {
//    printf("Encoding B\n");
    b[index] =  (short int) ((b[index] & 0x0f)  |  ((val << 4) & 0xf0));
    b[index + 1] = (val >> 4 ) & 0xff;
    return 0;	
}

int twosToInt(int val,int len) {   // Convert twos compliment to integer
// from https://www.raspberrypi.org/forums/viewtopic.php?t=55815
	
      if(val & (1 << (len - 1)))
         val = val - (1 << len);

      return(val);
}





void get_tlm_fox() {

  //  Reading I2C voltage and current sensors

  FILE * uptime_file = fopen("/proc/uptime", "r");
  fscanf(uptime_file, "%f", & uptime_sec);
  uptime = (int) uptime_sec;
  #ifdef DEBUG_LOGGING
  printf("Reset Count: %d Uptime since Reset: %ld \n", reset_count, uptime);
  #endif
  fclose(uptime_file);

  int i;
  //	long int sync = SYNC_WORD;
  long int sync = syncWord;

  smaller = (int) (S_RATE / (2 * freq_Hz));

  //	short int b[DATA_LEN];
  short int b[dataLen];
  memset(b, 0, sizeof(b));

  //	short int h[HEADER_LEN];
  short int h[headerLen];
  memset(h, 0, sizeof(h));

  memset(buffer, 0xa5, sizeof(buffer));

  //	short int b10[DATA_LEN], h10[HEADER_LEN];
  //	short int rs_frame[RS_FRAMES][223];
  //	unsigned char parities[RS_FRAMES][PARITY_LEN],inputByte;
  //	short int b10[dataLen], h10[headerLen];
  short int rs_frame[rsFrames][223];
  unsigned char parities[rsFrames][parityLen], inputByte;

  int id, frm_type = 0x01, STEMBoardFailure = 1, NormalModeFailure = 0, groundCommandCount = 0;
  int PayloadFailure1 = 0, PayloadFailure2 = 0;
  int PSUVoltage = 0, PSUCurrent = 0, Resets = 0, Rssi = 2048;
  int batt_a_v = 0, batt_b_v = 0, batt_c_v = 0, battCurr = 0;
  int posXv = 0, negXv = 0, posYv = 0, negYv = 0, posZv = 0, negZv = 0;
  int posXi = 0, negXi = 0, posYi = 0, negYi = 0, posZi = 0, negZi = 0;
  int head_offset = 0;
  //  int xAngularVelocity = (-0.69)*(-10)*(-10) + 45.3 * (-10) + 2078, yAngularVelocity = (-0.69)*(-6)*(-6) + 45.3 * (-6) + 2078, zAngularVelocity = (-0.69)*(6)*(6) + 45.3 * (6) + 2078; // XAxisAngularVelocity
  //  int xAngularVelocity = 2078, yAngularVelocity = 2078, zAngularVelocity = 2078;  // XAxisAngularVelocity Y and Z set to 0
  // int xAngularVelocity = 2048, yAngularVelocity = 2048, zAngularVelocity = 2048; // XAxisAngularVelocity Y and Z set to 0
  // int RXTemperature = 0, temp = 0, spin = 0;;
  // float xAccel = 0.0, yAccel = 0.0, zAccel = 0.0;
  // float BME280pressure = 0.0, BME280altitude = 0.0, BME280humidity = 0.0, BME280temperature = 0.0;
  // float XSsensor1 = 0.0, XSsensor2 = 0.0, XSsensor3 = 0.0;
  // int sensor1 = 0, sensor2 = 2048, sensor3 = 2048;

  short int buffer_test[bufLen];
  int buffSize;
  buffSize = (int) sizeof(buffer_test);

  if (mode == FSK)
    id = 7;
  else
    id = 0; // 99 in h[6]

  //  for (int frames = 0; frames < FRAME_CNT; frames++) 
  for (int frames = 0; frames < frameCnt; frames++) {

    if (firstTime != ON) {
      // delay for sample period
      digitalWrite(txLed, txLedOn);
      #ifdef DEBUG_LOGGING
      printf("Tx LED On\n");
      #endif

      while ((millis() - sampleTime) < (unsigned int)samplePeriod)
        sleep((unsigned int)sleepTime);

      digitalWrite(txLed, txLedOff);
      #ifdef DEBUG_LOGGING
      printf("Tx LED Off\n");
      #endif

      printf("Sample period: %d\n", millis() - (unsigned int)sampleTime);
      sampleTime = (int) millis();
    } else
      printf("first time - no sleep\n");

    int count1;
    char * token;
    char cmdbuffer[1000];

    FILE * file = popen(pythonStr, "r");
    fgets(cmdbuffer, 1000, file);
    //  printf("result: %s\n", cmdbuffer);
    pclose(file);

    const char space[2] = " ";
    token = strtok(cmdbuffer, space);

    float voltage[9], current[9], sensor[17], other[3];
    memset(voltage, 0, sizeof(voltage));
    memset(current, 0, sizeof(current));
    memset(sensor, 0, sizeof(sensor));
    memset(other, 0, sizeof(other));

    for (count1 = 0; count1 < 8; count1++) {
      if (token != NULL) {
        voltage[count1] = (float) atof(token);
        #ifdef DEBUG_LOGGING
        //		printf("voltage: %f ", voltage[count1]);
        #endif
        token = strtok(NULL, space);
        if (token != NULL) {
          current[count1] = (float) atof(token);
          if ((current[count1] < 0) && (current[count1] > -0.5))
            current[count1] *= (-1.0f);
          #ifdef DEBUG_LOGGING
          //		 printf("current: %f\n", current[count1]);
          #endif
          token = strtok(NULL, space);
        }
      }
    }

    //	 printf("\n"); 	  

    batteryVoltage = voltage[map[BAT]];
    if (batteryVoltage < 3.5) {
      NormalModeFailure = 1;
      printf("Safe Mode!\n");
    } else
      NormalModeFailure = 0;

    FILE * cpuTempSensor = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (cpuTempSensor) {
      double cpuTemp;
      fscanf(cpuTempSensor, "%lf", & cpuTemp);
      cpuTemp /= 1000;

      #ifdef DEBUG_LOGGING
      printf("CPU Temp Read: %6.1f\n", cpuTemp);
      #endif

      other[IHU_TEMP] = (double)cpuTemp;

      //    IHUcpuTemp = (int)((cpuTemp * 10.0) + 0.5);
    }
    fclose(cpuTempSensor);

    char sensor_payload[500];

    if (payload == ON) {
      STEMBoardFailure = 0;

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
      //     int retry = FALSE;
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
      sensor_payload[i++] = ' ';
      //    sensor_payload[i++] = '\n';
      sensor_payload[i] = '\0';
      printf("Payload string: %s \n", sensor_payload);

      if ((sensor_payload[0] == 'O') && (sensor_payload[1] == 'K')) // only process if valid payload response
      {
        int count1;
        char * token;
        //   char cmdbuffer[1000];

        //	FILE *file = popen("python3 /home/pi/CubeSatSim/python/voltcurrent.py 1 11", "r");	
        //    	fgets(cmdbuffer, 1000, file);
        //	printf("result: %s\n", cmdbuffer);
        //    	pclose(file);

        const char space[2] = " ";
        token = strtok(sensor_payload, space);
        for (count1 = 0; count1 < 17; count1++) {
          if (token != NULL) {
            sensor[count1] = (float) atof(token);
            #ifdef DEBUG_LOGGING
            printf("sensor: %f ", sensor[count1]);
            #endif
            token = strtok(NULL, space);
          }
        }
        printf("\n");

      }

    }

    if (sim_mode) {
      // simulated telemetry 

      double time = ((long int)millis() - time_start) / 1000.0;

      if ((time - eclipse_time) > period) {
        eclipse = (eclipse == 1) ? 0 : 1;
        eclipse_time = time;
        printf("\n\nSwitching eclipse mode! \n\n");
      }

      /*
        double Xi = eclipse * amps_max[0] * sin(2.0 * 3.14 * time / (46.0 * speed)) * fabs(sin(2.0 * 3.14 * time / (46.0 * speed))) + rnd_float(-2, 2);	  
        double Yi = eclipse * amps_max[1] * sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14/2.0)) * fabs(sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14/2.0))) + rnd_float(-2, 2);	  
        double Zi = eclipse * amps_max[2] * sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2])  * fabs(sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2])) + rnd_float(-2, 2);
      */
      double Xi = eclipse * amps_max[0] * (float) sin(2.0 * 3.14 * time / (46.0 * speed)) + rnd_float(-2, 2);
      double Yi = eclipse * amps_max[1] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14 / 2.0)) + rnd_float(-2, 2);
      double Zi = eclipse * amps_max[2] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2]) + rnd_float(-2, 2);

      double Xv = eclipse * volts_max[0] * (float) sin(2.0 * 3.14 * time / (46.0 * speed)) + rnd_float(-0.2, 0.2);
      double Yv = eclipse * volts_max[1] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + (3.14 / 2.0)) + rnd_float(-0.2, 0.2);
      double Zv = 2.0 * eclipse * volts_max[2] * (float) sin((2.0 * 3.14 * time / (46.0 * speed)) + 3.14 + angle[2]) + rnd_float(-0.2, 0.2);

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

      tempS += (eclipse > 0) ? ((temp_max - tempS) / 50.0f) : ((temp_min - tempS) / 50.0f);
      tempS += +rnd_float(-1.0, 1.0);
      //  IHUcpuTemp = (int)((tempS + rnd_float(-1.0, 1.0)) * 10 + 0.5);
      other[IHU_TEMP] = tempS;

      voltage[map[BUS]] = rnd_float(5.0, 5.005);
      current[map[BUS]] = rnd_float(158, 171);

      //  float charging = current[map[PLUS_X]] + current[map[MINUS_X]] + current[map[PLUS_Y]] + current[map[MINUS_Y]] + current[map[PLUS_Z]] + current[map[MINUS_Z]];
      float charging = eclipse * (fabs(amps_max[0] * 0.707) + fabs(amps_max[1] * 0.707) + rnd_float(-4.0, 4.0));

      current[map[BAT]] = ((current[map[BUS]] * voltage[map[BUS]]) / batt) - charging;

      //  printf("charging: %f bat curr: %f bus curr: %f bat volt: %f bus volt: %f \n",charging, current[map[BAT]], current[map[BUS]], batt, voltage[map[BUS]]);

      batt -= (batt > 3.5) ? current[map[BAT]] / 30000 : current[map[BAT]] / 3000;
      if (batt < 3.0) {
        batt = 3.0;
        NormalModeFailure = 1;
        printf("Safe Mode!\n");
      } else
        NormalModeFailure = 0;

      if (batt > 4.5)
        batt = 4.5;

      voltage[map[BAT]] = batt + rnd_float(-0.01, 0.01);

      // end of simulated telemetry
    }

    for (count1 = 0; count1 < 8; count1++) {
      if (voltage[count1] < voltage_min[count1])
        voltage_min[count1] = voltage[count1];
      if (current[count1] < current_min[count1])
        current_min[count1] = current[count1];

      if (voltage[count1] > voltage_max[count1])
        voltage_max[count1] = voltage[count1];
      if (current[count1] > current_max[count1])
        current_max[count1] = current[count1];

      printf("Vmin %f Vmax %f Imin %f Imax %f \n", voltage_min[count1], voltage_max[count1], current_min[count1], current_max[count1]);
    }

    if ((sensor_payload[0] == 'O') && (sensor_payload[1] == 'K')) {
      for (count1 = 0; count1 < 17; count1++) {
        if (sensor[count1] < sensor_min[count1])
          sensor_min[count1] = sensor[count1];
        if (sensor[count1] > sensor_max[count1])
          sensor_max[count1] = sensor[count1];

        printf("Smin %f Smax %f \n", sensor_min[count1], sensor_max[count1]);
      }
    }

    for (count1 = 0; count1 < 3; count1++) {
      if (other[count1] < other_min[count1])
        other_min[count1] = other[count1];
      if (other[count1] > other_max[count1])
        other_max[count1] = other[count1];

      printf("Other min %f max %f \n", other_min[count1], other_max[count1]);
    }

   if (mode == FSK) {	  
    if (loop % 8 == 0) {
      printf("Sending MIN frame \n");
      frm_type = 0x03;
      for (count1 = 0; count1 < 17; count1++) {
        if (count1 < 3)
          other[count1] = other_min[count1];
        if (count1 < 8) {
          voltage[count1] = voltage_min[count1];
          current[count1] = current_min[count1];
        }
        if (sensor_min[count1] != 1000.0) // make sure values are valid
          sensor[count1] = sensor_min[count1];
      }
    }
    if ((loop + 4) % 8 == 0) {
      printf("Sending MAX frame \n");
      frm_type = 0x02;
      for (count1 = 0; count1 < 17; count1++) {
        if (count1 < 3)
          other[count1] = other_max[count1];
        if (count1 < 8) {
          voltage[count1] = voltage_max[count1];
          current[count1] = current_max[count1];
        }
        if (sensor_max[count1] != -1000.0) // make sure values are valid
          sensor[count1] = sensor_max[count1];
      }
    }
   }
    memset(rs_frame, 0, sizeof(rs_frame));
    memset(parities, 0, sizeof(parities));

    FILE * uptime_file = fopen("/proc/uptime", "r");
    fscanf(uptime_file, "%f", & uptime_sec);
    uptime = (int) uptime_sec;
    fclose(uptime_file);
    printf("Reset Count: %d Uptime since Reset: %ld \n", reset_count, uptime);

    h[0] = (short int) ((h[0] & 0xf8) | (id & 0x07)); // 3 bits
    //    printf("h[0] %x\n", h[0]);
    h[0] = (short int) ((h[0] & 0x07) | ((reset_count & 0x1f) << 3));
    //    printf("h[0] %x\n", h[0]);
    h[1] = (short int) ((reset_count >> 5) & 0xff);
    //    printf("h[1] %x\n", h[1]);
    h[2] = (short int) ((h[2] & 0xf8) | ((reset_count >> 13) & 0x07));
    //    printf("h[2] %x\n", h[2]);
    h[2] = (short int) ((h[2] & 0x0e) | ((uptime & 0x1f) << 3));
    //    printf("h[2] %x\n", h[2]);
    h[3] = (short int) ((uptime >> 5) & 0xff);
    h[4] = (short int) ((uptime >> 13) & 0xff);
    h[5] = (short int) ((h[5] & 0xf0) | ((uptime >> 21) & 0x0f));
    h[5] = (short int) ((h[5] & 0x0f) | (frm_type << 4));

    if (mode == BPSK)
      h[6] = 99;

    posXi = (int)(current[map[PLUS_X]] + 0.5) + 2048;
    posYi = (int)(current[map[PLUS_Y]] + 0.5) + 2048;
    posZi = (int)(current[map[PLUS_Z]] + 0.5) + 2048;
    negXi = (int)(current[map[MINUS_X]] + 0.5) + 2048;
    negYi = (int)(current[map[MINUS_Y]] + 0.5) + 2048;
    negZi = (int)(current[map[MINUS_Z]] + 0.5) + 2048;

    posXv = (int)(voltage[map[PLUS_X]] * 100);
    posYv = (int)(voltage[map[PLUS_Y]] * 100);
    posZv = (int)(voltage[map[PLUS_Z]] * 100);
    negXv = (int)(voltage[map[MINUS_X]] * 100);
    negYv = (int)(voltage[map[MINUS_Y]] * 100);
    negZv = (int)(voltage[map[MINUS_Z]] * 100);

    batt_c_v = (int)(voltage[map[BAT]] * 100);

    battCurr = (int)(current[map[BAT]] + 0.5) + 2048;
    PSUVoltage = (int)(voltage[map[BUS]] * 100);
    PSUCurrent = (int)(current[map[BUS]] + 0.5) + 2048;

    if (payload == ON)
      STEMBoardFailure = 0;

    //  if (payload == ON)
    //	  STEMBoardFailure = 0;

    // read payload sensor if available

    encodeA(b, 0 + head_offset, batt_a_v);
    encodeB(b, 1 + head_offset, batt_b_v);
    encodeA(b, 3 + head_offset, batt_c_v);

    //  encodeB(b, 4 + head_offset, (int)(xAccel * 100 + 0.5) + 2048);	  // Xaccel
    //  encodeA(b, 6 + head_offset, (int)(yAccel * 100 + 0.5) + 2048);	  // Yaccel
    //  encodeB(b, 7 + head_offset, (int)(zAccel * 100 + 0.5) + 2048);	  // Zaccel

    encodeB(b, 4 + head_offset, (int)(sensor[ACCEL_X] * 100 + 0.5) + 2048); // Xaccel
    encodeA(b, 6 + head_offset, (int)(sensor[ACCEL_Y] * 100 + 0.5) + 2048); // Yaccel
    encodeB(b, 7 + head_offset, (int)(sensor[ACCEL_Z] * 100 + 0.5) + 2048); // Zaccel

    encodeA(b, 9 + head_offset, battCurr);

    //  encodeB(b, 10 + head_offset,(int)(BME280temperature * 10 + 0.5));	// Temp
    encodeB(b, 10 + head_offset, (int)(sensor[TEMP] * 10 + 0.5)); // Temp	  

    if (mode == FSK) {
      encodeA(b, 12 + head_offset, posXv);
      encodeB(b, 13 + head_offset, negXv);
      encodeA(b, 15 + head_offset, posYv);
      encodeB(b, 16 + head_offset, negYv);
      encodeA(b, 18 + head_offset, posZv);
      encodeB(b, 19 + head_offset, negZv);

      encodeA(b, 21 + head_offset, posXi);
      encodeB(b, 22 + head_offset, negXi);
      encodeA(b, 24 + head_offset, posYi);
      encodeB(b, 25 + head_offset, negYi);
      encodeA(b, 27 + head_offset, posZi);
      encodeB(b, 28 + head_offset, negZi);
    } else // BPSK
    {
      encodeA(b, 12 + head_offset, posXv);
      encodeB(b, 13 + head_offset, posYv);
      encodeA(b, 15 + head_offset, posZv);
      encodeB(b, 16 + head_offset, negXv);
      encodeA(b, 18 + head_offset, negYv);
      encodeB(b, 19 + head_offset, negZv);

      encodeA(b, 21 + head_offset, posXi);
      encodeB(b, 22 + head_offset, posYi);
      encodeA(b, 24 + head_offset, posZi);
      encodeB(b, 25 + head_offset, negXi);
      encodeA(b, 27 + head_offset, negYi);
      encodeB(b, 28 + head_offset, negZi);
    }

    encodeA(b, 30 + head_offset, PSUVoltage);
    //  encodeB(b, 31 + head_offset,(spin * 10) + 2048);	  
    encodeB(b, 31 + head_offset, ((int)(other[SPIN] * 10)) + 2048);

    //  encodeA(b, 33 + head_offset,(int)(BME280pressure + 0.5));  // Pressure
    //  encodeB(b, 34 + head_offset,(int)(BME280altitude + 0.5));   // Altitude

    encodeA(b, 33 + head_offset, (int)(sensor[PRES] + 0.5)); // Pressure
    encodeB(b, 34 + head_offset, (int)(sensor[ALT] * 10.0 + 0.5)); // Altitude

    encodeA(b, 36 + head_offset, Resets);
    //  encodeB(b, 37 + head_offset,  Rssi);	
    encodeB(b, 37 + head_offset, (int)(other[RSSI] + 0.5) + 2048);

    //  encodeA(b, 39 + head_offset,  IHUcpuTemp);
    encodeA(b, 39 + head_offset, (int)(other[IHU_TEMP] * 10 + 0.5));

    //  encodeB(b, 40 + head_offset,  xAngularVelocity);
    //  encodeA(b, 42 + head_offset,  yAngularVelocity);
    //  encodeB(b, 43 + head_offset,  zAngularVelocity);

    encodeB(b, 40 + head_offset, (int)(sensor[GYRO_X] + 0.5) + 2048);
    encodeA(b, 42 + head_offset, (int)(sensor[GYRO_Y] + 0.5) + 2048);
    encodeB(b, 43 + head_offset, (int)(sensor[GYRO_Z] + 0.5) + 2048);

    //  encodeA(b, 45 + head_offset, (int)(BME280humidity + 0.5));  // in place of sensor1
    encodeA(b, 45 + head_offset, (int)(sensor[HUMI] + 0.5)); // in place of sensor1

    encodeB(b, 46 + head_offset, PSUCurrent);
    //  encodeA(b, 48 + head_offset, (int)(XSsensor2) + 2048);
    //  encodeB(b, 49 + head_offset, (int)(XSsensor3 * 100 + 0.5) + 2048);

    encodeA(b, 48 + head_offset, (int)(sensor[XS2]) + 2048);
    encodeB(b, 49 + head_offset, (int)(sensor[XS3] * 100 + 0.5) + 2048);

    // camera = ON;

    int status = STEMBoardFailure + NormalModeFailure * 2 + PayloadFailure1 * 4 + PayloadFailure2 * 8 +
      (i2c_bus0 == OFF) * 16 + (i2c_bus1 == OFF) * 32 + (i2c_bus3 == OFF) * 64 + (camera == OFF) * 128 + groundCommandCount * 256;

    encodeA(b, 51 + head_offset, status);
    //  encodeA(b, 51 + head_offset, STEMBoardFailure + NormalModeFailure * 2 + (i2c_bus0 == OFF) * 16 + (i2c_bus1 == OFF) * 32 + (i2c_bus3 == OFF) * 64  + (0) * 128 + 1 * 256 + 1 * 512 + 1 * 1024 + 1*2048); 
    encodeB(b, 52 + head_offset, rxAntennaDeployed + txAntennaDeployed * 2);

    if (txAntennaDeployed == 0) {
      txAntennaDeployed = 1;
      printf("TX Antenna Deployed!\n");
    }
    
    if (mode == BPSK) {  // WOD field experiments
      encodeA(b, 63 + head_offset, 0xff);  
      encodeB(b, 74 + head_offset, 0xff);	
    }
    short int data10[headerLen + rsFrames * (rsFrameLen + parityLen)];
    short int data8[headerLen + rsFrames * (rsFrameLen + parityLen)];

    int ctr1 = 0;
    int ctr3 = 0;
    for (i = 0; i < rsFrameLen; i++) {
      for (int j = 0; j < rsFrames; j++) {
        if (!((i == (rsFrameLen - 1)) && (j == 2))) // skip last one for BPSK
        {
          if (ctr1 < headerLen) {
            rs_frame[j][i] = h[ctr1];
            update_rs(parities[j], h[ctr1]);
            //      				printf("header %d rs_frame[%d][%d] = %x \n", ctr1, j, i, h[ctr1]);
            data8[ctr1++] = rs_frame[j][i];
            //				printf ("data8[%d] = %x \n", ctr1 - 1, rs_frame[j][i]);
          } else {
            rs_frame[j][i] = b[ctr3 % dataLen];
            update_rs(parities[j], b[ctr3 % dataLen]);
            //  				printf("%d rs_frame[%d][%d] = %x %d \n", 
            //  					ctr1, j, i, b[ctr3 % DATA_LEN], ctr3 % DATA_LEN);
            data8[ctr1++] = rs_frame[j][i];
            //			printf ("data8[%d] = %x \n", ctr1 - 1, rs_frame[j][i]);
            ctr3++;
          }
        }
      }
    }

    #ifdef DEBUG_LOGGING
    //	printf("\nAt end of data8 write, %d ctr1 values written\n\n", ctr1);
    /*
    	  printf("Parities ");
    		for (int m = 0; m < parityLen; m++) {
    		 	printf("%d ", parities[0][m]);
    		}
    		printf("\n");
    */
    #endif

    int ctr2 = 0;
    memset(data10, 0, sizeof(data10));

    for (i = 0; i < dataLen * payloads + headerLen; i++) // 476 for BPSK
    {
      data10[ctr2] = (Encode_8b10b[rd][((int) data8[ctr2])] & 0x3ff);
      nrd = (Encode_8b10b[rd][((int) data8[ctr2])] >> 10) & 1;
      //		printf ("data10[%d] = encoded data8[%d] = %x \n",
      //		 	ctr2, ctr2, data10[ctr2]); 

      rd = nrd; // ^ nrd;
      ctr2++;
    }
    for (i = 0; i < parityLen; i++) {
      for (int j = 0; j < rsFrames; j++) {
        data10[ctr2++] = (Encode_8b10b[rd][((int) parities[j][i])] & 0x3ff);
        nrd = (Encode_8b10b[rd][((int) parities[j][i])] >> 10) & 1;
        //	printf ("data10[%d] = encoded parities[%d][%d] = %x \n",
        //		 ctr2 - 1, j, i, data10[ctr2 - 1]); 

        rd = nrd;
      }
    }
    #ifdef DEBUG_LOGGING
    // 	printf("\nAt end of data10 write, %d ctr2 values written\n\n", ctr2);
    #endif

    int data;
    int val;
    //int offset = 0;

    #ifdef DEBUG_LOGGING
    //	printf("\nAt start of buffer loop, syncBits %d samples %d ctr %d\n", syncBits, samples, ctr);
    #endif

    for (i = 1; i <= syncBits * samples; i++) {
      write_wave(ctr, buffer);
      //		printf("%d ",ctr);
      if ((i % samples) == 0) {
        int bit = syncBits - i / samples + 1;
        val = sync;
        data = val & 1 << (bit - 1);
        //   	printf ("%d i: %d new frame %d sync bit %d = %d \n",
        //  		 ctr/SAMPLES, i, frames, bit, (data > 0) );
        if (mode == FSK) {
          phase = ((data != 0) * 2) - 1;
          //		printf("Sending a %d\n", phase);
        } else {
          if (data == 0) {
            phase *= -1;
            if ((ctr - smaller) > 0) {
              for (int j = 1; j <= smaller; j++)
                buffer[ctr - j] = buffer[ctr - j] * 0.4;
            }
            flip_ctr = ctr;
          }
        }
      }
    }
    #ifdef DEBUG_LOGGING
    //	printf("\n\nValue of ctr after header: %d Buffer Len: %d\n\n", ctr, buffSize);
    #endif
    for (i = 1; i <= (10 * (headerLen + dataLen * payloads + rsFrames * parityLen) * samples); i++) // 572   
    {
      write_wave(ctr, buffer);
      if ((i % samples) == 0) {
        int symbol = (int)((i - 1) / (samples * 10));
        int bit = 10 - (i - symbol * samples * 10) / samples + 1;
        val = data10[symbol];
        data = val & 1 << (bit - 1);
        //		printf ("%d i: %d new frame %d data10[%d] = %x bit %d = %d \n",
        //	    		 ctr/SAMPLES, i, frames, symbol, val, bit, (data > 0) );
        if (mode == FSK) {
          phase = ((data != 0) * 2) - 1;
          //			printf("Sending a %d\n", phase);
        } else {
          if (data == 0) {
            phase *= -1;
            if ((ctr - smaller) > 0) {
              for (int j = 1; j <= smaller; j++)
                buffer[ctr - j] = buffer[ctr - j] * 0.4;
            }
            flip_ctr = ctr;
          }
        }
      }
    }
  }
  #ifdef DEBUG_LOGGING
  //	printf("\nValue of ctr after looping: %d Buffer Len: %d\n", ctr, buffSize);
  //	printf("\ctr/samples = %d ctr/(samples*10) = %d\n\n", ctr/samples, ctr/(samples*10));
  #endif

  int error = 0;
  // int count;
  //  for (count = 0; count < dataLen; count++) {
  //      printf("%02X", b[count]);
  //  }
  //  printf("\n");

  // socket write

  if (!socket_open && transmit) {
    printf("Opening socket!\n");
 //   struct sockaddr_in address;
 //   int valread;
    struct sockaddr_in serv_addr;
    //    char *hello = "Hello from client"; 
    //    char buffer[1024] = {0}; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      printf("\n Socket creation error \n");
      error = 1;
    }

    memset( & serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form 
    if (inet_pton(AF_INET, "127.0.0.1", & serv_addr.sin_addr) <= 0) {
      printf("\nInvalid address/ Address not supported \n");
      error = 1;
    }

    if (connect(sock, (struct sockaddr * ) & serv_addr, sizeof(serv_addr)) < 0) {
      printf("\nConnection Failed \n");
      printf("Error: %s \n", strerror(errno));
      error = 1;
    }
    if (error == 1)
    ; //rpitxStatus = -1;
    else
      socket_open = 1;
  }

  if (!error && transmit) {
    //	digitalWrite (0, LOW);
    printf("Sending %d buffer bytes over socket after %d ms!\n", ctr, (long unsigned int)millis() - start);
    start = millis();
    int sock_ret = send(sock, buffer, (unsigned int)(ctr * 2 + 2), 0);
    printf("Millis5: %d Result of socket send: %d \n", (unsigned int)millis() - start, sock_ret);

    if (sock_ret < (ctr * 2 + 2)) {
      printf("Not resending\n");
      //	 	sock_ret = send(sock, buffer[sock_ret], ctr * 2 + 2 - sock_ret, 0);
      //       		printf("Millis10: %d Result of socket send: %d \n", millis() - start, sock_ret);
    }

    if (sock_ret == -1) {
      printf("Error: %s \n", strerror(errno));
      socket_open = 0;
      //rpitxStatus = -1;
    }
  }
  if (!transmit) {
    fprintf(stderr, "\nNo CubeSatSim Band Pass Filter detected.  No transmissions after the CW ID.\n");
    fprintf(stderr, " See http://cubesatsim.org/wiki for info about building a CubeSatSim\n\n");
  }
  //    digitalWrite (0, HIGH);

  if (mode == FSK)
    firstTime = 0;
  else if (frames_sent > 0) //5)
    firstTime = 0;

  return;
}


