#ifndef UTILS_H
#define UTILS_H

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>


float rnd_float(double min,double max);
int test_i2c_bus(int bus);
void gen_tlm(float *current, float *voltage, int *map);

#endif
