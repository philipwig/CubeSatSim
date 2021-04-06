#ifndef UTILS_H
#define UTILS_H

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>


double rnd_float(double min, double max);
int test_i2c_bus(int bus);
void gen_sim_telemetry(double *current, double *voltage, int *map);
void read_config_file(char *file_path, int mode, char *callsign, char *latlong_str, int *num_resets);
void payload_init(int *uart_fd, int *payload);
double get_cpu_temp();

#endif
