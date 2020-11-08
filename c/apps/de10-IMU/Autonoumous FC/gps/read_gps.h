//
// Created by rahul on 11/8/20.
//


#ifndef PATMOS_READ_GPS_H
#define PATMOS_READ_GPS_H


#include "gps.h"
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include "../basic_lib/timer.h"
#include "../basic_lib/uart.h"
#include <time.h>

int loop_counter;
unsigned char gps_data=0;

struct gps_tpv tpv;
int result;
//---- to filter Data
char str[500];
int str_i[500];
char str_c[500];
char cRMC[6] = "$GNRMC";
char cVTG[6] = "$GNVTG";
char cGGA[6] = "$GNGGA";
char cGGL[6] = "$GNGGL";
char cGSA[6] = "$GNGSA";
// Drone GPS = GNRMC, NEO6M GPS = GPRMC

void gps_setup(void);

void read_gps(void);

#endif //PATMOS_READ_GPS_H
