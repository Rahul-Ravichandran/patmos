//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_ACTUATOR_RECEIVER_H
#define PATMOS_ACTUATOR_RECEIVER_H
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

//Receiver controller register
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )


//writes pwm signlas of width=data to the esc
void actuator_write(unsigned int actuator_id, unsigned int data)
{
    *(MOTOR + actuator_id) = data;
}

//get pulse width data from receiver
int receiver_read(unsigned int receiver_id){

    unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
    unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

    return pulse_high_time;
}





#endif //PATMOS_ACTUATOR_RECEIVER_H
