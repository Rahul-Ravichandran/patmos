#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>

void micros(int microseconds)
{
  unsigned int timer_ms = (get_cpu_usecs());
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
}
void millis(int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = (get_cpu_usecs()/1000);
}
//LEDs
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )

//I2C controller
#define I2C ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_I2C ) )

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68

#define BARO_REG                   0xA0   // R


//MCU6050 registers
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_TEMP_OUT_H         0x41   // R
#define MPU6050_TEMP_OUT_L         0x42   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_GYRO_CONFIG        0x1B   // R
#define MPU6050_ACCEL_CONFIG       0x1C   // R
#define MPU6050_CONFIG_REG         0x1A   // R

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3
//Receiver controller
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

const unsigned int CPU_PERIOD = 20; //CPU period in ns.
#define compass_address 0x1E

unsigned char gps_data=0;
float compass_x, compass_y, compass_z;
int loop_counter;

void actuator_write(unsigned int actuator_id, unsigned int data)
{
  *(MOTOR + actuator_id) = data;
}

//Reads from propulsion specified by propulsion ID (0 to 4)
int receiver_read(unsigned int receiver_id){
  return *(RECEIVER + receiver_id);
  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
}

unsigned long loop_timer;
int temperature=0;
double acc_x=0, acc_y=0, acc_z=0, acc_total_vector=0;
short int acc_axis[4]={0,0,0,0}, gyro_axis[4]={0,0,0,0};
double gyro_pitch=0.0, gyro_roll=0.0, gyro_yaw=0.0;
int cal_int=0, loop_counter=0;
double gyro_axis_cal[4]={0.0,0.0,0.0,0.0};
unsigned int MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
bool first_angle=false;
float angle_roll_acc=0.0, angle_pitch_acc=0.0, angle_pitch=0.0, angle_roll=0.0;

//Barometer variables.
unsigned int C[7];
unsigned int barometer_counter, temperature_counter;
int OFF, OFF_C2, SENS, SENS_C1, P;
unsigned int raw_pressure, raw_temperature, temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int dT, dT_C5;
unsigned int start;



unsigned int signature = 0;
unsigned int ACCEL_X_H = 0;
unsigned int ACCEL_X_L = 0;
unsigned int ACCEL_Y_H = 0;
unsigned int ACCEL_Y_L = 0;
unsigned int ACCEL_Z_H = 0;
unsigned int ACCEL_Z_L = 0;
unsigned int TEMP_L = 0;
unsigned int TEMP_H = 0;
unsigned int GYRO_X_H = 0;
unsigned int GYRO_X_L = 0;
unsigned int GYRO_Y_H = 0;
unsigned int GYRO_Y_L = 0;
unsigned int GYRO_Z_H = 0;
unsigned int GYRO_Z_L = 0;

//Writes to i2c, returns -1 if there was an error, 0 if succeded
int i2c_write(unsigned char chipaddress, unsigned char regaddress, unsigned char data){
  I2C = ((((unsigned int) data & 0x000000FF) << 16) | (((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) & 0xFFFFFFFE;
  if ((I2C & 0x00000100) != 0)
  {
    return -1;
  }else{
    return 0;
  }
}

//Reads to i2c, returns the read value (8 bits), if there was an error the returned value is -1 (0xFFFFFFFF)
int i2c_read(unsigned char chipaddress, unsigned char regaddress){
  I2C = ((((unsigned int) regaddress & 0x000000FF) << 8) | (((unsigned int) chipaddress & 0x0000007F) << 1)) | 0x00000001;
  unsigned int I2C_tmp = I2C;
  if ((I2C_tmp & 0x00000100) != 0)
  {
    return -1;
  }else{
    return (int)((unsigned int)(I2C_tmp) & 0x000000FF);
  }
}

//Blinks the LEDs once
void blink_once(){
  int i, j;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0001;
  for (i=2000; i!=0; --i)
    for (j=2000; j!=0; --j)
      LED = 0x0000;
  return;
}

void LED_out(int i){
  if(i==1) LED = 0x0001;
  else LED = 0x0000;
  return;
}
void check_barometer(void) {
  loop_counter = 0;

    //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    //These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++) {
      C[start] = i2c_read(MS5611_address, 0xA0 + start*2) << 8 | i2c_read(MS5611_address, 0xA0 + start*2 +1);                    //Start communication with the MPU-6050.
    }
    //Print the 6 calibration values on the screen.
    printf("C1 = %d\n",C[1]);
    printf("C2 = %d\n",C[2]);
    printf("C3 = %d\n",C[3]);
    printf("C4 = %d\n",C[4]);
    printf("C5 = %d\n",C[5]);
    printf("C6 = %d\n",C[6]);

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

    start = 0;

  for(int i=0;i<1000;i++) {                                           //Stay in this loop until the data variable data holds a q.
    loop_timer = get_cpu_usecs() + 4000;                                 //Set the loop_timer variable to the current micros() value + 4000.

    barometer_counter ++;                                         //Increment the barometer_counter variable for the next step.

    if (barometer_counter == 1) {
      if (temperature_counter == 0) {
        //Get temperature data from MS-5611
        raw_temperature = i2c_read(MS5611_address, 0x58) << 16 | i2c_read(MS5611_address, 0x58 + 1) << 8 | i2c_read(MS5611_address, 0x58 + 2);
      }
      else {
        //Get pressure data from MS-5611
        raw_pressure = i2c_read(MS5611_address, 0x48) << 16 | i2c_read(MS5611_address, 0x48 + 1) << 8 | i2c_read(MS5611_address, 0x48 + 2);
      }

      temperature_counter ++;
      if (temperature_counter > 9) {
        temperature_counter = 0;
        //Request temperature data
        i2c_write(MS5611_address, 0x58, 0x00);
      }
      else {
        //Request pressure data
        i2c_write(MS5611_address, 0x48, 0x00);
      }
    }
    if (barometer_counter == 2) {
      //Calculate pressure as explained in the datasheet of the MS-5611.
      dT = C[5];
      dT <<= 8;
      dT *= -1;
      dT += raw_temperature;

      OFF = OFF_C2 + ((int)dT * (int)C[4]) / pow(2, 7);

      SENS = SENS_C1 + ((int)dT * (int)C[3]) / pow(2, 8);

      P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

      if (actual_pressure == 0) {
        actual_pressure = P;
        actual_pressure_fast = P;
        actual_pressure_slow = P;
      }

      actual_pressure_fast = actual_pressure_fast * (float)0.92 + P * (float)0.08;
      actual_pressure_slow = actual_pressure_slow * (float)0.99 + P * (float)0.01;
      actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
      if (actual_pressure_diff > 8)actual_pressure_diff = 8;
      if (actual_pressure_diff < -8)actual_pressure_diff = -8;
      if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
      actual_pressure = actual_pressure_slow;
      if (start < 200){
        start++;
        actual_pressure = 0;
      }
      else printf("%f \n", actual_pressure);
    }
    if (barometer_counter == 3) {
      barometer_counter = 0;
    }
    while (loop_timer > get_cpu_usecs());
  }
  loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
  start = 0;
}

int main(int argc, char **argv)
{
  printf("Hello Baro!\n");
  

  // loop_timer = get_cpu_usecs();
  
  // for(int i=0;i<100;i++)
  check_barometer();
  return 0;
}
  
