#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "MadgwickAHRS.h"
void micros(int microseconds)
{
  unsigned int timer_ms = (get_cpu_usecs());
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
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

  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
}
int low[5]={0,1068,1100,1108,1068}, center[5]={0,1488,1504,1504,1468}, high[5]={0,1892,1908,1904,1864};
int start;
unsigned long loop_timer;
int temperature=0;
double acc_x=0, acc_y=0, acc_z=0, acc_total_vector=0;
short int acc_axis[4]={0,0,0,0}, gyro_axis[4]={0,0,0,0};
double gyro_pitch=0.0, gyro_roll=0.0, gyro_yaw=0.0;
int cal_int=0, loop_counter=0;
double gyro_axis_cal[4]={0.0,0.0,0.0,0.0};
int gyro_address = MPU6050_I2C_ADDRESS;
bool first_angle=false;
float angle_roll_acc=0.0, angle_pitch_acc=0.0, angle_pitch=0.0, angle_roll=0.0;
int receiver_input[5];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

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

void intr_handler(void) {
  // exc_prologue();

  // read the receiver pwm duty cycle
  receiver_input[1] = receiver_read(0);
  receiver_input[2] = receiver_read(1);
  receiver_input[3] = receiver_read(2);
  receiver_input[4] = receiver_read(3);

  // exc_epilogue();
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
int convert_receiver_channel(unsigned int function)
{
  unsigned int  channel, reverse;                                                       //First we declare some local variables
  int actual; ///(0,th,roll,pitch,yaw)
  int difference;

  if(function==1)
  {
    reverse = 0;                      //Reverse channel when most significant bit is set
    channel =2;
  }
  else if(function==2)
  {
    reverse = 0;
    channel=3;
  }
  else if(function==3)
  {
    reverse = 0;
    channel=1;
  }
  else
  {
    reverse = 0;                                                            //If the most significant is not set there is no reverse
    channel =4;
  }

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  // low = 1000;  //Store the low value for the specific receiver input channel
  // center = 1500; //Store the center value for the specific receiver input channel
  // high = 2000;   //Store the high value for the specific receiver input channel

  if(actual < center[channel]){                                                         //The actual receiver value is lower than the center value
    if(actual < low[channel])actual = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center[channel]){                                                                        //The actual receiver value is higher than the center value
    if(actual > high[channel])actual = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void gyro_signalen(){

  receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  //Read the MPU-6050
  ACCEL_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
  ACCEL_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_L);
  ACCEL_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
  ACCEL_Y_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_L);
  ACCEL_Z_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
  ACCEL_Z_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_L);
  TEMP_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
  TEMP_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_L);
  GYRO_X_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
  GYRO_X_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_L);
  GYRO_Y_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
  GYRO_Y_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_L);
  GYRO_Z_H = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);
  GYRO_Z_L = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_L);
  
    acc_axis[1] = (ACCEL_X_H<<8|ACCEL_X_L);                    //Add the low and high byte to the acc_x variable.
    acc_axis[2] = (ACCEL_Y_H<<8|ACCEL_Y_L);                  //Add the low and high byte to the acc_y variable.
    acc_axis[3] = (ACCEL_Z_H<<8|ACCEL_Z_L);                    //Add the low and high byte to the acc_z variable.
    temperature = (TEMP_H<<8|TEMP_L);                    //Add the low and high byte to the temperature variable.
    gyro_axis[1] = (GYRO_X_H<<8|GYRO_X_L);                   //Read high and low part of the angular data.
    gyro_axis[2] = (GYRO_Y_H<<8|GYRO_Y_L);                   //Read high and low part of the angular data.
    gyro_axis[3] = (GYRO_Z_H<<8|GYRO_Z_L);                   //Read high and low part of the angular data.

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[1];           //Set gyro_roll to the correct axis that was stored in the EEPROM.
  gyro_pitch = gyro_axis[2];          //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  gyro_pitch *= -1;              //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[3];            //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  gyro_yaw *= -1;                //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[2];                //Set acc_x to the correct axis that was stored in the EEPROM.
  acc_x *= -1;                   //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[1];                //Set acc_y to the correct axis that was stored in the EEPROM.
  acc_z = acc_axis[3];                //Set acc_z to the correct axis that was stored in the EEPROM.
  acc_z *= -1;                   //Invert acc_z if the MSB of EEPROM bit 30 is set.


   //   printf("-----------------------\n");
   // printf("ACCEL_X = 0x%.2X%.2X (%d)\n", ACCEL_X_H, ACCEL_X_L, (short int)((ACCEL_X_H << 8) | ACCEL_X_L));
   // printf("ACCEL_Y = 0x%.2X%.2X (%d)\n", ACCEL_Y_H, ACCEL_Y_L, (short int)((ACCEL_Y_H << 8) | ACCEL_Y_L));
   // printf("ACCEL_Z = 0x%.2X%.2X (%d)\n", ACCEL_Z_H, ACCEL_Z_L, (short int)((ACCEL_Z_H << 8) | ACCEL_Z_L));
   // printf("TEMP    = 0x%.2X%.2X (%.1f C)\n", TEMP_H, TEMP_L, ((double)((short int)((TEMP_H << 8) | TEMP_L)) + 12412.0) / 340.0 ); //using datasheet formula for T in degrees Celsius
   // printf("GYRO_X  = 0x%.2X%.2X (%d)\n", GYRO_X_H, GYRO_X_L, (short int)((GYRO_X_H << 8) | GYRO_X_L));
   // printf("GYRO_Y  = 0x%.2X%.2X (%d)\n", GYRO_Y_H, GYRO_Y_L, (short int)((GYRO_Y_H << 8) | GYRO_Y_L));
   // printf("GYRO_Z  = 0x%.2X%.2X (%d)\n", GYRO_Z_H, GYRO_Z_L, (short int)((GYRO_Z_H << 8) | GYRO_Z_L));
}

void set_gyro_registers()
{
  //Setup the MPU-6050
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00));                    //Set the register bits as 00000000 to activate the gyro
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08));                   //Set the register bits as 00001000 (500dps full scale)
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10));                  //Set the register bits as 00010000 (+/- 8g full scale range)
  while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)

  
}



int main(int argc, char **argv)
{
  printf("Hello MCU6050!\n");
  loop_counter = 0;                                                                   //Reset the loop_counter variable.
  cal_int = 0;
  start =0;
  first_angle = false;
  blink_once();

  // signature = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I);
  // printf("Signature = 0x%.2X\n", signature);

  // printf("PWR_MGMT_1 = 0x%.2X\n", i2c_read(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1));
  // printf("Getting MPU-6050 out of sleep mode.\n");

  actuator_write(m1, 1000);                                               //give motors 1000us pulse.
  actuator_write(m2, 1000);
  actuator_write(m3, 1000);
  actuator_write(m4, 1000);

  set_gyro_registers();

  for (cal_int = 0; cal_int < 2000 ; cal_int ++)
  {
    if(cal_int % 15 == 0)LED_out(1);                //Change the led status to indicate calibration.
    gyro_signalen();                                                                //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
    // micros(3000);                                                                 //Wait 3 milliseconds before the next loop.
    LED_out(0);
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                         //Divide the yaw total by 2000.
  printf("gyro callibration done\n");

  intr_handler();

  start = 0;
  LED_out(0); 


  loop_timer = get_cpu_usecs();
  float dt=0.01;
  //for (int i = 0; i < 5; i++) {
  // for (int j=0;j<3000;j++)
  while(1)
  {
    while(get_cpu_usecs() - loop_timer < 10000);                                                 //Start the pulse after 4000 micro seconds.
    // dt = get_cpu_usecs() - loop_timer;
    // printf("dt:%f\n",dt);
    loop_timer = get_cpu_usecs();                                                                //Reset the zero timer.

    ///////////////////////////////////////////////////////////////////////////////////////////
    //To calculate quadcopter angles.
    ////////////////////////////////////////////////////////////////////////////////////////////
    intr_handler();
    gyro_signalen();



    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950 && receiver_input_channel_1 < 1050 && receiver_input_channel_2 > 1950)
    {
      start = -1;
      // printf("motors stop\n");
    }




////new cmplementry filter
    angle_pitch += (gyro_pitch / 65.5)*dt;                                           //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += (gyro_roll / 65.5)*dt;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

    angle_pitch -= angle_roll * sin(gyro_yaw * (dt/65.5)*(3.142/180));                         //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_yaw * (dt/65.5)*(3.142/180));                         //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));           //Calculate the total accelerometer vector.


    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
    }
    if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
      angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
    }
    angle_pitch_acc -= 11.5;                                                   //Accelerometer calibration value for pitch.
    angle_roll_acc += 12.5;                                                    //Accelerometer calibration value for roll.


    if(!first_angle)
    {
      angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
      angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
      first_angle = true;
    }
    else
    {
      angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
    }



////////////complementry filter arduino version
    // angle_pitch += gyro_pitch * 0.0000611;                                           //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    // angle_roll += gyro_roll * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

    // angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the roll angle to the pitch angel.
    // angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the pitch angle to the roll angel.

    // //Accelerometer angle calculations
    // acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));           //Calculate the total accelerometer vector.


    // //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    // if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    //   angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
    // }
    // if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    //   angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
    // }
    // angle_pitch_acc -= 11.5;                                                   //Accelerometer calibration value for pitch.
    // angle_roll_acc += 12.5;                                                    //Accelerometer calibration value for roll.


    // if(!first_angle)
    // {
    //   angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
    //   angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
    //   first_angle = true;
    // }
    // else
    // {
    //   angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    //   angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
    // }

  ////madwick filter
    // MadgwickAHRSupdateIMU(gyro_roll, gyro_pitch,gyro_yaw,acc_x,acc_y,acc_z);

    // printf("q0: %f       q1: %f      q2: %f    q3: %f \n", q0, q1, q2,q3);

    // // roll (x-axis rotation)
    // double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    // double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    // angle_roll = atan2(sinr_cosp, cosr_cosp);

    // // pitch (y-axis rotation)
    // double sinp = 2 * (q0 * q2 - q3 * q1);
    // if (abs(sinp) >= 1)
    //     angle_pitch = (sinp/abs(sinp))*(M_PI / 2); // use 90 degrees if out of range
    // else
    //     angle_pitch = asin(sinp);

    // // yaw (z-axis rotation)
    // double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    // double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    // gyro_yaw = atan2(siny_cosp, cosy_cosp);

    // if(loop_counter == 0)printf("Pitch: %f       Roll: %f      Yaw: %f  \n", angle_pitch*57.296, angle_roll*57.296, gyro_yaw*57.296);






    if(loop_counter == 0)printf("Pitch: ");
    if(loop_counter == 1)printf("%f",angle_pitch);
    if(loop_counter == 2)printf(" Roll: ");
    if(loop_counter == 3)printf("%f",angle_roll);
    if(loop_counter == 4)printf(" Yaw: ");
    if(loop_counter == 5)printf("%f",gyro_yaw / 65.5);
    if(loop_counter == 5)printf("\n");
    loop_counter++;
    if(loop_counter==30)loop_counter=0;

    if(start==-1)break;

    // printf("diff loop_timer:  %ld\n", loop_timer);

  }
  return 0;
}
  
