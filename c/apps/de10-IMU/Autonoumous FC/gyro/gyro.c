//
// Created by rahul on 11/8/20.
//

#include "gyro.h"


void gyro_setup()
{
    //Setup the MPU-6050 registers
    while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00));                    //Set the register bits as 00000000 to activate the gyro
    while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08));                   //Set the register bits as 00001000 (500dps full scale)
    while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10));                  //Set the register bits as 00010000 (+/- 8g full scale range)
    while(i2c_write(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)

}

void callibrate_gyro()
{
    cal_int = 0;                                                                        //Set the cal_int variable to zero.
    if (cal_int != 2000) {
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
            if (cal_int % 15 == 0) LED_out(1);                     //Change the led status every 125 readings to indicate calibration.
            gyro_signalen();                                                                //Read the gyro output.
            gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
            LED_out(0);                                                                       //Small delay to simulate a 250Hz loop during calibration.
        }
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
        gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
        gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
    }
    printf("gyro callibration done\n");

}

void gyro_signalen()
{

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

    acc_axis[1] = (ACCEL_X_H<<8|ACCEL_X_L);                                 //Add the low and high byte to the acc_x variable.
    acc_axis[2] = (ACCEL_Y_H<<8|ACCEL_Y_L);                                 //Add the low and high byte to the acc_y variable.
    acc_axis[3] = (ACCEL_Z_H<<8|ACCEL_Z_L);                                 //Add the low and high byte to the acc_z variable.
    temperature = (TEMP_H<<8|TEMP_L);                                       //Add the low and high byte to the temperature variable.
    gyro_axis[1] = (GYRO_X_H<<8|GYRO_X_L);                                  //Read high and low part of the angular data.
    gyro_axis[2] = (GYRO_Y_H<<8|GYRO_Y_L);                                  //Read high and low part of the angular data.
    gyro_axis[3] = (GYRO_Z_H<<8|GYRO_Z_L);                                  //Read high and low part of the angular data.

    if(cal_int == 2000)
    {
        gyro_axis[1] -= gyro_axis_cal[1];                                     //Only compensate after the calibration.
        gyro_axis[2] -= gyro_axis_cal[2];                                     //Only compensate after the calibration.
        gyro_axis[3] -= gyro_axis_cal[3];                                     //Only compensate after the calibration.
    }
    gyro_roll = gyro_axis[1];                                               //Set gyro_roll to the correct axis.
    gyro_pitch = gyro_axis[2];                                              //Set gyro_pitch to the correct axis.
    gyro_pitch *= -1;                                                       //Invert gyro_pitch to change the axis of sensor data.
    gyro_yaw = gyro_axis[3];                                                //Set gyro_yaw to the correct axis.
    gyro_yaw *= -1;                                                         //Invert gyro_yaw to change the axis of sensor data.


    acc_x = acc_axis[2];                                                    //Set acc_x to the correct axis.
    acc_x *= -1;                                                            //Invert acc_x.
    acc_y = acc_axis[1];                                                    //Set acc_y to the correct axis.
    acc_z = acc_axis[3];                                                    //Set acc_z to the correct axis.
    acc_z *= -1;                                                            //Invert acc_z.

}

void vertical_acceleration_calculations(void) {
    acc_z_average_short_rotating_mem_location++;
    if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

    acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
    acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
    acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

    if (acc_z_average_short_rotating_mem_location == 0) {
        acc_z_average_long_rotating_mem_location++;

        if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

        acc_z_average_long_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
        acc_z_average_long[acc_z_average_long_rotating_mem_location] = acc_z_average_short_total / 25;
        acc_z_average_long_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
    }
    acc_z_average_total = acc_z_average_long_total / 50;


    acc_alt_integrated += acc_total_vector - acc_z_average_total;
    if (acc_total_vector - acc_z_average_total < 400 || acc_total_vector - acc_z_average_total > 400) {
        if (acc_z_average_short_total / 25 - acc_z_average_total < 500 && acc_z_average_short_total / 25 - acc_z_average_total > -500)
            if (acc_alt_integrated > 200)acc_alt_integrated -= 200;
            else if (acc_alt_integrated < -200)acc_alt_integrated += 200;
    }
}
