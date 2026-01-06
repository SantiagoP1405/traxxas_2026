/*
 ***************************************************************************
 
    Euler_Streaming.pde - part of sample SW for using BNO055 with Arduino
 
   (C) All rights reserved by ROBERT BOSCH GMBH
 
   Copyright (C) 2014 Bosch Sensortec GmbH
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 **************************************************************************/
/*  Date: 2014/01/07
     Revision: 1.2
 
*/
 
#include "BNO055_support.h"     //Contains the bridge code between the API and Arduino
#include <Wire.h>
 
//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2
 
//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)
 
//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_quaternion myQuat;
struct bno055_gyro myGyro;
struct bno055_linear_accel myLinAcc;
 
unsigned long lastTime = 0;
 
void setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin();
 
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
 
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(10);
 
  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
}
 
void loop() //This code is looped forever
{
  if ((millis() - lastTime) >= 100) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();
 
    bno055_read_euler_hrp(&myEulerData);            //Update Euler data into the structure
    bno055_read_quaternion_wxyz(&myQuat);
    bno055_read_gyro_xyz(&myGyro);
    bno055_read_linear_accel_xyz(&myLinAcc);
 
    float yaw   = myEulerData.h / 16.0;
    float roll  = myEulerData.r / 16.0;
    float pitch = myEulerData.p / 16.0;

    // gyro viene en 1 LSB = 1/16 dps → conv a rad/s
    float gx = (myGyro.x / 16.0) * (M_PI / 180.0);
    float gy = (myGyro.y / 16.0) * (M_PI / 180.0);
    float gz = (myGyro.z / 16.0) * (M_PI / 180.0);

    // linear accel viene en mg → m/s^2
    float ax = myLinAcc.x / 100.0;
    float ay = myLinAcc.y / 100.0;
    float az = myLinAcc.z / 100.0;

    // quaternion: librería ya da signo correcto
    float qw = myQuat.w / 16384.0;
    float qx = myQuat.x / 16384.0;
    float qy = myQuat.y / 16384.0;
    float qz = myQuat.z / 16384.0;

    // enviar CSV para ROS:
    // yaw,pitch,roll,qx,qy,qz,qw,gx,gy,gz,ax,ay,az
    Serial.print(yaw); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.print(roll); Serial.print(",");
    Serial.print(qx); Serial.print(",");
    Serial.print(qy); Serial.print(",");
    Serial.print(qz); Serial.print(",");
    Serial.print(qw); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.println(az);
  }
}