/**
 ******************************************************************************
   @file    LSM6DSOX_HelloWorld.ino
   @author  Benhalor
   @version V1.0.1
   @date    January 2020
   @brief   Basic example for using the LSM6DSOX library .
 ******************************************************************************
   @attention
   <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
     3. Neither the name of STMicroelectronics nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************
*/

/* WIRING
  In order to use the Adafruit lsm6dsox sensor with a ST nucleo board,
  plug Nucleo "+3.3V" to AdafruitLSM6DOX "VIN",
  plug Nucleo "GND" to AdafruitLSM6DOX "GND",
  plug Nucleo "SCL"(D15) to AdafruitLSM6DOX "SCL",
  plug Nucleo "SDA"(D14) to AdafruitLSM6DOX "SDA".*/

#include "LSM6DSOXSensor.h"

// Declare LSM6DSOX sensor. Sensor address can have 2 values LSM6DSOX_I2C_ADD_L (corresponds to 0x6A I2C address) or LSM6DSOX_I2C_ADD_H (corresponds to 0x6B I2C address)
// On Adafruit lsm6dsox sensor, LSM6DSOX_I2C_ADD_L is the default address
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Default clock is 100kHz. LSM6DSOX also supports 400kHz, let's use it
  Wire.setClock(400000);

  // Init the sensor
  lsm6dsoxSensor.begin();

  // Enable accelerometer and gyroscope, and check success
  if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
    Serial.println("Success enabling accelero and gyro");
  } else {
    Serial.println("Error enabling accelero and gyro");
  }

  // Read ID of device and check that it is correct
  uint8_t id;
  lsm6dsoxSensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
  } else {
    Serial.println("Receviced correct ID for LSM6DSOX sensor");
  }

  // Set accelerometer scale at +- 2G. Available values are +- 2, 4, 8, 16 G
  lsm6dsoxSensor.Set_X_FS(2);

  // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
  lsm6dsoxSensor.Set_G_FS(125);


  // Set Accelerometer sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_X_ODR(208.0f);


  // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_G_ODR(208.0f);


}

void loop() {

  // Read accelerometer
  uint8_t acceleroStatus;
  lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
  if (acceleroStatus == 1) { // Status == 1 means a new data is available
    int32_t acceleration[3];
    lsm6dsoxSensor.Get_X_Axes(acceleration);
    // Plot data for each axis in mg
    Serial.print("AccelerationX="); Serial.print(acceleration[0]); Serial.print("mg, AccelerationY="); Serial.print(acceleration[1]); Serial.print("mg, AccelerationZ="); Serial.print(acceleration[2]); Serial.println("mg");
  }

  // Read gyroscope
  uint8_t gyroStatus;
  lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
  if (gyroStatus == 1) { // Status == 1 means a new data is available
    int32_t rotation[3];
    lsm6dsoxSensor.Get_G_Axes(rotation);
    // Plot data for each axis in milli degrees per second
    Serial.print("RotationX="); Serial.print(rotation[0]); Serial.print("mdps, RotationY="); Serial.print(rotation[1]); Serial.print("mdps, RotationZ="); Serial.print(rotation[2]); Serial.println("mdps");
  }

  delay(10);

}