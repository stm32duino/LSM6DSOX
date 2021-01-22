/**
 ******************************************************************************
   @file    LSM6DSOX_fifo_example_i2c.ino
   @author  Benhalor
   @version V1.0.1
   @date    January 2020
   @brief   Basic example for using the LSM6DSOX library with FIFO.
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
  plug Nucleo "+5V" to AdafruitLSM6DOX "VIN",
  plug Nucleo "GND" to AdafruitLSM6DOX "GND",
  plug Nucleo "SCL"(D15) to AdafruitLSM6DOX "SCL",
  plug Nucleo "SDA"(D14) to AdafruitLSM6DOX "SDA".*/

#include "LSM6DSOXSensor.h"

// Declare LSM6DSOX sensor. Sensor address can have 2 values LSM6DSOX_I2C_ADD_L (corresponds to 0x6A I2C address) or LSM6DSOX_I2C_ADD_H (corresponds to 0x6B I2C address) 
// On Adafruit lsm6dsox board, LSM6DSOX_I2C_ADD_L is the default address 
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Default clock is 100kHz. LSM6DSOX also supports 400kHz, let's use it
  Wire.setClock(400000);

  // Init the sensor
  lsm6dsoxSensor.begin();

  // Enable accelerometer and gyroscope, and check success
  if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK) {
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


  // Set Accelerometer sample rate to 52 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_X_ODR(52.0f);


  // Set Gyroscope sample rate to 52 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_G_ODR(52.0f);

  // Set FIFO batch data rate for each sensor. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_FIFO_X_BDR(52.0f);
  lsm6dsoxSensor.Set_FIFO_G_BDR(52.0f);

  // Setup Watermark : the max number of sensor values in the FIFO.
  lsm6dsoxSensor.Set_FIFO_Watermark_Level(200u);
  // Enable the watermark
  lsm6dsoxSensor.Set_FIFO_Stop_On_Fth(1);

  // Set FIFO to continuous mode. Continuous mode provides a continuous FIFO update: as new data arrives, the older data is discarded.
  lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);
}

void loop() {

  // Given the 52Hz batch data rate, for both accelero,and gyro, aroud 2000ms will be required to reach the 200 watermark threshold.
  delay(2000);

  uint8_t fifoStatus = 0;
  uint16_t NumSamples = 0;
  uint8_t Tag;
  int32_t acceleration[3];
  int32_t rotation[3];

  // Check that FIFO is full.
  lsm6dsoxSensor.Get_FIFO_Full_Status(&fifoStatus);
  if (fifoStatus == 0) {
    Serial.println("Accelero Gyro sensor : Data is not ready. Waiting for data");
    Serial.print("Accelero Gyro sensor : Number of available samples :");
    Serial.println(NumSamples);
  }
  while (fifoStatus == 0) { // If not full, wait unit FIFO is full
    lsm6dsoxSensor.Get_FIFO_Full_Status(&fifoStatus);
  }

  for (int32_t i = 0; i < 200; i++) {
    // both accelerometer and gyroscope write in the FIFO. Before reading a data from the FIFO, the first step is to check the TAG of this data
    lsm6dsoxSensor.Get_FIFO_Tag(&Tag);
    
    // Tag 1 corresponds to Gyroscope data
    if (Tag == 1u) { 
      lsm6dsoxSensor.Get_FIFO_G_Axes(rotation);
      // Plot data for each axe in milli degres per second
      Serial.print("RotationX="); Serial.print(rotation[0]); Serial.print("mdps, RotationY="); Serial.print(rotation[1]); Serial.print("mdps, RotationZ="); Serial.print(rotation[2]); Serial.println("mdps");

    } 
    
    // Tag 2 corresponds to Accelerometer data
    else if (Tag == 2u) {
      lsm6dsoxSensor.Get_FIFO_X_Axes(acceleration); 
      // Plot data for each axe in mg
      Serial.print("AccelerationX="); Serial.print(acceleration[0]); Serial.print("mg, AccelerationY="); Serial.print(acceleration[1]); Serial.print("mg, AccelerationZ="); Serial.print(acceleration[2]); Serial.println("mg");
    }
  }




}
