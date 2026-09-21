/*
   @file    LSM6DSOX_Datalog_Terminal_I3C_ENTDAA.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSOX sensor with I3C dynamic address assignment
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

#include "LSM6DSOXSensor.h"

LSM6DSOXSensor sensor(&I3C);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LSM6DSOX DAA ===");

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }

  I3CDiscoveredDevice devices[8] = {};
  size_t found = 0;

  if (I3C.discover(devices, 8, &found)) {
    Serial.println("discover() failed");
    while (1) {}
  }

  uint8_t sensorDynAddr = 0U;

  for (size_t i = 0; i < found; ++i) {
    Serial.println(devices[i].pid, HEX);
    if (devices[i].pid == LSM6DSOX_I3C_PID) {
      sensorDynAddr = devices[i].dynAddr;
      Serial.print("sensorDynAddr=");
      Serial.println(sensorDynAddr, HEX);
      break;
    }
  }

  if (sensorDynAddr == 0U) {
    Serial.println("Sensor not found");
    while (1) {}
  }

  if (sensor.begin(sensorDynAddr) != LSM6DSOX_OK) {
    Serial.println("sensor.begin() failed");
    while (1) {}
  }

  if (!I3C.setClock(12500000)) {
    Serial.println("setClock() failed");
    while (1) {}
  }

  if (sensor.Enable_X() != LSM6DSOX_OK) {
    Serial.println("sensor.Enable_X() failed");
    while (1) {}
  }

  if (sensor.Enable_G() != LSM6DSOX_OK) {
    Serial.println("sensor.Enable_G() failed");
    while (1) {}
  }

  Serial.println("LSM6DSOX ready");
}

void loop()
{
  int32_t accel[3] = {0};
  int32_t angrate[3] = {0};

  if (sensor.Get_X_Axes(accel) == LSM6DSOX_OK && sensor.Get_G_Axes(angrate) == LSM6DSOX_OK) {
    Serial.print("Accel-X[mg]:");
    Serial.print(accel[0]);
    Serial.print(",Accel-Y[mg]:");
    Serial.print(accel[1]);
    Serial.print(",Accel-Z[mg]:");
    Serial.println(accel[2]);

    Serial.print("AngRate-X[mdps]:");
    Serial.print(angrate[0]);
    Serial.print(",AngRate-Y[mdps]:");
    Serial.print(angrate[1]);
    Serial.print(",AngRate-Z[mdps]:");
    Serial.println(angrate[2]);
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
