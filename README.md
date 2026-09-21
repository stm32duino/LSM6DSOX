# LSM6DSOX
Arduino library to support the LSM6DSOX 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C, I3C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

For I3C it is then required to create an I3C interface before accessing to the sensors:

    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSOXSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSOXSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with SETDASA (static-to-dynamic address assignment):

    LSM6DSOXSensor AccGyr(&I3C, LSM6DSOX_I3C_ADD_H);
    I3C.resetDynamicAddresses();
    I3C.assignDynamicAddress(AccGyr.getStaticAddress(), LSM6DSOX_DYNAMIC_ADDRESS);
    AccGyr.begin(LSM6DSOX_DYNAMIC_ADDRESS);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with ENTDAA (dynamic address discovery):

    LSM6DSOXSensor AccGyr(&I3C);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.discover(devices, 8, &found);
    // find dynAddr by matching LSM6DSOX_I3C_PID in discovered devices
    AccGyr.begin(dynAddr);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.Get_X_Axes(accelerometer);  
    AccGyr.Get_G_Axes(gyroscope);

# Examples

There are several examples with the LSM6DSOX library.
* LSM6DSOX_HelloWorld_I2C: This application provides a simple example of usage of the LSM6DSOX IMU 6-axis over I2C.
* LSM6DSOX_FIFO_Interrupt_I2C: This application shows how to use the LSM6DSOX FIFO in interrupt mode over I2C.
* LSM6DSOX_FIFO_Polling_I2C: This application shows how to use the LSM6DSOX FIFO in polling mode over I2C.
* LSM6DSOX_Datalog_Terminal_I3C: This application shows how to use LSM6DSOX accelerometer and gyroscope over I3C using SETDASA.
* LSM6DSOX_Datalog_Terminal_I3C_ENTDAA: This application shows how to discover and use LSM6DSOX dynamic address over I3C.

## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSOX

The LSM6DSOX datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsox.html
