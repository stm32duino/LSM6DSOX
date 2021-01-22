/**
 ******************************************************************************
 * @file    LSM6DSOXSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Abstract Class of an LSM6DSOX Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DSOXSensor_H__
#define __LSM6DSOXSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "lsm6dsox_reg.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DSOX_ACC_SENSITIVITY_FS_2G   0.061f
#define LSM6DSOX_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSOX_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSOX_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSOX_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_2000DPS  70.000f


/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  LSM6DSOX_OK = 0,
  LSM6DSOX_ERROR =-1
} LSM6DSOXStatusTypeDef;

typedef enum
{
  LSM6DSOX_INT1_PIN,
  LSM6DSOX_INT2_PIN,
} LSM6DSOX_SensorIntPin_t;

typedef enum
{
  LSM6DSOX_ACC_HIGH_PERFORMANCE_MODE,
  LSM6DSOX_ACC_LOW_POWER_NORMAL_MODE,
  LSM6DSOX_ACC_ULTRA_LOW_POWER_MODE
} LSM6DSOX_ACC_Operating_Mode_t;

typedef enum
{
  LSM6DSOX_GYRO_HIGH_PERFORMANCE_MODE,
  LSM6DSOX_GYRO_LOW_POWER_NORMAL_MODE
} LSM6DSOX_GYRO_Operating_Mode_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LSM6DSOX_Event_Status_t;

typedef struct {
  unsigned int is_mlc1 : 1;
  unsigned int is_mlc2 : 1;
  unsigned int is_mlc3 : 1;
  unsigned int is_mlc4 : 1;
  unsigned int is_mlc5 : 1;
  unsigned int is_mlc6 : 1;
  unsigned int is_mlc7 : 1;
  unsigned int is_mlc8 : 1;
} LSM6DSOX_MLC_Status_t;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an LSM6DSOX Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LSM6DSOXSensor
{
  public:
    LSM6DSOXSensor(TwoWire *i2c, uint8_t address=LSM6DSOX_I2C_ADD_H);
    LSM6DSOXSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    LSM6DSOXStatusTypeDef begin();
    LSM6DSOXStatusTypeDef end();
    LSM6DSOXStatusTypeDef ReadID(uint8_t *Id);
    LSM6DSOXStatusTypeDef Enable_X();
    LSM6DSOXStatusTypeDef Disable_X();
    LSM6DSOXStatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    LSM6DSOXStatusTypeDef Get_X_ODR(float *Odr);
    LSM6DSOXStatusTypeDef Set_X_ODR(float Odr);
    LSM6DSOXStatusTypeDef Set_X_ODR_With_Mode(float Odr, LSM6DSOX_ACC_Operating_Mode_t Mode);
    LSM6DSOXStatusTypeDef Get_X_FS(int32_t *FullScale);
    LSM6DSOXStatusTypeDef Set_X_FS(int32_t FullScale);
    LSM6DSOXStatusTypeDef Get_X_AxesRaw(int16_t *Value);
    LSM6DSOXStatusTypeDef Get_X_Axes(int32_t *Acceleration);
    
    LSM6DSOXStatusTypeDef Enable_G();
    LSM6DSOXStatusTypeDef Disable_G();
    LSM6DSOXStatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    LSM6DSOXStatusTypeDef Get_G_ODR(float *Odr);
    LSM6DSOXStatusTypeDef Set_G_ODR(float Odr);
    LSM6DSOXStatusTypeDef Set_G_ODR_With_Mode(float Odr, LSM6DSOX_GYRO_Operating_Mode_t Mode);
    LSM6DSOXStatusTypeDef Get_G_FS(int32_t *FullScale);
    LSM6DSOXStatusTypeDef Set_G_FS(int32_t FullScale);
    LSM6DSOXStatusTypeDef Get_G_AxesRaw(int16_t *Value);
    LSM6DSOXStatusTypeDef Get_G_Axes(int32_t *AngularRate);
    
    LSM6DSOXStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    LSM6DSOXStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    LSM6DSOXStatusTypeDef Set_Interrupt_Latch(uint8_t Status);
    
    LSM6DSOXStatusTypeDef Enable_Free_Fall_Detection(LSM6DSOX_SensorIntPin_t IntPin);
    LSM6DSOXStatusTypeDef Disable_Free_Fall_Detection();
    LSM6DSOXStatusTypeDef Set_Free_Fall_Threshold(uint8_t Threshold);
    LSM6DSOXStatusTypeDef Set_Free_Fall_Duration(uint8_t Duration);
    
    LSM6DSOXStatusTypeDef Enable_Pedometer();
    LSM6DSOXStatusTypeDef Disable_Pedometer();
    LSM6DSOXStatusTypeDef Get_Step_Count(uint16_t *StepCount);
    LSM6DSOXStatusTypeDef Step_Counter_Reset();
    
    LSM6DSOXStatusTypeDef Enable_Tilt_Detection(LSM6DSOX_SensorIntPin_t IntPin);
    LSM6DSOXStatusTypeDef Disable_Tilt_Detection();
    
    LSM6DSOXStatusTypeDef Enable_Wake_Up_Detection(LSM6DSOX_SensorIntPin_t IntPin);
    LSM6DSOXStatusTypeDef Disable_Wake_Up_Detection();
    LSM6DSOXStatusTypeDef Set_Wake_Up_Threshold(uint8_t Threshold);
    LSM6DSOXStatusTypeDef Set_Wake_Up_Duration(uint8_t Duration);
    
    LSM6DSOXStatusTypeDef Enable_Single_Tap_Detection(LSM6DSOX_SensorIntPin_t IntPin);
    LSM6DSOXStatusTypeDef Disable_Single_Tap_Detection();
    LSM6DSOXStatusTypeDef Enable_Double_Tap_Detection(LSM6DSOX_SensorIntPin_t IntPin);
    LSM6DSOXStatusTypeDef Disable_Double_Tap_Detection();
    LSM6DSOXStatusTypeDef Set_Tap_Threshold(uint8_t Threshold);
    LSM6DSOXStatusTypeDef Set_Tap_Shock_Time(uint8_t Time);
    LSM6DSOXStatusTypeDef Set_Tap_Quiet_Time(uint8_t Time);
    LSM6DSOXStatusTypeDef Set_Tap_Duration_Time(uint8_t Time);
    
    LSM6DSOXStatusTypeDef Enable_6D_Orientation(LSM6DSOX_SensorIntPin_t IntPin);
    LSM6DSOXStatusTypeDef Disable_6D_Orientation();
    LSM6DSOXStatusTypeDef Set_6D_Orientation_Threshold(uint8_t Threshold);
    LSM6DSOXStatusTypeDef Get_6D_Orientation_XL(uint8_t *XLow);
    LSM6DSOXStatusTypeDef Get_6D_Orientation_XH(uint8_t *XHigh);
    LSM6DSOXStatusTypeDef Get_6D_Orientation_YL(uint8_t *YLow);
    LSM6DSOXStatusTypeDef Get_6D_Orientation_YH(uint8_t *YHigh);
    LSM6DSOXStatusTypeDef Get_6D_Orientation_ZL(uint8_t *ZLow);
    LSM6DSOXStatusTypeDef Get_6D_Orientation_ZH(uint8_t *ZHigh);
    
    LSM6DSOXStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    LSM6DSOXStatusTypeDef Get_X_Event_Status(LSM6DSOX_Event_Status_t *Status);
    LSM6DSOXStatusTypeDef Set_X_SelfTest(uint8_t Status);
    
    LSM6DSOXStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    LSM6DSOXStatusTypeDef Set_G_SelfTest(uint8_t Status);
    
    LSM6DSOXStatusTypeDef Get_FIFO_Num_Samples(uint16_t *NumSamples);
    LSM6DSOXStatusTypeDef Get_FIFO_Full_Status(uint8_t *Status);
    LSM6DSOXStatusTypeDef Set_FIFO_INT1_FIFO_Full(uint8_t Status);
    LSM6DSOXStatusTypeDef Set_FIFO_Watermark_Level(uint16_t Watermark);
    LSM6DSOXStatusTypeDef Set_FIFO_Stop_On_Fth(uint8_t Status);
    LSM6DSOXStatusTypeDef Set_FIFO_Mode(uint8_t Mode);
    LSM6DSOXStatusTypeDef Get_FIFO_Tag(uint8_t *Tag);
    LSM6DSOXStatusTypeDef Get_FIFO_Data(uint8_t *Data);
    LSM6DSOXStatusTypeDef Get_FIFO_X_Axes(int32_t *Acceleration);
    LSM6DSOXStatusTypeDef Set_FIFO_X_BDR(float Bdr);
    LSM6DSOXStatusTypeDef Get_FIFO_G_Axes(int32_t *AngularVelocity);
    LSM6DSOXStatusTypeDef Set_FIFO_G_BDR(float Bdr);

    LSM6DSOXStatusTypeDef Get_MLC_Status(LSM6DSOX_MLC_Status_t *Status);
    LSM6DSOXStatusTypeDef Get_MLC_Output(uint8_t *Output);
    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {        
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i=0; i<NumByteToRead; i++) {
          *(pBuffer+i) = dev_spi->transfer(0x00);
        }
         
        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }
		
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i=0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {  
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i=0; i<NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;                    
      }
  
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
  
    LSM6DSOXStatusTypeDef Set_X_ODR_When_Enabled(float Odr);
    LSM6DSOXStatusTypeDef Set_X_ODR_When_Disabled(float Odr);
    LSM6DSOXStatusTypeDef Set_G_ODR_When_Enabled(float Odr);
    LSM6DSOXStatusTypeDef Set_G_ODR_When_Disabled(float Odr);
  
  

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;
    
    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;
    
    lsm6dsox_odr_xl_t acc_odr;
    lsm6dsox_odr_g_t gyro_odr;
    
    uint8_t acc_is_enabled;
    uint8_t gyro_is_enabled;
    
    
    lsm6dsox_ctx_t reg_ctx;
    
};

#ifdef __cplusplus
 extern "C" {
#endif
int32_t LSM6DSOX_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t LSM6DSOX_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
