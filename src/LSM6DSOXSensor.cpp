/**
 ******************************************************************************
 * @file    LSM6DSOXSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Implementation of an LSM6DSOX Inertial Measurement Unit (IMU) 3 axes
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


/* Includes ------------------------------------------------------------------*/

#include "LSM6DSOXSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DSOXSensor::LSM6DSOXSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = LSM6DSOX_io_write;
  reg_ctx.read_reg = LSM6DSOX_io_read;
  reg_ctx.handle = (void *)this;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DSOXSensor::LSM6DSOXSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LSM6DSOX_io_write;
  reg_ctx.read_reg = LSM6DSOX_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0; 
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;  
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::begin()
{
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); 
  }

  /* Disable I3C */
  if (lsm6dsox_i3c_disable_set(&reg_ctx, LSM6DSOX_I3C_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dsox_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable BDU */
  if (lsm6dsox_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* FIFO mode selection */
  if (lsm6dsox_fifo_mode_set(&reg_ctx, LSM6DSOX_BYPASS_MODE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = LSM6DSOX_XL_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsox_xl_data_rate_set(&reg_ctx, LSM6DSOX_XL_ODR_OFF) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsox_xl_full_scale_set(&reg_ctx, LSM6DSOX_2g) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Select default output data rate. */
  gyro_odr = LSM6DSOX_GY_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsox_gy_data_rate_set(&reg_ctx, LSM6DSOX_GY_ODR_OFF) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsox_gy_full_scale_set(&reg_ctx, LSM6DSOX_2000dps) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }
  
  acc_is_enabled = 0;
  gyro_is_enabled = 0;

  return LSM6DSOX_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::end()
{
  /* Disable both acc and gyro */
  if (Disable_X() != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (Disable_G() != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset CS configuration */
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, INPUT); 
  }

  return LSM6DSOX_OK;
}


/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::ReadID(uint8_t *Id)
{
  if (lsm6dsox_device_id_get(&reg_ctx, Id) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable the LSM6DSOX accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U)
  {
    return LSM6DSOX_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsox_xl_data_rate_set(&reg_ctx, acc_odr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  acc_is_enabled = 1;

  return LSM6DSOX_OK;
}

/**
 * @brief  Disable the LSM6DSOX accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U)
  {
    return LSM6DSOX_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsox_xl_data_rate_get(&reg_ctx, &acc_odr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsox_xl_data_rate_set(&reg_ctx, LSM6DSOX_XL_ODR_OFF) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  acc_is_enabled = 0;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX accelerometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_Sensitivity(float *Sensitivity)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_fs_xl_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsox_xl_full_scale_get(&reg_ctx, &full_scale) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case LSM6DSOX_2g:
      *Sensitivity = LSM6DSOX_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSOX_4g:
      *Sensitivity = LSM6DSOX_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSOX_8g:
      *Sensitivity = LSM6DSOX_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSOX_16g:
      *Sensitivity = LSM6DSOX_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSOX accelerometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_ODR(float *Odr)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_odr_xl_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsox_xl_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM6DSOX_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSOX_XL_ODR_1Hz6:
      *Odr = 1.6f;
      break;

    case LSM6DSOX_XL_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case LSM6DSOX_XL_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case LSM6DSOX_XL_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case LSM6DSOX_XL_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case LSM6DSOX_XL_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case LSM6DSOX_XL_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case LSM6DSOX_XL_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case LSM6DSOX_XL_ODR_1667Hz:
      *Odr = 1667.0f;
      break;

    case LSM6DSOX_XL_ODR_3333Hz:
      *Odr = 3333.0f;
      break;

    case LSM6DSOX_XL_ODR_6667Hz:
      *Odr = 6667.0f;
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX accelerometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_X_ODR(float Odr)
{
  return Set_X_ODR_With_Mode(Odr, LSM6DSOX_ACC_HIGH_PERFORMANCE_MODE);
}

/**
 * @brief  Set the LSM6DSOX accelerometer sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the accelerometer operating mode
 * @note   This function switches off the gyroscope if Ultra Low Power Mode is set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_X_ODR_With_Mode(float Odr, LSM6DSOX_ACC_Operating_Mode_t Mode)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;

  switch (Mode)
  {
    case LSM6DSOX_ACC_HIGH_PERFORMANCE_MODE:
    {
      /* We must uncheck Low Power and Ultra Low Power bits if they are enabled */
      lsm6dsox_ctrl5_c_t val1;
      lsm6dsox_ctrl6_c_t val2;

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL5_C, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (val1.xl_ulp_en)
      {
        /* Power off the accelerometer */
        if (acc_is_enabled == 1U)
        {
          if (lsm6dsox_xl_data_rate_set(&reg_ctx, LSM6DSOX_XL_ODR_OFF) != LSM6DSOX_OK)
          {
            return LSM6DSOX_ERROR;
          }
        }

        val1.xl_ulp_en = 0;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL5_C, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL6_C, (uint8_t *)&val2, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (val2.xl_hm_mode)
      {
        val2.xl_hm_mode = 0;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL6_C, (uint8_t *)&val2, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      /* ODR should be at least 12.5Hz */
      if (Odr < 12.5f)
      {
        Odr = 12.5f;
      }
      break;
    }
    case LSM6DSOX_ACC_LOW_POWER_NORMAL_MODE:
    {
      /* We must uncheck Ultra Low Power bit if it is enabled */
      /* and check the Low Power bit if it is unchecked       */
      lsm6dsox_ctrl5_c_t val1;
      lsm6dsox_ctrl6_c_t val2;

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL5_C, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (val1.xl_ulp_en)
      {
        /* Power off the accelerometer */
        if (acc_is_enabled == 1U)
        {
          if (lsm6dsox_xl_data_rate_set(&reg_ctx, LSM6DSOX_XL_ODR_OFF) != LSM6DSOX_OK)
          {
            return LSM6DSOX_ERROR;
          }
        }

        val1.xl_ulp_en = 0;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL5_C, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL6_C, (uint8_t *)&val2, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (!val2.xl_hm_mode)
      {
        val2.xl_hm_mode = 1U;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL6_C, (uint8_t *)&val2, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      /* Now we need to limit the ODR to 208 Hz if it is higher */
      if (Odr > 208.0f)
      {
        Odr = 208.0f;
      }
      break;
    }
    case LSM6DSOX_ACC_ULTRA_LOW_POWER_MODE:
    {
      /* We must uncheck Low Power bit if it is enabled                   */
      /* and check the Ultra Low Power bit if it is unchecked             */
      /* We must switch off gyro otherwise Ultra Low Power does not work  */
      lsm6dsox_ctrl5_c_t val1;
      lsm6dsox_ctrl6_c_t val2;

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL6_C, (uint8_t *)&val2, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (val2.xl_hm_mode)
      {
        val2.xl_hm_mode = 0;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL6_C, (uint8_t *)&val2, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      /* Disable Gyro */
      if (gyro_is_enabled == 1U)
      {
        if (Disable_G() != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL5_C, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (!val1.xl_ulp_en)
      {
        /* Power off the accelerometer */
        if (acc_is_enabled == 1U)
        {
          if (lsm6dsox_xl_data_rate_set(&reg_ctx, LSM6DSOX_XL_ODR_OFF) != LSM6DSOX_OK)
          {
            return LSM6DSOX_ERROR;
          }
        }

        val1.xl_ulp_en = 1U;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL5_C, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      /* Now we need to limit the ODR to 208 Hz if it is higher */
      if (Odr > 208.0f)
      {
        Odr = 208.0f;
      }
      break;
    }
    default:
      ret = LSM6DSOX_ERROR;
      break;
  }
  
  /* Check if the component is enabled */
  if (acc_is_enabled == 1U)
  {
    ret = Set_X_ODR_When_Enabled(Odr);
  }
  else
  {
    ret = Set_X_ODR_When_Disabled(Odr);
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX accelerometer sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_X_ODR_When_Enabled(float Odr)
{
  lsm6dsox_odr_xl_t new_odr;

  new_odr = (Odr <=    1.6f) ? LSM6DSOX_XL_ODR_1Hz6
          : (Odr <=   12.5f) ? LSM6DSOX_XL_ODR_12Hz5
          : (Odr <=   26.0f) ? LSM6DSOX_XL_ODR_26Hz
          : (Odr <=   52.0f) ? LSM6DSOX_XL_ODR_52Hz
          : (Odr <=  104.0f) ? LSM6DSOX_XL_ODR_104Hz
          : (Odr <=  208.0f) ? LSM6DSOX_XL_ODR_208Hz
          : (Odr <=  417.0f) ? LSM6DSOX_XL_ODR_417Hz
          : (Odr <=  833.0f) ? LSM6DSOX_XL_ODR_833Hz
          : (Odr <= 1667.0f) ? LSM6DSOX_XL_ODR_1667Hz
          : (Odr <= 3333.0f) ? LSM6DSOX_XL_ODR_3333Hz
          :                    LSM6DSOX_XL_ODR_6667Hz;

  /* Output data rate selection. */
  if (lsm6dsox_xl_data_rate_set(&reg_ctx, new_odr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX accelerometer sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_X_ODR_When_Disabled(float Odr)
{
  acc_odr = (Odr <=    1.6f) ? LSM6DSOX_XL_ODR_1Hz6
          : (Odr <=   12.5f) ? LSM6DSOX_XL_ODR_12Hz5
          : (Odr <=   26.0f) ? LSM6DSOX_XL_ODR_26Hz
          : (Odr <=   52.0f) ? LSM6DSOX_XL_ODR_52Hz
          : (Odr <=  104.0f) ? LSM6DSOX_XL_ODR_104Hz
          : (Odr <=  208.0f) ? LSM6DSOX_XL_ODR_208Hz
          : (Odr <=  417.0f) ? LSM6DSOX_XL_ODR_417Hz
          : (Odr <=  833.0f) ? LSM6DSOX_XL_ODR_833Hz
          : (Odr <= 1667.0f) ? LSM6DSOX_XL_ODR_1667Hz
          : (Odr <= 3333.0f) ? LSM6DSOX_XL_ODR_3333Hz
          :                    LSM6DSOX_XL_ODR_6667Hz;

  return LSM6DSOX_OK;
}


/**
 * @brief  Get the LSM6DSOX accelerometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_FS(int32_t *FullScale)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_fs_xl_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsox_xl_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM6DSOX_2g:
      *FullScale =  2;
      break;

    case LSM6DSOX_4g:
      *FullScale =  4;
      break;

    case LSM6DSOX_8g:
      *FullScale =  8;
      break;

    case LSM6DSOX_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX accelerometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_X_FS(int32_t FullScale)
{
  lsm6dsox_fs_xl_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? LSM6DSOX_2g
         : (FullScale <= 4) ? LSM6DSOX_4g
         : (FullScale <= 8) ? LSM6DSOX_8g
         :                    LSM6DSOX_16g;

  if (lsm6dsox_xl_full_scale_set(&reg_ctx, new_fs) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX accelerometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_AxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsox_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSOX_OK;
}


/**
 * @brief  Get the LSM6DSOX accelerometer sensor axes
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_Axes(int32_t *Acceleration)
{
  axis3bit16_t data_raw;
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (lsm6dsox_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Get LSM6DSOX actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSOX_OK;
}


/**
 * @brief  Enable the LSM6DSOX gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U)
  {
    return LSM6DSOX_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsox_gy_data_rate_set(&reg_ctx, gyro_odr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  gyro_is_enabled = 1;

  return LSM6DSOX_OK;
}


/**
 * @brief  Disable the LSM6DSOX gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U)
  {
    return LSM6DSOX_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsox_gy_data_rate_get(&reg_ctx, &gyro_odr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsox_gy_data_rate_set(&reg_ctx, LSM6DSOX_GY_ODR_OFF) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  gyro_is_enabled = 0;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX gyroscope sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_G_Sensitivity(float *Sensitivity)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_fs_g_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsox_gy_full_scale_get(&reg_ctx, &full_scale) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case LSM6DSOX_125dps:
      *Sensitivity = LSM6DSOX_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSOX_250dps:
      *Sensitivity = LSM6DSOX_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSOX_500dps:
      *Sensitivity = LSM6DSOX_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSOX_1000dps:
      *Sensitivity = LSM6DSOX_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSOX_2000dps:
      *Sensitivity = LSM6DSOX_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSOX gyroscope sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_G_ODR(float *Odr)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_odr_g_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsox_gy_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM6DSOX_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSOX_GY_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case LSM6DSOX_GY_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case LSM6DSOX_GY_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case LSM6DSOX_GY_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case LSM6DSOX_GY_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case LSM6DSOX_GY_ODR_417Hz:
      *Odr = 417.0f;
      break;

    case LSM6DSOX_GY_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case LSM6DSOX_GY_ODR_1667Hz:
      *Odr =  1667.0f;
      break;

    case LSM6DSOX_GY_ODR_3333Hz:
      *Odr =  3333.0f;
      break;

    case LSM6DSOX_GY_ODR_6667Hz:
      *Odr =  6667.0f;
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX gyroscope sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_G_ODR(float Odr)
{
  return Set_G_ODR_With_Mode(Odr, LSM6DSOX_GYRO_HIGH_PERFORMANCE_MODE);
}

/**
 * @brief  Set the LSM6DSOX gyroscope sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the gyroscope operating mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_G_ODR_With_Mode(float Odr, LSM6DSOX_GYRO_Operating_Mode_t Mode)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;

  switch (Mode)
  {
    case LSM6DSOX_GYRO_HIGH_PERFORMANCE_MODE:
    {
      /* We must uncheck Low Power bit if it is enabled */
      lsm6dsox_ctrl7_g_t val1;

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL7_G, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (val1.g_hm_mode)
      {
        val1.g_hm_mode = 0;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL7_G, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }
      break;
    }
    case LSM6DSOX_GYRO_LOW_POWER_NORMAL_MODE:
    {
      /* We must check the Low Power bit if it is unchecked */
      lsm6dsox_ctrl7_g_t val1;

      if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_CTRL7_G, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      if (!val1.g_hm_mode)
      {
        val1.g_hm_mode = 1U;
        if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_CTRL7_G, (uint8_t *)&val1, 1) != LSM6DSOX_OK)
        {
          return LSM6DSOX_ERROR;
        }
      }

      /* Now we need to limit the ODR to 208 Hz if it is higher */
      if (Odr > 208.0f)
      {
        Odr = 208.0f;
      }
      break;
    }
    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  /* Check if the component is enabled */
  if (gyro_is_enabled == 1U)
  {
    ret = Set_G_ODR_When_Enabled(Odr);
  }
  else
  {
    ret = Set_G_ODR_When_Disabled(Odr);
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX gyroscope sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_G_ODR_When_Enabled(float Odr)
{
  lsm6dsox_odr_g_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSOX_GY_ODR_12Hz5
          : (Odr <=   26.0f) ? LSM6DSOX_GY_ODR_26Hz
          : (Odr <=   52.0f) ? LSM6DSOX_GY_ODR_52Hz
          : (Odr <=  104.0f) ? LSM6DSOX_GY_ODR_104Hz
          : (Odr <=  208.0f) ? LSM6DSOX_GY_ODR_208Hz
          : (Odr <=  417.0f) ? LSM6DSOX_GY_ODR_417Hz
          : (Odr <=  833.0f) ? LSM6DSOX_GY_ODR_833Hz
          : (Odr <= 1667.0f) ? LSM6DSOX_GY_ODR_1667Hz
          : (Odr <= 3333.0f) ? LSM6DSOX_GY_ODR_3333Hz
          :                    LSM6DSOX_GY_ODR_6667Hz;

  /* Output data rate selection. */
  if (lsm6dsox_gy_data_rate_set(&reg_ctx, new_odr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX gyroscope sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_G_ODR_When_Disabled(float Odr)
{
  gyro_odr = (Odr <=   12.5f) ? LSM6DSOX_GY_ODR_12Hz5
           : (Odr <=   26.0f) ? LSM6DSOX_GY_ODR_26Hz
           : (Odr <=   52.0f) ? LSM6DSOX_GY_ODR_52Hz
           : (Odr <=  104.0f) ? LSM6DSOX_GY_ODR_104Hz
           : (Odr <=  208.0f) ? LSM6DSOX_GY_ODR_208Hz
           : (Odr <=  417.0f) ? LSM6DSOX_GY_ODR_417Hz
           : (Odr <=  833.0f) ? LSM6DSOX_GY_ODR_833Hz
           : (Odr <= 1667.0f) ? LSM6DSOX_GY_ODR_1667Hz
           : (Odr <= 3333.0f) ? LSM6DSOX_GY_ODR_3333Hz
           :                    LSM6DSOX_GY_ODR_6667Hz;

  return LSM6DSOX_OK;
}


/**
 * @brief  Get the LSM6DSOX gyroscope sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_G_FS(int32_t  *FullScale)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_fs_g_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsox_gy_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM6DSOX_125dps:
      *FullScale =  125;
      break;

    case LSM6DSOX_250dps:
      *FullScale =  250;
      break;

    case LSM6DSOX_500dps:
      *FullScale =  500;
      break;

    case LSM6DSOX_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSOX_2000dps:
      *FullScale = 2000;
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX gyroscope sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_G_FS(int32_t FullScale)
{
  lsm6dsox_fs_g_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSOX_125dps
         : (FullScale <= 250)  ? LSM6DSOX_250dps
         : (FullScale <= 500)  ? LSM6DSOX_500dps
         : (FullScale <= 1000) ? LSM6DSOX_1000dps
         :                       LSM6DSOX_2000dps;

  if (lsm6dsox_gy_full_scale_set(&reg_ctx, new_fs) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX gyroscope sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_G_AxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsox_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSOX_OK;
}


/**
 * @brief  Get the LSM6DSOX gyroscope sensor axes
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_G_Axes(int32_t *AngularRate)
{
  axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (lsm6dsox_angular_rate_raw_get(&reg_ctx, data_raw.u8bit) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Get LSM6DSOX actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSOX_OK;
}


/**
 * @brief  Get the LSM6DSOX register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lsm6dsox_read_reg(&reg_ctx, Reg, Data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}


/**
 * @brief  Set the LSM6DSOX register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lsm6dsox_write_reg(&reg_ctx, Reg, &Data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the interrupt latch
 * @param  Status value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Interrupt_Latch(uint8_t Status)
{
  if (Status > 1U)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_int_notification_set(&reg_ctx, (lsm6dsox_lir_t)Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable free fall detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_Free_Fall_Detection(LSM6DSOX_SensorIntPin_t IntPin)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* FF_DUR setting */
  if (lsm6dsox_ff_dur_set(&reg_ctx, 0x06) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dsox_wkup_dur_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* SLEEP_DUR setting */
  if (lsm6dsox_act_sleep_dur_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* FF_THS setting */
  if (lsm6dsox_ff_threshold_set(&reg_ctx, LSM6DSOX_FF_TSH_312mg) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSOX_INT1_PIN:
      if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val1.free_fall = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    case LSM6DSOX_INT2_PIN:
      if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val2.free_fall = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable free fall detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_Free_Fall_Detection()
{
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Disable free fall event on both INT1 and INT2 pins */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.free_fall = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val2.free_fall = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* FF_DUR setting */
  if (lsm6dsox_ff_dur_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* FF_THS setting */
  if (lsm6dsox_ff_threshold_set(&reg_ctx, LSM6DSOX_FF_TSH_156mg) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set free fall threshold
 * @param  Threshold free fall detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Free_Fall_Threshold(uint8_t Threshold)
{
  if (lsm6dsox_ff_threshold_set(&reg_ctx, (lsm6dsox_ff_ths_t)Threshold) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}


/**
 * @brief  Set free fall duration
 * @param  Duration free fall detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Free_Fall_Duration(uint8_t Duration)
{
  if (lsm6dsox_ff_dur_set(&reg_ctx, Duration) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}


/**
 * @brief  Enable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_Pedometer()
{
  lsm6dsox_pin_int1_route_t val;
  lsm6dsox_emb_sens_t emb_sens;

  /* Output Data Rate selection */
  if (Set_X_ODR(26.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Save current embedded features */
  if (lsm6dsox_embedded_sens_get(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Turn off embedded features */
  if (lsm6dsox_embedded_sens_off(&reg_ctx) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Wait for 10 ms */
  delay(10);

  /* Enable pedometer algorithm. */
  emb_sens.step = PROPERTY_ENABLE;

  if (lsm6dsox_pedo_sens_set(&reg_ctx, LSM6DSOX_PEDO_BASE_MODE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Turn on embedded features */
  if (lsm6dsox_embedded_sens_set(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable step detector on INT1 pin */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val.step_detector = PROPERTY_ENABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Disable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_Pedometer()
{
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_emb_sens_t emb_sens;

  /* Disable step detector on INT1 pin */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.step_detector = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Save current embedded features */
  if (lsm6dsox_embedded_sens_get(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable pedometer algorithm. */
  emb_sens.step = PROPERTY_DISABLE;

  if (lsm6dsox_embedded_sens_set(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get step count
 * @param  StepCount step counter
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_Step_Count(uint16_t *StepCount)
{
  if (lsm6dsox_number_of_steps_get(&reg_ctx, (uint8_t *)StepCount) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable step counter reset
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Step_Counter_Reset()
{
  if (lsm6dsox_steps_reset(&reg_ctx) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable tilt detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_Tilt_Detection(LSM6DSOX_SensorIntPin_t IntPin)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;
  lsm6dsox_emb_sens_t emb_sens;

  /* Output Data Rate selection */
  if (Set_X_ODR(26.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Save current embedded features */
  if (lsm6dsox_embedded_sens_get(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Turn off embedded features */
  if (lsm6dsox_embedded_sens_off(&reg_ctx) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Wait for 10 ms */
  delay(10);

  /* Enable tilt algorithm. */
  emb_sens.tilt = PROPERTY_ENABLE;

  /* Turn on embedded features */
  if (lsm6dsox_embedded_sens_set(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable tilt event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSOX_INT1_PIN:
      if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val1.tilt = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    case LSM6DSOX_INT2_PIN:
      if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val2.tilt = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable tilt detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_Tilt_Detection()
{
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;
  lsm6dsox_emb_sens_t emb_sens;

  /* Disable tilt event on both INT1 and INT2 pins */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.tilt = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val2.tilt = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Save current embedded features */
  if (lsm6dsox_embedded_sens_get(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable tilt algorithm. */
  emb_sens.tilt = PROPERTY_DISABLE;

  if (lsm6dsox_embedded_sens_set(&reg_ctx, &emb_sens) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable wake up detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_Wake_Up_Detection(LSM6DSOX_SensorIntPin_t IntPin)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dsox_wkup_dur_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set wake up threshold. */
  if (lsm6dsox_wkup_threshold_set(&reg_ctx, 0x02) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSOX_INT1_PIN:
      if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val1.wake_up = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    case LSM6DSOX_INT2_PIN:
      if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val2.wake_up = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}


/**
 * @brief  Disable wake up detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_Wake_Up_Detection()
{
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Disable wake up event on both INT1 and INT2 pins */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.wake_up = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val2.wake_up = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset wake up threshold. */
  if (lsm6dsox_wkup_threshold_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dsox_wkup_dur_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set wake up threshold
 * @param  Threshold wake up detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Wake_Up_Threshold(uint8_t Threshold)
{
  /* Set wake up threshold. */
  if (lsm6dsox_wkup_threshold_set(&reg_ctx, Threshold) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set wake up duration
 * @param  Duration wake up detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Wake_Up_Duration(uint8_t Duration)
{
  /* Set wake up duration. */
  if (lsm6dsox_wkup_dur_set(&reg_ctx, Duration) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable single tap detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_Single_Tap_Detection(LSM6DSOX_SensorIntPin_t IntPin)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap threshold. */
  if (lsm6dsox_tap_threshold_x_set(&reg_ctx, 0x08) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap shock time window. */
  if (lsm6dsox_tap_shock_set(&reg_ctx, 0x02) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap quiet time window. */
  if (lsm6dsox_tap_quiet_set(&reg_ctx, 0x01) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSOX_INT1_PIN:
      if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val1.single_tap = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    case LSM6DSOX_INT2_PIN:
      if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val2.single_tap = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable single tap detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_Single_Tap_Detection()
{
	lsm6dsox_pin_int1_route_t val1;
	lsm6dsox_pin_int2_route_t val2;

  /* Disable single tap event on both INT1 and INT2 pins */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.single_tap = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val2.single_tap = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap quiet time window. */
  if (lsm6dsox_tap_quiet_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap shock time window. */
  if (lsm6dsox_tap_shock_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap threshold. */
  if (lsm6dsox_tap_threshold_x_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable double tap detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_Double_Tap_Detection(LSM6DSOX_SensorIntPin_t IntPin)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap threshold. */
  if (lsm6dsox_tap_threshold_x_set(&reg_ctx, 0x08) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap shock time window. */
  if (lsm6dsox_tap_shock_set(&reg_ctx, 0x03) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap quiet time window. */
  if (lsm6dsox_tap_quiet_set(&reg_ctx, 0x03) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Set tap duration time window. */
  if (lsm6dsox_tap_dur_set(&reg_ctx, 0x08) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Single and double tap enabled. */
  if (lsm6dsox_tap_mode_set(&reg_ctx, LSM6DSOX_BOTH_SINGLE_DOUBLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSOX_INT1_PIN:
      if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val1.double_tap = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    case LSM6DSOX_INT2_PIN:
      if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val2.double_tap = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable double tap detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_Double_Tap_Detection()
{
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Disable double tap event on both INT1 and INT2 pins */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.double_tap = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val2.double_tap = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Only single tap enabled. */
  if (lsm6dsox_tap_mode_set(&reg_ctx, LSM6DSOX_ONLY_SINGLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap duration time window. */
  if (lsm6dsox_tap_dur_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap quiet time window. */
  if (lsm6dsox_tap_quiet_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap shock time window. */
  if (lsm6dsox_tap_shock_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset tap threshold. */
  if (lsm6dsox_tap_threshold_x_set(&reg_ctx, 0x00) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (lsm6dsox_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set tap threshold
 * @param  Threshold tap threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Tap_Threshold(uint8_t Threshold)
{
  /* Set tap threshold. */
  if (lsm6dsox_tap_threshold_x_set(&reg_ctx, Threshold) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set tap shock time
 * @param  Time tap shock time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Tap_Shock_Time(uint8_t Time)
{
  /* Set tap shock time window. */
  if (lsm6dsox_tap_shock_set(&reg_ctx, Time) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set tap quiet time
 * @param  Time tap quiet time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Tap_Quiet_Time(uint8_t Time)
{
  /* Set tap quiet time window. */
  if (lsm6dsox_tap_quiet_set(&reg_ctx, Time) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set tap duration time
 * @param  Time tap duration time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Tap_Duration_Time(uint8_t Time)
{
  /* Set tap duration time window. */
  if (lsm6dsox_tap_dur_set(&reg_ctx, Time) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Enable 6D orientation detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Enable_6D_Orientation(LSM6DSOX_SensorIntPin_t IntPin)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* 6D orientation enabled. */
  if (lsm6dsox_6d_threshold_set(&reg_ctx, LSM6DSOX_DEG_60) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSOX_INT1_PIN:
      if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val1.six_d = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    case LSM6DSOX_INT2_PIN:
      if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }

      val2.six_d = PROPERTY_ENABLE;

      if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
      {
        return LSM6DSOX_ERROR;
      }
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable 6D orientation detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Disable_6D_Orientation()
{
  lsm6dsox_pin_int1_route_t val1;
  lsm6dsox_pin_int2_route_t val2;

  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (lsm6dsox_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val1.six_d = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int1_route_set(&reg_ctx, val1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_pin_int2_route_get(&reg_ctx, NULL, &val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val2.six_d = PROPERTY_DISABLE;

  if (lsm6dsox_pin_int2_route_set(&reg_ctx, NULL, val2) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  /* Reset 6D orientation. */
  if (lsm6dsox_6d_threshold_set(&reg_ctx, LSM6DSOX_DEG_80) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set 6D orientation threshold
 * @param  Threshold 6D Orientation detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_6D_Orientation_Threshold(uint8_t Threshold)
{
  if (lsm6dsox_6d_threshold_set(&reg_ctx, (lsm6dsox_sixd_ths_t)Threshold) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of XLow orientation
 * @param  XLow the status of XLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_6D_Orientation_XL(uint8_t *XLow)
{
  lsm6dsox_d6d_src_t data;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *XLow = data.xl;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of XHigh orientation
 * @param  XHigh the status of XHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_6D_Orientation_XH(uint8_t *XHigh)
{
  lsm6dsox_d6d_src_t data;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *XHigh = data.xh;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of YLow orientation
 * @param  YLow the status of YLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_6D_Orientation_YL(uint8_t *YLow)
{
  lsm6dsox_d6d_src_t data;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *YLow = data.yl;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of YHigh orientation
 * @param  YHigh the status of YHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_6D_Orientation_YH(uint8_t *YHigh)
{
  lsm6dsox_d6d_src_t data;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *YHigh = data.yh;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of ZLow orientation
 * @param  ZLow the status of ZLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_6D_Orientation_ZL(uint8_t *ZLow)
{
  lsm6dsox_d6d_src_t data;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *ZLow = data.zl;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of ZHigh orientation
 * @param  ZHigh the status of ZHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_6D_Orientation_ZH(uint8_t *ZHigh)
{
  lsm6dsox_d6d_src_t data;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *ZHigh = data.zh;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX ACC data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  if (lsm6dsox_xl_flag_data_ready_get(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the status of all hardware events
 * @param  pObj the device pObj
 * @param  Status the status of all hardware events
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_X_Event_Status(LSM6DSOX_Event_Status_t *Status)
{
  uint8_t tilt_ia = 0U;
  lsm6dsox_wake_up_src_t wake_up_src;
  lsm6dsox_tap_src_t tap_src;
  lsm6dsox_d6d_src_t d6d_src;
  lsm6dsox_emb_func_src_t func_src;
  lsm6dsox_md1_cfg_t md1_cfg;
  lsm6dsox_md2_cfg_t md2_cfg;
  lsm6dsox_emb_func_int1_t int1_ctrl;
  lsm6dsox_emb_func_int2_t int2_ctrl;

  (void)memset((void *)Status, 0x0, sizeof(LSM6DSOX_Event_Status_t));

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_TAP_SRC, (uint8_t *)&tap_src, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_D6D_SRC, (uint8_t *)&d6d_src, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_mem_bank_set(&reg_ctx, LSM6DSOX_EMBEDDED_FUNC_BANK) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_EMB_FUNC_SRC, (uint8_t *)&func_src, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_EMB_FUNC_INT1, (uint8_t *)&int1_ctrl, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_EMB_FUNC_INT2, (uint8_t *)&int2_ctrl, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_mem_bank_set(&reg_ctx, LSM6DSOX_USER_BANK) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_MD1_CFG, (uint8_t *)&md1_cfg, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_MD2_CFG, (uint8_t *)&md2_cfg, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if (lsm6dsox_tilt_flag_data_ready_get(&reg_ctx, &tilt_ia) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U))
  {
    if (wake_up_src.ff_ia == 1U)
    {
      Status->FreeFallStatus = 1;
    }
  }

  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U))
  {
    if (wake_up_src.wu_ia == 1U)
    {
      Status->WakeUpStatus = 1;
    }
  }

  if ((md1_cfg.int1_single_tap == 1U) || (md2_cfg.int2_single_tap == 1U))
  {
    if (tap_src.single_tap == 1U)
    {
      Status->TapStatus = 1;
    }
  }

  if ((md1_cfg.int1_double_tap == 1U) || (md2_cfg.int2_double_tap == 1U))
  {
    if (tap_src.double_tap == 1U)
    {
      Status->DoubleTapStatus = 1;
    }
  }

  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U))
  {
    if (d6d_src.d6d_ia == 1U)
    {
      Status->D6DOrientationStatus = 1;
    }
  }

  if (int1_ctrl.int1_step_detector == 1U)
  {
    if (func_src.step_detected == 1U)
    {
      Status->StepStatus = 1;
    }
  }

  if ((int1_ctrl.int1_tilt == 1U) || (int2_ctrl.int2_tilt == 1U))
  {
    if (tilt_ia == 1U)
    {
      Status->TiltStatus = 1;
    }
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set self test
 * @param  val the value of st_xl in reg CTRL5_C
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_X_SelfTest(uint8_t val)
{
  lsm6dsox_st_xl_t reg;

  reg = (val == 0U)  ? LSM6DSOX_XL_ST_DISABLE
      : (val == 1U)  ? LSM6DSOX_XL_ST_POSITIVE
      : (val == 2U)  ? LSM6DSOX_XL_ST_NEGATIVE
      :                LSM6DSOX_XL_ST_DISABLE;

  if (lsm6dsox_xl_self_test_set(&reg_ctx, reg) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX GYRO data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  if (lsm6dsox_gy_flag_data_ready_get(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set self test
 * @param  val the value of st_xl in reg CTRL5_C
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_G_SelfTest(uint8_t val)
{
  lsm6dsox_st_g_t reg;

  reg = (val == 0U)  ? LSM6DSOX_GY_ST_DISABLE
      : (val == 1U)  ? LSM6DSOX_GY_ST_POSITIVE
      : (val == 2U)  ? LSM6DSOX_GY_ST_NEGATIVE
      :                LSM6DSOX_GY_ST_DISABLE;


  if (lsm6dsox_gy_self_test_set(&reg_ctx, reg) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO number of samples
 * @param  NumSamples number of samples
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Num_Samples(uint16_t *NumSamples)
{
  if (lsm6dsox_fifo_data_level_get(&reg_ctx, NumSamples) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO full status
 * @param  Status FIFO full status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Full_Status(uint8_t *Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_FIFO_STATUS2, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *Status = reg.fifo_status2.fifo_full_ia;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO overrun status
 * @param  Status FIFO overrun status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Overrun_Status(uint8_t *Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_FIFO_STATUS2, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *Status = reg.fifo_status2.fifo_ovr_ia;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO watermark status
 * @param  Status FIFO watermark status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Watermark_Status(uint8_t *Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_FIFO_STATUS2, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  *Status = reg.fifo_status2.fifo_wtm_ia;

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO full interrupt on INT1 pin
 * @param  Status FIFO full interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_INT1_FIFO_Full(uint8_t Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_INT1_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  reg.int1_ctrl.int1_fifo_full = Status;

  if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_INT1_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO overrun interrupt on INT1 pin
 * @param  Status FIFO overrun interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_INT1_FIFO_Overrun(uint8_t Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_INT1_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  reg.int1_ctrl.int1_fifo_ovr = Status;

  if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_INT1_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO threshold interrupt on INT1 pin
 * @param  Status FIFO threshold interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_INT1_FIFO_Threshold(uint8_t Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_INT1_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  reg.int1_ctrl.int1_fifo_th = Status;

  if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_INT1_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO full interrupt on INT2 pin
 * @param  Status FIFO full interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_INT2_FIFO_Full(uint8_t Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_INT2_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  reg.int2_ctrl.int2_fifo_full = Status;

  if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_INT2_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO overrun interrupt on INT2 pin
 * @param  Status FIFO overrun interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_INT2_FIFO_Overrun(uint8_t Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_INT2_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  reg.int2_ctrl.int2_fifo_ovr = Status;

  if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_INT2_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO threshold interrupt on INT2 pin
 * @param  Status FIFO threshold interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_INT2_FIFO_Threshold(uint8_t Status)
{
  lsm6dsox_reg_t reg;

  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_INT2_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  reg.int2_ctrl.int2_fifo_th = Status;

  if (lsm6dsox_write_reg(&reg_ctx, LSM6DSOX_INT2_CTRL, &reg.byte, 1) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO watermark level
 * @param  Watermark FIFO watermark level
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Watermark_Level(uint16_t Watermark)
{
  if (lsm6dsox_fifo_watermark_set(&reg_ctx, Watermark) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO stop on watermark
 * @param  Status FIFO stop on watermark status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Stop_On_Fth(uint8_t Status)
{
  if (lsm6dsox_fifo_stop_on_wtm_set(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO mode
 * @param  Mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Mode(uint8_t Mode)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dsox_fifo_mode_t)Mode)
  {
    case LSM6DSOX_BYPASS_MODE:
    case LSM6DSOX_FIFO_MODE:
    case LSM6DSOX_STREAM_TO_FIFO_MODE:
    case LSM6DSOX_BYPASS_TO_STREAM_MODE:
    case LSM6DSOX_STREAM_MODE:
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  if (ret == LSM6DSOX_ERROR)
  {
    return ret;
  }

  if (lsm6dsox_fifo_mode_set(&reg_ctx, (lsm6dsox_fifo_mode_t)Mode) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSOX FIFO tag
 * @param  Tag FIFO tag
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Tag(uint8_t *Tag)
{
  lsm6dsox_fifo_tag_t tag_local;

	if (lsm6dsox_fifo_sensor_tag_get(&reg_ctx, &tag_local) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

	*Tag = (uint8_t)tag_local;

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO raw data
 * @param  Data FIFO raw data array [6]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Data(uint8_t *Data)
{
  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_FIFO_DATA_OUT_X_L, Data, 6) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO sample
 * @param  Sample FIFO sample array [multiple of 7]
 * @param  Count Count of samples to get.
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_Sample(uint8_t *Sample, uint16_t Count)
{
  if (lsm6dsox_read_reg(&reg_ctx, LSM6DSOX_FIFO_DATA_OUT_TAG, Sample, Count * 7) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO accelero single sample (16-bit data per 3 axes) and calculate acceleration [mg]
 * @param  Acceleration FIFO accelero axes [mg]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_X_Axes(int32_t *Acceleration)
{
  uint8_t data[6];
  int16_t data_raw[3];
  float sensitivity = 0.0f;
  float acceleration_float[3];

  if (Get_FIFO_Data(data) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  if (Get_X_Sensitivity(&sensitivity) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  acceleration_float[0] = (float)data_raw[0] * sensitivity;
  acceleration_float[1] = (float)data_raw[1] * sensitivity;
  acceleration_float[2] = (float)data_raw[2] * sensitivity;

  Acceleration[0] = (int32_t)acceleration_float[0];
  Acceleration[1] = (int32_t)acceleration_float[1];
  Acceleration[2] = (int32_t)acceleration_float[2];

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO accelero BDR value
 * @param  Bdr FIFO accelero BDR value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_X_BDR(float Bdr)
{
  lsm6dsox_bdr_xl_t new_bdr;

  new_bdr = (Bdr <=    0.0f) ? LSM6DSOX_XL_NOT_BATCHED
            : (Bdr <=   12.5f) ? LSM6DSOX_XL_BATCHED_AT_12Hz5
            : (Bdr <=   26.0f) ? LSM6DSOX_XL_BATCHED_AT_26Hz
            : (Bdr <=   52.0f) ? LSM6DSOX_XL_BATCHED_AT_52Hz
            : (Bdr <=  104.0f) ? LSM6DSOX_XL_BATCHED_AT_104Hz
            : (Bdr <=  208.0f) ? LSM6DSOX_XL_BATCHED_AT_208Hz
            : (Bdr <=  416.0f) ? LSM6DSOX_XL_BATCHED_AT_417Hz
            : (Bdr <=  833.0f) ? LSM6DSOX_XL_BATCHED_AT_833Hz
            : (Bdr <= 1660.0f) ? LSM6DSOX_XL_BATCHED_AT_1667Hz
            : (Bdr <= 3330.0f) ? LSM6DSOX_XL_BATCHED_AT_3333Hz
            :                    LSM6DSOX_XL_BATCHED_AT_6667Hz;

  if (lsm6dsox_fifo_xl_batch_set(&reg_ctx, new_bdr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX FIFO gyro single sample (16-bit data per 3 axes) and calculate angular velocity [mDPS]
 * @param  AngularVelocity FIFO gyro axes [mDPS]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_FIFO_G_Axes(int32_t *AngularVelocity)
{
  uint8_t data[6];
  int16_t data_raw[3];
  float sensitivity = 0.0f;
  float angular_velocity_float[3];

  if (Get_FIFO_Data(data) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  if (Get_G_Sensitivity(&sensitivity) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  angular_velocity_float[0] = (float)data_raw[0] * sensitivity;
  angular_velocity_float[1] = (float)data_raw[1] * sensitivity;
  angular_velocity_float[2] = (float)data_raw[2] * sensitivity;

  AngularVelocity[0] = (int32_t)angular_velocity_float[0];
  AngularVelocity[1] = (int32_t)angular_velocity_float[1];
  AngularVelocity[2] = (int32_t)angular_velocity_float[2];

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO gyro BDR value
 * @param  Bdr FIFO gyro BDR value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_G_BDR(float Bdr)
{
  lsm6dsox_bdr_gy_t new_bdr;

  new_bdr = (Bdr <=    0.0f) ? LSM6DSOX_GY_NOT_BATCHED
            : (Bdr <=   12.5f) ? LSM6DSOX_GY_BATCHED_AT_12Hz5
            : (Bdr <=   26.0f) ? LSM6DSOX_GY_BATCHED_AT_26Hz
            : (Bdr <=   52.0f) ? LSM6DSOX_GY_BATCHED_AT_52Hz
            : (Bdr <=  104.0f) ? LSM6DSOX_GY_BATCHED_AT_104Hz
            : (Bdr <=  208.0f) ? LSM6DSOX_GY_BATCHED_AT_208Hz
            : (Bdr <=  416.0f) ? LSM6DSOX_GY_BATCHED_AT_417Hz
            : (Bdr <=  833.0f) ? LSM6DSOX_GY_BATCHED_AT_833Hz
            : (Bdr <= 1660.0f) ? LSM6DSOX_GY_BATCHED_AT_1667Hz
            : (Bdr <= 3330.0f) ? LSM6DSOX_GY_BATCHED_AT_3333Hz
            :                    LSM6DSOX_GY_BATCHED_AT_6667Hz;

  if (lsm6dsox_fifo_gy_batch_set(&reg_ctx, new_bdr) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_MLC_Status(LSM6DSOX_MLC_Status_t *Status)
{
  if (lsm6dsox_mlc_status_get(&reg_ctx, (lsm6dsox_mlc_status_mainpage_t *)Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_MLC_Output(uint8_t *Output)
{
  if (lsm6dsox_mlc_out_get(&reg_ctx, Output) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Get the LSM6DSOX timestamp enable status
 * @param  Status Timestamp enable status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Get_Timestamp_Status(uint8_t *Status)
{
  if (lsm6dsox_timestamp_get(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX timestamp enable status
 * @param  Status Timestamp enable status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_Timestamp_Status(uint8_t Status)
{
  if (lsm6dsox_timestamp_set(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO timestamp decimation
 * @param  Decimation FIFO timestamp decimation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Timestamp_Decimation(uint8_t Decimation)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dsox_odr_ts_batch_t)Decimation)
  {
    case LSM6DSOX_NO_DECIMATION:
    case LSM6DSOX_DEC_1:
    case LSM6DSOX_DEC_8:
    case LSM6DSOX_DEC_32:
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }

  if (ret == LSM6DSOX_ERROR)
  {
    return ret;
  }

  if (lsm6dsox_fifo_timestamp_decimation_set(&reg_ctx, (lsm6dsox_odr_ts_batch_t)Decimation) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX FIFO compression initialization status
 * @param  Status FIFO compression initialization status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Compression_Algo_Init(uint8_t Status)
{
  if (lsm6dsox_compression_algo_init_set(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO compression enable status
 * @param  Status FIFO compression enable status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Compression_Algo_Enable(uint8_t Status)
{
  lsm6dsox_emb_sens_t val;
  if (lsm6dsox_embedded_sens_get(&reg_ctx, &val) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  val.fifo_compr = Status;

  if (lsm6dsox_embedded_sens_set(&reg_ctx, &val) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

/**
 * @brief  Set the LSM6DSOX FIFO compression configuration and enable status
 * @param  Compression FIFO compression and enable status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Compression_Algo_Set(uint8_t Compression)
{
  LSM6DSOXStatusTypeDef ret = LSM6DSOX_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dsox_uncoptr_rate_t)Compression)
  {
    case LSM6DSOX_CMP_DISABLE:
    case LSM6DSOX_CMP_ALWAYS:
    case LSM6DSOX_CMP_8_TO_1:
    case LSM6DSOX_CMP_16_TO_1:
    case LSM6DSOX_CMP_32_TO_1:
      break;

    default:
      ret = LSM6DSOX_ERROR;
      break;
  }


  if (ret == LSM6DSOX_ERROR)
  {
    return ret;
  }

  if (lsm6dsox_compression_algo_set(&reg_ctx, (lsm6dsox_uncoptr_rate_t)Compression) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSOX FIFO compression real time enable status
 * @param  Status FIFO compression real time enable status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSOXStatusTypeDef LSM6DSOXSensor::Set_FIFO_Compression_Algo_Real_Time_Set(uint8_t Status)
{
  if (lsm6dsox_compression_algo_real_time_set(&reg_ctx, Status) != LSM6DSOX_OK)
  {
    return LSM6DSOX_ERROR;
  }

  return LSM6DSOX_OK;
}

int32_t LSM6DSOX_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LSM6DSOXSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LSM6DSOX_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LSM6DSOXSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
