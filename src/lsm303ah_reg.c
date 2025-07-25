/**
  ******************************************************************************
  * @file    lsm303ah_reg.c
  * @author  Sensors Software Solution Team
  * @brief   LSM303AH driver file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "lsm303ah_reg.h"

/**
  * @defgroup  LSM303AH
  * @brief     This file provides a set of functions needed to drive the
  *            lsm303ah enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  LSM303AH_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak lsm303ah_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                 uint8_t *data,
                                 uint16_t len)
{
  int32_t ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak lsm303ah_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                  uint8_t *data,
                                  uint16_t len)
{
  int32_t ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LSM303AH_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t lsm303ah_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.061f);
}

float_t lsm303ah_from_fs4g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.122f);
}

float_t lsm303ah_from_fs8g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.244f);
}

float_t lsm303ah_from_fs16g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.488f);
}

float_t lsm303ah_from_lsb_to_mgauss(int16_t lsb)
{
  return ((float_t)lsb * 1.5f);
}

float_t lsm303ah_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup  Data Generation
  * @brief     This section groups all the functions concerning data generation.
  * @{
  *
  */

/**
  * @brief  Read all the interrupt/status flag of the device.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get FIFO_SRC, STATUS_DUP, WAKE_UP_SRC,
  *                TAP_SRC, 6D_SRC, FUNC_CK_GATE, FUNC_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_all_sources_get(const stmdev_ctx_t *ctx,
                                    lsm303ah_xl_all_sources_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SRC_A,
                          (uint8_t *) & (val->fifo_src_a), 1);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_STATUS_DUP_A,
                            (uint8_t *) & (val->status_dup_a), 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_SRC_A,
                            (uint8_t *) & (val->wake_up_src_a), 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_SRC_A,
                            (uint8_t *) & (val->tap_src_a), 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_6D_SRC_A,
                            (uint8_t *) & (val->_6d_src_a), 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CK_GATE_A,
                            (uint8_t *) & (val->func_ck_gate_a), 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_SRC_A,
                            (uint8_t *) & (val->func_src_a), 1);
  }

  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of bdu in reg CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_block_data_update_set(const stmdev_ctx_t *ctx,
                                          uint8_t val)
{
  lsm303ah_ctrl1_a_t ctrl1_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);

  if (ret == 0)
  {
    ctrl1_a.bdu = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);
  }

  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get the values of bdu in reg CTRL1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_block_data_update_get(const stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  lsm303ah_ctrl1_a_t ctrl1_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);
  *val = ctrl1_a.bdu;

  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param val     Change the values of bdu in reg CFG_REG_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_block_data_update_set(const stmdev_ctx_t *ctx,
                                          uint8_t val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  if (ret == 0)
  {
    cfg_reg_c_m.bdu = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_C_M,
                             (uint8_t *)&cfg_reg_c_m, 1);
  }

  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get the values of bdu in reg CFG_REG_C.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_block_data_update_get(const stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.bdu;

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of ble in reg CFG_REG_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_data_format_set(const stmdev_ctx_t *ctx,
                                    lsm303ah_mg_ble_t val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  if (ret == 0)
  {
    cfg_reg_c_m.ble = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_C_M,
                             (uint8_t *)&cfg_reg_c_m, 1);
  }

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of ble in reg CFG_REG_C.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_data_format_get(const stmdev_ctx_t *ctx,
                                    lsm303ah_mg_ble_t *val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  switch (cfg_reg_c_m.ble)
  {
    case LSM303AH_MG_LSB_AT_LOW_ADD:
      *val = LSM303AH_MG_LSB_AT_LOW_ADD;
      break;

    case LSM303AH_MG_MSB_AT_LOW_ADD:
      *val = LSM303AH_MG_MSB_AT_LOW_ADD;
      break;

    default:
      *val = LSM303AH_MG_LSB_AT_LOW_ADD;
      break;
  }

  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of fs in reg CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_full_scale_set(const stmdev_ctx_t *ctx,
                                   lsm303ah_xl_fs_t val)
{
  lsm303ah_ctrl1_a_t ctrl1_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);

  if (ret == 0)
  {
    ctrl1_a.fs = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);
  }

  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fs in reg CTRL1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_full_scale_get(const stmdev_ctx_t *ctx,
                                   lsm303ah_xl_fs_t *val)
{
  lsm303ah_ctrl1_a_t ctrl1_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);

  switch (ctrl1_a.fs)
  {
    case LSM303AH_XL_2g:
      *val = LSM303AH_XL_2g;
      break;

    case LSM303AH_XL_16g:
      *val = LSM303AH_XL_16g;
      break;

    case LSM303AH_XL_4g:
      *val = LSM303AH_XL_4g;
      break;

    case LSM303AH_XL_8g:
      *val = LSM303AH_XL_8g;
      break;

    default:
      *val = LSM303AH_XL_2g;
      break;
  }

  return ret;
}

/**
  * @brief  Accelerometer data rate selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of odr in reg CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_data_rate_set(const stmdev_ctx_t *ctx,
                                  lsm303ah_xl_odr_t val)
{
  lsm303ah_ctrl1_a_t ctrl1_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);

  if (ret == 0)
  {
    ctrl1_a.odr = (uint8_t)val & 0x0FU;
    ctrl1_a.hf_odr = ((uint8_t)val & 0x10U) >> 4;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);
  }

  return ret;
}

/**
  * @brief  Accelerometer data rate selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of odr in reg CTRL1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_data_rate_get(const stmdev_ctx_t *ctx,
                                  lsm303ah_xl_odr_t *val)
{
  lsm303ah_ctrl1_a_t ctrl1_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL1_A, (uint8_t *)&ctrl1_a, 1);

  switch ((ctrl1_a.hf_odr << 4) + ctrl1_a.odr)
  {
    case LSM303AH_XL_ODR_OFF:
      *val = LSM303AH_XL_ODR_OFF;
      break;

    case LSM303AH_XL_ODR_1Hz_LP:
      *val = LSM303AH_XL_ODR_1Hz_LP;
      break;

    case LSM303AH_XL_ODR_12Hz5_LP:
      *val = LSM303AH_XL_ODR_12Hz5_LP;
      break;

    case LSM303AH_XL_ODR_25Hz_LP:
      *val = LSM303AH_XL_ODR_25Hz_LP;
      break;

    case LSM303AH_XL_ODR_50Hz_LP:
      *val = LSM303AH_XL_ODR_50Hz_LP;
      break;

    case LSM303AH_XL_ODR_100Hz_LP:
      *val = LSM303AH_XL_ODR_100Hz_LP;
      break;

    case LSM303AH_XL_ODR_200Hz_LP:
      *val = LSM303AH_XL_ODR_200Hz_LP;
      break;

    case LSM303AH_XL_ODR_400Hz_LP:
      *val = LSM303AH_XL_ODR_400Hz_LP;
      break;

    case LSM303AH_XL_ODR_800Hz_LP:
      *val = LSM303AH_XL_ODR_800Hz_LP;
      break;

    case LSM303AH_XL_ODR_12Hz5_HR:
      *val = LSM303AH_XL_ODR_12Hz5_HR;
      break;

    case LSM303AH_XL_ODR_25Hz_HR:
      *val = LSM303AH_XL_ODR_25Hz_HR;
      break;

    case LSM303AH_XL_ODR_50Hz_HR:
      *val = LSM303AH_XL_ODR_50Hz_HR;
      break;

    case LSM303AH_XL_ODR_100Hz_HR:
      *val = LSM303AH_XL_ODR_100Hz_HR;
      break;

    case LSM303AH_XL_ODR_200Hz_HR:
      *val = LSM303AH_XL_ODR_200Hz_HR;
      break;

    case LSM303AH_XL_ODR_400Hz_HR:
      *val = LSM303AH_XL_ODR_400Hz_HR;
      break;

    case LSM303AH_XL_ODR_800Hz_HR:
      *val = LSM303AH_XL_ODR_800Hz_HR;
      break;

    case LSM303AH_XL_ODR_1k6Hz_HF:
      *val = LSM303AH_XL_ODR_1k6Hz_HF;
      break;

    case LSM303AH_XL_ODR_3k2Hz_HF:
      *val = LSM303AH_XL_ODR_3k2Hz_HF;
      break;

    case LSM303AH_XL_ODR_6k4Hz_HF:
      *val = LSM303AH_XL_ODR_6k4Hz_HF;
      break;

    default:
      *val = LSM303AH_XL_ODR_OFF;
      break;
  }

  return ret;
}

/**
  * @brief  The STATUS_REG register.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_status_reg_get(const stmdev_ctx_t *ctx,
                                   lsm303ah_status_a_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STATUS_A, (uint8_t *) val, 1);

  return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_status_get(const stmdev_ctx_t *ctx,
                               lsm303ah_status_reg_m_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STATUS_REG_M, (uint8_t *) val, 1);

  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of drdy in reg STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  lsm303ah_status_a_t status_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STATUS_A, (uint8_t *)&status_a, 1);
  *val = status_a.drdy;

  return ret;
}

/**
  * @brief  Magnetic set of data available.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of zyxda in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_status_reg_m_t status_reg_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STATUS_REG_M,
                          (uint8_t *)&status_reg_m, 1);
  *val = status_reg_m.zyxda;

  return ret;
}

/**
  * @brief  Magnetic set of data overrun.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of zyxor in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_data_ovr_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_status_reg_m_t status_reg_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STATUS_REG_M,
                          (uint8_t *)&status_reg_m, 1);
  *val = status_reg_m.zyxor;

  return ret;
}

/**
  * @brief  These registers comprise a 3 group of 16-bit number and represent
  *         hard-iron offset in order to compensate environmental effects. Data
  *         format is the same of output data raw: two's complement with
  *         1LSb = 1.5mG. These values act on the magnetic output data value in
  *         order to delete the environmental offset.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that contains data to write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_user_offset_set(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  buff[1] = (uint8_t)((uint16_t)val[0] / 256U);
  buff[0] = (uint8_t)((uint16_t)val[0] - (buff[1] * 256U));
  buff[3] = (uint8_t)((uint16_t)val[1] / 256U);
  buff[2] = (uint8_t)((uint16_t)val[1] - (buff[3] * 256U));
  buff[5] = (uint8_t)((uint16_t)val[2] / 256U);
  buff[4] = (uint8_t)((uint16_t)val[2] - (buff[5] * 256U));
  ret = lsm303ah_write_reg(ctx, LSM303AH_OFFSET_X_REG_L_M, buff, 6);

  return ret;
}

/**
  * @brief  These registers comprise a 3 group of 16-bit number and represent
  *         hard-iron offset in order to compensate environmental effects. Data
  *         format is the same of output data raw: two's complement with
  *         1LSb = 1.5mG. These values act on the magnetic output data value in
  *         order to delete the environmental offset.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_user_offset_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_OFFSET_X_REG_L_M, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Operating mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of md in reg CFG_REG_A
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_operating_mode_set(const stmdev_ctx_t *ctx,
                                       lsm303ah_mg_md_t val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  if (ret == 0)
  {
    cfg_reg_a_m.md = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_A_M,
                             (uint8_t *)&cfg_reg_a_m, 1);
  }

  return ret;
}

/**
  * @brief  Operating mode selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of md in reg CFG_REG_A.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_operating_mode_get(const stmdev_ctx_t *ctx,
                                       lsm303ah_mg_md_t *val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  switch (cfg_reg_a_m.md)
  {
    case LSM303AH_MG_CONTINUOUS_MODE:
      *val = LSM303AH_MG_CONTINUOUS_MODE;
      break;

    case LSM303AH_MG_SINGLE_TRIGGER:
      *val = LSM303AH_MG_SINGLE_TRIGGER;
      break;

    case LSM303AH_MG_POWER_DOWN:
      *val = LSM303AH_MG_POWER_DOWN;
      break;

    default:
      *val = LSM303AH_MG_CONTINUOUS_MODE;
      break;
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of odr in reg CFG_REG_A
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_data_rate_set(const stmdev_ctx_t *ctx,
                                  lsm303ah_mg_odr_t val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  if (ret == 0)
  {
    cfg_reg_a_m.odr = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_A_M,
                             (uint8_t *)&cfg_reg_a_m, 1);
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of odr in reg CFG_REG_A.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_data_rate_get(const stmdev_ctx_t *ctx,
                                  lsm303ah_mg_odr_t *val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  switch (cfg_reg_a_m.odr)
  {
    case LSM303AH_MG_ODR_10Hz:
      *val = LSM303AH_MG_ODR_10Hz;
      break;

    case LSM303AH_MG_ODR_20Hz:
      *val = LSM303AH_MG_ODR_20Hz;
      break;

    case LSM303AH_MG_ODR_50Hz:
      *val = LSM303AH_MG_ODR_50Hz;
      break;

    case LSM303AH_MG_ODR_100Hz:
      *val = LSM303AH_MG_ODR_100Hz;
      break;

    default:
      *val = LSM303AH_MG_ODR_10Hz;
      break;
  }

  return ret;
}

/**
  * @brief  Enables high-resolution/low-power mode.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of lp in reg CFG_REG_A
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_power_mode_set(const stmdev_ctx_t *ctx,
                                   lsm303ah_mg_lp_t val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  if (ret == 0)
  {
    cfg_reg_a_m.lp = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_A_M,
                             (uint8_t *)&cfg_reg_a_m, 1);
  }

  return ret;
}

/**
  * @brief  Enables high-resolution/low-power mode.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of lp in reg CFG_REG_A.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_power_mode_get(const stmdev_ctx_t *ctx,
                                   lsm303ah_mg_lp_t *val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  switch (cfg_reg_a_m.lp)
  {
    case LSM303AH_MG_HIGH_RESOLUTION:
      *val = LSM303AH_MG_HIGH_RESOLUTION;
      break;

    case LSM303AH_MG_LOW_POWER:
      *val = LSM303AH_MG_LOW_POWER;
      break;

    default:
      *val = LSM303AH_MG_HIGH_RESOLUTION;
      break;
  }

  return ret;
}

/**
  * @brief  Enables the magnetometer temperature compensation.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of comp_temp_en in reg CFG_REG_A
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_offset_temp_comp_set(const stmdev_ctx_t *ctx,
                                         uint8_t val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  if (ret == 0)
  {
    cfg_reg_a_m.comp_temp_en = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_A_M,
                             (uint8_t *)&cfg_reg_a_m, 1);
  }

  return ret;
}

/**
  * @brief  Enables the magnetometer temperature compensation.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of comp_temp_en in reg CFG_REG_A.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_offset_temp_comp_get(const stmdev_ctx_t *ctx,
                                         uint8_t *val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);
  *val = cfg_reg_a_m.comp_temp_en;

  return ret;
}

/**
  * @brief  Set/Reset mode.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of set_rst in reg CFG_REG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_set_rst_mode_set(const stmdev_ctx_t *ctx,
                                     lsm303ah_mg_set_rst_t val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  if (ret == 0)
  {
    cfg_reg_b_m.set_rst = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_B_M,
                             (uint8_t *)&cfg_reg_b_m, 1);
  }

  return ret;
}

/**
  * @brief  Set/Reset mode.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of set_rst in reg CFG_REG_B.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_set_rst_mode_get(const stmdev_ctx_t *ctx,
                                     lsm303ah_mg_set_rst_t *val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  switch (cfg_reg_b_m.set_rst)
  {
    case LSM303AH_MG_SET_SENS_ODR_DIV_63:
      *val = LSM303AH_MG_SET_SENS_ODR_DIV_63;
      break;

    case LSM303AH_MG_SENS_OFF_CANC_EVERY_ODR:
      *val = LSM303AH_MG_SENS_OFF_CANC_EVERY_ODR;
      break;

    case LSM303AH_MG_SET_SENS_ONLY_AT_POWER_ON:
      *val = LSM303AH_MG_SET_SENS_ONLY_AT_POWER_ON;
      break;

    default:
      *val = LSM303AH_MG_SET_SENS_ODR_DIV_63;
      break;
  }

  return ret;
}

/**
  * @brief   Enables offset cancellation in single measurement mode. The
  *          OFF_CANC bit must be set to 1 when enabling offset cancellation
  *          in single measurement mode this means a call function:
  *          set_rst_mode(SENS_OFF_CANC_EVERY_ODR) is need.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of off_canc_one_shot in reg CFG_REG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_set_rst_sensor_single_set(const stmdev_ctx_t *ctx,
                                              uint8_t val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  if (ret == 0)
  {
    cfg_reg_b_m.off_canc_one_shot = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_B_M,
                             (uint8_t *)&cfg_reg_b_m, 1);
  }

  return ret;
}

/**
  * @brief   Enables offset cancellation in single measurement mode. The
  *          OFF_CANC bit must be set to 1 when enabling offset cancellation
  *          in single measurement mode this means a call function:
  *          set_rst_mode(SENS_OFF_CANC_EVERY_ODR) is need.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of off_canc_one_shot in reg CFG_REG_B.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_set_rst_sensor_single_get(const stmdev_ctx_t *ctx,
                                              uint8_t *val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);
  *val = cfg_reg_b_m.off_canc_one_shot;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Dataoutput
  * @brief   This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Module output value (8-bit).[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_acceleration_module_raw_get(const stmdev_ctx_t *ctx,
                                             uint8_t *buff)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_MODULE_8BIT_A, buff, 1);

  return ret;
}

/**
  * @brief  Temperature data output register (r). L and H registers together
  *         express a 16-bit word in two's complement.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_temperature_raw_get(const stmdev_ctx_t *ctx,
                                        uint8_t *buff)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_OUT_T_A, buff, 1);

  return ret;
}

/**
  * @brief  Linear acceleration output register.
  *         The value is expressed as a 16-bit word in two's complement.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_OUT_X_L_A, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Magnetic output value.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_magnetic_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_OUTX_L_REG_M, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Number of steps detected by step counter routine.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_number_of_steps_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_L_A, buff, 2);
  *val = buff[1];
  *val = (*val * 256U) +  buff[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  common
  * @brief   This section groups common useful functions.
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WHO_AM_I_A, buff, 1);

  return ret;
}

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WHO_AM_I_M, buff, 1);

  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of if_add_inc in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_auto_increment_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  if (ret == 0)
  {
    ctrl2_a.if_add_inc = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  }

  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of if_add_inc in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_auto_increment_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  *val = ctrl2_a.if_add_inc;

  return ret;
}

/**
  * @brief  Enable access to the embedded functions/sensor
  *         hub configuration registers.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of func_cfg_en in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_mem_bank_set(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_func_cfg_en_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  lsm303ah_ctrl2_adv_a_t ctrl2_adv_a;
  int32_t ret;


  if (val == LSM303AH_XL_ADV_BANK)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

    if (ret == 0)
    {
      ctrl2_a.func_cfg_en = (uint8_t)val;
      ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
    }
  }

  else
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_ADV_A,
                            (uint8_t *)&ctrl2_adv_a, 1);

    if (ret == 0)
    {
      ctrl2_adv_a.func_cfg_en = (uint8_t)val;
      ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_ADV_A,
                               (uint8_t *)&ctrl2_adv_a, 1);
    }
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of soft_reset in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_reset_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  if (ret == 0)
  {
    ctrl2_a.soft_reset = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of soft_reset in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_reset_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  *val = ctrl2_a.soft_reset;

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of soft_rst in reg CFG_REG_A
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_reset_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  if (ret == 0)
  {
    cfg_reg_a_m.soft_rst = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_A_M,
                             (uint8_t *)&cfg_reg_a_m, 1);
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of soft_rst in reg CFG_REG_A.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_reset_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);
  *val = cfg_reg_a_m.soft_rst;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_boot_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  if (ret == 0)
  {
    ctrl2_a.boot = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of boot in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_boot_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  *val = ctrl2_a.boot;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of reboot in reg CFG_REG_A
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_boot_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);

  if (ret == 0)
  {
    cfg_reg_a_m.reboot = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_A_M,
                             (uint8_t *)&cfg_reg_a_m, 1);
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of reboot in reg CFG_REG_A.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_boot_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_A_M,
                          (uint8_t *)&cfg_reg_a_m, 1);
  *val = cfg_reg_a_m.reboot;

  return ret;
}

/**
  * @brief  Accelerometer Self-Test.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of st in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_self_test_set(const stmdev_ctx_t *ctx,
                                  lsm303ah_xl_st_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.st = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Accelerometer Self-Test.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of st in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_self_test_get(const stmdev_ctx_t *ctx,
                                  lsm303ah_xl_st_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  switch (ctrl3_a.st)
  {
    case LSM303AH_XL_ST_DISABLE:
      *val = LSM303AH_XL_ST_DISABLE;
      break;

    case LSM303AH_XL_ST_POSITIVE:
      *val = LSM303AH_XL_ST_POSITIVE;
      break;

    case LSM303AH_XL_ST_NEGATIVE:
      *val = LSM303AH_XL_ST_NEGATIVE;
      break;

    default:
      *val = LSM303AH_XL_ST_DISABLE;
      break;
  }

  return ret;
}

/**
  * @brief  Magnetometer self-test.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of self_test in reg CFG_REG_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_self_test_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  if (ret == 0)
  {
    cfg_reg_c_m.self_test = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_C_M,
                             (uint8_t *)&cfg_reg_c_m, 1);
  }

  return ret;
}

/**
  * @brief  Magnetometer self-test.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of self_test in reg CFG_REG_C.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_self_test_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.self_test;

  return ret;
}

/**
  * @brief  Accelerometer data ready mode.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of drdy_pulsed in reg CTRL5
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_data_ready_mode_set(const stmdev_ctx_t *ctx,
                                        lsm303ah_xl_drdy_pulsed_t val)
{
  lsm303ah_ctrl5_a_t ctrl5_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);

  if (ret == 0)
  {
    ctrl5_a.drdy_pulsed = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);
  }

  return ret;
}

/**
  * @brief  Accelerometer data ready mode.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of drdy_pulsed in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                        lsm303ah_xl_drdy_pulsed_t *val)
{
  lsm303ah_ctrl5_a_t ctrl5_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);

  switch (ctrl5_a.drdy_pulsed)
  {
    case LSM303AH_XL_DRDY_LATCHED:
      *val = LSM303AH_XL_DRDY_LATCHED;
      break;

    case LSM303AH_XL_DRDY_PULSED:
      *val = LSM303AH_XL_DRDY_PULSED;
      break;

    default:
      *val = LSM303AH_XL_DRDY_LATCHED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Filters
  * @brief   This section group all the functions concerning the filters
  *          configuration.
  * @{
  *
  */

/**
  * @brief  High-pass filter data selection on output register and FIFO.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of fds_slope in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_hp_path_set(const stmdev_ctx_t *ctx,
                                lsm303ah_xl_fds_slope_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  if (ret == 0)
  {
    ctrl2_a.fds_slope = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  }

  return ret;
}

/**
  * @brief  High-pass filter data selection on output register and FIFO.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fds_slope in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_hp_path_get(const stmdev_ctx_t *ctx,
                                lsm303ah_xl_fds_slope_t *val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  switch (ctrl2_a.fds_slope)
  {
    case LSM303AH_XL_HP_INTERNAL_ONLY:
      *val = LSM303AH_XL_HP_INTERNAL_ONLY;
      break;

    case LSM303AH_XL_HP_ON_OUTPUTS:
      *val = LSM303AH_XL_HP_ON_OUTPUTS;
      break;

    default:
      *val = LSM303AH_XL_HP_INTERNAL_ONLY;
      break;
  }

  return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of lpf in reg CFG_REG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_low_pass_bandwidth_set(const stmdev_ctx_t *ctx,
                                           lsm303ah_mg_lpf_t val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  if (ret == 0)
  {
    cfg_reg_b_m.lpf = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_B_M,
                             (uint8_t *)&cfg_reg_b_m, 1);
  }

  return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of lpf in reg CFG_REG_B.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_low_pass_bandwidth_get(const stmdev_ctx_t *ctx,
                                           lsm303ah_mg_lpf_t *val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  switch (cfg_reg_b_m.lpf)
  {
    case LSM303AH_MG_ODR_DIV_2:
      *val = LSM303AH_MG_ODR_DIV_2;
      break;

    case LSM303AH_MG_ODR_DIV_4:
      *val = LSM303AH_MG_ODR_DIV_4;
      break;

    default:
      *val = LSM303AH_MG_ODR_DIV_2;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Auxiliary_interface
  * @brief   This section groups all the functions concerning auxiliary
  *          interface.
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of sim in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_spi_mode_set(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_sim_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  if (ret == 0)
  {
    ctrl2_a.sim = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  }

  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_spi_mode_get(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_sim_t *val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  switch (ctrl2_a.sim)
  {
    case LSM303AH_XL_SPI_4_WIRE:
      *val = LSM303AH_XL_SPI_4_WIRE;
      break;

    case LSM303AH_XL_SPI_3_WIRE:
      *val = LSM303AH_XL_SPI_3_WIRE;
      break;

    default:
      *val = LSM303AH_XL_SPI_4_WIRE;
      break;
  }

  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of i2c_disable in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_i2c_interface_set(const stmdev_ctx_t *ctx,
                                      lsm303ah_xl_i2c_disable_t val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  if (ret == 0)
  {
    ctrl2_a.i2c_disable = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);
  }

  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of i2c_disable in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_i2c_interface_get(const stmdev_ctx_t *ctx,
                                      lsm303ah_xl_i2c_disable_t *val)
{
  lsm303ah_ctrl2_a_t ctrl2_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL2_A, (uint8_t *)&ctrl2_a, 1);

  switch (ctrl2_a.i2c_disable)
  {
    case LSM303AH_XL_I2C_ENABLE:
      *val = LSM303AH_XL_I2C_ENABLE;
      break;

    case LSM303AH_XL_I2C_DISABLE:
      *val = LSM303AH_XL_I2C_DISABLE;
      break;

    default:
      *val = LSM303AH_XL_I2C_ENABLE;
      break;
  }

  return ret;
}

/**
  * @brief  Enable/Disable I2C interface.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of i2c_dis in reg CFG_REG_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_i2c_interface_set(const stmdev_ctx_t *ctx,
                                      lsm303ah_mg_i2c_dis_t val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  if (ret == 0)
  {
    cfg_reg_c_m.i2c_dis = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_C_M,
                             (uint8_t *)&cfg_reg_c_m, 1);
  }

  return ret;
}

/**
  * @brief  Enable/Disable I2C interface.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of i2c_dis in reg CFG_REG_C.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_i2c_interface_get(const stmdev_ctx_t *ctx,
                                      lsm303ah_mg_i2c_dis_t *val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  switch (cfg_reg_c_m.i2c_dis)
  {
    case LSM303AH_MG_I2C_ENABLE:
      *val = LSM303AH_MG_I2C_ENABLE;
      break;

    case LSM303AH_MG_I2C_DISABLE:
      *val = LSM303AH_MG_I2C_DISABLE;
      break;

    default:
      *val = LSM303AH_MG_I2C_ENABLE;
      break;
  }

  return ret;
}

/**
  * @brief  Connect/Disconnects pull-up in if_cs pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of if_cs_pu_dis in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_cs_mode_set(const stmdev_ctx_t *ctx,
                                lsm303ah_xl_if_cs_pu_dis_t val)
{
  lsm303ah_fifo_ctrl_a_t fifo_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_CTRL_A,
                          (uint8_t *)&fifo_ctrl_a, 1);

  if (ret == 0)
  {
    fifo_ctrl_a.if_cs_pu_dis = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FIFO_CTRL_A,
                             (uint8_t *)&fifo_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief  Connect/Disconnects pull-up in if_cs pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of if_cs_pu_dis in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_cs_mode_get(const stmdev_ctx_t *ctx,
                                lsm303ah_xl_if_cs_pu_dis_t *val)
{
  lsm303ah_fifo_ctrl_a_t fifo_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_CTRL_A,
                          (uint8_t *)&fifo_ctrl_a, 1);

  switch (fifo_ctrl_a.if_cs_pu_dis)
  {
    case LSM303AH_XL_PULL_UP_CONNECTED:
      *val = LSM303AH_XL_PULL_UP_CONNECTED;
      break;

    case LSM303AH_XL_PULL_UP_DISCONNECTED:
      *val = LSM303AH_XL_PULL_UP_DISCONNECTED;
      break;

    default:
      *val = LSM303AH_XL_PULL_UP_CONNECTED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   main_serial_interface
  * @brief   This section groups all the functions concerning main serial
  *          interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  Push-pull/open-drain selection on interrupt pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of pp_od in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_mode_set(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_pp_od_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.pp_od = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Push-pull/open-drain selection on interrupt pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_mode_get(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_pp_od_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  switch (ctrl3_a.pp_od)
  {
    case LSM303AH_XL_PUSH_PULL:
      *val = LSM303AH_XL_PUSH_PULL;
      break;

    case LSM303AH_XL_OPEN_DRAIN:
      *val = LSM303AH_XL_OPEN_DRAIN;
      break;

    default:
      *val = LSM303AH_XL_PUSH_PULL;
      break;
  }

  return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of h_lactive in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_polarity_set(const stmdev_ctx_t *ctx,
                                     lsm303ah_xl_h_lactive_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.h_lactive = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of h_lactive in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_polarity_get(const stmdev_ctx_t *ctx,
                                     lsm303ah_xl_h_lactive_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  switch (ctrl3_a.h_lactive)
  {
    case LSM303AH_XL_ACTIVE_HIGH:
      *val = LSM303AH_XL_ACTIVE_HIGH;
      break;

    case LSM303AH_XL_ACTIVE_LOW:
      *val = LSM303AH_XL_ACTIVE_LOW;
      break;

    default:
      *val = LSM303AH_XL_ACTIVE_HIGH;
      break;
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of lir in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_int_notification_set(const stmdev_ctx_t *ctx,
                                         lsm303ah_xl_lir_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.lir = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_int_notification_get(const stmdev_ctx_t *ctx,
                                         lsm303ah_xl_lir_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  switch (ctrl3_a.lir)
  {
    case LSM303AH_XL_INT_PULSED:
      *val = LSM303AH_XL_INT_PULSED;
      break;

    case LSM303AH_XL_INT_LATCHED:
      *val = LSM303AH_XL_INT_LATCHED;
      break;

    default:
      *val = LSM303AH_XL_INT_PULSED;
      break;
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change union of registers from CTRL4 to
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                       lsm303ah_xl_pin_int1_route_t val)
{
  lsm303ah_ctrl4_a_t ctrl4_a;
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL4_A, (uint8_t *)&ctrl4_a, 1);

  if (ret == 0)
  {
    ctrl4_a.int1_drdy         = val.int1_drdy;
    ctrl4_a.int1_fth          = val.int1_fth;
    ctrl4_a.int1_6d           = val.int1_6d;
    ctrl4_a.int1_tap          = val.int1_tap;
    ctrl4_a.int1_ff           = val.int1_ff;
    ctrl4_a.int1_wu           = val.int1_wu;
    ctrl4_a.int1_s_tap        = val.int1_s_tap;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL4_A, (uint8_t *)&ctrl4_a, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                            (uint8_t *)&wake_up_dur_a, 1);
  }

  if (ret == 0)
  {
    wake_up_dur_a.int1_fss7   = val.int1_fss7;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                             (uint8_t *)&wake_up_dur_a, 1);
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get union of registers from CTRL4 to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                       lsm303ah_xl_pin_int1_route_t *val)
{
  lsm303ah_ctrl4_a_t ctrl4_a;
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL4_A, (uint8_t *)&ctrl4_a, 1);

  if (ret == 0)
  {
    val->int1_drdy          = ctrl4_a.int1_drdy;
    val->int1_fth           = ctrl4_a.int1_fth;
    val->int1_6d            = ctrl4_a.int1_6d;
    val->int1_tap           = ctrl4_a.int1_tap;
    val->int1_ff            = ctrl4_a.int1_ff;
    val->int1_wu            = ctrl4_a.int1_wu;
    val->int1_s_tap         = ctrl4_a.int1_s_tap;
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                            (uint8_t *)&wake_up_dur_a, 1);
  }

  val->int1_fss7 = wake_up_dur_a.int1_fss7;

  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change union of registers from CTRL5 to
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                       lsm303ah_xl_pin_int2_route_t val)
{
  lsm303ah_ctrl5_a_t ctrl5_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);

  if (ret == 0)
  {
    ctrl5_a.int2_boot       = val.int2_boot;
    ctrl5_a.int2_tilt       = val.int2_tilt;
    ctrl5_a.int2_sig_mot    = val.int2_sig_mot;
    ctrl5_a.int2_step       = val.int2_step;
    ctrl5_a.int2_fth        = val.int2_fth;
    ctrl5_a.int2_drdy       = val.int2_drdy;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get union of registers from CTRL5 to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                       lsm303ah_xl_pin_int2_route_t *val)
{
  lsm303ah_ctrl5_a_t ctrl5_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);
  val->int2_boot     = ctrl5_a.int2_boot;
  val->int2_tilt     = ctrl5_a.int2_tilt;
  val->int2_sig_mot  = ctrl5_a.int2_sig_mot;
  val->int2_step     = ctrl5_a.int2_step;
  val->int2_fth      = ctrl5_a.int2_fth;
  val->int2_drdy     = ctrl5_a.int2_drdy;

  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of int2_on_int1 in reg CTRL5
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_all_on_int1_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_ctrl5_a_t ctrl5_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);

  if (ret == 0)
  {
    ctrl5_a.int2_on_int1 = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);
  }

  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of int2_on_int1 in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_all_on_int1_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_ctrl5_a_t ctrl5_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL5_A, (uint8_t *)&ctrl5_a, 1);
  *val = ctrl5_a.int2_on_int1;

  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of drdy_on_pin in reg CFG_REG_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_drdy_on_pin_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  if (ret == 0)
  {
    cfg_reg_c_m.int_mag = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_C_M,
                             (uint8_t *)&cfg_reg_c_m, 1);
  }

  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of drdy_on_pin in reg CFG_REG_C_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_drdy_on_pin_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.int_mag;

  return ret;
}

/**
  * @brief  Interrupt signal on INT_DRDY pin.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of int_on_pin in reg CFG_REG_C_M
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_on_pin_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);

  if (ret == 0)
  {
    cfg_reg_c_m.int_mag_pin = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_C_M,
                             (uint8_t *)&cfg_reg_c_m, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt signal on INT_DRDY pin.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of int_on_pin in reg CFG_REG_C_M
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_on_pin_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_C_M,
                          (uint8_t *)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.int_mag_pin;

  return ret;
}

/**
  * @brief  Interrupt generator configuration register.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change registers INT_CRTL_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_gen_conf_set(const stmdev_ctx_t *ctx,
                                     lsm303ah_int_crtl_reg_m_t *val)
{
  int32_t ret;

  ret = lsm303ah_write_reg(ctx, LSM303AH_INT_CRTL_REG_M,
                           (uint8_t *) val, 1);

  return ret;
}

/**
  * @brief  Interrupt generator configuration register.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers INT_CRTL_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_gen_conf_get(const stmdev_ctx_t *ctx,
                                     lsm303ah_int_crtl_reg_m_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_CRTL_REG_M, (uint8_t *) val, 1);

  return ret;
}

/**
  * @brief  Interrupt generator source register.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers INT_SOURCE_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_gen_source_get(const stmdev_ctx_t *ctx,
                                       lsm303ah_int_source_reg_m_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_SOURCE_REG_M,
                          (uint8_t *) val, 1);

  return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on generator.
  *         Data format is the same of output data raw: two's complement with
  *         1LSb = 1.5mG.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that contains data to write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_gen_threshold_set(const stmdev_ctx_t *ctx,
                                          uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = lsm303ah_write_reg(ctx, LSM303AH_INT_THS_L_REG_M, buff, 2);

  return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on generator.
  *         Data format is the same of output data raw: two's complement with
  *         1LSb = 1.5mG.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_int_gen_threshold_get(const stmdev_ctx_t *ctx,
                                          uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_THS_L_REG_M, buff, 2);
  *val = buff[1];
  *val = (*val * 256U) +  buff[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  interrupt_pins
  * @brief   This section groups all the functions that manage interrupt pins
  * @{
  *
  */



/**
  * @}
  *
  */

/**
  * @defgroup  Wake_Up_event
  * @brief   This section groups all the functions that manage the Wake Up
  *          event generation.
  * @{
  *
  */

/**
  * @brief  The interrupt block recognition checks data after/before the
  *         hard-iron correction to discover the interrupt.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of int_on_dataoff in reg CFG_REG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_offset_int_conf_set(const stmdev_ctx_t *ctx,
                                        lsm303ah_mg_int_on_dataoff_t val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  if (ret == 0)
  {
    cfg_reg_b_m.int_on_dataoff = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CFG_REG_B_M,
                             (uint8_t *)&cfg_reg_b_m, 1);
  }

  return ret;
}

/**
  * @brief  The interrupt block recognition checks data after/before the
  *         hard-iron correction to discover the interrupt.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of int_on_dataoff in reg CFG_REG_B.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_mg_offset_int_conf_get(const stmdev_ctx_t *ctx,
                                        lsm303ah_mg_int_on_dataoff_t *val)
{
  lsm303ah_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CFG_REG_B_M,
                          (uint8_t *)&cfg_reg_b_m, 1);

  switch (cfg_reg_b_m.int_on_dataoff)
  {
    case LSM303AH_MG_CHECK_BEFORE:
      *val = LSM303AH_MG_CHECK_BEFORE;
      break;

    case LSM303AH_MG_CHECK_AFTER:
      *val = LSM303AH_MG_CHECK_AFTER;
      break;

    default:
      *val = LSM303AH_MG_CHECK_BEFORE;
      break;
  }

  return ret;
}

/**
  * @brief  Threshold for wakeup [1 LSb = FS_XL / 64].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of wu_ths in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_wkup_threshold_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_wake_up_ths_a_t wake_up_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                          (uint8_t *)&wake_up_ths_a, 1);

  if (ret == 0)
  {
    wake_up_ths_a.wu_ths = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                             (uint8_t *)&wake_up_ths_a, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for wakeup [1 LSb = FS_XL / 64].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of wu_ths in reg WAKE_UP_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_wkup_threshold_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  lsm303ah_wake_up_ths_a_t wake_up_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                          (uint8_t *)&wake_up_ths_a, 1);
  *val = wake_up_ths_a.wu_ths;

  return ret;
}

/**
  * @brief  Wakeup duration [1 LSb = 1 / ODR].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of wu_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_wkup_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                          (uint8_t *)&wake_up_dur_a, 1);

  if (ret == 0)
  {
    wake_up_dur_a.wu_dur = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                             (uint8_t *)&wake_up_dur_a, 1);
  }

  return ret;
}

/**
  * @brief  Wakeup duration [1 LSb = 1 / ODR].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of wu_dur in reg WAKE_UP_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_wkup_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                          (uint8_t *)&wake_up_dur_a, 1);
  *val = wake_up_dur_a.wu_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Activity/Inactivity_detection
  * @brief   This section groups all the functions concerning
  *          activity/inactivity detection.
  * @{
  *
  */
/**
  * @brief  Enables gyroscope Sleep mode.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of sleep_on in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_sleep_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_wake_up_ths_a_t wake_up_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                          (uint8_t *)&wake_up_ths_a, 1);

  if (ret == 0)
  {
    wake_up_ths_a.sleep_on = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                             (uint8_t *)&wake_up_ths_a, 1);
  }

  return ret;
}

/**
  * @brief  Enables gyroscope Sleep mode.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sleep_on in reg WAKE_UP_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_sleep_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_wake_up_ths_a_t wake_up_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                          (uint8_t *)&wake_up_ths_a, 1);
  *val = wake_up_ths_a.sleep_on;

  return ret;
}

/**
  * @brief  Duration to go in sleep mode [1 LSb = 512 / ODR].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_act_sleep_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                          (uint8_t *)&wake_up_dur_a, 1);

  if (ret == 0)
  {
    wake_up_dur_a.sleep_dur = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                             (uint8_t *)&wake_up_dur_a, 1);
  }

  return ret;
}

/**
  * @brief  Duration to go in sleep mode [1 LSb = 512 / ODR].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sleep_dur in reg WAKE_UP_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_act_sleep_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                          (uint8_t *)&wake_up_dur_a, 1);
  *val = wake_up_dur_a.sleep_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  tap_generator
  * @brief   This section groups all the functions that manage the tap and
  *          double tap event generation.
  * @{
  *
  */

/**
  * @brief  Enable Z direction in tap recognition.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_z_en in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_detection_on_z_set(const stmdev_ctx_t *ctx,
                                           uint8_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.tap_z_en = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Enable Z direction in tap recognition.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_z_en in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_detection_on_z_get(const stmdev_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  *val = ctrl3_a.tap_z_en;

  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_y_en in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_detection_on_y_set(const stmdev_ctx_t *ctx,
                                           uint8_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.tap_y_en = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_y_en in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_detection_on_y_get(const stmdev_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  *val = ctrl3_a.tap_y_en;

  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_x_en in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_detection_on_x_set(const stmdev_ctx_t *ctx,
                                           uint8_t val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);

  if (ret == 0)
  {
    ctrl3_a.tap_x_en = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  }

  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_x_en in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_detection_on_x_get(const stmdev_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303ah_ctrl3_a_t ctrl3_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_CTRL3_A, (uint8_t *)&ctrl3_a, 1);
  *val = ctrl3_a.tap_x_en;

  return ret;
}

/**
  * @brief  Threshold for tap recognition [1 LSb = FS/32].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_ths in reg TAP_6D_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_threshold_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_tap_6d_ths_a_t tap_6d_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_6D_THS_A,
                          (uint8_t *)&tap_6d_ths_a, 1);

  if (ret == 0)
  {
    tap_6d_ths_a.tap_ths = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_TAP_6D_THS_A,
                             (uint8_t *)&tap_6d_ths_a, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for tap recognition [1 LSb = FS/32].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_ths in reg TAP_6D_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_tap_6d_ths_a_t tap_6d_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_6D_THS_A,
                          (uint8_t *)&tap_6d_ths_a, 1);
  *val = tap_6d_ths_a.tap_ths;

  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value of
  *         these bits is 00b which corresponds to 4*ODR_XL time. If the
  *         SHOCK[1:0] bits are set to a different value, 1LSB corresponds to
  *         8*ODR_XL time.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of shock in reg INT_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_shock_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_int_dur_a_t int_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_DUR_A,
                          (uint8_t *)&int_dur_a, 1);

  if (ret == 0)
  {
    int_dur_a.shock = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_INT_DUR_A,
                             (uint8_t *)&int_dur_a, 1);
  }

  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value of
  *         these bits is 00b which corresponds to 4*ODR_XL time. If the
  *         SHOCK[1:0] bits are set to a different value, 1LSB corresponds to
  *         8*ODR_XL time.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of shock in reg INT_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_shock_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_int_dur_a_t int_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_DUR_A,
                          (uint8_t *)&int_dur_a, 1);
  *val = int_dur_a.shock;

  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which there
  *         must not be any overthreshold event. The default value of these
  *         bits is 00b which corresponds to 2*ODR_XL time. If the QUIET[1:0]
  *         bits are set to a different value, 1LSB corresponds to
  *         4*ODR_XL time.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of quiet in reg INT_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_quiet_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_int_dur_a_t int_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_DUR_A,
                          (uint8_t *)&int_dur_a, 1);

  if (ret == 0)
  {
    int_dur_a.quiet = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_INT_DUR_A,
                             (uint8_t *)&int_dur_a, 1);
  }

  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which there
  *         must not be any overthreshold event. The default value of these
  *         bits is 00b which corresponds to 2*ODR_XL time. If the QUIET[1:0]
  *         bits are set to a different value, 1LSB corresponds to
  *         4*ODR_XL time.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of quiet in reg INT_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_quiet_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_int_dur_a_t int_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_DUR_A,
                          (uint8_t *)&int_dur_a, 1);
  *val = int_dur_a.quiet;

  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses the
  *         maximum time between two consecutive detected taps to determine a
  *         double tap event. The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to a
  *         different value, 1LSB corresponds to 32*ODR_XL time.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of lat in reg INT_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_int_dur_a_t int_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_DUR_A,
                          (uint8_t *)&int_dur_a, 1);

  if (ret == 0)
  {
    int_dur_a.lat = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_INT_DUR_A,
                             (uint8_t *)&int_dur_a, 1);
  }

  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses the
  *         maximum time between two consecutive detected taps to determine a
  *         double tap event. The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to a
  *         different value, 1LSB corresponds to 32*ODR_XL time.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of lat in reg INT_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_int_dur_a_t int_dur_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_INT_DUR_A,
                          (uint8_t *)&int_dur_a, 1);
  *val = int_dur_a.lat;

  return ret;
}

/**
  * @brief  Single/double-tap event enable/disable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of single_double_tap in regWAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_mode_set(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_single_double_tap_t val)
{
  lsm303ah_wake_up_ths_a_t wake_up_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                          (uint8_t *)&wake_up_ths_a, 1);

  if (ret == 0)
  {
    wake_up_ths_a.single_double_tap = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                             (uint8_t *)&wake_up_ths_a, 1);
  }

  return ret;
}

/**
  * @brief  Single/double-tap event enable/disable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of single_double_tap in reg WAKE_UP_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_mode_get(const stmdev_ctx_t *ctx,
                                 lsm303ah_xl_single_double_tap_t *val)
{
  lsm303ah_wake_up_ths_a_t wake_up_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_THS_A,
                          (uint8_t *)&wake_up_ths_a, 1);

  switch (wake_up_ths_a.single_double_tap)
  {
    case LSM303AH_XL_ONLY_SINGLE:
      *val = LSM303AH_XL_ONLY_SINGLE;
      break;

    case LSM303AH_XL_ONLY_DOUBLE:
      *val = LSM303AH_XL_ONLY_DOUBLE;
      break;

    default:
      *val = LSM303AH_XL_ONLY_SINGLE;
      break;
  }

  return ret;
}

/**
  * @brief  TAP source register[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers TAP_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tap_src_get(const stmdev_ctx_t *ctx,
                                lsm303ah_tap_src_a_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_SRC_A, (uint8_t *) val, 1);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Six_position_detection(6D/4D)
  * @brief   This section groups all the functions concerning six
  *          position detection (6D).
  * @{
  *
  */

/**
  * @brief  Threshold for 4D/6D function.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of 6d_ths in reg TAP_6D_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_6d_threshold_set(const stmdev_ctx_t *ctx,
                                     lsm303ah_xl_6d_ths_t val)
{
  lsm303ah_tap_6d_ths_a_t tap_6d_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_6D_THS_A,
                          (uint8_t *)&tap_6d_ths_a, 1);

  if (ret == 0)
  {
    tap_6d_ths_a._6d_ths = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_TAP_6D_THS_A,
                             (uint8_t *)&tap_6d_ths_a, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for 4D/6D function.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of 6d_ths in reg TAP_6D_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_6d_threshold_get(const stmdev_ctx_t *ctx,
                                     lsm303ah_xl_6d_ths_t *val)
{
  lsm303ah_tap_6d_ths_a_t tap_6d_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_6D_THS_A,
                          (uint8_t *)&tap_6d_ths_a, 1);

  switch (tap_6d_ths_a._6d_ths)
  {
    case LSM303AH_XL_DEG_80:
      *val = LSM303AH_XL_DEG_80;
      break;

    case LSM303AH_XL_DEG_70:
      *val = LSM303AH_XL_DEG_70;
      break;

    case LSM303AH_XL_DEG_60:
      *val = LSM303AH_XL_DEG_60;
      break;

    case LSM303AH_XL_DEG_50:
      *val = LSM303AH_XL_DEG_50;
      break;

    default:
      *val = LSM303AH_XL_DEG_80;
      break;
  }

  return ret;
}

/**
  * @brief  4D orientation detection enable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of 4d_en in reg TAP_6D_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_4d_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_tap_6d_ths_a_t tap_6d_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_6D_THS_A,
                          (uint8_t *)&tap_6d_ths_a, 1);

  if (ret == 0)
  {
    tap_6d_ths_a._4d_en = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_TAP_6D_THS_A,
                             (uint8_t *)&tap_6d_ths_a, 1);
  }

  return ret;
}

/**
  * @brief  4D orientation detection enable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of 4d_en in reg TAP_6D_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_4d_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_tap_6d_ths_a_t tap_6d_ths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_TAP_6D_THS_A,
                          (uint8_t *)&tap_6d_ths_a, 1);
  *val = tap_6d_ths_a._4d_en;

  return ret;
}

/**
  * @brief  6D source register.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get union of registers from 6D_SRC to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_6d_src_get(const stmdev_ctx_t *ctx,
                               lsm303ah_6d_src_a_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_6D_SRC_A, (uint8_t *) val, 1);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  free_fall
  * @brief   This section group all the functions concerning the
  *          free fall detection.
  * @{
  *
  */

/**
  * @brief  Free-fall duration [1 LSb = 1 / ODR].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of ff_dur in reg WAKE_UP_DUR/FREE_FALL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_ff_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  lsm303ah_free_fall_a_t free_fall_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                          (uint8_t *)&wake_up_dur_a, 1);

  if (ret == 0)
  {
    wake_up_dur_a.ff_dur = (val & 0x20U) >> 5;
    ret = lsm303ah_write_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                             (uint8_t *)&wake_up_dur_a, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_FREE_FALL_A,
                            (uint8_t *)&free_fall_a, 1);
  }

  if (ret == 0)
  {
    free_fall_a.ff_dur = 0x1FU & val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FREE_FALL_A,
                             (uint8_t *)&free_fall_a, 1);
  }

  return ret;
}

/**
  * @brief  Free-fall duration [1 LSb = 1 / ODR].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of ff_dur in reg WAKE_UP_DUR/FREE_FALL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_ff_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_wake_up_dur_a_t wake_up_dur_a;
  lsm303ah_free_fall_a_t free_fall_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_WAKE_UP_DUR_A,
                          (uint8_t *)&wake_up_dur_a, 1);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_FREE_FALL_A,
                            (uint8_t *)&free_fall_a, 1);
  }

  *val = (wake_up_dur_a.ff_dur << 5) + free_fall_a.ff_dur;

  return ret;
}

/**
  * @brief  Free-fall threshold [1 LSB = 31.25 mg].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of ff_ths in reg FREE_FALL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_ff_threshold_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_free_fall_a_t free_fall_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FREE_FALL_A,
                          (uint8_t *)&free_fall_a, 1);

  if (ret == 0)
  {
    free_fall_a.ff_ths = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FREE_FALL_A,
                             (uint8_t *)&free_fall_a, 1);
  }

  return ret;
}

/**
  * @brief  Free-fall threshold [1 LSB = 31.25 mg].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of ff_ths in reg FREE_FALL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_ff_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_free_fall_a_t free_fall_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FREE_FALL_A,
                          (uint8_t *)&free_fall_a, 1);
  *val = free_fall_a.ff_ths;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Fifo
  * @brief   This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief   Module routine result is send to
  *          FIFO instead of X,Y,Z acceleration data.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of module_to_fifo in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_xl_module_batch_set(const stmdev_ctx_t *ctx,
                                             uint8_t val)
{
  lsm303ah_fifo_ctrl_a_t fifo_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_CTRL_A,
                          (uint8_t *)&fifo_ctrl_a, 1);

  if (ret == 0)
  {
    fifo_ctrl_a.module_to_fifo = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FIFO_CTRL_A,
                             (uint8_t *)&fifo_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief   Module routine result is send to
  *          FIFO instead of X,Y,Z acceleration data.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of module_to_fifo in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_xl_module_batch_get(const stmdev_ctx_t *ctx,
                                             uint8_t *val)
{
  lsm303ah_fifo_ctrl_a_t fifo_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_CTRL_A,
                          (uint8_t *)&fifo_ctrl_a, 1);
  *val = fifo_ctrl_a.module_to_fifo;

  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of fmode in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_mode_set(const stmdev_ctx_t *ctx,
                                  lsm303ah_xl_fmode_t val)
{
  lsm303ah_fifo_ctrl_a_t fifo_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_CTRL_A,
                          (uint8_t *)&fifo_ctrl_a, 1);

  if (ret == 0)
  {
    fifo_ctrl_a.fmode = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FIFO_CTRL_A,
                             (uint8_t *)&fifo_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fmode in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_mode_get(const stmdev_ctx_t *ctx,
                                  lsm303ah_xl_fmode_t *val)
{
  lsm303ah_fifo_ctrl_a_t fifo_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_CTRL_A,
                          (uint8_t *)&fifo_ctrl_a, 1);

  switch (fifo_ctrl_a.fmode)
  {
    case LSM303AH_XL_BYPASS_MODE:
      *val = LSM303AH_XL_BYPASS_MODE;
      break;

    case LSM303AH_XL_FIFO_MODE:
      *val = LSM303AH_XL_FIFO_MODE;
      break;

    case LSM303AH_XL_STREAM_TO_FIFO_MODE:
      *val = LSM303AH_XL_STREAM_TO_FIFO_MODE;
      break;

    case LSM303AH_XL_BYPASS_TO_STREAM_MODE:
      *val = LSM303AH_XL_BYPASS_TO_STREAM_MODE;
      break;

    case LSM303AH_XL_STREAM_MODE:
      *val = LSM303AH_XL_STREAM_MODE;
      break;

    default:
      *val = LSM303AH_XL_BYPASS_MODE;
      break;
  }

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_watermark in reg FIFO_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret;

  ret = lsm303ah_write_reg(ctx, LSM303AH_FIFO_THS_A, (uint8_t *)&val, 1);

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_watermark in reg FIFO_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_watermark_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_THS_A, val, 1);

  return ret;
}

/**
  * @brief  FIFO full, 256 unread samples.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of diff in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_full_flag_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  lsm303ah_fifo_src_a_t fifo_src_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SRC_A,
                          (uint8_t *)&fifo_src_a, 1);
  *val = fifo_src_a.diff;

  return ret;
}

/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_ovr in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_fifo_src_a_t fifo_src_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SRC_A,
                          (uint8_t *)&fifo_src_a, 1);
  *val = fifo_src_a.fifo_ovr;

  return ret;
}

/**
  * @brief  FIFO threshold status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fth in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_fifo_src_a_t fifo_src_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SRC_A,
                          (uint8_t *)&fifo_src_a, 1);
  *val = fifo_src_a.fth;

  return ret;
}

/**
  * @brief  The number of unread samples stored in FIFO.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of diff in reg FIFO_SAMPLES.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_data_level_get(const stmdev_ctx_t *ctx,
                                        uint16_t *val)
{
  lsm303ah_fifo_src_a_t fifo_src_a;
  uint8_t fifo_sample_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SRC_A,
                          (uint8_t *)&fifo_src_a, 1);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SAMPLES_A, &fifo_sample_a, 1);
    *val = fifo_src_a.diff;
    *val = *val << 8;
    *val += fifo_sample_a;
  }

  return ret;
}

/**
  * @brief  FIFO_SRCregister.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_fifo_src_get(const stmdev_ctx_t *ctx,
                                 lsm303ah_fifo_src_a_t *val)
{
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FIFO_SRC_A, (uint8_t *) val, 1);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Pedometer
  * @brief   This section groups all the functions that manage pedometer.
  * @{
  *
  */

/**
  * @brief  Minimum threshold value for step counter routine.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of sc_mths in reg STEP_COUNTER_MINTHS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_threshold_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_step_counter_minths_a_t step_counter_minths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                          (uint8_t *)&step_counter_minths_a, 1);

  if (ret == 0)
  {
    step_counter_minths_a.sc_mths = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                             (uint8_t *)&step_counter_minths_a, 1);
  }

  return ret;
}

/**
  * @brief  Minimum threshold value for step counter routine.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sc_mths in reg  STEP_COUNTER_MINTHS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_threshold_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  lsm303ah_step_counter_minths_a_t step_counter_minths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                          (uint8_t *)&step_counter_minths_a, 1);
  *val = step_counter_minths_a.sc_mths;

  return ret;
}

/**
  * @brief  Pedometer data range.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of pedo4g in reg STEP_COUNTER_MINTHS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_full_scale_set(const stmdev_ctx_t *ctx,
                                        lsm303ah_xl_pedo4g_t val)
{
  lsm303ah_step_counter_minths_a_t step_counter_minths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                          (uint8_t *)&step_counter_minths_a, 1);

  if (ret == 0)
  {
    step_counter_minths_a.pedo4g = (uint8_t)val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                             (uint8_t *)&step_counter_minths_a, 1);
  }

  return ret;
}

/**
  * @brief  Pedometer data range.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of pedo4g in reg STEP_COUNTER_MINTHS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_full_scale_get(const stmdev_ctx_t *ctx,
                                        lsm303ah_xl_pedo4g_t *val)
{
  lsm303ah_step_counter_minths_a_t step_counter_minths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                          (uint8_t *)&step_counter_minths_a, 1);

  switch (step_counter_minths_a.pedo4g)
  {
    case LSM303AH_XL_PEDO_AT_2g:
      *val = LSM303AH_XL_PEDO_AT_2g;
      break;

    case LSM303AH_XL_PEDO_AT_4g:
      *val = LSM303AH_XL_PEDO_AT_4g;
      break;

    default:
      *val = LSM303AH_XL_PEDO_AT_2g;
      break;
  }

  return ret;
}

/**
  * @brief  Reset pedometer step counter.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of rst_nstep in reg STEP_COUNTER_MINTHS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_step_reset_set(const stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  lsm303ah_step_counter_minths_a_t step_counter_minths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                          (uint8_t *)&step_counter_minths_a, 1);

  if (ret == 0)
  {
    step_counter_minths_a.rst_nstep = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                             (uint8_t *)&step_counter_minths_a, 1);
  }

  return ret;
}

/**
  * @brief  Reset pedometer step counter.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of rst_nstep in reg STEP_COUNTER_MINTHS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_step_reset_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  lsm303ah_step_counter_minths_a_t step_counter_minths_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNTER_MINTHS_A,
                          (uint8_t *)&step_counter_minths_a, 1);
  *val = step_counter_minths_a.rst_nstep;

  return ret;
}

/**
  * @brief  Step detection flag.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of step_detect in reg FUNC_CK_GATE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_step_detect_flag_get(const stmdev_ctx_t *ctx,
                                              uint8_t *val)
{
  lsm303ah_func_ck_gate_a_t func_ck_gate_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CK_GATE_A,
                          (uint8_t *)&func_ck_gate_a, 1);
  *val = func_ck_gate_a.step_detect;

  return ret;
}

/**
  * @brief  Enable pedometer algorithm.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of step_cnt_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_sens_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);

  if (ret == 0)
  {
    func_ctrl_a.step_cnt_on = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FUNC_CTRL_A,
                             (uint8_t *)&func_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief  Enable pedometer algorithm.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of step_cnt_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_sens_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);
  *val = func_ctrl_a.step_cnt_on;

  return ret;
}

/**
  * @brief  Minimum number of steps to start the increment step counter.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of deb_step in reg PEDO_DEB_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_debounce_steps_set(const stmdev_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303ah_pedo_deb_reg_a_t pedo_deb_reg_a;
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_PEDO_DEB_REG_A,
                            (uint8_t *)&pedo_deb_reg_a, 1);
  }

  if (ret == 0)
  {
    pedo_deb_reg_a.deb_step = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_PEDO_DEB_REG_A,
                             (uint8_t *)&pedo_deb_reg_a, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Minimum number of steps to start the increment step counter.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of deb_step in reg PEDO_DEB_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_debounce_steps_get(const stmdev_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303ah_pedo_deb_reg_a_t pedo_deb_reg_a;
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_PEDO_DEB_REG_A,
                            (uint8_t *)&pedo_deb_reg_a, 1);
  }

  if (ret == 0)
  {
    *val = pedo_deb_reg_a.deb_step;
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Debounce time. If the time between two consecutive steps is greater
  *         than DEB_TIME*80ms, the debouncer is reactivated.
  *         Default value: 01101[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of deb_time in reg PEDO_DEB_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_timeout_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_pedo_deb_reg_a_t pedo_deb_reg_a;
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_PEDO_DEB_REG_A,
                            (uint8_t *)&pedo_deb_reg_a, 1);
  }

  if (ret == 0)
  {
    pedo_deb_reg_a.deb_time = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_PEDO_DEB_REG_A,
                             (uint8_t *)&pedo_deb_reg_a, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Debounce time. If the time between two consecutive steps is greater
  *         than DEB_TIME*80ms, the debouncer is reactivated.
  *         Default value: 01101[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of deb_time in reg PEDO_DEB_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_timeout_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_pedo_deb_reg_a_t pedo_deb_reg_a;
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_PEDO_DEB_REG_A,
                            (uint8_t *)&pedo_deb_reg_a, 1);
  }

  if (ret == 0)
  {
    *val = pedo_deb_reg_a.deb_time;
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Period of time to detect at least one step to generate step
  *         recognition [1 LSb = 1.6384 s].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_steps_period_set(const stmdev_ctx_t *ctx,
                                          uint8_t *buff)
{
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_write_reg(ctx, LSM303AH_STEP_COUNT_DELTA_A, buff, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Period of time to detect at least one step to generate step
  *         recognition [1 LSb = 1.6384 s].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_pedo_steps_period_get(const stmdev_ctx_t *ctx,
                                          uint8_t *buff)
{
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_STEP_COUNT_DELTA_A, buff, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  significant_motion
  * @brief   This section groups all the functions that manage the
  *          significant motion detection.
  * @{
  *
  */

/**
  * @brief   Significant motion event detection status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sig_mot_detect in reg FUNC_CK_GATE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_motion_data_ready_flag_get(const stmdev_ctx_t *ctx,
                                               uint8_t *val)
{
  lsm303ah_func_ck_gate_a_t func_ck_gate_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CK_GATE_A,
                          (uint8_t *)&func_ck_gate_a, 1);
  *val = func_ck_gate_a.sig_mot_detect;

  return ret;
}

/**
  * @brief  Enable significant motion detection function.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of sign_mot_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_motion_sens_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);

  if (ret == 0)
  {
    func_ctrl_a.sign_mot_on = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FUNC_CTRL_A,
                             (uint8_t *)&func_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief  Enable significant motion detection function.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sign_mot_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_motion_sens_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);
  *val = func_ctrl_a.sign_mot_on;

  return ret;
}

/**
  * @brief  These bits define the threshold value which corresponds to the
  *         number of steps to be performed by the user upon a change of
  *         location before the significant motion interrupt is generated.
  *         It is expressed as an 8-bit unsigned value.
  *         The default value of this field is equal to 6 (= 00000110b).[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of sm_ths in reg SM_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_motion_threshold_set(const stmdev_ctx_t *ctx,
                                         uint8_t val)
{
  lsm303ah_sm_ths_a_t sm_ths_a;
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_SM_THS_A, (uint8_t *)&sm_ths_a, 1);
  }

  if (ret == 0)
  {
    sm_ths_a.sm_ths = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_SM_THS_A, (uint8_t *)&sm_ths_a, 1);
  }

  if (ret == 0)
  {
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @brief  These bits define the threshold value which corresponds to the
  *         number of steps to be performed by the user upon a change of
  *         location before the significant motion interrupt is generated.
  *         It is expressed as an 8-bit unsigned value.
  *         The default value of this field is equal to 6 (= 00000110b).[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sm_ths in reg SM_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_motion_threshold_get(const stmdev_ctx_t *ctx,
                                         uint8_t *val)
{
  lsm303ah_sm_ths_a_t sm_ths_a;
  int32_t ret;

  ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_ADV_BANK);

  if (ret == 0)
  {
    ret = lsm303ah_read_reg(ctx, LSM303AH_SM_THS_A, (uint8_t *)&sm_ths_a, 1);
  }

  if (ret == 0)
  {
    *val = sm_ths_a.sm_ths;
    ret = lsm303ah_xl_mem_bank_set(ctx, LSM303AH_XL_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  tilt_detection
  * @brief   This section groups all the functions that manage the tilt
  *          event detection.
  * @{
  *
  */

/**
  * @brief  Tilt event detection status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tilt_int in reg FUNC_CK_GATE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tilt_data_ready_flag_get(const stmdev_ctx_t *ctx,
                                             uint8_t *val)
{
  lsm303ah_func_ck_gate_a_t func_ck_gate_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CK_GATE_A,
                          (uint8_t *)&func_ck_gate_a, 1);
  *val = func_ck_gate_a.tilt_int;

  return ret;
}

/**
  * @brief  Enable tilt calculation.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of tilt_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tilt_sens_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);

  if (ret == 0)
  {
    func_ctrl_a.tilt_on = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FUNC_CTRL_A,
                             (uint8_t *)&func_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief  Enable tilt calculation.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tilt_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_tilt_sens_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);
  *val = func_ctrl_a.tilt_on;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  module
  * @brief   This section groups all the functions that manage
  *          module calculation
  * @{
  *
  */

/**
  * @brief  Module processing enable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Change the values of module_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_module_sens_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);

  if (ret == 0)
  {
    func_ctrl_a.module_on = val;
    ret = lsm303ah_write_reg(ctx, LSM303AH_FUNC_CTRL_A,
                             (uint8_t *)&func_ctrl_a, 1);
  }

  return ret;
}

/**
  * @brief  Module processing enable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of module_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm303ah_xl_module_sens_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm303ah_func_ctrl_a_t func_ctrl_a;
  int32_t ret;

  ret = lsm303ah_read_reg(ctx, LSM303AH_FUNC_CTRL_A,
                          (uint8_t *)&func_ctrl_a, 1);
  *val = func_ctrl_a.module_on;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @}
  *
  */
