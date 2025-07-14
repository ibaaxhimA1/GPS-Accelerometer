#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "lsm303ah_reg.h"
#include "Accelerometer.h"

/* LSM303AH I2C addresses */


#define I2C_NODE DT_NODELABEL(i2c0)

/* I2C device specifications */
static const struct i2c_dt_spec xl_i2c = {
    .bus = DEVICE_DT_GET(I2C_NODE),
    .addr = LSM303AH_I2C_ADD_XL,
};

static const struct i2c_dt_spec mg_i2c = {
    .bus = DEVICE_DT_GET(I2C_NODE),
    .addr = LSM303AH_I2C_ADD_MG,
};

/* Data buffers */
static int16_t accel_data[3];
static int16_t mag_data[3];

/* Platform functions */


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    const struct i2c_dt_spec *i2c_dev = (const struct i2c_dt_spec *)handle;
    uint8_t write_buf[len + 1];

    if (len > 1) reg |= 0x80;  
    
    write_buf[0] = reg;
    memcpy(&write_buf[1], bufp, len);
    
    return i2c_write_dt(i2c_dev, write_buf, len + 1);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    const struct i2c_dt_spec *i2c_dev = (const struct i2c_dt_spec *)handle;
    
    if (len > 1) reg |= 0x80;  
    
    return i2c_write_read_dt(i2c_dev, &reg, 1, bufp, len);
}

void lsm303ah_config_and_print(void)
{
    stmdev_ctx_t dev_ctx_xl = {.write_reg = platform_write, .read_reg = platform_read, .handle = (void *)&xl_i2c};
    stmdev_ctx_t dev_ctx_mg = {.write_reg = platform_write, .read_reg = platform_read, .handle = (void *)&mg_i2c};
    uint8_t whoami, rst;
    lsm303ah_reg_t reg;

    printk("LSM303AH sensor starting...\n");

    
    if (!device_is_ready(xl_i2c.bus) || !device_is_ready(mg_i2c.bus)) {
        printk("I2C device not ready\n");
        return;
    }

    k_msleep(100);  

    /* Check device IDs */
    lsm303ah_xl_device_id_get(&dev_ctx_xl, &whoami);
    if (whoami != LSM303AH_ID_XL) {
        printk("Accelerometer ID error: 0x%02X\n", whoami);
        return;
    }

    lsm303ah_mg_device_id_get(&dev_ctx_mg, &whoami);
    if (whoami != LSM303AH_ID_MG) {
        printk("Magnetometer ID error: 0x%02X\n", whoami);
        return;
    }
   
    /* Configure sensors */
    lsm303ah_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
    lsm303ah_mg_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
    lsm303ah_xl_full_scale_set(&dev_ctx_xl, LSM303AH_XL_2g);
    lsm303ah_xl_data_rate_set(&dev_ctx_xl, LSM303AH_XL_ODR_100Hz_LP);
    lsm303ah_mg_data_rate_set(&dev_ctx_mg, LSM303AH_MG_ODR_10Hz);
    lsm303ah_mg_operating_mode_set(&dev_ctx_mg, LSM303AH_MG_CONTINUOUS_MODE);
    lsm303ah_mg_set_rst_mode_set(&dev_ctx_mg, LSM303AH_MG_SENS_OFF_CANC_EVERY_ODR);
    lsm303ah_mg_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);

    printk("Sensor initialized\n");

    
    while (1) {
        /* Read accelerometer */
        lsm303ah_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
        if (reg.status_a.drdy) {
            lsm303ah_acceleration_raw_get(&dev_ctx_xl, accel_data);
            printk("A: %d %d %d\n", accel_data[0], accel_data[1], accel_data[2]);
        }
        else printk("Acceleration data not ready");

        /* Read magnetometer */
        lsm303ah_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
        if (reg.status_reg_m.zyxda) {
            lsm303ah_magnetic_raw_get(&dev_ctx_mg, mag_data);
            printk("M: %d %d %d\n", mag_data[0], mag_data[1], mag_data[2]);
        }
        else printk("Magnetic data not ready");

        k_msleep(100);
    }
}