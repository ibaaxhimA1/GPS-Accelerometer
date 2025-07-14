#define LSM303AH_I2C_ADD_XL    0x1D
#define LSM303AH_I2C_ADD_MG    0x1E

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void lsm303ah_config_and_print(void);