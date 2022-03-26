#ifndef OV7675_H
#define OV7675_H

#ifdef __cplusplus
extern "C" {
#endif

// Camera 8bit I2C (SCCB) slave address
#define OV7675_I2C_ADDRESS    0x42

#define OV7675_REG_PROD_ID_1  0x0A
#define OV7675_REG_PROD_ID_2  0x0B

#define OV7675_REG_PROD_ID_1_VALUE  0x76
#define OV7675_REG_PROD_ID_2_VALUE  0x73

#ifdef __cplusplus
}
#endif

#endif // OV7675_H