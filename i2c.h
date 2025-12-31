#ifndef I2C_H_
#define I2C_H_

#include "S32K144.h"
typedef enum {
    I2C_IDLE = 0,

    I2C_START,
    I2C_SEND_ADDR,
    I2C_SEND_DATA,

    I2C_RESTART,
    I2C_SEND_ADDR_READ,
    I2C_READ_DATA,

    I2C_STOP,
    I2C_DONE,
    I2C_ERROR
} I2C_State_t;

void LPI2C0_Init(void);
//void LPI2C0_SendFrame(uint8_t slaveAddr, uint8_t *data, uint8_t length);
uint8_t LPI2C0_SendFrame(uint8_t slaveAddr, uint8_t *data, uint8_t length);
uint8_t LPI2C0_ReadByte(uint8_t slaveAddr, uint8_t *data );
#endif

