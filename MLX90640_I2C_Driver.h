//
// Created by 김동진 on 18/04/2019.
//

#ifndef NDKTEST_MLX90640_I2C_DRIVER_H
#define NDKTEST_MLX90640_I2C_DRIVER_H

#include <cstdint>

void MLX90640_I2CInit(void);
int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data);
int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
void MLX90640_I2CFreqSet(int freq);

#endif //NDKTEST_MLX90640_I2C_DRIVER_H
