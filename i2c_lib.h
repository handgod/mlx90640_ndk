//
// Created by 김동진 on 18/04/2019.
//

#include <cstdint>

#ifndef NDKTEST_I2C_LIB_H
#define NDKTEST_I2C_LIB_H

#endif //NDKTEST_I2C_LIB_H
class I2C {
private :
    int _fd;

public :
    I2C();
    ~I2C(void);
    int setup(char *device, uint8_t address);
    int dispose();
    int send(uint8_t *buffer, uint8_t num_bytes);
    int recv(uint8_t *buffer, uint8_t num_bytes);
};