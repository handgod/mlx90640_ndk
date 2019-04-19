//
// Created by 김동진 on 18/04/2019.
//
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "i2c_lib.h"

I2C::I2C(){
    this->_fd = -1;
}

I2C::~I2C(void){
    this->dispose();
}

int I2C::setup(char *device, uint8_t address) {
    int ret;

    this->_fd = open(device, O_RDWR);
    if (this->_fd < 0)
        return this->_fd;

    ret = ioctl(this->_fd, I2C_SLAVE_FORCE, address);
    if (ret < 0)
        return ret;

    return this->_fd;
}

int I2C::dispose() {
    return close(this->_fd);
}

int I2C::send(uint8_t *buffer, uint8_t num_bytes) {
    return write(this->_fd, buffer, num_bytes);
}

int I2C::recv(uint8_t *buffer, uint8_t num_bytes) {
    return read(this->_fd, buffer, num_bytes);
}