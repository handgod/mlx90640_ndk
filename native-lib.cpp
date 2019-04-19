#include <jni.h>
#include <string>
#include <iostream>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

#define MLX90640_IOC_MAGIC 'm'
#define MLX90640_IOC_RESET _IO(MLX90640_IOC_MAGIC, 0)
#define MLX90640_IOC_READ_KBUFFER _IOR(MLX90640_IOC_MAGIC, 1, int)
#define MLX90640_IOC_MAXNR 1
//float buffer_[1024] = {0,};
std::mutex readMutex_;

const uint8_t MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
float mlx90640To[768];
paramsMLX90640 mlx90640;
#define TA_SHIFT 8

int driver_ioc_test() {
    int fd = open("/dev/mlx90640", O_RDWR);             // Open the device with read/write access
    if (fd < 0){
        //printf("Failed to open the device...\n");
        return -1;
    }

    if (ioctl(fd, MLX90640_IOC_READ_KBUFFER, mlx90640To) < 0) {
        //printf("[kdj6724] ioctl error\n");
        return -1;
    }
    close(fd);
    return 0;
}

int mlx90640_test_init() {
    int status;
    uint16_t eeMLX90640[832];
    MLX90640_I2CInit();

    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0)
        std::cout << "Failed to load system parameters" << std::endl;

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0)
        std::cout << "Parameter extraction failed" << std::endl;

    //Once params are extracted, we can release eeMLX90640 array

    //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.5Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 1Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 2Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 4Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 8Hz effective - Works
    //MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 16Hz effective - Works at 800kHz
    MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 32Hz effective - Works at 800kHz
    //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 64Hz effective - fails
    return 0;
}

int mlx90640_calculate_result() {

    uint16_t mlx90640Frame[834];
    memset(mlx90640Frame, 0, sizeof(mlx90640Frame));
    memset(mlx90640Frame, 0, sizeof(mlx90640Frame));
    readMutex_.lock();
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    readMutex_.unlock();
    return 0;
}

int mlx90640_image_result() {

    uint16_t mlx90640Frame[834];
    memset(mlx90640Frame, 0, sizeof(mlx90640Frame));
    readMutex_.lock();
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    MLX90640_GetImage(mlx90640Frame, &mlx90640, mlx90640To);
    readMutex_.unlock();
    return 0;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_opertivem_ndktest_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_opertivem_ndktest_MainActivity_mlx90640Init(
        JNIEnv *env, jobject obj) {
    mlx90640_test_init();
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_opertivem_ndktest_MainActivity_getCalBuffer(
        JNIEnv *env, jobject obj) {
    jfloatArray result;
    result = env->NewFloatArray(sizeof(mlx90640To));
    mlx90640_calculate_result();
    env->SetFloatArrayRegion(result, 0, sizeof(mlx90640To), (jfloat*)mlx90640To);
    return result;
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_opertivem_ndktest_MainActivity_getImgBuffer(
        JNIEnv *env, jobject obj) {
    jfloatArray result;
    result = env->NewFloatArray(sizeof(mlx90640To));
    mlx90640_image_result();
    env->SetFloatArrayRegion(result, 0, sizeof(mlx90640To), (jfloat*)mlx90640To);
    return result;
}