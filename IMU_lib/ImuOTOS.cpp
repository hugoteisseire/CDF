#include "ImuOTOS.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <thread>
#include <chrono>

ImuOTOS::ImuOTOS(const char* device, uint8_t address)
    : device_(device), address_(address), file_(-1) {}

bool ImuOTOS::openBus() {
    file_ = open(device_, O_RDWR);
    if (file_ < 0) return false;
    return ioctl(file_, I2C_SLAVE, address_) >= 0;
}

void ImuOTOS::closeBus() {
    if (file_ >= 0) close(file_);
    file_ = -1;
}

bool ImuOTOS::reset() { return writeRegister(0x07, 0x01); }
bool ImuOTOS::calibrate(uint8_t samples) { return writeRegister(0x06, samples); }
bool ImuOTOS::enableSignalProcessing(bool var, bool rot, bool acc, bool lut) {
    uint8_t config = (var ? 0x08 : 0) | (rot ? 0x04 : 0) | (acc ? 0x02 : 0) | (lut ? 0x01 : 0);
    return writeRegister(0x0E, config);
}

float ImuOTOS::getLinearScalar()
{
    // Read the linear scalar from the device
    uint8_t rawScalar;
    if (!readRegister(0x04, rawScalar)) return -1.0f;

    // Convert to float, multiples of 0.1%
    return (((int8_t)rawScalar) * 0.001f) + 1.0f;
}

bool ImuOTOS::setLinearScalar(float scalar)
{

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return writeRegister(0x04, rawScalar);
}

float ImuOTOS::getAngularScalar()
{
    // Read the angular scalar from the device
    uint8_t rawScalar;
    if (!readRegister(0x05, rawScalar)) return -1.0f;

    // Convert to float, multiples of 0.1%
    return ((((int8_t)rawScalar) * 0.001f) + 1.0f);
}

bool ImuOTOS::setAngularScalar(float scalar)
{

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return writeRegister(0x05, rawScalar);
}

bool ImuOTOS::readPose(ImuPose& pose) {
    int16_t rawX, rawY, rawH;
    if (!readRegister16(0x20, rawX) ||
        !readRegister16(0x22, rawY) ||
        !readRegister16(0x24, rawH)) {
        return false;
    }
    pose.x = (double)rawX * kInt16ToMeter;
    pose.y = (double)rawY * kInt16ToMeter;
    pose.h = (double)rawH * kInt16ToRad;
    return true;
}
bool ImuOTOS::readAcceleration(ImuPose& acceleration) {
    int16_t rawX, rawY, rawH;

    // Lecture des registres d'accélération
    if (!readRegister16(0x2C, rawX) || // Acceleration X Low/High Byte
        !readRegister16(0x2E, rawY) || // Acceleration Y Low/High Byte
        !readRegister16(0x30, rawH)) { // Acceleration H Low/High Byte
        return false;
    }
    // Conversion en m/s² et rad/s²
    acceleration.x = (double)rawX * kInt16ToMpss;
    acceleration.y = (double)rawY * kInt16ToMpss;
    acceleration.h = (double)rawH * kInt16ToRpss;

    return true;
}

bool ImuOTOS::readVelocity(ImuPose& velocity) {
    int16_t rawX, rawY, rawH;

    // Lecture des registres de vitesse
    if (!readRegister16(0x26, rawX) || // Velocity X Low/High Byte
        !readRegister16(0x28, rawY) || // Velocity Y Low/High Byte
        !readRegister16(0x2A, rawH)) { // Velocity H Low/High Byte
        return false;
    }

    // Conversion en m/s et rad/s
    velocity.x = (double)rawX * kInt16ToMps;
    velocity.y = (double)rawY * kInt16ToMps;
    velocity.h = (double)rawH * kInt16ToRps;

    return true;
}


bool ImuOTOS::writePose(const ImuPose& pose) {
    int16_t rawX = pose.x * kMeterToInt16;
    int16_t rawY = pose.y * kMeterToInt16;
    int16_t rawH = pose.h * kRadToInt16;
    uint8_t buffer[7];
    buffer[0] = 0x20;
    buffer[1] = rawX & 0xFF;
    buffer[2] = (rawX >> 8) & 0xFF;
    buffer[3] = rawY & 0xFF;
    buffer[4] = (rawY >> 8) & 0xFF;
    buffer[5] = rawH & 0xFF;
    buffer[6] = (rawH >> 8) & 0xFF;
    return (write(file_, buffer, 7) == 7);
}

// Méthodes utilitaires
bool ImuOTOS::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return (write(file_, buffer, 2) == 2);
}

bool ImuOTOS::readRegister(uint8_t reg, uint8_t &value) {
    if (write(file_, &reg, 1) != 1) return false;
    return (read(file_, &value, 1) == 1);
}

bool ImuOTOS::readRegister16(uint8_t reg, int16_t &value) {
    if (write(file_, &reg, 1) != 1) return false;
    uint8_t buffer[2];
    if (read(file_, buffer, 2) != 2) return false;
    value = (buffer[1] << 8) | buffer[0];
    return true;
}

