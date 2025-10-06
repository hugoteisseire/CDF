

#ifndef IMUOTOS_H
#define IMUOTOS_H

#include <cstdint>

static constexpr float M_PI = 3.14159265358979323846f;

struct ImuPose {
    double x;
    double y;
    double h;
};

class ImuOTOS {
public:
    ImuOTOS(const char* device, uint8_t address);
    bool openBus();
    void closeBus();
    bool reset();
    bool calibrate(uint8_t samples);
    bool enableSignalProcessing(bool var, bool rot, bool acc, bool lut);
    bool readPose(ImuPose& pose);
    bool writePose(const ImuPose& pose);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readRegister16(uint8_t reg, int16_t &value);

    bool readVelocity(ImuPose& velocity);
    bool readAcceleration(ImuPose& acceleration);
    float getLinearScalar();
    bool setLinearScalar(float scalar);

    float getAngularScalar();
    bool setAngularScalar(float scalar);


        // Conversion factors
    static constexpr float kMeterToInch = 39.37f;
    static constexpr float kInchToMeter = 1.0f / kMeterToInch;
    static constexpr float kRadianToDegree = 180.0f / M_PI;
    static constexpr float kDegreeToRadian = M_PI / 180.0f;
    // Conversion factor for the linear position registers. 16-bit signed
    // registers with a max value of 10 meters (394 inches) gives a resolution
    // of about 0.0003 mps (0.012 ips)
    static constexpr float kMeterToInt16 = 32768.0f / 10.0f;
    static constexpr float kInt16ToMeter = 1.0f / kMeterToInt16;

    // Conversion factor for the linear velocity registers. 16-bit signed
    // registers with a max value of 5 mps (197 ips) gives a resolution of about
    // 0.00015 mps (0.006 ips)
    static constexpr float kMpsToInt16 = 32768.0f / 5.0f;
    static constexpr float kInt16ToMps = 1.0f / kMpsToInt16;

    // Conversion factor for the linear acceleration registers. 16-bit signed
    // registers with a max value of 157 mps^2 (16 g) gives a resolution of
    // about 0.0048 mps^2 (0.49 mg)
    static constexpr float kMpssToInt16 = 32768.0f / (16.0f * 9.80665f);
    static constexpr float kInt16ToMpss = 1.0f / kMpssToInt16;

    // Conversion factor for the angular position registers. 16-bit signed
    // registers with a max value of pi radians (180 degrees) gives a resolution
    // of about 0.00096 radians (0.0055 degrees)
    static constexpr float kRadToInt16 = 32768.0f / M_PI;
    static constexpr float kInt16ToRad = 1.0f / kRadToInt16;

    // Conversion factor for the angular velocity registers. 16-bit signed
    // registers with a max value of 34.9 rps (2000 dps) gives a resolution of
    // about 0.0011 rps (0.061 degrees per second)
    static constexpr float kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
    static constexpr float kInt16ToRps = 1.0f / kRpsToInt16;

    // Conversion factor for the angular acceleration registers. 16-bit signed
    // registers with a max value of 3141 rps^2 (180000 dps^2) gives a
    // resolution of about 0.096 rps^2 (5.5 dps^2)
    static constexpr float kRpssToInt16 = 32768.0f / (M_PI * 1000.0f);
    static constexpr float kInt16ToRpss = 1.0f / kRpssToInt16;

private:
    const char* device_;
    uint8_t address_;
    int file_;
};

#endif // IMUOTOS_H