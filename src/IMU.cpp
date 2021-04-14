#include "IMU.h"
#include "Wire.h"

#define SCL_PIN 16
#define SDA_PIN 17
#define I2C_CLK 400000

#define IMU_ADDR 0x69
#define ACC_SCALE 16384.0
#define GYRO_SCALE 131.0
#define TEMP_SCALE 326.8
#define TEMP_OFFSET 25.0

#define ERROR_SAMPLES 1000
#define ACC_ERROR_X 0.006
#define ACC_ERROR_Y -0.05
#define ACC_ERROR_Z -0.004
#define GYRO_ERROR_X -0.0026
#define GYRO_ERROR_Y 0.0027
#define GYRO_ERROR_Z -0.0056

IMU::IMU() {

    LPFEnabled = false;

    // Clear accelerometer buffer
    for (size_t i = 0; i < 3; i++) {
        acc[i] = 0.0;
        filtAcc[i][0] = 0.0;
        filtAcc[i][1] = 0.0;
    }

    // Clear gyroscope buffer
    for (size_t i = 0; i < 3; i++) {
        gyro[i] = 0.0;
        filtGyro[i][0] = 0.0;
        filtGyro[i][1] = 0.0;

        angle[i] = 0.0;
        euler[i] = 0.0;
    }
    
    // Clear temperature buffer
    temp = 0.0;
    filtTemp[0] = 0.0;
    filtTemp[1] = 0.0;

}

IMU::~IMU() {}

void IMU::enable() {

    // Setup I2C
    Wire1.setSCL(SCL_PIN);
    Wire1.setSDA(SDA_PIN);
    Wire1.begin();
    Wire1.setClock(I2C_CLK);

    // Update PWR_MGMT_1 register to disable sleep mode
    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(0x6B);
    Wire1.write(0);
    Wire1.endTransmission(true);

    // Update CONFIG register to enable device
    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(0x1A);
    Wire1.write(0);
    Wire1.endTransmission(true);

}

void IMU::update() {
    updateAcc();
    updateGyro();
    updateTemp();
}

void IMU::updateAcc() {

    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(0x3B); // First register of 6 accelerometer registers
    Wire1.endTransmission(false);
    Wire1.requestFrom(IMU_ADDR, 6, true);

    int16_t x = ((int16_t) Wire1.read() << 8 | Wire1.read());
    int16_t y = ((int16_t) Wire1.read() << 8 | Wire1.read());
    int16_t z = ((int16_t) Wire1.read() << 8 | Wire1.read());

    // Convert from IMU axis to rocket axis
    acc[0] = -1 * (((float) y) / ACC_SCALE) - ACC_ERROR_X;
    acc[1] = -1 * (((float) z) / ACC_SCALE) - ACC_ERROR_Y;
    acc[2] = -1 * (((float) x) / ACC_SCALE) - ACC_ERROR_Z;

    if (LPFEnabled) {
        for (size_t i = 0; i < 3; i++) {
            // Shift output values
            filtAcc[i][1] = filtAcc[i][0];
            // Compute new output value
            filtAcc[i][0] = LPFCoeff[0] * acc[i] + LPFCoeff[1] * filtAcc[i][1];
        }
    }

}

void IMU::updateGyro() {

    // Get time for calculating orientation
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000; // Convert to seconds

    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(0x43); // First register of 6 gyroscope registers
    Wire1.endTransmission(false);
    Wire1.requestFrom(IMU_ADDR, 6, true);

    int16_t x = ((int16_t) Wire1.read() << 8 | Wire1.read());
    int16_t y = ((int16_t) Wire1.read() << 8 | Wire1.read());
    int16_t z = ((int16_t) Wire1.read() << 8 | Wire1.read());

    // Convert from IMU axis to rocket axis
    gyro[0] = -1 * (((float) y) / GYRO_SCALE) - GYRO_ERROR_X;
    gyro[1] = -1 * (((float) z) / GYRO_SCALE) - GYRO_ERROR_Y;
    gyro[2] = -1 * (((float) x) / GYRO_SCALE) - GYRO_ERROR_Z;

    if (LPFEnabled) {
        for (size_t i = 0; i < 3; i++) {
            // Shift output values
            filtGyro[i][1] = filtGyro[i][0];
            // Compute new output value
            filtGyro[i][0] = LPFCoeff[0] * gyro[i] + LPFCoeff[1] * filtGyro[i][1];
        }
    }

    for (size_t i = 0; i < 3; i++) {
        angle[i] = angle[i] + (getGyro(i) * elapsedTime);
        if (angle[i] > 180) {
            angle[i] = angle[i] - 360;
        } else if (angle[i] < -180) {
            angle[i] = angle[i] + 360;
        }
    }
    
    euler[0] = euler[0]
    + ((getGyro(2) 
    + (getGyro(0) * sin(euler[0]) * tan(euler[1]))
    + (getGyro(1) * cos(euler[0]) * tan(euler[1]))) * elapsedTime);

    euler[1] = euler[1]
    + (((getGyro(0) * cos(euler[0]))
    - (getGyro(1) * sin(euler[0]))) * elapsedTime);

    euler[2] = euler[2]
    + ((((getGyro(0) * sin(euler[0])) / (cos(euler[1])))
    + ((getGyro(1) * cos(euler[0])) / (cos(euler[1])))) * elapsedTime);

    // Keep in range of -Pi to Pi
    if (euler[0] > PI) {
        euler[0] = euler[0] - (2 * PI);
    } else if (euler[0] < -PI) {
        euler[0] = euler[0] + (2 * PI);
    }

    // Keep in range of -PI/2 to Pi/2
    if (euler[1] > (PI/2)) {
        euler[1] = euler[1] - PI;
    } else if (euler[1] < (-PI/2)) {
        euler[1] = euler[1] + PI;
    }
    
    
    if (euler[2] > PI) {
        euler[2] = euler[2] - (2 * PI);
    } else if (euler[2] < -PI) {
        euler[2] = euler[2] + (2 * PI);
    }
    

}

void IMU::updateTemp() {

    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(0x41); // First register of 2 temperature registers
    Wire1.endTransmission(false);
    Wire1.requestFrom(IMU_ADDR, 2, true);

    int16_t t = ((int16_t) Wire1.read() << 8 | Wire1.read());

    temp = (((float) t) / TEMP_SCALE) + TEMP_OFFSET;

    if (LPFEnabled) {

        // Shift output values
        filtTemp[1] = filtTemp[0];
        // Compute new output value
        filtTemp[0] = LPFCoeff[0] * temp + LPFCoeff[1] * filtTemp[1];
        
    }

}

void IMU::enableLPF(float cutoffFreqHz, float sampleTimeS) {
    
    LPFEnabled = true;

    // Compute equivalent 'RC' constant from cutoff frequency
    float RC = 1.0 / (6.28318530718 * cutoffFreqHz);

    // Compute filter coefficients
    LPFCoeff[0] = sampleTimeS / (sampleTimeS + RC);
    LPFCoeff[1] = RC / (sampleTimeS + RC);

}

void IMU::disableLPF() {
    
    LPFEnabled = false;

    // Clear coefficient data
    LPFCoeff[0] = 0.0;
    LPFCoeff[1] = 0.0;

}

float IMU::getAcc(int axis) {
    if (LPFEnabled) {
        return filtAcc[axis][0];
    } else {
        return acc[axis];
    }
}

float IMU::getGyro(int axis) {
    if (LPFEnabled) {
        return ((filtGyro[axis][0] * PI) / 180);
    } else {
        return ((gyro[axis] * PI) / 180);
    }
}

float IMU::getAngle(int axis) {
    return angle[axis];
}

float IMU::getEuler(int axis) {
    return euler[axis] * (180 / PI);
}

float IMU::getTemp() {
    if (LPFEnabled) {
        return filtTemp[0];
    } else {
        return temp;
    }
}

int IMU::getID() {
    
    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(0x75); // WHOAMI register
    Wire1.endTransmission(false);
    Wire1.requestFrom(IMU_ADDR, 1, true);

    return Wire1.read();

}

void IMU::calcAccError() {

    int i = 0;
    while (i < ERROR_SAMPLES) {
        updateAcc();
        // Sum values for x and y as they should be 0 when rocket is vertical
        for (size_t j = 0; j < 2; j++) {
            accError[j] = accError[j] + acc[j];
        }

        // Sum values for z plus 1 as value should be -1 when rocket is vertical
        accError[2] = accError[2] + acc[2] + 1;

        delay(10);
        i++;
    }

    for (size_t j = 0; j < 3; j++) {
        accError[j] = accError[j] / ERROR_SAMPLES;
    }
    

}

void IMU::calcGyroError() {

    int i = 0;
    while (i < ERROR_SAMPLES) {
        updateGyro();
        for (size_t j = 0; j < 3; j++) {
            gyroError[j] = gyroError[j] + gyro[j];
        }
        delay(10);
        i++;
    }

    for (size_t j = 0; j < 3; j++) {
        gyroError[j] = gyroError[j] / ERROR_SAMPLES;
    }

}