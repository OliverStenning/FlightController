#ifndef IMU_H
#define IMU_H

class IMU {
    private:
        bool LPFEnabled, CFEnabled;
        float LPFCoeff[2]; 
        float previousTime, currentTime, elapsedTime;

    public:
        float acc[3];
        float gyro[3];
        float temp;

        float filtAcc[3][2];
        float filtGyro[3][2];
        float filtTemp[2];

        float accAngle[3];
        float angle[3];
        float euler[3];

        float accError[3];
        float gyroError[3];

        IMU();
        ~IMU();

        void enable();
        void update();

        void updateAcc();
        void updateGyro();
        void updateTemp();

        void enableLPF(float cutoffFreqHz, float sampleTimeS);
        void disableLPF();

        float getAcc(int axis);
        float getGyro(int axis);
        float getAngle(int axis);
        float getEuler(int axis);
        float getTemp();

        int getID();
        void calcAccError();
        void calcGyroError();
};

#endif