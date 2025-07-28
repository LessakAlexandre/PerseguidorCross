#ifndef GYRO_HPP
#define GYRO_HPP

    #include <Arduino.h>
    #include <LSM6DSRSensor.h>
//------------Definições----------//
    //GIROSCOPIO
        volatile float g_angle1 =     0.0f;
        volatile float g_angle2 =     0.0f;

        volatile float g_lastAngle1 = 0.0f;
        volatile float g_lastAngle2 = 0.0f;

        volatile float g_totalDisp1 = 0.0f; // cm
        volatile float g_totalDisp2 = 0.0f; // cm

    // Giroscópio (Z) em graus/s (dps)
        volatile float g_gyroZ_dps =  0.0f;
        float gyroOffset =            0;
      //I2C
        #define I2C_SDA               5
        #define I2C_SCL               4
        #define LSM6DSR_ADDRESS LSM6DSR_I2C_ADD_L
        TwoWire I2C_BUS = TwoWire(0);
        LSM6DSRSensor AccGyr(&I2C_BUS, LSM6DSR_ADDRESS);

class giroscopio{
    private:
        int32_t gyroRaw[3];
        const int numSamples = 1000;  
        float sumGyro = 0;
        float Offset;
    public:
        float calibrateSensor();
        float leitura();
    };


#endif