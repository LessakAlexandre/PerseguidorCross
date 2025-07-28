#include "gyro.hpp"
float giroscopio::calibrateSensor(){
    int32_t gyroData[3];
        Serial.println("Calibrando o sensor... Mantenha-o imóvel.");
        delay(2000);
        for (int i = 0; i < numSamples; i++) {
            AccGyr.Get_G_Axes(gyroData);
            sumGyro += gyroData[2];
            delay(5);
        }
        Offset = sumGyro / numSamples;
        
        Serial.print("Giroscópio Offsets: ");
        Serial.println(gyroOffset);
        return Offset;
}
float giroscopio::leitura(){
    AccGyr.Get_G_Axes(gyroRaw);
    return gyroRaw[2];
}