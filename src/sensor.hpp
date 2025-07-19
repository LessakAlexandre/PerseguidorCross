#ifndef SENSOR_HPP
#define SENSOR_HPP

#define NUM_SENSORES 16

//---------------------------CLASSES------------------------------------//
class sensor{
    private:
        float lastPosicao;
        long somaPonderada = 0;
        long somaValores = 0;
        int sensorMin[NUM_SENSORES];
        int sensorMax[NUM_SENSORES];
        int sensorValues[NUM_SENSORES];
        int sensorPos[NUM_SENSORES] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100};
    public:
        sensor(float PosicaoInicial);
        void calibracao();
        volatile float leitura();
};

#endif