#ifndef PID_hpp
#define PID_hpp

class pid{
    private:
      float kp;
      float ki;
      float kd;
      float integral = 0.0;
      float lastError = 0.0;
      float posicaoDesejada;
      float lastPosicao = posicaoDesejada;
    public:
      pid(float KP, float KI , float KD,float POSICAO_DESEJADA );
      float corretcion(volatile float posicao,float dt);

};

#endif