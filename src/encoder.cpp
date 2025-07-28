#include "encoder.hpp"

encoder::encoder(int PIN_ENCODER):ENCODER(PIN_ENCODER,SPI_BUS_SPEED){  
    PIN = PIN_ENCODER;
}
float encoder::leitura(){
    return ENCODER.readAngleDegree();
}
bool encoder::SPI(){
    return ENCODER.initSPI();
}