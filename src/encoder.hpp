#ifndef ENCODER_HPP
#define ENCODER_HPP

    #include <AS5047P.h>
class encoder{
    private:
        int PIN;
        int SPI_BUS_SPEED = 10000000;
        AS5047P ENCODER;
    public:
        encoder(int PIN_ENCODER);
        float leitura();
        bool SPI();
    };

#endif