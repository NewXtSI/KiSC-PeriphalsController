#include "thunderv5.h"

#include <SPI.h>

ThunderV5 thunderV5;

void        
ThunderV5::begin(int8_t pinCS, int8_t pinSCK, int8_t pinMOSI, int8_t pinMISO, int8_t pinACK) {
    _pinCS = pinCS;
    _pinSCK = pinSCK;
    _pinMOSI = pinMOSI;
    _pinMISO = pinMISO;
    _pinACK = pinACK;

    pinMode(_pinCS, OUTPUT);
    pinMode(_pinSCK, OUTPUT);
    pinMode(_pinMOSI, OUTPUT);
    pinMode(_pinMISO, INPUT_PULLUP);
    pinMode(_pinACK, INPUT_PULLUP);

    digitalWrite(_pinCS, HIGH);
    digitalWrite(_pinSCK, LOW);
    digitalWrite(_pinMOSI, LOW);


    _running = true;
}

