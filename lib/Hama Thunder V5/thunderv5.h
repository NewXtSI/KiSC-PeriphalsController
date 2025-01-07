#ifndef INCLUDE_THUNDERV5_INCLUDED
#define INCLUDE_THUNDERV5_INCLUDED

#include "Arduino.h"
#include <functional>

class ThunderV5 {
 public:
                    ThunderV5();
        void        begin(int8_t pinCS, int8_t pinSCK, int8_t pinMOSI, int8_t pinMISO, int8_t pinACK);
        void        end();
        void        loop();
 private:
        int8_t      _pinCS = -1;
        int8_t      _pinSCK = -1;
        int8_t      _pinMOSI = -1;
        int8_t      _pinMISO = -1;
        int8_t      _pinACK = -1;

        bool        _running = false;
        bool        waitAck(uint8_t level = LOW, uint16_t timeoutus = 1000);
        uint8_t     transfer(uint8_t data);

        uint8_t     startTransfer();
        void        endTransfer();
};

extern ThunderV5 thunderV5;
#endif  /* INCLUDE_THUNDERV5_INCLUDED */
