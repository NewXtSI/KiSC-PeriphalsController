#ifndef INCLUDE_PINS_INCLUDED
#define INCLUDE_PINS_INCLUDED

// Analog inputs

#define ANALOGPIN_THROTTLE  35
#define ANALOGPIN_BRAKE     34
#define ANALOGPIN_STEERING  32

// I2C Pins
#define I2C_SDA             21
#define I2C_SCL             22
// secondary I2C
#define I2C_SDA2            17
#define I2C_SCL2            16

// PWM Pins
#define STEERING_SERVO      13

// SPI Pins        5V Rot   3V3 Gelb GND Schwarz
#define SPI_MISO            19      // Gelb
#define SPI_MOSI            23      // Rot
#define SPI_SCK             18      // Weiß
#define SPI_ACK             5       // Weiß
#define SPI_CS              15      // Schwarz
#endif  /* INCLUDE_PINS_INCLUDED */
