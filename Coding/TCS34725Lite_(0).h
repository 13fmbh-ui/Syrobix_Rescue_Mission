#ifndef TCS34725_LITE_H
#define TCS34725_LITE_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class TCS34725Lite {
public:
  TCS34725Lite(uint8_t sdaPin, uint8_t sclPin);

  bool begin();
  void setIntegrationTime(uint8_t itime);
  void setGain(uint8_t gain);
  void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
  bool isGreen(uint16_t thresholdC = 200, float minGreenRatio = 0.35f, float margin = 0.05f);

private:
  uint8_t _sda;
  uint8_t _scl;
  uint8_t _atime;

  // Low-level I2C methods
  void i2cBegin();
  void sclRelease();
  void i2cStart();
  void i2cStop();
  bool i2cWriteByte(uint8_t data);
  uint8_t i2cReadByte(bool ack);

  // Register access methods
  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);
  uint16_t read16(uint8_t reg);
};

#endif