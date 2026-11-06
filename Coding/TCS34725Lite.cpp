#include "TCS34725Lite.h"

#define TCS34725_ADDRESS      0x29
#define TCS34725_COMMAND_BIT  0x80
#define TCS34725_AUTOINC      0xA0   // command bit + auto-increment
#define TCS34725_ENABLE       0x00
#define TCS34725_ATIME        0x01
#define TCS34725_CONTROL      0x0F
#define TCS34725_ID           0x12
#define TCS34725_STATUS       0x13
#define TCS34725_CDATAL       0x14

TCS34725Lite::TCS34725Lite(uint8_t sdaPin, uint8_t sclPin) {
  _sda = sdaPin;
  _scl = sclPin;
  _atime = 0xEB;
}

// ---------------- Low-level bit-banged I2C ----------------

void TCS34725Lite::i2cBegin() {
  pinMode(_sda, INPUT_PULLUP);
  pinMode(_scl, INPUT_PULLUP);
}

// Release SCL and wait (with timeout) for the slave to stop clock stretching
void TCS34725Lite::sclRelease() {
  pinMode(_scl, INPUT_PULLUP);
  uint8_t guard = 0;
  while (digitalRead(_scl) == LOW && guard < 250) {
    delayMicroseconds(2);
    guard++;
  }
}

void TCS34725Lite::i2cStart() {
  pinMode(_sda, INPUT_PULLUP);
  sclRelease();
  delayMicroseconds(4);
  pinMode(_sda, OUTPUT); digitalWrite(_sda, LOW);
  delayMicroseconds(4);
  pinMode(_scl, OUTPUT); digitalWrite(_scl, LOW);
  delayMicroseconds(4);
}

void TCS34725Lite::i2cStop() {
  pinMode(_sda, OUTPUT); digitalWrite(_sda, LOW);
  delayMicroseconds(4);
  sclRelease();
  delayMicroseconds(4);
  pinMode(_sda, INPUT_PULLUP);
  delayMicroseconds(4);
}

bool TCS34725Lite::i2cWriteByte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80) {
      pinMode(_sda, INPUT_PULLUP);
    } else {
      pinMode(_sda, OUTPUT); digitalWrite(_sda, LOW);
    }
    data <<= 1;
    delayMicroseconds(2);
    sclRelease();
    delayMicroseconds(4);
    pinMode(_scl, OUTPUT); digitalWrite(_scl, LOW);
    delayMicroseconds(2);
  }
  // ACK bit
  pinMode(_sda, INPUT_PULLUP);
  delayMicroseconds(2);
  sclRelease();
  delayMicroseconds(2);
  bool ack = (digitalRead(_sda) == LOW);
  pinMode(_scl, OUTPUT); digitalWrite(_scl, LOW);
  delayMicroseconds(2);
  return ack;
}

uint8_t TCS34725Lite::i2cReadByte(bool ack) {
  uint8_t data = 0;
  pinMode(_sda, INPUT_PULLUP);
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    sclRelease();
    delayMicroseconds(2);
    if (digitalRead(_sda) == HIGH) data |= 1;
    pinMode(_scl, OUTPUT); digitalWrite(_scl, LOW);
    delayMicroseconds(2);
  }
  if (ack) {
    pinMode(_sda, OUTPUT); digitalWrite(_sda, LOW);
  } else {
    pinMode(_sda, INPUT_PULLUP);
  }
  delayMicroseconds(2);
  sclRelease();
  delayMicroseconds(2);
  pinMode(_scl, OUTPUT); digitalWrite(_scl, LOW);
  delayMicroseconds(2);
  pinMode(_sda, INPUT_PULLUP);
  return data;
}

// ---------------- Register access ----------------

void TCS34725Lite::writeRegister(uint8_t reg, uint8_t value) {
  i2cStart();
  i2cWriteByte((TCS34725_ADDRESS << 1) | 0);
  i2cWriteByte(TCS34725_COMMAND_BIT | reg);
  i2cWriteByte(value);
  i2cStop();
}

uint8_t TCS34725Lite::read8(uint8_t reg) {
  i2cStart();
  i2cWriteByte((TCS34725_ADDRESS << 1) | 0);
  i2cWriteByte(TCS34725_COMMAND_BIT | reg);
  i2cStop();

  i2cStart();
  i2cWriteByte((TCS34725_ADDRESS << 1) | 1);
  uint8_t v = i2cReadByte(false);
  i2cStop();
  return v;
}

uint16_t TCS34725Lite::read16(uint8_t reg) {
  i2cStart();
  i2cWriteByte((TCS34725_ADDRESS << 1) | 0);
  i2cWriteByte(TCS34725_AUTOINC | reg);
  i2cStop();

  i2cStart();
  i2cWriteByte((TCS34725_ADDRESS << 1) | 1);
  uint8_t low  = i2cReadByte(true);
  uint8_t high = i2cReadByte(false);
  i2cStop();

  return ((uint16_t)high << 8) | low;
}

// ---------------- Sensor control ----------------

bool TCS34725Lite::begin() {
  i2cBegin();

  uint8_t id = read8(TCS34725_ID);
  if (id != 0x44 && id != 0x10 && id != 0x4D) return false;  // wiring / wrong chip

  writeRegister(TCS34725_ENABLE, 0x01);   // PON
  delay(3);
  writeRegister(TCS34725_ENABLE, 0x03);   // PON + AEN
  setIntegrationTime(0xEB);               // ~50 ms
  setGain(0x01);                          // 4x
  delay(60);                              // let the first integration finish
  return true;
}
void TCS34725Lite::setIntegrationTime(uint8_t itime) {
  _atime = itime;
  writeRegister(TCS34725_ATIME, itime);
}

void TCS34725Lite::setGain(uint8_t gain) {
  writeRegister(TCS34725_CONTROL, gain);
}

void TCS34725Lite::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  // Wait for a completed integration (AVALID bit)
  uint32_t t = millis();
  while (!(read8(TCS34725_STATUS) & 0x01)) {
    if (millis() - t > 120) break;
  }
  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_CDATAL + 2);
  *g = read16(TCS34725_CDATAL + 4);
  *b = read16(TCS34725_CDATAL + 6);
}

// ---------------- Color decision ----------------

bool TCS34725Lite::isGreen(uint16_t thresholdC, float minGreenRatio, float margin) {
  const uint8_t N = 4;
  uint32_t R = 0, G = 0, B = 0, C = 0;

  for (uint8_t i = 0; i < N; i++) {
    uint16_t r, g, b, c;
    getRawData(&r, &g, &b, &c);
    R += r; G += g; B += b; C += c;
  }
  R /= N; G /= N; B /= N; C /= N;

  // Max count = (256 - ATIME) * 1024, capped at 65535. Treat 90% of it as saturated.
  uint32_t maxCount = (uint32_t)(256 - _atime) * 1024UL;
  if (maxCount > 65535UL) maxCount = 65535UL;
  uint32_t satLimit = (maxCount * 9UL) / 10UL;

  if (C < thresholdC || C >= satLimit) return false;   // too dark or saturated

  float sum = (float)(R + G + B);
  if (sum <= 0.0f) return false;

  float gRatio = (float)G / sum;
  bool gOverR = ((float)G - (float)R) > margin * (float)C;
  bool gOverB = ((float)G - (float)B) > margin * (float)C;

  return (gRatio > minGreenRatio) && gOverR && gOverB;
}