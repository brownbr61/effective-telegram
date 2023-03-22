#include "Sensor.h"

void initSensor(struct Sensor* this, uint32_t pinNum, uint16_t (*readFunction)()) {
  this->pin = pinNum;
  this->read = readFunction;
}