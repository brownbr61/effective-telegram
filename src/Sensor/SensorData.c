#include "SensorData.h"
#include <stdlib.h>

void initSensorData(struct SensorData* this) {
  initFilter(&this->filter, 8, &geometric);
  initSensor(&this->sensor, 0);
  this->get = &getSensorData;
  this->setMean = &setMean;
  this->setVar = &setVar;
  this->setMean(this);
  this->setVar(this);
}

uint64_t getSensorData(struct SensorData* this) {
  return this->filter.filter(&this->filter,this->sensor.read(&this->sensor));
}

void setMean(struct SensorData* this) {
  for(uint16_t i = 0; i < (1 <<this->filter.shft); i++)
    this->filter.filter(&this->filter,this->sensor.read(&this->sensor));

  for(uint16_t i = 0; i < (1 << this->filter.shft); i++)
    this->mean += this->filter.filter(&this->filter,this->sensor.read(&this->sensor));
  this->mean = this->mean << this->filter.shft;
}

void setVar(struct SensorData* this) {
  for(uint16_t i = 0; i < (1 <<this->filter.shft); i++)
    this->var += abs(
        this->filter.filter(&this->filter,this->sensor.read(&this->sensor))
        - this->mean);
  this->var = (this->var >> this->filter.shft);
}

bool diverges(struct SensorData* this) {
  return abs(this->filter.fOut - this->mean) > this->var;
}