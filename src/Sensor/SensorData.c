#include "SensorData.h"
void initSensorData(struct SensorData* this) {
  initFilter(&this->filter, 8, &geometric);
  initSensor(&this->sensor, 0, 0);
}