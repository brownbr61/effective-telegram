// #include "Filter.h"
// #include "Sensor.h"

struct SensorData {
  struct Sensor sensor;
  struct Filter filter;
  uint64_t threshold; // may need to create a comparator logic class
  uint64_t (*get)(struct SensorData*);
};

void initSensorData(struct SensorData*);
void getSensorData(struct SensorData*);