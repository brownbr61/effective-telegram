#define NUM_SENSORS 3

struct Sensor {
  uint32_t pin;
  uint16_t (*read)(struct Sensor*);
};

struct Sensor sensors[NUM_SENSORS];

void initSensor(struct Sensor*, uint32_t);
uint16_t readSensor(struct Sensor*);
