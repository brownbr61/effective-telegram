struct Sensor {
  uint32_t pin;
  uint16_t (*read)(struct Sensor*);
};

void initSensor(struct Sensor*, uint32_t);
uint16_t readSensor(struct Sensor*);