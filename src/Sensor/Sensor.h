struct Sensor {
  uint32_t pin;
  uint16_t (*read)();
};

void initSensor(struct Sensor*, uint32_t, uint16_t (*)());