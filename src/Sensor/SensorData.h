
struct SensorData {
  struct Sensor sensor;
  struct Filter filter;
  uint64_t mean; // may need to create a comparator logic class
  uint8_t var;
  uint64_t (*get)(struct SensorData*);
  void (*setMean)(struct SensorData*);
  void (*setVar)(struct SensorData*);
  bool (*diverges)(struct SensorData*);
};

void initSensorData(struct SensorData*);
uint64_t getSensorData(struct SensorData*);
void setMean(struct SensorData*);
void setVar(struct SensorData*);
bool diverges(struct SensorData*);