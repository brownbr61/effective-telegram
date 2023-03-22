#include "Filter.h"

uint64_t geometric(struct Filter* this, uint16_t fIn) {
  this->fOut += fIn - (this->fOut >> this->shft);
  return this->fOut;
}

void initFilter(struct Filter* this, uint8_t shift, uint64_t (*filterFunction)(struct Filter*, uint16_t)) {
  this->shft = shift;
  this->fOut = 0;
  this->filter = filterFunction;
}