struct Filter {
  uint64_t fOut;
  uint8_t shft;
  uint64_t (*filter)(struct Filter*, uint16_t fIn);
};

void initFilter(struct Filter*, uint8_t, uint64_t (*this)(struct Filter*, uint16_t));

uint64_t geometric(struct Filter*, uint16_t fIn);