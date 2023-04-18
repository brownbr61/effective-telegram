struct Filter {
  uint64_t fOut;
  uint8_t shft;
  uint64_t (*filter)(struct Filter*, uint16_t fIn);
};

void initFilter(struct Filter* this, 
                uint8_t shift, 
                uint64_t (*filterFunction)(struct Filter*, uint16_t), 
                uint16_t dIn);

uint64_t geometric(struct Filter*, uint16_t fIn);