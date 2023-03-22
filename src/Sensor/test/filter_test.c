#include <stdint.h>
#include "filter_test_data.c"
#include "../Filter.c"
#include <stdio.h>
#include <stdbool.h>

bool basicBitchTest() {
  struct Filter basicBitchFilter;
  initFilter(&basicBitchFilter, 7, geometric);
  bool failure = 0;
  for (int i = 0; i < 256; i++) {
    if(basicBitchOut[i] != basicBitchFilter.filter(&basicBitchFilter,basicBitchIn[i])) {
      printf("%u: %u %lu\n",i, basicBitchOut[i], basicBitchFilter.fOut);
      failure = 1;
    }
  }
  return failure;
}

bool Test(bool (*test)(), char* testName) {
  bool failure = test();
  printf("Running test: %s\n",testName);
  if(failure)
    printf("FAIL!\n\n");
  else
    printf("PASS!\n\n");
  return failure;
}

int main() {
  printf("running tests!\n\n");
  int failCount = 0;
  failCount += Test(&basicBitchTest, "src/Sensor/test/filter_test.c: basicBitchTest()");
  printf("%u failures\n\n",failCount);
  return failCount;
}