#include <stdint.h>
#include "filter_test_data.c"
#include "../Filter.c"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

void initDatOut(char* str,char* testName, int len) {
  char* s = str;
  while (s - str < len)
    *(s++) = 0;
  s = str;
  strcat(s,"data/out/");
  strcat(s,testName);
  strcat(s,".csv");
}

bool basicBitchTest(FILE* fptr) {
  struct Filter basicBitchFilter;
  initFilter(&basicBitchFilter, 7, geometric);
  bool failure = 0;
  for (int i = 0; i < 256; i++) {
    fprintf(fptr,"%lu\n",basicBitchFilter.fOut);
    if(basicBitchOut[i] != basicBitchFilter.filter(&basicBitchFilter,basicBitchIn[i])) {
      printf("%u: %u %lu\n",i, basicBitchOut[i], basicBitchFilter.fOut);
      failure = 1;
    }
  }
  return failure;
}

bool led_off_test(FILE* fptr) {
  // init & define filter
  struct Filter led_off_filter;
  initFilter(&led_off_filter, 8, geometric);
  uint16_t *filterIn = led_off_in;
  uint64_t *filterOut = led_off_out;
  uint64_t arrayLength = 70669;

  //set return value
  bool failure = 0;

  // do test
  for (uint64_t i = 0; i < arrayLength /* size of array*/; i++) {
    // do filter
    led_off_filter.filter(&led_off_filter,filterIn[i]);

    // print filter in, output to *.csv
    fprintf(fptr,"%u, %lu\n",filterIn[i],led_off_filter.fOut);
    
    // report error on command line
    if(led_off_out[i] != led_off_filter.fOut) {
      printf("%lu: %lu %lu\n",i, filterOut[i], led_off_filter.fOut);
      failure = 1;
    }
  }

  return failure;
}

bool Test(bool (*test)(FILE *fptr), char* testName) {
  char fileName[100];
  initDatOut(fileName,testName,100);
  FILE *fptr = fopen(fileName,"w");
  printf("Running test: src/Sensor/test/%s\n",testName);
  bool failure = test(fptr);
  if(failure)
    printf("FAIL!\n\n");
  else
    printf("PASS!\n\n");
  fclose(fptr);
  return failure;
}

int main() {
  printf("running tests!\n\n");
  int failCount = 0;

  failCount += Test(&basicBitchTest, "filter_test.c.basicBitchTest");
  failCount += Test(&led_off_test, "filter_test.c.led_off_test");

  printf("%u failures\n\n",failCount);
  return failCount;
}