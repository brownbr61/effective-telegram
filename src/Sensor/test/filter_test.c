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
  printf("%u failures\n\n",failCount);
  return failCount;
}