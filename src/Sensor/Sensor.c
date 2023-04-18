#include "Sensor.h"

void calibrate_ADC_manual() {    
  // Ensure ADEN = 0
  if ((ADC1->CR & ADC_CR_ADEN) != 0) {
    // Clear ADEN
    ADC1->CR |= ADC_CR_ADDIS;
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0) {
    // HAL_Delay(500);
  }
  // Clear DMAEN
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  
  // Launch calibration
  ADC1->CR |= ADC_CR_ADCAL;
  
  // Wait until ADCAL = 0
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
    // HAL_Delay(500);
  }


  // if ((ADC1->CR & ADC_CR_ADEN) != 0)
  //   ADC1->CR |= ADC_CR_ADDIS;
  // while ((ADC1->CR & ADC_CR_ADEN) != 0) 
  //   for (volatile uint64_t i = 0; i < 24000; i++) {}

  // ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  // ADC1->CR |= ADC_CR_ADCAL;

  // while ((ADC1->CR & ADC_CR_ADCAL) != 0)
  //   for (volatile uint64_t i = 0; i < 24000; i++) {}
}

void initSensor(struct Sensor* this, uint32_t pinNum) {
  this->pin = pinNum;
  this->read = &readSensor;
   	 
  // Configure PC0 to analog mode
  GPIOC->MODER &= ~(0x3);   		 // Clear
  GPIOC->MODER |= 0x3;
  
  // Enable ADC1 in RCC
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  /*** Configure ADC ***/
  // Set 8-bit resolution
  ADC1->CFGR1 |= (1 << 4);
    
  // Turn off hardware triggers
  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));
    
  // Turn on continuous conversion mode
  ADC1->CFGR1 |= (1 << 13);

  // short analogPin = (1 << 10) | (1 << 13) | (1 << 14);
  short analogPin = (1 << 10);

  ADC1->CHSELR &= ~(analogPin);
  ADC1->CHSELR |=  (analogPin);

  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));

  calibrate_ADC_manual();

  ADC1->CHSELR &= ~(analogPin);
  ADC1->CHSELR |= 1 << 10;
    
  // Ensure ADSTP, ADSTART, ADDIS are = 0
  ADC1->CR &= ~(1 << 4); //STP
  ADC1->CR &= ~(1 << 2); //START
  ADC1->CR &= ~(1 << 1); //DIS

  ADC1->CR |= 0x1;

  ADC1->CR |= 1 << 2;
}

uint16_t readSensor(struct Sensor* this) {
  ADC1->CHSELR |= 1 << this->pin;
  // for(volatile int i = 0; i < 2400; i++) {}
  uint16_t data = ADC1->DR;
  ADC1->CHSELR &= ~(1 << this->pin);
  return data;
} 