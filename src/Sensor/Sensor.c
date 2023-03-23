#include "Sensor.h"

void initSensor(struct Sensor* this, uint32_t pinNum) {
  this->pin = pinNum;
  this->read = &readSensor;

  GPIOC->MODER &= ~(0xFF);
  GPIOC->MODER |=  (0xFF);
  GPIOC->PUPDR |= ~(0xFF);
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  ADC1->CFGR1 &= ~(1 <<  4);
  ADC1->CFGR1 &=  (1 <<  4);
  ADC1->CFGR1 |= ~(1 << 13);
  ADC1->CFGR1 |=  (1 << 13);

  short analogPin = (1 << 10) | (1 << 13) | (1 << 14);

  ADC1->CHSELR &= ~(analogPin);
  ADC1->CHSELR |=  (analogPin);

  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));

  if ((ADC1->CR & ADC_CR_ADEN) != 0)
    ADC1->CR |= ADC_CR_ADDIS;
  while ((ADC1->CR & ADC_CR_ADEN) != 0) 
    for (volatile uint64_t i = 0; i < 24000; i++) {}

  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  ADC1->CR |= ADC_CR_ADCAL;

  while ((ADC1->CR & ADC_CR_ADCAL) != 0)
    for (volatile uint64_t i = 0; i < 24000; i++) {}

  ADC1->CHSELR &= ~(analogPin);

  ADC1->CR &= ~(1 << 4); // STOP
  ADC1->CR &= ~(1 << 2); // START
  ADC1->CR &= ~(1 << 1); // DIS

  ADC1->CR |= 0x1;

  ADC1->CR |= 1 << 2;
}

uint16_t readSensor(struct Sensor* this) {
  ADC1->CHSELR |= 1 << this->pin;
  uint16_t data = ADC1->DR;
  ADC1->CHSELR &= ~(1 << this->pin);
  return data;
} 