// #include "stm32f072xb.h"

struct LEDs {
  uint8_t red;
  uint8_t orange;
  uint8_t green;
  uint8_t blue;
  void (*set)(struct LEDs*);
};

void setLEDs(struct LEDs* this) {
  GPIOC->ODR &= ((this->green << 9) | (this->orange << 8) | (this->blue << 7) | (this->red << 6));
  GPIOC->ODR |= ((this->green << 9) | (this->orange << 8) | (this->blue << 7) | (this->red << 6));
}

void initLEDs(struct LEDs* this) {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xFF000);
  GPIOC->MODER |= 0x55000;
  this->red = 0;
  this->orange = 0;
  this->green = 0;
  this->blue = 0;
  this->set = &setLEDs;
}