#include "main.h"

/**
 * Main program.
 * Default: 8MHz clock
 * 
 *          call HSI48_EN if in need of 48MHz clock (for lab purpose, you don't need it! ).
 *          consult rcc perepheral for more info.
 */
int main(void)
{ 
  // Enable system clock to be 1ms per tick
  SysTick_Config(1000);

  // Never returns 
  while(1);
}
