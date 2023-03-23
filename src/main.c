#include "main.h"

/**
 * Main program.
 * Default: 8MHz clock
 * 
 *          call HSI48_EN if in need of 48MHz clock (for lab purpose, you don't need it! ).
 *          consult rcc perepheral for more info.
 */

struct UART_INT uart;
int main(void)
{ 
  HSI48_EN();
  // Enable system clock to be 1ms per tick
  SysTick_Config(1000);

  struct LEDs leds;
  struct SensorData ledSensor;

  initLEDs(&leds);
  initUart(&uart);
  initSensorData(&ledSensor);


  leds.red = 1;
  leds.blue = 1;
  leds.green = 0;
  leds.orange = 1;

  leds.set(&leds);

  while(1) {
    uart.transmit(93);
    leds.green = ledSensor.diverges(&ledSensor);
    leds.set(&leds);
  }
}
