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
  leds.red = 1;
  leds.set(&leds);

  initUart(&uart);
  leds.orange = 1;
  leds.set(&leds);

  initSensorData(&ledSensor);
  leds.green = 1;
  leds.set(&leds);

  // todo: init motors and drivers

  while(1) {
    uart.transmit(ledSensor.sensor.read(&ledSensor.sensor));
    leds.blue = ledSensor.diverges(&ledSensor);
    leds.set(&leds);
  }
}
