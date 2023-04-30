#include "main.h"

/**
 * Main program.
 * Default: 8MHz clock
 * 
 *          call HSI48_EN if in need of 48MHz clock (for lab purpose, you don't need it! ).
 *          consult rcc perepheral for more info.
 */

struct UART_INT uart;
int main(void) {
    HSI48_EN();
    // Enable system clock to be 1ms per tick
    SysTick_Config(1000);

    struct LEDs leds;
    struct SensorData ledSensor;

  initLEDs(&leds);
  leds.red = 1;
  leds.set(&leds);
  for (volatile int i = 0; i < 2400000; i++) {}

  initUart(&uart);
  leds.orange = 1;
  leds.set(&leds);
  for (volatile int i = 0; i < 2400000; i++) {}

    initSensorData(&ledSensor);

  initMotion(&leds);

  leds.green = 1;
  leds.set(&leds);
  for (volatile int i = 0; i < 2400000; i++) {}

  uart.transmit(0);
  uart.transmit(1);

  // uint16_t tmp;
  int i = 0;

  while(1) {
    // tmp = (uint16_t)(ledSensor.sensor.read(&ledSensor.sensor));
    // ledSensor.filter.filter(&ledSensor.filter, tmp);
    // uart.transmit(tmp);
    if ((i++ % 1200) != 0) {
      leds.blue = leds.blue == 0;
      // uart.transmit(ledSensor.filter.fOut);
    }
    leds.set(&leds);
  }
  // initMotion(&leds);
  leds.blue = 1;
  leds.set(&leds);
  DELAY(5000);

  moveForward();
  stop();

    // Check to see it Pending Reg is actually being set for tick interrupts
    while(1) {
        if (EXTI->PR & 0x00000008) {
            leds.orange = 1;
            leds.set(&leds);
        }
        if (EXTI->PR & 0x00000004) {
            leds.red = 1;
            leds.set(&leds);
        }
    }
}

// Convenience methods
void stop() {
    move(0);
}

void moveForward() {
    move(1);
}

void moveRight() {
    move(2);
}

void moveBackward() {
    move(3);
}

void moveLeft() {
    move(4);
}

// Actual movement logic
void move(uint8_t direction) {
    const int FORWARD = 1;
    const int REVERSE = -1;
    const int TARGET_RPM = 100; // todo: placeholder

    struct Motor *left = &motors[0];
    struct Motor *right = &motors[1];

    switch (direction) {
        case 0: // Stop
            left->stop(left);
            right->stop(right);
            return;
        case 1: // Forward
            left->spin(left, TARGET_RPM, FORWARD);
            right->spin(right, TARGET_RPM, FORWARD);
            return;
        case 2: // Right
            left->spin(left, TARGET_RPM, FORWARD);
            right->spin(right, TARGET_RPM, REVERSE);
            return;
        case 3: // Backward
            left->spin(left, TARGET_RPM, REVERSE);
            right->spin(right, TARGET_RPM, REVERSE);
            return;
        case 4: // Left
            left->spin(left, TARGET_RPM, REVERSE);
            right->spin(right, TARGET_RPM, FORWARD);
    }
}