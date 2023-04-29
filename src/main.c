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

  initMotion(&leds, &uart);
  moveForward();
  stop();

//  leds.blue = 1;
//  leds.set(&leds);


//   while(1) {
//     uart.transmit(ledSensor.sensor.read(&ledSensor.sensor));
//     leds.blue = ledSensor.diverges(&ledSensor);
//     leds.set(&leds);
//   }
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