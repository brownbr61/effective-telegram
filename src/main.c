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
  leds.green = 1;
  leds.set(&leds);
  DELAY(500);
  leds.green = 0;
  leds.set(&leds);

  initMotion(&leds);
  leds.blue = 1;
  leds.set(&leds);

  moveForward();
  DELAY(5000);
  stop();

//   while(1) {
//     uart.transmit(ledSensor.sensor.read(&ledSensor.sensor));
//     leds.blue = ledSensor.diverges(&ledSensor);
//     leds.set(&leds);
//   }
}

// Convenience methods
void stop(void) {
    move(0);
}

void moveForward(void) {
    move(1);
}

void moveRight(void) {
    move(2);
}

void moveBackward(void) {
    move(3);
}

void moveLeft(void) {
    move(4);
}

// Actual movement logic
void move(uint8_t direction) {
    const int FORWARD = 1;
    const int REVERSE = -1;
    const int TARGET_RPM = 100; // todo: placeholder

    struct Motor *fl = &motors[0];   // Front left
    struct Motor *fr = &motors[1];   // Front right
    struct Motor *rr = &motors[2];   // Rear right
    struct Motor *rl = &motors[3];   // Rear left

    switch (direction) {
        case 0: // Stop
            fl->stop(fr);
            fr->stop(fr);
            rr->stop(rr);
            rl->stop(rl);
            return;
        case 1: // Forward
            fl->spin(fl, TARGET_RPM, FORWARD);
            fr->spin(fr, TARGET_RPM, FORWARD);
            rr->spin(rr, TARGET_RPM, FORWARD);
            rl->spin(rl, TARGET_RPM, FORWARD);
            return;
        case 2: // Right
            // todo: NEED TO TEST, might need to swap w/ left
            fl->spin(fl, TARGET_RPM, REVERSE);
            fr->spin(fr, TARGET_RPM, FORWARD);
            rr->spin(rr, TARGET_RPM, REVERSE);
            rl->spin(rl, TARGET_RPM, FORWARD);
            return;
        case 3: // Backward
            fl->spin(fl, TARGET_RPM, REVERSE);
            fr->spin(fr, TARGET_RPM, REVERSE);
            rr->spin(rr, TARGET_RPM, REVERSE);
            rl->spin(rl, TARGET_RPM, REVERSE);
            return;
        case 4: // Left
            // todo: NEED TO TEST, might need to swap w/ right
            fl->spin(fl, TARGET_RPM, FORWARD);
            fr->spin(fr, TARGET_RPM, REVERSE);
            rr->spin(rr, TARGET_RPM, FORWARD);
            rl->spin(rl, TARGET_RPM, REVERSE);
    }
}