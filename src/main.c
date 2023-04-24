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

  initMotion();

  while(1) {
    uart.transmit(ledSensor.sensor.read(&ledSensor.sensor));
    leds.blue = ledSensor.diverges(&ledSensor);
    leds.set(&leds);
  }
}

// Convenience methods
void moveForward(void) {
    move(0);
}

void moveRight(void) {
    move(1);
}

void moveBackward(void) {
    move(2);
}

void moveLeft(void) {
    move(3);
}

void move(uint8_t direction) {
    const int FORWARD = 1;
    const int REVERSE = -1;
    const TARGET_RPM = 100;

    struct Motor *fl = motors[0];   // Front left
    struct Motor *fr = motors[1];   // Front right
    struct Motor *rr = motors[2];   // Rear right
    struct Motor *rl = motors[3];   // Rear left

    switch (direction) {
        // Forward
        case 0:
            fl->spinMotor(fl, TARGET_RPM, FORWARD);
            fr->spinMotor(fr, TARGET_RPM, FORWARD);
            rr->spinMotor(rr, TARGET_RPM, FORWARD);
            rl->spinMotor(rl, TARGET_RPM, FORWARD);
            return;
            // Right
        case 1:
            // todo: NEED TO TEST, might need to swap w/ left
            fl->spinMotor(fl, TARGET_RPM, REVERSE);
            fr->spinMotor(fr, TARGET_RPM, FORWARD);
            rr->spinMotor(rr, TARGET_RPM, REVERSE);
            rl->spinMotor(rl, TARGET_RPM, FORWARD);
            return;
            // Backward
        case 2:
            fl->spinMotor(fl, TARGET_RPM, REVERSE);
            fr->spinMotor(fr, TARGET_RPM, REVERSE);
            rr->spinMotor(rr, TARGET_RPM, REVERSE);
            rl->spinMotor(rl, TARGET_RPM, REVERSE);
            return;
            // Left
        case 3:
            // todo: NEED TO TEST, might need to swap w/ right
            fl->spinMotor(fl, TARGET_RPM, FORWARD);
            fr->spinMotor(fr, TARGET_RPM, REVERSE);
            rr->spinMotor(rr, TARGET_RPM, FORWARD);
            rl->spinMotor(rl, TARGET_RPM, REVERSE);
            return;
    }
}