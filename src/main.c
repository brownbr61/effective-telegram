#include "main.h"

/**
 * Main program.
 * Default: 8MHz clock
 * 
 *          call HSI48_EN if in need of 48MHz clock (for lab purpose, you don't need it! ).
 *          consult rcc perepheral for more info.
 */
#define THRESHOLD_DIST 10
#define NO_TURN 0
#define LEFT_TURN 1
#define RIGHT_TURN 2

struct UART_INT uart;
int main(void) {
    uint8_t DRAG_MODE = 0;

    HSI48_EN();
    // Enable system clock to be 1ms per tick
    SysTick_Config(1000);

    struct LEDs leds;
    struct SensorData ledSensor;

    initLEDs(&leds);
    initUart(&uart);
    initMotion(&leds, &uart);

    // todo: test this works with LEDs
    initSensors();

    if (DRAG_MODE) {
        moveForward();
    } else {
        moveForward();
        hokeyPokey();
    }

}

void hokeyPokey() {
    struct Sensor *left_sensor = &(sensors[0]);
    struct Sensor *center_sensor = &(sensors[1]);
    struct Sensor *right_sensor = &(sensors[2]);

    // Poll values in sensors
    while(1) {
        uint16_t left_dist = left_sensor->read();
        uint16_t center_dist = center_sensor->read();
        uint16_t right_dist = right_sensor->read();

        // Stop if any of our sensors have a close value
        if ((left > THRESHOLD_DIST) || (center > THRESHOLD_DIST) || (right > THRESHOLD_DIST)) {
            stop();
        }

        int leftDetect = left_dist - THRESHOLD_DIST > 0;
        int centerDetect = center_dist = THRESHOLD_DIST > 0;
        int rightDetect = right_dist - THRESHOLD_DIST > 0;

        // In ms
        int shortTime = 500;
        int longTime = 1000;

        if (leftDetect && centerDetect && rightDetect) {
            turnRight();
            moveForward();
            HAL_Delay(shortTime);
            stop();
            turnLeft();
        } else if (leftDetect && centerDetect) {
            turnRight();
            moveForward();
            HAL_Delay(longTime);
            stop();
            turnLeft();
        } else if (rightDetect && centerDetect) {
            turnLeft();
            moveForward();
            HAL_Delay(longTime);
            stop();
            turnRight();
        } else if (leftDetect && rightDetect) {
            turnRight();
            moveForward();
            HAL_Delay(longTime);
            stop();
            turnLeft();
        } else if (leftDetect) {
            turnRight();
            moveForward();
            HAL_Delay(shortTime);
            stop();
            turnLeft();
        } else if (centerDetect) {
            turnRight();
            moveForward();
            HAL_Delay(shortTime);
            stop();
            turnLeft();
        } else if (rightDetect) {
            turnLeft();
            moveForward();
            HAL_Delay(shortTime);
            stop();
            turnRight();
        } else {
            moveForward();
        }
        // todo: add delay here
    }
}

// Finds left wall and aligns itself towards finish line, but does not move after
void findLeftWall() {
    moveForward();
    // Delay 1 seconds
    stop();
    turnLeft();
    moveForward();

    int foundLeftWall = 0;
    while (!foundLeftWall) {
        uint16_t left_dist = left_sensor->read();
        uint16_t center_dist = center_sensor->read();
        uint16_t right_dist = right_sensor->read();

        // Stop if any of our sensors have a close value
        if ((left > THRESHOLD_DIST) || (center > THRESHOLD_DIST) || (right > THRESHOLD_DIST)) {
            stop();
        }

        int leftDetect = left_dist - THRESHOLD_DIST > 0;
        int centerDetect = center_dist = THRESHOLD_DIST > 0;
        int rightDetect = right_dist - THRESHOLD_DIST > 0;

        // Todo: we need to calibrate center detect such that this picks up correctly
        if (centerDetect) {
            foundLeftWall = 1;
        }
        // todo: add delay
    }
    stop();
    turnRight();
}

// Convenience methods
void stop() {
    move(0);
}

void moveForward() {
    move(1);
}

void turnRight() {
    move(2);
}

void moveBackward() {
    move(3);
}

void turnLeft() {
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

void initSensors() {
    uint32_t sensorPins[3] = {0, 1, 2};
    for (int i = 0; i < NUM_SENSORS; i++) {
        struct Sensor *sensor = &(sensors[i]);
        initSensor(sensor, sensorPins[i]);
    }
    // todo: may need to init filter here too
}