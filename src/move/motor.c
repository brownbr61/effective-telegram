#include "motor.h"
#include "../main.h"

// Sets up the entire motor drive system
void initMotion(struct LEDs *leds_in, struct UART_INT *uart_in) {
    leds = leds_in;
    uart_ptr = uart_in;

    initMotors();
    initPWM();
    initEncoders();
}

/* Initializes all motors in the forward direction in a stopped state */
void initMotors() {
    int pwm_in_pins[] = {0, 1};
    int mtr_A_dir_pins[] = {4, 5};
    int mtr_B_dir_pins[] = {6, 7};

    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &(motors[i]);
        motor->id = i + 1; // Range is 1-2
        motor->direction = 1;
        motor->target_ticks = 100;
        motor->num_ticks = 0;
        motor->error = 0;
        motor->error_integral = 0;

        motor->setCycle = &pwm_setDutyCycle;
        motor->correctError = &PI_update;
        motor->spin = &spinMotor;
        motor->stop = &stopMotor;

        motor->pwmGpio = GPIOA;
        motor->pwm_in_pin = pwm_in_pins[i];

        motor->dirGpio = GPIOB;
        switch (i) {
            case 0:
                motor->dir_pin_A = mtr_A_dir_pins[0];
                motor->dir_pin_B = mtr_A_dir_pins[1];
                break;
            case 1:
                motor->dir_pin_A = mtr_B_dir_pins[0];
                motor->dir_pin_B = mtr_B_dir_pins[1];
                break;
        }
    }
}

// Sets up the PWM and direction signals to drive the H-Bridge
void initPWM() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Set up pin PA0-1 for H-bridge PWM output (TIMER 2 CH1-2)
    GPIOA->MODER |= (1 << 1);
    GPIOA->MODER &= ~(1 << 0);
    GPIOA->MODER |= (1 << 3);
    GPIOA->MODER &= ~(1 << 2);

    // Set PA0-1 to AF1
    GPIOA->AFR[0] &= 0xFFFFFF00; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 1);
    GPIOA->AFR[0] |= (1 << 5);

    // Set up GPIO output pins for motor direction control
    int allDirPins[4] = {4, 5, 6, 7};
    GPIOB->MODER &= (0xFF << 4);    // Clear
    for (int i = 0; i < (NUM_MOTORS * 2); i++) {
        int pinIdx = allDirPins[i] * 2;
        GPIOB->MODER &= ~(11 << pinIdx);  // Clear
        GPIOB->MODER |= (1 << pinIdx);    // Assign
    }

    // For each motor, initialize one direction pin to high, the other low
    for (int i = 0; i < (NUM_MOTORS*2); i++) {
        int pinIdx = allDirPins[i];
        // Set pins in even indices high, odd indices low
        if (i%2 == 0) {
            GPIOB->ODR |= (1 << pinIdx);
        } else {
            GPIOB->ODR &= ~(1 << pinIdx);
        }
    }

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;                          // Clears; defaults to edge-aligned upcounting
    TIM2->CCMR1 = 0;                        // (prevents having to manually clear bits)
    TIM2->CCER = 0;

    // Set output-compare CH1-4 to PWM1 mode and enable CCR1 preload buffer
    TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //| TIM_CCMR1_OC1PE); // Enable channel 1
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); //| TIM_CCMR1_OC2PE); // Enable channel 2

    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);        // Enable capture-compare channel 1-2
    TIM2->PSC = 1;                         // Run timer on 24Mhz
    TIM2->ARR = 2400;                      // PWM at 20kHz

    TIM2->CCR1 = 2000;                        // Start PWMs at 0% duty cycle
    TIM2->CCR2 = 2000;

    TIM2->CR1 |= TIM_CR1_CEN;              // Enable timer

    // DEBUGGING
    //TIM2->EGR |= 1; // todo: forces register update
}

/* Sets up four GPIO pins for inputs and enable interrupts on rising and falling edge of encoder wave.
 * Sets up timer for speed calculation and PI updates. */
void initEncoders(struct MotorPinout *mp) {
    // Enable EXTI in NVIC
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // GPIOB2-3
    // Clear to input mode
    GPIOB->MODER &= ~(1 << 5);
    GPIOB->MODER &= ~(1 << 4);
    GPIOB->MODER &= ~(1 << 7);
    GPIOB->MODER &= ~(1 << 6);

    // Clear to low speed
    GPIOB->OSPEEDR &= ~(1 << 5);
    GPIOB->OSPEEDR &= ~(1 << 4);
    GPIOB->OSPEEDR &= ~(1 << 7);
    GPIOB->OSPEEDR &= ~(1 << 6);

    // Set up pull-up resistor
    GPIOB->PUPDR &= ~(1 << 5);
    GPIOB->PUPDR &= ~(1 << 4);
    GPIOB->PUPDR |= (1 << 4);
    GPIOB->PUPDR &= ~(1 << 5);
    GPIOB->PUPDR &= ~(1 << 4);
    GPIOB->PUPDR |= (1 << 4);

    // Turn on interrupts
    EXTI->IMR &= ~(1 << 2);
    EXTI->IMR |= (1 << 2);
    EXTI->IMR &= ~(1 << 3);
    EXTI->IMR |= (1 << 3);

    // Turn on rising trigger
    EXTI->RTSR &= ~(1 << 2);
    EXTI->RTSR |= (1 << 2);
    EXTI->RTSR &= ~(1 << 3);
    EXTI->RTSR |= (1 << 3);

    // Turn on falling trigger
    EXTI->FTSR &= ~(1 << 2);
    EXTI->FTSR |= (1 << 2);
    EXTI->FTSR &= ~(1 << 3);
    EXTI->FTSR |= (1 << 3);

    // Enable SYSCFG peripheral (this is on APB2 bus)
    RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
//    do {
//        uint_32_t tmp;
//        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
//        tmp = RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
//    } while (0U)
    // todo: is this correct or do we need to do more to enable here?

    // Configure multiplexer to route PA2-3 to EXTI1
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB | SYSCFG_EXTICR1_EXTI3_PB;

    // Enable interrupts
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 1);

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 11;
    TIM6->ARR = 30000;

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
//    leds->green = 1;
//    leds->set(&leds);

    uint64_t tickSum = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &motors[i];
        motor->num_ticks = encoderCounts[i];    // Assign count value to appropriate struct
        tickSum += encoderCounts[i];
        encoderCounts[i] = 0;                   // Reset
    }
    uint64_t avgTicks = tickSum / NUM_MOTORS;
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &motors[i];
        motor->correctError(motor, avgTicks);
    }
    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
//    leds->green = 0;
//    leds->set(&leds);
}

/* The handler fired for each tick interrupt.
 * Increments the appropriate count variable according to which encoder fired the event. */
void EXTI2_3_IRQHandler(void) {
    leds->green = 1;
    leds->set(leds);

    // NOTE: there is no way to make this pin agnostic, have to manually update consts below if pins are changed
    int ENC_PINS[] = {2, 3};
    int COUNT_IDX_OFFSET = 7;

    // Check which bit is pending (represents which encoder fired tick interrupt)
    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = ENC_PINS[i];

        uint16_t pinIsPending = EXTI->PR & (1 << pinIdx);
        if (pinIsPending) {
            // Get corresponding count index and increment
            int countIdx = pinIdx - COUNT_IDX_OFFSET;
            encoderCounts[countIdx] += 1;

            // Clear pending interrupt flag
            EXTI->PR &= ~(1 << pinIdx);
        }
    }

//    leds->green = 0;
//    leds->set(leds);
}

// Calculates error for single motor and resets PWM signal
void PI_update(struct Motor* this, uint64_t avgDist) {

    // Calculate error signal
    this->error = (this->target_ticks - this->num_ticks);

    // Calculate integral error
    this->error_integral += this->error;

    // Clamp the value of the integral to a positive range
    this->error_integral = (this->error_integral > 3200) ? 3200 : this->error_integral;
    this->error_integral = (this->error_integral > 0) ? 0 : this->error_integral;

    // Calculate mean distance traveled by both wheels (in units of ticks)
    this->error_distance = avgDist - this->error_distance;

    // Calculate proportional error & add to output
    int16_t output = Ki * this->error_integral + Kp * this->error;

    // Divide output to get value into proper range (0-100)
    output = output >> 5;

    // Clamp the value for PWM input range
    output = (output > 100) ? 100 : (output < 0) ? 0 : output;

    this->setCycle(this, output);
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor* this, uint8_t duty) {
    uint32_t TEST_VAL = 1000;
    if(duty <= 100) {

//        // DEBUGGING
//        leds->orange = 1;
//        leds->set(leds);

        uint32_t autoReload = this->pwmTimer->ARR;
        uint8_t motorIdx = this->id;
        switch (motorIdx) {
            case 1:
                // todo: change this back after testing
                //uart_ptr->transmit(222);
                //this->pwmTimer->CCR1 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                this->pwmTimer->CCR1 = TEST_VAL;
                break;
            case 2:
                //this->pwmTimer->CCR2 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                this->pwmTimer->CCR2 = TEST_VAL;
                break;
        }

//        // DEBUGGING
//        leds->orange = 0;
//        leds->set(leds);
    }
}

/* Sets the target RPM and direction for this motor */
// todo: we are getting through this
void spinMotor(struct Motor* this, uint16_t targetRpm, int dir) {
    if (targetRpm > 0) {
        this->target_ticks = targetRpm;
        uint8_t pinIdxA = this->dir_pin_A;
        uint8_t pinIdxB = this->dir_pin_B;

        if (dir > 0) {
            // Pin A to high, pin B to low
            this->dirGpio->ODR |= (1 << pinIdxA);
            this->dirGpio->ODR &= ~(1 << pinIdxB);
        } else if (dir < 0) {
            this->dirGpio->ODR |= (1 << pinIdxB);
            this->dirGpio->ODR &= ~(1 << pinIdxA);
        } else {
            uart_ptr->transmit(255);
        }
    }
}

/* Stops the motor by setting the target RPM to zero. Does not affect direction. */
void stopMotor(struct Motor *this) {
    this->target_ticks = 0;
}

