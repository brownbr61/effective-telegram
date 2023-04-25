#include "motor.h"

void fillDirPins(int *allPins, struct MotorPinout *mp);
// Sets up the entire motor drive system
void initMotion(struct LEDs *leds_in) {
    transmit2bytes(0);
    struct MotorPinout mp;
    assignPins(&mp);
    transmit2bytes(1);
    initMotors(&mp);
    transmit2bytes(2);
    initPWMs(&mp);
    transmit2bytes(3);
    initEncoders(&mp);
    transmit2bytes(4);
    leds = leds_in;
}

/* Initializes all motors in the forward direction in a stopped state */
void initMotors(struct MotorPinout *mp) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &(motors[i]);
        motor->direction = 1;
        motor->target_rpm = 0;
        motor->motor_speed = 0;
        motor->error = 0;
        motor->error_integral = 0;

        motor->setCycle = &pwm_setDutyCycle;
        motor->correctError = &PI_update;
        motor->spin = &spinMotor;
        motor->stop = &stopMotor;

        motor->pwmGpio = mp->pwmGpio;
        *motor->pwm_in_pin = mp->pwm_in_pins[i];
        *motor->pwm_alt_fxn_code = mp->pwm_alt_fxn_codes[i];

        motor->dirGpio = mp->dirGpio;
        switch (i) {
            case 0:
                motor->dir_pin_A = mp->mtr_A_dir_pins[0];
                motor->dir_pin_B = mp->mtr_A_dir_pins[1];
                return;
            case 1:
                motor->dir_pin_A = mp->mtr_B_dir_pins[0];
                motor->dir_pin_B = mp->mtr_B_dir_pins[1];
                return;
            case 2:
                motor->dir_pin_A = mp->mtr_C_dir_pins[0];
                motor->dir_pin_B = mp->mtr_C_dir_pins[1];
                return;
            case 3:
                motor->dir_pin_A = mp->mtr_D_dir_pins[0];
                motor->dir_pin_B = mp->mtr_D_dir_pins[1];
                return;
        }
    }
}

/* Pin assignment for the PWM input and motor direction outputs.
 * Note: using a single timer for all four PWM input pins to keep motors at synched RPMs */
void assignPins(struct MotorPinout *mp) {
    mp->pwm_in_pins[0] = 6;  // PC6
    mp->pwm_in_pins[1] = 7;  // PC7
    mp->pwm_in_pins[2] = 8;  // PC8
    mp->pwm_in_pins[3] = 9;  // PC9

    mp->pwm_alt_fxn_codes[0] = 0x0;  // AF0/0000
    mp->pwm_alt_fxn_codes[1] = 0x0;  // AF0/0000
    mp->pwm_alt_fxn_codes[2] = 0x0;  // AF0/0000
    mp->pwm_alt_fxn_codes[3] = 0x0;  // AF0/0000

    mp->mtr_A_dir_pins[0] = 4;   // PB4
    mp->mtr_A_dir_pins[1] = 5;   // PB5
    mp->mtr_B_dir_pins[0] = 6;   // PB6
    mp->mtr_B_dir_pins[1] = 7;   // PB7
    mp->mtr_C_dir_pins[0] = 8;   // PB8
    mp->mtr_C_dir_pins[1] = 9;   // PB9
    mp->mtr_D_dir_pins[0] = 10;  // PB10
    mp->mtr_D_dir_pins[1] = 11;  // PB11

    mp->enc_pins[0] = 8;   // PA8
    mp->enc_pins[1] = 9;   // PA9
    mp->enc_pins[2] = 10;  // PA10
    mp->enc_pins[3] = 11;  // PA11

    mp->pwm_alt_fxn_codes[0] = 0x2;  // AF2/0010
    mp->pwm_alt_fxn_codes[1] = 0x2;  // AF2/0010
    mp->pwm_alt_fxn_codes[2] = 0x2;  // AF2/0010
    mp->pwm_alt_fxn_codes[3] = 0x2;  // AF2/0010
    
    mp->pwmGpio = GPIOC;
    mp->dirGpio = GPIOB;
    mp->encGpio = GPIOA;
    mp->pwmTimer = TIM3;
    mp->encTimer = TIM1;
    mp->extLine = EXTI4_15_IRQn;            // NOTE: IF THIS CHANGES, NEED TO MANUALLY UPDATE INTERRUPT HANDLER
    mp->sysCfgExtiBucket = 2;
    mp->exti_codes[0] = SYSCFG_EXTICR3_EXTI8_PA;
    mp->exti_codes[1] = SYSCFG_EXTICR3_EXTI9_PA;
    mp->exti_codes[2] = SYSCFG_EXTICR3_EXTI10_PA;
    mp->exti_codes[3] = SYSCFG_EXTICR3_EXTI11_PA;

}

// Sets up the PWM and direction signals to drive the H-Bridge
void initPWMs(struct MotorPinout *mp) {

    // Set all four pin choices to alt fxn mode
    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = mp->pwm_in_pins[i] * 2;     // 2 bits per pin for this register
        mp->pwmGpio->MODER |= (1 << (pinIdx+1));
        mp->pwmGpio->MODER &= ~(1 << pinIdx);
    }

    // Set selected pins to correct alternate functions (AF0-AF6)
    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = mp->pwm_in_pins[i] * 4;     // 4 bits per pin for this register
        int afVal = mp->pwm_alt_fxn_codes[i];

        // Pins 7+ in AFR[1]
        if (pinIdx > 7) {
            mp->pwmGpio->AFR[1] &= ~(0xF << pinIdx);    // Clear
            mp->pwmGpio->AFR[1] |= (afVal << pinIdx);   // Assign
        // Pins 0-6 in AFR[0]
        } else {
            mp->pwmGpio->AFR[0] &= ~(0xF << pinIdx);    // Clear
            mp->pwmGpio->AFR[0] |= (afVal << pinIdx);   // Assign
        }
    }

    // Set up GPIO output pins for motor direction control
    int allDirPins[8];
    fillDirPins(allDirPins, mp);
    for (int i = 0; i < (NUM_MOTORS*2); i++) {
        int pinIdx = allDirPins[i]*2;           // 2 bits per pin for this register
        mp->dirGpio->MODER &= ~(11 << pinIdx);  // Clear
        mp->dirGpio->MODER |= (1 << pinIdx);    // Assign
    }

    // todo: does this set forward or backwards? Need to test
    // For each motor, initialize one direction pin to high, the other low
    for (int i = 0; i < (NUM_MOTORS*2); i++) {
        int pinIdx = allDirPins[i];             // 1 bit per pin for this register
        // Set pins in even indices high, odd indices low
        if (i%2 == 0) {
            mp->dirGpio->ODR |= (1 << pinIdx);
        } else {
            mp->dirGpio->ODR &= ~(1 << pinIdx);
        }
    }

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // todo: this is not pin agnostic
    mp->pwmTimer->CR1 = 0;
    mp->pwmTimer->CCMR1 = 0;                       // (prevents having to manually clear bits)
    mp->pwmTimer->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    // todo: didn't alter these at all... may need to change according to our TT motors?
    mp->pwmTimer->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    mp->pwmTimer->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    mp->pwmTimer->PSC = 1;                         // Run timer on 24Mhz
    mp->pwmTimer->ARR = 1200;                      // PWM at 20kHz

    mp->pwmTimer->CCR1 = 0;                        // Start PWMs at 0% duty cycle
    mp->pwmTimer->CCR2 = 0;
    mp->pwmTimer->CCR3 = 0;
    mp->pwmTimer->CCR4 = 0;

    mp->pwmTimer->CR1 |= TIM_CR1_CEN;              // Enable timer
}

/* Internal Helper Function
 * Fills the provided array with the direction pins from the provided struct in the following order:
 * [MotorA_pin0, motorA_pin1, motorB_pin0, motorB_pin1, motorC_pin0, motorC_pin1, motorD_pin0, motorD_pin1]*/
void fillDirPins(int *allPins, struct MotorPinout *mp) {
    allPins[0] = mp->mtr_A_dir_pins[0];
    allPins[1] = mp->mtr_A_dir_pins[1];

    allPins[2] = mp->mtr_B_dir_pins[0];
    allPins[3] = mp->mtr_B_dir_pins[1];

    allPins[4] = mp->mtr_C_dir_pins[0];
    allPins[5] = mp->mtr_C_dir_pins[1];

    allPins[6] = mp->mtr_D_dir_pins[0];
    allPins[7] = mp->mtr_D_dir_pins[1];
}

/* Sets up four GPIO pins for inputs and enable interrupts on rising and falling edge of encoder wave.
 * Sets up timer for speed calculation and PI updates. */
void initEncoders(struct MotorPinout *mp) {
    // Enable EXTI in NVIC
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // todo: this is not pin agnostic

    // Enable interrupts
    NVIC_EnableIRQ(mp->extLine);
    // todo: is this sufficient for enabling EXTI interrupts or do I need to mask interrupts specifically on the EXTI->IMR register?

    // Set interrupt priority
    NVIC_SetPriority(mp->extLine, 3);

    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = mp->enc_pins[i] * 2;            // 2 bits per pin for MODER, OSPEEDR, and PUPDR

        // Set GPIO pins to input mode
        mp->encGpio->MODER &= ~(11 << pinIdx); 		// Clear to input mode

        // Set input speed to low (todo: read more about this, does it need to change?)
        mp->encGpio->OSPEEDR &= ~(11 << pinIdx);    // Clear to low speed mode

        // Enable pull-down resistor (todo: do we need to do this?)
        mp->encGpio->PUPDR &= ~(11 << pinIdx);      // Clear
        mp->encGpio->PUPDR |= (10 << pinIdx);       // Set to pull-down
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = mp->enc_pins[i];        // 1 bit per pin for IMR, RTSR, FTSR

        EXTI->IMR &= ~(1 << pinIdx);         // Clear
        EXTI->IMR |= (1 << pinIdx);          // Turn on interrupts

        EXTI->RTSR &= ~(1 << pinIdx);       // Clear
        EXTI->RTSR |= (1 << pinIdx);        // Turn on rising trigger

        EXTI->FTSR &= ~(1 << pinIdx);       // Clear
        EXTI->FTSR |= (1 << pinIdx);        // Turn on falling trigger
    }

    // Enable SYSCFG peripheral (this is on APB2 bus)
    RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;

    // Configure the multiplexer to route PA8-11 to EXTI2
    int bucketIdx = mp->sysCfgExtiBucket;
    for (int i = 0; i < NUM_MOTORS; i++) {
        uint16_t extiCode = mp->exti_codes[i];
        SYSCFG->EXTICR[bucketIdx] |= extiCode;
    }

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Select PSC and ARR values that give an appropriate interrupt rate
    // todo: calculate rate here
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

    leds->orange = 1;
    leds->set(leds);

    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &motors[i];
        motor->motor_speed = encoderCounts[i];  // Assign count value to appropriate struct
        encoderCounts[i] = 0;                   // Reset
        motor->correctError(motor); // todo: add params here
    }

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
    leds->orange = 0;
    leds->set(leds);
}

/* The handler fired for each tick interrupt.
 * Increments the appropriate count variable according to which encoder fired the event. */
// todo: discuss thread safety with team
void EXTI4_15_IRQHandler(void) {
    leds->red = 1;
    leds->set(leds);

    // NOTE: there is no way to make this pin agnostic, have to manually update consts below if pins are changed
    int ENC_PINS[] = {8, 9, 10, 11};
    // int ENC_INT_LINE = EXTI4_15_IRQn;
    int COUNT_IDX_OFFSET = 7;

    // this assumes the 4_15 handler will resume before accepting another interrupt

    // Check which bit is pending (represents which encoder fired tick interrupt)
    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = ENC_PINS[i];       // One bit per pin for PR register

        uint16_t pinIsPending = EXTI->PR & (1 << pinIdx);
        if (pinIsPending) {
            // Get corresponding count index and increment
            int countIdx = pinIdx - COUNT_IDX_OFFSET;
            encoderCounts[countIdx] += 1;

            // Clear pending interrupt flag
            EXTI->PR &= ~(1 << pinIdx);
        }
    }
        leds->red = 0;
        leds->set(leds);
}

// Calculates error for single motor and resets PWM signal
void PI_update(struct Motor *this) {
    // Calculate error signal
    this->error = (this->target_rpm * CONV_MOTOR_SPEED_TO_TARGET_RPM - this->motor_speed);

    // Calculate integral error
    this->error_integral += this->error;

    // Clamp the value of the integral to a positive range
    // todo: does this bound change?
    this->error_integral = (this->error_integral > 3200) ? 3200 : this->error_integral;
    this->error_integral = (this->error_integral > 0) ? 0 : this->error_integral;

    // Calculate Distance error & add to output
    //int16_t derror = meand - distance

    // Calculate proportional error & add to output
    int16_t output = Ki * this->error_integral + Kp * this->error;

    // Divide output to get value into proper range (0-100)
    output = output >> 5;

    // Clamp the value for PWM input range
    output = (output > 100) ? 100 : output;
    output = (output < 0) ? 0 : output;

    this->setCycle(this, output);
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor *this, uint8_t duty) {
    if(duty <= 100) {
        uint32_t autoReload = this->pwmTimer->ARR;
        uint8_t motorIdx = this->id;
        switch (motorIdx) {
            case 1:
                this->pwmTimer->CCR1 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                break;
            case 2:
                this->pwmTimer->CCR2 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                break;
            case 3:
                this->pwmTimer->CCR3 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                break;
            case 4:
                this->pwmTimer->CCR4 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                break;
            // default:
            //     ERROR("Incorrect motor index supplied to set PWM duty cycle \n\r");
        }
    } 
    // else {
    //     ERROR("Invalid duty cycle value supplied to set PWM duty cycle \n\r");
    // }
}

/* Sets the target RPM and direction for this motor */
void spinMotor(struct Motor *this, uint16_t targetRpm, int dir) {
    if (targetRpm > 0) {
        this->target_rpm = targetRpm;
        uint8_t pinIdxA = this->dir_pin_A;
        uint8_t pinIdxB = this->dir_pin_B;

        // todo: test that forwards/back matches the pins we've selected here, may need to flip
        if (dir > 0) {
            // Pin A to high, pin B to low
            this->dirGpio->ODR |= (1 << pinIdxA);
            this->dirGpio->ODR &= ~(1 << pinIdxB);
        } else if (dir < 0) {
            this->dirGpio->ODR |= (1 << pinIdxB);
            this->dirGpio->ODR &= ~(1 << pinIdxA);
        }
    } 
    // else {
    //     ERROR("Cant spin at an RPM < 0");
    // }
    // todo: what else do we need to get going here?
}

/* Stops the motor by setting the target RPM to zero. Does not affect direction. */
void stopMotor(struct Motor *this) {
    this->target_rpm = 0;
    // todo: is setting target_rpm sufficient here?
}
