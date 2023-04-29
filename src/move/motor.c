#include "motor.h"
#include "../main.h"

void fillDirPins(int *allPins, struct MotorPinout *mp);
// Sets up the entire motor drive system
void initMotion(struct LEDs *leds_in, struct UART_INT *uart_in) {
    leds = leds_in;
    uart_ptr = uart_in;
    struct MotorPinout mp;
    assignPins(&mp);
    initMotors(&mp);
    //initDirection(&mp);

    initPWMsT2(&mp);
//    lab_pwm_init();
//    initPWMsT1(&mp);

    //initEncoders(&mp);
}

/* Initializes all motors in the forward direction in a stopped state */
void initMotors(struct MotorPinout *mp) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &(motors[i]);
        motor->id = i + 1; // Range is 1-4
        motor->direction = 1;
        motor->target_rpm = 100;
        motor->motor_speed = 0;
        motor->error = 0;
        motor->error_integral = 0;

        motor->setCycle = &pwm_setDutyCycle;
        motor->correctError = &PI_update;
        motor->spin = &spinMotor;
        motor->stop = &stopMotor;

        // todo: change when we get timer3 working
        motor->pwmGpio = mp->pwmGpio;
        motor->pwm_in_pin = mp->pwm_in_pins[i];
        motor->pwm_alt_fxn_code = mp->pwm_alt_fxn_codes[i];

        motor->dirGpio = mp->dirGpio;
        switch (i) {
            case 0:
                motor->dir_pin_A = mp->mtr_A_dir_pins[0];
                motor->dir_pin_B = mp->mtr_A_dir_pins[1];
                break;
            case 1:
                motor->dir_pin_A = mp->mtr_B_dir_pins[0];
                motor->dir_pin_B = mp->mtr_B_dir_pins[1];
                break;
            case 2:
                motor->dir_pin_A = mp->mtr_C_dir_pins[0];
                motor->dir_pin_B = mp->mtr_C_dir_pins[1];
                break;
            case 3:
                motor->dir_pin_A = mp->mtr_D_dir_pins[0];
                motor->dir_pin_B = mp->mtr_D_dir_pins[1];
                break;
        }
    }
}

/* Pin assignment for the PWM input and motor direction outputs.
 * Note: using a single timer for all four PWM input pins to keep motors at synched RPMs */
void assignPins(struct MotorPinout *mp) {
    mp->pwm_in_pins[0] = 0;  // PA0
    mp->pwm_in_pins[1] = 1;  // PA1
    mp->pwm_in_pins[2] = 2;  // PA2
    mp->pwm_in_pins[3] = 3;  // PA3

    mp->pwm_alt_fxn_codes[0] = 0x2;  // AF2/0010
    mp->pwm_alt_fxn_codes[1] = 0x2;  // AF2/0010
    mp->pwm_alt_fxn_codes[2] = 0x2;  // AF2/0010
    mp->pwm_alt_fxn_codes[3] = 0x2;  // AF2/0010

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

    mp->enc_alt_fxn_codes[0] = 0x2;  // AF2/0010
    mp->enc_alt_fxn_codes[1] = 0x2;  // AF2/0010
    mp->enc_alt_fxn_codes[2] = 0x2;  // AF2/0010
    mp->enc_alt_fxn_codes[3] = 0x2;  // AF2/0010

    mp->pwmGpio = GPIOA;
    mp->dirGpio = GPIOB;
    mp->encGpio = GPIOA;
    mp->pwmTimer = TIM2;
    mp->encTimer = TIM1;
    mp->extLine = EXTI4_15_IRQn;            // NOTE: IF THIS CHANGES, NEED TO MANUALLY UPDATE INTERRUPT HANDLER
    mp->sysCfgExtiBucket = 2;
    mp->exti_codes[0] = SYSCFG_EXTICR3_EXTI8_PA;
    mp->exti_codes[1] = SYSCFG_EXTICR3_EXTI9_PA;
    mp->exti_codes[2] = SYSCFG_EXTICR3_EXTI10_PA;
    mp->exti_codes[3] = SYSCFG_EXTICR3_EXTI11_PA;
}

// Sets up the PWM and direction signals to drive the H-Bridge
// todo: this works
void initPWMsT2(struct MotorPinout *mp) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Set up pin PA0-3 for H-bridge PWM output (TIMER 2 CH1-4)
    mp->pwmGpio->MODER |= (1 << 1);
    mp->pwmGpio->MODER &= ~(1 << 0);
    mp->pwmGpio->MODER |= (1 << 3);
    mp->pwmGpio->MODER &= ~(1 << 2);
    mp->pwmGpio->MODER |= (1 << 5);
    mp->pwmGpio->MODER &= ~(1 << 4);
    mp->pwmGpio->MODER |= (1 << 7);
    mp->pwmGpio->MODER &= ~(1 << 6);

    // Set PA0-3 to AF1
    mp->pwmGpio->AFR[0] &= 0xFFFF0000; // clear PA4 bits,
    mp->pwmGpio->AFR[0] |= (1 << 1);
    mp->pwmGpio->AFR[0] |= (1 << 5);
    mp->pwmGpio->AFR[0] |= (1 << 9);
    mp->pwmGpio->AFR[0] |= (1 << 13);

    // Set up GPIO output pins for motor direction control
    int allDirPins[8];
    fillDirPins(allDirPins, mp);
    mp->dirGpio->MODER &= (0xFF << 4);    // Clear
    for (int i = 0; i < (NUM_MOTORS * 2); i++) {
        int pinIdx = allDirPins[i] * 2;
        mp->dirGpio->MODER &= ~(11 << pinIdx);  // Clear
        mp->dirGpio->MODER |= (1 << pinIdx);    // Assign
    }

    // For each motor, initialize one direction pin to high, the other low
    for (int i = 0; i < (NUM_MOTORS*2); i++) {
        int pinIdx = allDirPins[i];
        // Set pins in even indices high, odd indices low
        if (i%2 == 0) {
            mp->dirGpio->ODR |= (1 << pinIdx);
        } else {
            mp->dirGpio->ODR &= ~(1 << pinIdx);
        }
    }

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // todo: this is not pin agnostic
    mp->pwmTimer->CR1 = 0;                          // Clears; defaults to edge-aligned upcounting
    mp->pwmTimer->CCMR1 = 0;                        // (prevents having to manually clear bits)
    mp->pwmTimer->CCER = 0;

    // Set output-compare CH1-4 to PWM1 mode and enable CCR1 preload buffer
    mp->pwmTimer->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //| TIM_CCMR1_OC1PE); // Enable channel 1
    mp->pwmTimer->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); //| TIM_CCMR1_OC2PE); // Enable channel 2
    mp->pwmTimer->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
    mp->pwmTimer->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);

    mp->pwmTimer->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E); //| TIM_CCER_CC3E | TIM_CCER_CC4E);           // Enable capture-compare channel 1-4
    mp->pwmTimer->PSC = 1;                         // Run timer on 24Mhz
    mp->pwmTimer->ARR = 2400;                      // PWM at 20kHz

    mp->pwmTimer->CCR1 = 2000;                        // Start PWMs at 0% duty cycle
    mp->pwmTimer->CCR2 = 2000;
    mp->pwmTimer->CCR3 = 2000;
    mp->pwmTimer->CCR4 = 2000;

    mp->pwmTimer->CR1 |= TIM_CR1_CEN;              // Enable timer

    // DEBUGGING
    mp->pwmTimer->EGR = 0;
    mp->pwmTimer->EGR |= (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);
    mp->pwmTimer->EGR |= 1; // todo: forces register update
}

void initDirection(struct MotorPinout *mp) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set up GPIO output pins for motor direction control
    int allDirPins[8];
    fillDirPins(allDirPins, mp);
    mp->dirGpio->MODER &= (0xFF << 4);    // Clear
    for (int i = 0; i < (NUM_MOTORS * 2); i++) {
        int pinIdx = allDirPins[i] * 2;
        mp->dirGpio->MODER &= ~(11 << pinIdx);  // Clear
        mp->dirGpio->MODER |= (1 << pinIdx);    // Assign
    }

    // For each motor, initialize one direction pin to high, the other low
    for (int i = 0; i < (NUM_MOTORS*2); i++) {
        int pinIdx = allDirPins[i];
        // Set pins in even indices high, odd indices low
        if (i%2 == 0) {
            mp->dirGpio->ODR |= (1 << pinIdx);
        } else {
            leds->set(leds);
            mp->dirGpio->ODR &= ~(1 << pinIdx);
        }
    }
}

void initPWMsT1(struct MotorPinout *mp) {

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set up pin PA8 for H-bridge PWM output (TIMER 1 CH1)
    mp->pwmGpio->MODER |= (1 << 17);
    mp->pwmGpio->MODER &= ~(1 << 16);

    // Set PA8 to AF2
    mp->pwmGpio->AFR[1] &= 0xFFFFFFF0; // clear PA4 bits,
    mp->pwmGpio->AFR[1] = 0x00000002;

    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // todo: this is not pin agnostic
    TIM1->CR1 = 0;                          // Clears; defaults to edge-aligned upcounting
    TIM1->CCMR1 = 0;                        // (prevents having to manually clear bits)
    TIM1->CCER = 0;

    // Set output-compare CH1-4 to PWM1 mode and enable CCR1 preload buffer
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //| TIM_CCMR1_OC1PE); // Enable channel 1

    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); //| TIM_CCER_CC3E | TIM_CCER_CC4E);           // Enable capture-compare channel 1-4
    TIM1->PSC = 1;                         // Run timer on 24Mhz
    TIM1->ARR = 2400;                      // PWM at 20kHz

    TIM1->CCR1 = 2000;                        // Start PWMs at 0% duty cycle
    TIM1->CCR2 = 2000;

    TIM1->CR1 |= TIM_CR1_CEN;              // Enable timer

    // DEBUGGING
    TIM1->EGR |= 1; // todo: forces register update

    //As the preload registers are transferred to the shadow registers only when an update event
    //occurs, before starting the counter, all registers must be initialized by setting the UG bit in
    //the TIMx_EGR register.
}

void initPWMsT15(struct MotorPinout *mp) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Set all four pin choices to alt fxn mode
    // Set up pin PA2-3 for H-bridge PWM output (TIMER 15 CH1+2)

    mp->pwmGpio->MODER |= (1 << 5);
    mp->pwmGpio->MODER &= ~(1 << 4);

    mp->pwmGpio->MODER |= (1 << 7);
    mp->pwmGpio->MODER &= ~(1 << 6);

//    // Set PA2-3 to AF4,
    mp->pwmGpio->AFR[0] &= 0xFFFF00FF; // clear PA2-3 bits,
//    mp->pwmGpio->AFR[0] |= (1 << 24);
//    mp->pwmGpio->AFR[0] |= (1 << 28);


    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // todo: this is not pin agnostic
    TIM15->CR1 = 0;                          // Clears; defaults to edge-aligned upcounting
    TIM15->CCMR1 = 0;                        // (prevents having to manually clear bits)
    TIM15->CCER = 0;

    // TODO: FIRST GET RID OF PRELOAD HERE
    // Set output-compare CH1-4 to PWM1 mode and enable CCR1 preload buffer
    TIM15->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //| TIM_CCMR1_OC1PE); // Enable channel 1
    TIM15->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); //| TIM_CCMR1_OC2PE); // Enable channel 2
//    mp->pwmTimer->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); //| TIM_CCMR2_OC3PE); // Enable channel 3
//    mp->pwmTimer->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); //| TIM_CCMR2_OC4PE); // Enable channel 4

    TIM15->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM15->PSC = 1;                         // Run timer on 24Mhz
    TIM15->ARR = 2400;                      // PWM at 20kHz

    //the output pin begins the PWM period at a
    //low state and goes high once the timer’s counter matches the CCRx register; this output resets to low
    //again when the next period starts. The location of the CCRx value relative to the ARR register value
    //determines the overall ratio of on/off (duty cycle)

    //consider PWM mode 1. The reference PWM signal OCxREF is
    //high as long as TIMx_CNT <TIMx_CCRx else it becomes low.
    TIM15->CCR1 = 2000;                        // Start PWMs at 0% duty cycle
    TIM15->CCR2 = 2000;

    TIM15->CR1 |= TIM_CR1_CEN;              // Enable timer

    // DEBUGGING todo: try this second
    TIM15->EGR |= 1; // todo: forces register update

    //As the preload registers are transferred to the shadow registers only when an update event
    //occurs, before starting the counter, all registers must be initialized by setting the UG bit in
    //the TIMx_EGR register.
}



void lab_pwm_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 9);
    GPIOA->MODER &= ~(1 << 8);

    // Set PA4 to AF4,
    GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 18);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 2400;                      // PWM at 20kHz
    TIM14->CCR1 = 2000;                        // Start PWM at 0% duty cycle

    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer

    TIM14->EGR |= 1;
}

void lab_pwm_16(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set up pin PA6 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 13);
    GPIOA->MODER &= ~(1 << 12);

    // Set PA6 to AF5,
    GPIOA->AFR[0] &= 0xF0FFFFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= 0x05000000;

    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->CR1 = 0;                         // Clear control registers
    TIM16->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM16->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM16->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // | TIM_CCMR1_OC1PE);
    TIM16->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM16->PSC = 1;                         // Run timer on 24Mhz
    TIM16->ARR = 2400;                      // PWM at 20kHz
    TIM16->CCR1 = 2000;                        // Start PWM at 0% duty cycle

    TIM16->CR1 |= TIM_CR1_CEN;              // Enable timer

    TIM16->EGR |= 1;
}

void lab_pwm_17(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set up pin PA7 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 15);
    GPIOA->MODER &= ~(1 << 14);

    // Set PA7 to AF5,
    GPIOA->AFR[0] &= 0x0FFFFFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= 0x50000000;

    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->CR1 = 0;                         // Clear control registers
    TIM17->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM17->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM17->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // | TIM_CCMR1_OC1PE);
    TIM17->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM17->PSC = 1;                         // Run timer on 24Mhz
    TIM17->ARR = 2400;                      // PWM at 20kHz
    TIM17->CCR1 = 2000;                        // Start PWM at 0% duty cycle

    TIM17->CR1 |= TIM_CR1_CEN;              // Enable timer

    TIM17->EGR |= 1;
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
        int pinIdx = mp->enc_pins[i] * 2;               // Two bits per pin here

        // Set GPIO pins to input mode
        mp->encGpio->MODER &= ~(11 << pinIdx); 		// Clear to input mode

        // Set input speed to low (todo: read more about this, does it need to change?)
        mp->encGpio->OSPEEDR &= ~(11 << pinIdx);    // Clear to low speed mode

        // Enable pull-down resistor (todo: do we need to do this?)
        mp->encGpio->PUPDR &= ~(11 << pinIdx);      // Clear
        mp->encGpio->PUPDR |= (10 << pinIdx);       // Set to pull-down
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = mp->enc_pins[i];

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

    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &motors[i];
        motor->motor_speed = encoderCounts[i];  // Assign count value to appropriate struct
        encoderCounts[i] = 0;                   // Reset
        motor->correctError(motor); // todo: add params here
    }

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

/* The handler fired for each tick interrupt.
 * Increments the appropriate count variable according to which encoder fired the event. */
void EXTI4_15_IRQHandler(void) {
    leds->red = 1;
    leds->set(leds);

    // NOTE: there is no way to make this pin agnostic, have to manually update consts below if pins are changed
    int ENC_PINS[] = {8, 9, 10, 11};
    int ENC_INT_LINE = EXTI4_15_IRQn;
    int COUNT_IDX_OFFSET = 7;

    // this assumes the 4_15 handler will resume before accepting another interrupt

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

    // todo: Calculate Distance error & add to output
    //int16_t derror = meand - distance

    // Calculate proportional error & add to output
    int16_t output = Ki * this->error_integral + Kp * this->error;

    // Divide output to get value into proper range (0-100)
    output = output >> 5;

    // Clamp the value for PWM input range
    output = (output > 100) ? 100 : output;
    output = (output < 0) ? 0 : output;

    this->setCycle(this, 30);
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor *this, uint8_t duty) {
    uint32_t TEST_VAL = 2000;
    if(duty <= 100) {
        leds->orange = 1;
        leds->set(leds);
        uint32_t autoReload = this->pwmTimer->ARR;
        uint8_t motorIdx = this->id;
        switch (motorIdx) {
            case 1:
                //uart_ptr->transmit(222);
                //this->pwmTimer->CCR1 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                this->pwmTimer->CCR1 = TEST_VAL;
                break;
            case 2:
                //this->pwmTimer->CCR2 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                this->pwmTimer->CCR2 = TEST_VAL;
                break;
            case 3:
                //this->pwmTimer->CCR3 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                this->pwmTimer->CCR3 = TEST_VAL;
                break;
            case 4:
                //this->pwmTimer->CCR4 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                this->pwmTimer->CCR4 = TEST_VAL;
                break;
                // default:
                //     ERROR("Incorrect motor index supplied to set PWM duty cycle \n\r");
        }
        leds->orange = 0;
        leds->set(leds);
    }
    // else {
    //     ERROR("Invalid duty cycle value supplied to set PWM duty cycle \n\r");
    // }
}

/* Sets the target RPM and direction for this motor */
void spinMotor(struct Motor *this, uint16_t targetRpm, int dir) {
    // DEBUG
    leds->blue = 1;
    leds->set(leds);

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
        } else {
            uart_ptr->transmit(255);
        }
    }

    // DEBUG
    DELAY(100);
    leds->blue = 0;
    leds->set(leds);

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

