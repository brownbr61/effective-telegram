//
// Created by Stephanie Georges on 4/15/23.
//

#include "motor.h"

// Sets up the entire motor drive system
void initMotor(void) {
    struct Motor m;

    struct MotorPinout mp;
    assignPins(mp);
    initPWMs(*mp);
    initEncoders(*mp);
    initADC();

    encoderCounts = {0, 0, 0, 0};
};

// todo: how to determine direction

/* Pin assignment for the PWM input and motor direction outputs.
 * Note: using a single timer for all four PWM input pins to keep motors at synched RPMs */
void assignPins(struct MotorPinout *mp) {
    mp->pwm_in_pins = {6, 7, 8, 9};          // PC6, PC7, PC8, PC9
    mp->pwm_alt_fxn_codes = {0x0, 0x0, 0x0, 0x0}; // AF0/0000, AF0/0000, AF0/0000, AF0/0000
    mp->mtr_A_dir = {4, 5};                  // PB4, PB5
    mp->mtr_B_dir_pins = {6, 7};             // PB6, PB7
    mp->mtr_C_dir_pins = {8, 9};             // PB8, PB9
    mp->mtr_D_dir_pins = {10, 11};           // PB10, PB11
    pm->enc_pins = {8, 9, 10, 11};           // PA8, PA9, PA10, PA11
    mp->pwm_alt_fxn_codes = {0x2, 0x2, 0x2, 0x2}; // AF2/0010, AF2/0010, AF2/0010, AF2/0010
    mp->pwmGpio = GPIOC;
    mp->dirGpio = GPIOB;
    mp->encGpio = GPIOA;
    mp->pwmTimer = TIM3;
    mp->encTimer = TIM1;
    mp->extLine = EXTI4_15_IRQn;            // NOTE: IF THIS CHANGES, NEED TO MANUALLY UPDATE INTERRUPT HANDLER
    mp->sysCfgExtiBucket = 2;
    mp->exti_codes = {SYSCFG_EXTICR3_EXTI8_PA, SYSCFG_EXTICR3_EXTI9_PA, SYSCFG_EXTICR3_EXTI10_PA, SYSCFG_EXTICR3_EXTI11_PA};
}

// Sets up the PWM and direction signals to drive the H-Bridge
void initPWMs(struct MotorPinout *mp) {
    static const int NUM_PINS = 4;

    // Set all four pin choices to alt fxn mode
    for (int i = 0; i < NUM_PINS; i++) {
        int pinIdx = mp->pwm_in_pins[i] * 2;     // 2 bits per pin for this register
        mp->pwmGpio->MODER |= (1 << (pinIdx+1));
        mp->pwmGpio->MODER &= ~(1 << pinIdx);
    }

    // Set selected pins to correct alternate functions (AF0-AF6)
    for (int i = 0; i < NUM_PINS; i++) {
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
    for (int i = 0; i < (NUM_PINS*2); i++) {
        int pinIdx = allDirPins[i]*2;           // 2 bits per pin for this register
        mp->dirGpio->MODER &= ~(11 << pinIdx);  // Clear
        mp->dirGpio->MODER |= (1 << pinIdx);    // Assign
    }

    // For each motor, initialize one direction pin to high, the other low
    for (int i = 0; i < (NUM_PINS*2); i++) {
        int pinIdx = allDirPins[i];             // 1 bit per pin for this register
        // Set pins in even indices high, odd indices low
        if (i%2 == 0) {
            mp->dirGpio->ODR |= (1 << pinIdx);
        } else {
            mp->dirGpio->ODR &= ~(1 << pinIdx);
        }
    }

    // Set up PWM timer
    RCC->APB1ENR |= APB; // todo: FIX
    mp->pwmTimer->CR1 = 0;
    mp->pwmTimer->CCMR1 = 0;                       // (prevents having to manually clear bits)
    mp->pwmTimer->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    // todo: didn't alter these at all... may need to change according to our TT motors?
    mp->pwmTimer->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    mp->pwmTimer->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    mp->pwmTimer->PSC = 1;                         // Run timer on 24Mhz
    mp->pwmTimer->ARR = 1200;                      // PWM at 20kHz
    mp->pwmTimer->CCR1 = 0;                        // Start PWM at 0% duty cycle

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
    const int NUM_PINS = 4;

    // Enable EXTI in NVIC
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // todo: make agnostic

    // Enable interrupts
    NVIC_EnableIRQ(mp->extLine);
    // todo: is this sufficient for enabling EXTI interrupts or do I need to mask interrupts specifically on the EXTI->IMR register?

    // Set interrupt priority
    NVIC_SetPriority(mp->extLine, 3);

    for (int i = 0; i < NUM_PINS; i++) {
        int pinIdx = mp->enc_pins[i] * 2;            // 2 bits per pin for MODER, OSPEEDR, and PUPDR

        // Set GPIO pins to input mode
        mp->encGpio->MODER &= ~(11 << pinIdx); 		// Clear to input mode

        // Set input speed to low (todo: read more about this, does it need to change?)
        mp->encGpio->OSPEEDR &= ~(11 << pinIdx);    // Clear to low speed mode

        // Enable pull-down resistor (todo: do we need to do this?)
        mp->encGpio->PUPDR &= ~(11 << pinIdx);      // Clear
        mp->encGpio->PUPDR |= (10 << pinIdx);       // Set to pull-down
    }

    for (int i = 0; i < NUM_PINS; i++) {
        int pinIdx = mp->enc_pins[i];        // 1 bit per pin for IDR, RTSR, FTSR

        EXTI->IDR &= ~(1 << pinIdx);         // Clear
        EXTI->IDR |= (1 << pinIdx);          // Turn on interrupts

        EXTI->RTSR &= ~(1 << pinIdx);       // Clear
        EXTI->RTSR |= (1 << pinIdx);        // Turn on rising trigger

        EXTI->FTSR &= ~(1 << pinIdx);       // Clear
        EXTI->FTSR |= (1 << pinIdx);        // Turn on falling trigger
    }

    // Enable SYSCFG peripheral (this is on APB2 bus)
    RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;

    // Configure the multiplexer to route PA8-11 to EXTI2
    int bucketIdx = mp->sysCfgExtiBucket;
    for (int i = 0; i < NUM_PINS; i++) {
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
    // todo: how will this change for four motors instead of one
    motor_speed = (TIM1->CNT - 0x7FFF);
    TIM1->CNT = 0x7FFF; // Reset back to center point

    // todo: if we haven't had any ticks come in since our last interrupt, can we assume we're at a stop?
    // todo: then we can decide to set direction?

    // Call the PI update function
    PI_update();

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

/* The handler fired for each tick interrupt.
 * Increments the appropriate count variable according to which encoder fired the event. */
void EXTI4_15_IRQHandler(void)
    // NOTE: there is no way to make this pin agnostic, have to manually update consts below if pins are changed
    const int ENC_PINS[] = {8, 9, 10, 11};
    const int ENC_INT_LINE = EXTI4_15_IRQn;
    const int COUNT_IDX_OFFSET = 7;
    const int NUM_PINS = 4;

    // Mask all other tick interrupts so we don't double count
    // this assumes the 4_15 handler will resume before accepting another interrupt
    NVIC_DisableIRQ(ENC_INT_LINE);

    // Check which bit is pending (represents which encoder fired tick interrupt)
    for (int i = 0; i < NUM_PINS; i++) {
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

    // Enable tick interrupts again
    NVIC_EnableIRQ(ENC_INT_LINE);
}
