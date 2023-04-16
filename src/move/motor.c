//
// Created by Stephanie Georges on 4/15/23.
//

#include "motor.h"

// Sets up the entire motor drive system
void initMotor(void) {
    struct MotorPins mp;
    assignMotorPins(*mp);
    initPWMs(*mp);

    struct EncoderPins ep;
    assignEncoderPins(*ep);
    initEncoder(*ep);

    struct ADCPins ap;
    assignAdcPins(*ap);
    initADC(*ap);
}

/* Pin assignment for the PWM input and motor direction outputs.
 * Note: using a single timer for all four PWM input pins to keep motors at synched RPMs */
void assignMotorPins(struct MotorPins *mp) {
    mp->pwm_in = {6, 7, 8, 9};          // PC6, PC7, PC8, PC9
    mp->alt_fxn = {0x0, 0x0, 0x0, 0x0}; // AF0/0000, AF0/0000, AF0/0000, AF0/0000
    mp->mtr_A_dir = {4, 5};             // PB4, PB5
    mp->mtr_B_dir = {6, 7};             // PB6, PB7
    mp->mtr_C_dir = {8, 9};             // PB8, PB9
    mp->mtr_D_dir = {10, 11};           // PB10, PB11
    mp->pwmGpio = GPIOC;
    mp->dirGpio = GPIOB;
    mp->encGpio = GPIOB;
}

// Sets up the PWM and direction signals to drive the H-Bridge
void initPWMs(struct MotorPins *mp) {
    static const int NUM_PINS = 4;

    // Set all four pin choices to alt fxn mode
    for (int i = 0; i < NUM_PINS; i++) {
        int pinIdx = mp->pwm_in[i] * 2;     // 2 bits per pin for this register
        mp->pwmGpio->MODER |= (1 << (pinIdx+1));
        mp->pwmGpio->MODER &= ~(1 << pinIdx);
    }

    // Set selected pins to correct alternate functions (AF0-AF6)
    for (int i = 0; i < NUM_PINS; i++) {
        int pinIdx = mp->pwm_in[i] * 4;     // 4 bits per pin for this register
        int afVal = mp->alt_fxn[i];

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
    RCC->APB1ENR |= mp->apb1TimerEnable;
    mp->pwmTimer->CR1 = 0;
    mp->pwmTimer->CCMR1 = 0;                       // (prevents having to manually clear bits)
    mp->pwmTimer->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
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
void fillDirPins(int *allPins, struct MotorPins *mp) {
    allPins[0] = mp->mtr_A_dir[0];
    allPins[1] = mp->mtr_A_dir[1];

    allPins[2] = mp->mtr_B_dir[0];
    allPins[3] = mp->mtr_B_dir[1];

    allPins[4] = mp->mtr_C_dir[0];
    allPins[5] = mp->mtr_C_dir[1];

    allPins[6] = mp->mtr_D_dir[0];
    allPins[7] = mp->mtr_D_dir[1];
}

// Sets up encoder interface to read motor speed
void initEncoders(struct EncoderPins *ep) {

    // todo: port this to agnostic 4 pins

    // Set up encoder input pins (TIMER 3 CH1 and CH2)
    // todo: should this be a diff timer with four channels?
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20) );

    // Set up encoder interface (TIM3 encoder input mode)
    // todo: can we use the same input mode for rotary encoder instead of quadrature encoder
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    // todo: can we use the same timer to do this?
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 11;
    TIM6->ARR = 30000;

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}