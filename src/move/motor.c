//
// Created by Stephanie Georges on 4/15/23.
//

#include "motor.h"

// Sets up the entire motor drive system
void initMotor(void) {
    struct MotorPinout mp;
    assignPins(*mp);
    initPWMs(*mp);
    initEncoders(*mp);
    initADC();
}

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
    pm->enc_pins = {8, 9, 10, 11};           // PA8, PA9, PA10, PA11 w/ TIM1
    mp->pwm_alt_fxn_codes = {0x2, 0x2, 0x2, 0x2}; // AF2/0010, AF2/0010, AF2/0010, AF2/0010
    mp->pwmGpio = GPIOC;
    mp->dirGpio = GPIOB;
    mp->encGpio = GPIOA;
    mp->pwmTimer = TIM3;
    mp->encTimer = TIM1;
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
    RCC->APB1ENR |= APB;    // todo: left off here
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

void initEncodersV2(struct MotorPinout *mp) {


    // todo: figure out how timer 1 and diff channels route to individual registers

    // todo: what mode does timer need to be in? capture and compare? def not encoder input

    // todo: what to set ARR values

    // todo:
}

void getEncoderCounts() {

}

// Sets up encoder interface to read motor speed
void initEncoders(struct MotorPinout *mp) {
    // todo: port this to agnostic 4 pins

    // Set up encoder input pins (TIMER 1 CH1-4)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // todo: this is not pin agnostic
    mp->encGpio->MODER &= ~(GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
    mp->encGpio->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

    // Set selected pins to correct alternate functions (AF0-AF6)
    for (int i = 0; i < NUM_PINS; i++) {
        int pinIdx = mp->enc_pins[i] * 4;     // 4 bits per pin for this register
        int afVal = mp->enc_alt_fxn_codes[i];

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

    // Set up encoder interface (TIM3 encoder input mode)
    // todo: can we use the same input mode for rotary encoder instead of quadrature encoder
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // todo: this is not pin agnostic
    mp->encTimer->CCMR1 = 0;
    mp->encTimer->CCER = 0;
    mp->encTimer->SMCR = 0;
    mp->encTimer->CR1 = 0;

    // todo: didn't change the CNT/ARR values here...
    mp->encTimer->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    mp->encTimer->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    mp->encTimer->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    mp->encTimer->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    mp->encTimer->CR1 |= TIM_CR1_CEN;                               // Enable timer

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    // todo: can we use the same timer to do this? should be able to
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 11;
    TIM6->ARR = 30000;

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    // todo: this is not pin/timer agnostic - can I pass arg here?
    motor_speed = (TIM1->CNT - 0x7FFF);
    TIM1->CNT = 0x7FFF; // Reset back to center point

    // Call the PI update function
    PI_update();

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void initADC() {
    // todo: need to port this to pin agnostic
    // Configure PA1 for ADC input (used for current monitoring)
    GPIOA->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1);

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;                        // Default resolution is 12-bit (RES[1:0] = 00 --> 12-bit)
    ADC1->CFGR1 |= ADC_CFGR1_CONT;          // Set to continuous mode
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;      // Enable channel 1

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}
