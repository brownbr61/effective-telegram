//
// Motor min. operating speed 200 RPM (+/- 10%) w/ 1:48 gear ratio
//

#ifndef EFFECTIVE_TELEGRAM_MOTOR_H
#define EFFECTIVE_TELEGRAM_MOTOR_H
struct Motor {
    uint8_t duty_cycle;         // Output PWM duty cycle
    int16_t target_rpm;         // Desired speed target
    uint16_t motor_speed;       // Measured motor speed
    int8_t adc_value;           // ADC measured motor current
    int16_t error;              // Speed error signal
    int16_t error_integral;     // Integrated error signal
    uint8_t Kp;                 // Proportional gain
    uint8_t Ki;                 // Integral gain
    uint16_t (*turn)(struct Motor*);
};

/* A single object to assign/control our motor connections to the STM32 Discovery board.
 * Note: I was able to use the same GPIO ports for all direction and encoder ports
 * If we have to split these up to multiple ports (e.g. GPIOA, GPIOB) then will have to add properties to this struct */
struct MotorPinout {
    // PWM controls
    GPIO_TypeDef *pwmGpio;      // GPIO Port used for PWM inputs
    int pwm_in_pins[4];         // Used for PWM input to the motors
    int pwm_alt_fxn_codes[4];   // The alternate function codes corresponding to the chosen PWM input pins
    TIM_TypeDef *pwmTimer;      // Timer used for PWM input pins (must coordinate with pwm_in array)

    // Direction controls
    GPIO_TypeDef *dirGpio;      // GPIO Port used for motor direction pins
    int mtr_A_dir_pins[2];      // Motor A direction control pins
    int mtr_B_dir_pins[2];      // Motor B direction control pins
    int mtr_C_dir_pins[2];      // Motor C direction control pins
    int mtr_D_dir_pins[2];      // Motor D direction control pins

    // Encoder controls
    GPIO_TypeDef *encGpio;      // GPIO Port used for motor encoder pins
    int enc_pins[4];            // Used for encoder A outputs from H-Bridge
    int enc_alt_fxn_codes[4];   // The alternate function codes corresponding to the chosen encoder pins
    TIM_TypeDef *encTimer;      // Timer associated with encoder pins; used to calculate speed (must coordinate with enc array)
};

void assignPins(struct MotorPinout *mp);

void initMotor(struct Motor*, uint32_t);

// Sets up all PWM pins and direction signals to drive the H-Bridges
void initPWMs(struct MotorPinout *mp);

void initEncoders(struct MotorPinout *ep);

// Set the duty cycle of the PWM, accepts (0-100)
void setDutyCycle(uint8_t duty);

// PI control code is called within a timer interrupt
void updatePI(void);

void initADC(struct MotorPinout *mp);

#endif //EFFECTIVE_TELEGRAM_MOTOR_H
