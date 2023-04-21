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

/* Note: I was able to use the same GPIO ports for all direction and encoder ports
 * If we have to split these up to multiple ports (e.g. GPIOA, GPIOB) then will have to add properties to this struct */
struct MotorPins {
    int pwm_in[4];              // Used for PWM input to the motors
    int alt_fxn[4];             // The alternate function codes corresponding to the chosen PWM input pins
    int mtr_A_dir[2];           // Motor A direction control pins
    int mtr_B_dir[2];           // Motor B direction control pins
    int mtr_C_dir[2];           // Motor C direction control pins
    int mtr_D_dir[2];           // Motor D direction control pins
    GPIO_TypeDef *pwmGpio;      // GPIO Port used for PWM inputs
    GPIO_TypeDef *dirGpio;      // GPIO Port used for motor direction pins
    GPIO_TypeDef *encGpio;      // GPIO Port used for motor encoder pins
    TIM_TypeDef *pwmTimer;      // Timer used for PWM input pins (must coordinate with pwm_in array)
    uint32_t apb1TimerEnable;   // Value to turn on timer in RCC (NOTE: ASSUMES USING APB1ENR!!!)
};

struct EncoderPins {
    // todo: fill in
};

void assignMotorPins(struct MotorPins *mp);
void assignEncoderPins(struct EncoderPins *ep);

void initMotor(struct Motor*, uint32_t);

// Sets up all PWM pins and direction signals to drive the H-Bridges
void initPWMs(struct MotorPins *mp);
void initEncoders(struct EncoderPins *ep);

// Set the duty cycle of the PWM, accepts (0-100)
void setDutyCycle(uint8_t duty);

// PI control code is called within a timer interrupt
void updatePI(void);

#endif //EFFECTIVE_TELEGRAM_MOTOR_H
