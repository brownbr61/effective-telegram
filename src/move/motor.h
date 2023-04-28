#define NUM_MOTORS 4

// todo: these values will need to be tested & changed
#define CONV_MOTOR_SPEED_TO_TARGET_RPM 2
#define Kp 1            // Proportional gain
#define Ki 1            // Integral gain

/* A struct representing the state of a single motor */
struct Motor {
    uint8_t id;                         // Which motor this object represents (1-4)
    int direction;                      // Direction of movement (<0 = reverse; >0 = forward)
    int16_t target_rpm;                 // Desired speed target
    uint16_t motor_speed;               // Measured motor speed
    uint16_t error;                     // Speed error signal
    uint16_t error_integral;            // Integrated error signal
    void (*spin)(struct Motor*, uint16_t, int);
    void (*stop)(struct Motor*);
    void (*setCycle)(struct Motor*, uint8_t);
    void (*correctError)(struct Motor*);

    // Pointers to the pins that control this motor (convenience variables)
    GPIO_TypeDef *pwmGpio;
    uint8_t pwm_in_pin;
    uint8_t pwm_alt_fxn_code;
    TIM_TypeDef *pwmTimer;

    GPIO_TypeDef *dirGpio;
    uint8_t dir_pin_A;
    uint8_t dir_pin_B;

    // todo: do we need any other mp pointers linked in here?
};

// Starting at top left (US driver's side front), motors go clockwise
// 0 = front left
// 1 = front right
// 2 = rear right
// 3 = rear left
struct Motor motors[4];

void lab_pwm_init(void);
void lab_pwm_16(void);
void lab_pwm_17(void);


uint64_t encoderCounts[] = { 0, 0, 0, 0 };

/* A single object to assign/control our motor connections to the STM32 Discovery board.
 * Note: I was able to use the same GPIO ports for all direction and encoder ports
 * If we have to split these up to multiple ports (e.g. GPIOA, GPIOB) then will have to add properties to this struct */
struct MotorPinout {
    // PWM controls
    GPIO_TypeDef *pwmGpio;          // GPIO Port used for PWM inputs
    uint8_t pwm_in_pins[4];         // Used for PWM input to the motors
    uint8_t pwm_alt_fxn_codes[4];   // The alternate function codes corresponding to the chosen PWM input pins
    TIM_TypeDef *pwmTimer;          // Timer used for PWM input pins (must coordinate with pwm_in array)

    // Direction controls
    GPIO_TypeDef *dirGpio;          // GPIO Port used for motor direction pins
    uint8_t mtr_A_dir_pins[2];      // Motor A direction control pins
    uint8_t mtr_B_dir_pins[2];      // Motor B direction control pins
    uint8_t mtr_C_dir_pins[2];      // Motor C direction control pins
    uint8_t mtr_D_dir_pins[2];      // Motor D direction control pins

    // Encoder controls
    GPIO_TypeDef *encGpio;          // GPIO Port used for motor encoder pins
    uint8_t enc_pins[4];            // Used for encoder A outputs from H-Bridge
    uint8_t enc_alt_fxn_codes[4];   // The alternate function codes corresponding to the chosen encoder pins
    TIM_TypeDef *encTimer;          // Timer associated with encoder pins; used to calculate speed (must coordinate with enc array)
    uint8_t extLine;               // EXTI peripheral line tied to count interrupts
    uint8_t sysCfgExtiBucket;       // The index which the chosen encoder pins correspond to in the SYSCFG_EXTICR buckets (values 0-3) [i.e. 0: 0-3, 1: 4-7, etc]
    uint16_t exti_codes[4];         // The configuration values for setting the SYS_CFG->EXTICR register; corresponds to encoder pins
};

void initPWMsT1(struct MotorPinout*);
void initPWMsT2(struct MotorPinout*);
void initPWMsT15(struct MotorPinout*);

// Debugging
struct LEDs *leds;
struct UART_INT *uart_ptr;

// Initializes entire class and structure necessary for motion
void initMotion(struct LEDs*, struct UART_INT*);

// Initializes all four motor structs
void initMotors(struct MotorPinout*);

// Initializes direction pins for all four motors
void initDirection(struct MotorPinout*);

// Sets up all PWM pins and direction signals to drive the H-Bridges
void initPWMs(struct MotorPinout*);

// Sets up GPIO inputs for encoder signals + sets up timer for speed check interrupts
void initEncoders(struct MotorPinout*);

// Assigns the pins related to motion
void assignPins(struct MotorPinout*);

// PI control code is called within a timer interrupt
void PI_update(struct Motor*);

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor*, uint8_t);

// Go at given speed
void spinMotor(struct Motor*, uint16_t, int);

// Don't go
void stopMotor(struct Motor*);
