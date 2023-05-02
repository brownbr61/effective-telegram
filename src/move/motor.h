#define NUM_MOTORS 2
#define Kp 1            // Proportional gain
#define Ki 1            // Integral gain

/* A struct representing the state of a single motor */
struct Motor {
    uint8_t id;                         // Which motor this object represents (1-4)
    int direction;                      // Direction of movement (<0 = reverse; >0 = forward)
    int16_t target_ticks;               // Aka desired speed target
    uint16_t num_ticks;                 // Aka motor speed
    uint16_t error;                     // Speed error signal
    uint16_t error_integral;            // Integrated error signal
    uint16_t error_distance;            // Distance error signal
    void (*spin)(struct Motor*, uint16_t, int);
    void (*stop)(struct Motor*);
    void (*setCycle)(struct Motor*, uint8_t);
    void (*correctError)(struct Motor*, uint64_t);

    // Pointers to the pins that control this motor (convenience variables)
    GPIO_TypeDef *pwmGpio;
    uint8_t pwm_in_pin;
    uint8_t pwm_alt_fxn_code;
    TIM_TypeDef *pwmTimer;

    GPIO_TypeDef *dirGpio;
    uint8_t dir_pin_A;
    uint8_t dir_pin_B;
};

// Order: left, right
struct Motor motors[NUM_MOTORS];
uint64_t encoderCounts[] = {0, 0};

/* A single object to assign/control our motor connections to the STM32 Discovery board.
 * Note: I was able to use the same GPIO ports for all direction and encoder ports
 * If we have to split these up to multiple ports (e.g. GPIOA, GPIOB) then will have to add properties to this struct */

void initPWM();

// Debugging
struct LEDs *leds;
struct UART_INT *uart_ptr;

// Initializes entire class and structure necessary for motion
void initMotion(struct LEDs*, struct UART_INT*);

// Initializes all four motor structs
void initMotors();

// Initializes direction pins for all four motors
void initDirection();

// Sets up all PWM pins and direction signals to drive the H-Bridges
void initPWMs();

// Sets up GPIO inputs for encoder signals + sets up timer for speed check interrupts
void initEncoders();

// PI control code is called within a timer interrupt
void PI_update(struct Motor*, uint64_t);

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor*, uint8_t);

// Go at given speed
void spinMotor(struct Motor*, uint16_t, int);

// Don't go
void stopMotor(struct Motor*);
