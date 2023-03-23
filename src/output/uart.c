
struct UART_INT {
  void (*transmit)(uint8_t);
};

void transmitValue(uint8_t val) {
  // Wait until our tx reg is ready
  volatile int wait = 1;
  while (wait)
    if (USART3->ISR & 0x80)
      wait = 0;
  // Write to tx reg
  USART3->TDR = val;
}

void initUart(struct UART_INT* this) {
  // Set PC10 and PC11 to alt fx mode
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xF00000);    // Clear
  GPIOC->MODER |= 0xA00000;   		 // 1010

  // Set PC10 to AF1 for USART3_TX/RX
  GPIOC->AFR[1] &= ~(0xFF00);   	 // Clear
  GPIOC->AFR[1] |= 0x1100;   		 // Set 11 and 10 to 0001

  // Enable system clock for USART3
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  // Set Baud rate
  uint64_t t_baud = 115200;
  uint64_t f_clk = SystemCoreClock;
  uint64_t brr = f_clk/t_baud;
  USART3->BRR &= ~(0xFFFFFFFF);   	 // Clear
  USART3->BRR |= brr;

  // Enable tx
  USART3->CR1 |= (1 << 3);

  // Enable rx
  USART3->CR1 |= (1 << 2);

  // Enable rx not-empty interrupt
  USART3->CR1 |= (1 << 5);

  // Enable USART
  USART3->CR1 |= (1 << 0);

  // Enable USART3 interrupt on NVIC
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(USART3_4_IRQn, 1);
  this->transmit = &transmitValue;
}