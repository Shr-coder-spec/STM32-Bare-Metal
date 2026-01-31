#include <stdio.h>
#include <string.h>
#include <stdint.h>
// Base addresses (from STM32F446RE reference manual)
#define RCC_AHB1ENR   (*(volatile unsigned int*)0x40023830)
#define RCC_APB1ENR   (*(volatile unsigned int*)0x40023840)

#define GPIOA_MODER   (*(volatile unsigned int*)0x40020000)
#define GPIOA_OTYPER  (*(volatile unsigned int*)0x40020004)
#define GPIOA_OSPEEDR (*(volatile unsigned int*)0x40020008)
#define GPIOA_ODR     (*(volatile unsigned int*)0x40020014)
#define GPIOA_AFRL     (*(volatile unsigned int*)0x40020020)
#define GPIOA_AFRH     (*(volatile unsigned int*)0x40020024)

#define GPIOB_MODER   (*(volatile unsigned int*)0x40020400)
#define GPIOB_OTYPER  (*(volatile unsigned int*)0x40020404)
#define GPIOB_AFRL    (*(volatile unsigned int*)0x40020420)
#define GPIOB_AFRH    (*(volatile unsigned int*)0x40020424)
#define GPIOB_IDR     (*(volatile unsigned int*)0x40020410)
#define GPIOB_PUPDR   (*(volatile unsigned int*)0x4002040C)
#define GPIOB_OSPEEDR (*(volatile unsigned int*)0x40020408)

#define TIM2_CR1      (*(volatile unsigned int*)0x40000000)
#define TIM2_PSC      (*(volatile unsigned int*)0x40000028)
#define TIM2_ARR      (*(volatile unsigned int*)0x4000002C)
#define TIM2_DIER     (*(volatile unsigned int*)0x4000000C)
#define TIM2_SR       (*(volatile unsigned int*)0x40000010)

#define TIM3_CR1      (*(volatile unsigned int*)0x40000400)
#define TIM3_CNT      (*(volatile unsigned int*)0x40000424)
#define TIM3_PSC      (*(volatile unsigned int*)0x40000428)
#define TIM3_ARR      (*(volatile unsigned int*)0x4000042C)

#define USART2_BRR    (*(volatile unsigned int*)0x40004408)
#define USART2_CR1    (*(volatile unsigned int*)0x4000440C)
#define USART2_SR     (*(volatile unsigned int*)0x40004400)
#define USART2_DR     (*(volatile unsigned int*)0x40004404)

#define I2C2_CR1      (*(volatile unsigned int*)0x40005800)
#define I2C2_CR2      (*(volatile unsigned int*)0x40005804)
#define I2C2_SR1      (*(volatile unsigned int*)0x40005814)
#define I2C2_SR2      (*(volatile unsigned int*)0x40005818)
#define I2C2_DR       (*(volatile unsigned int*)0x40005810)
#define I2C2_CCR      (*(volatile unsigned int*)0x4000581C)
#define I2C2_TRISE    (*(volatile unsigned int*)0x40005820)

// NVIC registers
#define NVIC_ISER0    (*(volatile unsigned int*)0xE000E100)
#define SCB_SCR       (*(volatile uint32_t*)0xE000ED10)

volatile uint8_t wake_flag = 0;
/* ADS1115 I2C addresses */
#define ADS1115_1 (0x48 << 1)
#define ADS1115_2 (0x49 << 1)

void delay_ms(uint16_t ms)
{
    uint16_t start = TIM3_CNT;
    while ((uint16_t)(TIM3_CNT - start) < ms);
}

void UART2_SendChar(char c)
{
    while(!(USART2_SR & (1<<7))); // Wait until TX buffer empty
    USART2_DR = c;
}
void UART2_SendString(const char *s)
{
    while(*s)
    {
        while(!(USART2_SR & (1<<7)));  // TXE
        USART2_DR = *s++;
    }
}

static void MX_USART2_UART_Init(void){
	RCC_APB1ENR |= (1 << 17); // Enable USART2 clock
	RCC_AHB1ENR |= (1 << 0);  // Enable GPIOA clock

	// Set PA2 (TX) and PA3 (RX) to AF7
	GPIOA_MODER &= ~((3 << (2*2)) | (3 << (3*2))); // clearing
	GPIOA_MODER |=  ((2 << (2*2)) | (2 << (3*2))); //AF mode
	GPIOA_AFRL |=  (7 << (2*4)) | (7 << (3*4));    //AF7

	// Configure USART2
	USART2_BRR = 0x0008B;  // For 16 MHz clock 115200 baudrate = 8.68
	USART2_CR1 = (1 << 13) | (1 << 3) | (1 << 2); // UE=1, TE=1, RE=1
}

static void MX_I2C2_Init(void){
	// Enable clocks
	    RCC_AHB1ENR |= (1 << 1);    // GPIOB clock
	    RCC_APB1ENR |= (1 << 22);   // I2C2 clock

	    /* ----------- Configure PB10 (SCL) as AF4 ----------- */
	    GPIOB_MODER &= ~(3 << (10 * 2));          // clear mode
	    GPIOB_MODER |=  (2 << (10 * 2));          // AF mode

	    GPIOB_OTYPER |= (1 << 10);                // open-drain

	    GPIOB_PUPDR &= ~(3 << (10 * 2));		  //clear
	    GPIOB_PUPDR |=  (1 << (10 * 2));          // pull-up

	    GPIOB_AFRH &= ~(0xF << ((10 - 8) * 4));   // clear AF
	    GPIOB_AFRH |=  (4 << ((10 - 8) * 4));     // AF4 = I2C2_SCL

	    /* ----------- Configure PB3 (SDA) as AF9 ----------- */
	    GPIOB_MODER &= ~(3 << (3 * 2));           // clear mode
	    GPIOB_MODER |=  (2 << (3 * 2));           // AF mode

	    GPIOB_OTYPER |= (1 << 3);                 // open-drain

	    GPIOB_PUPDR &= ~(3 << (3 * 2));           // clear
	    GPIOB_PUPDR |=  (1 << (3 * 2));           // pull-up

	    //GPIOB_OSPEEDR |= (3 << (10*2)) | (3 << (3*2));  // very high speed
	    GPIOB_AFRL &= ~(0xF << (3 * 4));          // clear AF
	    GPIOB_AFRL |=  (9 << (3 * 4));            // AF9 = I2C2_SDA

	    /* ----------- Configure I2C2 peripheral ----------- */
	    I2C2_CR1 = 0;                            // disable I2C before changing registers
	    I2C2_CR2 = 16;                            // 16 MHz peripheral clock
	    I2C2_CCR = 80;                            // 100 kHz standard mode
	    // Sm mode or SMBus:
	    // Thigh = CCR * TPCLK1
	    // Tlow = CCR * TPCLK1
	    I2C2_TRISE = 17;                          // (freq + 1)

	    I2C2_CR1 |= (1 << 0);                     // PE = 1 (enable I2C2)
        // Enable I2C2
//    UART2_SendString("\r\nI2C2 initialized\r\n");
//    UART2_SendString("SDA=");
//    UART2_SendChar((GPIOB_IDR & (1<<3)) ? '1' : '0');  // Correct
//    UART2_SendString(" SCL=");
//    UART2_SendChar((GPIOB_IDR & (1<<10)) ? '1' : '0');
//    UART2_SendString("\r\n");

}

void MX_TIM2_Init(void)
{
    RCC_APB1ENR |= (1 << 0);    // Enable TIM2 clock

    TIM2_PSC = 15999;           // 16MHz / 16000 = 1kHz
    TIM2_ARR = 4999;             // 1kHz / 1000 = 1Hz → 1 second

    TIM2_DIER |= 1;             // Enable update interrupt

    NVIC_ISER0 |= (1 << 28);    // Enable TIM2 IRQ (position 28)

    TIM2_CR1 |= 1;              // Enable counter
}

void MX_TIM3_Init(void)
{
    RCC_APB1ENR |= (1 << 1);      // Enable TIM3 clock

    TIM3_PSC = 15999;             // 16 MHz / 16000 = 1 kHz → 1 ms tick
    TIM3_ARR = 0xFFFF;            // Max count
    TIM3_CR1 |= 1;                // Enable counter

    UART2_SendString("TIM3 initialized for delay\r\n");
}


void I2C2_Start(void)
{
    //if (I2C2_SR2 & (1<<1)) UART2_SendString("BUSY\r\n");
    I2C2_CR1 |= (1<<8);                        // START
    while(!(I2C2_SR1 & (1<<0)));               // SB


   // (void)I2C2_SR1;                            // clear SB by reading SR1
   // UART2_SendString("\r\nI2C2 started\r\n");
}

void I2C2_Stop(void)
{
    I2C2_CR1 |= (1<<9);
   // UART2_SendString("\r\nI2C2 stopped\r\n");
}

void I2C2_SendAddress(uint8_t addr)
{
    I2C2_DR = addr;
   // UART2_SendString("\r\nI2C2 sending address\r\n");
    while(!(I2C2_SR1 & (1<<1))); //matching
    volatile uint32_t temp = I2C2_SR2; // clear ADDR flag
    (void)temp;
   // UART2_SendString("\r\nI2C2 sent address\r\n");
}

void I2C2_WriteByte(uint8_t data)
{
	while(!(I2C2_SR1 & (1<<7)));  // until DR is empty
	    I2C2_DR = data;
	    while(!(I2C2_SR1 & (1<<2)));  // BTF (byte fully shifted)
   // UART2_SendString("\r\nI2C2 wrote byte\r\n");

}

uint8_t I2C2_ReadByte_ACK(void)
{
    I2C2_CR1 |= (1<<10);
    while(!(I2C2_SR1 & (1<<6)));
   // UART2_SendString("\r\nI2C2 read byte ack\r\n");
    return I2C2_DR;
}

uint8_t I2C2_ReadByte_NACK(void)
{
    // 1) Disable ACK
    I2C2_CR1 &= ~(1<<10);

    // 2) Generate STOP
    I2C2_CR1 |= (1<<9);
    // 3) Wait RXNE

    while(!(I2C2_SR1 & (1<<6)));

    // 4) Read data
    uint8_t data = I2C2_DR;

   // UART2_SendString("\r\nI2C2 read byte nack\r\n");
    return data;
}
uint16_t I2C2_Read2Bytes(uint8_t addr_read)
{
    uint8_t msb, lsb;

    // After we issue START and send addr_read, wait ADDR then:
    while(!(I2C2_SR1 & (1<<1)));        // ADDR
    //set POS=1, ACK=0 BEFORE clearing ADDR
    I2C2_CR1 |=  (1<<11);               // POS = 1
    I2C2_CR1 &= ~(1<<10);               // ACK = 0
    (void)I2C2_SR2;                     // clear ADDR

    // Now wait BTF (both bytes shifted into shift reg)
    while(!(I2C2_SR1 & (1<<2)));        // BTF


    I2C2_CR1 |= (1<<9);                 // STOP

    // Read
    msb = I2C2_DR;
    lsb = I2C2_DR;

    I2C2_CR1 &= ~(1<<11);

    return ((uint16_t)msb<<8) | lsb;
}

void ADS1115_WriteConfig(uint16_t config, uint8_t addr)
{
    I2C2_Start();
    I2C2_SendAddress(addr | 0); // write
    I2C2_WriteByte(0x01);       // pointer register = CONFIG
    I2C2_WriteByte(config >> 8);
    I2C2_WriteByte(config & 0xFF);
    I2C2_Stop();
    //UART2_SendString("\r\nI2C2 configured\r\n");
}

uint16_t ADS1115_ReadConversion(uint8_t addr)
{
    // Write pointer = 0x00 (conversion register)
    I2C2_Start();
    I2C2_SendAddress(addr | 0);     // write
    I2C2_WriteByte(0x00);           // pointer

    // two bytes with POS sequence
    I2C2_CR1 |= (1<<8);                     // START
    while(!(I2C2_SR1 & (1<<0)));            // SB
    (void)I2C2_SR1;

    I2C2_DR = (addr | 1);                   // read address
    return I2C2_Read2Bytes(addr | 1);
}

uint16_t ADS1115_StartAndRead(uint8_t ch, uint8_t address)
{
    uint16_t cfg = 0;

    cfg |= (1 << 15); // start single-shot
    cfg |= (0x4000 + ch * 0x1000); // MUX
    cfg |= 0x0200;   // ±4.096V
    cfg |= (1 << 8); // single-shot mode
    cfg |= (4 << 5); // 128 SPS
    cfg |= 0x0003;   // disable comparator

    ADS1115_WriteConfig(cfg, address);

    delay_ms(9);

    return ADS1115_ReadConversion(address);
}

void TIM2_IRQHandler(void)
{
    if (TIM2_SR & 1)        // UIF flag
    {
    	// ADC 1 (0x48)

    	uint16_t raw1_ch[4];
        UART2_SendString("Woke up\r\n");
    	    raw1_ch[0] = ADS1115_StartAndRead(0, ADS1115_1);
    	    raw1_ch[1] = ADS1115_StartAndRead(1, ADS1115_1);

    	    // ADC 2 (0x49)
    	    raw1_ch[2] = ADS1115_StartAndRead(0, ADS1115_2);
    	    raw1_ch[3] = ADS1115_StartAndRead(1, ADS1115_2);

    	    char msg[80];
    	    sprintf(msg,
    	            "ADC1: CH0=%d  CH1=%d   ADC2: CH0=%d  CH1=%d\r\n",
    	            raw1_ch[0], raw1_ch[1], raw1_ch[2], raw1_ch[3]);

    	    UART2_SendString(msg);
    	    // Seat occupancy check
    	    for(int i = 0; i<=3; i++)
    	       if (raw1_ch[i] > 15000)
    	           UART2_SendString("Seat Occupied\r\n");
    	       else
    	           UART2_SendString("Seat Empty\r\n");

    	TIM2_SR &= ~1;      // clear flag
        //wake_flag = 1;
    }
}

static inline void enter_sleep_mode(void)
{
    SCB_SCR &= ~(1 << 2);   // SLEEPDEEP = 0
    __asm volatile ("wfi");
}

#define __WFI() __asm volatile("wfi")

int main(void)
{
    //MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    UART2_SendString("\r\nI2C2 ADS1115 Register-Level Test\r\n");

    while (1)
    {
    	wake_flag = 0;
        UART2_SendString("Went to Sleep\r\n");
    	__WFI();
    }
}
