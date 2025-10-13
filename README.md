### Embedded Controller - STM32F411 Driver Library

Written by: Kim Sang Yoon

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10/11

MCU: STM32F411RE, Nucleo-64



***

### GPIO Digital InOut

#### Header File

`#include "ecGPIO.h"`

```c
/*----------------------------------------------------------------\

@ Embedded Controller by Sang-Yoon Kim - Handong Global University

Author           : Sang Yoon Kim

Created          : 09-16-2025

Modified         : 09-16-2025

Language/ver     : C in Keil uVision

  

Description      : Distributed to Students for LAB_GPIO

/----------------------------------------------------------------*/

  
  

#ifndef __ECGPIO2_H

#define __ECGPIO2_H

  

#include "stm32f411xe.h"

#include "ecRCC2.h"

#include "ecPinNames.h"

  

#define INPUT  0x00

#define OUTPUT 0x01

#define AF     0x02

#define ANALOG 0x03

  

#define HIGH 1

#define LOW  0

  

#define LED_PIN

#define BUTTON_PIN

  

#ifdef __cplusplus

 extern "C" {

#endif /* __cplusplus */

void GPIO_init(PinName_t pinName, int mode);

void GPIO_write(PinName_t pinName, int Output);

int  GPIO_read(PinName_t pinName);

void GPIO_mode(PinName_t pinName, int mode);

void GPIO_ospeed(PinName_t pinName, int speed);

void GPIO_otype(PinName_t pinName, int type);

void GPIO_pupd(PinName_t pinName, int pupd);

// Initialize 7 DOUT pins for 7 segment leds

void seven_seg_FND_init(void);

  

// Select display: 0 to 3

// Display a number 0 - 9 only

void seven_seg_FND_display(uint8_t  num, uint8_t select);

  
  

#ifdef __cplusplus

}

#endif /* __cplusplus */

#endif // __ECGPIO2_H

```

#### GPIO\_init()

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15
* **mode**: INPUT(0), OUTPUT(1), AF(02), ANALOG (03)

**Example code**

```c
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```

#### GPIO\_mode()

Configures GPIO pin modes: In/Out/AF/Analog

```c
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15
* **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

**Example code**

```c
GPIO_mode(GPIOA, 5, OUTPUT);
```

#### GPIO\_write()

Write the data to GPIO pin: High, Low

```c
write(GPIO_TypeDef *Port, int pin, int output);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15
* **output**: LOW(0), HIGH(1)

**Example code**

```c
GPIO_write(GPIOA, 5, 1);  // 1: High
```

#### GPIO\_read()

Read the data from GPIO pin

```c
int  GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15

**Example code**

```c
GPIO_read(GPIOC, 13);
```

#### GPIO\_ospeed()

Configures output speed of GPIO pin : Low, Mid, Fast, High

```c
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15
* **speed**: LOW\_SPEED(0), MID\_SPEED(1), FAST\_SPEED(2) , HIGH\_SPEED(3)

**Example code**

```c
GPIO_ospeed(GPIOA, 5, 2);  // 2: FAST_SPEED
```

#### GPIO\_otype()

Configures output type of GPIO pin: Push-Pull / Open-Drain

```c
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15
* **type**: PUSH\_PULL(0), OPEN\_DRAIN(1)

**Example code**

```
GPIO_otype(GPIOA, 5, 0);  // 0: Push-Pull
```

#### GPIO\_pupd() 

Configures Pull-up/Pull-down mode of GPIO pin: No Pull-up, Pull-down/ Pull-up/ Pull-down/ Reserved

```
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupd);
```

**Parameters**

* **Port:** Port Number, GPIOA\~GPIOH
* **pin**: pin number (int) 0\~15
* **pupd**: NO\_PUPD(0), PULL\_UP(1), PULL\_DOWN(2), RESERVED(3)

**Example code**

```
GPIO_pupdr(GPIOA, 5, 0);  // 0: No Pull-up, Pull-down
```

#### Example Code

```c++
#include "ecSTM32F411.h"

#define LED_PIN 5
#define BUTTON_PIN 13

// Initialiization 
void setup(void) {
	RCC_PLL_init();
	SysTick_init();
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    
}
	
int main(void) { 
	setup();
	
	while(1){
		delay_ms(500);  
		GPIO_write(GPIOA, LED_PIN, LOW);
		delay_ms(500);  
		GPIO_write(GPIOA, LED_PIN, HIGH);
	}
}
```

#### seven_seg_FND_init()

Intiate the each pins state of 7-segment display

```c
void seven_seg_FND_init(void);
```

**Parameters**
void

**Example code**

```c

void seven_seg_FND_init(void){

for(int i = 0 ; i < 12 ; i++){

    GPIO_init(pinsFND[i],OUTPUT);

}  
for(int i = 0; i < 12; i++){

    GPIO_write(pinsFND[i],LOW);

}
};

```

#### seven_seg_FND_display()

Intiate the each pins state of 7-segment display

```c
void seven_seg_FND_display(uint8_t  num, uint8_t select);
```

**Parameters**
- **num:** Number that will be displayed(Inteager 0 ~ 9)
- **select:** Display that will be turned on(Display 1 ~ 4)

**Example code**

```c

void seven_seg_FND_display(uint8_t  num, uint8_t select){

    // // Turn off all digits to prevent ghosting

    // for(int d = 8; d < 12; d++){

    //     GPIO_write(pinsFND[d], LOW);

    // }
    // Segment patterns for 0..9 (bit0=a ... bit6=g, bit7=dp)

    uint8_t value[10] = {

        0b00111111,

        0b00000110,

        0b01011011,

        0b01001111,

        0b01100110,

        0b01101101,

        0b01111101,

        0b00000111,

        0b01111111,

        0b01101111

    };

    // Write segments on PB_7..PB_0 (pinsFND[0]..pinsFND[7])

    for(int i = 0; i < 8; i++){

        uint8_t bit = (value[num] >> i) & 0x01;

        GPIO_write(pinsFND[i], bit ? HIGH : LOW);

    }

  

    // Enable selected digit

    GPIO_write(pinsFND[8 + select], HIGH);

  

    //select display_0

};
```
#### seven_seg_FND_MultiFlex()

Intiate the each pins state of 7-segment display multiplex

```c
void sevensegment_display_MultiPlex(uint8_t num)
```

**Parameters**
- **num:** Number that user want 


**Example code**

```c

void sevensegment_display_MultiPlex(uint8_t num){
    // 7-segment segment pins (a–g, dp)
    PinName_t pinsSEG[8] = {PB_7, PB_6, PB_5, PB_4, PB_3, PB_2, PB_1, PB_0};

    // 7-segment digit-select pins:
    // pinsFNDselect[0] = LSD (ones place, PA_10), pinsFNDselect[1] = MSD (tens place, PA_11)
    PinName_t pinsFNDselect[4] = {PA_10, PA_11, PC_4, PC_3};

    // For num in 0–9: use only LSD (PA_10).
    // For num in 10–19: use both LSD (PA_10) and MSD (PA_11).
    // Assumption: MSD = pinsFNDselect[1] (PA_11), LSD = pinsFNDselect[0] (PA_10)

    // Segment decoding table matching PB_7 to PB_0
    // Bit order: 0b D P G F E D C B A
    const uint8_t segs[10] = {
        0b11111100, // 0
        0b01100000, // 1
        0b11011010, // 2
        0b11110010, // 3
        0b01100110, // 4
        0b10110110, // 5
        0b10111110, // 6
        0b11100000, // 7
        0b11111110, // 8
        0b11110110  // 9
    };

    // 1) Turn all digits off first (to avoid ghosting)
    // Only two digits are used here
    GPIO_write(pinsFNDselect[0], LOW); // disable LSD
    GPIO_write(pinsFNDselect[1], LOW); // disable MSD

    // 2) Split the number into LSD and MSD
    int digit_lsd = num % 10; // ones place
    int digit_msd = num / 10; // tens place (0 or 1)

    // 3) Output depending on the current multiplexing state
    if (multiplex_state == 0) {
        // --- Display LSD (ones place) ---

        // Write segment data for LSD
        uint8_t segment_data = segs[digit_lsd];
        for (int j = 0; j < 8; j++) {
            GPIO_write(pinsSEG[j], (segment_data >> j) & 0x01); // set segment pins
        }

        // Enable LSD (PA_10)
        GPIO_write(pinsFNDselect[0], HIGH);

        // Next state will show MSD
        multiplex_state = 1;

    } else {
        // --- Display MSD (tens place) ---

        if (num >= 10) {
            // Only show MSD when num >= 10 (always 1 for 10–19)

            // Write segment data for MSD (digit_msd is 1 here)
            uint8_t segment_data = segs[digit_msd];
            for (int j = 0; j < 8; j++) {
                GPIO_write(pinsSEG[j], (segment_data >> j) & 0x01); // set segment pins
            }

            // Enable MSD (PA_11)
            GPIO_write(pinsFNDselect[1], HIGH);

        } else {
            // For 0–9, keep the tens digit disabled (or display 0 if desired).
            // Digits are already LOW, so no extra enable here.
            // Optionally clear segments to reduce afterimage.
            for (int j = 0; j < 8; j++) {
                GPIO_write(pinsSEG[j], LOW); // clear all segment pins
            }
        }
        multiplex_state = 0;
    }
}

```
### GPIO EXTI
#### Header File
#### EXTI_init()

Initiate the EXTI

```c
void EXTI_init(PinName_t pinName, int trig_type,int priority)
```

**Parameters**
- pinName:**Number of Pin
- **trig_type:** Display that will be turned on(Display 1 ~ 4)
- priority : 

**Example code**

```c

void EXTI_init(PinName_t pinName, int trig_type,int priority){

  

    GPIO_TypeDef *port;

    unsigned int pin;

    ecPinmap(pinName,&port,&pin);

    // SYSCFG peripheral clock enable  

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;      

    // Connect External Line to the GPIO

    int EXTICR_port;

    if      (port == GPIOA) EXTICR_port = 0;

    else if (port == GPIOB) EXTICR_port = 1;

    else if (port == GPIOC) EXTICR_port = 2;

    else if (port == GPIOD) EXTICR_port = 3;

    else                                        EXTICR_port = 4;

SYSCFG->EXTICR[1] &= ~(0xFU << 0);    // clear 4 bits

SYSCFG->EXTICR[1] |=  (EXTICR_port << 0);  // set 4 bits

// Configure Trigger edge

if (trig_type == FALL) EXTI->FTSR |= (1U << pin);// Falling trigger enable

else if (trig_type == RISE) EXTI->RTSR |= (1U << pin);// Rising trigger enable

else if (trig_type == BOTH){ // Both falling/rising trigger enable

       EXTI->RTSR |= (1U<<pin);

       EXTI->FTSR |= (1U<<pin);

    }

// Configure Interrupt Mask (Interrupt enabled)

EXTI->IMR  |= (1U << pin); // not masked

  

    // NVIC(IRQ) Setting

    int EXTI_IRQn = 0;

if (pin < 5)        EXTI_IRQn = EXTI0_IRQn + pin;   // EXTI0~4_IRQn

else if (pin < 10)  EXTI_IRQn = EXTI9_5_IRQn;       // EXTI5~9

else                EXTI_IRQn = EXTI15_10_IRQn;     // EXTI10~15

  

    NVIC_SetPriority(EXTI_IRQn, 0); // EXTI priority

    NVIC_EnableIRQ(EXTI_IRQn);  // EXTI IRQ enable

}
# EC-sykim-009

#### EXTI_init()

Initiate the EXTI

```c
void EXTI_init(PinName_t pinName, int trig_type,int priority)
```


#### EXTI_enable();

Enable the EXTI CLOCK

```c
void EXTI_enable(PinName_t pinName)
```

**Parameters**
- pinName:**Number of Pin


**Example code**

```c

void EXTI_enable(PinName_t pinName){
GPIO_TypeDef *port; unsigned int pin;

    ecPinmap(pinName , &port , &pin);

    EXTI->IMR |= (1U << pin);
}
# EC-sykim-009

#### EXTI_enable();

Enable the EXTI CLOCK

```c
void EXTI_enable(PinName_t pinName)
```

#### EXTI_disable();

Enable the EXTI CLOCK

```c
void EXTI_disable(PinName_t pinName)
```

**Parameters**
- pinName:**Number of Pin


**Example code**

```c

void EXTI_disable(PinName_t pinName) {

    GPIO_TypeDef *port; unsigned int pin;

    ecPinmap(pinName , &port , &pin);

    EXTI->IMR &= ~(1U << pin);        

}
# EC-sykim-009

#### EXTI_disable();

Enable the EXTI CLOCK

```c
void EXTI_disable(PinName_t pinName)
```


#### is_pending_EXTI();

check the pending state of EXTI

```c
uint32_t is_pending_EXTI(PinName_t pinName)
```

**Parameters**
- pinName:**Number of Pin


**Example code**

```c

uint32_t is_pending_EXTI(PinName_t pinName) {

    GPIO_TypeDef *port;

    unsigned int pin;

    ecPinmap(pinName , &port , &pin);

    //Todo Port, Pin Configuration

    uint32_t EXTI_PRx = (1 << pin);         // check  EXTI pending  

    return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);

}
# EC-sykim-009

#### is_pending_EXTI();

check the pending state of EXTI

```c
uint32_t is_pending_EXTI(PinName_t pinName)
```

#### clear_pending_EXTI();

clear the pending state of EXTI

```c
void clear_pending_EXTI(PinName_t pinName)
```

**Parameters**
- pinName:**Number of Pin


**Example code**

```c

void clear_pending_EXTI(PinName_t pinName) {

    GPIO_TypeDef *port;

    unsigned int pin;

    ecPinmap(pinName , &port , &pin);

    EXTI->PR = (1U << pin);    

}
# EC-sykim-009
GPIO EXTI
#### Header File
#### EXTI_init()

Initiate the EXTI

```c
void EXTI_init(PinName_t pinName, int trig_type,int priority)
```


### GPIO_SysTick
#### Header File
#### SysTick_init()

intiate the SysTick

```c
void SysTick_init(void)
```

**Parameters**


**Example code**

```c

void SysTick_init(void){    

    //  SysTick Control and Status Register

    SysTick->CTRL = 0;                                          // Disable SysTick IRQ and SysTick Counter

  

    // Select processor clock

    // 1 = processor clock;  0 = external clock

    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

  

    // uint32_t MCU_CLK=EC_SYSTEM_CLK

    // SysTick Reload Value Register

    SysTick->LOAD = MCU_CLK_PLL / 1000 - 1;                     // 1ms, for HSI PLL = 84MHz.

  

    // SysTick Current Value Register

    SysTick->VAL = 0;

  

    // Enables SysTick exception request

    // 1 = counting down to zero asserts the SysTick exception request

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // Enable SysTick IRQ and SysTick Timer

    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    NVIC_SetPriority(SysTick_IRQn, 16);     // Set Priority to 1

    NVIC_EnableIRQ(SysTick_IRQn);           // Enable interrupt in NVIC

}
```

#### SysTick_Handler();

intiate the SysTick Counter Function 

```c
void SysTick_Handler(void)
```

**Parameters**
**Example code**

```c

void SysTick_Handler(void){

    SysTick_counter();  

}```

#### SysTick_counter()

Upcounting the msTicks

```c
void SysTick_counter()
```

**Parameters**
**Example code**

```c
  

void SysTick_counter(){

    msTicks++;

}

```

#### delay_ms ()

delay the System

```c
void delay_ms (uint32_t mesc)
```

**Parameters**
uint32_t mesc : mili sec
**Example code**

```c
  

void delay_ms (uint32_t mesc){

  uint32_t curTicks;

  

  curTicks = msTicks;

  while ((msTicks - curTicks) < mesc);

  msTicks = 0;

}

```

#### SysTick_reset()

delay the System

```c
void SysTick_reset(void)
```

**Parameters**

**Example code**

```c
  

void SysTick_reset(void)

{

    // SysTick Current Value Register

    SysTick->VAL = 0;

}

```

#### SysTick_val()

delay the System

```c
uint32_t SysTick_val(void)
```

**Parameters**

**Example code**

```c
  

uint32_t SysTick_val(void) {

    return SysTick->VAL;

}

```

### ecRCC
#### Header File
#### RCC_HSI_init()

intiate the HSI Clock

```c
void RCC_HSI_init()
```

**Parameters**


**Example code**

```c

void RCC_HSI_init() {

    // Enable High Speed Internal Clock (HSI = 16 MHz)

  //RCC->CR |= ((uint32_t)RCC_CR_HSION);

    RCC->CR |= 0x00000001U;

  // wait until HSI is ready

  //while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}

    while ( (RCC->CR & 0x00000002U) == 0 ) {;}

  // Select HSI as system clock source

  RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW);                                // not essential

  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;                               //00: HSI16 oscillator used as system clock

  

    // Wait till HSI is used as system clock source

  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != 0 );

    //EC_SYSTEM_CLK=16000000;

        //EC_SYSCLK=16000000;

        EC_SYSCLK=16000000;

}
```

#### RCC_PLL_init()

intiate the PLL Clock

```c
void RCC_PLL_init()
```

**Parameters**


**Example code**

```c

void RCC_PLL_init() {  

    // To correctly read data from FLASH memory, the number of wait states (LATENCY)

  // must be correctly programmed according to the frequency of the CPU clock

  // (HCLK) and the supply voltage of the device.      

    FLASH->ACR &= ~FLASH_ACR_LATENCY;

    FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;

    // Enable the Internal High Speed oscillator (HSI)

    RCC->CR |= RCC_CR_HSION;

    while((RCC->CR & RCC_CR_HSIRDY) == 0);

    // Disable PLL for configuration

    RCC->CR    &= ~RCC_CR_PLLON;

    // Select clock source to PLL

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;        // Set source for PLL: clear bits

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // Set source for PLL: 0 =HSI, 1 = HSE

    // Make PLL as 84 MHz

    // f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 84/8 = 168 MHz

    // f(PLL_R) = f(VCO clock) / PLLP = 168MHz/2 = 84MHz

    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 84U << 6;

    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 8U ;

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;  // 00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8  

    // Enable PLL after configuration

    RCC->CR   |= RCC_CR_PLLON;

    while((RCC->CR & RCC_CR_PLLRDY)>>25 != 0);

    // Select PLL as system clock

    RCC->CFGR &= ~RCC_CFGR_SW;

    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until System Clock has been selected

    while ((RCC->CFGR & RCC_CFGR_SWS) != 8UL);

    // The maximum frequency of the AHB and APB2 is 100MHz,

    // The maximum frequency of the APB1 is 50 MHz.

    RCC->CFGR &= ~RCC_CFGR_HPRE;        // AHB prescaler = 1; SYSCLK not divided (84MHz)

    RCC->CFGR &= ~RCC_CFGR_PPRE1;       // APB high-speed prescaler (APB1) = 2, HCLK divided by 2 (42MHz)

    RCC->CFGR |=  RCC_CFGR_PPRE1_2;

    RCC->CFGR &= ~RCC_CFGR_PPRE2;       // APB high-speed prescaler (APB2) = 1, HCLK not divided    (84MHz)

    EC_SYSCLK=84000000;

}
```

#### RCC_GPIOA_enable()

enable the GPIOA

```c
void RCC_GPIOA_enable()
```

**Parameters**


**Example code**

```c
void RCC_GPIOA_enable()

{

    // RCC Peripheral Clock Enable Register

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

}
```
#### RCC_GPIOB_enable()

enable the GPIOB

```c
void RCC_GPIOB_enable()
```

**Parameters**


**Example code**

```c
void RCC_GPIOB_enable()

{

    // RCC Peripheral Clock Enable Register

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

}
```
#### RCC_GPIOC_enable()

enable the GPIOC

```c
void RCC_GPIOC_enable()
```

**Parameters**


**Example code**

```c
void RCC_GPIOC_enable()

{

    // RCC Peripheral Clock Enable Register

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

}
```
#### RCC_GPIOD_enable()

enable the GPIOD

```c
void RCC_GPIOD_enable()
```

**Parameters**


**Example code**

```c
void RCC_GPIOD_enable()

{

    // RCC Peripheral Clock Enable Register

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

}
```
#### RCC_GPIOE_enable()

enable the GPIOE

```c
void RCC_GPIOE_enable()
```

**Parameters**


**Example code**

```c
void RCC_GPIOE_enable()

{

    // RCC Peripheral Clock Enable Register

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

}
```
#### RCC_GPIOH_enable()

enable the GPIOH

```c
void RCC_GPIOH_enable()
```

**Parameters**


**Example code**

```c
void RCC_GPIOH_enable()

{

    // RCC Peripheral Clock Enable Register

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

}
```
