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

### TIMER
#### Header File
#### TIM_init()

Initiate the TIMER

```c
void TIM_init(TIM_TypeDef* TIMx)
```

**Parameters**
- TIM_TypeDef* TIMx : Timer that will be initiated

**Example code**

```c

void TIM_init(TIM_TypeDef* TIMx){    

    // Previous version:  void TIM_init(TIM_TypeDef* TIMx, uint32_t msec)  

    // 1. Enable Timer CLOCK

    if(TIMx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    else if(TIMx ==TIM2) RCC->APB1ENR |= 1<<0;

    else if(TIMx ==TIM3) RCC->APB1ENR |= 1<<1;

    // repeat for TIM4, TIM5, TIM9, TIM11

    else if(TIMx ==TIM4) RCC->APB1ENR |= 1<<2;

    else if(TIMx ==TIM5) RCC->APB1ENR |= 1<<3;

    else if(TIMx ==TIM9) RCC->APB2ENR |= 1<<16;

    else if(TIMx ==TIM11) RCC->APB2ENR |= 1<<18;

    // 2. Set CNT period

     uint32_t msec=1;

    TIM_period_ms(TIMx, msec);

    // 3. CNT Direction

    TIMx->CR1 &=~(1<<4);                    // Upcounter    

    // 4. Enable Timer Counter

    TIMx->CR1 |= TIM_CR1_CEN;      

}
```

#### TIM_period_us()

Select the Timer Period

```c
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec)
```

**Parameters**
- TIM_TypeDef* TIMx : Selected Timer
- usec : Timer period usec unit 

**Example code**

```c

void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){  

    //  Q. Which combination of PSC and ARR for msec unit?

    //  Q. What are the possible range (in sec ?)

  

    // 0.01ms(100kHz, ARR = 1) to 655 msec (ARR = 0xFFFF)

    // 0.01ms(100kHz, ARR = 1) to 40,000,000 msec (ARR = 0xFFFF FFFF)

  

    // 1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)

    uint16_t PSCval;

    uint32_t Sys_CLK;

  

    if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL)

        Sys_CLK = 84000000;

    else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI)

        Sys_CLK = 16000000;

    if (TIMx == TIM2 || TIMx == TIM5){

        uint32_t ARRval;

        PSCval = Sys_CLK/1000000;                       // 84 or 16 --> f_cnt = 1MHz

        ARRval = Sys_CLK/PSCval/1000000 * usec;                     // ARRval= 1*usec

        TIMx->PSC = PSCval - 1;

        TIMx->ARR = ARRval - 1;            

    }

    else{

        uint16_t ARRval;

  

        PSCval = Sys_CLK/1000000;                       // 84 or 16 --> f_cnt = 1MHz

        ARRval = Sys_CLK/PSCval/1000000 * usec;                     // ARRval= 1*usec

        TIMx->PSC = PSCval - 1;

        TIMx->ARR = ARRval - 1;

    }          

}
```

#### TIM_period_ms()

Select the Timer Period

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec)
```

**Parameters**
- TIM_TypeDef* TIMx : Selected Timer
- msec : Timer period msec unit 

**Example code**

```c

void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){

    //  Q. Which combination of PSC and ARR for msec unit?

    //  Q. What are the possible range (in msec ?)

  

    // 0.02ms(50kHz, ARR=1) to 1.3sec (ARR=0xFFFF)

    //uint32_t prescaler = 1680;

  

    // 0.1ms(10kHz, ARR = 1) to 6.5sec (ARR = 0xFFFF)

    uint16_t PSCval;

    uint32_t Sys_CLK;

    if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL )

         Sys_CLK = 84000000;

    else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI)

        Sys_CLK = 16000000;

    if (TIMx == TIM2 || TIMx == TIM5){

        uint32_t ARRval;        

        PSCval = Sys_CLK/100000;        // 840 or 160   --> PSC_clk=f_cnt = 100kHz

        ARRval = (Sys_CLK/PSCval/1000) * msec;      // 100kHz*msec,  ARRval=100 for 1msec

        TIMx->PSC = PSCval - 1;

        TIMx->ARR = ARRval - 1;

    }

    else{

        uint16_t ARRval;

  

              PSCval = Sys_CLK / 1000;                   // ★ f_cnt = 1 kHz

        ARRval = (uint16_t)((Sys_CLK / PSCval / 1000) * msec); // = 1 * msec            

        TIMx->PSC = PSCval - 1;

        TIMx->ARR = ARRval - 1;

    }

}```


#### TIM_period()

Select the Timer Period

```c
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec)
```

**Parameters**
- TIM_TypeDef* TIMx : Selected Timer
- msec : Timer period msec unit 

**Example code**

```c

void TIM_period(TIM_TypeDef* TIMx, uint32_t msec){

    TIM_period_ms(TIMx, msec);

}
```

#### TIM_UI_init()

Initiate the TIMER(Upgrade Ver)

```c
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec)
```

**Parameters**
- TIM_TypeDef* TIMx : Selected Timer
- msec : Timer period msec unit 

**Example code**

```c

void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec){

    // 1. Initialize Timer  

    TIM_init(TIMx);

    // TIM_period_us(TIMx,msec);

    TIM_period_ms(TIMx,msec);

    // 2. Enable Update Interrupt

    TIM_UI_enable(TIMx);

    // 3. NVIC Setting

    uint32_t IRQn_reg =0;

    if(TIMx == TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;

    else if(TIMx == TIM2)  IRQn_reg = TIM2_IRQn;//< TIM2 global Interrupt                                             */

    // repeat for TIM3, TIM4, TIM5, TIM9, TIM10, TIM11

    else if(TIMx == TIM3)  IRQn_reg =  TIM3_IRQn; //< TIM3 global Interrupt

    else if(TIMx == TIM4)  IRQn_reg = TIM4_IRQn;  //< TIM4 global Interrupt

    else if(TIMx == TIM5)  IRQn_reg = TIM5_IRQn; //< TIM4 global Interrupt

    else if(TIMx == TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;

    else if(TIMx == TIM10)  IRQn_reg = TIM1_UP_TIM10_IRQn;

    else if(TIMx == TIM11)  IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;

    NVIC_EnableIRQ(IRQn_reg);              

    NVIC_SetPriority(IRQn_reg,2);

}
```


#### TIM_UI_enable()

Enable Timer

```c
void TIM_UI_enable(TIM_TypeDef* TIMx)
```

**Parameters**
- TIM_TypeDef* TIMx : Timer that will be Enabled

**Example code**

```c

void TIM_UI_enable(TIM_TypeDef* TIMx){

    TIMx->DIER |= 1<<0;         // Enable Timer Update Interrupt        

}
```

#### TIM_UI_disable()

Disable Timer

```c
void TIM_UI_disable(TIM_TypeDef* TIMx)
```

**Parameters**
- TIM_TypeDef* TIMx : Timer that will be Disabled

**Example code**

```c

void TIM_UI_disable(TIM_TypeDef* TIMx){

    TIMx->DIER &= ~(1<<0);              // Disable Timer Update Interrupt      

}
```

#### is_UIF()

Check the timer tick

```c
uint32_t is_UIF(TIM_TypeDef *TIMx)
```

**Parameters**
- TIM_TypeDef* TIMx : Timer that will be checked

**Example code**

```c

uint32_t is_UIF(TIM_TypeDef *TIMx){

    return (TIMx->SR & 1);

}
```


#### clear_UIF()

clear the timer tick

```c
void clear_UIF(TIM_TypeDef *TIMx)
```

**Parameters**
- TIM_TypeDef* TIMx : Timer that will be checked

**Example code**

```c

void clear_UIF(TIM_TypeDef *TIMx){

    TIMx->SR &= ~1;

}
```

### PWM
#### Header File
#### PWM_init()

Initiate the PWM 

```c
void PWM_init(PinName_t pinName)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected

**Example code**

```c

void PWM_init(PinName_t pinName){

  

// 0. Match TIMx from  Port and Pin    

    GPIO_TypeDef *port;

    unsigned int pin;

    ecPinmap(pinName, &port, &pin);

    TIM_TypeDef *TIMx;

    int chN;

    PWM_pinmap(pinName, &TIMx, &chN);

// 1. Initialize GPIO port and pin as AF    

    GPIO_init(pinName,AF);  // AF=2

  

    // GPIO_otype(port, pin, EC_PUSH_PULL);     //if necessary

    // GPIO_pupd(port\, pin, EC_PU);                    //if necessary

// 2. Configure GPIO AFR by Pin num.    

    //  AFR[0] for pin: 0~7,     AFR[1] for pin 8~15

    //  AFR=1 for TIM1,TIM2 AFR=2 for TIM3 etc  

   if((TIMx == TIM1) || (TIMx == TIM2)){

    port->AFR[pin >>3] |=0x01 <<(4*(pin %8 ));

   }

   else if(TIMx == TIM3 || TIMx==TIM4 || TIMx==TIM5)

   {

    port->AFR[pin>>3]|=0x02<<(4*(pin % 8));

   }

// 3. Initialize Timer

    TIM_init(TIMx); // with default msec=1msec value.      

    TIMx->CR1 &= ~TIM_CR1_CEN;

// 3-2. Direction of Counter

    //YOUR CODE GOES HERE

    TIMx->CR1 &= ~TIM_CR1_DIR;                          // Counting direction: 0 = up-counting, 1 = down-counting

// 4. Configure Timer Output mode as PWM

    uint32_t ccVal = TIMx->ARR/2;  // default value  CC=ARR/2

    if(chN == 1){

        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1

        TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // OC1M = 110 for PWM Mode 1 output on ch1. #define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)

        TIMx->CCMR1 |= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)

        TIMx->CCR1  = ccVal;                                                                // Output Compare Register for channel 1 (default duty ratio = 50%)    

        TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high  

        TIMx->CCER  |= TIM_CCER_CC1E;                                               // Enable output for ch1

    }

    else if(chN == 2){

        TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                  

        TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

        TIMx->CCMR1 |= TIM_CCMR1_OC2PE;                    

        TIMx->CCR1  = ccVal;                                                                    

        TIMx->CCER &= ~TIM_CCER_CC2P;                    

        TIMx->CCER  |= TIM_CCER_CC2E;                                              

    }

    else if(chN == 3){

        TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear ouput compare mode bits for channel 3                  

        TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

        TIMx->CCMR2 |= TIM_CCMR2_OC3PE;                  

        TIMx->CCR1  = ccVal;                                                                    

        TIMx->CCER &= ~TIM_CCER_CC3P;                    

        TIMx->CCER  |= TIM_CCER_CC3E;                                                      

    }

    else if(chN == 4){

        TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;                                    

        TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

        TIMx->CCMR2 |= TIM_CCMR2_OC4PE;                  

        TIMx->CCR1  = ccVal;                                                                    

        TIMx->CCER &= ~TIM_CCER_CC4P;                    

        TIMx->CCER  |= TIM_CCER_CC4E;                                                      

    }  

// 5. Enable Timer Counter

    // For TIM1 ONLY

    if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;                    // Main output enable (MOE): 0 = Disable, 1 = Enable    

    // Enable timers

    TIMx->CR1  |= TIM_CR1_CEN;                                                      // Enable counter

}
```

#### PWM_period_ms()

Selec the PWM period

```c
void PWM_period_ms(PinName_t pinName,  uint32_t msec)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- msec : PWM period (unit : msec)

**Example code**

```c

void PWM_period_ms(PinName_t pinName,  uint32_t msec){

// 0. Match TIMx from  Port and Pin    

    GPIO_TypeDef *port;

    unsigned int pin;  

    ecPinmap(pinName, &port, &pin);

    TIM_TypeDef *TIMx;

    int chN;        

    PWM_pinmap(pinName, &TIMx, &chN);

// 1. Set Counter Period in msec

    TIM_period_ms(TIMx, msec);

}```

#### PWM_period() 

Selec the PWM period

```c
void PWM_period(PinName_t pinName,  uint32_t msec)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- msec : PWM period (unit : msec)

**Example code**

```c

void PWM_period(PinName_t pinName,  uint32_t msec){

    PWM_period_ms(pinName,  msec);
}```
#### PWM_period_us() 

Selec the PWM period

```c
void PWM_period_us(PinName_t pinName,  uint32_t usec)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- usec : PWM period (unit : usec)

**Example code**

```c

void PWM_period_us(PinName_t pinName,  uint32_t usec){

  

// 0. Match TIMx from  Port and Pin    

    GPIO_TypeDef *port;

    unsigned int pin;  

    ecPinmap(pinName, &port, &pin);

    TIM_TypeDef *TIMx;

    int chN;        

    PWM_pinmap(pinName, &TIMx, &chN);

  

// 1. Set Counter Period in usec

    TIM_period_us(TIMx, usec);  //YOUR CODE GOES HERE
}```
#### PWM_pulsewidth()

Selec the PWM period

```c
void PWM_pulsewidth(PinName_t pinName, double pulse_width_ms)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- pulse_width_ms : select the pulse width range (unit : ms)

**Example code**

```c

void PWM_pulsewidth(PinName_t pinName, double pulse_width_ms){

// 0. Match TIMx from  Port and Pin    

    GPIO_TypeDef *port;

    unsigned int pin;  

    ecPinmap(pinName, &port, &pin);

    TIM_TypeDef *TIMx;

    int chN;        

    PWM_pinmap(pinName, &TIMx, &chN);

  

// 1. Declaration System Frequency and Prescaler

    uint32_t fsys = 0;

    uint32_t psc = TIMx->PSC;

  

// 2. Check System CLK: PLL or HSI

    if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL)        fsys = 84000;  // for msec 84MHz/1000 [msec]

    else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) fsys = 16000;

  

// 3. Configure prescaler PSC

    float fclk = fsys/(psc+1);                  // fclk=fsys/(psc+1);

    uint32_t value = pulse_width_ms *fclk - 1;                  // pulse_width_ms *fclk - 1;

  

    switch(chN){

        case 1: TIMx->CCR1 = value; break;

        case 2: TIMx->CCR2 = value; break;

        case 3: TIMx->CCR3 = value; break;

        case 4: TIMx->CCR4 = value; break;

        // REPEAT for CHn=2,  3, 4

        // REPEAT for CHn=2,  3, 4

        // REPEAT for CHn=2,  3, 4

        default: break;

    }

}```
#### PWM_period_us() 

Selec the PWM period

```c
void PWM_period_us(PinName_t pinName,  uint32_t usec)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- usec : select the PWM period (unit : usec)

**Example code**

```c

void PWM_pulsewidth_ms(PinName_t pinName, double pulse_width_ms){

    PWM_pulsewidth(pinName, pulse_width_ms);

}```

#### PWM_pulsewidth_us() 

Selec the PWM period

```c
void PWM_pulsewidth_us(PinName_t pinName, double pulse_width_us)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- pulse_width_ms : select the pulse width range (unit : us)

**Example code**

```c

void PWM_pulsewidth_us(PinName_t pinName, double pulse_width_us){

// 0. Match TIMx from  Port and Pin    

    GPIO_TypeDef *port;

    unsigned int pin;  

    ecPinmap(pinName, &port, &pin);

    TIM_TypeDef *TIMx;

    int chN;        

    PWM_pinmap(pinName, &TIMx, &chN);

// 1. Declaration system frequency and prescaler

    uint32_t fsys = 0;

    uint32_t psc = TIMx->PSC;

  

// 2. Check System CLK: PLL or HSI

    if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL)        fsys = 84;  // for msec 84MHz/1000000 [usec]

    else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) fsys = 16;

  

// 3. Configure prescaler PSC

    float fclk = fsys/(psc+1);              // fclk=sfys/(psc+1);

    uint32_t value = pulse_width_us *fclk - 1;                  // pulse_width_ms *fclk - 1;

    switch(chN){

        case 1: TIMx->CCR1 = value; break;

        case 2: TIMx->CCR2 = value; break;

        case 3: TIMx->CCR3 = value; break;

        case 4: TIMx->CCR4 = value; break;

        default: break;

    }

}```

#### PWM_duty() 

Selec the PWM duty

```c
void PWM_duty(PinName_t pinName, float duty)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- duty : select the PWM duty

**Example code**

```c

void PWM_duty(PinName_t pinName, float duty){

// 0. Match TIMx from  Port and Pin    

    GPIO_TypeDef *port;

    unsigned int pin;  

    ecPinmap(pinName, &port, &pin);

    TIM_TypeDef *TIMx;

    int chN;        

    PWM_pinmap(pinName, &TIMx, &chN);

  

// 1. Configure prescaler PSC

  

    uint32_t value = ((TIMx->ARR)+1)*duty;                                  // (ARR+1)*dutyRatio - 1

  

    if(duty <= 0.f)

        value = 0;

    else if(duty >= 1.f)

        value = TIMx->ARR + 1;

    if(chN == 1)      { TIMx->CCR1 = value; }  

    else if(chN == 2)      { TIMx->CCR2 = value; }

    else if(chN == 3)      { TIMx->CCR3 = value; }

    else{ TIMx->CCR4 = value; }         //set channel      

    // REPEAT for CHn=2,  3, 4

    // REPEAT for CHn=2,  3, 4

    // REPEAT for CHn=2,  3, 4

  
}```

#### PWM_pinmap() 

PWM pinmap function

```c
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN)
```

**Parameters**
- PinName_t pinName : pin Name that will be selected
- TIMx : Timer
- chN : Number of Channel

**Example code**

```c

void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN){

    GPIO_TypeDef *port;

    unsigned int pin;      

    ecPinmap(pinName, &port, &pin);

  if(port == GPIOA) {

      switch(pin){

         case 0 : *TIMx = TIM2; *chN    = 1; break;

         case 1 : *TIMx = TIM2; *chN = 2; break;

         case 5 : *TIMx = TIM2; *chN = 1; break;

         case 6 : *TIMx = TIM3; *chN = 1; break;

         //case 7: TIMx = TIM1; *chN = 1N; break;

         case 8 : *TIMx = TIM1; *chN = 1; break;

         case 9 : *TIMx = TIM1; *chN = 2; break;

         case 10: *TIMx = TIM1; *chN = 3; break;

         case 15: *TIMx = TIM2; *chN = 1; break;

         default: break;

      }        

   }

   else if(port == GPIOB) {

      switch(pin){

         //case 0: TIMx = TIM1; *chN = 2N; break;

         //case 1: TIMx = TIM1; *chN = 3N; break;

         case 3 : *TIMx = TIM2; *chN = 2; break;

         case 4 : *TIMx = TIM3; *chN = 1; break;

         case 5 : *TIMx = TIM3; *chN = 2; break;

         case 6 : *TIMx = TIM4; *chN = 1; break;

         case 7 : *TIMx = TIM4; *chN = 2; break;

         case 8 : *TIMx = TIM4; *chN = 3; break;

         case 9 : *TIMx = TIM4; *chN = 4; break;

         case 10: *TIMx = TIM2; *chN = 3; break;    

         default: break;

      }

   }

   else if(port == GPIOC){

      switch(pin){

         case 6 : *TIMx = TIM3; *chN = 1; break;

         case 7 : *TIMx = TIM3; *chN = 2; break;

         case 8 : *TIMx = TIM3; *chN = 3; break;

         case 9 : *TIMx = TIM3; *chN = 4; break;

         default: break;

      }

   }

     // TIM5 needs to be added, if used.

}

}```

