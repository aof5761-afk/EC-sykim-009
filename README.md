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

#### GPIO\_pupd() <- 수정 필요

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
# EC-sykim-009
