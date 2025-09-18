/*----------------------------------------------------------------\
@ Embedded Controller by Sang-Yoon Kim - Handong Global University
Author           : Sang Yoon Kim
Created          : 09-16-2025
Modified         : 09-16-2025
Language/ver     : C in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO2.h"

void GPIO_init(PinName_t pinName, int mode){     
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName, &Port, &pin);
	
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
    if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
    if (Port == GPIOE)
		RCC_GPIOE_enable();
    if (Port == GPIOH)
		RCC_GPIOH_enable();        
	//[TO-DO] YOUR CODE GOES HERE
	// Make it for GPIOB, GPIOD..GPIOH

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(pinName, mode);
}


void GPIO_mode(PinName_t pinName, int mode){
 	GPIO_TypeDef *port;
	unsigned int pin;
	ecPinmap(pinName, &port, &pin);
    
	port->MODER &= ~(3UL<<(2*pin));     
	port->MODER |= mode<<(2*pin);    
}

// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(PinName_t pinName, int speed){
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);

    port->OSPEEDR &= ~(3UL << (2*pin));

    port->OSPEEDR |= ((uint32_t)speed << (2*pin));
}


// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, int type){
  GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);

    port->OTYPER &= ~(1UL << (pin));

    port->OTYPER |= ((uint32_t)type << (pin));
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, int pupd){
 GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);

   port->PUPDR &= ~(3UL << (2*pin));

    port->PUPDR|= ((uint32_t)pupd << (2*pin));
}

int GPIO_read(PinName_t pinName){
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);

 
    return ( (port->IDR >> pin) & 0x1 );
}

void GPIO_write(PinName_t pinName, int output){
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);

    if(output) 
        port->ODR |= (1UL << pin);   // HIGH
    else 
        port->ODR &= ~(1UL << pin);  // LOW
}

