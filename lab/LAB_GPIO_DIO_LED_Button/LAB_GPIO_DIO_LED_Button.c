#include "ecRCC2.h"
#include "ecGPIO2.h"

#define LED_PIN     PB_12
#define BUTTON_PIN  PA_4

void setup(void) {
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  
    GPIO_init(LED_PIN, OUTPUT);    
}

int main(void) { 
    setup();
    int buttonState = 0;
    int prevButtonState = 1;   
    int ledState = 0;         

    while(1){
        buttonState = GPIO_read(BUTTON_PIN);

       
        if(prevButtonState == 1 && buttonState == 0){
           
            ledState = !ledState;
            GPIO_write(LED_PIN, ledState);
        }

        
        prevButtonState = buttonState;
    }
}
