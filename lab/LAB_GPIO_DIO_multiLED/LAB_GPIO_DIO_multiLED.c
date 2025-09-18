#include "ecRCC2.h"
#include "ecGPIO2.h"

#define BUTTON_PIN   PA_4                  // Button connected to PA4 (Input Pull-up)
static const PinName_t LEDS[4] = { PB_12, PB_13, PB_14, PB_15 }; // Four LEDs as outputs

static void set_only_led(int idx) {
  for (int i=0; i<4; i++) {
    GPIO_write(LEDS[i], (i==idx) ? HIGH : LOW);  // Only idx LED ON
  }
}

static void setup(void) {
  RCC_HSI_init();                       // Initialize system clock

  GPIO_init(BUTTON_PIN, INPUT);         // Configure button as input
  GPIO_pupd(BUTTON_PIN, EC_PU);         // Enable internal pull-up resistor

  // Configure all LED pins as outputs
  for (int i=0; i<4; i++) {
    GPIO_init(LEDS[i], OUTPUT);
    GPIO_otype(LEDS[i], 0);             // Push-pull
    GPIO_pupd(LEDS[i], EC_PU);          // Default pull-up (not essential for output)
    GPIO_ospeed(LEDS[i], 1);            // Medium speed
  }

  set_only_led(0);                      // Start with LED0 ON
}

int main(void) {
  setup();

  int cur = 1, prev = 1;   // Track button states
  int idx = 0;             // LED index (0~3)

  while (1) {
    cur = GPIO_read(BUTTON_PIN);        // Read button input

    // Detect falling edge: Released (1) → Pressed (0)
    if (prev == 1 && cur == 0) {
      idx = (idx + 1) & 0x3;            // Increase index, wrap around 0~3
      set_only_led(idx);                // Update LEDs
    }

    prev = cur;                         // Save current state
  }
}
