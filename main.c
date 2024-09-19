#include "apm32f4xx.h"  // Change this to match your MCU series if necessary

#define LED_PIN    0      // Pin 0 corresponds to PE0
#define LED_PORT   GPIOE  // GPIOE for the LED

// Function prototypes
void SysTick_Handler(void);
void GPIO_Init(void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);
volatile uint32_t tick = 0;

int main(void) {
    // Initialize GPIO and SysTick
    GPIO_Init();
    SysTick_Init();
    while (1) {        // Toggle PE0
        LED_PORT->ODATA ^= (1 << LED_PIN);
        // Delay for 1 second
        delay_ms(1000);
    }
}

void GPIO_Init(void) {
    // Enable clock for GPIOE
    RCM->AHB1CLKEN_B.PEEN=1;

    // Set PE0 as output (mode = 01)
    LED_PORT->MODE &= ~(3 << (LED_PIN * 2)); // Clear the two bits first
    LED_PORT->MODE |= (1 << (LED_PIN * 2));  // Set the output mode (01)
    // Set PE0 output type to push-pull
    LED_PORT->OMODE &= ~(1 << LED_PIN);
    // Set output speed to low
    LED_PORT->OSSEL &= ~(3 << (LED_PIN * 2));
    // Set PE0 to no pull-up, no pull-down
    LED_PORT->PUPD &= ~(3 << (LED_PIN * 2));
}

void SysTick_Init(void) {
    // Configure SysTick to trigger every 1ms
    SysTick_Config(SystemCoreClock / 1000);  // SystemCoreClock is the MCU clock speed in Hz
}

void SysTick_Handler(void) {
    tick++;  // Increment the tick counter
}

void delay_ms(uint32_t ms) {
    uint32_t startTick = tick;
    while ((tick - startTick) < ms);
}
