#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecSysTick2.h"
#include "ecSTM32F4v2.h"
#include "ecEXTI2.h" // 

#define BUTTON_PIN PA_4 // 사용할 버튼 핀 (예: PA_4)

// --- 전역 변수 ---
int count = 0;
int select = 0;


// --- 초기화 함수 ---
void setup(void)
{
    RCC_PLL_init();
    SysTick_init();
    seven_seg_FND_init();

    // --- 👇 버튼 및 인터럽트 설정 추가 ---
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, EC_PU); // 내부 풀업 저항 사용
    EXTI_init(BUTTON_PIN, FALL, 0); // 하강 엣지에서 인터럽트 발생
}

// --- 메인 함수 ---
int main(void) { 
    setup();
    
    while(1){
        // 1. 현재 숫자를 7세그먼트에 표시
        // 1자리만 사용하므로 두 번째 인자는 0으로 고정
        seven_seg_FND_display(count, select);
		delay_ms(1000);
		count++;
		if (count == 10) count =0;
    }
}

void EXTI4_IRQHandler(void){
    if(is_pending_EXTI(BUTTON_PIN)){
        count = -1;
        clear_pending_EXTI(BUTTON_PIN); // 인터럽트 플래그 정리
    }
}