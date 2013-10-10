/* Reed Jones
 * Daniel Volz
 * Will Xie
 */

#include "system_stm32f10x.c"
#include "stm32f10x_conf.h"

//timer
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc/stm32f10x_tim.h"
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c"
//nvic
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc/misc.h"
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c"

// Green LED is PB5, pin 41, active low
#define GREEN_LED GPIO_Pin_5
#define RED_LED   GPIO_Pin_4
#define LEDS      (GREEN_LED)//|RED_LED)

void ledInit(void) {
  // Initialize GPIO pin for green LED
  GPIO_InitTypeDef initStruct;
  initStruct.GPIO_Pin = LEDS;
  initStruct.GPIO_Speed = GPIO_Speed_50MHz;
  initStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &initStruct);
}

void TIM_Config(void) { 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Enable TIM2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* TIM2 configuration */

  /* Setting TIM_Period to k-1 slows the clock by a factor of k,
     so the frequency would be 72MHz/1200 = 60KHz */
  TIM_TimeBaseStructure.TIM_Period = 1200 - 1;       
  /* Prescaler = (freq_base / (2*freq_desired)) - 1
     Now account for the factor of 600 in the prescaler. This will result in a 1Hz clock signal for TIM2. */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock/(2*600)) - 1);
  /* No further clock division is necessary since we have already acheived 1Hz */
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;

  /* Immediate load of TIM2 value */
  TIM_PrescalerConfig(TIM2, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);

  /* Clear TIM2 update pending flag */
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);

  /* Configure two bits for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable TIM2 Update interrupts */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  /* TIM2 enable counters */
  TIM_Cmd(TIM2, ENABLE);
}


void ledToggle(void) {
  static unsigned int state = 0;
  if (state) {
    // Pull up, off
    GPIO_SetBits(GPIOB, LEDS);
    state ^= 1;
  } else {
    // Pull down, on
    GPIO_ResetBits(GPIOB, LEDS);
    state ^= 1;
  }
}


int main(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);
  ledInit();
  TIM_Config();
  while (1);
}

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void) {
  /* Clear TIM2 update interrupt */
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  
  ledToggle();
}
