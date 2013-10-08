#include "stm32f10x_conf.h"
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc/stm32f10x_flash.h"
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c"

//timer
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc/stm32f10x_tim.h"
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c"
//nvic
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc/misc.h"
#include "lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c"

#include "lab2.h"

// Green LED is PB5, pin 41, active low
#define RED_LED GPIO_Pin_4
#define GREEN_LED GPIO_Pin_5


// Motor handling
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t T2_CCR1_Val = 0;
uint16_t T2_CCR2_Val = 2400;
uint16_t T3_CCR1_Val = 0;
uint16_t T3_CCR2_Val = 0;


uint16_t CCR1_Val = 63;
uint16_t CCR2_Val = 63;
uint16_t CCR3_Val = 63;
uint16_t CCR4_Val = 63;
uint16_t PrescalerValue = 0;

int TIM2_Period = 4800-1;
int TIM3_Period = 666-1;
uint16_t capture = 0;

  int divider = 3279;
  int divider_count_1 = 0;
  int divider_count_2 = 0;
  int divider_count_3 = 0;

  /* LED will turn on if DEBUG set to 1 */
  int DEBUG = 0;
  int motorSelected = 0;


ErrorStatus HSEStartUpStatus;
#define SYSCLK_FREQ_72MHz  72000000
uint32_t SystemCoreClock = SYSCLK_FREQ_72MHz;        /*!< System Clock Frequency (Core Clock) */
void SetSysClockTo72(void);
void motorSwitch(void);

void ledInit(void) {
  // Initialize GPIO pin for green LED
  GPIO_InitTypeDef initStruct;
  initStruct.GPIO_Pin = GREEN_LED;
  initStruct.GPIO_Speed = GPIO_Speed_50MHz;
  initStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &initStruct);
  initStruct.GPIO_Pin = RED_LED;
  GPIO_Init(GPIOB, &initStruct);
}

void TIM2_Config(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Enable TIM2, TIM3 and TIM4 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* TIM2 configuration */
  TIM_TimeBaseStructure.TIM_Period = 4800 - 1;       
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock/1200) - 1);
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  //TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T2_CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

  TIM_OCInitStructure.TIM_Pulse = T2_CCR2_Val;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

  /* Immediate load of TIM2,TIM3 and TIM4 Precaler values */
  //TIM_PrescalerConfig(TIM2, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);
  //TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

  /* Clear TIM2, TIM3 and TIM4 update pending flags */
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);

  /* Configure two bits for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable TIM2, TIM3 and TIM4 Update interrupts */
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

  /* TIM2, TIM3 and TIM4 enable counters */
  TIM_Cmd(TIM2, ENABLE);
}

void ledToggle(void) {
  static unsigned int  state = 0;
  if (state) {
    // Pull up, off
    GPIO_SetBits(GPIOB, GREEN_LED);
    state ^= 1;
  } else {
    // Pull down, on
    GPIO_ResetBits(GPIOB, GREEN_LED);
    state ^= 1;
  }
}
void redLedToggle(void){
  static unsigned int  state = 0;
  if (state) {
    // Pull up, off
    GPIO_SetBits(GPIOB, RED_LED);
    state ^= 1;
  } else {
    // Pull down, on
    GPIO_ResetBits(GPIOB, RED_LED);
    state ^= 1;
  }

}

void motorSwitch(void) {
 if(motorSelected == 0){
   TIM3->CCR4 = 0;
   TIM4->CCR3 = 63;
   motorSelected++;
 }
 else if(motorSelected == 1){
   TIM4->CCR3 = 0;
   TIM4->CCR4 = 63;
   motorSelected++;
 }
 else if(motorSelected == 2){
   TIM4->CCR4 = 0;
   TIM3->CCR3 = 63;
   motorSelected++;
 }
 else if(motorSelected == 3){
   TIM3->CCR3 = 0;
   TIM3->CCR4 = 63;
   motorSelected = 0;
 }
}

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  /* Clear TIM2 update interrupt */
  //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    /* LED3 toggling with frequency = 219.7 Hz */
 //    ledToggle();
     //redLedToggle();
 // redLedToggle();
  motorSwitch();

    capture = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, (capture + T2_CCR1_Val) % TIM2_Period);
  }
  if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
  //  redLedToggle();

    capture = TIM_GetCapture2(TIM2);
    TIM_SetCompare2(TIM2, (capture + T2_CCR2_Val) % TIM2_Period);
  }
}
void TIM3_IRQHandler(void)
{

  //10Hz
  /* Clear TIM2 update interrupt */
  //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  //redLedToggle();
  MotorSpeeds* p_motorSpeedsPtr;
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	divider_count_1++;
	if(divider_count_1 > divider){
		//do stuff
		ledToggle();
		divider_count_1 = 0;
	}
	
    capture = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, (capture + T3_CCR1_Val) % TIM3_Period);
  }
  //100Hz
  if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	divider_count_2++;
	if(divider_count_2 > (divider/10)){
		//do stuff
		redLedToggle();
		divider_count_2 = 0;
	}
    capture = TIM_GetCapture2(TIM3);
    TIM_SetCompare2(TIM3, (capture + T3_CCR2_Val) % TIM3_Period);
  }
  
  //1Hz
  /* Clear TIM2 update interrupt */
  //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  //redLedToggle();
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	divider_count_3++;
	if(divider_count_3 > divider*10){
		//do stuff
		motorSwitch();
		
		//updateMotorSpeeds(
		
		calculateOrientation();
		divider_count_3 = 0;
	}
	
    capture = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, (capture + T3_CCR1_Val) % TIM3_Period);
  }
}

/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemCoreClock variable.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;  

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CFGR &= (uint32_t)0xF8FF0000;
  

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x00FF0000;


  SetSysClockTo72();
}

  /**
  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @param  None
  * @retval None
  */
void SetSysClockTo72(void)
{
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */    

    /* Go to infinite loop */
    while (1)
    {
    }
  }

}

/**
  * @brief  Configure the TIM3 Ouput Channels.
  * @param  None
  * @retval None
  */
void motor_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function push-pull */
  DEBUG = 1;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void motorHandler(void)
{
  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
  NVIC_InitTypeDef  NVIC_InitStructure;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 732;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T3_CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // CCR2_Val;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  /* Clear TIM2, TIM3 and TIM4 update pending flags */
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);

  /* Configure two bits for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable TIM2, TIM3 and TIM4 Update interrupts */
  TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);


  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

void motor_RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

int main(void) {
  SystemInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);
 
  TIM2_Config();

  motor_RCC_Configuration();
  motor_GPIO_Configuration();
  motorHandler();

  DEBUG = 1;
if(DEBUG) {
 ledInit();
}


  while (1) {
  }
}

