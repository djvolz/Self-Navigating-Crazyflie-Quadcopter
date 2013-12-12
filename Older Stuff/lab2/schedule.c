uint16_t T2_CCR1_Valx = 0;
uint16_t T2_CCR2_Valx = 2400;

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
  TIM_OCInitStructure.TIM_Pulse = T2_CCR1_Valx;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

  TIM_OCInitStructure.TIM_Pulse = T2_CCR2_Valx;
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
  //TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

  /* TIM2, TIM3 and TIM4 enable counters */
  TIM_Cmd(TIM2, ENABLE);
}

