// Nofrete
// configuration of STM32 internal peripherals
// RCC, GPIO, USART3, ADC2, NVIC

/*******************************************************************/

void GPIO_Config(void)
{
    // enable GPIO A, B and C in RCC
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

       LL_GPIO_InitTypeDef GPIO_Init_Struct;

    // GPIO A: PA7 => Poti analog input
    LL_GPIO_StructInit(&GPIO_Init_Struct);
    GPIO_Init_Struct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_Init_Struct.Pin = LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOA, &GPIO_Init_Struct);

    // GPIO A: output PA5 => A_UP, PA9 => A_DOWN, PA8 => B_UP, PA6 => B_DOWN
    LL_GPIO_StructInit(&GPIO_Init_Struct);
    GPIO_Init_Struct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_Init_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_Init_Struct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_Init_Struct.Pull = LL_GPIO_PULL_NO;
    GPIO_Init_Struct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(GPIOA, &GPIO_Init_Struct);

    // GPIO B: alternate 7 for PB8 => USART3_RX, PB9 => USART3 TX
    LL_GPIO_StructInit(&GPIO_Init_Struct);
    GPIO_Init_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_Init_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_Init_Struct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_Init_Struct.Pull = LL_GPIO_PULL_NO;
    GPIO_Init_Struct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_Init_Struct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOB, &GPIO_Init_Struct);

    // GPIO B: input PB6 => Lochscheibe A
    LL_GPIO_StructInit(&GPIO_Init_Struct);
    GPIO_Init_Struct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_Init_Struct.Pin = LL_GPIO_PIN_6;
    GPIO_Init_Struct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_Init_Struct);

    // GPIO C: input PC7 => Lochscheibe B
    LL_GPIO_StructInit(&GPIO_Init_Struct);
    GPIO_Init_Struct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_Init_Struct.Pin = LL_GPIO_PIN_7;
    GPIO_Init_Struct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_Init_Struct);
}

/*******************************************************************/
// PB6 and PC7 => EXTI

void EXTI_Config(void)
{
    // enable SYSCFG matrix
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE6);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE7);

     LL_EXTI_InitTypeDef EXTI_InitStruct;
    LL_EXTI_StructInit (&EXTI_InitStruct);

    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6 | LL_EXTI_LINE_7;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    EXTI_InitStruct.LineCommand = ENABLE;

    // Struktur an EXTI übergeben und damit die Register beschreiben
    LL_EXTI_Init(&EXTI_InitStruct);
}

/*******************************************************************/
// TODO: more reasonable interrupt priorities

void NVIC_Config(void)
{
    uint32_t priority_grouping = 5;
    NVIC_SetPriorityGrouping(priority_grouping);
    uint32_t encoded_priority = NVIC_EncodePriority(priority_grouping,0,0);

    NVIC_SetPriority(TIM6_DAC_IRQn, encoded_priority);	// lowest
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

       NVIC_SetPriority(USART3_IRQn, encoded_priority);
    NVIC_EnableIRQ(USART3_IRQn);

    NVIC_SetPriority(EXTI9_5_IRQn, encoded_priority);	// highest priority
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/*******************************************************************/

void USART3_Config(void)
{
    // enable USART3 in RCC
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    LL_USART_InitTypeDef USART_Init_Struct;
    LL_USART_StructInit(&USART_Init_Struct);
                
    USART_Init_Struct.BaudRate = 9600;
    USART_Init_Struct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_Init_Struct.StopBits = LL_USART_STOPBITS_1;
    USART_Init_Struct.Parity = LL_USART_PARITY_NONE;
    USART_Init_Struct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_Init_Struct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    
    LL_USART_Init(USART3, &USART_Init_Struct);
    
    LL_USART_Enable(USART3);
    LL_USART_EnableIT_RXNE(USART3);
}

/*******************************************************************/
// TIM6 for 10 Hz used for potentiometer reading

void Timer_Config(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    LL_TIM_InitTypeDef TIM_Init_Struct;
    LL_TIM_StructInit(&TIM_Init_Struct);
    
    TIM_Init_Struct.Prescaler = 63999;  // assuming 64 Mhz => 1 kHz
    TIM_Init_Struct.Autoreload = 99;    // 10 Hz
        
    LL_TIM_Init(TIM6, &TIM_Init_Struct);
    
    LL_TIM_EnableIT_UPDATE(TIM6);
}

/*******************************************************************/
// ADC2 configuration and channel setup
// we continously read PA7 = potentiometer

void ADC_Config(void)
{
    // enable ADCs in RCC
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

    LL_ADC_CommonInitTypeDef ADC_CommonStruct;
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC2), LL_ADC_CLOCK_SYNC_PCLK_DIV1);  // use AHB clock for ADC
    
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
    LL_ADC_REG_StructInit(&ADC_REG_InitStruct);

    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;   // channel sequence list is only one entry long

    LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

    // create channel sequence list with PA7 = ADC2_IN4 = channel 4 at rank 1
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    
    // configure arbitrary sample time, not so relevant for potentiometer
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_61CYCLES_5);
}

/*******************************************************************/
// ADC calibration and startup

void ADC_Start(void)
{
    // check hardware constraint, ADC must not be enabled
    if (LL_ADC_IsEnabled(ADC2)) return;
    // enable analog ref voltage
    LL_ADC_EnableInternalRegulator(ADC2);   // requires 10 µs
    // wait with the pre-configured TIM6, then stop TIM6
    LL_TIM_EnableCounter(TIM6);
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) LL_TIM_ClearFlag_UPDATE(TIM6);
    while (!LL_TIM_IsActiveFlag_UPDATE(TIM6)){};
    LL_TIM_DisableCounter(TIM6);
    LL_TIM_ClearFlag_UPDATE(TIM6);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);

    // start calibration and wait until it is finished
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {};

    // insert 4 cycles, do just anything
    uint32_t calibration_value = LL_ADC_GetCalibrationFactor(ADC2,LL_ADC_SINGLE_ENDED);

    // enable ADC2 and wait until it is working
    LL_ADC_Enable(ADC2);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) {};
    // and start regular conversion
    LL_ADC_REG_StartConversion(ADC2);
}

/************************ (C) COPYRIGHT STMicroelectronics *********/
/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */ 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* Enable HSI and wait for activation*/
    LL_RCC_HSI_Enable(); 
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1) 
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  
  /* Set systick to 1ms in using frequency set to 64MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ ((HSI_VALUE / 2), LL_RCC_PLL_MUL_16) */
  LL_Init1msTick(64000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}
/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */
