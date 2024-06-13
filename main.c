/*
  ******************************************************************************
  HIER BITTE NAMEN + MATRIKELNUMMER EINTRAGEN
	
	In diesem Projekt gilt:
	*=============================================================================
  *        SYSCLK(Hz)                             | 64000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 2
  *=============================================================================
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "functions.h"

/** @addtogroup STM32F3xx_LL_Examples
  * @{
  */

/** @addtogroup Templates_LL
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Private functions ---------------------------------------------------------*/


void GPIO_Config(void);
void USART3_Config(void);
void Timer_Config(void);
void ADC_Config(void);
void ADC_Start(void);
void EXTI_Config(void);
void NVIC_Config(void);

void check_if_traction_hold(void);
void check_traction_is_bottom(void);
void check_traction_is_top(void);
void check_brake_is_bottom(void);
void check_brake_is_top(void);
void check_if_brake_hold(void);

void start_operation(void);


/*******************************************************************/

typedef enum
{
    S_INIT,
		S_STATUS,
    S_CAL,
    S_OPERATION,
} E_STATE;

// expected state: A is free to move, B is either engaged or free
typedef enum
{
	S_EVAL_POT,	// read value and determine the position
	S_GOTO_POS_A,
	S_HOLD1_A,
	S_RELEASE_B,	// if not already released
	S_TURN_UP_A,
	S_HOLD2_A,
	S_GOTO_POS_B,
	S_BRAKE_B,
	S_RELEASE_A, // => from here to S_EVLA_POT or loop to S_GOTO_POS_A

	S_HOLD3_A,
	S_RELEASE2_B,
	S_GOTO_DOWN_B,
	S_TURN_DOWN_A,
	S_HOLD4_A,
	S_BRAKE2_B,
	S_RELEASE2_A,
	S_GOTO_POS2_A, // => from here to S_EVLA_POT or loop to S_GOTO_POS_A

} E_SUBSTATE;

E_STATE state_A = S_INIT;
E_STATE state_B = S_INIT;
E_SUBSTATE state_op = S_GOTO_POS_A;

int32_t pos_traction = 0, pos_brake = 0;
int32_t target_traction = -1, target_brake = -1;
int32_t end_traction = 0, end_brake = 0;
uint8_t dir_brake = 'n', dir_traction = 'n';    // up, down, neutral
uint8_t traction_is_top = 0;
uint8_t break_is_top = 0;
uint16_t work_position = 0;
uint16_t target_position = 0;
int adcValue=0;

/*******************************************************************/

int main(void)
{
    SystemClock_Config();
    GPIO_Config();
    EXTI_Config();
    USART3_Config();
    Timer_Config();
    ADC_Config();
    ADC_Start();
    NVIC_Config();

    // Timer 6 starten	
    LL_TIM_EnableCounter(TIM6);

    state_B = S_CAL;
    state_A = S_CAL;
		check_if_traction_hold();

    while (1); 
}

/*******************************************************************
IO: 
Linear drive A: PA5 = UP, PA9 = DOWN, B: PA8 = UP, PA6 = DOWN
Punched disk A: PB6, B: PC7
Potentiometer: PA7
USART3 RX: PB8, TX: PB9

TIM6 is configured with 10 Hz timer interrupt

 *******************************************************************
Statemachine:

startup -> reset -> status -> calibration -> workmode

reset:
tx 00 => expect 04 done 05 is
tx 80 => expect 24 done 25 is

status:
stopper A bottom: tx 90 => 08 nein 09 ja
stopper A top: tx 91 => 0a nein 0b ja
stopper B bottom: tx 92 => 28 nein 29 ja
stopper B top: tx 93 => 2a nein 2b ja

calibration:
- if not yet stopper top: move up till rx 0d (A) or 2d (B) => stop
- set counter to 0 count up
- move down till rx 0c (A) or 2c (B) => stop
- read counter
- calculate step width
- set counter to 0 count up
- done

workmode:
- 

*/

/*******************************************************************/
// called every 100 ms = 10 Hz
// read out potentionmeter, smoothen, and derive directive

#define ADC_MIN 280
#define ADC_MAX 4075
void TIM6_DAC_IRQHandler()
{
    adcValue = LL_ADC_REG_ReadConversionData12(ADC2);
		// mapping from ADC_MIN .. ADC_MAX to 0 .. 11
		if (state_op == S_EVAL_POT)
		{
			target_position = (int)((float)(adcValue - ADC_MIN) / (ADC_MAX-ADC_MIN) * 13);
			if  (work_position < target_position)
			{
				state_op = S_GOTO_POS_A;
				move_traction_to_pos();
			}
			else
			{
				if (work_position > target_position)
				{
					work_position--;
					state_op = S_HOLD3_A;
					set_traction_to_hold();
				}
			}
		}
    LL_TIM_ClearFlag_UPDATE(TIM6);
}

/*******************************************************************/
// received a byte from NOFRETETE

void USART3_IRQHandler()
{
    int recvd;
    
    // read from RDR performs also reset RXNE flag
    recvd = LL_USART_ReceiveData8(USART3);
    switch (recvd)
    {
				case 0x02:
					check_if_brake_hold();
				break;
				case 0x03:
				release_traction();
				break;

        case 0x04: // barrel traction action finished
        case 0x05: // barrel traction already in the state
				
				if(state_A == S_CAL)
				{					
					check_if_brake_hold();
					return;
				}
				
				switch (state_op)
				{
					case S_HOLD1_A:
					state_op = S_RELEASE_B;
					release_brake();
					break;
					
					case S_TURN_UP_A:
					state_op = S_HOLD2_A;
					set_traction_to_hold();
					break;
					
					case S_HOLD2_A:
					state_op = S_GOTO_POS_B;
					move_brake_to_pos();
					break;
					
					case S_RELEASE_A:
					state_op = S_EVAL_POT;
					work_position++;
					break;
					
					case S_HOLD3_A:
					state_op = S_RELEASE2_B;
					release_brake();
					break;
					case S_TURN_DOWN_A:
						state_op= S_HOLD4_A;
						set_traction_to_hold();
						break;
					case S_HOLD4_A:
						state_op = S_BRAKE2_B;
						activate_brake();
						break;
					
					case S_RELEASE2_A:
					state_op = S_GOTO_POS2_A;
					move_traction_to_pos();
					break;
					
					default: ;
				}
        break;

					case 0x22:
						check_traction_is_top();
						break;
					case 0x23:
						release_brake();
						break;

				case 0x24: // break action finished
        case 0x25: // break action already in the state
				if(state_B == S_CAL)
				{
					check_traction_is_top();
					return;
				}
				
				switch (state_op)
				{
					case S_RELEASE_B:
					state_op = S_TURN_UP_A;
					set_turn_barrel(31, UP);
					break;
					
					case S_BRAKE_B:
					state_op = S_RELEASE_A;
					release_traction();
					break;
					
					case S_RELEASE2_B:
					state_op = S_GOTO_DOWN_B;
					move_brake_to_pos();
					break;
					
					case S_GOTO_DOWN_B:
						state_op = S_BRAKE_B;
						activate_brake();
						break;

					case S_BRAKE2_B:
					state_op = S_RELEASE2_A;
					release_traction();
					break;

					default: ;
				}
        break;

        case 0x08: // status stopper A bottom not reached
        case 0x09: // status stopper A bottom active
        case 0x0a: // status stopper A top not reached
				traction_is_top = 0;
				move_traction_up();
				check_brake_is_top();
				break;
        case 0x0b: // status stopper A top active
				traction_is_top = 1;
				end_traction = pos_traction;
				move_traction_down();
				check_brake_is_top();
				break;
        case 0x28: // status stopper B bottom not reached
        case 0x29: // status stopper B bottom active
        case 0x2a: // status stopper B top not reached
				break_is_top = 0;
				move_brake_up();
				break;
        case 0x2b: // status stopper B top active
				break_is_top = 1;
				end_brake = pos_brake;
				move_brake_down();
        break;

        case 0x0c: // stopper A bottom reached
        stop_traction();
        if (state_A == S_CAL) {
            // position might be negative if the lin drive was not at the bottom
            end_traction = -pos_traction;
						pos_traction = 0;
            state_A = S_OPERATION;
						if (state_B == S_OPERATION) {
								start_operation();
						}
        }
        break;
        case 0x0d: // stopper A top reached
        stop_traction();
        if (state_A == S_CAL) {
            pos_traction = 0;
            move_traction_down();
        }
        break;

        case 0x2c: // stopper B bottom reached
        stop_brake();
        if (state_B == S_CAL) {
            // position might be negative if the lin drive was not at the bottom
            end_brake = -pos_brake;
						pos_brake = 0;
            state_B = S_OPERATION;
						if (state_A == S_OPERATION) {
								start_operation();
						}
        }
        break;
        case 0x2d: // stopper B top reached
        stop_brake();
        if (state_B == S_CAL) {
            pos_brake = 0;
            move_brake_down();
        }
        break;

        default: ;   // everything else should not come => safety reaction
//        stop_traction();
//        stop_brake();
        // reset nofretete and restart statemachine
    }
}

/*******************************************************************/

#define CORR_TRANSACTION 1
int32_t get_pos_traction (uint16_t role_pos)
{
	return (int32_t)((float)end_traction / 12.5 * role_pos + CORR_TRANSACTION);
}

#define CORR_brake 1.5
int32_t get_pos_brake (uint16_t role_pos)
{
	return (int32_t)((float)end_brake / 12.5 * (role_pos + 0.5) + CORR_brake);
}

// both sides are calibrated and back at the bottom.
void start_operation(void)
{
	state_op = S_GOTO_POS_A;
	target_traction = get_pos_traction (work_position);
	if (work_position == target_position) state_op = S_EVAL_POT;
  else move_traction_to_pos();
}
// stop linear drive for A
void stop_traction()
{
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_5);
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_9);
		if (state_op == S_GOTO_POS_A)
		{
				state_op = S_HOLD1_A;
				set_traction_to_hold();
		}
		else
		{
				if(state_op == S_GOTO_POS2_A)
				{
					state_op = S_EVAL_POT;
				}
		}
}

// stop linear drive for B
void stop_brake()
{
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_8);
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_6);
		switch (state_op)
		{
			case S_GOTO_POS_B:
			state_op = S_BRAKE_B;
			activate_brake();
			break;
			case S_GOTO_DOWN_B:
			state_op = S_TURN_DOWN_A;
			set_turn_barrel (20, DOWN);
			break;

			default: ;
		}
}

// linear drive for A to go DOWN 
void move_traction_down()
{
    dir_traction = 'd';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_9);
}

// linear drive for A to go UP 
void move_traction_up()
{
    dir_traction = 'u';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_5);
}

// linear drive for B to go DOWN
void move_brake_down()
{
    dir_brake = 'd';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_6);
}

// linear drive for B to go UP 
void move_brake_up()
{
    dir_brake = 'u';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_8);
}


void move_brake_to_pos(void)
{
	if (state_op == S_GOTO_DOWN_B && work_position > 0)
		target_brake = get_pos_brake (work_position-1);
	else
		target_brake = get_pos_brake (work_position);
	if (target_brake > pos_brake)
		move_brake_up();
	else if (target_brake < pos_brake)
		move_brake_down();
	else
		stop_brake();
}

void move_traction_to_pos(void)
{
	if (state_op == S_GOTO_POS2_A && work_position > 0)
		target_traction = get_pos_traction (work_position-1);
	else
		target_traction = get_pos_traction (work_position);
	if (target_traction > pos_traction)
		move_traction_up();
	else if (target_traction < pos_traction)
		move_traction_down();
	else
		stop_traction();
}

/*******************************************************************/
// interrupt from punched disk
// NB after stopping, some more interrupts are anticipated
// (except for end position, where the motor is stopped by hardware)

void EXTI9_5_IRQHandler()
{
    if(LL_EXTI_IsActiveFlag_0_31 (LL_EXTI_LINE_6))   // PB6 = Lochscheibe A
    {
        if (dir_traction == 'u') pos_traction++;
        else if (dir_traction == 'd') pos_traction--;
        if (state_A != S_CAL && pos_traction == target_traction) stop_traction();
        LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_6);
    }
    if(LL_EXTI_IsActiveFlag_0_31 (LL_EXTI_LINE_7))   // PC7 = Lochscheibe B
    {
        if (dir_brake == 'u') pos_brake++;
        else if (dir_brake == 'd') pos_brake--;
        if (state_B != S_CAL && pos_brake == target_brake) stop_brake();
        LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_7);
    }
}

// ========================================================================


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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */
