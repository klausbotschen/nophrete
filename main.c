// Nofretete - building a pyramid

// + Stm32f3xx_ll_bus
// + stm32f3xx_ll_rcc
// + stm32f3xx_ll_system.h
// + stm32f3xx_ll_exti.h
// + stm32f3xx_ll_adc
// + stm32f3xx_ll_gpio
// + stm32f3xx_ll_tim
// + stm32f3xx_ll_usart

#include "main.h"

void GPIO_Config(void);
void USART3_Config(void);
void Timer_Config(void);
void ADC_Config(void);
void ADC_Start(void);
void EXTI_Config(void);
void NVIC_Config(void);
void SystemClock_Config(void);

void start_calibration(void);
void stop_traction(void);
void stop_break(void);
void traction_up(void);
void traction_down(void);
void break_up(void);
void break_down(void);

/*******************************************************************/

typedef enum
{
    S_INIT,
    S_CAL,
    S_OPERATION,
} E_STATE;

E_STATE state = S_INIT;

int32_t pos_traction = 0, pos_break = 0, target_traction = 0, target_break = 0;
int32_t end_traction = 0, end_break = 0;
uint8_t dir_break = 'n', dir_traction = 'n';    // up, down, neutral


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

    start_calibration();

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
// we always wait for a response before sending the next command
// therefore the while should never loop
// and we can use this in interrupts.

void sendCmd (uint8_t cmd)
{
    while (!LL_USART_IsActiveFlag_TXE(USART3)) {};
    LL_USART_TransmitData8 (USART3,cmd);
}

/*******************************************************************/
// called every 100 ms = 10 Hz
// read out potentionmeter, smoothen, and derive directive

void TIM6_DAC_IRQHandler()
{
    int adcValue=0;

    adcValue = LL_ADC_REG_ReadConversionData12(ADC2);

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
        case 0x04: // barrel action finished
        case 0x05: // barrel action ongoing
        break;

        case 0x24: // break action finished
        case 0x25: // break action ongoing
        break;

        case 0x08: // status stopper A bottom not reached
        case 0x09: // status stopper A bottom active
        case 0x0a: // status stopper A top not reached
        case 0x0b: // status stopper A top active
        case 0x28: // status stopper B bottom not reached
        case 0x29: // status stopper B bottom active
        case 0x2a: // status stopper B top not reached
        case 0x2b: // status stopper B top active
        break;

        case 0x0c: // stopper A bottom reached
        stop_traction();
        if (state == S_CAL) {
            // position might be negative if the lin drive was not at the bottom
            end_traction = end_traction - pos_traction;
            state = S_OPERATION;
        }
        break;
        case 0x0d: // stopper A top reached
        stop_traction();
        if (state == S_CAL) {
            end_traction = pos_traction;
            traction_down();
        }
        break;

        case 0x2c: // stopper B bottom reached
        stop_break();
        if (state == S_CAL) {
            // position might be negative if the lin drive was not at the bottom
            end_break = end_break - pos_break;
            state = S_OPERATION;
        }
        break;
        case 0x2d: // stopper B top reached
        stop_break();
        if (state == S_CAL) {
            end_break = pos_break;
            break_down();
        }
        break;

        default:    // everything else should not come => safety reaction
        stop_traction();
        stop_break();
        // reset nofretete and restart statemachine
    }
}

/*******************************************************************/

// start the linear drive for A and B
void start_calibration()
{
    state = S_CAL;
    traction_up();
    break_up();
}

// stop linear drive for A
void stop_traction()
{
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_5);
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_9);
}

// stop linear drive for B
void stop_break()
{
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_8);
    LL_GPIO_ResetOutputPin (GPIOA,LL_GPIO_PIN_6);
}

// linear drive for A to go DOWN 
void traction_down()
{
    dir_traction = 'd';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_9);
}

// linear drive for A to go UP 
void traction_up()
{
    dir_traction = 'u';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_5);
}

// linear drive for B to go DOWN
void break_down()
{
    dir_break = 'd';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_6);
}

// linear drive for B to go UP 
void break_up()
{
    dir_break = 'u';
    LL_GPIO_SetOutputPin (GPIOA,LL_GPIO_PIN_8);
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
        if (pos_traction == target_traction) stop_traction();
        LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_6);
    }
    if(LL_EXTI_IsActiveFlag_0_31 (LL_EXTI_LINE_7))   // PC7 = Lochscheibe B
    {
        if (dir_break == 'u') pos_break++;
        else if (dir_break == 'd') pos_break--;
        if (pos_break == target_break) stop_break();
        LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_7);
    }
}

// ========================================================================
