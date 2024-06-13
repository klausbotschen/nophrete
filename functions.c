#include "stm32f3xx_ll_usart.h"
#include "functions.h"

/*******************************************************************/
// we always wait for a response before sending the next command
// therefore the while should never loop
// and we can use this in interrupts.



void send_cmd (uint8_t cmd)
{  
    LL_USART_TransmitData8 (USART3,cmd);
}


void check_if_traction_hold(void)
{
		send_cmd (0x82);
}
void check_traction_is_bottom(void)
{
		send_cmd (0x90);
}
void check_traction_is_top(void)
{
		send_cmd (0x91);
}
void check_brake_is_bottom(void)
{
		send_cmd (0x92);
}
void check_brake_is_top(void)
{
		send_cmd (0x93);
}

void check_if_brake_hold(void)
{
		send_cmd(0x83);
}

void set_traction_to_hold(void)
{
		send_cmd(0x7f);
}

void release_traction(void)
{
		send_cmd(0x00);
}

void set_turn_barrel(uint16_t steps, DIRECTION direction)
{
	if(direction == UP)
	{
		send_cmd(0x40+steps);
	}
	else
	{
		send_cmd(0x00+steps);
	}
	
}

void release_brake(void)
{
		send_cmd(0x80);
}
void activate_brake(void)
{
		send_cmd(0x81);
}
