/*
 * servo.cpp
 *
 *  Created on: 17. 12. 2016
 *      Author: Patrik Bakyta
 */

#include "stm32f4xx.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stdlib.h>
#include <misc.h>
#include <servo.h>

uint8_t value;

void initSYSTEMCLOCK(void) {

	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	RCC_SYSCLKConfig(RCC_CFGR_SW_HSI);
	SystemCoreClockUpdate();

	//uint32_t SystemClockValue = SystemCoreClock;

	return;

}

void initUSART(void) {

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	 GPIO_InitTypeDef GPIO_InitStruct;    // this is for the GPIO pins used as TX and RX
	 USART_InitTypeDef USART_InitStruct;  // this is for the USART1 initilization
	 NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	 /* enable APB2 peripheral clock for USART1
	  * note that only USART1 and USART6 are connected to APB2
	  * the other USARTs are connected to APB1
	  */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	 /* enable the peripheral clock for the pins used by
	  * USART1, PB6 for TX and PB7 for RX
	  */
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	 /* This sequence sets up the TX and RX pins
	  * so they work correctly with the USART1 peripheral
	  */
	 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			 // the pins are configured as alternate function so the USART peripheral has access to them
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 // this defines the IO speed and has nothing to do with the baudrate!
	 GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		 // this defines the output type as push pull mode (as opposed to open drain)
	 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			 // this activates the pullup resistors on the IO pins
	 GPIO_Init(GPIOB, &GPIO_InitStruct);				 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	 /* The RX and TX pins are now connected to their AF
	  * so that the USART1 can take over control of the
	  * pins
	  */
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	 /* Now the USART_InitStruct is used to define the
	  * properties of USART1
	  */
	 USART_InitStruct.USART_BaudRate = 9600;				 // the baudrate is set to the value we passed into this init function
	 USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	 USART_InitStruct.USART_StopBits = USART_StopBits_1;	 // we want 1 stop bit (standard)
	 USART_InitStruct.USART_Parity = USART_Parity_No;		 // we don't want a parity bit (standard)
	 USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	 USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	 USART_Init(USART1, &USART_InitStruct);					 // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	 /* Here the USART1 receive interrupt is enabled
	  * and the interrupt controller is configured
	  * to jump to the USART1_IRQHandler() function
	  * if the USART1 receive interrupt occurs
	  */
	 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // we want to configure the USART1 interrupts
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8; // this sets the priority group of the USART1 interrupts
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // this sets the subpriority inside the group
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // the USART1 interrupts are globally enabled
	 NVIC_Init(&NVIC_InitStructure);							  // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	 // finally this enables the complete USART1 peripheral
	 USART_Cmd(USART1, ENABLE);

	 return;
}

extern "C" void USART1_IRQHandler(void){
	// this is the interrupt request handler (IRQ) for ALL USART1 interrupts

	if (USART_GetITStatus(USART1, USART_IT_RXNE)==SET) {

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		value = USART_ReceiveData(USART1);
		// Send data back for verification
		USART_SendData(USART1,value);

		TIM4->CCR3 = value; // pre servo, PWM = 2000, value = cca 120-180

	}

	return;
}

void initPWM(void) {

	/*Structures used in the configuration*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable TIM4 Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//Enabled GPIOB we are going to use PB8 which is linked to TIM4_CH3 according to the
	//documentation (202-page datasheet, page 58 - TIM4_CH3)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Initalise pin 8 B - relating to timer 4 channel 3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

	/* Setup PWM */
	/* Setup timer defaults */
	TIM_TimeBaseStructure.TIM_Period = 2000-1; // 50Hz PWM / 20ms
	TIM_TimeBaseStructure.TIM_Prescaler = 80-1; // 100kHz vstupne hodiny
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* Configure timer for PWM - same settings for both channels */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//notice the number 3 in TIM_OC3Init
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* Start timer */
	TIM_Cmd(TIM4, ENABLE);

	return;
}
