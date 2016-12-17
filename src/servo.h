/*
 * servo.h
 *
 *  Created on: 17. 12. 2016
 *      Author: Patrik Bakyta
 */

#ifndef SERVO_H_
#define SERVO_H_


void initSYSTEMCLOCK(void);

void initUSART(void);
extern "C" void USART1_IRQHandler(void);

void initPWM(void);


#endif /* SERVO_H_ */
