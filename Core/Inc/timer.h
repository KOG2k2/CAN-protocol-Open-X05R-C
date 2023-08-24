/*
 * timer.h
 *
 *  Created on: Aug 21, 2023
 *      Author: Admin
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

extern int timer1_flag;
extern int timer1_counter;

extern int timer2_flag;
extern int timer2_counter;

extern int timer3_flag;
extern int timer3_counter;

extern int timer4_flag;
extern int timer4_counter;

extern int timer5_flag;
extern int timer5_counter;

extern int timer6_flag;
extern int timer6_counter;

extern int timer7_flag;
extern int timer7_counter;

extern int timer8_flag;
extern int timer8_counter;

extern int timer9_flag;
extern int timer9_counter;

extern int timer10_flag;
extern int timer10_counter;

void setTimer1(int duration);
void setTimer2(int duration);
void setTimer3(int duration);
void setTimer4(int duration);
void setTimer5(int duration);
void setTimer6(int duration);
void setTimer7(int duration);
void setTimer8(int duration);
void setTimer9(int duration);
void setTimer10(int duration);
void timerRun();

#endif /* INC_TIMER_H_ */
