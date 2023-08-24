/*
 * buton.h
 *
 *  Created on: Aug 19, 2023
 *      Author: Admin
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
//#include "global.h"

#define NUM_OF_BUTTON 2
#define NORMAL_STATE SET
#define PRESSED_STATE RESET

#define USER_BUTT 0

int isButtonPressed(int index);

void subKeyProcess(int index);;
void getKeyInput();

#endif /* INC_BUTTON_H_ */
