/*
 * lcdUI.h
 *
 *  Created on: Sep 1, 2014
 *      Author: Oliver
 */

#ifndef LCDUI_H_
#define LCDUI_H_

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"

#define MAX_ERRORS 50
#define MAX_MESSAGES 50

typedef enum {
	lcdLiveDebug = 0,
	lcdMenuNavigation,
	lcdInLCDApp
} lcdMode;

typedef struct {
	char		msg[20];
	int16_t		timeStamp;
}lcdUIMessage;

void		lcdUIInit( int16_t display );
void		lcdUIAddError(char *new_error, ...);
void		lcdUIAddMessage(char *new_message, ...);
short		lcdUIShowLiveDebug(int16_t line, char *fmt, ...);
void		lcdUIEnableDebug( void );
void		lcdUIDisableDebug( void );


//Private
msg_t		MenuTask( void *arg );


#endif /* LCDUI_H_ */
