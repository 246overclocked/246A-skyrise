

#include <stdlib.h>
#include <math.h>

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "chprintf.h"
#include "vex.h"



static const char menuOptions[] = {
		"Battery",
		"Live Debug",
		"Analog",
		"Digital",
		"Motors",
		"Messages",
		"Errors"
};

static lcdUIMessage errors[MAX_ERRORS];
static lcdUIMessage messages[MAX_ERRORS];

static int16_t startTime = chTimeNow()/CH_FREQUENCY;
static int16_t display = VEX_LCD_DISPLAY_1;

static int16_t activeLcdMenuOption = 0;
static int16_t activeLcdMenuLevel = 0;
static short liveDebugEnabled = TRUE;

int16_t
getSystemTime ( void ){
	return chTimeNow()/CH_FREQUENCY - startTime;
}

void
lcdUIInit( int16_t _display)
{
	display = _display;
	StartTask( MenuTask , NORMALPRIO + 2 );
}

void
lcdUIAddError(char *new_error, ...)
{
	for (int i=0; i < MAX_ERRORS; i++)
	{
		if ( errors[i] == NULL ){
			sprintf( errors[i].msg, new_error, args );
			errors[i].timeStamp = getSystemTime();
			return;
		}
	}
}

void
lcdUIAddMessage(char *new_message, ...)
{
	for (int i=0; i < MAX_MESSAGES; i++)
	{
		if ( messages[i] == NULL ){
			sprintf( messages[i].msg, new_message, args );
			messages[i].timeStamp = getSystemTime();
			return;
		}
	}
}

short
lcdUIShowLiveDebug(int16_t line, char *fmt, ...)
{
	if (!liveDebugEnabled)
		return FALSE;

	activeLcdMenuOption = 1;
	activeLcdMenuLevel = 1;
	vexLcdClearLine(display, 1);
	vexLcdPrintf(display, 2, "< Debug  >" );
	vexLcdPrintf(display, line, fmt, args );

	return TRUE;
}

void
lcdUIEnableDebug( void )
{
	liveDebugEnabled = TRUE;
}

void
lcdUIDisableDebug( void )
{
	liveDebugEnabled = FALSE;
}

msg_t
MenuTask( void *arg )
{
	(void)arg;
	vexTaskRegister("lcdMenu");
	int16_t buttons;
	while (!chThdShouldTerminate())
	{
		switch (activeLcdMenuOption)
		{
		case 0: //Battery
			vexLcdPrint(display, 1, "%1.2fV, %1.2fV", vexSpiGetMainBattery()/1000, vexAdcGet( kVexDigital_2 ) / 270.0);
			vexLcdPrint(display, 2, " Battery >");
			break;
		case 1: //Debug
			if (activeLcdMenuLevel==0)
			{
				vexLcdClearLine(display, 1);
				vexLcdPrint(display, 2, "< Debug  >");
			}
			break;
		case 2: //Analog

			break;
		case 3: //Digital
			break;
		case 4: //Motors
			break;
		case 5: //Messages
			break;
		case 6: //Errors
			break;
		}
		if(buttons!=vexLcdButtonGet(display))
		{
			buttons = vexLcdButtonGet(display);
			if ( buttons == kLcdButtonCenter )
			{
				activeLcdMenuLevel=1-activeLcdMenuLevel;
			}

		}
	}
	return(msg_t)0;
}


