#include "touch_process.h"
#include "XPT2046_touch.h"
#include "ili9341.h"
#include <stdio.h>

typedef struct{
	uint16_t x;				// last valid x coordinate
	uint16_t y;				// last valid y coordinate
	uint8_t was_touch;		// event
}t_touch_dev;

t_touch_dev td = {0,0,0};	// create object

void tp_doProcessMain(void)
{
	char str[100]; // buffer for string

	// if was touching -> make all we want -> it's not IRQ handler routine
	if (td.was_touch) 
	{
		// unblock event
		td.was_touch = 0;

		// draw touched pixel
		lcdDrawPixel(td.x, td.y, COLOR_RED);

		// draw touched coordinates
		lcdSetTextFont(&Font16);
		lcdSetTextColor(COLOR_MAGENTA, COLOR_WHITE);
		lcdSetCursor(50, 200);
		// create string in the desired format
		sprintf(str, "You pressed X = %u Y = %u", td.x, td.y);
		// print string to LCD
		lcdPrintf(str);
	}
}

// it's IRQ handler routine -> make our deal as fast as can
void tp_doProcessIRQ(void)
{
	uint16_t x, y;

	// if previous event is unhandled -> lost current
	if (td.was_touch) return;

	if (XPT2046_isTouchPressed()) // touch pressed?
	{
		if (XPT2046_doGetTouchGetCoordinates(&x, &y)) // get pressed point if valid touching was
		{
			// store valid data
			td.x = x;
			td.y = y;

			// event up
			td.was_touch = 1;
		}
	}
}


