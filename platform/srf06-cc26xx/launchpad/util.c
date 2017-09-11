//#include <stdio.h>
//#include <stdarg.h>
#include "config.h"
#include "util.h"
#include "w5500.h"

#include "usci.h" // for CC2650 Pin configuration 
#include "clock.h" // for delay functions
//#include "ti-lib.h" // for delay functions

//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */

/*
 * W5500 hardware reset
 */
void Reset_W5500(void) {
	WIZ_RESET_0;
	delay_msec(20);	//Delay in microseconds
	WIZ_RESET_1;
	delay_msec(200);	//Delay in miliseconds
	wizPowerUp();
	delay_msec(200);	//Delay in miliseconds
}

void delay_msec(uint16_t  time_ms){
	uint16 c = 0;
	while (c++ < time_ms) {
		clock_delay_usec(1000); 
	}
}
