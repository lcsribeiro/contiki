#include "Types.h"
#include "gpio.h"
#include "ioc.h"

//Rx e Tx invertidos 
#define RXD        IOID_3  // BIT1 
#define TXD        IOID_2  // BIT2
   
#define WIZ_SCLK_PIN			IOID_10  		//BIT5
#define WIZ_MISO_PIN			IOID_8   		//BIT6	 conector J2
#define WIZ_MOSI_PIN			IOID_9   		//BIT7  conector J2
//#define WIZ_SPI_OUT_PORT		DOUT31_0		//P1OUT
#define WIZ_SPI_IN_PORT			DIN31_0			//P1IN
//#define WIZ_SPI_SCLK_PORT		DOUT31_0		//P1OUT
#define WIZ_SCS_PIN			    IOID_11	 		//BIT0
//#define WIZ_SCS_PORT			DOUT31_0		//P2OUT

#define WIZ_PWDN_PIN			IOID_21 		//BIT0 // JP3
//#define WIZ_PWDN_PORT			DOUT31_01		//P2OUT
#define WIZ_RESET_PIN			IOID_24 		//BIT4 // JP5
//#define WIZ_RESET_PORT			DOUT31_0		//P1OUT

/*
 * End board specific
 */
#define WIZ_RESET_0				GPIO_writeDio(WIZ_RESET_PIN,0) //WIZ_RESET_PORT &= ~WIZ_RESET_PIN
#define WIZ_RESET_1				GPIO_writeDio(WIZ_RESET_PIN,1) //WIZ_RESET_PORT |= WIZ_RESET_PIN
#define WIZ_POWER_UP			GPIO_writeDio(WIZ_PWDN_PIN,0) //WIZ_PWDN_PORT &= ~WIZ_PWDN_PIN
#define WIZ_POWER_DOWN			GPIO_writeDio(WIZ_PWDN_PIN,1) //WIZ_PWDN_PORT |= WIZ_PWDN_PIN
#define WIZ_SELECT				GPIO_writeDio(WIZ_SCS_PIN,0) //WIZ_SCS_PORT &= ~WIZ_SCS_PIN
#define WIZ_DESELECT			GPIO_writeDio(WIZ_SCS_PIN,1) //WIZ_SCS_PORT |= WIZ_SCS_PIN
#define WIZ_IE_ENABLE			IOCPortConfigureSet(IOID_11, IOC_PORT_GPIO,IOC_INT_ENABLE); //IOCFG11 |= EDGE_IRQ_EN
#define WIZ_IE_DISABLE			IOCPortConfigureSet(IOID_11, IOC_PORT_GPIO,IOC_INT_DISABLE); //IOCFG11 &= ~EDGE_IRQ_EN

	//IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_RX,IOC_IOMODE_NORMAL);
	//IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_TX,IOC_IOMODE_NORMAL);
#define wizPowerUp()			WIZ_POWER_UP
#define wizPowerDown()			WIZ_POWER_DOWN
#define wizSelect()				WIZ_SELECT
#define wizDeselect()			WIZ_DESELECT
#define wizEnableInterrupt()	WIZ_IE_ENABLE
#define wizDisableInterrupt()	WIZ_IE_DISABLE
