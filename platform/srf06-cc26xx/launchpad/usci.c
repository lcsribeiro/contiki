#include <stdio.h>
#include "usci.h"
#include "util.h"
#include "board-spi.h"
#include "ti-lib.h"

#include "ioc.h"
#include "gpio.h"

extern void IINCHIP_CSon();
extern void IINCHIP_CSoff();

#define PRINTF(...) printf(__VA_ARGS__)

/**
 * Initializes the SPI 
 **/
void Init_SPI(){


//	  IOCPortConfigureSet(BOARD_IOID_SPI_MOSI, IOC_PORT_MCU_SSI0_RX,IOC_STD_INPUT);
//	  IOCPortConfigureSet(BOARD_IOID_SPI_MISO, IOC_PORT_MCU_SSI0_TX,IOC_STD_OUTPUT);
//	  IOCPortConfigureSet(BOARD_IOID_SPI_SCLK, IOC_PORT_MCU_SSI0_CLK,IOC_STD_OUTPUT);
//	  IOCPortConfigureSet(BOARD_IOID_SPI_CS, IOC_PORT_MCU_SSI0_FSS,IOC_STD_OUTPUT);

	IOCPortConfigureSet(IOID_6, IOC_PORT_GPIO, IOC_STD_OUTPUT);
	GPIO_writeDio(IOID_6,1);

//	  board_spi_open((uint32_t)(ti_lib_sys_ctrl_clock_get()/12), BOARD_IOID_SPI_SCLK);
	board_spi_open((uint32)(ti_lib_sys_ctrl_clock_get()/12), WIZ_SCLK_PIN);
}

uint8 CC2650_SpiSendData(uint8 byte)
{   
	//board_spi_open(ti_lib_sys_ctrl_clock_get()/12, WIZ_SCLK_PIN);
	//board_spi_flush();
	//board_spi_write(byte, 1);
	//board_spi_read(byte, 1);
	//board_spi_close();
	//return byte;
	return board_spi_write(byte, 1);
}
