/*
 * Copyright (c) 2010, Loughborough University - Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *         Example to demonstrate-test cc2530 sensor functionality
 *
 *         B1 turns LED_GREEN on and off.
 *
 *         The node takes readings from the various sensors every x seconds and
 *         prints out the results.
 *
 *         We use floats here to translate the AD conversion results to
 *         meaningful values. However, our printf does not have %f support so
 *         we use an ugly hack to print out the value by extracting the integral
 *         part and then the fractional part. Don't try this at home.
 *
 *         Temperature:
 *           Math is correct, the sensor needs calibration per device.
 *           I currently use default values for the math which may result in
 *           very incorrect values in degrees C.
 *           See TI Design Note DN102 about the offset calibration.
 *
 *         Supply Voltage (VDD):
 *           For VDD, math is correct, conversion is correct.
 *           See DN101 for details.
 *
 *         Make sure you enable/disable things in contiki-conf.h
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "config.h"
#include "w5500.h"
#include "usci.h"
#include "socket.h"
#include "util.h"
#include "ti-lib.h"

#include "contiki.h"
#include "contiki-conf.h"
#include "dev/leds.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "board.h"
#include "gpio.h"

#include "dev/button-sensor.h"


//void SetNetInfo(wiz_NetInfo *netinfo);
//extern void GetNetInfo(wiz_NetInfo *netinfo);

#define PRINTF(...) printf(__VA_ARGS__)

/*---------------------------------------------------------------------------*/
PROCESS(sensors_test_process, "Sensor Test Process");

AUTOSTART_PROCESSES(&sensors_test_process);


/*---------------------------------------------------------------------------*/

uip_ip6addr_t endereco;
static struct uip_udp_conn *com;

static uint32_t buffer[40];

//char * dados = (( char *) buffer);

/*
 * Network Information for ioShield-L
 */
uint8 mac[6] = {0xAB, 0x08, 0xDC, 0x11, 0x22, 0x92};    // MAC Address
uint8 ip[4] = {192, 168, 1, 119};                       // IP Address
uint8 gw[4] = {192, 168, 1, 1};                         // Gateway Address
uint8 sn[4] = {255, 255, 255, 0};                       // SubnetMask Address
uint8 dns[4] = {168, 126, 63, 1};                       // DNS Server Address

uint8 destip[4] = {192, 168, 1, 214};           // Destination IP Address for HTTP Client
uint16 destport = 80;                           // Destination Port number for HTTP Client

//TX MEM SIZE- SOCKET 0-7:2KB
//RX MEM SIZE- SOCKET 0-7:2KB
uint8 txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
uint8 rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};

uint8 TX_BUF[TX_RX_MAX_BUF_SIZE];
uint8 RX_BUF[TX_RX_MAX_BUF_SIZE];

uint8 getMAC[6];
uint8 getIP[4];
uint8 getGateway[4];
uint8 getSubnet[4];

/*
 * End of especification for ioShield-L
 */
void Init_W5500(void)
{
    wiz_NetInfo netinfo;
    device_init(txsize, rxsize); // Socket Tx, Rx memory size init function

//#ifndef __WIZ550IO__
    uint8 i;
    for(i=0; i<4; i++){
        netinfo.mac[i] = mac[i];
        //PRINTF("\n\r %X",netinfo.mac[i]);
        netinfo.ip[i] = ip[i];
        netinfo.sn[i] = sn[i];
        netinfo.gw[i] = gw[i];
        netinfo.dns[i] = dns[i];
    }
    netinfo.mac[i] = mac[i];
    i++;
    netinfo.mac[i] = mac[i];

    SetNetInfo(&netinfo);
    PRINTF("\r\nCC2650 LaunchPad - W5500 TestBoard\r\n");
//#else
//    PRINTF("\r\nCC2650 LaunchPad - WIZ550io BoosterPack\r\n");
//#endif

    GetNetInfo(&netinfo);

    PRINTF("\r\nMAC : %X:%X:%X:%X:%X:%X", netinfo.mac[0],netinfo.mac[1],netinfo.mac[2],netinfo.mac[3],netinfo.mac[4],netinfo.mac[5]);
	PRINTF("\r\nIP : %d.%d.%d.%d\n", netinfo.ip[0],netinfo.ip[1],netinfo.ip[2],netinfo.ip[3]);
	PRINTF("\r\nDNS : %d.%d.%d.%d\n", netinfo.dns[0],netinfo.dns[1],netinfo.dns[2],netinfo.dns[3]);
}

/**
@brief  This function gets PHY Status register in common register.
 */
uint8 getPHY(void)
{
    //return IINCHIP_READ(PHY);
	return IINCHIP_READ_COMMON(WIZC_PHYCFG);
}

void Check_link(void)
{
    uint8 link_status;
    static uint8 count = 0;
    // Link check
	do
	{
        Reset_W5500(); // W5500 Hardware Reset

		delay_msec(2000 + (100 * count));
		count++;

		link_status = (uint8)((getPHY() >> 5) & 0x01); // or PHYSTATUS & 0x20, PHY register defined (COMMON_BASE + 0x0035)
	} while(link_status != 0x01);
}

PROCESS_THREAD(sensors_test_process, ev, data)
{
  static struct etimer et;


  PROCESS_BEGIN();

  //ti_lib_int_master_enable();
  //leds_init();
  //leds_on(LEDS_ALL);
  board_init();
  Init_SPI();
  Init_W5500();
  delay_msec(50);


  PRINTF("========================\n");
  PRINTF("Iniciando Processo\n");
  PRINTF("========================\n");

  /* Set an etimer. We take sensor readings when it expires and reset it. */
  etimer_set(&et, CLOCK_SECOND * 1);

  //bbbb::4820:3d53:b336:e3ee
  uip_ip6addr(&endereco,0xbbbb,0,0,0,0x7194,0x785c,0x37ab,0xda6c);


  com = udp_new(&endereco,UIP_HTONS(8802),NULL);


  while(1)
  {
	GPIO_writeDio(IOID_6,1);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);


    PRINTF("Debug teste\n");
    buffer[0] = 0x79;
    //TCPSend();
    //uip_udp_packet_send(com,buffer,1);
    etimer_reset(&et);
  }
  PROCESS_END();
}

