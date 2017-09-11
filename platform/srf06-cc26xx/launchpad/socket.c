/**
 * @file        w5500/socket.c
 * @brief       TCP/IP Chip Device Driver Source File - For w5500
 * @version 1.0
 * @date        2013/02/27
 * @par Revision
 *      2013/02/27 - 1.0 Release
 * @author
 * \n\n @par Copyright (C) 2013 WIZnet. All rights reserved.
 */

#include "config.h"
#include <stdio.h>
#include <string.h>
#include "w5500.h"
#include "socket.h"
#include "util.h"
//lucas
//#define PRINTF(...) printf(__VA_ARGS__)

extern uint8 I_STATUS[TOTAL_SOCK_NUM];
//static uint16 RMASK[MAX_SOCK_NUM]; /**< Variable for Rx buffer MASK in each channel */
extern uint16 SSIZE[TOTAL_SOCK_NUM]; /**< Max Tx buffer size by each channel */
extern uint16 RSIZE[TOTAL_SOCK_NUM]; /**< Max Rx buffer size by each channel */
//static uint16 SBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Tx buffer base address by each channel */
//static uint16 RBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Rx buffer base address by each channel */

static uint8 DNS[4]={0};
static dhcp_mode DHCP = NETINFO_STATIC;
static uint16 local_port = 0xC000;  // Dynamic Port: C000(49152) ~ FFFF(65535)

//static uint32 tcp_close_elapse[TOTAL_SOCK_NUM] = {0,};
//static uint32 tcp_resend_elapse[TOTAL_SOCK_NUM] = {0,};
static uint16 txrd_checker[TOTAL_SOCK_NUM];


/**
 * Initialize the w5500 device
 * @param tx_size Tx socket buffer size array
 * @param rx_size Rx socket buffer size array
 */
void device_init(uint8 *tx_size, uint8 *rx_size)
{
    //device_SW_reset();
    device_mem_init(tx_size, rx_size);
}


/**
@brief  This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/

void device_SW_reset(void)
{
    setMR(MR_RST);
    DBGA("MR value is %02x",IINCHIP_READ_COMMON(WIZC_MR));
}


/**
@brief  This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
Maximum memory size for Tx, Rx in the W5500 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and receiption from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to centain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/

void device_mem_init(uint8 * tx_size, uint8 * rx_size)
{
  int16 i;
  int16 ssum, rsum; //<-
  DBG("device_mem_init()");
  ssum = 0;
  rsum = 0;

    for (i = 0 ; i < TOTAL_SOCK_NUM; i++)       // Set the size, masking and base address of Tx & Rx memory by each channel
    {
        IINCHIP_WRITE_SOCKETREG( i, WIZS_TXMEM_SIZE, tx_size[i]);
        IINCHIP_WRITE_SOCKETREG( i, WIZS_RXMEM_SIZE, rx_size[i]);

        DBGA("tx_size[%d]: %d, Sn_TXMEM_SIZE = %d",i, tx_size[i], IINCHIP_READ_SOCKETREG( i, WIZS_TXMEM_SIZE));
        DBGA("rx_size[%d]: %d, Sn_RXMEM_SIZE = %d",i, rx_size[i], IINCHIP_READ_SOCKETREG( i, WIZS_RXMEM_SIZE));
        SSIZE[i] = (int16)(0);
        RSIZE[i] = (int16)(0);

        if (ssum <= 16384)
        //if (ssum <= 8192)
        {
            switch( tx_size[i] )
            {
            case 1:
                SSIZE[i] = (int16)(1024);
                break;
            case 2:
                SSIZE[i] = (int16)(2048);
                break;
            case 4:
                SSIZE[i] = (int16)(4096);
                break;
            case 8:
                SSIZE[i] = (int16)(8192);
                break;
#if 1 //--4Channel���� ���� �ȵ� --20120522
            case 16:
                SSIZE[i] = (int16)(16384);
                break;
#endif//--4Channel���� ���� �ȵ�
            default: // by Ssoo����Ʈ �� 2K --20120522
                RSIZE[i] = (int16)(2048);
                break;
            }
        }

        if (rsum <= 16384)
        //if (rsum <= 8192)
        {
            switch( rx_size[i] )
            {
            case 1:
                RSIZE[i] = (int16)(1024);
                break;
            case 2:
                RSIZE[i] = (int16)(2048);
                break;
            case 4:
                RSIZE[i] = (int16)(4096);
                break;
            case 8:
                RSIZE[i] = (int16)(8192);
                break;
#if 1 //--4Channel���� ���� �ȵ�  --20120522
            case 16:
                RSIZE[i] = (int16)(16384);
                break;
#endif//--4Channel���� ���� �ȵ�
            default: // by Ssoo����Ʈ �� 2K --20120522
                RSIZE[i] = (int16)(2048);
                break;
            }
        }
        ssum += SSIZE[i];
        rsum += RSIZE[i];

//        if (i != 0)             // Sets base address of Tx and Rx memory for channel #1,#2,#3
//    {
//      SBUFBASEADDRESS[i] = SBUFBASEADDRESS[i-1] + SSIZE[i-1];
//      RBUFBASEADDRESS[i] = RBUFBASEADDRESS[i-1] + RSIZE[i-1];
//    }

    DBGA("ch = %d",i);
    DBGA("SBUFBASEADDRESS = %d",(uint16)SBUFBASEADDRESS[i]);
    DBGA("RBUFBASEADDRESS = %d",(uint16)RBUFBASEADDRESS[i]);
    DBGA("SSIZE = %d",SSIZE[i]);
    DBGA("RSIZE = %d",RSIZE[i]);
    }
}

/**
@brief  This function set the network information.
*/
//lucas
void SetNetInfo(wiz_NetInfo *ni)
{
    if(ni->mac[0] != 0x00 || ni->mac[1] != 0x00 || ni->mac[2] != 0x00 ||
        ni->mac[3] != 0x00 || ni->mac[4] != 0x00 || ni->mac[5] != 0x00)
        setSHAR(ni->mac);   // set local MAC address
    if(ni->ip[0] != 0x00 || ni->ip[1] != 0x00 || ni->ip[2] != 0x00 || ni->ip[3] != 0x00)
       setSIPR(ni->ip);    // set local IP address
    if(ni->sn[0] != 0x00 || ni->sn[1] != 0x00 || ni->sn[2] != 0x00 || ni->sn[3] != 0x00)
        setSUBR(ni->sn);    // set Subnet mask
    if(ni->gw[0] != 0x00 || ni->gw[1] != 0x00 || ni->gw[2] != 0x00 || ni->gw[3] != 0x00)
        setGAR(ni->gw);     // set Gateway address
    if(ni->dns[0] != 0x00 || ni->dns[1] != 0x00 || ni->dns[2] != 0x00 || ni->dns[3] != 0x00){
        DNS[0] = ni->dns[0];
        DNS[1] = ni->dns[1];
        DNS[2] = ni->dns[2];
        DNS[3] = ni->dns[3];
    }

    if(ni->dhcp != 0) DHCP = ni->dhcp;
}

/**
 * Clear specific device address to all zero.
 * @param member the member variable of @ref netinfo_member which means each of @ref wiz_NetInfo struct member.
 */
void ClsNetInfo(netinfo_member member)
{
    uint8 zero[6] = {0,};

    switch(member) {
    //case NI_MAC_ADDR: // If need, uncomment
    //  setSHAR(zero);
    //  break;
    case NI_IP_ADDR:
        setSIPR(zero);
        break;
    case NI_SN_MASK:
        setSUBR(zero);
        break;
    case NI_GW_ADDR:
        setGAR(zero);
        break;
    case NI_DNS_ADDR:
        DNS[0] = DNS[1] = DNS[2] = DNS[3] = 0;
        break;
    default:
        ERRA("wrong member value (%d)", member);
    }
}

/**
@brief  This function get the network information.
*/
void GetNetInfo(wiz_NetInfo *netinfo)
{
    getSHAR(netinfo->mac); // get local MAC address
    getSIPR(netinfo->ip); // get local IP address
    getSUBR(netinfo->sn); // get subnet mask address
    getGAR(netinfo->gw); // get gateway address
    netinfo->dns[0] = DNS[0];
    netinfo->dns[1] = DNS[1];
    netinfo->dns[2] = DNS[2];
    netinfo->dns[3] = DNS[3];
    netinfo->dhcp = DHCP;
}

/**
@brief  This function get the Destination network information for UDP
@return     None.
*/
void GetDstInfo(uint8 s, uint8 *dstip, uint16 *dstport)
{
    getDIPR(s, dstip);
    getDPORT(s, dstport);
}

/**
@brief  This function set the network option.
@return     None.
*/
void SetSocketOption(uint8 option_type, uint16 option_value)
{
    switch(option_type){
    case 0:
        setRTR(option_value); // set retry duration for data transmission, connection, closing ...
        break;
    case 1:
        setRCR((uint8)(option_value&0x00FF)); // set retry count (above the value, assert timeout interrupt)
        break;
    case 2:
        setIMR((uint8)(option_value&0x00FF)); // set interrupt mask.
        break;
    default:
        break;
    }
}

/**
@brief  This function get the TCP socket status.
@return     TCP socket status.
*/
int8 GetTCPSocketStatus(uint8 s)
{
    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    }

    switch(getSn_SR(s)){
    case SOCK_CLOSED: return SOCKSTAT_CLOSED;             // closed
    case SOCK_INIT: return SOCKSTAT_INIT;                 // init state
    case SOCK_LISTEN: return SOCKSTAT_LISTEN;             // listen state
    case SOCK_SYNSENT: return SOCKSTAT_SYNSENT;           // connection state
    case SOCK_SYNRECV: return SOCKSTAT_SYNRECV;           // connection state
    case SOCK_ESTABLISHED: return SOCKSTAT_ESTABLISHED;   // success to connect
    case SOCK_FIN_WAIT: return SOCKSTAT_FIN_WAIT;         // closing state
    case SOCK_CLOSING: return SOCKSTAT_CLOSING;           // closing state
    case SOCK_TIME_WAIT: return SOCKSTAT_TIME_WAIT;       // closing state
    case SOCK_CLOSE_WAIT: return SOCKSTAT_CLOSE_WAIT;     // closing state
    case SOCK_LAST_ACK: return SOCKSTAT_LAST_ACK;         // closing state
    default:
        //if((IINCHIP_READ(Sn_MR(Sn_MR_TCP))&0x0F) != Sn_MR_TCP)
                        //return SOCKERR_NOT_UDP;
        if((IINCHIP_READ_SOCKETREG(s, WIZC_MR)&0x0F) != Sn_MR_TCP)
            return SOCKERR_NOT_TCP;
        else return SOCKERR_WRONG_STATUS;
    }
}

/**
@brief  This function get the UDP socket status.
@return     UDP socket status.
*/
int8 GetUDPSocketStatus(uint8 s)
{
    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_UDP;
    }

    switch(getSn_SR(s)){
    case SOCK_CLOSED: return SOCKSTAT_CLOSED; //  closed
    case SOCK_UDP: return SOCKSTAT_UDP;       //  udp socket
#if 0
    case SOCK_MACRAW: return 12;    //  mac raw mode socket
    case SOCK_PPPOE: return 13;     //  pppoe socket
#endif
    default:
        //if((IINCHIP_READ(Sn_MR(Sn_MR_UDP))&0x0F) != Sn_MR_UDP)
            //return SOCKERR_NOT_UDP;
        if((IINCHIP_READ_SOCKETREG(s, WIZC_MR)&0x0F) != Sn_MR_UDP)
            return SOCKERR_NOT_UDP;
        else return SOCKERR_WRONG_STATUS;
    }
}

/**
@brief  This Socket function get the TX free buffer size.
@return     size of TX free buffer size.
*/
uint16 GetSocketTxFreeBufferSize(uint8 s)
{
    return getSn_TX_FSR(s); // get socket TX free buf size
}

/**
@brief  This Socket function get the RX recv buffer size.
@return     size of RX recv buffer size.
*/
uint16 GetSocketRxRecvBufferSize(uint8 s)
{
    return getSn_RX_RSR(s); // get socket RX recv buf size
}














/* W5500 Socket Code for Test */
/************************************************************************************************************************************/
/**
@brief	This Socket function open TCP server socket.
@return 	1 - success, 0 - fail.
*/  
int8 TCPServerOpen(SOCKET s, uint16 port)
{	
	TCPClose(s);

	IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, Sn_MR_TCP);
	IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((port & 0xff00) >> 8));
	IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(port & 0x00ff));
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR)){LOGF(".");}        // wait to process the command...
	DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));
	    //PRINTF("\r\nOPEN -- Sn_SR = %x(<-%X) , Protocol = %x(<-%X:TCP) \r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_INIT ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_TCP);
    
	//if (IINCHIP_READ(Sn_SR(s)) != SOCK_INIT) {
	    //DBGA("wrong status(%d)", IINCHIP_READ(Sn_SR(s)));
	if(IINCHIP_READ_SOCKETREG(s, WIZS_SR) != SOCK_INIT) {
	    DBGA("wrong status(%d)", IINCHIP_READ_SOCKETREG(s, WIZS_SR));
	    return SOCKERR_WRONG_STATUS;
	} else {
	    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_LISTEN);
	    //while(IINCHIP_READ(Sn_CR(s)));              // wait to process the command...
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_LISTEN);
	    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
	            //PRINTF("\r\nLISTEN -- Sn_SR = %x(<-%x) , Protocol = %x(<-%X:TCP)\r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_LISTEN ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_TCP);
	}
	
	return RET_OK;
}


/**
@brief	This Socket function open TCP client socket.
@return 	1 - success, 0 - fail.
*/  
int8 TCPClientOpen(SOCKET s, uint16 port, uint8 * destip, uint16 destport)
{    
    uint8 srcip[4], snmask[4];

	if (port == 0) {   // if don't set the source port, set local_port number.
	  if(local_port == 0xffff) local_port = 0xc000;
	  else local_port++;
	  port = local_port;
	}

	TCPClose(s);
			
	IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, Sn_MR_TCP);
	IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((port & 0xff00) >> 8));
	IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(port & 0x00ff));
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
	DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));

	getSIPR(srcip);
	getSUBR(snmask);
	
	if( ((destip[0] == 0xFF) && (destip[1] == 0xFF) &&
	     (destip[2] == 0xFF) && (destip[3] == 0xFF)) ||
	    ((destip[0] == 0x00) && (destip[1] == 0x00) &&
	     (destip[2] == 0x00) && (destip[3] == 0x00)) || (port == 0x00) )
	{
	    DBG("invalid ip or port");
	    DBGA("SOCK(%d)-[%02x.%02x.%02x.%02x, %d]",s,
	        destip[0], destip[1], destip[2], destip[3] , port);
	    return SOCKERR_WRONG_ARG;
	}
	else if( (srcip[0]==0 && srcip[1]==0 && srcip[2]==0 && srcip[3]==0) &&
	    (snmask[0]!=0 || snmask[1]!=0 || snmask[2]!=0 || snmask[3]!=0) ) //Mikej : ARP Errata
	{
	    DBG("Source IP is NULL while SN Mask is Not NULL");
	    return SOCKERR_NULL_SRC_IP;
	}
	else
	{
	    //IINCHIP_WRITE(Sn_DIPR0(s),dip[0]);    // set destination IP
	    //IINCHIP_WRITE((Sn_DIPR0(s) + 1),dip[1]);
	    //IINCHIP_WRITE((Sn_DIPR0(s) + 2),dip[2]);
	    //IINCHIP_WRITE((Sn_DIPR0(s) + 3),dip[3]);
	    //IINCHIP_WRITE(Sn_DPORT0(s),(uint8)((dport & 0xff00) >> 8));
	    //IINCHIP_WRITE((Sn_DPORT0(s) + 1),(uint8)(dport & 0x00ff));
	    ////SetSubnet(sn);  // for ARP Errata
	    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_CONNECT);
	    //while (IINCHIP_READ(Sn_CR(s)) );  // wait for completion
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 0, destip[0]); // set destination IP
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 1, destip[1]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 2, destip[2]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 3, destip[3]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 0, (uint8)((destport & 0xff00) >> 8));
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 1, (uint8)(destport & 0x00ff));
	    //SetSubnet(sn);    // for ARP Errata
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR,Sn_CR_CONNECT);
	    while (IINCHIP_READ_SOCKETREG(s, WIZS_CR)); // wait for completion
	}
	
	return RET_OK;
}


/**
@brief	This Socket function open UDP socket.
@return 	1 - success, 0 - fail.
*/  
int8 UDPOpen(SOCKET s, uint16 port)
{
	if(s > TOTAL_SOCK_NUM) {
	    ERRA("wrong socket number(%d)", s);
	    return SOCKERR_NOT_UDP;
	} else DBG("start");
	
	if (port == 0) {    // if don't set the source port, set local_port number.
	    if(local_port == 0xffff) local_port = 0xc000;
	    else local_port++;
	    port = local_port;
	}
	
	UDPClose(s);
	
	//IINCHIP_WRITE(Sn_MR(s),Sn_MR_UDP);
	//IINCHIP_WRITE(Sn_PORT0(s),(uint8)((port & 0xff00) >> 8));
	//IINCHIP_WRITE((Sn_PORT0(s) + 1),(uint8)(port & 0x00ff));
	//IINCHIP_WRITE(Sn_CR(s),Sn_CR_OPEN); // run sockinit Sn_CR
	//while(IINCHIP_READ(Sn_CR(s)));    // wait to process the command...
	//DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ(Sn_SR(s)), IINCHIP_READ(Sn_MR(s)));
	
	IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, Sn_MR_UDP);
	IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((port & 0xff00) >> 8));
	IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(port & 0x00ff));
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
	DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));
	    //PRINTF("\r\nOPEN -- Sn_SR = %.2x(<-%.2X) , Protocol = %.2x(<-%.2X:UDP) \r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_UDP ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_UDP);
	return RET_OK;
}


/**
@brief	This function close the TCP socket and parameter is "s" which represent the socket number
@return 	1 - success, 0 - fail.
*/ 
int8 TCPClose(SOCKET s)
{
	uint8 status=0;
	uint8 cnt=0;

#ifdef __DEF_IINCHIP_DBG__
	PRINTF("TCPClose()\r\n");
#endif

	//IINCHIP_WRITE(Sn_CR(s),Sn_CR_DISCON);
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_DISCON);
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...

	status = getSn_SR(s);
	if(status == SOCK_ESTABLISHED)
		return RET_FAIL;

	// FIN wait
	while(status != SOCK_CLOSED)
	{
		//Delay_ms(100);
		clock_delay_msec(100);
		cnt++;
		if(cnt > 2) break;
	}

	//IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_CLOSE);
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...

	IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, 0xFF);  // interrupt all clear

	return RET_OK;
}


/**
@brief	This function close the UDP socket and parameter is "s" which represent the socket number
@return 	1 - success, 0 - fail.
*/ 
int8 UDPClose(SOCKET s)
{
	if(s > TOTAL_SOCK_NUM) {
	    ERRA("wrong socket number(%d)", s);
	    return SOCKERR_NOT_UDP;
	} else DBG("start");
	
	//IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);
	//while(IINCHIP_READ(Sn_CR(s)));                // wait to process the command...
	//IINCHIP_WRITE(Sn_IR(s), 0xFF);            // interrupt all clear
	
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_CLOSE);
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));      // wait to process the command...
	IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, 0xFF);  // interrupt all clear
	
	return RET_OK;
}


/**
@brief	This function used to send the data in TCP mode
@return	1 for success else 0.
*/ 
int16 TCPSend(SOCKET s, const uint8 * src, uint16 len)
{
	uint8 status=0;
	uint16 ret=0;
	uint16 freesize=0;
	uint16 txrd;

#ifdef __DEF_IINCHIP_DBG__
	PRINTF("send()\r\n");
#endif

	status = getSn_SR(s);
	if(status == SOCK_CLOSED)			return SOCKERR_CLOSED;
	if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_TCP)	return SOCKERR_NOT_TCP;
	
	if(status == SOCK_FIN_WAIT)			return SOCKERR_FIN_WAIT;
	if(status != SOCK_ESTABLISHED && status != SOCK_CLOSE_WAIT)	return SOCKERR_NOT_ESTABLISHED;

	init_windowfull_retry_cnt(s);

	if (len > getIINCHIP_TxMAX(s)) ret = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.
	else ret = len;
	
	// if freebuf is available, start.
	do 
	{
		freesize = GetSocketTxFreeBufferSize(s);
#ifdef __DEF_IINCHIP_DBG__
		PRINTF("socket %d freesize(%d) empty or error\r\n", s, freesize);
#endif
	} while (freesize < ret);

	// copy data
	send_data_processing(s, (uint8 *)src, ret);
        
	//txrd_before_send = IINCHIP_READ(Sn_TX_RD0(s));
	//txrd_before_send = (txrd_before_send << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);
	txrd_checker[s] = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
	txrd_checker[s] = (txrd_checker[s] << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);

	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR,Sn_CR_SEND);
	while (IINCHIP_READ_SOCKETREG(s, WIZS_CR)); // wait to process the command...
	
	if(!(IINCHIP_READ_SOCKETREG(s, WIZS_IR) & Sn_IR_SEND_OK)) {
	    if(IINCHIP_READ_SOCKETREG(s, WIZS_SR) == SOCK_CLOSED) {
	        DBG("SOCK_CLOSED");
	        TCPClose(s);
	        return SOCKERR_CLOSED;
	    }
	    return SOCKERR_BUSY;
	//} else IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
	} else IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, Sn_IR_SEND_OK);
	
	//txrd = IINCHIP_READ(Sn_TX_RD0(s));
	//txrd = (txrd << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);
	
	txrd = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
	txrd = (txrd << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);
	
	if(txrd > txrd_checker[s]) return txrd - txrd_checker[s];
	else return (0xffff - txrd_checker[s]) + txrd + 1;
	
	//return RET_OK;
}


/**
@brief	This function used to send the data in TCP mode
@return	1 for success else 0.
*/ 
int16 TCPReSend(SOCKET s)
{
	uint8 status=0;	
	uint16 txrd;

	if(s > TOTAL_SOCK_NUM) {
	    ERRA("wrong socket number(%d)", s);
	    return SOCKERR_NOT_TCP;
	} else DBG("start");
	
	status = getSn_SR(s);
	if(status == SOCK_CLOSED)			return SOCKERR_CLOSED;
	if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_TCP)	return SOCKERR_NOT_TCP;
	if(status == SOCK_FIN_WAIT)			return SOCKERR_FIN_WAIT;
	if(status != SOCK_ESTABLISHED && status != SOCK_CLOSE_WAIT)	return SOCKERR_NOT_ESTABLISHED;

	if(incr_windowfull_retry_cnt(s) > WINDOWFULL_MAX_RETRY_NUM)
		return SOCKERR_WINDOW_FULL;

	//txrd_before_send = IINCHIP_READ(Sn_TX_RD0(s));
	//txrd_before_send = (txrd_before_send << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);
  
	//IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
	//while( IINCHIP_READ(Sn_CR(s)) ) ;
	
	txrd_checker[s] = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
	txrd_checker[s] = (txrd_checker[s] << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);
	
	IINCHIP_WRITE_SOCKETREG(s, WIZS_CR,Sn_CR_SEND);
	while (IINCHIP_READ_SOCKETREG(s, WIZS_CR)); // wait to process the command...
	
	if(!(IINCHIP_READ_SOCKETREG(s, WIZS_IR) & Sn_IR_SEND_OK)) {
	    if(IINCHIP_READ_SOCKETREG(s, WIZS_SR) == SOCK_CLOSED) {
	        DBG("SOCK_CLOSED");
	        TCPClose(s);
	        return SOCKERR_CLOSED;
	    }
	    return SOCKERR_BUSY;
	//} else IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
	} else IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, Sn_IR_SEND_OK);
	
	//txrd = IINCHIP_READ(Sn_TX_RD0(s));
	//txrd = (txrd << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);
	
	txrd = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
	txrd = (txrd << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);
	
	if(txrd > txrd_checker[s]) return txrd - txrd_checker[s];
	else return (0xffff - txrd_checker[s]) + txrd + 1;
	
	
}


/**
@brief	This function is an application I/F function which is used to receive the data in TCP mode.
		It continues to wait for data as much as the application wants to receive.
		
@return	received data size for success else -1.
*/ 
int16 TCPRecv(SOCKET s, uint8 * buf, uint16 len)
{
	uint8 status=0;
	uint16 RSR_len=0;

	if(s > TOTAL_SOCK_NUM) {
	    ERRA("wrong socket number(%d)", s);
	    return SOCKERR_NOT_TCP;
	} else if(len == 0) {
	    ERR("Zero length");
	    //return SOCKERR_WRONG_ARG; //W5500 �׽�Ʈ �Ѵٰ� �ּ� ó�� �� 
	}
    
	RSR_len = GetSocketRxRecvBufferSize(s);
    
	if(RSR_len == 0){
	    status = getSn_SR(s);
	    if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
	    //if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;
	    if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;
	
	    if(status == SOCK_CLOSE_WAIT) return SOCKERR_CLOSE_WAIT;
	    if(status != SOCK_ESTABLISHED && status != SOCK_CLOSE_WAIT) return SOCKERR_NOT_ESTABLISHED;
	} else {
	    if(len < RSR_len) RSR_len = len;
	    recv_data_processing(s, (uint8*)buf, RSR_len);
	    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
	    //while(IINCHIP_READ(Sn_CR(s)));                // wait to process the command...
	
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
	    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));      // wait to process the command...
	}
	
	return RSR_len;
}


/**
@brief	This function is an application I/F function which is used to send the data for other then TCP mode. 
		Unlike TCP transmission, The peer's destination address and the port is needed.
		
@return	This function return send data size for success else -1.
*/ 
int16 UDPSend(SOCKET s, const uint8 * buf, uint16 len, uint8 * addr, uint16 port)
{
	uint8 srcip[4], snmask[4], status = 0;
	
	if(s > TOTAL_SOCK_NUM) 
	{
	    ERRA("wrong socket number(%d)", s);
	    return SOCKERR_NOT_UDP;
	}
	else if(len == 0)
	{
	    ERR("Zero length");
	    return SOCKERR_WRONG_ARG; //W5500 UDPSend TEST���� �ּ�ó����
	} 
	else if(addr == NULL) 
	{
	    ERR("NULL Dst IP");
	    return SOCKERR_WRONG_ARG;
	} else DBG("start");
	
	status = getSn_SR(s);
	if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
	//if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
	if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
	if(status != SOCK_UDP) return SOCKERR_NOT_UDP;
	
	if (len > getIINCHIP_TxMAX(s)) len = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.
	
	getSIPR(srcip);
	getSUBR(snmask);
	
	if((addr[0]==0x00 && addr[1]==0x00 && addr[2]==0x00 &&
	    addr[3]==0x00) || (port==0x00))
	{
	    DBG("invalid ip or port");
	    DBGA("SOCK(%d)-[%02x.%02x.%02x.%02x, %d, %d]",s,
	        addr[0], addr[1], addr[2], addr[3] , port, len);
	    return SOCKERR_WRONG_ARG;
	}
	#if 0 //-- W5200�� Erratum2 => W5500���� ����! W5500������ �ʿ���� �κ�
	else if( (srcip[0]==0 && srcip[1]==0 && srcip[2]==0 && srcip[3]==0) &&
	    (snmask[0]!=0 || snmask[1]!=0 || snmask[2]!=0 || snmask[3]!=0) ) //Mikej : ARP Errata
	{
	    DBG("Source IP is NULL while SN Mask is Not NULL");
	    return SOCKERR_NULL_SRC_IP;
	}
	#endif
	else
	{
	    //IINCHIP_WRITE(Sn_DIPR0(s),addr[0]);
	    //IINCHIP_WRITE((Sn_DIPR0(s) + 1),addr[1]);
	    //IINCHIP_WRITE((Sn_DIPR0(s) + 2),addr[2]);
	    //IINCHIP_WRITE((Sn_DIPR0(s) + 3),addr[3]);
	    //IINCHIP_WRITE(Sn_DPORT0(s),(uint8)((port & 0xff00) >> 8));
	    //IINCHIP_WRITE((Sn_DPORT0(s) + 1),(uint8)(port & 0x00ff));
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 0, addr[0]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 1, addr[1]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 2, addr[2]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 3, addr[3]);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 0,(uint8)((port & 0xff00) >> 8));
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 1,(uint8)(port & 0x00ff));
	
	    send_data_processing(s, (uint8*)buf, len);  // copy data
	    //SetSubnet(sn);    // for ARP Errata
	
	    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
	    //while(IINCHIP_READ(Sn_CR(s)));  // wait to process the command...
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_SEND);
	    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));  // wait to process the command...
	}
	
	    //uint8 ir = IINCHIP_READ(Sn_IR(s));
	uint8 ir = IINCHIP_READ_SOCKETREG(s, WIZS_IR);
	
	//DBGA("WATCH UDP Send CHK - sock(%d)", s);
	if(!(ir & Sn_IR_SEND_OK)) {
	    if(ir & Sn_IR_TIMEOUT) {
	        DBG("send fail");
	        //IINCHIP_WRITE(Sn_IR(s), (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); // clear SEND_OK & TIMEOUT
	        IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); // clear SEND_OK & TIMEOUT Interrupt
	        return SOCKERR_TIME_OUT;
	    }
	    return SOCKERR_BUSY;
	//} else IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
	} else IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, Sn_IR_SEND_OK);
	//ClearSubnet();    // for ARP Errata
	    
	return len;
}


/**
@brief	This function is an application I/F function which is used to receive the data in other then
	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well. 
	
@return	This function return received data size for success else -1.
*/ 
int16 UDPRecv(SOCKET s, uint8 * buf, uint16 len, uint8 * addr, uint16 *port)
{
	uint8 prebuf[8], status = 0;
	uint16 tmp_len = 0, RSR_len = 0;
	
	if(s > TOTAL_SOCK_NUM) {
	    ERRA("wrong socket number(%d)", s);
	    return SOCKERR_NOT_UDP;
	} else if(len == 0) {
	    ERR("Zero length");
	    return SOCKERR_WRONG_ARG;
	}
	
	status = getSn_SR(s);
	if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
	//if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
	if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
	if(status != SOCK_UDP) return SOCKERR_NOT_UDP;
	
	RSR_len = GetSocketRxRecvBufferSize(s); // Check Receive Buffer of W5500
	if(RSR_len < 8) {
	    DBGA("wrong data received (%d)", RSR_len);
	    recv_data_ignore(s, RSR_len);
	    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
	    //while(IINCHIP_READ(Sn_CR(s)));
	
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
	    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
	
	    return SOCKERR_NOT_SPECIFIED;
	} else {
	    recv_data_processing(s, prebuf, 8);
	    //IINCHIP_WRITE(Sn_CR(s), Sn_CR_RECV);  // �����͸� ó���� �� �̰��� ����� �����
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);    // �����͸� ó���� �� �̰��� ����� �����
	
	    if(addr) {      // read peer's IP address, port number.
	        addr[0] = prebuf[0];
	        addr[1] = prebuf[1];
	        addr[2] = prebuf[2];
	        addr[3] = prebuf[3];
	    }
	    if(port) {
	        *port = prebuf[4];
	        *port = (*port << 8) + prebuf[5];
	    }
	    tmp_len = prebuf[6];
	    tmp_len = (tmp_len << 8) + prebuf[7];
	    //while(IINCHIP_READ(Sn_CR(s)));    // IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV); ��� �� �ؾ���, �ð� ������ �ٷ� ����
	
	    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));  // IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV); ��� �� �ؾ���, �ð� ������ �ٷ� ����
	
	    DBGA("UDP Recv - addr(%d.%d.%d.%d:%d), t(%d), R(%d)",
	        addr[0], addr[1], addr[2], addr[3], *port, tmp_len, RSR_len);
	    if(tmp_len == 0) {
	        ERR("UDP Recv len Zero - remove rest all");
	        recv_data_ignore(s, GetSocketRxRecvBufferSize(s));
	        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
	        //while(IINCHIP_READ(Sn_CR(s)));
	
	        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
	        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
	
	        return SOCKERR_NOT_SPECIFIED;
	    }
	    RSR_len = tmp_len;
	}
	
	if(len < RSR_len) {
	    tmp_len = RSR_len - len;
	    RSR_len = len;
	    DBGA("Recv buffer not enough - len(%d)", len);
	} else tmp_len = 0;
	
	//switch (IINCHIP_READ(Sn_MR(s)) & 0x07)
	switch (IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x07)
	{
	case Sn_MR_UDP:
	    recv_data_processing(s, (uint8*)buf, RSR_len);
	    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
	    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
	
	    if(tmp_len) {
	        //while(IINCHIP_READ(Sn_CR(s)));
	
	        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
	        DBG("Ignore rest data");
	        recv_data_ignore(s, tmp_len); // �ȹ����� ���� ó���� �����
	        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
	        //while(IINCHIP_READ(Sn_CR(s)));
	
	        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
	        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
	        tmp_len = GetSocketRxRecvBufferSize(s);
	        if(tmp_len) DBGA("another rest data(%d)", tmp_len);
	        else DBG("No rest data");
	    }
	    break;
	case Sn_MR_IPRAW:
	case Sn_MR_MACRAW:
	default :
	    break;
	}
	//while(IINCHIP_READ(Sn_CR(s)));
	while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
	
	return RSR_len;
}


































#if 0
/* W5500 Original Socket Code*/
/************************************************************************************************************************************/
/**
@brief  This Socket function open TCP server socket.
@return     1 - success, 0 - fail.
*/
int8 TCPServerOpen(uint8 s, uint16 port)
{
    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else DBG("start");

    if (port == 0) {    // if don't set the source port, set local_port number.
        if(local_port == 0xffff) local_port = 0xc000;
        else local_port++;
        port = local_port;
    }

    TCPClose(s);

    //IINCHIP_WRITE(Sn_MR(s),Sn_MR_TCP);
    //IINCHIP_WRITE(Sn_PORT0(s),(uint8)((port & 0xff00) >> 8));
    //IINCHIP_WRITE((Sn_PORT0(s) + 1),(uint8)(port & 0x00ff));
    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_OPEN); // run sockinit Sn_CR
    //while(IINCHIP_READ(Sn_CR(s)));    // wait to process the command...
    //DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ(Sn_SR(s)), IINCHIP_READ(Sn_MR(s)));

    IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, Sn_MR_TCP);
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((port & 0xff00) >> 8));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(port & 0x00ff));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR)){LOGF(".");}        // wait to process the command...
    DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));
        //PRINTF("\r\nOPEN -- Sn_SR = %.2x(<-%.2X) , Protocol = %.2x(<-%.2X:TCP) \r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_INIT ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_TCP);


    //if (IINCHIP_READ(Sn_SR(s)) != SOCK_INIT) {
        //DBGA("wrong status(%d)", IINCHIP_READ(Sn_SR(s)));
        if(IINCHIP_READ_SOCKETREG(s, WIZS_SR) != SOCK_INIT) {
        DBGA("wrong status(%d)", IINCHIP_READ_SOCKETREG(s, WIZS_SR));
        return SOCKERR_WRONG_STATUS;
    } else {
        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_LISTEN);
        //while(IINCHIP_READ(Sn_CR(s)));              // wait to process the command...
        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_LISTEN);
        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
                //PRINTF("\r\nLISTEN -- Sn_SR = %.2x(<-%.2x) , Protocol = %.2x(<-%.2X:TCP)\r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_LISTEN ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_TCP);
    }

    return RET_OK;
}

/**
@brief  This Socket function open TCP client socket.
@return     1 - success, 0 - fail.
*/
int8 TCPClientOpen(uint8 s, uint16 sport, uint8 *dip, uint16 dport)
{
    int8 ret;

    DBG("start");
    ret = TCPCltOpenNB(s, sport, dip, dport);
    if(ret != RET_OK) return ret;

    do {
        ret = TCPConnChk(s);
    } while(ret == SOCKERR_BUSY);

    return ret;
}

int8 TCPCltOpenNB(uint8 s, uint16 sport, uint8 *dip, uint16 dport)
{
    uint8 srcip[4], snmask[4];

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else if(dip == NULL) {
        ERR("NULL Dst IP");
        return SOCKERR_WRONG_ARG;
    } else DBG("start");

    if (sport == 0) {   // if don't set the source port, set local_port number.
        if(local_port == 0xffff) local_port = 0xc000;
        else local_port++;
        sport = local_port;
    }

    TCPClose(s);

    //IINCHIP_WRITE(Sn_MR(s),Sn_MR_TCP);
    //IINCHIP_WRITE(Sn_PORT0(s),(uint8)((sport & 0xff00) >> 8));
    //IINCHIP_WRITE((Sn_PORT0(s) + 1),(uint8)(sport & 0x00ff));
    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_OPEN); // run sockinit Sn_CR
    //while(IINCHIP_READ(Sn_CR(s)) );   // wait to process the command...
    //DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ(Sn_SR(s)), IINCHIP_READ(Sn_MR(s)));

    IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, Sn_MR_TCP);
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((sport & 0xff00) >> 8));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(sport & 0x00ff));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
    DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));

    getSIPR(srcip);
    getSUBR(snmask);

    if( ((dip[0] == 0xFF) && (dip[1] == 0xFF) &&
         (dip[2] == 0xFF) && (dip[3] == 0xFF)) ||
        ((dip[0] == 0x00) && (dip[1] == 0x00) &&
         (dip[2] == 0x00) && (dip[3] == 0x00)) || (sport == 0x00) )
    {
        DBG("invalid ip or port");
        DBGA("SOCK(%d)-[%02x.%02x.%02x.%02x, %d]",s,
            dip[0], dip[1], dip[2], dip[3] , sport);
        return SOCKERR_WRONG_ARG;
    }
    else if( (srcip[0]==0 && srcip[1]==0 && srcip[2]==0 && srcip[3]==0) &&
        (snmask[0]!=0 || snmask[1]!=0 || snmask[2]!=0 || snmask[3]!=0) ) //Mikej : ARP Errata
    {
        DBG("Source IP is NULL while SN Mask is Not NULL");
        return SOCKERR_NULL_SRC_IP;
    }
    else
    {
        //IINCHIP_WRITE(Sn_DIPR0(s),dip[0]);    // set destination IP
        //IINCHIP_WRITE((Sn_DIPR0(s) + 1),dip[1]);
        //IINCHIP_WRITE((Sn_DIPR0(s) + 2),dip[2]);
        //IINCHIP_WRITE((Sn_DIPR0(s) + 3),dip[3]);
        //IINCHIP_WRITE(Sn_DPORT0(s),(uint8)((dport & 0xff00) >> 8));
        //IINCHIP_WRITE((Sn_DPORT0(s) + 1),(uint8)(dport & 0x00ff));
        ////SetSubnet(sn);  // for ARP Errata
        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_CONNECT);
        //while (IINCHIP_READ(Sn_CR(s)) );  // wait for completion
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 0, dip[0]); // set destination IP
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 1, dip[1]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 2, dip[2]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 3, dip[3]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 0, (uint8)((dport & 0xff00) >> 8));
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 1, (uint8)(dport & 0x00ff));
        //SetSubnet(sn);    // for ARP Errata
        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR,Sn_CR_CONNECT);
        while (IINCHIP_READ_SOCKETREG(s, WIZS_CR)); // wait for completion
    }

    return RET_OK;
}

int8 TCPConnChk(uint8 s)
{
    uint8 socksr;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    }

    //socksr = IINCHIP_READ(Sn_SR(s));
    socksr = IINCHIP_READ_SOCKETREG(s, WIZS_SR);

    //if(socksr == SOCK_ESTABLISHED || socksr == SOCK_SYNSENT) {        ????????
    if(socksr == SOCK_ESTABLISHED) {
        //ClearSubnet();    // for ARP Errata
        return RET_OK;
    //} else if(IINCHIP_READ(Sn_IR(s)) & Sn_IR_TIMEOUT) {
        //IINCHIP_WRITE(Sn_IR(s), (Sn_IR_TIMEOUT));           // clear TIMEOUT Interrupt
        } else if(IINCHIP_READ_SOCKETREG(s, WIZS_IR) & Sn_IR_TIMEOUT) {
        IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, Sn_IR_TIMEOUT);   // clear TIMEOUT Interrupt
        //ClearSubnet();    // for ARP Errata
        return SOCKERR_TIME_OUT;
    }

    return SOCKERR_BUSY;
}

/**
@brief  This Socket function open UDP socket.
@return     0 - success, 1 - fail.
*/
int8 UDPOpen(uint8 s, uint16 port)
{
    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_UDP;
    } else DBG("start");

    if (port == 0) {    // if don't set the source port, set local_port number.
        if(local_port == 0xffff) local_port = 0xc000;
        else local_port++;
        port = local_port;
    }

    UDPClose(s);

    //IINCHIP_WRITE(Sn_MR(s),Sn_MR_UDP);
    //IINCHIP_WRITE(Sn_PORT0(s),(uint8)((port & 0xff00) >> 8));
    //IINCHIP_WRITE((Sn_PORT0(s) + 1),(uint8)(port & 0x00ff));
    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_OPEN); // run sockinit Sn_CR
    //while(IINCHIP_READ(Sn_CR(s)));    // wait to process the command...
    //DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ(Sn_SR(s)), IINCHIP_READ(Sn_MR(s)));

    IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, Sn_MR_UDP);
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((port & 0xff00) >> 8));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(port & 0x00ff));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
    DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));
        //PRINTF("\r\nOPEN -- Sn_SR = %.2x(<-%.2X) , Protocol = %.2x(<-%.2X:UDP) \r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_UDP ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_UDP);
    return RET_OK;
}

/**
@brief  This Socket function open UDP socket.
@return     1 - success, 0 - fail.
*/
int8 UDPOpenM(uint8 s, uint8 mode, uint16 port)
{

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_UDP;
    } else DBG("start");

    if (port == 0) {    // if don't set the source port, set local_port number.
        if(local_port == 0xffff) local_port = 0xc000;
        else local_port++;
        port = local_port;
    }

    UDPClose(s);

    IINCHIP_WRITE_SOCKETREG(s, WIZS_MR, (uint8)(Sn_MR_UDP|mode) );
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 0, (uint8)((port & 0xff00) >> 8));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_PORT0 + 1, (uint8)(port & 0x00ff));
    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_OPEN);  // run sockinit Sn_CR
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));        // wait to process the command...
    DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(s, WIZS_SR), IINCHIP_READ_SOCKETREG(s, WIZS_MR));
        //PRINTF("\r\nOPEN -- Sn_SR = %.2x(<-%.2X) , Protocol = %.2x(<-%.2X:UDP) \r\n", IINCHIP_READ_SOCKETREG(s, WIZS_SR), SOCK_UDP ,IINCHIP_READ_SOCKETREG(s, WIZS_MR), Sn_MR_UDP|mode);
    return RET_OK;
}

/**
@brief  This function close the TCP socket and parameter is "s" which represent the socket number
@return     1 - success, 0 - fail.
*/
int8 TCPClose(uint8 s)
{
    int8 ret;

    DBG("start");
    ret = TCPCloseNB(s);
    if(ret != RET_OK) return ret;

    do {
        ret = TCPCloseCHK(s);
    } while(ret == SOCKERR_BUSY);

    return ret;
}


int8 TCPCloseNB(uint8 s)
{
    uint8 status;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else DBG("start");

    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_DISCON);
    //while(IINCHIP_READ(Sn_CR(s)));            // wait to process the command...
    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_DISCON);
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR)){LOGF("#");}  // wait to process the command...

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) return SOCKERR_WRONG_STATUS;
    //else tcp_close_elapse[s] = wizpf_get_systick();
    else tcp_close_elapse[s]++; 

    return RET_OK;
}

int8 TCPCloseCHK(uint8 s)
{
#define TIMEOUT_CLOSE_WAIT  200  
    uint8 status;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else DBG("start");

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) goto END_OK;
    else if(wizpf_tick_elapse(tcp_close_elapse[s]) < TIMEOUT_CLOSE_WAIT)
        return SOCKERR_BUSY;

    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);
    //while(IINCHIP_READ(Sn_CR(s)));        // wait to process the command...
    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_CLOSE);
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR)){LOGF("?");}  // wait to process the command...

END_OK:
    //IINCHIP_WRITE(Sn_IR(s), 0xFF);    // interrupt all clear
    IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, 0xFF);  // interrupt all clear
    return RET_OK;
}


int8 TCPClsRcvCHK(uint8 s)
{
#define TIMEOUT_CLOSE_WAIT  200  
    uint8 status;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    }

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) goto END_OK;
    if(status == SOCK_CLOSE_WAIT) {
        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);
        //while(IINCHIP_READ(Sn_CR(s)));        // wait to process the command...
        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_CLOSE);
        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));  // wait to process the command...
    } else return SOCKERR_BUSY;

END_OK:
    //IINCHIP_WRITE(Sn_IR(s), 0xFF);    // interrupt all clear
    IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, 0xFF);  // interrupt all clear
    return RET_OK;
}

/**
@brief  This function close the UDP socket and parameter is "s" which represent the socket number
@return     1 - success, 0 - fail.
*/
int8 UDPClose(uint8 s)
{
    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_UDP;
    } else DBG("start");

    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);
    //while(IINCHIP_READ(Sn_CR(s)));                // wait to process the command...
    //IINCHIP_WRITE(Sn_IR(s), 0xFF);            // interrupt all clear

    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_CLOSE);
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));      // wait to process the command...
    IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, 0xFF);  // interrupt all clear

    return RET_OK;
}

/**
@brief  This function used to send the data in TCP mode
@return 1 for success else 0.
*/
int32 TCPSend(uint8 s, const int8 *src, uint16 len)
{
    int32 ret;

    while(1) {
        ret = TCPSendNB(s, src, len);
        if(ret == RET_OK) break;
        if(ret != SOCKERR_BUSY) return ret;
    }

    while(1) {
        ret = TCPSendCHK(s);
        if(ret >= 0 || ret != SOCKERR_BUSY) break;
    }

    return ret;
}

int8 TCPSendNB(uint8 s, const int8 *src, uint16 len)
{
    uint8 status = 0;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else if(len == 0) {
        ERR("Zero length");
        return SOCKERR_WRONG_ARG;
    } else DBG("start");

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
    //if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;
    if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;

    if(status == SOCK_FIN_WAIT) return SOCKERR_FIN_WAIT;
    if(status != SOCK_ESTABLISHED && status != SOCK_CLOSE_WAIT) return SOCKERR_NOT_ESTABLISHED;

    init_windowfull_retry_cnt(s);
    if(len > getIINCHIP_TxMAX(s)) len = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.
    if(GetSocketTxFreeBufferSize(s) < len) return SOCKERR_BUSY;

    send_data_processing(s, (uint8*)src, len);  // copy data

    //txrd_checker[s] = IINCHIP_READ(Sn_TX_RD0(s));
    //txrd_checker[s] = (txrd_checker[s] << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);

    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
    //while(IINCHIP_READ(Sn_CR(s)));                // wait to process the command...

    txrd_checker[s] = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
    txrd_checker[s] = (txrd_checker[s] << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);

    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR,Sn_CR_SEND);
    while (IINCHIP_READ_SOCKETREG(s, WIZS_CR)); // wait to process the command...

    return RET_OK;
}

/**
@brief  This function used to send the data in TCP mode
@return 1 for success else 0.
*/
int32 TCPReSend(uint8 s)
{
    int32 ret;

    while(1) {
        ret = TCPReSendNB(s);
        if(ret == RET_OK) break;
        if(ret != SOCKERR_BUSY) return ret;
    }

    while(1) {
        ret = TCPSendCHK(s);
        if(ret >= 0 || ret != SOCKERR_BUSY) break;
    }

    return ret;
}

int8 TCPReSendNB(uint8 s)
{
    uint8 status=0;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else DBG("start");

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
    //if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;
        if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;

    if(status == SOCK_FIN_WAIT) return SOCKERR_FIN_WAIT;
    if(status != SOCK_ESTABLISHED && status != SOCK_CLOSE_WAIT) return SOCKERR_NOT_ESTABLISHED;

    status = incr_windowfull_retry_cnt(s);
    if(status == 1) tcp_resend_elapse[s] = wizpf_get_systick();
    else if(status > WINDOWFULL_MAX_RETRY_NUM) return SOCKERR_WINDOW_FULL;
    else if(wizpf_tick_elapse(tcp_resend_elapse[s]) < WINDOWFULL_WAIT_TIME)
        return SOCKERR_BUSY;

    //txrd_checker[s] = IINCHIP_READ(Sn_TX_RD0(s));
    //txrd_checker[s] = (txrd_checker[s] << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);

    //IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
    //while(IINCHIP_READ(Sn_CR(s)));                // wait to process the command...

    txrd_checker[s] = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
    txrd_checker[s] = (txrd_checker[s] << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);

    IINCHIP_WRITE_SOCKETREG(s, WIZS_CR,Sn_CR_SEND);
    while (IINCHIP_READ_SOCKETREG(s, WIZS_CR)); // wait to process the command...

    return RET_OK;
}

int32 TCPSendCHK(uint8 s)
{
    uint16 txrd;

    //if(!(IINCHIP_READ(Sn_IR(s)) & Sn_IR_SEND_OK)) {
        //if(IINCHIP_READ(Sn_SR(s)) == SOCK_CLOSED) {
    if(!(IINCHIP_READ_SOCKETREG(s, WIZS_IR) & Sn_IR_SEND_OK)) {
        if(IINCHIP_READ_SOCKETREG(s, WIZS_SR) == SOCK_CLOSED) {
            DBG("SOCK_CLOSED");
            TCPClose(s);
            return SOCKERR_CLOSED;
        }
        return SOCKERR_BUSY;
    //} else IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
    } else IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, Sn_IR_SEND_OK);

    //txrd = IINCHIP_READ(Sn_TX_RD0(s));
    //txrd = (txrd << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);

    txrd = IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 0);
    txrd = (txrd << 8) + IINCHIP_READ_SOCKETREG(s, WIZS_TX_RD0 + 1);

    if(txrd > txrd_checker[s]) return txrd - txrd_checker[s];
    else return (0xffff - txrd_checker[s]) + txrd + 1;
}

/**
@brief  This function is an application I/F function which is used to receive the data in TCP mode.
        It continues to wait for data as much as the application wants to receive.

@return received data size for success else -1.
*/
int32 TCPRecv(uint8 s, int8 *buf, uint16 len)
{
    uint8 status = 0;
    uint16 RSR_len = 0;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_TCP;
    } else if(len == 0) {
        ERR("Zero length");
        //return SOCKERR_WRONG_ARG; //W5500 �׽�Ʈ �Ѵٰ� �ּ� ó�� �� 
    }

    RSR_len = GetSocketRxRecvBufferSize(s); // Check Receive Buffer of W5500
    if(RSR_len == 0){
        status = getSn_SR(s);
        if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
        //if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;
        if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_TCP) return SOCKERR_NOT_TCP;

        if(status == SOCK_CLOSE_WAIT) return SOCKERR_CLOSE_WAIT;
        if(status != SOCK_ESTABLISHED && status != SOCK_CLOSE_WAIT) return SOCKERR_NOT_ESTABLISHED;
    } else {
        if(len < RSR_len) RSR_len = len;
        recv_data_processing(s, (uint8*)buf, RSR_len);
        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
        //while(IINCHIP_READ(Sn_CR(s)));                // wait to process the command...

        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));      // wait to process the command...
    }

    return RSR_len;
}

/**
@brief  This function is an application I/F function which is used to send the data for other then TCP mode.
        Unlike TCP transmission, The peer's destination address and the port is needed.

@return This function return send data size for success else -1.
*/
int32 UDPSend(uint8 s, const int8 *buf, uint16 len, uint8 *addr, uint16 port)
{
    int32 ret = 0;

    ret = UDPSendNB(s, buf, len, addr, port);
    if(ret < RET_OK) return ret;
    else len = ret;

    do {
        ret = UDPSendCHK(s);
        if(ret == RET_OK) return len;
        if(ret != SOCKERR_BUSY) return ret;
    } while(1);
}

int32 UDPSendNB(uint8 s, const int8 *buf, uint16 len, uint8 *addr, uint16 port)
{
    uint8 srcip[4], snmask[4], status = 0;

    if(s > TOTAL_SOCK_NUM) 
    {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_UDP;
    }
    else if(len == 0)
    {
        ERR("Zero length");
        return SOCKERR_WRONG_ARG; //W5500 UDPSend TEST���� �ּ�ó����
    } 
    else if(addr == NULL) 
    {
        ERR("NULL Dst IP");
        return SOCKERR_WRONG_ARG;
    } else DBG("start");

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
    //if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
    if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
    if(status != SOCK_UDP) return SOCKERR_NOT_UDP;

    if (len > getIINCHIP_TxMAX(s)) len = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.

    getSIPR(srcip);
    getSUBR(snmask);

    if((addr[0]==0x00 && addr[1]==0x00 && addr[2]==0x00 &&
        addr[3]==0x00) || (port==0x00))
    {
        DBG("invalid ip or port");
        DBGA("SOCK(%d)-[%02x.%02x.%02x.%02x, %d, %d]",s,
            addr[0], addr[1], addr[2], addr[3] , port, len);
        return SOCKERR_WRONG_ARG;
    }
#if 0 //-- W5200�� Erratum2 => W5500���� ����! W5500������ �ʿ���� �κ�
    else if( (srcip[0]==0 && srcip[1]==0 && srcip[2]==0 && srcip[3]==0) &&
        (snmask[0]!=0 || snmask[1]!=0 || snmask[2]!=0 || snmask[3]!=0) ) //Mikej : ARP Errata
    {
        DBG("Source IP is NULL while SN Mask is Not NULL");
        return SOCKERR_NULL_SRC_IP;
    }
#endif
    else
    {
        //IINCHIP_WRITE(Sn_DIPR0(s),addr[0]);
        //IINCHIP_WRITE((Sn_DIPR0(s) + 1),addr[1]);
        //IINCHIP_WRITE((Sn_DIPR0(s) + 2),addr[2]);
        //IINCHIP_WRITE((Sn_DIPR0(s) + 3),addr[3]);
        //IINCHIP_WRITE(Sn_DPORT0(s),(uint8)((port & 0xff00) >> 8));
        //IINCHIP_WRITE((Sn_DPORT0(s) + 1),(uint8)(port & 0x00ff));
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 0, addr[0]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 1, addr[1]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 2, addr[2]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DIPR0 + 3, addr[3]);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 0,(uint8)((port & 0xff00) >> 8));
        IINCHIP_WRITE_SOCKETREG(s, WIZS_DPORT0 + 1,(uint8)(port & 0x00ff));

        send_data_processing(s, (uint8*)buf, len);  // copy data
        //SetSubnet(sn);    // for ARP Errata

        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);
        //while(IINCHIP_READ(Sn_CR(s)));  // wait to process the command...
        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_SEND);
        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));  // wait to process the command...
    }

    return len;
}

int8 UDPSendCHK(uint8 s)
{
    //uint8 ir = IINCHIP_READ(Sn_IR(s));
    uint8 ir = IINCHIP_READ_SOCKETREG(s, WIZS_IR);

    //DBGA("WATCH UDP Send CHK - sock(%d)", s);
    if(!(ir & Sn_IR_SEND_OK)) {
        if(ir & Sn_IR_TIMEOUT) {
            DBG("send fail");
            //IINCHIP_WRITE(Sn_IR(s), (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); // clear SEND_OK & TIMEOUT
            IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, (Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); // clear SEND_OK & TIMEOUT Interrupt
            return SOCKERR_TIME_OUT;
        }
        return SOCKERR_BUSY;
    //} else IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);
    } else IINCHIP_WRITE_SOCKETREG(s, WIZS_IR, Sn_IR_SEND_OK);
    //ClearSubnet();    // for ARP Errata

    return RET_OK;
}

/**
@brief  This function is an application I/F function which is used to receive the data in other then
    TCP mode. This function is used to receive UDP and MAC_RAW mode, and handle the header as well.

@return This function return received data size for success else -1.
*/
int32 UDPRecv(uint8 s, int8 *buf, uint16 len, uint8 *addr, uint16 *port)
{
    uint8 prebuf[8], status = 0;
    uint16 tmp_len = 0, RSR_len = 0;

    if(s > TOTAL_SOCK_NUM) {
        ERRA("wrong socket number(%d)", s);
        return SOCKERR_NOT_UDP;
    } else if(len == 0) {
        ERR("Zero length");
        return SOCKERR_WRONG_ARG;
    }

    status = getSn_SR(s);
    if(status == SOCK_CLOSED) return SOCKERR_CLOSED;
    //if((IINCHIP_READ(Sn_MR(s))&0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
    if((IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x0F) != Sn_MR_UDP) return SOCKERR_NOT_UDP;
    if(status != SOCK_UDP) return SOCKERR_NOT_UDP;

    RSR_len = GetSocketRxRecvBufferSize(s); // Check Receive Buffer of W5500
    if(RSR_len < 8) {
        DBGA("wrong data received (%d)", RSR_len);
        recv_data_ignore(s, RSR_len);
        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
        //while(IINCHIP_READ(Sn_CR(s)));

        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));

        return SOCKERR_NOT_SPECIFIED;
    } else {
        recv_data_processing(s, prebuf, 8);
        //IINCHIP_WRITE(Sn_CR(s), Sn_CR_RECV);  // �����͸� ó���� �� �̰��� ����� �����
        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);    // �����͸� ó���� �� �̰��� ����� �����

        if(addr) {      // read peer's IP address, port number.
            addr[0] = prebuf[0];
            addr[1] = prebuf[1];
            addr[2] = prebuf[2];
            addr[3] = prebuf[3];
        }
        if(port) {
            *port = prebuf[4];
            *port = (*port << 8) + prebuf[5];
        }
        tmp_len = prebuf[6];
        tmp_len = (tmp_len << 8) + prebuf[7];
        //while(IINCHIP_READ(Sn_CR(s)));    // IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV); ��� �� �ؾ���, �ð� ������ �ٷ� ����

        while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));  // IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV); ��� �� �ؾ���, �ð� ������ �ٷ� ����

        DBGA("UDP Recv - addr(%d.%d.%d.%d:%d), t(%d), R(%d)",
            addr[0], addr[1], addr[2], addr[3], *port, tmp_len, RSR_len);
        if(tmp_len == 0) {
            ERR("UDP Recv len Zero - remove rest all");
            recv_data_ignore(s, GetSocketRxRecvBufferSize(s));
            //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
            //while(IINCHIP_READ(Sn_CR(s)));

            IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
            while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));

            return SOCKERR_NOT_SPECIFIED;
        }
        RSR_len = tmp_len;
    }

    if(len < RSR_len) {
        tmp_len = RSR_len - len;
        RSR_len = len;
        DBGA("Recv buffer not enough - len(%d)", len);
    } else tmp_len = 0;

    //switch (IINCHIP_READ(Sn_MR(s)) & 0x07)
    switch (IINCHIP_READ_SOCKETREG(s, WIZS_MR) & 0x07)
    {
    case Sn_MR_UDP:
        recv_data_processing(s, (uint8*)buf, RSR_len);
        //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
        IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);

        if(tmp_len) {
            //while(IINCHIP_READ(Sn_CR(s)));

            while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
            DBG("Ignore rest data");
            recv_data_ignore(s, tmp_len); // �ȹ����� ���� ó���� �����
            //IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
            //while(IINCHIP_READ(Sn_CR(s)));

            IINCHIP_WRITE_SOCKETREG(s, WIZS_CR, Sn_CR_RECV);
            while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));
            tmp_len = GetSocketRxRecvBufferSize(s);
            if(tmp_len) DBGA("another rest data(%d)", tmp_len);
            else DBG("No rest data");
        }
        break;
    case Sn_MR_IPRAW:
    case Sn_MR_MACRAW:
    default :
        break;
    }
    //while(IINCHIP_READ(Sn_CR(s)));
    while(IINCHIP_READ_SOCKETREG(s, WIZS_CR));

    return RSR_len;
}

#endif
/************************************************************************************************************************************/





































/**
 * MACRAW��带 �̿��� PPPoE�� �����ϱ� ���� �߰��մϴ�.
 * MACRAW open / send / recv / close �߰� @2013-06-07 ���� 9:58:35
 * W5500 CORE TEST������ �߰� W5200���� �߰��ؾ��մϴ�.
 */

uint8 MACRAWClose(void)
{
    uint8 sock_num = 0;                                //MACRAW�� ���� 0���� ����Ѵ�.
    IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_CR, Sn_CR_CLOSE);
    while(IINCHIP_READ_SOCKETREG(sock_num, WIZS_CR));  // wait to process the command...
    IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_IR, 0xFF);  // interrupt all clear
    return RET_OK;
}

uint8 MACRAWOpen(void)
{
    uint8 sock_num = 0;
    uint16 dummyPort = 0;
    

    MACRAWClose();
    IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_MR , Sn_MR_MACRAW );
    IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_PORT0     ,(uint8)((dummyPort & 0xff00) >> 8));
    IINCHIP_WRITE_SOCKETREG(sock_num, (WIZS_PORT0+ 1),(uint8)(dummyPort & 0x00ff));
    IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_CR ,Sn_CR_OPEN); // run sockinit Sn_CR
    while( IINCHIP_READ_SOCKETREG(sock_num, WIZS_CR ) );           // wait to process the command...
    DBGA("Sn_SR = %.2x , Protocol = %.2x", IINCHIP_READ_SOCKETREG(sock_num, WIZS_SR), IINCHIP_READ_SOCKETREG(sock_num, WIZS_MR));
    PRINTF("\r\nOPEN -- Sn_SR = %.2x(<-%.2X) , Protocol = %.2x(<-%.2X:MACRAW) \r\n", IINCHIP_READ_SOCKETREG(sock_num, WIZS_SR), SOCK_MACRAW ,IINCHIP_READ_SOCKETREG(sock_num, WIZS_MR), Sn_MR_MACRAW);
        
    return RET_OK;
}


uint16 MACRAWSend( const uint8 * buf, uint16 len )
{
   uint8 sock_num = 0;

   if (len > getIINCHIP_TxMAX(sock_num)) len = getIINCHIP_TxMAX(sock_num); // check size not to exceed MAX size.

#if 1
   PRINTF("len : %d \r\n", len);
   PRINTF("MAC RAW msg SEND\r\n");
   PRINTF("dest mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
   PRINTF("src  mac=%.2X.%.2X.%.2X.%.2X.%.2X.%.2X\r\n",buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
   PRINTF("type    =%.2X%.2X\r\n",buf[12],buf[13]);
#endif

   send_data_processing(sock_num, (uint8 *)buf, len);
   IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_CR,Sn_CR_SEND);
   while( IINCHIP_READ_SOCKETREG(sock_num, WIZS_CR) ); // wait to process the command...
   while ( (IINCHIP_READ_SOCKETREG(sock_num, WIZS_IR) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK );
   IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_IR, Sn_IR_SEND_OK);

   return len;
}

uint16 MACRAWRecv( uint8 * buf, uint16 len )
{
   uint8 sock_num = 0;
   uint16 data_len = 0;
   uint16 ptr = 0;

#if 1
   PRINTF("macrecv()\r\n");
#endif

   if ( len > 0 )
   {

#if 1
      PRINTF("ISR_RX: rd_ptr : %.4x\r\n", ptr);
#endif
      data_len = 0;

      ptr = IINCHIP_READ_SOCKETREG(sock_num, WIZS_RX_RD0);
      ptr = (uint16)((ptr & 0x00ff) << 8) + IINCHIP_READ_SOCKETREG(sock_num, WIZS_RX_RD0 + 1);
      
      //-- read_dlen
      data_len = IINCHIP_READ_RXBUF(sock_num, ptr);
      ptr++;
      data_len = ((data_len<<8) + IINCHIP_READ_RXBUF(sock_num, ptr)) - 2; // -2�� dlen�� length 2byte
      ptr++;

#if 1
      PRINTF("\r\nptr: %X, data_len: %X", ptr, data_len);
#endif
      if(data_len > 1514)
      {
         PRINTF("data_len over 1514\r\n");
         PRINTF("\r\nptr: %X, data_len: %X", ptr, data_len);

#if 1
         IINCHIP_READ_RXBUF_SEQ(sock_num, 0, 64, (uint8*)(buf));
          uint16 idx;
          //for(idx=0; idx<data_len; idx++)
          for(idx=0; idx<1024; idx++)
          {
            if(idx>0 && (idx%16)==0)   PRINTF("\r\n");
            PRINTF("%.2X, ", buf[idx]);
          }
          PRINTF("------------\r\n");
#endif
         //while(1);
         /** recommand : close and open **/
         MACRAWClose();
         MACRAWOpen();
         return 0;
      }

      IINCHIP_READ_RXBUF_SEQ(sock_num, ptr, data_len, (uint8*)(buf));
      ptr += data_len;
#if 1
      PRINTF("ptr: %X \r\n", ptr);
#endif
      IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_RX_RD0,(uint8)((ptr & 0xff00) >> 8));
      IINCHIP_WRITE_SOCKETREG(sock_num, (WIZS_RX_RD0 + 1),(uint8)(ptr & 0x00ff));
      IINCHIP_WRITE_SOCKETREG(sock_num, WIZS_CR, Sn_CR_RECV);
      while( IINCHIP_READ_SOCKETREG(sock_num, WIZS_CR) ) ;
   }

#if 1
   PRINTF("macrecv() end ..\r\n");
#endif

   return data_len;
}

/*
void IINCHIP_WRITE_COMMON( uint16 addr,  uint8 data);
uint8 IINCHIP_READ_COMMON(uint16 addr);
void IINCHIP_READ_COMMON_SEQ(uint16 addr, uint8 len, uint8 * data); 
void IINCHIP_WRITE_COMMON_SEQ(uint16 addr, uint8 len, uint8 * data); 
*/

void SetPPPoeInfo(uint8 *dmac, uint8 *ip, uint16 sid, uint16 ptimer)
{

    //uint8 sock_num = 0;
    uint16 i;

#if 1
    PRINTF("-Server's MAC : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\r\n", dmac[0], dmac[1], dmac[2], dmac[3], dmac[4], dmac[5]);
    PRINTF("-PPPoE IP : %.3d.%.3d.%.3d.%.3d\r\n", ip[0], ip[1], ip[2], ip[3]);
    PRINTF("-Session ID : 0x%.2X%.2X\r\n", (uint8)(sid >> 8), (uint8)sid);
#endif    

    IINCHIP_WRITE_COMMON( WIZC_MR, (IINCHIP_READ_COMMON(WIZC_MR)|MR_PPPOE) );// Set PPPoE bit in MR(Common Mode Register) : enable socket0 pppoe

    // Write PPPoE server's MAC address, Session ID and IP address.
    // must be setted these value.
    for (i = 0; i < 6; i++) IINCHIP_WRITE_COMMON((WIZC_PDHA0+i), dmac[i]);
    for (i = 0; i < 4; i++) IINCHIP_WRITE_COMMON((WIZC_SIPR0+i),  ip[i]);
    //IINCHIP_WRITE_COMMON_SEQ( WIZC_PDHA0, 6, dmac);
    //IINCHIP_WRITE_COMMON_SEQ( WIZC_SIPR0, 4, ip);
    IINCHIP_WRITE_COMMON((WIZC_PSID0),   (uint8)(sid >> 8));
    IINCHIP_WRITE_COMMON((WIZC_PSID0+1), (uint8)(sid));
    IINCHIP_WRITE_COMMON( WIZC_PTIMER, ptimer );                          // 5sec timeout

#if 1
    PRINTF("Read Server's MAC :");
    for (i = 0; i < 6; i++) PRINTF("%.2X:",IINCHIP_READ_COMMON(WIZC_PDHA0+i));
    PRINTF("\r\n");
    PRINTF("Read PPPoE IP :");
    for (i = 0; i < 4; i++) PRINTF("%.3d:",IINCHIP_READ_COMMON(WIZC_SIPR0+i));
    PRINTF("\r\n");
    PRINTF("Session ID : 0x%.2X%.2X\r\n", IINCHIP_READ_COMMON(WIZC_PSID0), IINCHIP_READ_COMMON(WIZC_PSID0+1) );
#endif  

//IINCHIP_WRITE_SOCKETREG(0, WIZS_MR,Sn_MR_PPPOE);
//IINCHIP_WRITE_SOCKETREG(0, WIZS_CR,Sn_CR_OPEN);
//while( IINCHIP_READ_COMMON(WIZS_CR) );


}



