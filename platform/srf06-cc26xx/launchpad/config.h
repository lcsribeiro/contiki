#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Types.h"

//=================================================



//=================================================
//lucas
//#define __W5500__
#define __WIZ550IO__

#define TCP_LISTEN_PORT	5000
#define UDP_LISTEN_PORT	5000

#define TOTAL_SOCK_NUM 8

#define SOCK_DHCP		    0	// UDP
#define SOCK_DNS		    1	// UDP
#define SOCK_SMTP		    2	// TCP

#define MAX_BUF_SIZE		1460
#define KEEP_ALIVE_TIME	    30	// 30sec

#define ON                  1
#define OFF                 0

#define HIGH                1
#define LOW                 0

#define RET_FAIL	        1
#define RET_OK		        0
#define RET_NOK		        -1

//#define __GNUC__

#define TX_RX_MAX_BUF_SIZE	(256)

extern uint8 TX_BUF[TX_RX_MAX_BUF_SIZE];
extern uint8 RX_BUF[TX_RX_MAX_BUF_SIZE];

#define ApplicationAddress 	0x08004000









//------------------------------------------- LOG ---------------------------------------------
#if !defined(WIZ_LOG_LEVEL) || (WIZ_LOG_LEVEL < 0) || (WIZ_LOG_LEVEL > 3)
#define WIZ_LOG_LEVEL 2
#endif

#if (WIZ_LOG_LEVEL > 0) && defined(PRINT_TIME_LOG) && !defined(FILE_LOG_SILENCE)
#define ERR(fmt)  do { PRINTF("### ERROR ### [%5d.%03d] %s(%d): "fmt"\r\n", \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__); } while(0)
#define ERRA(fmt, ...)  do { PRINTF("### ERROR ### [%5d.%03d] %s(%d): "fmt"\r\n", \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#define ERRF(fmt) do { PRINTF("### ERROR ### [%5d.%03d] %s(%d): "fmt, \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__); } while(0)
#define ERRFA(fmt, ...) do { PRINTF("### ERROR ### [%5d.%03d] %s(%d): "fmt, \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#elif (WIZ_LOG_LEVEL > 0) && !defined(PRINT_TIME_LOG) && !defined(FILE_LOG_SILENCE)
#define ERR(fmt)  do { PRINTF("### ERROR ### %s(%d): "fmt"\r\n", __FUNCTION__, __LINE__); } while(0)
#define ERRA(fmt, ...)  do { PRINTF("### ERROR ### %s(%d): "fmt"\r\n", __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#define ERRF(fmt) do { PRINTF("### ERROR ### %s(%d): "fmt, __FUNCTION__, __LINE__); } while(0)
#define ERRFA(fmt, ...) do { PRINTF("### ERROR ### %s(%d): "fmt, __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#else
#define ERR(fmt)
#define ERRA(fmt, ...)
#define ERRF(fmt)
#define ERRFA(fmt, ...)
#endif

#if (WIZ_LOG_LEVEL > 1) && defined(PRINT_TIME_LOG) && !defined(FILE_LOG_SILENCE)
#define LOG(fmt)  do { PRINTF("[%5d.%03d] "fmt"\r\n", \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000); } while(0)
#define LOGA(fmt, ...)  do { PRINTF("[%5d.%03d] "fmt"\r\n", \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __VA_ARGS__); } while(0)
#define LOGF(fmt) do { PRINTF("[%5d.%03d] "fmt, \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000); } while(0)
#define LOGFA(fmt, ...) do { PRINTF("[%5d.%03d] "fmt, \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __VA_ARGS__); } while(0)
#elif (WIZ_LOG_LEVEL > 1) && !defined(PRINT_TIME_LOG) && !defined(FILE_LOG_SILENCE)
#define LOG(fmt)  do { PRINTF(fmt"\r\n"); } while(0)
#define LOGA(fmt, ...)  do { PRINTF(fmt"\r\n", __VA_ARGS__); } while(0)
#define LOGF(fmt) do { PRINTF(fmt); } while(0)
#define LOGFA(fmt, ...) do { PRINTF(fmt, __VA_ARGS__); } while(0)
#else
#define LOG(fmt)
#define LOGA(fmt, ...)
#define LOGF(fmt)
#define LOGFA(fmt, ...)
#endif

#if (WIZ_LOG_LEVEL > 2) && defined(PRINT_TIME_LOG) && !defined(FILE_LOG_SILENCE)
#define DBG(fmt)  do { PRINTF("[D] [%5d.%03d] %s(%d): "fmt"\r\n", \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__); } while(0)
#define DBGA(fmt, ...)  do { PRINTF("[D] [%5d.%03d] %s(%d): "fmt"\r\n", \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#define DBGF(fmt) do { PRINTF("[D] [%5d.%03d] %s(%d): "fmt, \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__); } while(0)
#define DBGFA(fmt, ...) do { PRINTF("[D] [%5d.%03d] %s(%d): "fmt, \
	wizpf_get_systick()/1000, wizpf_get_systick()%1000, __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#elif (WIZ_LOG_LEVEL > 2) && !defined(PRINT_TIME_LOG) && !defined(FILE_LOG_SILENCE)
#define DBG(fmt)  do { PRINTF("[D] %s(%d): "fmt"\r\n", __FUNCTION__, __LINE__); } while(0)
#define DBGA(fmt, ...)  do { PRINTF("[D] %s(%d): "fmt"\r\n", __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#define DBGF(fmt) do { PRINTF("[D] %s(%d): "fmt, __FUNCTION__, __LINE__); } while(0)
#define DBGFA(fmt, ...) do { PRINTF("[D] %s(%d): "fmt, __FUNCTION__, __LINE__, __VA_ARGS__); } while(0)
#else
#define DBG(fmt)
#define DBGA(fmt, ...)
#define DBGF(fmt)
#define DBGFA(fmt, ...)
#endif

#if (WIZ_LOG_LEVEL > 2) && !defined(FILE_LOG_SILENCE)
#define DBGCRTC(cond_v, fmt) do { if(cond_v) {ERR(fmt); while(1); } } while(0)
#define DBGCRTCA(cond_v, fmt, ...) do { if(cond_v) {ERRA(fmt, __VA_ARGS__); while(1); } } while(0)
#define DBGDUMP(data_p, len_v) print_dump(data_p, len_v)
#define DBGFUNC(func_p) func_p
#else
#define DBGCRTC(cond_v, fmt)
#define DBGCRTCA(cond_v, fmt, ...)
#define DBGDUMP(data_p, len_v)
#define DBGFUNC(func_p)
#endif

#if (WIZ_LOG_LEVEL > 0) && !defined(FILE_LOG_SILENCE)
#define NL1	PRINTF("\r\n")
#define NL2	PRINTF("\r\n\r\n")
#define NL3	PRINTF("\r\n\r\n\r\n")
#else
#define NL1
#define NL2
#define NL3
#endif




#endif

