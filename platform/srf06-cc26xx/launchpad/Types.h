/*
*
@file		type.h
*
*/

#ifndef _TYPE_H_
#define _TYPE_H_


/***************************************************
 * attribute for mcu ( types, ... ) 
 ***************************************************/

#define	MAX_SOCK_NUM		8	/**< Maxmium number of socket  */
//#define __DEF_IINCHIP_DBG__
//#define __DEF_IINCHIP_INT__  // Interrupt disable


/**
@brief	 __DEF_IINCHIP_MAP_xxx__ : define memory map for iinchip 
*/
#define __DEF_IINCHIP_MAP_BASE__ 0x0000
    #define COMMON_BASE 0x0000
#define __DEF_IINCHIP_MAP_TXBUF__ (COMMON_BASE + 0x8000) /* Internal Tx buffer address of the iinchip */
#define __DEF_IINCHIP_MAP_RXBUF__ (COMMON_BASE + 0xC000) /* Internal Rx buffer address of the iinchip */
#define __DEF_IINCHIP_PPP

#ifndef NULL
#define NULL		((void *) 0)
#endif

//typedef enum { false, true } bool;

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif

/*
 * The 8-bit signed data type.
 */
typedef signed char int8;
/**
 * The volatile 8-bit signed data type.
 */
typedef volatile char vint8;
/**
 * The 8-bit unsigned data type.
 */
typedef unsigned char uint8;
/**
 * The volatile 8-bit unsigned data type.
 */
typedef volatile unsigned char vuint8;

/**
 * The 16-bit signed data type.
 */
typedef signed short int16;
/**
 * The volatile 16-bit signed data type.
 */
//typedef volatile int vint16;
/**
 * The 16-bit unsigned data type.
 */
typedef unsigned short uint16;
/**
 * The volatile 16-bit unsigned data type.
 */
//typedef volatile unsigned int vuint16;
/**
 * The 32-bit signed data type.
 */
typedef signed long int32;
/**
 * The volatile 32-bit signed data type.
 */
//typedef volatile long vint32;
/**
 * The 32-bit unsigned data type.
 */
typedef unsigned long uint32;
/**
 * The volatile 32-bit unsigned data type.
 */
//typedef volatile unsigned long vuint32;

/* bsd */
//typedef uint8			u_char;		/**< 8-bit value */
typedef uint8 			SOCKET;
//typedef uint16			u_short;	/**< 16-bit value */
//typedef uint16			u_int;		/**< 16-bit value */
//typedef uint32			u_long;		/**< 32-bit value */

typedef union _un_l2cval {
	uint32	lVal;
	uint8	cVal[4];
}un_l2cval;

typedef union _un_i2cval {
	uint16	iVal;
	uint8	cVal[2];
}un_i2cval;




#endif		/* _TYPE_H_ */
