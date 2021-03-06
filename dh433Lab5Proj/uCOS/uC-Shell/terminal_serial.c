/*
*********************************************************************************************************
*                                               uC/Shell
*                                            Shell utility
*
*                           (c) Copyright 2007-2013; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                              TERMINAL
*
*                                    COMMUNICATIONS PORT
*
* Filename      : terminal_serial.c
* Version       : V1.03.01
* Programmer(s) : BAN
* Modified from Template by: Todd Morton, 04/11/2017
* Uses BasicIO functions for the serial IO
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "terminal.h"
#include  "BasicIO.h"
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                        TerminalSerial_Init()
*
* Description : Initialize serial communications.
*
* Argument(s) : none.
*
* Return(s)   : DEF_OK,   if interface was opened.
*               DEF_FAIL, otherwise.
*
* Caller(s)   : Terminal_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_BOOLEAN  TerminalSerial_Init (void){
    BIOOpen(BIO_BIT_RATE_115200);
    return (DEF_OK);
}


/*
*********************************************************************************************************
*                                        TerminalSerial_Exit()
*
* Description : Uninitialize serial communications.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Terminal_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  TerminalSerial_Exit (void)
{
    //Still to be done.
}
/*
*********************************************************************************************************
*                                         TerminalSerial_Wr()
*
* Description : Serial output.
*
* Argument(s) : pbuf        Pointer to the buffer to transmit.
*
*               buf_len     Number of bytes in the buffer.
*
* Return(s)   : none.
*
* Caller(s)   : Terminal_Out().
*
* Note(s)     : Currently this only outputs null terminated strings. TDM 04/11/2017
*********************************************************************************************************
*/

CPU_INT16S  TerminalSerial_Wr (void *pbuf, CPU_SIZE_T buf_len){
    INT8C *bufptr = pbuf;
    (void)buf_len;
    while (*bufptr != '\0'){              //until a null is reached
        BIOWrite(*bufptr);
        bufptr++;
    }
    return (0u);
}


/*
*********************************************************************************************************
*                                       TerminalSerial_RdByte()
*
* Description : Serial byte input.
*
* Argument(s) : none.
*
* Return(s)   : Byte read from port.
*
* Caller(s)   : various.
*
* Note(s)     : Reads only ASCII chars right now. TDM
*********************************************************************************************************
*/

CPU_INT08U TerminalSerial_RdByte(void){
    INT8C rd_char;
    rd_char = BIORead();
    return((CPU_INT08U)rd_char);
}


/*
*********************************************************************************************************
*                                       TerminalSerial_WrByte()
*
* Description : Serial byte output.
*
* Argument(s) : c           Byte to write.
*
* Return(s)   : none.
*
* Caller(s)   : various.
*
* Note(s)     : Only writes ASCII characters. TDM
*********************************************************************************************************
*/

void TerminalSerial_WrByte (CPU_INT08U  c){
    BIOWrite((INT8C)c);
}
