/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2018-2023 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
***************************************************************************
*  History:
*
*  16.03.2018  mifi  First Version, tested with the FRDM-K64F board.
**************************************************************************/
#if defined(USE_BOARD_FRDMK64F)
#define __TALBOARD_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include "tal.h"

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_sim.h"

#if defined(TAL_ENABLE_ETH)
#include "crypto.h"
#include "ipstack_conf.h"
#endif

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

#if defined(TAL_ENABLE_ETH)
static MD5_CTX MD5ctx;
static uint8_t MD5digit[MD5_SIZE];
#endif

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_BoardEnableCOMx                                                  */
/*                                                                       */
/*  Board and hardware depending functionality to enable the COM port.   */
/*  If this port is not supported by the board, return an error.         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_BoardEnableCOM1 (void)
{
   /* Enable PORTB and UART0 clock */
   CLOCK_EnableClock(kCLOCK_Uart0);
   CLOCK_EnableClock(kCLOCK_PortB);

   /* Config PTB16/ PTB17 as UART0 */
   PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3); /* UART0_RX */
   PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3); /* UART0_TX */

   SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART0TXSRC_MASK)))    /* Mask bits to zero which are setting */
      | SIM_SOPT5_UART0TXSRC(0)        /* UART 0 transmit data source select: UART0_TX pin */
    );

   return(TAL_OK);
} /* tal_BoardEnableCOM1 */

TAL_RESULT tal_BoardEnableCOM2 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM2 */

TAL_RESULT tal_BoardEnableCOM3 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM3 */

TAL_RESULT tal_BoardEnableCOM4 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM4 */

TAL_RESULT tal_BoardEnableCOM5 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM5 */

TAL_RESULT tal_BoardEnableCOM6 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM6 */

/*************************************************************************/
/*  tal_BoardEnableCANx                                                  */
/*                                                                       */
/*  Board and hardware depending functionality to enable the CAN port.   */
/*  If this port is not supported by the board, return an error.         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_BoardEnableCAN1 (void)
{
   return(TAL_ERR_CAN_PORT_NOHW);
} /* tal_BoardEnableCAN1 */

#if defined(TAL_ENABLE_ETH)
/*************************************************************************/
/*  tal_BoardGetMACAddress                                               */
/*                                                                       */
/*  Retrieve the MAC address of the board.                               */
/*  In case of an error, a default address will be used.                 */
/*                                                                       */
/*  In    : pAddress                                                     */
/*  Out   : pAddress                                                     */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT tal_BoardGetMACAddress (int iface, uint8_t *pAddress)
{
   TAL_RESULT  Error = TAL_OK;

   (void)iface;

#if !defined(USE_IP_DEFAULT_MAC_ADDR)
   sim_uid_t uid;
   uint8_t  *msg;

   Error = TAL_OK;
   
   /*
    * Calculating the MAC address with the 
    * help of UniqueId and the MD5 hash.   
    */
   SIM_GetUniqueId(&uid);
   
   msg = (uint8_t*)&uid;
   MD5_Init(&MD5ctx);
   MD5_Update(&MD5ctx, msg, sizeof(sim_uid_t));
   MD5_Final(MD5digit, &MD5ctx);

   pAddress[0] = 0x00;
   pAddress[1] = 0x04;
   pAddress[2] = 0x9f;
   pAddress[3] = MD5digit[1];
   pAddress[4] = MD5digit[3];
   pAddress[5] = MD5digit[7];
#else
   uint8_t  MACAddress[6] = IP_DEFAULT_MAC_ADDR;

   Error = TAL_OK;

   pAddress[0] = MACAddress[0];
   pAddress[1] = MACAddress[1];
   pAddress[2] = MACAddress[2];
   pAddress[3] = MACAddress[3];
   pAddress[4] = MACAddress[4];
   pAddress[5] = MACAddress[5];
#endif

   return(Error);
} /* tal_BoardGetMACAddress */
#endif /* defined(TAL_ENABLE_ETH) */

/*************************************************************************/
/*  tal_BoardRTCSetTM                                                    */
/*                                                                       */
/*  Set the onboard RTC time by TM                                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTCSetTM (struct tm *pTM)
{
   (void)pTM;
} /* tal_BoardRTCSetTM */

/*************************************************************************/
/*  tal_BoardRTCSetUnixtime                                              */
/*                                                                       */
/*  Set the onboard RTC time by Unixtime                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTCSetUnixtime (uint32_t Unixtime)
{
   (void)Unixtime;
} /* tal_BoardRTCSetUnixtime */

/*************************************************************************/
/*  tal_BoardRTC2System                                                  */
/*                                                                       */
/*  Set the system time from the RTC.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTC2System (void)
{
} /* tal_BoardRTC2System */

#endif /* USE_BOARD_FRDMK64F */

/*** EOF ***/
