/**************************************************************************
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
*  01.05.2018  mifi  First Version, tested with a FRDM-K64F board.
*  01.07.2023  mifi  Reworked for the NXP SDK v2.11.0.
**************************************************************************/
#define __ETH_CONFIG_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include "tal.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_phy.h"
#include "fsl_enet.h"

#include "device\phyksz8081\fsl_phyksz8081.h"
#include "mdio\enet\fsl_enet_mdio.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static mdio_handle_t mdioHandle;
static phy_handle_t phyHandle;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  GPIO_CONFIG                                                          */
/*                                                                       */
/*  Configures the GPIO ports.                                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void GPIO_CONFIG (void)
{
   CLOCK_EnableClock(kCLOCK_PortA);                   /* Port A Clock Gate Control: Clock enabled */
   CLOCK_EnableClock(kCLOCK_PortB);                   /* Port B Clock Gate Control: Clock enabled */
   CLOCK_EnableClock(kCLOCK_PortC);                   /* Port C Clock Gate Control: Clock enabled */

   PORT_SetPinMux(PORTA, 12, kPORT_MuxAlt4);          /* PORTA12 (pin 42) is configured as RMII0_RXD1 */
   PORT_SetPinMux(PORTA, 13, kPORT_MuxAlt4);          /* PORTA13 (pin 43) is configured as RMII0_RXD0 */
   PORT_SetPinMux(PORTA, 14, kPORT_MuxAlt4);          /* PORTA14 (pin 44) is configured as RMII0_CRS_DV */
   PORT_SetPinMux(PORTA, 15, kPORT_MuxAlt4);          /* PORTA15 (pin 45) is configured as RMII0_TXEN */
   PORT_SetPinMux(PORTA, 16, kPORT_MuxAlt4);          /* PORTA16 (pin 46) is configured as RMII0_TXD0 */
   PORT_SetPinMux(PORTA, 17, kPORT_MuxAlt4);          /* PORTA17 (pin 47) is configured as RMII0_TXD1 */
   PORT_SetPinMux(PORTA, 5,  kPORT_MuxAlt4);          /* PORTA5 (pin 39) is configured as RMII0_RXER */
   const port_pin_config_t portb0_pin53_config = {
      kPORT_PullUp,                                   /* Internal pull-up resistor is enabled */
      kPORT_FastSlewRate,                             /* Fast slew rate is configured */
      kPORT_PassiveFilterDisable,                     /* Passive filter is disabled */
      kPORT_OpenDrainEnable,                          /* Open drain is enabled */
      kPORT_LowDriveStrength,                         /* Low drive strength is configured */
      kPORT_MuxAlt4,                                  /* Pin is configured as RMII0_MDIO */
      kPORT_UnlockRegister                            /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTB, 0, &portb0_pin53_config); /* PORTB0 (pin 53) is configured as RMII0_MDIO */
   PORT_SetPinMux(PORTB, 1,  kPORT_MuxAlt4);          /* PORTB1 (pin 54) is configured as RMII0_MDC */
   PORT_SetPinMux(PORTC, 16, kPORT_MuxAlt4);          /* PORTC16 (pin 90) is configured as ENET0_1588_TMR0 */
   PORT_SetPinMux(PORTC, 17, kPORT_MuxAlt4);          /* PORTC17 (pin 91) is configured as ENET0_1588_TMR1 */
   PORT_SetPinMux(PORTC, 18, kPORT_MuxAlt4);          /* PORTC18 (pin 92) is configured as ENET0_1588_TMR2 */

} /* GPIO_CONFIG */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  BoardETHConfig                                                       */
/*                                                                       */
/*  Configures pin routing and optionally pin electrical features.       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void BoardETHConfig (void)
{
   GPIO_CONFIG();

} /* BoardETHConfig */

/*************************************************************************/
/*  BoardGetPhyHandle                                                    */
/*                                                                       */
/*  Return PHY handle depending of the iface.                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
struct _phy_handle *BoardGetPhyHandle (int iface)
{
   struct _phy_handle *pHandle = NULL;

   if (0 == iface)
   {
      mdioHandle.ops                  = &enet_ops;
      mdioHandle.resource.base        = ENET;
      mdioHandle.resource.csrClock_Hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);

      phyHandle.phyAddr    = 0;
      phyHandle.mdioHandle = &mdioHandle;
      phyHandle.ops        = &phyksz8081_ops;

      pHandle = &phyHandle;
   }

   return(pHandle);
} /* BoardGetPhyHandle */

/*** EOF ***/
