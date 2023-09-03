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
#define __TALCPU_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdlib.h>
#include "tal.h"

#include "fsl_clock.h"
#include "fsl_sysmpu.h"
#include "fsl_wdog.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define MCG_PLL_DISABLE                0u    /* MCGPLLCLK disabled */
#define OSC_CAP0P                      0u    /* Oscillator 0pF capacitor load */
#define SIM_OSC32KSEL_RTC32KCLK_CLK    2u    /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
#define SIM_PLLFLLSEL_MCGPLLCLK_CLK    1u    /* PLLFLL select: MCGPLLCLK clock */

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

const osc_config_t oscConfig_BOARD_BootClockRUN =
{
   .freq        = 50000000U,                 /* Oscillator frequency: 50000000Hz */
   .capLoad     = (OSC_CAP0P),               /* Oscillator capacity load: 0pF */
   .workMode    = kOSC_ModeExt,              /* Use external clock */
   .oscerConfig =
   {
      .enableMode = kOSC_ErClkEnable,        /* Enable external reference clock, disable external reference clock in STOP mode */
   }
};

const mcg_config_t mcgConfig_BOARD_BootClockRUN =
{
   .mcgMode         = kMCG_ModePEE,          /* PEE - PLL Engaged External */
   .irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
   .ircs            = kMCG_IrcSlow,          /* Slow internal reference clock selected */
   .fcrdiv          = 0x0U,                  /* Fast IRC divider: divided by 1 */
   .frdiv           = 0x0U,                  /* FLL reference clock divider: divided by 32 */
   .drs             = kMCG_DrsLow,           /* Low frequency range */
   .dmx32           = kMCG_Dmx32Default,     /* DCO has a default range of 25% */
   .oscsel          = kMCG_OscselOsc,        /* Selects System Oscillator (OSCCLK) */
   .pll0Config =
   {
      .enableMode = MCG_PLL_DISABLE,         /* MCGPLLCLK disabled */
      .prdiv      = 0x13U,                   /* PLL Reference divider: divided by 20 */
      .vdiv       = 0x18U,                   /* VCO divider: multiplied by 48 */
   },
};

const sim_clock_config_t simConfig_BOARD_BootClockRUN =
{
   .pllFllSel = SIM_PLLFLLSEL_MCGPLLCLK_CLK, /* PLLFLL select: MCGPLLCLK clock */
   .er32kSrc  = SIM_OSC32KSEL_RTC32KCLK_CLK, /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
   .clkdiv1   = 0x1240000U,                  /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV3: /3, OUTDIV4: /5 */
};

static uint32_t dHiResPeriod     = 0;
static uint32_t dHiResPeriodHalf = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  NewSysTick_Config                                                    */
/*                                                                       */
/*  Based on the "core_cm4.h" version, but an AHB clock divided by 8 is  */
/*  used for the SysTick clock source.                                   */
/*                                                                       */
/*  The function initializes the System Timer and its interrupt, and     */
/*  starts the System Tick Timer. Counter is in free running mode to     */
/*  generate periodic interrupts.                                        */
/*                                                                       */
/*  In    : ticks                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static uint32_t NewSysTick_Config (uint32_t ticks)
{
   /* Reload value impossible */
   if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk)  return (1);

   /* Set reload register */
   SysTick->LOAD = ticks - 1;

   /* Set Priority for Systick Interrupt */
   NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIO);

   /* Load the SysTick Counter Value */
   SysTick->VAL = 0;

   /*
    * SysTick IRQ and SysTick Timer must be
    * enabled with tal_CPUSysTickStart later.
    */

   return(ticks);
} /* NewSysTick_Config */

/*************************************************************************/
/*  CLOCK_CONFIG_SetFllExtRefDiv                                         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void CLOCK_CONFIG_SetFllExtRefDiv (uint8_t frdiv)
{
    MCG->C1 = ((MCG->C1 & ~MCG_C1_FRDIV_MASK) | MCG_C1_FRDIV(frdiv));
} /* CLOCK_CONFIG_SetFllExtRefDiv */

/*************************************************************************/
/*  BootClockRUN                                                         */
/*                                                                       */
/*  Set clock RUN mode.                                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void BootClockRUN (void)
{
   /* Set the system clock dividers in SIM to safe value. */
   CLOCK_SetSimSafeDivs();

   /* Initializes OSC0 according to board configuration. */
   CLOCK_InitOsc0(&oscConfig_BOARD_BootClockRUN);
   CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockRUN.freq);

   /* Configure the Internal Reference clock (MCGIRCLK). */
   CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockRUN.irclkEnableMode,
                                 mcgConfig_BOARD_BootClockRUN.ircs,
                                 mcgConfig_BOARD_BootClockRUN.fcrdiv);

   /* Configure FLL external reference divider (FRDIV). */
   CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_BootClockRUN.frdiv);

   /* Set MCG to PEE mode. */
   CLOCK_BootToPeeMode(mcgConfig_BOARD_BootClockRUN.oscsel,
                       kMCG_PllClkSelPll0,
                       &mcgConfig_BOARD_BootClockRUN.pll0Config);

   /* Set the clock configuration in SIM module. */
   CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
} /* BootClockRUN */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_CPUInit                                                          */
/*                                                                       */
/*  "Initialize" the CPU.                                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUInit (void)
{
   /* Set clock RUN mode after boot */
   BootClockRUN();
   SYSMPU_Enable(SYSMPU, 0);  /*lint !e747*/

   /* Update clock info */
   SystemCoreClockUpdate();

   /* Init SysTick */
   dHiResPeriod = NewSysTick_Config(SystemCoreClock / OS_TICKS_PER_SECOND);

   /*
    * dHiResPeriod value must be a 16bit count, but here it is
    * bigger. Therefore dHiResPeriod must be divided by 2.
    */
   dHiResPeriodHalf = dHiResPeriod / 2;

   /* Configure Priority Grouping (core_cm4.h) */
   NVIC_SetPriorityGrouping(7);
} /* tal_CPUInit */

/*************************************************************************/
/*  tal_CPUSysTickStart                                                  */
/*                                                                       */
/*  Start the SysTick.                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUSysTickStart (void)
{
   /* Enable SysTick IRQ and SysTick Timer */
   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
} /* tal_CPUSysTickStart */

/*************************************************************************/
/*  tal_CPUIrqEnable                                                     */
/*                                                                       */
/*  Enable the given IRQ.                                                */
/*                                                                       */
/*  In    : IRQ                                                          */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqEnable (int IRQ)
{
   NVIC_EnableIRQ((IRQn_Type)IRQ);
} /* tal_CPUIrqEnable */

/*************************************************************************/
/*  tal_CPUIrqDisable                                                    */
/*                                                                       */
/*  Disable the given IRQ.                                               */
/*                                                                       */
/*  In    : IRQ                                                          */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqDisable (int IRQ)
{
   NVIC_DisableIRQ((IRQn_Type)IRQ);
} /* tal_CPUIrqDisable */

/*************************************************************************/
/*  tal_CPUIrqDisableAll                                                 */
/*                                                                       */
/*  Disable all interrupts.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqDisableAll (void)
{
   /*lint +rw(_to_semi) */
   /*lint -d__disable_irq=_to_semi */

   __disable_irq();

} /* tal_CPUIrqDisableAll */

/*************************************************************************/
/*  tal_CPUIrqSetPriority                                                */
/*                                                                       */
/*  Set priority of the given IRQ.                                       */
/*                                                                       */
/*  In    : IRQ, Priority                                                */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqSetPriority (int IRQ, int Priority)
{
   NVIC_SetPriority((IRQn_Type)IRQ, (uint32_t)Priority);
} /* tal_CPUIrqSetPriority */

/*************************************************************************/
/*  tal_CPUStatGetHiResPeriod                                            */
/*                                                                       */
/*  Return the HiResPeriod value.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: HiResPeriod                                                  */
/*************************************************************************/
uint32_t tal_CPUStatGetHiResPeriod (void)
{
   return(dHiResPeriodHalf);
} /* tal_CPUStatGetHiResPeriod */

/*************************************************************************/
/*  tal_CPUStatGetHiResCnt                                               */
/*                                                                       */
/*  Return the HiRes counter.                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: HiRes counter                                                */
/*************************************************************************/
uint32_t tal_CPUStatGetHiResCnt (void)
{
   uint32_t dValue;

   /* Get milliseconds */
   dValue  = (OS_TimeGet() << 16);

   /* The SysTick counts down from HiResPeriod, therefore HiResPeriod - X */

   /* dHiResPeriodHalf is used, therefore divide the time by 2 too */
   dValue |= (uint16_t)((dHiResPeriod - SysTick->VAL) / 2);

   return(dValue);
} /* tal_CPUStatGetHiResCnt */

/*************************************************************************/
/*  tal_CPUGetFrequencyCPU                                               */
/*                                                                       */
/*  Return the clock frequency of the CPU in MHz.                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Frequency                                                    */
/*************************************************************************/
uint32_t tal_CPUGetFrequencyCPU (void)
{
   return( CLOCK_GetFreq(kCLOCK_CoreSysClk) );
} /* tal_CPUGetFrequencyCPU */

uint32_t tal_CPUGetFrequencyBUS (void)
{
   return( CLOCK_GetFreq(kCLOCK_BusClk) );
}

uint32_t tal_CPUGetFrequencyOSC (void)
{
   return( CLOCK_GetFreq(kCLOCK_Osc0ErClk) );
}

uint32_t tal_CPUGetFrequencyLPO (void)
{
   return( CLOCK_GetFreq(kCLOCK_LpoClk) );
}

/*************************************************************************/
/*  tal_CPUInitHWDog                                                     */
/*                                                                       */
/*  Initialize the Hardware Watchdog.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUInitHWDog (void)
{
   wdog_config_t   config;
   static uint8_t bInitDone = 0;

   if (0 == bInitDone)
   {
      bInitDone = 1;

      WDOG_GetDefaultConfig(&config);
      /* Set timeout to 1 second */
      config.timeoutValue = tal_CPUGetFrequencyLPO() * 1;
      WDOG_Init(WDOG, &config);
   }

} /* tal_CPUInitHWDog */

/*************************************************************************/
/*  Name  : tal_CPUTriggerHWDog                                          */
/*                                                                       */
/*  Trigger the Hardware Watchdog here.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUTriggerHWDog (void)
{
   WDOG_Refresh(WDOG);
} /* tal_CPUTriggerHWDog */

/*************************************************************************/
/*  tal_CPUReboot                                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUReboot (void)
{
#if defined(__DEBUG__)
   term_printf("\r\n*** Reboot ***\r\n");
   OS_TimeDly(500);
#endif

   /*
    * Init watchdog, if not done before
    */
#if defined(__FLASH__) || defined(ENABLED_WDOG)
   tal_CPUInitHWDog();
#endif

   /*
    * Wait for watchdog reset
    */
   TAL_CPU_DISABLE_ALL_INTS();
   while (1)
   {
      __asm__ ("nop");
   }
   TAL_CPU_ENABLE_ALL_INTS(); /*lint !e527*/

} /* tal_CPUReboot */

/*************************************************************************/
/*  SysTick_Handler                                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void SysTick_Handler (void)
{
   TAL_CPU_IRQ_ENTER();

   OS_TimerCallback();

   TAL_CPU_IRQ_EXIT();
} /* SysTick_Handler */

/*** EOF ***/
