/*
 * timer.c
 *
 *  Created on: Dec 13, 2017
 *      Author: Srikant
 */


/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* WSTK specific includes */
#include "boards.h"
#include "btMesh-configuration.h"
#include "board_features.h"
#include "retargetserial.h"
#include "graphics.h"

/* Silicon Labs radio board specific includes */
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Included if the Silicon Labs radio board has a SPI flash */
#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

/* emLib (HW drivers) specific includes */
#include <em_gpio.h>

/* Bluetooth LE stack includes */
#include <native_gecko.h>
#include <gecko_configuration.h>

/* Bluetooth LE GATT database */
#include "gatt_db.h"

/* Bluetooth mesh stack includes */
#include "gecko_bgapi_mesh_node_native.h"
#include "mesh_generic_model_capi_types.h"
#include "gecko_bgapi_mesh_generic_server_native.h"
#include "mesh_lib.h"

/* EFR32 hardware initialization */
#include "InitDevice.h"

#include "em_timer.h"

#include "em_cmu.h"

#include "em_gpio.h"
#include "timer.h"

//#define LED_PORT (gpioPortD)
//#define LED_PIN   (2)


#define TIMER_TOP                   100
#define DUTY_CYCLE                  1
#define TIMER_CHANNEL               2
#define PIN_LOC

void timer_init()
{

     CMU_ClockEnable(cmuClock_GPIO, true);
     CMU_ClockEnable(cmuClock_TIMER0, true);

      // Enable LED output
     // GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

      // Create the timer count control object initializer
      TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
      timerCCInit.mode = timerCCModePWM;
      timerCCInit.cmoa = timerOutputActionToggle;

      // Configure CC channel 2
      TIMER_InitCC(TIMER0, TIMER_CHANNEL, &timerCCInit);

//      TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0&(~_TIMER_ROUTELOC0_CC2LOC_MASK))| (TIMER_ROUTELOC0_CC2LOC_LOC16);
//      // Route CC2 to location 1 (PE3) and enable pin for cc2
//      TIMER0->ROUTEPEN = TIMER0->ROUTEPEN | TIMER_ROUTEPEN_CC2PEN;

      // Set Top Value
      TIMER_TopSet(TIMER0, TIMER_TOP);

      // Set the PWM duty cycle here!
      TIMER_CompareBufSet(TIMER0, TIMER_CHANNEL, DUTY_CYCLE);

      // Create a timerInit object, based on the API default
      TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
      timerInit.prescale = timerPrescale256;

      TIMER_Init(TIMER0, &timerInit);

TIMER0->CMD = TIMER_CMD_STOP;

}
