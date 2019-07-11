/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Bluetooth mesh light switch example
 *
 * This example implements a Bluetooth mesh light switch.
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

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

/* emLib and emDrv (HW drivers) specific includes */
#include <em_gpio.h>
#include <gpiointerrupt.h>

/* Bluetooth LE stack includes */
#include <native_gecko.h>
#include <gecko_configuration.h>

/* Bluetooth LE GATT database */
#include "gatt_db.h"

/* Bluetooth mesh stack includes */
#include "gecko_bgapi_mesh_node_native.h"
#include "gecko_bgapi_mesh_generic_client_native.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"

/* EFR32 hardware initialization */
#include "InitDevice.h"

// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

// heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + 8000];

/*
 * Maximum number of Bluetooth advertisement sets.
 * 1 is allocated for Bluetooth LE stack
 * 1 one for Bluetooth mesh stack
 * 1 needs to be allocated for each Bluetooth mesh network
 *   - Currently up to 4 networks are supported at a time
 */
#define MAX_ADVERTISERS (2 + 4)

// Bluetooth stack configuration
const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100,
  .gattdb = &bg_gattdb_data,
};

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

#define TIMER_ID_RESTART    78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_PROVISIONING   66
#define TIMER_ID_RETRANS    10

/**
 *  LCD content can be updated one row at a time using function LCD_write().
 *  Row number is passed as parameter,the possible values are defined below.
 */
#define LCD_ROW_NAME     1  /* 1st row, device name */
#define LCD_ROW_STATUS     2    /* 2nd row, node status */
#define LCD_ROW_CONNECTION 3    /* 3rd row, connection status */
#define LCD_ROW_AUTH_CODE 4
#define LCD_ROW_MAX        4    /* total number of rows used */

#define LCD_ROW_LEN        32   /* up to 32 characters per each row */

/** global variables */
static uint16 _my_index = 0xffff; /* Index of the Primary Element of the Node */
static uint16 _my_address = 0;    /* Address of the Primary Element of the Node */
static uint8 switch_pos = 0;      /* current position of the switch  */
static uint8 request_count;       /* number of on/off requests to be sent */
static uint8 trid = 0;        /* transaction identifier */
static char LCD_data[LCD_ROW_MAX][LCD_ROW_LEN];   /* 2D array for storing the LCD content */
static uint8 num_connections = 0;     /* number of active Bluetooth connections */
static uint8 conn_handle = 0xFF;      /* handle of the last opened LE connection */

/**
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 */
#define LED_STATE_OFF    0   /* light off (both LEDs turned off)   */
#define LED_STATE_ON     1   /* light on (both LEDs turned on)     */
#define LED_STATE_PROV   3   /* provisioning (LEDs blinking)       */

/**
 *  These are needed to support radio boards with active-low and
 *  active-high LED configuration
 */
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
#define LED_DEFAULT_STATE  1
#else
/* LED GPIO is active-high */
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_DEFAULT_STATE  0
#endif


/*Authentication code data*/

char temp[10];
   uint16 Auth_code;

/**
 * Update the state of LEDs. Takes one parameter LED_STATE_xxx that defines
 * the new state.
 */
static void LED_set_state(int state)
{
  switch (state) {
    case LED_STATE_OFF:
      TURN_LED_OFF(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_OFF(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_ON:
      TURN_LED_ON(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_ON(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_PROV:
      GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;

    default:
      break;
  }
}

#if (EMBER_AF_BOARD_TYPE == BRD4304A)
/**
 * LNA init. Required only for MGM12P radio board (BRD4304A)
 */
#define PRS_CH_CTRL_SIGSEL_RACLNAEN 0x00000003UL
#define PRS_CH_CTRL_SIGSEL_RACPAEN 0x00000004UL
#define PRS_CH_CTRL_SOURCESEL_RAC (0x00000051UL << 8)

static void LNA_init(void)
{
  GPIO_PinModeSet(gpioPortD, 10, gpioModePushPullAlternate, 0);
  GPIO_PinModeSet(gpioPortD, 11, gpioModePushPullAlternate, 0);

  PRS->CH[5].CTRL = PRS_CH_CTRL_SIGSEL_RACLNAEN
                    | PRS_CH_CTRL_SOURCESEL_RAC
                    | PRS_CH_CTRL_EDSEL_OFF;

  PRS->CH[6].CTRL = PRS_CH_CTRL_ORPREV
                    | PRS_CH_CTRL_SIGSEL_RACPAEN
                    | PRS_CH_CTRL_SOURCESEL_RAC
                    | PRS_CH_CTRL_EDSEL_OFF;

  PRS->ROUTELOC1 |= PRS_ROUTELOC1_CH5LOC_LOC0;  // loc0 -> PD10
  PRS->ROUTELOC1 |= PRS_ROUTELOC1_CH6LOC_LOC13; // loc12 -> PD11
  PRS->ROUTEPEN |= (PRS_ROUTEPEN_CH5PEN | PRS_ROUTEPEN_CH6PEN);
}
#endif

/**
 * This is a callback function that is invoked each time a GPIO interrupt in one of the pushbutton
 * inputs occurs. Pin number is passed as parameter.
 *
 * Note: this function is called from ISR context and therefore it is not possible to call any BGAPI
 * functions directly. The button state change is signaled to the application using gecko_external_signal()
 * that will generate an event gecko_evt_system_external_signal_id which is then handled in the main loop.
 */
void gpioint(uint8_t pin)
{
  if (pin == BSP_GPIO_PB0_PIN) {
    gecko_external_signal(0x1);
  } else if (pin == BSP_GPIO_PB1_PIN) {
    gecko_external_signal(0x2);
  }
}

/**
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger an interrupt on the
 * rising edge (button released).
 */
void enable_button_interrupts(void)
{
  GPIOINT_Init();

  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, true, false, true);
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN, true, false, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(BSP_GPIO_PB0_PIN, gpioint);
  GPIOINT_CallbackRegister(BSP_GPIO_PB1_PIN, gpioint);
}

/**
 * This function publishes one on/off request to change the state of light(s) in the group.
 * Global variable switch_pos holds the latest desired light state, possible values are
 * switch_pos = 1 -> PB1 was pressed, turn lights on
 * switch_pos = 0 -> PB0 was pressed, turn lights off
 *
 * This application sends multiple requests for each button press to improve reliability.
 * Parameter retrans indicates whether this is the first request or a re-transmission.
 * The transaction ID is not incremented in case of a re-transmission.
 */
void send_onoff_request(int retrans)
{
  uint16 resp;
  uint16 delay;
  struct mesh_generic_request req;

  req.kind = mesh_generic_request_on_off;
  req.on_off = switch_pos;

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    trid++;
  }

  /* delay for the request is calculated so that the last request will have a zero delay and each
   * of the previous request have delay that increases in 50 ms steps. For example, when using three
   * on/off requests per button press the delays are set as 100, 50, 0 ms
   */
  delay = (request_count - 1) * 50;

  resp = gecko_cmd_mesh_generic_client_publish(
    MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
    _my_index,
    trid,
    0,     // transition
    delay,
    0,     // flags
    mesh_generic_request_on_off,     // type
    1,     // param len
    &req.on_off     /// parameters data
    )->result;

//  resp = gecko_cmd_mesh_generic_client_set(
//    MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
//    _my_index, //client element index
//    0x0002,    //server address -- the light address
//    0,         //appkey index
//    trid,      // tid
//    0,     // transition
//    delay,  //delay time
//    1,     // flags
//    mesh_generic_request_on_off,     // type
//    1,     // param len
//    &req.on_off     /// parameters data
//    )->result;

  if (resp) {
    printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
  } else {
    printf("request sent, trid = %u, delay = %d\r\n", trid, delay);
  }

  /* keep track of how many requests has been sent */
  if (request_count > 0) {
    request_count--;
  }
}

/**
 * Handling of button presses. This function called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * parameter button defines which button was pressed, possible values
 * are 0 = PB0, 1 = PB1.
 *
 * This function is called from application context (not ISR) so it is safe to call BGAPI functions
 */
void handle_button_press(int button)
{
  // PB0 -> switch off, PB1 -> switch on
  switch_pos = button;

  printf("PB%d -> turn light(s) ", button);
  if (switch_pos) {
    printf("on\r\n");
  } else {
    printf("off\r\n");
  }
  request_count = 3; // request is sent 3 times to improve reliability

  /* send the first request */
  send_onoff_request(0);

  /* start a repeating soft timer to trigger re-transmission of the request after 50 ms delay */
  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50), TIMER_ID_RETRANS, 0);
}

/**
 * Switch node initialization. This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 */
void switch_node_init(void)
{
  mesh_lib_init(malloc, free, 8);
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

// this is needed by the LCD driver
int rtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency)
{
  return 0;
}

/**
 * button initialization. Configure pushbuttons PB0,PB1
 * as inputs.
 */
static void button_init()
{
  // configure pushbutton PB0 and PB1 as inputs
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, 1);
}

/**
 * LED initialization. Configure LED pins as outputs
 */
static void led_init()
{
  // configure LED0 and LED1 as outputs
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, LED_DEFAULT_STATE);
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, LED_DEFAULT_STATE);
}

/**
 * This function is used to write one line in the LCD. The parameter 'row' selects which line
 * is written, possible values are defined as LCD_ROW_xxx.
 */
void LCD_write(char *str, uint8 row)
{
  char LCD_message[LCD_ROW_MAX * LCD_ROW_LEN];
  char *pRow; /* pointer to selected row */
  int i;

  if (row > LCD_ROW_MAX) {
    return;
  }

  pRow  = &(LCD_data[row - 1][0]);

  sprintf(pRow, str);

  LCD_message[0] = 0;

  for (i = 0; i < LCD_ROW_MAX; i++) {
    pRow  = &(LCD_data[i][0]);
    strcat(LCD_message, pRow);
    strcat(LCD_message, "\n"); // add newline at end of reach row
  }

  graphWriteString(LCD_message);
}

/**
 * LCD initialization, called once at startup.
 */
void LCD_init(void)
{
  /* clear LCD_data table */
  memset(&LCD_data, 0, sizeof(LCD_data));

  /* initialize graphics driver and set the title text */
  graphInit("SILICON LABORATORIES\nBluetooth Mesh Demo\n\n");

  LCD_write("initializing", LCD_ROW_STATUS);
}

/**
 * Set device name in the GATT database. A unique name is genrerated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 */
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "switch node %x:%x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s'\r\n", name);

  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
  LCD_write(name, LCD_ROW_NAME);
}

/**
 *  this function is called to initiate factory reset. Factory reset may be initiated
 *  by keeping one of the WSTK pushbuttons pressed during reboot. Factory reset is also
 *  performed if it is requested by the provisioner (event gecko_evt_mesh_node_reset_id)
 */
void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  LCD_write("\n***\nFACTORY RESET\n***", LCD_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_endpoint_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

int main()
{
#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);
#endif /* FEATURE_SPI_FLASH */

  enter_DefaultMode_from_RESET();

#if (EMBER_AF_BOARD_TYPE == BRD4304A)
  LNA_init();
#endif

  gecko_init(&config);

#ifdef FEATURE_PTI_SUPPORT
  APP_ConfigEnablePti();
#endif // FEATURE_PTI_SUPPORT

  RETARGET_SerialInit();

  /* initialize LEDs and buttons. Note: some radio boards share the same GPIO for button & LED.
   * Initialization is done in this order so that default configuration will be "button" for those
   * radio boards with shared pins. led_init() is called later as needed to (re)initialize the LEDs
   * */
  led_init();
  button_init();

  LCD_init();

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
  }
}

/**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events are handled here.
 */
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  struct gecko_bgapi_mesh_node_cmd_packet *node_evt;
  struct gecko_msg_mesh_node_provisioning_failed_evt_t  *prov_fail_evt;

  if (NULL == evt) {
    return;
  }

  switch (evt_id) {
    case gecko_evt_system_boot_id:
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
      if (GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == 0 || GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN) == 0) {
        initiate_factory_reset();
      } else {
        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

        set_device_name(&pAddr->address);
        gecko_cmd_system_set_tx_power(-260);
        // Initialize Mesh stack in Node operation mode, wait for initialized event
        gecko_cmd_mesh_node_init_oob(0x00,0x02,0x03,0x04,0x04,0x04,0x01);

     //   gecko_cmd_mesh_node_init();


      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_RESTART:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_PROVISIONING:
          LED_set_state(LED_STATE_PROV);
          break;

        case TIMER_ID_RETRANS:
          send_onoff_request(1);   // param 1 indicates that this is a retransmission
          // stop retransmission timer if it was the last attempt
          if (request_count == 0) {
            gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_RETRANS, 0);
          }
          break;

        default:
          break;
      }

      break;

      case gecko_evt_mesh_node_display_output_oob_id:

        {
             struct gecko_msg_mesh_node_display_output_oob_evt_t *pOOB = (struct gecko_msg_mesh_node_display_output_oob_evt_t *)&(evt->data);
             printf("gecko_msg_mesh_node_display_output_oob_evt_t: action %d, size %d\r\n", pOOB->output_action, pOOB->output_size);
             for(int i=0;i<pOOB->data.len;i++)
             {
                 printf("%2.2x ", pOOB->data.data[i]);
             }
             Auth_code |= pOOB->data.data[14];
             Auth_code = Auth_code<<8;
             Auth_code |= pOOB->data.data[15];
             sprintf(temp, "Auth code %d",Auth_code);

             LCD_write(temp,LCD_ROW_AUTH_CODE);

         }
         break;


    case gecko_evt_mesh_node_initialized_id:
      printf("node initialized\r\n");

      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

      if (pData->provisioned) {
        printf("node is provisioned. address:%x, ivi:%d\r\n", pData->address, pData->ivi);

        _my_address = pData->address;
        _my_index = 0;   // index of primary element hardcoded to zero in this example

        enable_button_interrupts();
        switch_node_init();

        LCD_write("provisioned", LCD_ROW_STATUS);
      } else {
        printf("node is unprovisioned\r\n");
        LCD_write("unprovisioned", LCD_ROW_STATUS);

        printf("starting unprovisioned beaconing...\r\n");
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
      }
      break;

    case gecko_evt_system_external_signal_id:
    {
      if (evt->data.evt_system_external_signal.extsignals & 0x1) {
        handle_button_press(0);
      }
      if (evt->data.evt_system_external_signal.extsignals & 0x2) {
        handle_button_press(1);
      }
    }
    break;

    case gecko_evt_mesh_node_provisioning_started_id:
      printf("Started provisioning\r\n");
      LCD_write("provisioning...", LCD_ROW_STATUS);
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      led_init(); /* shared GPIO pins used as LED output */
#endif
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
      _my_index = 0;   // index of primary element hardcoded to zero in this example
      switch_node_init();
      printf("node provisioned, got index=%x\r\n", _my_index);
      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PROVISIONING, 0);
      LED_set_state(LED_STATE_OFF);
      LCD_write("provisioned", LCD_ROW_STATUS);

#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      button_init(); /* shared GPIO pins used as button input */
#endif
      enable_button_interrupts();
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      prov_fail_evt = (struct gecko_msg_mesh_node_provisioning_failed_evt_t  *)&(evt->data);
      printf("provisioning failed, code %x\r\n", prov_fail_evt->result);
      LCD_write("prov failed", LCD_ROW_STATUS);
      /* start a one-shot timer that will trigger soft reset after small delay */
      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
      break;

    case gecko_evt_mesh_node_key_added_id:
      node_evt = (struct gecko_bgapi_mesh_node_cmd_packet *)evt;
      printf("got new %s key with index %x\r\n", node_evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
             node_evt->data.evt_mesh_node_key_added.index);
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      printf("model config changed\r\n");
      break;

    case gecko_evt_le_connection_bt5_opened_id:
      printf("evt:gecko_evt_le_connection_bt5_opened_id\r\n");
      num_connections++;
      conn_handle = evt->data.evt_le_connection_bt5_opened.connection;
      LCD_write("connected", LCD_ROW_CONNECTION);
      break;

    case gecko_evt_le_connection_closed_id:
      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          LCD_write("", LCD_ROW_CONNECTION);
        }
      }
      break;

    case gecko_evt_mesh_node_reset_id:
      printf("evt gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    case gecko_evt_le_gap_adv_timeout_id:
    case gecko_evt_le_gap_bt5_adv_timeout_id:
    case gecko_evt_le_connection_parameters_id:
      // these events silently discarded
      break;

    default:
      printf("unhandled evt: %x\r\n", evt_id);
      break;
  }
}
