/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
// Board/nrf6310/ble/ble_app_hrs/main.c
/** @example Board/nrf6310/ble/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h" // Sito faio truksta!!
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "ble_sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "ble_bondmngr.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"

#include <stdbool.h>
#include <stdint.h>
#include "twi_master.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"


///////////////////////////////////////////////PINOUT REF TO ZB-CA1008
#define BUTTON_W        29// WAKEUP/LOCK BUTTON

#define TOUCH_INT       7

//#define ACCEL_DEN_G       12
#define ACCEL_DRDY_INT2_G       11
#define ACCEL_INT1_G        10
#define ACCEL_INT1_A        8
#define ACCEL_INT2_A        9

#define OLED_RST        14

#define BUZZER      17
#define BUZZER2      18

#define BAT_STATUS      28
#define BAT_LVL      //AIN2

#define I2C_SDA     16
#define I2C_SCL     15
///////////////////////////////////////////////PINOUT-END

#define DEVICE_NAME                          "ZEBE-1009"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "COOLDUDES"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                     40                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 5                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL             APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                       140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                 10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                 APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                      100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                      500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT                1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL     APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)           /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define FLASH_PAGE_SYS_ATTR                 (PSTORAGE_FLASH_PAGE_END - 3)                    /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                     (PSTORAGE_FLASH_PAGE_END - 1)                    /**< Flash page used for bond manager bonding information. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//////////////////////////////////////
//I2C DEFINES BELOW

#define ADDR_ACCEL_W                           0x30  
#define ADDR_ACCEL_R                           0x31  

#define ADDR_GYRO_W                            0xD4 
#define ADDR_GYRO_R                            0xD5

#define ADDR_OLED_W                          0x7A
#define ADDR_OLED_R                          0x7B
//    R/W bit:    0-write   1-read

#define ADDR_EEPROM_W                        0xA6
#define ADDR_EEPROM_R                        0xA7

#define ADDR_TOUCH_W                           0xB6 ///DDD 
#define ADDR_TOUCH_R                           0xB7 ///DDD 

//ssd1306 oled controller defines

#define OLED_WIDTH                  64
#define OLED_HEIGHT                 48

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A


//display items
#define img_bt_advertise 1
#define img_bt_connected 2
#define img_bt_disconnected 3
#define img_init 4



// MPR121 Register Defines
#define MHD_R	0x2B
#define NHD_R	0x2C
#define	NCL_R 	0x2D
#define	FDL_R	0x2E
#define	MHD_F	0x2F
#define	NHD_F	0x30
#define	NCL_F	0x31
#define	FDL_F	0x32
#define	ELE0_T	0x41
#define	ELE0_R	0x42
#define	ELE1_T	0x43
#define	ELE1_R	0x44
#define	ELE2_T	0x45
#define	ELE2_R	0x46
#define	ELE3_T	0x47
#define	ELE3_R	0x48
#define	ELE4_T	0x49
#define	ELE4_R	0x4A
#define	ELE5_T	0x4B
#define	ELE5_R	0x4C
#define	ELE6_T	0x4D
#define	ELE6_R	0x4E
#define	ELE7_T	0x4F
#define	ELE7_R	0x50
#define	ELE8_T	0x51
#define	ELE8_R	0x52
#define	ELE9_T	0x53
#define	ELE9_R	0x54
#define	ELE10_T	0x55
#define	ELE10_R	0x56
#define	ELE11_T	0x57
#define	ELE11_R	0x58
#define	FIL_CFG	0x5D
#define	ELE_CFG	0x5E
#define GPIO_CTRL0	0x73
#define	GPIO_CTRL1	0x74
#define GPIO_DATA	0x75
#define	GPIO_DIR	0x76
#define	GPIO_EN		0x77
#define	GPIO_SET	0x78
#define	GPIO_CLEAR	0x79
#define	GPIO_TOGGLE	0x7A
#define	ATO_CFG0	0x7B
#define	ATO_CFGU	0x7D
#define	ATO_CFGL	0x7E
#define	ATO_CFGT	0x7F

// Global Constants
#define TOU_THRESH	50
#define	REL_THRESH	15


static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_gap_sec_params_t                  m_sec_params;                              /**< Security requirements for this application. */
static ble_gap_adv_params_t                  m_adv_params;                              /**< Parameters to be passed to the stack when starting advertising. */
static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
static ble_hrs_t                             m_hrs;                                     /**< Structure used to identify the heart rate service. */
static bool                                  m_rr_interval_enabled = true;              /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static ble_sensorsim_cfg_t                   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t                 m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static ble_sensorsim_cfg_t                   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static ble_sensorsim_state_t                 m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static ble_sensorsim_cfg_t                   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static ble_sensorsim_state_t                 m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

static app_timer_id_t                        m_battery_timer_id;                        /**< Battery timer. */
static app_timer_id_t                        m_heart_rate_timer_id;                     /**< Heart rate measurement timer. */
static app_timer_id_t                        m_rr_interval_timer_id;                    /**< RR interval timer. */
static app_timer_id_t                        m_sensor_contact_timer_id;                 /**< Sensor contact detected timer. */


//oled screen size is 64x48
//oled buffer arrays below

static uint8_t test;

static uint8_t buffer_ff[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q5[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q6[16]  = {0x00, 0x01, 0xC3, 0x87, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q9[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC3, 0x87, 0x0F,};
static uint8_t buffer_bt_q10[16]  = {0x00, 0x00, 0x0F, 0x87, 0xC3, 0xE0, 0xF0, 0xF8, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q11[16]  = {0xFF, 0xFF, 0x3F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q12[16]  = {0xFF, 0xFF, 0x3F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q13[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, 0xF8,};
static uint8_t buffer_bt_q14[16]  = {0x00, 0x00, 0xF8, 0xF0, 0x61, 0x03, 0x07, 0x0F, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q15[16]  = {0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q16[16]  = {0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q18[16]  = {0x80, 0xC0, 0xE1, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer_bt_q20[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer10on[16]  = {0x0F, 0x03, 0x01, 0xF0, 0xFC, 0xFC, 0xF8, 0xE0, 0x01, 0x07, 0x3F, 0xFF, 0x01, 0x00, 0x00, 0x01,};
static uint8_t buffer11on[16]  = {0x0F, 0x3F, 0xFF, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};
static uint8_t buffer14on[16]  = {0xF0, 0xC0, 0x80, 0x0F, 0x3F, 0x3F, 0x1F, 0x07, 0x80, 0xE0, 0xF8, 0xFF, 0x80, 0x00, 0x00, 0xFF,};
static uint8_t buffer15on[16]  = {0xFC, 0xF8, 0xE0, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,};


/* Max cycles approximately to wait on RXDREADY and TXDREADY event, this is optimum way instead of using timers, this is not power aware, negetive side is this is not power aware */
#define MAX_TIMEOUT_LOOPS             (10000UL)        /*!< MAX while loops to wait for RXD/TXD event */





static bool twi_master_write(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
    uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for EVENTS_TXDSENT event*/

    if (data_length == 0)
    {
        /* gently return false for requesting data of size 0 */
        return false;
    }

    NRF_TWI1->TXD = *data++;
    NRF_TWI1->TASKS_STARTTX = 1;

    while (true)
    {
        while(NRF_TWI1->EVENTS_TXDSENT == 0 && (--timeout))
        {
        }

        if (timeout == 0)
        {
            NRF_TWI1->EVENTS_STOPPED = 0; 
            NRF_TWI1->TASKS_STOP = 1; 
            /* wait until stop sequence is sent and clear the EVENTS_STOPPED */ 
            while(NRF_TWI1->EVENTS_STOPPED == 0) 
            { 
            }
            /* timeout before receiving event*/
            return false;
        }

        NRF_TWI1->EVENTS_TXDSENT = 0;
        if (--data_length == 0)
        {
            break;
        }

        NRF_TWI1->TXD = *data++;
    }
    
    if (issue_stop_condition) 
    { 
        NRF_TWI1->EVENTS_STOPPED = 0; 
        NRF_TWI1->TASKS_STOP = 1; 
        /* wait until stop sequence is sent and clear the EVENTS_STOPPED */ 
        while(NRF_TWI1->EVENTS_STOPPED == 0) 
        { 
        } 
    }

    return true;
}


/**
 * Detects stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi_master_clear_bus(void)
{
    uint32_t twi_state;
    bool bus_clear;
    uint32_t clk_pin_config;
    uint32_t data_pin_config;
        
    // Save and disable TWI hardware so software can take control over the pins
    twi_state = NRF_TWI1->ENABLE;
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

    clk_pin_config = NRF_GPIO->PIN_CNF[I2C_SCL];
    NRF_GPIO->PIN_CNF[I2C_SCL] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    

    data_pin_config = NRF_GPIO->PIN_CNF[I2C_SDA];
    NRF_GPIO->PIN_CNF[I2C_SDA] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    
      
    TWI_SDA_HIGH();
    TWI_SCL_HIGH();
    TWI_DELAY();

    if (TWI_SDA_READ() == 1 && TWI_SCL_READ() == 1)
    {
        bus_clear = true;
    }
    else
    {
        uint_fast8_t i;
        bus_clear = false;

        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 for slave to respond) to SCL line and wait for SDA come high
        for (i=18; i--;)
        {
            TWI_SCL_LOW();
            TWI_DELAY();
            TWI_SCL_HIGH();
            TWI_DELAY();

            if (TWI_SDA_READ() == 1)
            {
                bus_clear = true;
                break;
            }
        }
    }
    
    NRF_GPIO->PIN_CNF[I2C_SCL] = clk_pin_config;
    NRF_GPIO->PIN_CNF[I2C_SDA] = data_pin_config;

    NRF_TWI1->ENABLE = twi_state;

    return bus_clear;
}


static bool twi_master_read(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
    uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for RXDREADY event*/

    if(data_length == 0)
    {
        /* gently return false for requesting data of size 0 */
        return false;
    }
        
    if (data_length == 1)
    {
        NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_STOP;
    }
    else
    {
        NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_SUSPEND;
    }
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;
    NRF_TWI1->TASKS_STARTRX = 1;
    while(true)
    {
        while((NRF_TWI1->EVENTS_RXDREADY == 0) && (--timeout))
        {
        }

        if(timeout == 0)
        {
            /* timeout before receiving event*/
            return false;
        }

        NRF_TWI1->EVENTS_RXDREADY = 0;
        *data++ = NRF_TWI1->RXD;

        /* configure PPI to stop TWI master before we get last BB event */
        if (--data_length == 1)
        {
            NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_STOP;
        }

        if (data_length == 0)
            break;

        NRF_TWI1->TASKS_RESUME = 1;
    }

    /* wait until stop sequence is sent and clear the EVENTS_STOPPED */
    while(NRF_TWI1->EVENTS_STOPPED == 0)
    {
    }
    NRF_TWI1->EVENTS_STOPPED = 0;

    NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
    return true;
}



bool twi_master_init(void)
{
    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is 
       disabled, these pins must be configured in the GPIO peripheral.
    */
    NRF_GPIO->PIN_CNF[I2C_SCL] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

    NRF_GPIO->PIN_CNF[I2C_SDA] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->EVENTS_TXDSENT = 0;
    NRF_TWI1->PSELSCL = I2C_SCL;
    NRF_TWI1->PSELSDA = I2C_SDA;
    NRF_TWI1->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos;
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TWI1->EVENTS_BB;
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TWI1->TASKS_SUSPEND;
    NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    return twi_master_clear_bus();
}

bool twi_master_transfer(uint8_t address, uint8_t *data, uint16_t data_length, bool issue_stop_condition)
{
    bool transfer_succeeded = true;
    if (data_length > 0 && twi_master_clear_bus())
    {
        NRF_TWI1->ADDRESS = (address >> 1);

        if ((address & TWI_READ_BIT))
        {
            transfer_succeeded = twi_master_read(data, data_length, issue_stop_condition);
        }
        else
        {
            transfer_succeeded = twi_master_write(data, data_length, issue_stop_condition);
        }
    }
    return transfer_succeeded;
}

void TOUCH_init(void)
{
	
	// Init sequence for touch controller MPR121
			


	// This group controls filtering when data is > baseline.	
	uint8_t frame[2]={MHD_R,0x01};
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	
	frame[0]=NHD_R;
	frame[1]=0x01;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);

	frame[0]=NCL_R;
	frame[1]=0x00;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	
	frame[0]=FDL_R;
	frame[1]=0x00;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	

	
		// Section B
	// This group controls filtering when data is < baseline.
	
	frame[0]=MHD_F;
	frame[1]=0x01;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	
	frame[0]=NHD_F;
	frame[1]=0x01;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	
	frame[0]=NCL_F;
	frame[1]=0xFF;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	
	frame[0]=FDL_F;
	frame[1]=0x02;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);
	
	
	// Section C
	// This group sets touch and release thresholds for each electrode
	
	
	frame[0]=ELE4_T;
	frame[1]=TOU_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE4_R;
	frame[1]=REL_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE5_T;
	frame[1]=TOU_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE5_R;
	frame[1]=REL_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE6_T;
	frame[1]=TOU_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE6_R;
	frame[1]=REL_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE7_T;
	frame[1]=TOU_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
	
	frame[0]=ELE7_R;
	frame[1]=REL_THRESH;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);	
		

	
	// Section D
	// Set the Filter Configuration
	// Set ESI2

	frame[0]=FIL_CFG;
	frame[1]=0x04;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);		
	
	
	// Section E
	// Electrode Configuration
	// Enable 6 Electrodes and set to run mode
	// Set ELE_CFG to 0x00 to return to standby mode
	// mpr121Write(ELE_CFG, 0x0C);	// Enables all 12 Electrodes
	
	
	frame[0]=ELE_CFG;
	frame[1]=0x08;
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, true);		
	
	
	// Section F
	// Enable Auto Config and auto Reconfig
	/*mpr121Write(ATO_CFG0, 0x0B);
	mpr121Write(ATO_CFGU, 0xC9);	// USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V
	mpr121Write(ATO_CFGL, 0x82);	// LSL = 0.65*USL = 0x82 @3.3V
	mpr121Write(ATO_CFGT, 0xB5);*/	// Target = 0.9*USL = 0xB5 @3.3V
			
	

}

void twi_oled_write(uint8_t *data)
{
    
    
      if (twi_master_clear_bus())
    {
        NRF_TWI1->ADDRESS = (ADDR_OLED_W >> 1);

        
            uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for EVENTS_TXDSENT event*/
            uint8_t data_length=17;

            if(data_length == 17)
            {
                    NRF_TWI1->TXD = 0x40;
            }
            else
            {
                    NRF_TWI1->TXD = *data++;
            }
            
            NRF_TWI1->TASKS_STARTTX = 1;

            while (true)
            {
                    while(NRF_TWI1->EVENTS_TXDSENT == 0 && (--timeout))
                    {
                    }

                    if (timeout == 0)
                    {
                            NRF_TWI1->EVENTS_STOPPED = 0; 
                            NRF_TWI1->TASKS_STOP = 1; 
                            /* wait until stop sequence is sent and clear the EVENTS_STOPPED */ 
                            while(NRF_TWI1->EVENTS_STOPPED == 0) 
                            { 
                            }
                            /* timeout before receiving event*/

                    }


                    --data_length;
                    
                    NRF_TWI1->EVENTS_TXDSENT = 0;
                    if (data_length == 0)
                    {
                            break;
                    }

                    NRF_TWI1->TXD = *data++;
            }
            
     
                    NRF_TWI1->EVENTS_STOPPED = 0; 
                    NRF_TWI1->TASKS_STOP = 1; 
                    /* wait until stop sequence is sent and clear the EVENTS_STOPPED */ 
                    while(NRF_TWI1->EVENTS_STOPPED == 0) 
                    { 
                    } 

        }
}

void OLED_clear(void) 
{
        for (uint8_t i=1; i<65; i++) {
            twi_oled_write(buffer_ff);          
    }
}

void OLED_display(uint8_t what) 
{
    /*
        */
    uint8_t framek[7]={
        0x00,SSD1306_MEMORYMODE,0x00,
        0x00,SSD1306_COLUMNADDR,32,95
    };
    twi_master_transfer(ADDR_OLED_W, framek, 7, true);
    
    
    if(what==img_bt_advertise)
    {
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q5);
        twi_oled_write(buffer_bt_q6);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q9);
        twi_oled_write(buffer_bt_q10);
        twi_oled_write(buffer_bt_q11);
        twi_oled_write(buffer_bt_q12);
        twi_oled_write(buffer_bt_q13);
        twi_oled_write(buffer_bt_q14);
        twi_oled_write(buffer_bt_q15);
        twi_oled_write(buffer_bt_q16);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q18);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q20);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
    }
    else if(what==img_bt_connected)
    {
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q5);
        twi_oled_write(buffer_bt_q6);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q9);
        twi_oled_write(buffer_bt_q10);
        twi_oled_write(buffer10on);
        twi_oled_write(buffer11on);
        twi_oled_write(buffer_bt_q13);
        twi_oled_write(buffer_bt_q14);
        twi_oled_write(buffer14on);
        twi_oled_write(buffer15on);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q18);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q20);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);  
    }
    else if(what==img_bt_disconnected)
    {
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q5);
        twi_oled_write(buffer_bt_q6);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q9);
        twi_oled_write(buffer_bt_q10);
        twi_oled_write(buffer_bt_q12);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q13);
        twi_oled_write(buffer_bt_q14);
        twi_oled_write(buffer_bt_q16);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q18);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_bt_q20);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
        twi_oled_write(buffer_ff);
    }
        for (uint8_t i=1; i<9; i++) {
            twi_oled_write(buffer_ff);          
    }
    
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

        OLED_display(img_bt_advertise);
        

}


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    ///nrf_gpio_pin_set(YLED_ASSERT);

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Uncomment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    ///battery_level = (uint8_t)ble_sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
        battery_level = 212;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
             )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    uint32_t        err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(p_context);

    heart_rate = (uint16_t)ble_sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    //       of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}



/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)ble_sensorsim_measure(&m_rr_interval_sim_state,
                                                      &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_WATCH);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
 
 
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
		
    };

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    ble_sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    ble_sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:

            OLED_display(img_bt_connected);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

                        case BLE_GAP_EVT_DISCONNECTED:
                    
            OLED_display(img_bt_disconnected);


            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Since we are not in a connection and have not started advertising, store bonds.
            err_code = ble_bondmngr_bonded_centrals_store();
            APP_ERROR_CHECK(err_code);

            advertising_start();


            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                                OLED_display(img_bt_disconnected);

                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_bondmngr_on_ble_evt(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for handling a Bond Manager error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for the Bond Manager initialization.
 */
static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bond_init_data;
    bool                bonds_delete;
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);
    
    // Clear all bonded centrals if the Bonds Delete button is pushed.
    ///bonds_delete = (nrf_gpio_pin_read(YBUTTON_2) == 0);///ddd

    // Initialize the Bond Manager.
    bond_init_data.flash_page_num_bond     = FLASH_PAGE_BOND;
    bond_init_data.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bond_init_data.evt_handler             = NULL;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;

    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



void OLED_invertDisplay(bool i) {
    uint8_t frame[2]={0x00,0x00};
    
  if (i) {
        frame[1]=(SSD1306_INVERTDISPLAY);
  } else {
        frame[1]=(SSD1306_NORMALDISPLAY);
  }
    twi_master_transfer(ADDR_OLED_W, frame, 2, true);
    
}

void OLED_init(void)
{
    
    // Init sequence for 64x48 OLED module
            
    uint8_t frame[51]={
    0x00,SSD1306_DISPLAYOFF,
    0x00,SSD1306_SETDISPLAYCLOCKDIV,0x80,
    0x00,SSD1306_SETMULTIPLEX,0x2F,
    0x00,SSD1306_SETDISPLAYOFFSET,0,
    0x00,SSD1306_SETSTARTLINE | 0x0,//////good
    0x00,SSD1306_CHARGEPUMP,0x14,
    0x00,SSD1306_MEMORYMODE,0x00,
    0x00,SSD1306_SEGREMAP | 0x1,
    0x00,SSD1306_COMSCANDEC, 0xC8,////////////////
    0x00,SSD1306_SETCOMPINS,0x12,
    0x00,SSD1306_SETCONTRAST,0xCF,
    0x00,SSD1306_SETPRECHARGE,0x22,
    0x00,SSD1306_SETVCOMDETECT,0x00,
    0x00,SSD1306_DISPLAYALLON_RESUME,
    0x00,SSD1306_NORMALDISPLAY,
    0x00,SSD1306_DISPLAYON
    };
    
    
        // Setup reset pin direction (used by both SPI and I2C)  
        nrf_gpio_pin_set(OLED_RST);

        // VDD (3.3V) goes high at start, lets just chill for a ms
        nrf_delay_us(1000);
        // bring reset low
        nrf_gpio_pin_clear(OLED_RST);
        // wait 10ms
        nrf_delay_us(1000);
        // bring out of reset
        nrf_gpio_pin_set(OLED_RST);
        // turn on VCC (9V?)
    
        nrf_delay_us(100);

    twi_master_transfer(ADDR_OLED_W, frame, 51, true);
    
    
    OLED_invertDisplay(true);
    
    OLED_clear();


}




// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED_startscrollright(uint8_t start, uint8_t stop){

    uint8_t frame[16]={
        0x00,SSD1306_RIGHT_HORIZONTAL_SCROLL,
        0x00,0x00,
        0x00,start,
        0x00,0x00,
        0x00,stop,
        0x00,0x00,
        0x00,0xFF,
        0x00,SSD1306_ACTIVATE_SCROLL
    };

    twi_master_transfer(ADDR_OLED_W, frame, 16, true);
    
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED_startscrollleft(uint8_t start, uint8_t stop){
    
        uint8_t frame[16]={
        0x00,SSD1306_LEFT_HORIZONTAL_SCROLL,
        0x00,0x00,
        0x00,start,
        0x00,0x00,
        0x00,stop,
        0x00,0x00,
        0x00,0xFF,
        0x00,SSD1306_ACTIVATE_SCROLL
    };

    twi_master_transfer(ADDR_OLED_W, frame, 16, true);

}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED_startscrolldiagright(uint8_t start, uint8_t stop){
    
    uint8_t frame[22]={
        0x00,SSD1306_SET_VERTICAL_SCROLL_AREA,
        0x00,0x00,
        0x00,OLED_HEIGHT,
        0x00,SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL,
        0x00,0x00,
        0x00,start,
        0x00,0x00,
        0x00,stop,
        0x00,0x01,
        0x00,SSD1306_ACTIVATE_SCROLL
    };

    twi_master_transfer(ADDR_OLED_W, frame, 22, true);
    
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED_startscrolldiagleft(uint8_t start, uint8_t stop){

    
        uint8_t frame[22]={
        0x00,SSD1306_SET_VERTICAL_SCROLL_AREA,
        0x00,0x00,
        0x00,OLED_HEIGHT,
        0x00,SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL,
        0x00,0x00,
        0x00,start,
        0x00,0x00,
        0x00,stop,
        0x00,0x01,
        0x00,SSD1306_ACTIVATE_SCROLL
    };

    twi_master_transfer(ADDR_OLED_W, frame, 22, true);
    

}

void OLED_stopscroll(void){
    
    uint8_t frame[2]={
        0x00,SSD1306_DEACTIVATE_SCROLL
    };

    twi_master_transfer(ADDR_OLED_W, frame, 2, true);
    
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void OLED_dim(uint8_t contrast) {
///  uint8_t contrast;
/*
  if (dim) {
    contrast = 0; // Dimmed display
  } else {
    contrast = 0xCF;
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
 */   
    uint8_t frame[3]={
        0x00,SSD1306_SETCONTRAST,contrast   
    };

    twi_master_transfer(ADDR_OLED_W, frame, 3, true);

}



static void GPIO_init(void)
{

    NRF_GPIO->PIN_CNF[BUTTON_W] = 
                                                            (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);   
    /*      
    NRF_GPIO->PIN_CNF[ACCEL_DEN_G] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);      
    */      
            
    NRF_GPIO->PIN_CNF[ACCEL_DRDY_INT2_G] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);           

    NRF_GPIO->PIN_CNF[ACCEL_INT1_G] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);       
        
    NRF_GPIO->PIN_CNF[ACCEL_INT1_A] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);       

    NRF_GPIO->PIN_CNF[ACCEL_INT2_A] =
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);       
        
    NRF_GPIO->PIN_CNF[ACCEL_INT2_A] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);       


    NRF_GPIO->PIN_CNF[BUZZER] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                                            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[BUZZER2] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                                            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
                                                                                            
    NRF_GPIO->PIN_CNF[OLED_RST] = 
                                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                                            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[BAT_STATUS] = 
                                                            (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)| 
                                                            (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)| 
                                                            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)| 
                                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)| 
                                                            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);   



/*

#define BAT_LVL      //AIN2

*/
        
        
}



/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
  GPIO_init();
  twi_master_init();
  OLED_init();
	TOUCH_init();
    
		
	uint8_t frame[1]={0x00};
	uint8_t rxdata[1]={0x22};
	/*
	twi_master_transfer(ADDR_TOUCH_W, frame, 2, false);
	twi_master_transfer(ADDR_TOUCH_R, rxdata, 1, true);
	*/


  ble_stack_init();
  bond_manager_init();
  timers_init();
  gap_params_init();
  advertising_init();
  services_init();
  sensor_sim_init();
  conn_params_init();
  sec_params_init();



OLED_invertDisplay(0);

    // Start execution.
  application_timers_start();
  advertising_start();


	OLED_dim(0x00);




// Enter main loop.
    for (;;)
    {
        power_manage();
    }
        
        
}


