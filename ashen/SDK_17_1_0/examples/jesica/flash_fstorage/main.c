#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "boards.h"

// Flash region to use
#define FLASH_START_ADDR  0x00027000
#define FLASH_END_ADDR    0x00028000
#define BLE_CONN_CFG_TAG 3

// Dummy data to write
static uint32_t thres_def_data[] = {1, 2, 3};

// Forward-declare handler
static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) = {
    .evt_handler = fstorage_evt_handler,
    .start_addr  = FLASH_START_ADDR,
    .end_addr    = FLASH_END_ADDR,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt) {
    if (p_evt->result != NRF_SUCCESS) {
        // Error handling
        return;
    }

    switch (p_evt->id) {
        case NRF_FSTORAGE_EVT_ERASE_RESULT:
            bsp_board_led_on(LED2_G); // Erase OK
            // Write after erase
            ret_code_t err = nrf_fstorage_write(
                &fstorage,
                FLASH_START_ADDR,
                thres_def_data,
                sizeof(thres_def_data),
                NULL
            );
            APP_ERROR_CHECK(err);
            break;

        case NRF_FSTORAGE_EVT_WRITE_RESULT:
            bsp_board_led_on(LED2_B); // Write OK
            break;

        default:
            break;
    }
}

// Main
int main(void) {
    // init LEDs
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_leds_off();

    // Enable SoftDevice
    ret_code_t err = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err);

    uint32_t ram_start = 0;
    err = nrf_sdh_ble_default_cfg_set(BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err);

    err = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err);

    // init fstorage with SoftDevice backend
    err = nrf_fstorage_init(&fstorage, &nrf_fstorage_sd, NULL);
    APP_ERROR_CHECK(err);

    // erase one page
    err = nrf_fstorage_erase(&fstorage, FLASH_START_ADDR, 1, NULL);
    APP_ERROR_CHECK(err);

    // idle forever - wait for flash event handler
    for (;;) {
        __WFE();
    }
}