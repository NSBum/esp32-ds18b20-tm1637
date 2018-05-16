/*
 * MIT License
 *
 * Copyright (c) 2018 Alan Duncan
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Portions of this software are derived from an example work by David Antliff,
 * the developer of the esp32-owb and esp32-ds18b20 libraries.
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"

#include "nvs_flash.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#include "tm1637.h"

static const char MAINTAG = "DS18B20-TM1637";

/*
 *  CONFIGURATION VARIABLES
 *
 *  To configure variable that are prefixed 'CONFIG'
 *  use the `make menuconfig` utility.
 */
#define GPIO_DS18B20_0       (CONFIG_ONE_WIRE_GPIO)
#define GPIO_TM1637_1_CLK    (CONFIG_TM1637_1_CLK)
#define GPIO_TM1637_1_DIO    (CONFIG_TM1637_1_DIO)
#define GPIO_TM1637_2_CLK    (CONFIG_TM1637_2_CLK)
#define GPIO_TM1637_2_DIO    (CONFIG_TM1637_2_DIO)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (1000)   // milliseconds betwen temperature measurements

//  TM1637-based 4 digit, seven-segment displays
tm1637_led_t *led[2];

//  Array of pointers to DS18B20 devices
DS18B20_Info * devices[MAX_DEVICES] = {0};
//  Array of OneWire bus ROM codes
OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
//  The OneWire bus object
OneWireBus * owb;

// Known ROM code (LSB first), optionally we can use these in lieu of the search
/*

OneWireBus_ROMCode known_device0 = {
    .fields.family = { 0x28 },
    .fields.serial_number = { 0xFF, 0x11, 0x35, 0x02, 0x17, 0x04 },
    .fields.crc = { 0x9C },
};
OneWireBus_ROMCode known_device1 = {
    .fields.family = { 0x28 },
    .fields.serial_number = { 0xFF, 0x37, 0x1D, 0x03, 0x17, 0x05 },
    .fields.crc = { 0x61 },
};

*/

int num_devices = 0;
float readings[MAX_DEVICES] = { 0 };

//  opaque collection of flags
static EventGroupHandle_t owb_event_group;
// flag bits in the event group
const int SEARCH_COMPLETE_BIT = BIT0;
const int DS18B20_SETUP_COMPLETE_BIT = BIT1;
const int DS18B20_TEMP_READY = BIT2;

esp_err_t nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void owb_search_task(void *pvParameters ) {
    xEventGroupClearBits(owb_event_group, SEARCH_COMPLETE_BIT);

    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);

    owb_use_crc(owb, true);              // enable CRC check for ROM code

    // Find all connected devices
    ESP_LOGV(MAINTAG,"Finding devices");
    
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while( found ) {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    ESP_LOGI(MAINTAG,"Found %d devices", num_devices)

    xEventGroupSetBits(owb_event_group, SEARCH_COMPLETE_BIT);
    vTaskDelete(NULL);
}

void ds18b20_init_devices(void *pvParameters) {
    xEventGroupClearBits(owb_event_group, DS18B20_SETUP_COMPLETE_BIT);
    xEventGroupWaitBits(owb_event_group, SEARCH_COMPLETE_BIT, 1, 0, portMAX_DELAY);
    for( int i = 0; i < num_devices; ++i ) {
        devices[i] = ds18b20_malloc();;
        ds18b20_init(devices[i], owb, device_rom_codes[i]); // associate with bus and device
        ds18b20_use_crc(devices[i], true);           // enable CRC check for temperature readings
        ds18b20_set_resolution(devices[i], DS18B20_RESOLUTION);
    }
    xEventGroupSetBits(owb_event_group, DS18B20_SETUP_COMPLETE_BIT);
    vTaskDelete(NULL);
}

void owb_get_temps(void *pvParameters) {
    //  wait for bus scan to complete
    xEventGroupWaitBits(owb_event_group, DS18B20_SETUP_COMPLETE_BIT, 1, 0, portMAX_DELAY);

    // Read temperatures more efficiently by starting conversions on all devices at the same time
    int errors_count[MAX_DEVICES] = {0};
    int sample_count = 0;
    if (num_devices > 0) {
        TickType_t last_wake_time = xTaskGetTickCount();

        while (1) {
            xEventGroupClearBits(owb_event_group, DS18B20_TEMP_READY);
            last_wake_time = xTaskGetTickCount();

            ds18b20_convert_all(owb);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(devices[0]);

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

            for (int i = 0; i < num_devices; ++i) {
                errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
            }

            // Print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
            for (int i = 0; i < num_devices; ++i) {
                if (errors[i] != DS18B20_OK) {
                    ++errors_count[i];
                }
                char rom_code_s[17];
                owb_string_from_rom_code(device_rom_codes[i], rom_code_s, sizeof(rom_code_s));
                printf("  %d - %s: %.2f    %d errors\n", i, rom_code_s, readings[i], errors_count[i]);
            }
            xEventGroupSetBits(owb_event_group, DS18B20_TEMP_READY);
            tm1637_set_float(led[0],readings[0]);
            tm1637_set_float(led[1],readings[1]);
            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
}

void app_main() {
    esp_log_level_set("*", ESP_LOG_INFO);

    //  init NVS
    ESP_ERROR_CHECK( nvs_init() );

    owb_event_group = xEventGroupCreate();

    //  initialize our displays
    led[0] = tm1637_init(GPIO_TM1637_1_CLK,GPIO_TM1637_1_DIO);
    led[1] = tm1637_init(GPIO_TM1637_2_CLK,GPIO_TM1637_2_DIO);

    // Allow bus to stabilize a bit before communicating
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    xTaskCreate(&owb_search_task,"owbsearch",4096,NULL,5,NULL);
    xTaskCreate(&ds18b20_init_devices,"initdevives",4096,NULL,5,NULL);
    xTaskCreate(&owb_get_temps,"gettemps",4096,NULL,5,NULL);

    while( 1 ) { }
      
    // clean up dynamically allocated data
    for (int i = 0; i < num_devices; ++i) {
        ds18b20_free(&devices[i]);
    }

    printf("Restarting now.\n");
    fflush(stdout);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
}
