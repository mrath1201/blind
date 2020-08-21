#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>
#include <button.h>
#include "ota-api.h"

const int step_gpio = 0;
const int led_gpio = 2;

const int button_up = 12;
const int button_down = 13;

// Installation on the left
// clock wise = down
// anti clock wise = up

// Installation on the right
// clock wise = up
// anti clock wise = down

const int direction_gpio = 14; // 1 = HIGH; Rotate stepper motor in clock wise direction, 0 = LOW; Rotate stepper motor in anti clock wise direction


#ifndef HK_PASSWORD
#error HK_PASSWORD is not specified
#endif
#ifndef FW
#error FW is not specified
#endif

bool calibrate = false;
int calibrate_state = 0;
int calibrate_steps = 100000;
int calibrate_steps_up = 0;
int calibrate_steps_down = 0;
int delay = 10;

#define POSITION_OPEN 100
#define POSITION_CLOSED 0
#define POSITION_STATE_CLOSING 0
#define POSITION_STATE_OPENING 1
#define POSITION_STATE_STOPPED 2

TaskHandle_t stepperTask;
homekit_characteristic_t current_position;
homekit_characteristic_t target_position;
homekit_characteristic_t position_state;
homekit_characteristic_t hold_position;
homekit_accessory_t *accessories[];


homekit_characteristic_t name          = HOMEKIT_CHARACTERISTIC_(NAME, "RedBlind");
homekit_characteristic_t ota_trigger   = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer  = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "RedOrbit");
homekit_characteristic_t serial        = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "0000001");
homekit_characteristic_t model         = HOMEKIT_CHARACTERISTIC_(MODEL,         "RedBlind");
homekit_characteristic_t revision      = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  FW);

void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}

void direction_write(int direction) {
    gpio_write(direction_gpio, direction);
    position_state.value.int_value = direction;
}

void step_write_task(int steps, int delay, bool calibrate) {
    for (int step=0; step < steps; step++) {
        gpio_write(step_gpio, 1);
        if (calibrate) {
            if (position_state.value.int_value == POSITION_STATE_CLOSING){
                calibrate_steps_down = step;
            }
            if (position_state.value.int_value == POSITION_STATE_OPENING){
                calibrate_steps_up = step;
            }
        }
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

void step_write(int steps, int delay, bool calibrate) {
    xTaskCreate(step_write_task, "Move steps", 256, NULL, tskIDLE_PRIORITY, &stepperTask);
    vTaskSuspend(stepperTask);
}

void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    printf("Resetting Wifi Config\n");

    wifi_config_reset();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Resetting HomeKit Config\n");

    homekit_server_reset();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Restarting\n");

    sdk_system_restart();

    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

void calibrate_task(int calibrate_state) {
  if (calibrate_state == 0) {
    direction_write(POSITION_STATE_CLOSING);
    step_write(calibrate_steps)
  } else if(calibrate_state == 1) {
    direction_write(POSITION_STATE_OPENING);
    step_write(calibrate_steps)
  }
}

void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(false);
    gpio_enable(direction_gpio, GPIO_OUTPUT);
    direction_write(1);

    gpio_enable(step_gpio, GPIO_OUTPUT);
    gpio_write(step_gpio, 0);

    gpio_enable(button_up, GPIO_INPUT);
    gpio_enable(button_down, GPIO_INPUT);
}

void button_up_callback(uint8_t gpio_num, button_event_t event) {
    // up button pressed
    if (position_state.value.int_value != POSITION_STATE_STOPPED){ // if moving, stop
	target_position.value.int_value = current_position.value.int_value;
	target_position_changed();
    }else{
        switch (event) {
            case button_event_single_press:
	            target_position.value.int_value = POSITION_OPEN;
                target_position_changed();
                break;
            case button_event_long_press:
                reset_configuration();
                break;
            default:
                printf("Unknown button event: %d\n", event);
        }
    }
}

void button_down_callback(uint8_t gpio_num, button_event_t event) {
    // down button pressed
    if (position_state.value.int_value != POSITION_STATE_STOPPED){ // if moving, stop
	target_position.value.int_value = current_position.value.int_value;
	target_position_changed();
    }else{
        switch (event) {
            case button_event_single_press:
                if (calibrate) {
                    if (calibrate_state == 0) {
                        vTaskSuspend(stepperTask);
                        calibrate_state = 1;
                        calibrate_task(calibrate_state);
                    } else if (calibrate_state == 1) {
                        vTaskSuspend(stepperTask);
                        hold_position.value.int_value = calibrate_steps_up;
                    }
                } else {
	                target_position.value.int_value = POSITION_CLOSED;
                    target_position_changed();
                }
                break;
            case button_event_long_press:
                calibrate = true;
                calibrate_task(calibrate_state);
                break;
            default:
                printf("Unknown button event: %d\n", event);
        }
    }
}

void blind_identify_task(void *_args) {
    // We identify the Sonoff by Flashing it's LED.
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(false);

    vTaskDelete(NULL);
}

void blind_identify(homekit_value_t _value) {
    printf("blind identify\n");
    xTaskCreate(blind_identify_task, "blind identify", 128, NULL, 2, NULL);
}

void on_update_target_position(homekit_characteristic_t *ch, homekit_value_t value, void *context);

homekit_characteristic_t current_position = {
    HOMEKIT_DECLARE_CHARACTERISTIC_CURRENT_POSITION(POSITION_CLOSED)
};

homekit_characteristic_t target_position = {
    HOMEKIT_DECLARE_CHARACTERISTIC_TARGET_POSITION(POSITION_CLOSED, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update_target_position))
};

homekit_characteristic_t position_state = {
    HOMEKIT_DECLARE_CHARACTERISTIC_POSITION_STATE(POSITION_STATE_STOPPED)
};
homekit_characteristic_t hold_position = {
    HOMEKIT_DECLARE_CHARACTERISTIC_HOLD_POSITION(POSITION_STATE_HOLD)
};

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_window_covering, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            &name,
            &manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, blind_identify),
            NULL
        }),
        HOMEKIT_SERVICE(WINDOW_COVERING, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "RedBlind"),
            &current_position,
            &target_position,
            &position_state,
            &hold_position,
            &ota_trigger,
            NULL
        }),
        NULL
    }),
    NULL
};

void on_update_target_position(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    target_position_changed();
}

void target_position_changed(){
    printf("Update target position to: %u\n", target_position.value.int_value);

    if (target_position.value.int_value == current_position.value.int_value) {
        printf("Current position equal to target. Stopping.\n");
        position_state.value.int_value = POSITION_STATE_STOPPED;
        homekit_characteristic_notify(&position_state, position_state.value);
        vTaskSuspend(stepperTask);
    } else {
        position_state.value.int_value = target_position.value.int_value > current_position.value.int_value
            ? POSITION_STATE_OPENING
            : POSITION_STATE_CLOSING;

        homekit_characteristic_notify(&position_state, position_state.value);
        vTaskResume(stepperTask);
    }
}

homekit_server_config_t config = {
    .accessories = accessories,
    .password = HK_PASSWORD,
    .setupId = HK_SETUP_ID,
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "RedBlind-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "RedBlind-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);

    create_accessory_name();

    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value, &model.value.string_value,&revision.value.string_value);
    //c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;

    wifi_config_init("blind", NULL, on_wifi_ready);

    gpio_init();

    if (button_create(button_gpio, 0, 4000, button_callback)) {
        printf("Failed to initialize button\n");
    }
}