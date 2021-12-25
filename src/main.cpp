#include <Arduino.h>
#include <WiFi.h>
#include "led_strip.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <ESP32HomeKit.h>

const char *ssid = "";
const char *password = "";

#define COLOR_ORDER GRB
#define NUM_LEDS 9

#define LED_PIN GPIO_NUM_7

#define SDA 5
#define SCL 6

#define BUZ 8

#define BUTTON_A 10
#define BUTTON_B 4
#define BUTTON_UP 3
#define BUTTON_DOWN 1
#define BUTTON_LEFT 0
#define BUTTON_RIGHT 2

#define RMT_TX_CHANNEL RMT_CHANNEL_0

static const char *TAG = "AmpyTrophy";

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

typedef struct rgb
{
    uint8_t r; // 0-100 %
    uint8_t g; // 0-100 %
    uint8_t b; // 0-100 %
} rgb_t;

typedef struct light_state
{
    bool on;
    uint16_t b; // 0-100
    uint16_t h; // 0-360
    uint16_t s; // 0-100
} light_state_t;

static light_state_t light_state_val;
static rgb_t rgb_val;

led_strip_t *strip;
rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_PIN, RMT_TX_CHANNEL);

static bool light_state2rgb(light_state_t *ls, rgb_t *rgb)
{
    bool res = true;
    uint16_t hi, v, F, P, Q, T;

    if (!rgb)
        return false;

    if (ls->h > 360)
        return false;
    if (ls->s > 100)
        return false;
    if (ls->b > 100)
        return false;

    v = ls->b;
    hi = (ls->h / 60) % 6;
    F = 100 * ls->h / 60 - 100 * hi;
    P = v * (100 - ls->s) / 100;
    Q = v * (10000 - F * ls->s) / 10000;
    T = v * (10000 - ls->s * (100 - F)) / 10000;

    switch (hi)
    {
    case 0:
        rgb->r = v;
        rgb->g = T;
        rgb->b = P;
        break;
    case 1:
        rgb->r = Q;
        rgb->g = v;
        rgb->b = P;
        break;
    case 2:
        rgb->r = P;
        rgb->g = v;
        rgb->b = T;
        break;
    case 3:
        rgb->r = P;
        rgb->g = Q;
        rgb->b = v;
        break;
    case 4:
        rgb->r = T;
        rgb->g = P;
        rgb->b = v;
        break;
    case 5:
        rgb->r = v;
        rgb->g = P;
        rgb->b = Q;
        break;
    default:
        return false;
    }
    return res;
}

static void init_ampy()
{
    light_state_val.on = true;
    light_state_val.b = 100;
    light_state_val.h = 0;
    light_state_val.s = 0;

    light_state2rgb(&light_state_val, &rgb_val);

    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(NUM_LEDS, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);

    // // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    u8g2.begin();
}

static void esp32_beep(unsigned int freq, unsigned int dur_ms)
{
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;

    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    ledc_timer.freq_hz = freq;                      // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;    // timer mode
    ledc_timer.timer_num = LEDC_TIMER_0;            // timer index
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;             // timer index

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = BUZ;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;

    ledc_channel_config(&ledc_channel);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4095));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    vTaskDelay(pdMS_TO_TICKS(dur_ms));
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

static void play_tune()
{
    int melody[] = {
        830, 830, 830, 830, 660, 740, 830, 740, 830};

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = {
        8, 8, 8, 4, 3, 3, 5, 8, 5};

    for (int thisNote = 0; thisNote < 9; thisNote++)
    {
        int noteDuration = 1000 / noteDurations[thisNote];
        esp32_beep(melody[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.1;
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void display_message(int lines, ...)
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvR12_te);

    va_list argp;
    va_start(argp, lines);
    for (int i = 0; i < lines; i++)
    {
        const char *line = va_arg(argp, char *);
        u8g2.drawStr(2, (12 * (i + 1)) + 4 * i, line);
    }
    va_end(argp);

    u8g2.sendBuffer();
}

static int identify(hap_acc_t *ha)
{
    return HAP_SUCCESS;
}

static int lightbulb_write(hap_write_data_t write_data[], int count,
                           void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++)
    {
        write = &write_data[i];
        /* Setting a default error value */
        *(write->status) = HAP_STATUS_VAL_INVALID;

        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON))
        {
            light_state_val.on = write->val.b;
            light_state2rgb(&light_state_val, &rgb_val);

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_BRIGHTNESS))
        {
            //     display_message(2, "Received Write for Light Brightness %d", write->val.i);
            light_state_val.b = write->val.i;
            light_state2rgb(&light_state_val, &rgb_val);

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_HUE))
        {
            //     display_message(2, "Received Write for Light Hue %f", write->val.f);
            light_state_val.h = write->val.f;
            light_state2rgb(&light_state_val, &rgb_val);

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SATURATION))
        {
            //     display_message(2, "Received Write for Light Saturation %f", write->val.f);
            light_state_val.s = write->val.f;
            light_state2rgb(&light_state_val, &rgb_val);

            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else
        {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
        /* If the characteristic write was successful, update it in hap core
         */
        if (*(write->status) == HAP_STATUS_SUCCESS)
        {
            hap_char_update_val(write->hc, &(write->val));
        }
        else
        {
            /* Else, set the return value appropriately to report error */
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static void wifi_connect()
{
    WiFi.setHostname("AmpyTrophy");
    WiFi.begin(ssid, password);

    int dots = 5;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);

        // Serial.println("Establishing connection to WiFi..");
        String connectingMessage = String("Connecting.....");
        connectingMessage.remove(10, dots);
        display_message(1, connectingMessage.c_str());
        dots--;
        if (dots < 0)
        {
            dots = 5;
        }
    }
}

static void homekit_connect()
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "AmpyTrophy",
        .model = "AmpyTrophy",
        .manufacturer = "AmpyTrophy",
        .serial_num = "001122334455",
        .fw_rev = "0.0.1",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .cid = HAP_CID_LIGHTING,
        .identify_routine = identify,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E', 'S', 'P', '3', '2', 'H', 'A', 'P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Create the Light Bulb Service. Include the "name" since this is a user visible service  */
    service = hap_serv_lightbulb_create(true);

    /* Add the optional characteristic to the Light Bulb Service */
    hap_serv_add_char(service, hap_char_name_create("My Light"));
    // hap_serv_add_char(service, hap_char_on_create(false));
    hap_serv_add_char(service, hap_char_active_create(light_state_val.on));
    hap_serv_add_char(service, hap_char_brightness_create(light_state_val.b));
    hap_serv_add_char(service, hap_char_hue_create(light_state_val.h));
    hap_serv_add_char(service, hap_char_saturation_create(light_state_val.s));

    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, lightbulb_write);

    /* Add the Light Bulb Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
             hap_get_paired_controller_count());

    /* TODO: Do the actual hardware initialization here */

    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code("111-22-333");
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id("ES32");

    /* After all the initializations are done, start the HAP core */
    hap_start();
}

void setup()
{
    delay(3000); // 3 second delay for recovery

    init_ampy();

    display_message(1, "AmpyTrophy '21");

    wifi_connect();

    display_message(2, "Wifi: Connected", WiFi.localIP().toString().c_str());

    homekit_connect();

    display_message(3, "Wifi: Connected", WiFi.localIP().toString().c_str(), "HomeKit: Enabled");

    play_tune();
}

void loop()
{

    if (light_state_val.on)
    {
        for (int i = 0; i < NUM_LEDS; i++)
        {
            ESP_ERROR_CHECK(strip->set_pixel(strip, i, rgb_val.r, rgb_val.g, rgb_val.b));
        }
        ESP_ERROR_CHECK(strip->refresh(strip, 50));
    }
    else
    {
        ESP_ERROR_CHECK(strip->clear(strip, 100));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}
