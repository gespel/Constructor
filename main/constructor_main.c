/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "slang-lib.h"
#include "driver/i2s_std.h"

static const char *TAG = "Constructor";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2
#define SAMPLE_RATE 44100
#define BLOCK_SIZE 512

static uint8_t s_led_state = 0;

i2s_chan_handle_t tx_handle;
SlangBufferCore *core;
uint16_t i2s_buffer[BLOCK_SIZE];


static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void init_i2s(void) {
    // Channel configuration with DMA buffers like Arduino code
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);

    // MSB configuration like Arduino (not Philips)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_18,
            .ws = GPIO_NUM_25,
            .dout = GPIO_NUM_26,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    i2s_channel_enable(tx_handle);
}

void audio_render_task(void *pvParameters) {
    size_t bytes_written;

    ESP_LOGI(TAG, "Audio rendering task started on core %d", xPortGetCoreID());

    while (1) {
        float *buf = renderBuffer(core);

        // Convert exactly like Arduino code
        for (int i = 0; i < BLOCK_SIZE; i++) {
            float sample = buf[i] + 1.0f; // Shift to positive range
            sample *= 7000.0f;
            //ESP_LOGI(TAG, "Sample %d: %f and %d", i, sample, (uint16_t)sample);
            i2s_buffer[i] = (uint16_t)sample;
        }

        // Write MONO data (like Arduino)
        esp_err_t ret = i2s_channel_write(tx_handle, i2s_buffer, sizeof(uint16_t) * BLOCK_SIZE, &bytes_written, portMAX_DELAY);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write error: %d", ret);
        }

        // Blink LED every block
        s_led_state = !s_led_state;
        blink_led();
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield
    }
}

void app_main(void) {
    init_i2s();

    // Only the oscillator - NO extra variables (x=10 adds DC offset!)
    char *p = "s = sawtoothosc(110);";
    int numTokens = 0;

    Token *tokens = tokenize(p, &numTokens);

    SlangInterpreter *interpreter = createSlangInterpreter(tokens, numTokens);
    core = createBufferCore(interpreter, SAMPLE_RATE, BLOCK_SIZE);

    interpret(interpreter);

    configure_led();

    ESP_LOGI(TAG, "Starting audio rendering task on core 1...");
    
    // Create audio task on core 1 with high priority
    xTaskCreatePinnedToCore(
        audio_render_task,      // Task function
        "audio_render",         // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        10,                     // Priority (high)
        NULL,                   // Task handle
        1                       // Core 1
    );

    ESP_LOGI(TAG, "Main task running on core %d", xPortGetCoreID());
    
    // Keep main task alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}