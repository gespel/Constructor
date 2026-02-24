/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "slang-lib.h"
#include "driver/i2s_std.h"

static const char *TAG = "Constructor";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2
#define SAMPLE_RATE 44100
#define BLOCK_SIZE 2048  // Smaller buffer for better performance

static uint8_t s_led_state = 0;

i2s_chan_handle_t tx_handle;
SlangBufferCore *core;
uint16_t i2s_buffer[BLOCK_SIZE * 2];  // STEREO: L+R channels
TaskHandle_t audio_task_handle = NULL;


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
    // Channel configuration matching Arduino code
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 1024;  // Match Arduino dma_buf_len
    chan_cfg.auto_clear = true;     // Match tx_desc_auto_clear
    
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);

    // Standard Philips I2S with STEREO - use PLL clock instead of APLL
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),  // Use default PLL
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
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
    int block_count = 0;

    ESP_LOGI(TAG, "Audio rendering task started on core %d", xPortGetCoreID());
    
    // Unsubscribe this task from the watchdog
    esp_task_wdt_delete(NULL);

    while (1) {
        float *buf = renderBuffer(core);
        
        if (buf == NULL) {
            ESP_LOGE(TAG, "renderBuffer returned NULL!");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Convert exactly like Arduino code and duplicate for stereo
        for (int i = 0; i < BLOCK_SIZE; i++) {
            float sample = buf[i];
            sample -= 0.2f;      // Arduino offset
            sample *= 4000.0f;  // Arduino scaling
            uint16_t sample_16 = (uint16_t)((int16_t)sample);
            
            // Send same sample to both L and R channels
            i2s_buffer[i * 2] = sample_16;      // Left
            i2s_buffer[i * 2 + 1] = sample_16;  // Right
        }

        // Write STEREO data (both channels)
        esp_err_t ret = i2s_channel_write(tx_handle, i2s_buffer, BLOCK_SIZE * 2 * sizeof(uint16_t), &bytes_written, portMAX_DELAY);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write error: %d", ret);
        }
        
        // Log first few samples for debugging
        if (block_count < 3) {
            ESP_LOGI(TAG, "Block %d: buf[0]=%d buf[100]=%d buf[200]=%d", 
                     block_count, buf[0], buf[100], buf[200]);
            block_count++;
        }

        // Blink LED every block
        s_led_state = !s_led_state;
        //blink_led();
        
        // Small yield to prevent watchdog timeout
        taskYIELD();
    }
}

void app_main(void) {
    configure_led();
    
    char *p = "s = stepsequencer([1, 1.333], 12); ss = stepsequencer([1, 2, 3, 4], 480);  lin = linenvelope(ss, 0.001, 0.05, 0.001, 0.07); c = sawtoothosc(s * ss * 110); lowpassfilter(MAIN, 1000 * lin);";
    int numTokens = 0;

    Token *tokens = tokenize(p, &numTokens);

    SlangInterpreter *interpreter = createSlangInterpreter(tokens, numTokens);
    
    core = createBufferCore(interpreter, SAMPLE_RATE, BLOCK_SIZE);

    interpret(interpreter);  // MUST interpret before createBufferCore


    ESP_LOGI(TAG, "Slang interpreter initialized");
    
    init_i2s();
    
    ESP_LOGI(TAG, "Starting audio rendering on core %d", xPortGetCoreID());
    
    size_t bytes_written;
    int led_counter = 0;
    
    while(1) {
        float *buf = renderBuffer(core);
        
        if (buf == NULL) {
            ESP_LOGE(TAG, "renderBuffer returned NULL!");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Optimized conversion loop
        for (int i = 0; i < BLOCK_SIZE; i++) {
            float sample = buf[i];
            
            // Fast clamp using ternary operators
            sample = (sample > 1.0f) ? 1.0f : ((sample < -1.0f) ? -1.0f : sample);
            
            // Convert and duplicate in one step
            uint16_t sample_16 = (uint16_t)((int16_t)(sample * 32767.0f));
            i2s_buffer[i * 2] = sample_16;      // Left
            i2s_buffer[i * 2 + 1] = sample_16;  // Right
        }

        esp_err_t ret = i2s_channel_write(tx_handle, i2s_buffer, BLOCK_SIZE * 2 * sizeof(uint16_t), &bytes_written, portMAX_DELAY);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write error: %d", ret);
        }

        if (++led_counter >= 100) {
            s_led_state = !s_led_state;
            blink_led();
            led_counter = 0;
        }
    }
}