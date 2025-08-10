#include <cstring>
#include <cstdio>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "apds9960.h"
}

#include "LD1125.h"
#include "DebounceRadar.h"
#include "RadarSensor.h"
#include "Events.h"
#include "JsonWrapper.h"

static const char* kTag = "apds9960_main";
static const char* kRadarTag = "ld1125_main";

static constexpr gpio_num_t kPinScl = GPIO_NUM_7;
static constexpr gpio_num_t kPinSda = GPIO_NUM_6;
static constexpr gpio_num_t kPinVl  = GPIO_NUM_21;

static constexpr int kI2cFreqHz = 400000;
static constexpr uint8_t kApdsAddr = APDS9960_I2C_ADDRESS;

static constexpr uart_port_t kUartPort = UART_NUM_1;
static constexpr gpio_num_t   kUartTx  = GPIO_NUM_3;
static constexpr gpio_num_t   kUartRx  = GPIO_NUM_2;

class PrintEP : public EventProc {
public:
    void Detected(Value* value_ptr) override {
        if (!value_ptr) return;
        JsonWrapper doc;
        doc.AddItem("event", std::string("detected"));
        value_ptr->toJson(doc);
        ESP_LOGI(kRadarTag, "%s", doc.ToString().c_str());
    }

    void Cleared() override {
        JsonWrapper doc;
        doc.AddItem("event", std::string("cleared"));
        ESP_LOGI(kRadarTag, "%s", doc.ToString().c_str());
    }

    void TrackingUpdate(Value* value_ptr) override {
        if (!value_ptr) return;
        const uint32_t interval_ms = static_cast<uint32_t>(CONFIG_LD1125_TRACKING_INTERVAL_MS);
        static uint32_t last_ms = 0;
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (interval_ms == 0 || now_ms - last_ms >= interval_ms) {
            JsonWrapper doc;
            doc.AddItem("event", std::string("tracking"));
            value_ptr->toJson(doc);
            ESP_LOGI(kRadarTag, "%s", doc.ToString().c_str());
            last_ms = now_ms;
        }
    }

    void PresenceUpdate(Value* value_ptr) override {
        const uint32_t interval_ms = static_cast<uint32_t>(CONFIG_LD1125_PRESENCE_INTERVAL_MS);
        static uint32_t last_ms = 0;
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (interval_ms == 0 || now_ms - last_ms >= interval_ms) {
            JsonWrapper doc;
            doc.AddItem("event", std::string("presence"));
            if (value_ptr) value_ptr->toJson(doc);
            ESP_LOGI(kRadarTag, "%s", doc.ToString().c_str());
            last_ms = now_ms;
        }
    }
};

static void RadarTask(void* arg) {
    (void)arg;

    PrintEP event_processor;

    LD1125 ld1125(&event_processor, kUartPort);
    ld1125.verifyTestMode();

    DebounceRadar debounced(&ld1125, &event_processor, 200);

    for (;;) {
        debounced.process(0.0f);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main(void) {
    esp_err_t esp_result;

    gpio_config_t vl_cfg;
    std::memset(&vl_cfg, 0, sizeof(vl_cfg));
    vl_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(kPinVl));
    vl_cfg.mode = GPIO_MODE_OUTPUT;
    vl_cfg.intr_type = GPIO_INTR_DISABLE;
    vl_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    vl_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    esp_result = gpio_config(&vl_cfg);
    if (esp_result != ESP_OK) {
        ESP_LOGE(kTag, "gpio_config(VL) failed: %s", esp_err_to_name(esp_result));
        return;
    }

    gpio_set_level(kPinVl, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_master_bus_config_t bus_cfg;
    std::memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = kPinSda;
    bus_cfg.scl_io_num = kPinScl;
    bus_cfg.glitch_ignore_cnt = 0;
    bus_cfg.flags.enable_internal_pullup = 1;

    i2c_master_bus_handle_t bus_handle = nullptr;
    esp_result = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (esp_result != ESP_OK) {
        ESP_LOGE(kTag, "i2c_new_master_bus failed: %s", esp_err_to_name(esp_result));
        return;
    }

    apds9960_handle_t sensor = apds9960_create(bus_handle, kApdsAddr);
    if (sensor == nullptr) {
        ESP_LOGE(kTag, "apds9960_create failed");
        return;
    }

    uint8_t device_id = 0;
    if (apds9960_get_deviceid(sensor, &device_id) == ESP_OK) {
        ESP_LOGI(kTag, "APDS9960 ID: 0x%02X", device_id);
    } else {
        ESP_LOGW(kTag, "Failed to read device ID");
    }

    apds9960_set_timeout(sensor, 1000);

    if (apds9960_enable(sensor, true) != ESP_OK) {
        ESP_LOGE(kTag, "Enable power failed");
        return;
    }

    apds9960_set_adc_integration_time(sensor, 10);
    apds9960_set_ambient_light_gain(sensor, APDS9960_AGAIN_4X);
    apds9960_enable_color_interrupt(sensor, false);

    if (apds9960_enable_color_engine(sensor, true) != ESP_OK) {
        ESP_LOGE(kTag, "Enable ALS failed");
        return;
    }

    uart_config_t uart_cfg;
    std::memset(&uart_cfg, 0, sizeof(uart_cfg));
    uart_cfg.baud_rate  = 115200;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_result = uart_param_config(kUartPort, &uart_cfg);
    if (esp_result != ESP_OK) {
        ESP_LOGE(kRadarTag, "uart_param_config failed: %s", esp_err_to_name(esp_result));
        return;
    }
    esp_result = uart_set_pin(kUartPort, kUartTx, kUartRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (esp_result != ESP_OK) {
        ESP_LOGE(kRadarTag, "uart_set_pin failed: %s", esp_err_to_name(esp_result));
        return;
    }
    esp_result = uart_driver_install(kUartPort, 2048, 0, 0, nullptr, 0);
    if (esp_result != ESP_OK) {
        ESP_LOGE(kRadarTag, "uart_driver_install failed: %s", esp_err_to_name(esp_result));
        return;
    }

    xTaskCreate(RadarTask, "radar_task", 4096, nullptr, 5, nullptr);

    ESP_LOGI(kTag, "Reading lux…");

    for (;;) {
        uint16_t red = 0;
        uint16_t green = 0;
        uint16_t blue = 0;
        uint16_t clear_ch = 0;

        if (!apds9960_color_data_ready(sensor)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (apds9960_get_color_data(sensor, &red, &green, &blue, &clear_ch) != ESP_OK) {
            ESP_LOGW(kTag, "Color read failed");
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        float lux = apds9960_calc_lux_from_rgb(red, green, blue);
        ESP_LOGI(kTag, "R:%u G:%u B:%u C:%u  Lux:%.2f", red, green, blue, clear_ch, lux);

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

