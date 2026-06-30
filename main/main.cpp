// ldr3 — ESP32-C3 ambient-light sensor on ESP-IDF v6.
//
// An APDS9960 ambient-light sensor (I2C) reports lux over MQTT. Settings live in
// NVS and are adjustable over MQTT (cmnd/<name>/settings) and HTTP (/config). A
// web server exposes /healthz, /config and OTA (/firmware) with rollback
// verification. Shared infrastructure (wifimanager, mqttwrapper, webserver,
// jsonwrapper, nvsstoragemanager) comes from the mianesp components.

#include <atomic>
#include <cstring>
#include <ctime>
#include <regex>
#include <string>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "esp_ota_ops.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

extern "C" {
#include "apds9960.h"
}

#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include "Settings.h"
#include "WifiManager.h"
#include "MqttClient.h"
#include "WebServer.h"
#include "LdrWebServer.h"

static const char* kTag = "ldr3";

static constexpr gpio_num_t kPinScl = GPIO_NUM_7;
static constexpr gpio_num_t kPinSda = GPIO_NUM_6;
static constexpr gpio_num_t kPinVl  = GPIO_NUM_21;

static constexpr uint8_t kApdsAddr = APDS9960_I2C_ADDRESS;

// Reference integration time the published lux is normalised to, so the value
// reflects brightness rather than the integration window (see luxTask). Gain is
// fixed (4x), so it needs no normalisation; if gain ever varies, divide by it
// here too.
static constexpr int kLuxRefIntegrationMs = 100;

static std::atomic<float> g_last_lux{-1.0f};

namespace {

struct App {
    Settings*    settings;
    MqttClient*  mqtt;
    WiFiManager* wifi;
};

std::string uptimeString() {
    uint32_t seconds = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t days = seconds / 86400; seconds %= 86400;
    uint32_t hours = seconds / 3600; seconds %= 3600;
    uint32_t minutes = seconds / 60;
    return std::to_string(days) + "d " + std::to_string(hours) + "h " +
           std::to_string(minutes) + "m";
}

std::string localIp() {
    char buf[16] = "0.0.0.0";
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        esp_ip4addr_ntoa(&ip.ip, buf, sizeof(buf));
    }
    return std::string(buf);
}

// ---- MQTT command handlers (cmnd/<name>/<cmd>) ----

esp_err_t handleSettings(MqttClient* client, const std::string& topic,
                         const JsonWrapper& d, void* ctx) {
    auto* app = static_cast<App*>(ctx);
    auto changes = app->settings->loadFromJson(d);
    app->settings->save();
    app->settings->log();
    std::string ack = Settings::changesToJson(changes);
    client->publish("tele/" + app->settings->sensorName + "/settingsack", ack);
    ESP_LOGI(kTag, "applied settings for %s: %s", topic.c_str(), ack.c_str());
    return ESP_OK;
}

esp_err_t handleRestart(MqttClient*, const std::string&, const JsonWrapper&, void*) {
    ESP_LOGW(kTag, "restart requested");
    esp_restart();
    return ESP_OK;
}

esp_err_t handleReprovision(MqttClient*, const std::string&, const JsonWrapper&, void* ctx) {
    ESP_LOGW(kTag, "reprovision requested");
    static_cast<App*>(ctx)->wifi->clear();  // clears Wi-Fi creds and restarts
    return ESP_OK;
}

// Periodic ambient-light publishing. Owns the APDS9960 handle (may be null if
// the sensor is absent — the task then idles).
struct LuxCtx {
    apds9960_handle_t sensor;
    Settings*         settings;
    MqttClient*       mqtt;
};

void luxTask(void* arg) {
    auto* ctx = static_cast<LuxCtx*>(arg);
    uint32_t last_ms = 0;
    int applied_integration_ms = -1;  // forces an apply on the first iteration
    for (;;) {
        // Apply the configured ADC integration time, picking up live changes
        // made over MQTT (cmnd/<name>/settings) or HTTP (/config). The driver
        // clamps the register, so out-of-range values are harmless.
        if (ctx->sensor) {
            const int want = ctx->settings->luxIntegrationMs;
            if (want > 0 && want != applied_integration_ms) {
                apds9960_set_adc_integration_time(ctx->sensor, static_cast<uint16_t>(want));
                applied_integration_ms = want;
                ESP_LOGI(kTag, "APDS9960 integration time set to %d ms", want);
            }
        }

        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        const int period_sec = ctx->settings->luxPeriodSec;

        if (ctx->sensor && period_sec > 0 &&
            (last_ms == 0 || now_ms - last_ms >= static_cast<uint32_t>(period_sec) * 1000U)) {
            uint16_t red = 0, green = 0, blue = 0, clear_ch = 0;
            if (apds9960_color_data_ready(ctx->sensor) &&
                apds9960_get_color_data(ctx->sensor, &red, &green, &blue, &clear_ch) == ESP_OK) {
                // raw: un-normalised illuminance index — scales with integration
                // time, so a bigger number / finer resolution but not comparable
                // across luxIntegrationMs changes.
                const float raw = apds9960_calc_lux_from_rgb(red, green, blue);
                // lux: normalised to kLuxRefIntegrationMs so it tracks brightness,
                // not the integration window. The longer integration still buys
                // resolution (more counts), but the value stays stable when
                // luxIntegrationMs changes, so thresholds survive.
                const int integ_ms = applied_integration_ms > 0 ? applied_integration_ms
                                                                 : kLuxRefIntegrationMs;
                const float lux = raw * static_cast<float>(kLuxRefIntegrationMs) /
                                  static_cast<float>(integ_ms);
                g_last_lux.store(lux, std::memory_order_relaxed);

                JsonWrapper doc;
                doc.AddItem("red", static_cast<int>(red));
                doc.AddItem("green", static_cast<int>(green));
                doc.AddItem("blue", static_cast<int>(blue));
                doc.AddItem("clear", static_cast<int>(clear_ch));
                doc.AddItem("raw", raw);
                doc.AddItem("lux", lux);
                doc.AddTime();
                const std::string topic = "tele/" + ctx->settings->sensorName + "/lux";
                ctx->mqtt->publish(topic, doc.ToString());
                ESP_LOGI(kTag, "published %s lux=%.3f raw=%.1f (%dms)",
                         topic.c_str(), lux, raw, integ_ms);
            }
            last_ms = now_ms;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void telemetryTask(void* arg) {
    auto* app = static_cast<App*>(arg);
    const std::string base = "tele/" + app->settings->sensorName + "/";

    // Wait (bounded) for SNTP so the init timestamp is real.
    for (int i = 0; i < 20 && time(nullptr) < 1700000000; ++i) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    JsonWrapper init;
    init.AddItem("version", 5);
    init.AddTime();
    init.AddTime(false, "gmt");
    init.AddItem("hostname", app->settings->sensorName);
    init.AddItem("ip", localIp());
    init.AddItem("settings", "cmnd/" + app->settings->sensorName + "/settings");
    app->mqtt->publish(base + "init", init.ToString());

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        JsonWrapper d;
        d.AddTime();
        d.AddItem("uptime", uptimeString());
        d.AddItem("heap_free", (int)esp_get_free_heap_size());
        d.AddItem("heap_min_free", (int)esp_get_minimum_free_heap_size());
        app->mqtt->publish(base + "status", d.ToString());
    }
}

// --- OTA rollback verification ---
//
// A freshly-OTA'd image boots in PENDING_VERIFY: the bootloader rolls it back
// to the previous slot on the next reset unless the running app declares itself
// good. We declare it good only once the device is back on the network, so an
// image that boots but can't reach Wi-Fi is rolled back instead of stranding an
// unreachable device. A wired first-flash is UNDEFINED, so this never touches
// normal/bench boots.
constexpr int OTA_VERIFY_TIMEOUT_MS = 120000;
SemaphoreHandle_t s_got_ip = nullptr;

void onGotIp(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP && s_got_ip) {
        xSemaphoreGive(s_got_ip);
    }
}

void otaVerifyTask(void*) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(kTag, "OTA: image pending verify; waiting up to %ds for connectivity",
                 OTA_VERIFY_TIMEOUT_MS / 1000);
        if (xSemaphoreTake(s_got_ip, pdMS_TO_TICKS(OTA_VERIFY_TIMEOUT_MS)) == pdTRUE) {
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(kTag, "OTA: connectivity confirmed, image marked valid");
        } else {
            ESP_LOGE(kTag, "OTA: no IP within timeout; rolling back to previous image");
            esp_ota_mark_app_invalid_rollback_and_reboot();  // reboots on success
            ESP_LOGE(kTag, "OTA: rollback not possible; keeping current image");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    vTaskDelete(nullptr);
}

// Configure the VL gating pin, bring up the I2C bus and the APDS9960. Returns
// null (and logs) if the sensor is absent.
apds9960_handle_t setupLightSensor() {
    gpio_config_t vl_cfg;
    std::memset(&vl_cfg, 0, sizeof(vl_cfg));
    vl_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(kPinVl));
    vl_cfg.mode = GPIO_MODE_OUTPUT;
    if (gpio_config(&vl_cfg) != ESP_OK) {
        ESP_LOGE(kTag, "gpio_config(VL) failed");
        return nullptr;
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
    if (i2c_new_master_bus(&bus_cfg, &bus_handle) != ESP_OK) {
        ESP_LOGE(kTag, "i2c_new_master_bus failed");
        return nullptr;
    }

    apds9960_handle_t sensor = apds9960_create(bus_handle, kApdsAddr);
    if (!sensor) {
        ESP_LOGW(kTag, "APDS9960 not found; lux disabled");
        return nullptr;
    }
    uint8_t device_id = 0;
    if (apds9960_get_deviceid(sensor, &device_id) != ESP_OK) {
        ESP_LOGW(kTag, "Failed to read APDS9960 device ID");
    }
    apds9960_set_timeout(sensor, 1000);
    if (apds9960_enable(sensor, true) != ESP_OK) {
        ESP_LOGE(kTag, "APDS9960 enable power failed");
        return nullptr;
    }
    // Integration time is owned by luxTask (applies settings.luxIntegrationMs and
    // honours live changes); gain stays fixed at 4x.
    apds9960_set_ambient_light_gain(sensor, APDS9960_AGAIN_4X);
    if (apds9960_enable_color_engine(sensor, true) != ESP_OK) {
        ESP_LOGE(kTag, "APDS9960 enable ALS failed");
        return nullptr;
    }
    return sensor;
}

}  // namespace

extern "C" void app_main(void) {
    // Silence info-level logs globally (keeps the per-reading lux JSON prints
    // quiet) but keep network bring-up visible so Wi-Fi association and
    // provisioning state can be seen.
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("ldr3", ESP_LOG_INFO);  // publish topic + lux value
    esp_log_level_set("WiFiManager", ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_INFO);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);  // prints "sta ip: ..."
    esp_log_level_set("MqttClient", ESP_LOG_INFO);

    static NvsStorageManager nvs;
    static Settings settings(nvs);
    settings.log();

    // Created before Wi-Fi starts so the got-IP handler can signal it.
    s_got_ip = xSemaphoreCreateBinary();

    // Wi-Fi: provisions (ESP-Touch v2) on first boot, else reconnects. onGotIp
    // feeds OTA rollback verification; publishes queue until MQTT connects.
    static WiFiManager wifi(nvs, onGotIp, nullptr);
    std::string host = settings.sensorName;
    wifi.configSetHostName(host);

    // NTP time in the configured timezone.
    setenv("TZ", settings.tz.c_str(), 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, settings.ntpServer.c_str());
    esp_sntp_init();

    // MQTT (plain mqtt://, optional creds, Last-Will on the status topic).
    static std::string statusTopic = "tele/" + settings.sensorName + "/status";
    static std::string lwt = "{\"status\":\"offline\"}";
    static esp_mqtt_client_config_t mcfg = {};
    mcfg.broker.address.uri                  = settings.mqttBrokerUri.c_str();
    mcfg.credentials.client_id               = settings.sensorName.c_str();
    mcfg.credentials.username                = settings.mqttUserName.c_str();
    mcfg.credentials.authentication.password = settings.mqttUserPassword.c_str();
    mcfg.session.last_will.topic             = statusTopic.c_str();
    mcfg.session.last_will.msg               = lwt.c_str();
    mcfg.session.last_will.msg_len           = (int)lwt.size();
    mcfg.session.last_will.qos               = 1;
    static MqttClient mqtt(mcfg, settings.sensorName);

    static App app{ &settings, &mqtt, &wifi };

    const std::string b = "cmnd/" + settings.sensorName + "/";
    mqtt.registerHandler(b + "settings",    std::regex(b + "settings"),    handleSettings,    &app);
    mqtt.registerHandler(b + "restart",     std::regex(b + "restart"),     handleRestart,     &app);
    mqtt.registerHandler(b + "reprovision", std::regex(b + "reprovision"), handleReprovision, &app);
    mqtt.start();

    // Web server: base /healthz, /reset, /set_hostname plus /config and OTA.
    static WebContext webctx(&wifi);
    static LdrWebServer web(&webctx, settings, g_last_lux);
    web.start();

    // OTA: verify a freshly-OTA'd image once it's back online; roll back if not.
    xTaskCreate(otaVerifyTask, "ota_verify", 4096, nullptr, 4, nullptr);

    apds9960_handle_t sensor = setupLightSensor();
    static LuxCtx lux_ctx{ sensor, &settings, &mqtt };

    xTaskCreate(luxTask, "lux", 4096, &lux_ctx, 4, nullptr);
    xTaskCreate(telemetryTask, "telemetry", 4096, &app, 4, nullptr);

    ESP_LOGI(kTag, "ldr3 started");
}
