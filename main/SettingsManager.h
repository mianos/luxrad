#pragma once
#include <string>
#include <type_traits>
#include <vector>
#include <utility>

#include "esp_log.h"
#include "NvsStorageManager.h"
#include "JsonWrapper.h"

class SettingsManager {
    NvsStorageManager nvs;
public:
    using ChangeList = std::vector<std::pair<std::string, std::string>>;

    SettingsManager(NvsStorageManager& nvs_in) : nvs(nvs_in) {
        loadSettings();
    }

    std::string mqttBrokerUri = "mqtt://mqtt2.mianos.com";
    std::string mqttUserName = "";
    std::string mqttUserPassword = "";
    std::string sensorName = "ldr";
    std::string tz = "AEST-10AEDT,M10.1.0,M4.1.0/3";
    std::string ntpServer = "time.google.com";
    int luxPeriodSec = 1;
    int presencePeriodSec = 1;

    std::string convertChangesToJson(const SettingsManager::ChangeList& changes) {
        cJSON* root = cJSON_CreateObject();
        for (const auto& pair_item : changes) {
            const std::string& key = pair_item.first;
            const std::string& value = pair_item.second;
            cJSON_AddStringToObject(root, key.c_str(), value.c_str());
        }
        char* rawJson = cJSON_Print(root);
        std::string jsonResponse(rawJson);
        cJSON_Delete(root);
        free(rawJson);
        return jsonResponse;
    }

    void loadSettings() {
        std::string value;

        nvs.retrieve("mqttBrokerUri", mqttBrokerUri);
        nvs.retrieve("mqttUserName", mqttUserName);
        nvs.retrieve("mqttPassword", mqttUserPassword);
        nvs.retrieve("sensorName", sensorName);
        nvs.retrieve("tz", tz);
        nvs.retrieve("ntpServer", ntpServer);
        if (nvs.retrieve("luxPeriodSec", value)) luxPeriodSec = std::stoi(value);
        if (nvs.retrieve("presencePeriodSec", value)) presencePeriodSec = std::stoi(value);
    }

    void toJsonWrapper(JsonWrapper& json) const {
        json.AddItem("mqttBrokerUri", mqttBrokerUri);
        json.AddItem("mqttUserName", mqttUserName);
        json.AddItem("mqttPassword", mqttUserPassword);
        json.AddItem("sensorName", sensorName);
        json.AddItem("tz", tz);
        json.AddItem("ntpServer", ntpServer);
        json.AddItem("luxPeriodSec", luxPeriodSec);
        json.AddItem("presencePeriodSec", presencePeriodSec);
    }

    std::string toJson() const {
        JsonWrapper json;
        toJsonWrapper(json);
        return json.ToString();
    }

    ChangeList updateFromJsonString(const std::string& jsonString) {
        const JsonWrapper json = JsonWrapper::Parse(jsonString);
        return updateFromJsonWrapper(json);
    }

    ChangeList updateFromJsonWrapper(const JsonWrapper& json) {
        ChangeList changes;
        updateFieldIfChanged(json, "mqttBrokerUri", mqttBrokerUri, changes);
        updateFieldIfChanged(json, "mqttUserName", mqttUserName, changes);
        updateFieldIfChanged(json, "mqttPassword", mqttUserPassword, changes);
        updateFieldIfChanged(json, "sensorName", sensorName, changes);
        updateFieldIfChanged(json, "tz", tz, changes);
        updateFieldIfChanged(json, "ntpServer", ntpServer, changes);
        updateFieldIfChanged(json, "luxPeriodSec", luxPeriodSec, changes);
        updateFieldIfChanged(json, "presencePeriodSec", presencePeriodSec, changes);

        for (const auto& pair_item : changes) {
            const std::string& key = pair_item.first;
            const std::string& value = pair_item.second;
            nvs.store(key, value);
        }
        return changes;
    }

private:
    template <typename T>
    void updateFieldIfChanged(const JsonWrapper& json,
                              const std::string& key,
                              T& field,
                              SettingsManager::ChangeList& changes) {
        if (json.ContainsField(key)) {
            T newValue;
            if (json.GetField(key, newValue)) {
                if (newValue != field) {
                    field = newValue;
                    if constexpr (std::is_same_v<T, std::string>) {
                        changes.emplace_back(key, field);
                    } else {
                        changes.emplace_back(key, std::to_string(field));
                    }
                }
            } else {
                ESP_LOGE("SettingsUpdate", "Failed to retrieve new value for %s", key.c_str());
            }
        }
    }
};

