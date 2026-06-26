#pragma once
#include <atomic>
#include <string>
#include "Events.h"
#include "Settings.h"
#include "MqttClient.h"
#include "JsonWrapper.h"
#include "esp_timer.h"

class MqttEventProc : public EventProc {
public:
    MqttEventProc(Settings& settings_in, MqttClient& mqtt_in,
                  const std::atomic<float>& lux_ref)
        : settings(settings_in), mqtt(mqtt_in), lastPresenceMs(0),
          lastLux(lux_ref) {}

    void Detected(Value* value_ptr) override {
        JsonWrapper doc;
        doc.AddItem("event", std::string("detected"));
        if (value_ptr) {
            value_ptr->toJson(doc);
        }
        addCurrentLux(doc);
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

    void Cleared() override {
        JsonWrapper doc;
        doc.AddItem("event", std::string("cleared"));
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

    // Tracking disabled per request
    void TrackingUpdate(Value* /*value_ptr*/) override {
        return;
    }

    void PresenceUpdate(Value* value_ptr) override {
        const uint32_t nowMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        const int periodSec = settings.presencePeriodSec;
        if (periodSec <= 0) {
            return;
        }
        const uint32_t requiredMs = static_cast<uint32_t>(periodSec) * 1000U;
        if (lastPresenceMs != 0 && (nowMs - lastPresenceMs) < requiredMs) {
            return;
        }
        lastPresenceMs = nowMs;

        JsonWrapper doc;
        doc.AddItem("event", std::string("presence"));
        if (value_ptr) {
            value_ptr->toJson(doc);
        }
        addCurrentLux(doc);
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

private:
    Settings& settings;
    MqttClient& mqtt;
    uint32_t lastPresenceMs;
    const std::atomic<float>& lastLux;

    std::string topicRadar() const {
        return "tele/" + settings.sensorName + "/radar";
    }

    // Attach the most recent ambient-light reading (updated by luxTask) so a
    // presence/detected event carries the current LDR for light control. Skips
    // it only when no valid reading exists yet (sensor absent or lux disabled).
    void addCurrentLux(JsonWrapper& doc) const {
        const float lux = lastLux.load(std::memory_order_relaxed);
        if (lux >= 0.0f) {
            doc.AddItem("lux", lux);
        }
    }
};

