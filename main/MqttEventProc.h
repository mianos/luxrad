#pragma once
#include <string>
#include "Events.h"
#include "SettingsManager.h"
#include "MqttClient.h"
#include "JsonWrapper.h"
#include "esp_timer.h"

class MqttEventProc : public EventProc {
public:
    MqttEventProc(SettingsManager& settings_in, MqttClient& mqtt_in)
        : settings(settings_in), mqtt(mqtt_in) {}

    void Detected(Value* value_ptr) override {
        JsonWrapper doc;
        doc.AddItem("event", std::string("detected"));
        if (value_ptr) value_ptr->toJson(doc);  // includes distance and strength
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

    void Cleared() override {
        JsonWrapper doc;
        doc.AddItem("event", std::string("cleared"));
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

    void TrackingUpdate(Value* value_ptr) override {
        if (!value_ptr) return;
        JsonWrapper doc;
        doc.AddItem("event", std::string("tracking"));
        value_ptr->toJson(doc);                 // includes distance and strength
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

    void PresenceUpdate(Value* value_ptr) override {
        JsonWrapper doc;
        doc.AddItem("event", std::string("presence"));
        if (value_ptr) value_ptr->toJson(doc);  // includes distance and strength
        doc.AddTime();
        mqtt.publish(topicRadar(), doc.ToString());
    }

private:
    SettingsManager& settings;
    MqttClient& mqtt;

    std::string topicRadar() const {
        return "tele/" + settings.sensorName + "/radar";
    }
};

