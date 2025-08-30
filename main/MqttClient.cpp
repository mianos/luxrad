#include <string>
#include <vector>
#include <deque>
#include <regex>
#include <algorithm>

#include "esp_log.h"
#include "cJSON.h"

#include "JsonWrapper.h"
#include "MqttClient.h"

static const char* kMqttTag = "MqttClient";

MqttClient::MqttClient(SettingsManager& settings_in)
    : settings(settings_in) {
  connected_sem = xSemaphoreCreateBinary();

  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.broker.address.uri = settings.mqttBrokerUri.c_str();
  mqtt_cfg.credentials.username = settings.mqttUserName.c_str();
  mqtt_cfg.credentials.client_id = settings.sensorName.c_str();
  mqtt_cfg.credentials.authentication.password = settings.mqttUserPassword.c_str();

  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(
      client,
      static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
      mqtt_event_handler,
      this);
  registerHandlers();
}

MqttClient::~MqttClient() {
  if (client != nullptr) {
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    client = nullptr;
  }
  if (connected_sem != nullptr) {
    vSemaphoreDelete(connected_sem);
    connected_sem = nullptr;
  }
}

void MqttClient::start() {
  esp_mqtt_client_start(client);
}

void MqttClient::wait_for_connection() {
  if (xSemaphoreTake(connected_sem, portMAX_DELAY) == pdTRUE) {
    ESP_LOGI(kMqttTag, "MQTT connected");
    xSemaphoreGive(connected_sem);
  } else {
    ESP_LOGE(kMqttTag, "MQTT connection wait failed");
  }
}

void MqttClient::publish(std::string topic, std::string data) {
  if (xSemaphoreTake(connected_sem, 0) == pdTRUE) {
    esp_mqtt_client_publish(client, topic.c_str(), data.c_str(), 0, 1, 0);
    xSemaphoreGive(connected_sem);
  } else {
    messageQueue.emplace_back(std::make_pair(std::move(topic), std::move(data)));
  }
}

void MqttClient::flushMessageQueue() {
  while (!messageQueue.empty()) {
    const auto& queued = messageQueue.front();
    esp_mqtt_client_publish(client, queued.first.c_str(), queued.second.c_str(), 0, 1, 0);
    messageQueue.pop_front();
  }
}

void MqttClient::subscribe(std::string topic) {
  if (std::find(subscriptions.begin(), subscriptions.end(), topic) == subscriptions.end()) {
    subscriptions.push_back(topic);
  }
  if (xSemaphoreTake(connected_sem, 0) == pdTRUE) {
    esp_mqtt_client_subscribe(client, topic.c_str(), 0);
    xSemaphoreGive(connected_sem);
  }
}

void MqttClient::resubscribe() {
  for (const auto& topic : subscriptions) {
    esp_mqtt_client_subscribe(client, topic.c_str(), 0);
  }
}

void MqttClient::registerHandlers() {
  const std::string& device = settings.sensorName;

  std::vector<HandlerBinding> local_handlers = {
      {
        "cmnd/+/settings",
        std::regex("cmnd/" + device + "/settings"),
        [](MqttClient* client_ptr, const std::string& topic, cJSON* data) {
          JsonWrapper json_data(data);
          auto& s = client_ptr->getSettings();
          auto changes = s.updateFromJsonWrapper(json_data);

          JsonWrapper ack;
          ack.AddItem("topic", topic);
          ack.AddItem("applied", s.convertChangesToJson(changes));
          ack.AddTime();
          client_ptr->publish("tele/" + s.sensorName + "/settings", ack.ToString());
        }
      }
  };

  for (const auto& hb : local_handlers) {
    subscribe(hb.subscriptionTopic);
    bindings.push_back({"", hb.matchPattern, hb.handler});
  }
}

void MqttClient::dispatchEvent(MqttClient* client_ptr,
                               const std::string& topic,
                               cJSON* data) {
  for (const auto& binding : client_ptr->bindings) {
    if (std::regex_match(topic, binding.matchPattern)) {
      if (binding.handler) {
        binding.handler(client_ptr, topic, data);
        return;
      }
    }
  }
  ESP_LOGW("MQTT_DISPATCH", "Unhandled topic: %s", topic.c_str());
}

void MqttClient::mqtt_event_handler(void* handler_args,
                                    esp_event_base_t base,
                                    int32_t event_id,
                                    void* event_data) {
  auto* self = static_cast<MqttClient*>(handler_args);
  auto* event = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(kMqttTag, "MQTT_EVENT_CONNECTED");
      xSemaphoreGive(self->connected_sem);
      self->resubscribe();
      self->flushMessageQueue();
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(kMqttTag, "MQTT_EVENT_DISCONNECTED");
      break;
    case MQTT_EVENT_DATA: {
      std::string topic(event->topic, event->topic_len);
      std::string payload(event->data, event->data_len);
      auto json_payload = JsonWrapper::Parse(payload);
      if (!json_payload.Empty()) {
        dispatchEvent(self, topic, json_payload.Release());
      } else {
        const char* err = cJSON_GetErrorPtr();
        if (err != nullptr) {
          ESP_LOGE(kMqttTag, "JSON parse error: %s", err);
        }
      }
    } break;
    case MQTT_EVENT_SUBSCRIBED:
    case MQTT_EVENT_UNSUBSCRIBED:
    case MQTT_EVENT_PUBLISHED:
    case MQTT_EVENT_ERROR:
    default:
      break;
  }
}

SettingsManager& MqttClient::getSettings() {
  return settings;
}

