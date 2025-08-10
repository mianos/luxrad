#include "DebounceRadar.h"
#include "esp_timer.h"

DebounceRadar::DebounceRadar(RadarSensor* sensor,
                             EventProc* event_processor,
                             uint32_t interval_ms)
    : RadarSensor(event_processor),
      sensor_(sensor),
      intervalMs_(interval_ms),
      nextSendTimeMs_(0) {
}

std::vector<std::unique_ptr<Value>> DebounceRadar::get_decoded_radar_data() {
    uint64_t now_ms = esp_timer_get_time() / 1000;
    auto raw_values = sensor_->get_decoded_radar_data();

    if (!raw_values.empty()) {
        if (lastSent_.empty() || !Equal(raw_values, lastSent_)) {
            lastSent_ = Clone(raw_values);
            nextSendTimeMs_ = now_ms + intervalMs_;
            return raw_values;
        }
        pending_ = Clone(raw_values);
    }

    if (now_ms >= nextSendTimeMs_ && !pending_.empty()) {
        auto to_send = std::move(pending_);
        pending_.clear();
        lastSent_ = Clone(to_send);
        nextSendTimeMs_ = now_ms + intervalMs_;
        return to_send;
    }

    return {};
}

std::vector<std::unique_ptr<Value>> DebounceRadar::Clone(
    const std::vector<std::unique_ptr<Value>>& src) {
    std::vector<std::unique_ptr<Value>> dst;
    dst.reserve(src.size());
    for (const auto& value_ptr : src) {
        dst.push_back(value_ptr->clone());
    }
    return dst;
}

bool DebounceRadar::Equal(
    const std::vector<std::unique_ptr<Value>>& a,
    const std::vector<std::unique_ptr<Value>>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t index = 0; index < a.size(); ++index) {
        if (!a[index]->isEqual(*b[index])) {
            return false;
        }
    }
    return true;
}

