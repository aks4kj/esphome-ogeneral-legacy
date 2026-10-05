#pragma once
// O General / Fujitsu General legacy wired remote (tested: ARG25ALC, 2003, R22, cooling-only) — climate entity on top
// of the MITM bridge (components/ogeneral_bridge).
// Reads the AC side of the bus (uart on the bridge's side B), so it shows what the AC really does, not what the
// wall remote believes. Bus facts (see docs/protocol.md and the bridge header):
//   remote poll    : 80 D7 xx F7        AC status : 80 E7 xx B7 7F      (xx: E8 = on, EC = off)
//   state command  : 80 DD FC|EC B4 B5 A0 81 (remote) / 80 ED ... (AC ack)
//   B4 COOL        : ((31 - T) << 3) | fan        B4 FAN-only : 0xF0 | (fan << 1)
//   fan            : AUTO=2 LOW=3 MID=5 HIGH=6    B5 : FD normal, BD = ENERGY SAVE
// Control from HA: OFF = remote OFF frame + poll override; anything else = full state command (bridge send_state).

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/ogeneral_bridge/ogeneral_bridge.h"
#include <vector>

namespace esphome {
namespace ogeneral {

static const char *const TAG = "ogeneral";

class OGeneralClimate : public climate::Climate, public Component, public uart::UARTDevice {
 public:
  void setup() override {
    this->mode = climate::CLIMATE_MODE_OFF;
    this->target_temperature = 24;
    this->fan_mode = climate::CLIMATE_FAN_AUTO;
  }

  void loop() override {
    const uint32_t now = millis();
    while (this->available()) {
      uint8_t c;
      this->read_byte(&c);
      this->buf_.push_back(c);
      this->last_rx_ = now;
    }
    if (!this->buf_.empty() && now - this->last_rx_ > 30) {
      this->handle_frame_(this->buf_);
      this->buf_.clear();
    }
  }

  climate::ClimateTraits traits() override {
    auto t = climate::ClimateTraits();
    t.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_COOL});   // cooling-only unit; FAN-only and ECO are decoded but not exposed
    t.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM,
                               climate::CLIMATE_FAN_HIGH});
    t.set_visual_min_temperature(18);
    t.set_visual_max_temperature(30);
    t.set_visual_temperature_step(1);
    return t;
  }

  void control(const climate::ClimateCall &call) override {
    auto *br = ogeneral_bridge::g_bridge;
    if (br == nullptr) return;
    const auto mode = call.get_mode().value_or(this->mode);
    if (mode == climate::CLIMATE_MODE_OFF) {
      br->set_power(false);
    } else {
      // anything else (on, mode, temperature, fan, eco) = one full state command, like the wall remote sends
      const auto m = climate::CLIMATE_MODE_COOL;
      const float t = call.get_target_temperature().value_or(this->target_temperature);
      const auto fan = call.get_fan_mode().value_or(this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO));
      br->send_state(encode_b4_(m, t, fan), false);   // ECO not exposed
    }
    // no optimistic update: the entity follows the AC's answers read from the bus
    this->publish_state();
  }

 protected:
  std::vector<uint8_t> buf_;
  uint32_t last_rx_{0};
  climate::ClimateMode last_on_mode_{climate::CLIMATE_MODE_COOL};

  static uint8_t fan_code_(climate::ClimateFanMode f) {
    switch (f) {
      case climate::CLIMATE_FAN_LOW: return 3;
      case climate::CLIMATE_FAN_MEDIUM: return 5;
      case climate::CLIMATE_FAN_HIGH: return 6;
      default: return 2;  // AUTO
    }
  }
  static uint8_t encode_b4_(climate::ClimateMode m, float temp, climate::ClimateFanMode f) {
    const uint8_t fan = fan_code_(f);
    if (m == climate::CLIMATE_MODE_FAN_ONLY) return 0xF0 | (fan << 1);
    int t = (int) lroundf(temp);
    if (t < 18) t = 18;
    if (t > 30) t = 30;
    return (uint8_t) (((31 - t) << 3) | fan);
  }

  static bool fan_from_code_(uint8_t c, climate::ClimateFanMode &out) {
    switch (c) {
      case 2: out = climate::CLIMATE_FAN_AUTO; return true;
      case 3: out = climate::CLIMATE_FAN_LOW; return true;
      case 5: out = climate::CLIMATE_FAN_MEDIUM; return true;
      case 6: out = climate::CLIMATE_FAN_HIGH; return true;
    }
    return false;
  }

  void handle_frame_(const std::vector<uint8_t> &f) {
    bool changed = false;
    for (size_t i = 0; i + 2 < f.size(); i++) {
      // AC status "80 E7 xx": the AC's real on/off
      if (f[i] == 0x80 && f[i + 1] == 0xE7) {
        const bool on = (f[i + 2] & 0x04) == 0;
        if (!on && this->mode != climate::CLIMATE_MODE_OFF) {
          this->last_on_mode_ = this->mode;
          this->mode = climate::CLIMATE_MODE_OFF;
          changed = true;
        } else if (on && this->mode == climate::CLIMATE_MODE_OFF) {
          this->mode = this->last_on_mode_;
          changed = true;
        }
      }
      // state command / ack "80 DD|ED FC|EC b4 b5 A0 81"
      if (i + 6 < f.size() && f[i] == 0x80 && (f[i + 1] == 0xDD || f[i + 1] == 0xED) &&
          (f[i + 2] == 0xFC || f[i + 2] == 0xEC) && f[i + 5] == 0xA0 && f[i + 6] == 0x81) {
        const uint8_t b4 = f[i + 3], b5 = f[i + 4];
        climate::ClimateFanMode fan;
        climate::ClimateMode m = climate::CLIMATE_MODE_OFF;
        if ((b4 & 0xF0) == 0xF0 && (b4 & 0x01) == 0 && fan_from_code_((b4 & 0x0F) >> 1, fan)) {
          m = climate::CLIMATE_MODE_COOL;   // FAN-only set on the wall remote: shown as COOL (mode not exposed)
          this->fan_mode = fan;
        } else if (fan_from_code_(b4 & 0x07, fan)) {
          m = climate::CLIMATE_MODE_COOL;
          this->target_temperature = 31 - (b4 >> 3);
          this->fan_mode = fan;
        }
        if (m != climate::CLIMATE_MODE_OFF) {
          this->last_on_mode_ = m;
          if (this->mode != climate::CLIMATE_MODE_OFF) this->mode = m;
        }
        (void) b5;
        changed = true;
      }
    }
    if (changed) this->publish_state();
  }
};

}  // namespace ogeneral
}  // namespace esphome
