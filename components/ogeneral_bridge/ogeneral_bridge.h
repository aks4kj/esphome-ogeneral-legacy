#pragma once
// O General / Fujitsu General legacy wired-remote bus (tested on ARG25ALC): transparent MITM between the wall remote and the AC + command injection from HA.
//
// Physical (verified on the bus from who reacts first to a key press). Default pins are for a Seeed XIAO ESP32-C6:
//   side A (GPIO17 RX / GPIO16 TX) = WALL REMOTE   -- the remote is the bus master: polls "80 D7 xx F7" every ~240 ms
//   side B (GPIO19 RX / GPIO20 TX) = AC INDOOR UNIT -- answers "80 E7 xx B7 7F" (xx: E8 = on, EC = off)
// Bus: single wire, 500 baud, frame per byte = start + 8 data (LSB first) + 9th bit + stop. Bit ~1.985 ms (remote).
// A key press makes the remote send a command frame INSTEAD of its poll (e.g. OFF = "80 DB FC", AC acks "80 EB FC").
//
// Forwarding is bit-level (ISR copies whichever side pulls low to the other side), so it adds only microseconds.
// Injection: when a packet is queued, the next remote poll is cut off (not forwarded to the AC) and our frame is sent
// to the AC instead, starting on the same edge as the remote's poll. The AC's answer to it is not forwarded back.
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "soc/gpio_sig_map.h"
#include "rom/gpio.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include <string>
#include <cstring>

namespace esphome {
namespace ogeneral_bridge {

static const char *const TAG = "ogeneral_bridge";

static gpio_num_t g_arx, g_atx, g_brx, g_btx;
static bool g_forward = true;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ISR code must not call flash-resident gpio_get/set_level (cache is off during flash writes) -> LL inline versions
static inline int IRAM_ATTR rd_(gpio_num_t p) { return gpio_ll_get_level(&GPIO, (uint32_t) p); }
static inline void IRAM_ATTR wr_(gpio_num_t p, int v) { gpio_ll_set_level(&GPIO, (uint32_t) p, (uint32_t) v); }

static const int64_t BLANK_US = 1500;      // ignore far-side low right after we release (TJA rise time)
static const int64_t MAX_LOW_US = 45000;   // longest legal low run is ~20 ms -> anything longer = latch
static const int64_t BIT_US = 1985;        // remote's bit time (measured)

// forwarding state
static volatile bool g_atx_low = false, g_btx_low = false;
static volatile int64_t g_atx_since = 0, g_btx_since = 0, g_atx_rel = 0, g_btx_rel = 0;
static volatile int64_t g_blank_until = 0, g_last_edge = 0, g_last_a_edge = 0;
static volatile uint32_t g_edges = 0, g_wd = 0;
static volatile bool g_block_a2b = false, g_block_b2a = false;
static volatile int64_t g_block_since = 0;

// injection queue: raw bit strings (1 = high/recessive), one packet replaces one remote poll
static const int PK_MAX = 8, PK_BITS = 192;
static uint8_t g_pk[PK_MAX][PK_BITS];
static uint16_t g_pk_n[PK_MAX];
static volatile int g_pk_head = 0, g_pk_tail = 0;
static volatile bool g_tx_active = false;
static volatile int64_t g_tx_start = 0;
static uint8_t g_tx_bits[PK_BITS];
static volatile int g_tx_nbits = 0;
static volatile uint32_t g_tx_sent = 0;

// HA power override: keep each side consistent. Byte 3 of both the remote poll "80 D7 xx F7" and the AC status
// "80 E7 xx B7 7F" is E8 (on, data bit 2 = 0) or EC (off, bit 2 = 1). While an override is active:
//   remote -> AC : bit 2 of the poll is forced to the HA state (the AC follows the poll)
//   AC -> remote : bit 2 of the status is forced to the remote's own belief (otherwise the remote shows "EE")
// Any command frame from the remote (a key press) cancels the override.
static const int64_t BPA = 1985, BPB = 2070;   // bit time of the remote / of the AC
static volatile int g_ov = -1;                 // -1 = none, 1 = force ON, 0 = force OFF
static volatile int64_t g_fa = 0, g_fb = 0;    // start of the current remote frame / AC frame
static int g_a1 = -1, g_a2 = -1, g_adec = -1, g_b1 = -1, g_b2 = -1, g_bdec = -1, g_bs = 0;
static volatile int g_belief = -1;             // remote's own bit 2 from its last poll
static volatile bool g_inv_a2b = false, g_inv_b2a = false;   // true while the forced level below applies
static volatile int g_lvl_a2b = 1, g_lvl_b2a = 1;              // level forced onto the far side (0 = low)
static volatile uint32_t g_flips_a = 0, g_flips_b = 0;
// measurement: how late the override window really started, and what each side actually received at the bit centre
static volatile int32_t g_late_a = 0, g_late_b = 0;
static volatile int64_t g_meas_fa = 0, g_meas_fb = 0;

// trace ring (state changes) + snapshot for debugging
struct Tr { int64_t t; uint8_t st; };
static Tr g_tr[1024];
static volatile uint32_t g_trn = 0;
static volatile uint8_t g_last_st = 255;
static Tr g_snap[700];
static volatile int g_snap_n = 0;
static volatile bool g_snap_ready = false;
static volatile int64_t g_snap_ref = 0;
static volatile uint32_t g_snap_until = 0;   // ring index at which to take the snapshot (0 = none)

static inline void IRAM_ATTR drive_(gpio_num_t pin, volatile bool &flag, volatile int64_t &since, volatile int64_t &rel,
                                    bool low, int64_t now) {
  if (flag == low) return;
  flag = low;
  wr_(pin, low ? 0 : 1);
  if (low) since = now; else rel = now;
}

static inline bool IRAM_ATTR queue_empty_() { return g_pk_head == g_pk_tail; }

// start sending the head packet to the AC (side B); called with g_mux held
static void IRAM_ATTR start_tx_(int64_t now) {
  int n = g_pk_n[g_pk_head];
  for (int i = 0; i < n; i++) g_tx_bits[i] = g_pk[g_pk_head][i];
  g_tx_nbits = n;
  g_pk_head = (g_pk_head + 1) % PK_MAX;
  // a frame shorter than the remote's 44-bit poll is delayed so that both end together: the AC then answers exactly when
  // the remote expects its reply, and the remote sees the AC's acknowledgement (that is how it learns the new state)
  int64_t delay = n < 44 ? (int64_t) (44 - n) * BIT_US : 0;
  g_tx_start = now + delay;
  g_tx_active = true;
  g_block_a2b = true;     // the remote's own poll must not reach the AC
  g_block_b2a = false;    // the AC's acknowledgement goes to the remote
  g_block_since = now;
  if (delay == 0) drive_(g_btx, g_btx_low, g_btx_since, g_btx_rel, g_tx_bits[0] == 0, now);
  g_tx_sent++;
}

static void IRAM_ATTR update_core_() {
  if (!g_forward) return;
  int64_t now = esp_timer_get_time();
  int arx = rd_(g_arx), brx = rd_(g_brx);
  static int last_a = 1, last_b = 1;
  if (arx != last_a || brx != last_b) {
    // remote poll start = remote falling edge after >= 25 ms of silence on both sides
    if (arx == 0 && last_a == 1 && !g_atx_low && !g_btx_low && now - g_last_edge > 25000) {
      if (g_block_a2b || g_block_b2a) { g_block_a2b = false; g_block_b2a = false; }   // previous injected cycle is over
      g_fa = now; g_fb = 0;
      g_a1 = g_a2 = g_adec = g_b1 = g_b2 = g_bdec = -1;
      g_bs = 0;
      if (!g_tx_active && !queue_empty_()) start_tx_(now);
    }
    if (arx != last_a) g_last_a_edge = now;
    last_a = arx; last_b = brx;
    g_last_edge = now;
  }
  if (now < g_blank_until) {
    if (!g_tx_active) drive_(g_btx, g_btx_low, g_btx_since, g_btx_rel, false, now);
    drive_(g_atx, g_atx_low, g_atx_since, g_atx_rel, false, now);
    return;
  }
  // watchdog: a forwarded TX held low far longer than any real byte = feedback latch -> release both, lock out briefly
  if ((g_atx_low && now - g_atx_since > MAX_LOW_US) || (!g_tx_active && g_btx_low && now - g_btx_since > MAX_LOW_US)) {
    drive_(g_atx, g_atx_low, g_atx_since, g_atx_rel, false, now);
    if (!g_tx_active) drive_(g_btx, g_btx_low, g_btx_since, g_btx_rel, false, now);
    g_blank_until = now + 100000;
    g_wd++;
    return;
  }
  // a side "talks" if its RX is low and that is not our own echo (and we did not just release it)
  bool a_ext = (arx == 0) && !g_atx_low && (now - g_atx_rel > BLANK_US);
  bool b_ext = (brx == 0) && !g_btx_low && (now - g_btx_rel > BLANK_US);

  bool want_btx, want_atx;
  if (g_tx_active) want_btx = g_btx_low;                 // tx engine owns side B
  else if (g_block_a2b) want_btx = false;
  else want_btx = g_btx_low ? (arx == 0) : a_ext;         // remote -> AC

  if (g_block_b2a) want_atx = false;
  else want_atx = g_atx_low ? (brx == 0) : b_ext;         // AC -> remote

  // AC frame start = the moment we begin forwarding the AC's first low bit to the remote (robust: no separate edge test)
  if (want_atx && !g_atx_low && b_ext && g_fa != 0 && g_fb == 0 && now - g_fa > 80000 && now - g_fa < 250000) g_fb = now;
  // override window: force the whole bit to the wanted level (receivers may sample anywhere in the bit)
  if (g_inv_a2b && !g_tx_active && !g_block_a2b) want_btx = (g_lvl_a2b == 0);
  if (g_inv_b2a && !g_block_b2a) want_atx = (g_lvl_b2a == 0);

  if (!g_tx_active && !g_block_a2b && !g_block_b2a && a_ext && b_ext) return;   // collision: keep current direction
  if (want_btx && want_atx && !g_tx_active) return;
  if (!g_tx_active) drive_(g_btx, g_btx_low, g_btx_since, g_btx_rel, want_btx, now);
  drive_(g_atx, g_atx_low, g_atx_since, g_atx_rel, want_atx, now);
}

static void IRAM_ATTR update_() {
  update_core_();
  uint8_t st = (rd_(g_arx) ? 1 : 0) | (rd_(g_brx) ? 2 : 0) | (g_atx_low ? 4 : 0) | (g_btx_low ? 8 : 0);
  if (st != g_last_st) {
    g_last_st = st;
    g_tr[g_trn & 1023].t = esp_timer_get_time();
    g_tr[g_trn & 1023].st = st;
    g_trn = g_trn + 1;
  }
}

static void IRAM_ATTR isr_(void *) {
  portENTER_CRITICAL_ISR(&g_mux);
  g_edges++;
  update_();
  portEXIT_CRITICAL_ISR(&g_mux);
}

// runs in the esp_timer task (ESP_TIMER_ISR dispatch at 250 us hangs the ESP32-C6 -> do not use)
static void IRAM_ATTR poll_cb_(void *) {
  portENTER_CRITICAL(&g_mux);
  int64_t now = esp_timer_get_time();
  if (g_tx_active) {
    int idx = now < g_tx_start ? -1 : (int) ((int32_t) (now - g_tx_start) / (int32_t) BIT_US);
    if (idx < 0) {
      // waiting for the aligned start
    } else if (idx >= g_tx_nbits) {
      g_tx_active = false;
      drive_(g_btx, g_btx_low, g_btx_since, g_btx_rel, false, now);
      if (g_snap_until == 0) g_snap_until = g_trn + 250;   // snapshot the AC's answer too
      g_snap_ref = g_tx_start;
    } else {
      drive_(g_btx, g_btx_low, g_btx_since, g_btx_rel, g_tx_bits[idx] == 0, now);
    }
  }
  // --- override windows (bit 2 of byte 3 = bit index 25 of the frame)
  if (g_fa != 0) {
    int64_t r = now - g_fa;
    if (g_a1 < 0 && r >= 27 * BPA / 2) g_a1 = rd_(g_arx);          // byte 2, bit 1 centre
    if (g_a2 < 0 && r >= 29 * BPA / 2) g_a2 = rd_(g_arx);          // byte 2, bit 2 centre (poll D7: both 1)
    // decide just before bit 25 (byte 3, bit 2): a poll (D7) gets the HA state, anything else cancels the override
    if (g_adec < 0 && r >= 25 * BPA - 400) {
      if (g_a1 == 1 && g_a2 == 1) {
        g_adec = (g_ov >= 0 && !g_block_a2b) ? 1 : 0;
        g_lvl_a2b = g_ov == 1 ? 0 : 1;
      } else {
        g_adec = 0;
        if (g_ov >= 0 && !g_block_a2b) g_ov = -1;                   // remote key press -> the user wins
        if (!g_block_a2b && g_snap_until == 0 && !g_snap_ready) { g_snap_until = g_trn + 400; g_snap_ref = g_fa; }
      }
    }
    if (!g_bs && r >= 25 * BPA + 1000) { g_bs = 1; if (g_a1 == 1 && g_a2 == 1) g_belief = rd_(g_arx); }   // remote's own bit 2 (mid-bit)
    bool inv = g_adec == 1 && r >= 25 * BPA - 400 && r < 26 * BPA;
    if (inv && !g_inv_a2b) { g_flips_a++; g_late_a = (int32_t) (r - (25 * BPA - 400)); g_meas_fa = g_fa; }
    g_inv_a2b = inv;
  }
  if (g_fb != 0) {
    int64_t r = now - g_fb;
    if (g_b1 < 0 && r >= 27 * BPB / 2) g_b1 = rd_(g_brx);
    if (g_b2 < 0 && r >= 29 * BPB / 2) g_b2 = rd_(g_brx);
    if (g_bdec < 0 && r >= 25 * BPB - 400) {
      g_bdec = (g_b1 == 1 && g_b2 == 1 && g_ov >= 0 && g_belief >= 0) ? 1 : 0;   // AC status (E7) -> show the remote its own state
      g_lvl_b2a = g_belief;
    }
    bool inv = g_bdec == 1 && r >= 25 * BPB - 400 && r < 26 * BPB;
    if (inv && !g_inv_b2a) {
      g_flips_b++;
      g_late_b = (int32_t) (r - (25 * BPB - 400));
      g_meas_fb = g_fb;
      if (g_flips_b == 6 && g_snap_until == 0 && !g_snap_ready) { g_snap_until = g_trn + 300; g_snap_ref = g_fa; }
    }
    g_inv_b2a = inv;
  }

  // remote poll is over (A quiet) -> stop blocking it; AC->remote stays blocked until the next poll or a timeout
  if (!g_tx_active && g_block_a2b && rd_(g_arx) == 1 && now - g_last_a_edge > 12000) g_block_a2b = false;
  if (!g_tx_active && g_block_b2a && now - g_block_since > 400000) g_block_b2a = false;

  if (g_snap_until != 0 && g_trn >= g_snap_until && !g_snap_ready) {
    uint32_t n = g_trn, from = n > 700 ? n - 700 : 0;
    for (uint32_t i = from; i < n; i++) g_snap[i - from] = g_tr[i & 1023];
    g_snap_n = (int) (n - from);
    g_snap_ready = true;
    g_snap_until = 0;
  }
  update_();
  portEXIT_CRITICAL(&g_mux);
}

class OGeneralBridge;
static OGeneralBridge *g_bridge = nullptr;   // used by the climate component

class OGeneralBridge : public Component {
 public:
  void set_pins(int arx, int atx, int brx, int btx) {
    g_arx = (gpio_num_t) arx; g_atx = (gpio_num_t) atx; g_brx = (gpio_num_t) brx; g_btx = (gpio_num_t) btx;
  }
  void set_forward(bool f) { g_forward = f; }
  float get_setup_priority() const override { return setup_priority::LATE; }  // after UART grabbed the RX pin

  // queue one frame as raw bits ("0"/"1", 1 = idle level); it replaces the next remote poll
  void inject_bits(const std::string &bits) {
    uint8_t buf[PK_BITS];
    int n = 0;
    for (char c : bits) {
      if (c != '0' && c != '1') continue;
      if (n < PK_BITS) buf[n++] = (c == '1');
    }
    if (n == 0) return;
    portENTER_CRITICAL(&g_mux);
    int nt = (g_pk_tail + 1) % PK_MAX;
    bool ok = nt != g_pk_head;
    if (ok) {
      memcpy(g_pk[g_pk_tail], buf, n);
      g_pk_n[g_pk_tail] = n;
      g_pk_tail = nt;
    }
    portEXIT_CRITICAL(&g_mux);
    ESP_LOGI(TAG, "inject %s: %d bits", ok ? "queued" : "DROPPED (queue full)", n);
  }

  // build a frame from bytes: start + 8 data LSB first + 9th bit + stop. 9th bit: 0 for the 2 header bytes, 1 after.
  void inject_hex(const std::string &hex) {
    std::string bits;
    int idx = 0;
    unsigned v = 0; int nib = 0;
    auto add = [&](uint8_t b) {
      bits += '0';
      for (int j = 0; j < 8; j++) bits += ((b >> j) & 1) ? '1' : '0';
      bits += (idx < 2) ? '0' : '1';
      bits += '1';
      idx++;
    };
    for (char c : hex) {
      int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
      if (d < 0) { if (nib == 1) { add(v); nib = 0; v = 0; } continue; }
      v = (v << 4) | d;
      if (++nib == 2) { add(v); nib = 0; v = 0; }
    }
    if (nib == 1) add(v);
    inject_bits(bits);
  }

  // HA power: force the on/off bit of the remote's poll (the AC follows the poll); OFF also sends the remote's own
  // OFF command frame (recorded from a real remote). The AC does NOT switch on from the poll bit alone:
  // ON needs the remote's full state command sequence (fixed recorded state 21 C / fan HIGH here; prefer send_state()).
  void set_power(bool on) {
    g_ov = on ? 1 : 0;
    if (!on) {
      inject_bits("000000001010110110110100011111111");  // 80 DB FC
    } else {
      // recorded remote frames, state byte 0x56 (21 C, fan HIGH): DD FC / poll F5 / DD EC / poll F5
      inject_bits("00000000101010111011010001111111001100110101011010111111010000001011111101000000101");
      inject_bits("00000000101011101011010000101111101010111111");
      inject_bits("00000000101010111011010001101111001100110101011010111111010000001011111101000000101");
      inject_bits("00000000101011101011010000101111101010111111");
    }
    ESP_LOGI(TAG, "power -> %s", on ? "ON" : "OFF");
  }

  // Full state command, built from the remote's recorded frames (7 captures with different B4):
  // only the 8 data bits of B4 (frame bits 37..44, LSB first) and B5 (bits 48..55) change between commands.
  //   B4 COOL: ((31 - T) << 3) | fan    FAN-only: 0xF0 | (fan << 1)    fan AUTO=2 LOW=3 MID=5 HIGH=6
  //   B5: 0xFD normal, 0xBD energy save
  void send_state(uint8_t b4, bool eco) {
    static const char *DD_FC = "00000000101010111011010001111111001100110101011010111111010000001011111101000000101";
    static const char *DD_EC = "00000000101010111011010001101111001100110101011010111111010000001011111101000000101";
    static const char *POLL_F5 = "00000000101011101011010000101111101010111111";
    const uint8_t b5 = eco ? 0xBD : 0xFD;
    std::string f1 = DD_FC, f3 = DD_EC;
    for (int j = 0; j < 8; j++) {
      f1[37 + j] = f3[37 + j] = ((b4 >> j) & 1) ? '1' : '0';
      f1[48 + j] = f3[48 + j] = ((b5 >> j) & 1) ? '1' : '0';
    }
    g_ov = 1;
    inject_bits(f1);
    inject_bits(POLL_F5);
    inject_bits(f3);
    inject_bits(POLL_F5);
    ESP_LOGI(TAG, "state -> B4=%02X B5=%02X", b4, b5);
  }

  void setup() override {
    g_bridge = this;
    gpio_config_t out = {};
    out.pin_bit_mask = (1ULL << g_atx) | (1ULL << g_btx);
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);
    gpio_set_level(g_atx, 1);
    gpio_set_level(g_btx, 1);

    gpio_config_t in = {};
    in.pin_bit_mask = (1ULL << g_arx) | (1ULL << g_brx);
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;
    in.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&in);
    // GPIO17 is UART0's native IOMUX RX pad; gpio_config() above switched the pad to plain GPIO -> re-route UART0 RX via the matrix
    esp_rom_gpio_connect_in_signal(g_arx, U0RXD_IN_IDX, false);
    esp_rom_gpio_connect_in_signal(g_brx, U1RXD_IN_IDX, false);   // listen-only UART on the AC side

    esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_LOGE(TAG, "isr service: %d", (int) e);
    gpio_isr_handler_add(g_arx, isr_, nullptr);
    gpio_isr_handler_add(g_brx, isr_, nullptr);

    esp_timer_create_args_t a = {};
    a.callback = &poll_cb_;
    a.name = "ogbridge";
    esp_timer_handle_t t;
    esp_timer_create(&a, &t);
    esp_timer_start_periodic(t, 250);  // tx bit engine + safety poll
    ESP_LOGI(TAG, "bridge up: A(remote) rx=%d tx=%d | B(AC) rx=%d tx=%d", g_arx, g_atx, g_brx, g_btx);
  }

  // level of a line (bit 0 = A/remote, bit 1 = B/AC) at time t, from the ISR-timestamped ring
  static int level_at_(int64_t t, int bit) {
    uint32_t n = g_trn;
    for (uint32_t k = 0; k < 600 && k < n; k++) {
      const Tr &e = g_tr[(n - 1 - k) & 1023];
      if (e.t <= t) return (e.st >> bit) & 1;
    }
    return -1;
  }

  void loop() override {
    int64_t tnow = esp_timer_get_time();
    if (g_meas_fb != 0 && tnow > g_meas_fb + 70000) {
      int64_t fb = g_meas_fb;
      g_meas_fb = 0;
      int64_t tc = fb + 51 * BPB / 2;                       // centre of bit 25 of the AC frame
      int rem = level_at_(tc, 0), ac = level_at_(tc, 1);
      ESP_LOGI("MEAS", "AC->remote: window late %d us | AC sent %d, remote got %d, wanted %d -> %s", (int) g_late_b, ac, rem,
               (int) g_belief, rem == g_belief ? "OK" : "FAIL");
    }
    if (g_meas_fa != 0 && tnow > g_meas_fa + 70000) {
      int64_t fa = g_meas_fa;
      g_meas_fa = 0;
      int64_t tc = fa + 51 * BPA / 2;
      int rem = level_at_(tc, 0), ac = level_at_(tc, 1);
      int want = g_ov == 1 ? 0 : 1;
      ESP_LOGI("MEAS", "remote->AC: window late %d us | remote sent %d, AC got %d, wanted %d -> %s", (int) g_late_a, rem, ac, want,
               ac == want ? "OK" : "FAIL");
    }
    if (g_snap_ready) {
      std::string o;
      char b2[40];
      int n = g_snap_n;
      int64_t ref = g_snap_ref;
      ESP_LOGI(TAG, "TRACE us relative to our tx start; st: 1=arx hi 2=brx hi 4=atx low 8=btx low");
      for (int i = 0; i < n; i++) {
        snprintf(b2, sizeof(b2), "%lld:%X ", (long long) (g_snap[i].t - ref), g_snap[i].st);
        o += b2;
        if ((i % 22) == 21 || i == n - 1) {
          ESP_LOGI(TAG, "TRACE %s", o.c_str());
          o.clear();
        }
      }
      g_snap_ready = false;
    }
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 10000) return;
    last = now;
    ESP_LOGI(TAG, "fwd=%d edges=%u wd=%u tx_sent=%u queue=%d ov=%d belief=%d flips a2b=%u b2a=%u", (int) g_forward,
             (unsigned) g_edges, (unsigned) g_wd, (unsigned) g_tx_sent, (int) ((g_pk_tail - g_pk_head + PK_MAX) % PK_MAX),
             (int) g_ov, (int) g_belief, (unsigned) g_flips_a, (unsigned) g_flips_b);
  }
};

}  // namespace ogeneral_bridge
}  // namespace esphome
