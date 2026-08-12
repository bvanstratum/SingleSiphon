/*
using a loadcell 6 click from mikroelektronika with esp32 and micro-ros
I am reading the loadcell data and publishing it to a micro-ros topic called "loadcell_data" using the geometry_msgs/WrenchStamped message type (reading stored in wrench.force.z; torque and force.x/y unused). The ADC samples continuously at 2000sps in the background; every Nth sample (LOADCELL_DECIMATION) is published immediately as its own message, each stamped with the board's agent-synced epoch time (rmw_uros_sync_session() + rmw_uros_epoch_nanos()) rather than relying on receive-time ordering, which isn't precise/even enough at this rate for Foxglove to plot correctly. A plain std_msgs/Float32 (no header) was tried first but has no per-sample timestamp; a batched Float32MultiArray was tried before that but failed outright — this library's UDP transport has a hardcoded 512-byte MTU and a ~200-float array message exceeds it every time.

WiFi/agent config and the connect/reconnect pattern below are copied from the
main actuator firmware (esp32_micro_ros/esp32_micro_ros.ino) so this board
joins the same micro-ROS agent over WiFi and can run standalone off a
battery — no USB/host tether needed, just power and the agent already
running on the laptop.

MAX11270 driver below is written against the MAX11270 datasheet (register
map, command byte format, CTRL1/CTRL2 bit layout) and cross-checked against
MikroElektronika's own official Load Cell 6 Click driver
(https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/loadcell6)
— register addresses/bit positions matched exactly, and the default
config + read sequence here mirrors their proven approach (unipolar mode,
single-cycle triggered reads) rather than a from-scratch guess.
*/

/*
old pinout
stm32?      | max11270  | color
d6          | syn
d9          | rdy
5v          | 5v
d3          | rst       |yellow
a3          | cs        | grn
a4          | sck       |   blu
a5          | sd0       | prpl
a6          | sd1       |gray
?3v3?(fell out)| 3v3    | wht
 gnd       |gnd         | blk

github repo for the company's official click driver (used as reference for
the MAX11270 register map / init sequence / read pattern below):
https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/loadcell6


==== load cell ==========
the load cell is CALT dyly-103-10kg
2mv/volt
6click| colors| load cell colors
e+    | red   | red
s-    |blue   | white
s+    | grn   | grn
e-    | blk   | blk
*/




#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <SPI.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/wrench_stamped.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/time_sync.h>
#include <esp_timer.h>

// ── WiFi / micro-ROS agent — same network + agent host as the main robot ──
const char* WIFI_SSID     = "sas-network";
const char* WIFI_PASS     = "mariners";
const char* AGENT_IP      = "192.168.4.100";
const uint16_t AGENT_PORT = 8888;

// ── MAX11270 pin wiring — XIAO ESP32-S3 (confirmed via Seeed's own pin
// multiplexing wiki, not guessed). D8/D9/D10 are the board's fixed hardware
// SPI pins (SCK/MISO/MOSI) — using them (rather than bit-banging on
// arbitrary GPIOs) gets correct SPI timing for free. CS/RDY/RST/SYN just
// need any free GPIO, so D0-D3 were picked to leave D4/D5 (I2C) and D6/D7
// (native USB-CDC handles Serial on this board, so these are free too) open
// for anything else later.
#define PIN_CS   D0
#define PIN_SCK  D8   // hardware SPI SCK
#define PIN_SDO  D9   // hardware SPI MISO <- MAX11270 SDO
#define PIN_SDI  D10  // hardware SPI MOSI -> MAX11270 SDI (old pinout's "sd1")
#define PIN_RDY  D1
#define PIN_RST  D2
#define PIN_SYN  D3

rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rcl_publisher_t  loadcell_publisher;
geometry_msgs__msg__WrenchStamped loadcell_msg;

// Impulse (mN*s) - trapezoidal integral of calibrated force over time,
// published on loadcell_data_impulse (same topic name/type/units as
// augment_bag_offline_impulse.py's offline computation, so live and
// post-hoc impulse are directly comparable). Resets to 0 on every tare
// (see reset_impulse(), called from both performTare() and
// applyRollingTare()) - reads as "impulse since the last tare" rather than
// an ever-growing since-boot total, which lines up with TestRunnerNode
// retaring right before each trial's pulse phase.
rcl_publisher_t  impulse_publisher;
geometry_msgs__msg__WrenchStamped impulse_msg;
static double  impulse_mNs               = 0.0;
static double  last_impulse_force_mN     = 0.0;
static int64_t last_impulse_stamp_ns     = 0;
static bool    have_last_impulse_sample  = false;

static inline void reset_impulse() {
  impulse_mNs = 0.0;
  have_last_impulse_sample = false;
}

// Tare-on-command: TestRunnerNode publishes True here (operator-triggered,
// via the settle/tare-approval loop worked out for the frequency-sweep
// experiment) to re-zero the load cell without a reflash. Low-rate,
// operator-paced - fine to spin inline in loop() rather than a dedicated
// FreeRTOS task the way the actuator board's higher-rate teleop subscriber
// needs.
rcl_subscription_t tare_command_subscriber;
std_msgs__msg__Bool tare_command_msg;
rclc_executor_t  executor;
bool tare_subscription_ready = false;

// The ADC runs the full 2000sps continuously in the background; DECIMATION
// publishes only every Nth sample (~1000sps effective) as its own message,
// each tiny enough to stay well under this library's 512-byte transport MTU.
#define LOADCELL_DECIMATION 1
static uint32_t loadcell_pollCounter = 0;

// rmw_uros_epoch_nanos() turned out to return an identical value across many
// consecutive calls (confirmed via `ros2 topic echo --field header.stamp` -
// same sec/nanosec for a dozen+ messages in a row), i.e. it isn't freely
// ticking between agent syncs the way its docs imply. Instead, capture the
// real-world offset ONCE at sync time and add the ESP32's own free-running
// microsecond timer (esp_timer_get_time(), which genuinely advances every
// call) per sample - see ROS_Connect() and loadcell_pollAndPublish().
static int64_t loadcell_epochOffsetNs = 0;

bool ros_connected = false;
unsigned long lastPingMs    = 0;

// ── MAX11270 registers (Table 8, MAX11270 datasheet) ──────────────────────
#define MAX11270_REG_STAT     0x00
#define MAX11270_REG_CTRL1    0x01
#define MAX11270_REG_CTRL2    0x02
#define MAX11270_REG_CTRL3    0x03
#define MAX11270_REG_CTRL4    0x04
#define MAX11270_REG_CTRL5    0x05
#define MAX11270_REG_DATA     0x06

// Command byte bits (Table 6/7)
#define MAX11270_CMD_START           0x80
#define MAX11270_CMD_REG_ACCESS_MODE 0x40  // MODE=1 (register access); MODE=0 for conversion command
#define MAX11270_CMD_READ            0x01  // R/W-bar bit: 1 = read, 0 = write

// STAT register bit (Table, status register)
#define MAX11270_STAT_RDY 0x0001

// Conversion command RATE[3:0] bits (Table 6 for the bit positions, Table 9
// for the sps mapping). 1010 = 2000sps in CONTINUOUS mode (SCYCLE=0) - the
// "CONTINUOUS DATA RATE, SCYCLE=0" column of Table 9, not the single-cycle
// column.
#define MAX11270_RATE_2000SPS_CONTINUOUS 0x0A

// CTRL1 bits
#define MAX11270_CTRL1_CONTSC 0x01
#define MAX11270_CTRL1_SCYCLE 0x02
#define MAX11270_CTRL1_FORMAT 0x04
#define MAX11270_CTRL1_U_B    0x08  // 1 = unipolar, 0 = bipolar
#define MAX11270_CTRL1_PD0    0x10
#define MAX11270_CTRL1_PD1    0x20
#define MAX11270_CTRL1_SYNC   0x40
#define MAX11270_CTRL1_EXTCK  0x80

// CTRL2 bits
#define MAX11270_CTRL2_PGAIN_X1   0x00
#define MAX11270_CTRL2_PGAIN_X128 0x07
#define MAX11270_CTRL2_PGAGEN     0x08
#define MAX11270_CTRL2_LPMODE     0x10
#define MAX11270_CTRL2_BUFEN      0x20

// CTRL3 bits — MikroE's default cfg writes back the POR reserved-bit state
// explicitly (0x41); no functional bits used here.
#define MAX11270_CTRL3_RESERVED 0x41

// CTRL4 bits — GPIO1/MB1 on this click board is wired straight to the load
// cell's E- (excitation return) terminal per the schematic, not to a hard
// ground rail. DIR1 configures GPIO1 as an output; DIO1 is deliberately left
// clear so that output is driven LOW, actively grounding the bridge's return
// leg. Without this write GPIO1 stays at its POR default (input / floating),
// which is exactly why E- was measured sitting at ~excitation voltage
// instead of ~0V - the bridge was never actually being excited.
#define MAX11270_CTRL4_DIO1 0x01
#define MAX11270_CTRL4_DIO2 0x02
#define MAX11270_CTRL4_DIO3 0x04
#define MAX11270_CTRL4_DIO4 0x08
#define MAX11270_CTRL4_DIR1 0x10
#define MAX11270_CTRL4_DIR2 0x20
#define MAX11270_CTRL4_DIR3 0x40

// CTRL5 bits — disable all cal-register corrections (no ADC self/system cal
// performed; load-cell calibration is done empirically via tare+scale below,
// same approach the official MikroE driver uses).
#define MAX11270_CTRL5_NOSCO  0x01
#define MAX11270_CTRL5_NOSCG  0x02
#define MAX11270_CTRL5_NOSYSO 0x04
#define MAX11270_CTRL5_NOSYSG 0x08

// PGA gain — x128, since the CALT DYLY-103 (2mV/V @ ~3V excitation) only
// outputs a few mV full-scale; x1 was leaving nearly all the ADC's range
// unused for signal.
#define PGA_GAIN_BITS MAX11270_CTRL2_PGAIN_X128

// ── Load-cell calibration ──────────────────────────────────────────────────
// From LoadCellCalibrator.ipynb's linregress fit of tare-corrected raw
// counts -> known load, negated from the raw fit so positive output means
// "load to the left" (same oriented convention as ForceCalibratorNode.py's
// calibration_slope - MUST be kept in sync with that notebook if the
// calibration is ever redone).
//
// No intercept term: the fit's ~32mN intercept was dropped on the
// assumption that it reflected the original calibration run's taring
// procedure (not retared between each weight) rather than something
// intrinsic to the sensor - since this firmware retares immediately before
// every trial (loadcell_tare_command), corrected_raw=0 is taken to mean
// force=0 by construction. NOT re-derived as a proper through-origin
// regression - if that assumption turns out wrong, redo the fit constrained
// through the origin (a different slope, not just the same one with b
// dropped) rather than re-adding this same intercept value.
//
// Applying this here (rather than downstream in ForceCalibratorNode) means
// /loadcell_data is published in real millinewtons, not raw ADC counts -
// which also means ForceFilterNode's measurement_noise_variance (tuned
// against raw-count-scale noise) needs retuning: variance scales as
// slope^2 under a linear transform, so roughly
// 3e6 * (0.08355)^2 ~= 20940 at this new scale, not the old 3e6.
#define CALIBRATION_SLOPE_MN_PER_COUNT -0.08355f

static int32_t loadcell_tare_raw = 0;

// 20000 (~10s at the 2000sps effective publish rate, LOADCELL_DECIMATION=1)
// - not the original 10, or even the earlier 2000 (~1s): a longer rolling
// window was wanted for the runtime tare average. Memory cost is
// TARE_NUM_SAMPLES * 4 bytes (int32_t) = ~78 KiB static SRAM for
// tare_buffer below - not PSRAM like the actuator board's much bigger
// buffers, so this is a real chunk of the ESP32-S3's ~512KB internal SRAM,
// shared with WiFi/micro-ROS/task stacks. Confirm this still builds/links
// and runs stably before trusting it - not verified by compiling here.
#define TARE_NUM_SAMPLES 20000

// Rolling buffer of the last TARE_NUM_SAMPLES raw samples, kept up to date
// on every published sample (see tare_buffer_push() in
// loadcell_pollAndPublish()) and on the boot tare's own fresh reads (see
// performTare()) so it's already primed by the time loop() would otherwise
// need ~10s to fill it from scratch. Runtime re-tares (tare_command_callback)
// average whatever's already in here instead of blocking to collect fresh
// samples - see applyRollingTare(). tare_buffer_sum is a running total
// (updated incrementally in tare_buffer_push, not recomputed from scratch
// each push) so both the push and the eventual average are O(1)/O(1), not
// O(TARE_NUM_SAMPLES) per sample.
static int32_t tare_buffer[TARE_NUM_SAMPLES];
static uint32_t tare_buffer_write_idx = 0;
static uint32_t tare_buffer_count = 0;  // caps at TARE_NUM_SAMPLES once full
static int64_t  tare_buffer_sum = 0;

static inline void tare_buffer_push(int32_t raw) {
  if (tare_buffer_count == TARE_NUM_SAMPLES) {
    tare_buffer_sum -= tare_buffer[tare_buffer_write_idx];  // evict oldest
  } else {
    tare_buffer_count++;
  }
  tare_buffer[tare_buffer_write_idx] = raw;
  tare_buffer_sum += raw;
  tare_buffer_write_idx = (tare_buffer_write_idx + 1) % TARE_NUM_SAMPLES;
}

// 2MHz, not MikroE's 100kHz default: at 2000sps each conversion needs to be
// read out well within its 500us period, and a 32-bit SPI transaction
// (8-bit command + 24-bit data) takes 320us at 100kHz alone, leaving no
// margin. 2MHz gives 16us per transaction - well under datasheet's 5MHz max.
SPISettings max11270_spi_settings(2000000, MSBFIRST, SPI_MODE0);

void max11270_writeReg(uint8_t regAddr, uint32_t value, uint8_t nBytes) {
  uint8_t cmd = MAX11270_CMD_START | MAX11270_CMD_REG_ACCESS_MODE | ((regAddr & 0x1F) << 1);
  SPI.beginTransaction(max11270_spi_settings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  for (int i = nBytes - 1; i >= 0; i--) {
    SPI.transfer((value >> (8 * i)) & 0xFF);
  }
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

uint32_t max11270_readReg(uint8_t regAddr, uint8_t nBytes) {
  uint8_t cmd = MAX11270_CMD_START | MAX11270_CMD_REG_ACCESS_MODE | ((regAddr & 0x1F) << 1) | MAX11270_CMD_READ;
  uint32_t value = 0;
  SPI.beginTransaction(max11270_spi_settings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  for (int i = 0; i < nBytes; i++) {
    value = (value << 8) | SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return value;
}

// Conversion-mode command byte (MODE=0) at 2000sps continuous. With CTRL1
// SCYCLE=0 this is sent ONCE (see setupLoadCell()) and the ADC free-runs,
// producing a fresh DATA register + RDYB pulse every ~500us on its own —
// unlike the old single-cycle approach, this must NOT be re-sent per read.
void max11270_triggerConversion() {
  uint8_t cmd = MAX11270_CMD_START | MAX11270_RATE_2000SPS_CONTINUOUS;  // MODE=0, CAL=0, IMPD=0
  SPI.beginTransaction(max11270_spi_settings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void max11270_reset() {
  digitalWrite(PIN_RST, LOW);
  delay(100);
  digitalWrite(PIN_RST, HIGH);
  delay(100);
}

// Writes a register then reads it straight back and compares. This is the
// fastest way to tell whether SPI is actually communicating at all — if
// readback doesn't match what was written, the problem is wiring/electrical
// (MOSI/MISO swapped, wrong CS pin, floating pin, no shared ground, etc.),
// not the register configuration logic itself.
void max11270_writeVerify(uint8_t regAddr, uint32_t value, const char* name) {
  max11270_writeReg(regAddr, value, 1);
  delay(5);
  uint32_t readback = max11270_readReg(regAddr, 1);
  Serial.printf("  %s: wrote 0x%02X, read back 0x%02X %s\n",
                name, (unsigned)value, (unsigned)readback,
                (readback == value) ? "OK" : "*** MISMATCH - check SPI wiring ***");
}

// Mirrors MikroE's loadcell6_default_cfg(): unipolar, single-cycle
// (continuous-retriggered), PGA enabled, all cal-register corrections
// disabled since no ADC self/system cal has been performed.
void max11270_defaultConfig() {
  max11270_reset();
  delay(10);

  Serial.println("MAX11270 config (with SPI readback verification):");

  // Bipolar (U_B cleared), not MikroE's unipolar default: unipolar mode
  // floors any AINP<=AINN differential to the 0x000000 zero-scale code, which
  // would explain a permanently-stuck-at-zero reading if S+/S- end up wired
  // with reversed polarity relative to what the board's silkscreen expects.
  //
  // SCYCLE/CONTSC both cleared -> true continuous conversion mode, not
  // MikroE's single-cycle-retriggered default: at 2000sps (500us/sample) the
  // overhead of re-issuing a convert command every read would eat a huge
  // chunk of that budget and actually restart the conversion in progress
  // (Table 5: "Conversion stops and a new conversion starts"). Continuous
  // mode free-runs once triggered — see max11270_triggerConversion().
  max11270_writeVerify(MAX11270_REG_CTRL1,
                        MAX11270_CTRL1_SYNC,
                        "CTRL1");
  delay(10);

  max11270_writeVerify(MAX11270_REG_CTRL2,
                        MAX11270_CTRL2_PGAGEN | PGA_GAIN_BITS,
                        "CTRL2");
  delay(10);

  max11270_writeVerify(MAX11270_REG_CTRL3, MAX11270_CTRL3_RESERVED, "CTRL3");
  delay(10);

  // Drives GPIO1 (= E-, the bridge's excitation return) low — see CTRL4 bit
  // comments above. This is the fix for the bridge never being excited.
  max11270_writeVerify(MAX11270_REG_CTRL4,
                        MAX11270_CTRL4_DIR1 | MAX11270_CTRL4_DIO4 |
                        MAX11270_CTRL4_DIO3 | MAX11270_CTRL4_DIO2,
                        "CTRL4");
  delay(10);

  max11270_writeVerify(MAX11270_REG_CTRL5,
                        MAX11270_CTRL5_NOSYSG | MAX11270_CTRL5_NOSYSO |
                        MAX11270_CTRL5_NOSCG  | MAX11270_CTRL5_NOSCO,
                        "CTRL5");
  delay(100);
}

// Decodes a raw 24-bit DATA register value. Bipolar + FORMAT=0 -> two's
// complement (datasheet CTRL1 FORMAT bit: "set FORMAT=0 to select two's
// complement" for bipolar range). 0x000000 is a real zero reading here, not
// a floor code, so sign-extend bit 23 instead of subtracting a unipolar
// midscale offset.
static inline int32_t max11270_decodeData(uint32_t raw) {
  if (raw & 0x800000) raw |= 0xFF000000;
  return (int32_t)raw;
}

// Waits (bounded) for RDYB and reads one sample. Assumes continuous
// conversions are already running (see max11270_triggerConversion(), called
// once at startup) — does NOT trigger a new conversion itself. Only used for
// the startup tare average, where a few ms of blocking wait per sample is
// fine; the normal-operation 2kHz path uses max11270_pollAdcRaw() instead so
// it never blocks the main loop.
bool max11270_readAdcRaw(int32_t* out) {
  unsigned long start = millis();
  while (digitalRead(PIN_RDY) == HIGH) {
    if (millis() - start > 5) {
      Serial.println("  max11270_readAdcRaw: TIMEOUT waiting for RDY to go low");
      return false;  // 2000sps -> ~500us expected; 5ms is generous
    }
  }
  *out = max11270_decodeData(max11270_readReg(MAX11270_REG_DATA, 3));
  return true;
}

// Non-blocking: returns true only if a fresh conversion is already sitting in
// DATA (RDYB already low). Meant to be called every pass through loop() so
// samples get picked up continuously without ever stalling WiFi/micro-ROS
// servicing waiting for the next one.
bool max11270_pollAdcRaw(int32_t* out) {
  if (digitalRead(PIN_RDY) == HIGH) {
    return false;
  }
  *out = max11270_decodeData(max11270_readReg(MAX11270_REG_DATA, 3));
  return true;
}

// Boot-only tare: average TARE_NUM_SAMPLES fresh blocking reads as the
// zero-load baseline, same idea as MikroE's loadcell6_tare() (they average
// 100 samples). Only used from setupLoadCell(), which runs inside setup()
// before loop() has ever executed once - the rolling tare_buffer (see
// applyRollingTare() below) can't have anything in it yet at that point no
// matter what, since it's only ever filled from inside loop(). Also pushes
// each sample into tare_buffer as it goes, so the buffer starts primed
// instead of needing another ~10s of loop() time to fill from scratch.
//
// On a failed read (n_ok == 0) keeps whatever loadcell_tare_raw already
// was (0 at boot, so a no-op here) rather than snapping to a hard 0.
// Printing every one of TARE_NUM_SAMPLES samples (fine at the original 10,
// still fine at 2000) turned into a real problem at 20000: even with
// Serial.setTxTimeoutMs(0) making individual writes non-blocking, a
// sustained burst of ~20000 back-to-back printf calls with no host
// draining the native USB CDC buffer was observed to stall boot until a
// serial monitor actually connects - apparently something below the
// software TX-timeout setting, not covered by it. No per-sample printing
// at all now - Foxglove (watching loadcell_data/loadcell_data_impulse)
// already shows whether a tare worked, a serial monitor isn't needed for
// that and this board shouldn't depend on one being attached to boot
// cleanly. Just the one summary line below.
void performTare() {
  int32_t sum = 0;
  int n_ok = 0;
  for (int i = 0; i < TARE_NUM_SAMPLES; i++) {
    int32_t raw;
    if (max11270_readAdcRaw(&raw)) {
      sum += raw;
      n_ok++;
      tare_buffer_push(raw);
    }
  }
  // Integer division - matches the integer-count domain raw ADC values
  // already live in, and how the result is later subtracted (raw -
  // loadcell_tare_raw is int32 - int32). Truncates any fractional
  // remainder, so this is exact to within +/-1 raw count, not sub-count
  // precise - negligible next to the ~a few million count full-scale range.
  loadcell_tare_raw = (n_ok > 0) ? (sum / n_ok) : loadcell_tare_raw;
  if (n_ok > 0) {
    Serial.printf("Boot tare = sum / n_ok = %ld / %d = %ld raw counts\n",
                  (long)sum, n_ok, (long)loadcell_tare_raw);
  } else {
    Serial.printf("Boot tare: all %d samples failed - keeping previous value %ld raw counts\n",
                  TARE_NUM_SAMPLES, (long)loadcell_tare_raw);
  }
  reset_impulse();
}

// Runtime re-tare (tare_command_callback, below): averages whatever's
// already sitting in the rolling tare_buffer instead of blocking to collect
// fresh samples - the buffer is continuously kept current by
// tare_buffer_push() calls in loadcell_pollAndPublish(), under the same
// live operating conditions (WiFi active, ROS publishing, etc.) as the
// signal it's being subtracted from, unlike a blocking read burst which
// runs with everything else on the board paused. Effectively instant - no
// loop()-blocking cost at all, unlike performTare().
void applyRollingTare() {
  if (tare_buffer_count == 0) {
    Serial.println("Rolling tare: buffer empty (loop() hasn't run yet?) - keeping previous tare value");
    return;
  }
  int32_t new_tare = (int32_t)(tare_buffer_sum / (int64_t)tare_buffer_count);
  Serial.printf("Rolling tare = sum / count = %lld / %lu = %ld raw counts (buffer %s)\n",
                (long long)tare_buffer_sum, (unsigned long)tare_buffer_count, (long)new_tare,
                (tare_buffer_count == TARE_NUM_SAMPLES) ? "full" : "still filling");
  loadcell_tare_raw = new_tare;
  reset_impulse();
}

// TestRunnerNode's tare-approval loop publishes True here each time the
// operator wants a re-tare; False is never sent but ignored defensively.
void tare_command_callback(const void* msg_in) {
  if (msg_in == NULL) return;
  const std_msgs__msg__Bool* in = (const std_msgs__msg__Bool*)msg_in;
  if (!in->data) return;
  Serial.println("Tare command received - applying rolling tare...");
  applyRollingTare();
}

void setupLoadCell() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_RDY, INPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, HIGH);
  pinMode(PIN_SYN, OUTPUT);
  digitalWrite(PIN_SYN, HIGH);
  SPI.begin(PIN_SCK, PIN_SDO, PIN_SDI, PIN_CS);

  max11270_defaultConfig();

  // Starts continuous conversion at 2000sps. Sent exactly once — CTRL1 has
  // SCYCLE=0 so the ADC free-runs from here on; re-sending this would
  // restart the conversion in progress (see comment on CTRL1 above).
  max11270_triggerConversion();
  delay(5);  // let the SINC filter settle (datasheet: RDYB stays high for
             // 5 conversion periods after a reset/start in continuous mode)

  performTare();  // startup tare - see performTare() for why the fallback
                  // behavior on a failed read is safe at both boot and runtime
}

// Called every pass through loop(): picks up any ADC sample that's ready
// (non-blocking) and, keeping only every Nth one (LOADCELL_DECIMATION),
// publishes it immediately as its own message stamped with the real synced
// time it was read (see rmw_uros_sync_session() in ROS_Connect()) — plain
// receive-time ordering isn't precise/even enough at this rate, so Foxglove
// needs a real per-sample header.stamp to plot it correctly instead of
// clumping many samples at the same visual timestamp.
void loadcell_pollAndPublish() {
  int32_t raw;
  if (!max11270_pollAdcRaw(&raw)) {
    return;
  }
  loadcell_pollCounter++;
  if ((loadcell_pollCounter % LOADCELL_DECIMATION) != 0) {
    return;  // discard - keeping every Nth sample, evenly spread
  }
  if (!ros_connected) {
    return;
  }
  // Keeps the rolling tare buffer current with the same samples actually
  // being published - see applyRollingTare()/tare_buffer_push() for why.
  tare_buffer_push(raw);
  int64_t now_ns = esp_timer_get_time() * 1000LL + loadcell_epochOffsetNs;
  double force_mN = (double)(CALIBRATION_SLOPE_MN_PER_COUNT * (raw - loadcell_tare_raw));

  loadcell_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
  loadcell_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
  loadcell_msg.wrench.force.z = force_mN;
  rcl_ret_t pub_ret = rcl_publish(&loadcell_publisher, &loadcell_msg, NULL);
  if (pub_ret != RCL_RET_OK) {
    Serial.printf("rcl_publish FAILED: ret=%d\n", (int)pub_ret);
  }

  // Trapezoidal integral of force_mN over time -> impulse in mN*s. First
  // sample after a reset_impulse() (boot or tare) has nothing to integrate
  // against yet, same as the offline script's handling of its first sample.
  if (have_last_impulse_sample) {
    double dt_s = (now_ns - last_impulse_stamp_ns) / 1e9;
    if (dt_s > 0) {
      impulse_mNs += 0.5 * (last_impulse_force_mN + force_mN) * dt_s;
    }
    // else: out-of-order or duplicate timestamp - contributes nothing,
    // same reasoning as augment_bag_offline_impulse.py.
  }
  last_impulse_force_mN = force_mN;
  last_impulse_stamp_ns = now_ns;
  have_last_impulse_sample = true;

  impulse_msg.header.stamp = loadcell_msg.header.stamp;
  impulse_msg.wrench.force.z = impulse_mNs;
  rcl_ret_t impulse_pub_ret = rcl_publish(&impulse_publisher, &impulse_msg, NULL);
  if (impulse_pub_ret != RCL_RET_OK) {
    Serial.printf("impulse rcl_publish FAILED: ret=%d\n", (int)impulse_pub_ret);
  }
}

void ROS_Connect() {
  Serial.println("Initializing micro-ROS...");
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);

  Serial.printf("Pinging agent at %s:%u ...\n", AGENT_IP, AGENT_PORT);
  Serial.print("Waiting for micro-ROS agent");
  uint32_t pingCount = 0;
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    pingCount++;
    Serial.print(".");
    if (pingCount % 10 == 0) {
      Serial.printf(" [%u pings]\n", pingCount);
      // Re-bind in case the agent restarted since the last transport call —
      // same fix as the main robot firmware (stale socket otherwise).
      set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
    }
    delay(500);
  }
  Serial.printf(" Agent found after %u pings!\n", pingCount);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_ret_t ret = rclc_node_init_default(&node, "esp32_loadcell_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Node init FAILED - check that the micro-XRCE-DDS agent is running and accessible at "
                    + String(AGENT_IP) + ":" + String(AGENT_PORT));
  } else {
    Serial.println("Node created");
  }

  // best_effort: reliable QoS crashes the ESP32 (same as the main robot firmware)
  ret = rclc_publisher_init_best_effort(
    &loadcell_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
    "loadcell_data"
  );
  if (ret != RCL_RET_OK) {
    Serial.println("Loadcell publisher init FAILED");
  } else {
    Serial.println("Loadcell publisher created on topic: loadcell_data");
  }

  ret = rclc_publisher_init_best_effort(
    &impulse_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
    "loadcell_data_impulse"
  );
  if (ret != RCL_RET_OK) {
    Serial.println("Impulse publisher init FAILED");
  } else {
    Serial.println("Impulse publisher created on topic: loadcell_data_impulse");
  }

  // best_effort - same reason as the publisher above. This board's first
  // subscriber; well under this build's RMW_UXRCE_MAX_SUBSCRIPTIONS=5 cap
  // (already confirmed maxed out on the actuator board's executor).
  ret = rclc_subscription_init_best_effort(
    &tare_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "loadcell_tare_command"
  );
  if (ret != RCL_RET_OK) {
    Serial.println("Tare command subscriber init FAILED");
  } else {
    tare_subscription_ready = true;
    Serial.println("Tare command subscriber created on topic: loadcell_tare_command");
  }

  if (tare_subscription_ready) {
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
      &executor, &tare_command_subscriber, &tare_command_msg, &tare_command_callback, ON_NEW_DATA);
  }

  // Allocates header.frame_id (rosidl_runtime_c__String) - called once, not
  // per-publish, same pattern used safely before with Float32MultiArray.
  geometry_msgs__msg__WrenchStamped__init(&loadcell_msg);
  geometry_msgs__msg__WrenchStamped__init(&impulse_msg);

  // Syncs this board's epoch clock with the agent's (NTP-style round trip)
  // so we get real wall-clock time instead of the ESP32's own arbitrary
  // since-boot clock. rmw_uros_epoch_nanos() itself turned out not to keep
  // ticking between syncs (see loadcell_epochOffsetNs comment above), so we
  // capture its value here ONCE alongside the ESP32's own free-running
  // microsecond timer, and extrapolate from that per-sample instead.
  rmw_ret_t sync_ret = rmw_uros_sync_session(1000);
  if (sync_ret == RMW_RET_OK) {
    loadcell_epochOffsetNs = rmw_uros_epoch_nanos() - (esp_timer_get_time() * 1000LL);
    Serial.println("Time synced with agent");
  } else {
    Serial.println("Time sync with agent FAILED - timestamps will be wrong");
  }

  ros_connected = true;
  lastPingMs = millis();
  Serial.println("micro-ROS connected");
}

void setup() {
  Serial.begin(115200);
  // This board's native USB CDC (Serial) blocks on write when nothing is
  // reading it (no monitor attached) — the TX buffer fills and every
  // Serial.print()/printf() call hangs forever. That starves WiFi/micro-ROS
  // servicing in the same loop and eventually trips the watchdog, which
  // looks exactly like "Serial breaks micro-ROS." Zero timeout makes writes
  // non-blocking: they drop silently instead of hanging when no one's
  // listening.
  Serial.setTxTimeoutMs(0);
  delay(1000);
  Serial.println("ESP32 Load Cell micro-ROS");

  setupLoadCell();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - wifi_start > 10000) {
      Serial.println(" TIMEOUT!");
      break;
    }
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " failed - continuing anyway");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());
  }

  ROS_Connect();
}

void loop() {
  // Picks up any ADC sample that's ready and publishes every Nth one
  // immediately, every single pass through loop().
  loadcell_pollAndPublish();

  // Tare-command subscription: zero-timeout spin_some, called every pass
  // like everything else here - operator-triggered and rare, so it doesn't
  // need its own dedicated task the way the actuator board's higher-rate
  // teleop subscriber does. A callback firing does briefly block this loop
  // (performTare()'s 10-sample average takes ~5-50ms), which is fine for an
  // occasional command but would NOT be fine if this were on the hot path.
  if (tare_subscription_ready) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
  }

  // Re-print current CTRL1/CTRL2/CTRL5 register contents periodically, not
  // just once at boot — USB-CDC serial drops/re-enumerates during a chip
  // reset, so the actual boot-time readback verification tends to get lost
  // before a log/monitor can attach. This way the same check is always
  // present in any sufficiently long capture, no need to race the boot window.
  static unsigned long lastRegCheckMs = 0;
  if (millis() - lastRegCheckMs >= 5000) {
    lastRegCheckMs = millis();
    Serial.println("Periodic register check (current contents, not a rewrite):");
    Serial.printf("  CTRL1 = 0x%02X\n", (unsigned)max11270_readReg(MAX11270_REG_CTRL1, 1));
    Serial.printf("  CTRL2 = 0x%02X\n", (unsigned)max11270_readReg(MAX11270_REG_CTRL2, 1));
    Serial.printf("  CTRL5 = 0x%02X\n", (unsigned)max11270_readReg(MAX11270_REG_CTRL5, 1));
    Serial.printf("  STAT  = 0x%04X\n", (unsigned)max11270_readReg(MAX11270_REG_STAT, 2));
  }

  // Watchdog: reboot if the agent connection is lost, so an unattended
  // battery-powered board recovers on its own instead of silently going
  // dark. 3 attempts @ 100ms (not a single short ping) so a transient blip
  // doesn't trigger a reboot — same fix just applied to the main robot
  // firmware after a real dropout during testing.
  if (millis() - lastPingMs >= 5000) {
    lastPingMs = millis();
    rmw_ret_t pingResult = rmw_uros_ping_agent(100, 3);
    if (ros_connected && pingResult != RMW_RET_OK) {
      Serial.println("micro-ROS agent lost - rebooting to reconnect...");
      delay(200);
      ESP.restart();
    }
  }
}
