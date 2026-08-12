// This is firmware code for a Xiao seeed esp32 micro that should take a frequency and amplitude over ros and control a small dc motor at that frequency and amplitude. To bring it up you want to start have ros running on a laptop that is connected to the same network as the esps are starting up and connecting to. This PC should ne ready to subscribe to the various topics. the code crashes if nothing is subscribed.
// For bringing up the debugging a separate terminal should be started.
//There is alos a ros directory on the root called singleSiphon that can be run to start everything together.
//
// ── Published WrenchStamped field mapping (PUBLISH_RATE Hz each) ─────────
// Both messages reuse WrenchStamped as a 6-scalar container, not because
// any of these are literally forces/torques - see ROS_Connect()'s own
// per-publisher comments for the full reasoning behind each field choice.
//
// micro_ros/telemetry - motion state:
//   field      | variable         | units
//   -----------|------------------|----------------
//   force.x    | currentPosition  | mDeg (encoder)
//   force.y    | desiredPosition  | mDeg
//   force.z    | (spare)          |
//   torque.x   | currentVelocity  | mDeg/s
//   torque.y   | (spare)          |
//   torque.z   | (spare)          |
//
// micro_ros/energetics - electrical/mechanical accounting:
//   field      | variable             | units
//   -----------|----------------------|----------------------------
//   force.x    | smoothedCurrent_mA   | mA (signed) - rolling average, not instantaneous - see its declaration comment
//   force.y    | signed_torque_Nm     | N*m (signed)
//   force.z    | power_W              | W (unsigned - see its own comment)
//   torque.x   | cumulativeEnergy_J   | J, mechanical (resets on mode switch)
//   torque.y   | cumulativeCharge_mAh | mAh, electrical charge (resets on mode switch)
//   torque.z   | (spare)              |

#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <ESP32Encoder.h>


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/wrench_stamped.h>

#define DEBUG
// Code switch to enable the teleop subscription for debug purposes
#define ENABLE_TELEOP_SUBSCRIPTION
//comment out to disable current prints in the loop
// #define DEBUG_ENABLE_CURRENT
// comment out to disable the power-pipeline diagnostic print below
// #define DEBUG_ENABLE_POWER

//Physical constants
#define PI 3.14159265358979323846
#define ADC_TIKS_TO_MIllI_VOLTS 0.8056640625f // 3300mV / 4096 ADC ticks
#define CURRENT_SENSOR_MILL_VOLTS_TO_MILLI_AMPS 1.0f/1.1f // 1.1V/A, 
//This was the old code

//#define FORMAT_LITTLEFS_IF_FAILED true
#define PWM_PIN     D1
#define PHASE_PIN   D0
#define CURRENT_PIN A2
#define ENCODER_A   D6  // GPIO6
#define ENCODER_B   D5  // GPIO43
#define FAULT_PIN   D3  // DRV8874 FAULT (open-drain, active-low) - free pin, see wiring discussion
#define MAX_WRITE 255

// Encoder scaling
#define SR              100.37f              // gearbox speed reduction ratio
#define CPR             12                   // encoder counts per revolution (before gearbox)
// TICKS_TO_mDEG defined as int variable below (user-specified)

// PI-controller
#define CONTROLER_DEBUG true
#define KP              1e-2f                // proportional gain (position, maps mDeg error → PWM duty)
#define KI              5e-6f                // integral gain (position)
#define CONTROL_RATE    1000                  // Hz
#define CONTROL_PERIOD_US (1000000 / CONTROL_RATE)  // 2500 µs
#define PUBLISH_RATE    100                   // Hz
#define PUBLISH_PERIOD_MS (1000 / PUBLISH_RATE)     // ~33 ms

//////////////////


// Motor driver PWM config
#define MOTOR_PWM_FREQ   20000  // 20 kHz (above audible range)
#define MOTOR_PWM_RES    8      // 8-bit: 0-255


// ROS time synchronization state
volatile int64_t  ros_time_anchor_ns   = 0;  // ROS epoch ns at buffer start
volatile uint32_t esp_micros_at_anchor = 0;  // ESP micros() at same moment
bool              ros_time_synced      = false;

unsigned long lastSyncMs = 0;  // tracks time of last successful sync (used for logging only)

void motorSetup() {
  pinMode(PWM_PIN,   OUTPUT);
  pinMode(PHASE_PIN, OUTPUT);
  ledcAttach(PWM_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
}

bool motorPhase = false;  // tracks current motor direction (false = forward)

// Signed PWM duty command (-255..255) - NOT a current, despite Current below
// now being real calibrated milliamps. Duty-to-current isn't a fixed
// relationship (depends on motor/voltage/load), so this stays raw rather
// than attempting some approximate conversion. Same sign convention as
// Current (positive = forward, negative = reverse) so the two plot
// sensibly on top of each other: is the controller commanding something and
// the motor just not responding, or is nothing being commanded at all?
// Written here (the single place all motor commands flow through, both
// control modes), read/published in loop() at PUBLISH_RATE - same
// cross-core pattern as Current, so also volatile.
volatile float commandedDuty = 0.0f;

// DRV8874 FAULT monitoring - event-based via interrupt (CHANGE) rather than
// polling, so a brief current-chopping pulse can't slip through unnoticed
// between poll checks. The ISR itself only records state (digitalRead(),
// micros(), a pending flag) - it does NOT call SerialDebug directly, since
// that takes a regular (non-ISR-safe) FreeRTOS mutex. loop() checks the
// pending flag and does the actual logging in normal task context.
volatile bool faultPinState     = HIGH;  // HIGH = no fault (pulled up, open-drain idle)
volatile bool faultEventPending = false;
volatile unsigned long faultEventMicros = 0;

void IRAM_ATTR faultISR() {
  faultPinState = digitalRead(FAULT_PIN);
  faultEventMicros = micros();
  faultEventPending = true;
}

// duty: 0-255, forward: true = forward, false = reverse
void motorSet(uint8_t duty, bool forward) {
  motorPhase = !forward;  // motorPhase true = reverse (matches old code convention)
  commandedDuty = forward ? (float)duty : -(float)duty;
  digitalWrite(PHASE_PIN, forward ? HIGH : LOW);
  ledcWrite(PWM_PIN, duty);
}

void motorStop() {
  ledcWrite(PWM_PIN, 0);
}


const char* WIFI_SSID = "sas-network";
const char* WIFI_PASS = "mariners";
const char* AGENT_IP = "192.168.4.100";
const uint16_t AGENT_PORT = 8888;
const char* TELEOP_TOPIC         = "actuator_1/freq";
const char* MODE_TOPIC           = "actuator_1/mode";           // 0=FREQ_SWEEP, 1=POSITION
const char* SETPOINT_TOPIC       = "actuator_1/setpoint";       // target angle in millidegrees (POSITION mode)
const char* DUMP_TRIGGER_TOPIC   = "actuator_1/dump_current_buffer"; // publish any float to trigger dump
const char* CURRENT_CHUNK_TOPIC  = "actuator_1/current_buffer";      // chunked UInt16MultiArray output
unsigned long last_data_time = 0;


ESP32Encoder encoder;
float currentPosition = 0.0f;      // encoder position in millidegrees
float previousPosition = 0.0f;     // position at the start of the current velocity window (see VELOCITY_WINDOW_SAMPLES)
float currentVelocity = 0.0f;      // measured shaft velocity in mDeg/s
unsigned long previousControlMicros = 0;

// Velocity is differenced over VELOCITY_WINDOW_SAMPLES control_task() ticks
// (~20ms), not every single 1ms tick - at 1kHz, ticks/sample was under 1
// even at the peak-velocity instant of the slowest expected test (0.1Hz
// sweep), making per-tick differencing pure quantization noise rather than
// real velocity (mostly zero, occasional spikes when a tick happened to
// land in that 1ms). ~7-8 ticks/window at that same peak velocity with a
// 20-sample window - a real, averaged estimate, at the cost of ~20ms lag.
// Does NOT affect currentPosition (still sampled every tick, for the
// position controller's error term) or dt/previousControlMicros (still
// every tick, for the POSITION mode's integral term) - only this velocity
// estimate is throttled.
#define VELOCITY_WINDOW_SAMPLES 50
uint32_t velocityWindowCounter = 0;
unsigned long velocityWindowStartMicros = 0;

rcl_publisher_t telemetry_publisher;    // micro_ros/telemetry — encoder position, desired position, velocity - see ROS_Connect()'s field-mapping comment
rcl_publisher_t energetics_publisher;   // micro_ros/energetics — current, torque, power, mech energy, charge - see ROS_Connect()'s field-mapping comment
rcl_publisher_t time_synced_publisher;  // actuator_1/time_synced — lets Foxglove/ros2 topic echo see sync status immediately
#ifdef ENABLE_TELEOP_SUBSCRIPTION
rcl_subscription_t teleop_subscriber;
rcl_subscription_t mode_subscriber;
rcl_subscription_t setpoint_subscriber;
bool teleop_subscription_ready = false;
#endif
geometry_msgs__msg__WrenchStamped telemetry_msg;
geometry_msgs__msg__WrenchStamped energetics_msg;
std_msgs__msg__Bool    time_synced_msg;
#ifdef ENABLE_TELEOP_SUBSCRIPTION
std_msgs__msg__Float32 teleop_msg;
std_msgs__msg__Float32 mode_msg;
std_msgs__msg__Float32 setpoint_msg;
#endif
volatile float Current               =  0.0f;  // current sensor reading, signed milliamps (see control_task() for the raw-ADC-to-mA conversion)

// Current is sampled at CONTROL_RATE (1kHz) but only PUBLISH_RATE (30Hz) of
// those samples ever get read for torque/power - without smoothing, each
// published torque_Nm/power_W is really just whichever single noisy 1kHz
// sample happened to land nearest the publish tick, ~33ms apart from the
// last one with no relation between them. current_avg_push() (called every
// control_task() tick, right after Current is computed) keeps a rolling
// boxcar average over CURRENT_AVG_WINDOW_SAMPLES - same running-sum circular
// buffer pattern as tare_buffer_push() in esp32LoadCell_mROS.ino. Window
// size is PUBLISH_PERIOD_MS samples (not a separate magic number) because
// at 1kHz, 1 sample = 1ms, so that's exactly one publish interval's worth -
// every published value is a genuine average of the samples since the last
// report, and it stays correct automatically if PUBLISH_RATE ever changes.
// smoothedCurrent_mA feeds torque/power (see control_task()); it does NOT
// replace the raw Current used for the published "Current" field itself or
// for the cumulativeCharge_mAh integral - that integral should reflect true
// instantaneous draw, not a smoothed proxy (the difference is negligible
// over time either way, but raw is the more direct/correct choice there).
#define CURRENT_AVG_WINDOW_SAMPLES PUBLISH_PERIOD_MS
static float currentAvgBuffer[CURRENT_AVG_WINDOW_SAMPLES];
static uint32_t currentAvgWriteIdx = 0;
static uint32_t currentAvgCount = 0;  // caps at CURRENT_AVG_WINDOW_SAMPLES once full
static float currentAvgSum = 0.0f;
volatile float smoothedCurrent_mA    =  0.0f;  // rolling average of Current over CURRENT_AVG_WINDOW_SAMPLES - see comment above

static inline void current_avg_push(float sample) {
  if (currentAvgCount == CURRENT_AVG_WINDOW_SAMPLES) {
    currentAvgSum -= currentAvgBuffer[currentAvgWriteIdx];  // evict oldest
  } else {
    currentAvgCount++;
  }
  currentAvgBuffer[currentAvgWriteIdx] = sample;
  currentAvgSum += sample;
  currentAvgWriteIdx = (currentAvgWriteIdx + 1) % CURRENT_AVG_WINDOW_SAMPLES;
  smoothedCurrent_mA = currentAvgSum / (float)currentAvgCount;
}

// ── Current-limit foldback (proportional + hysteresis) ──────────────────
// Independent, software-side protection layered on top of the DRV8874's
// own ~4.4A hardware cycle-by-cycle chopping (IMODE pulled low - see the
// Pololu carrier's default current limit) - that hardware limit protects
// the driver silicon; this one protects the motor/gearbox from sustained
// high current at a level WE choose, well below the hardware cutoff.
// HIGH_MA is set at this motor's own rated stall current (1.5A, per its
// datasheet) - normal operation stays well under that, so the limiter only
// engages once we're actually at/near stall, which is exactly the
// sustained-high-current condition worth folding back on. LOW_MA is a 300mA
// (20%) hysteresis margin below that.
#define CURRENT_LIMIT_HIGH_MA 1500.0f  // start folding back duty once |smoothedCurrent_mA| exceeds this
#define CURRENT_LIMIT_LOW_MA  1200.0f  // ...and don't stop folding back until it drops below this - hysteresis, so a measurement sitting right at one boundary can't chatter the limiter on/off every tick
volatile bool currentLimitActive = false;  // true while the foldback scale below is actively reducing duty - published/logged nowhere yet, but here if useful later

// Returns a [0,1] multiplier for the commanded duty - 1.0 means no
// limiting. Uses smoothedCurrent_mA, not raw Current, so ADC noise can't
// false-trigger this (see smoothedCurrent_mA's own declaration comment for
// why that's the right signal here too). Call once per control_task() tick
// - since motorSet() runs before Current/smoothedCurrent_mA are updated
// each tick (see control_task()), this scale is necessarily based on the
// PREVIOUS tick's current - a one-tick lag that's normal and expected for
// a protective limiter like this, not a bug.
float current_limit_scale() {
  float measured = fabsf(smoothedCurrent_mA);
  if (currentLimitActive) {
    if (measured < CURRENT_LIMIT_LOW_MA) {
      currentLimitActive = false;
      return 1.0f;
    }
  } else {
    if (measured > CURRENT_LIMIT_HIGH_MA) {
      currentLimitActive = true;
    } else {
      return 1.0f;
    }
  }
  // Proportional foldback toward CURRENT_LIMIT_HIGH_MA. fminf(1.0f, ...) -
  // not just the division - so a momentary near-zero measured value (e.g.
  // still-warming-up rolling average) can't produce a >1 scale and increase
  // duty instead of limiting it.
  return fminf(1.0f, CURRENT_LIMIT_HIGH_MA / measured);
}

volatile float signed_torque_Nm      =  0.0f;  // current_to_torque_function() output, sign-corrected and converted to N*m - see that function's domain-limit caveat
volatile float power_W               =  0.0f;  // fabsf(signed_torque_Nm) * angular velocity, Watts - computed every control_task() tick (see there), just read/published in loop()
volatile float cumulativeEnergy_J    =  0.0f;  // integral of power_W over time (1kHz, using the same real dt the POSITION-mode integral term uses), Joules (mechanical) - resets on mode switch, see mode_callback()
volatile float cumulativeCharge_mAh  =  0.0f;  // integral of fabsf(Current) over time (1kHz), milliamp-hours (electrical charge, NOT energy - no voltage sensing on this board to convert to Joules/Wh) - resets on mode switch

// ── Buffer dump state ────────────────────────────────────────────────────
#define DUMP_CHUNK_SIZE         240      // max reliable payload over micro-ROS WiFi UDP (481 bytes)
#define CHUNK_DATA_SAMPLES      (DUMP_CHUNK_SIZE - 5)   // words per chunk excluding header
#define TELEM_SAMPLES_PER_CHUNK (CHUNK_DATA_SAMPLES / 3) // triplets (cur, pos, des) per telem chunk
#define DUMP_PERIOD_MS          2
// Dump trigger dispatch: publish 1.0 → current ADC buffer, 2.0 → telemetry (cur+pos+des interleaved)
// Retransmit dispatch (one shared subscriber — the micro-ROS Arduino build here is capped
// at RMW_UXRCE_MAX_SUBSCRIPTIONS=5, baked into the prebuilt libmicroros.a, so a 6th
// subscriber silently fails to init): value < RETRANSMIT_TELEM_OFFSET → current-buffer
// chunk seq; value >= RETRANSMIT_TELEM_OFFSET → telemetry chunk seq (value - offset).
#define RETRANSMIT_TELEM_OFFSET 1000000
rcl_publisher_t     current_chunk_publisher;
rcl_publisher_t     telemetry_chunk_publisher;
rcl_subscription_t  dump_trigger_subscriber;
rcl_subscription_t  retransmit_subscriber;
std_msgs__msg__Float32          dump_trigger_msg;
std_msgs__msg__Float32          retransmit_msg;
std_msgs__msg__UInt16MultiArray chunk_msg;
static uint16_t chunk_data_buf[DUMP_CHUNK_SIZE + 6];
volatile bool     dumpInProgress     = false;
volatile uint32_t dumpReadIndex      = 0;
volatile uint32_t dumpEndIndex       = 0;
unsigned long     lastDumpMs         = 0;
volatile bool     telemDumpInProgress = false;
volatile uint32_t telemDumpReadIndex  = 0;
volatile uint32_t telemDumpEndIndex   = 0;
unsigned long     lastTelemDumpMs     = 0;

#define CURRENT_BUFFER_SIZE  (3 * 1024 * 1024 / sizeof(uint16_t))  // 1,572,864 samples (~1572s at 1kHz)
#define POSITION_BUFFER_SIZE (1 * 1024 * 1024 / sizeof(int16_t))   //   524,288 samples (~524s  at 1kHz)
uint16_t* currentBuffer  = nullptr;
int16_t*  positionBuffer = nullptr;  // actual encoder ticks (int16_t, lossless)
int16_t*  desiredBuffer  = nullptr;  // desired in encoder-tick equivalents (/ TICKS_TO_mDEG, rounded)
uint32_t currentBufferIndex = 0;
bool currentBufferFull  = false;
bool posBufferFull      = false;
float desiredPosition               = 0.0f;   // desired shaft angle in millidegrees
float TICKS_TO_mDEG = 360e3f / CPR / SR;
float half_amp                      = 180000.0f; // half of 360 deg amplitude in mDeg (overridden in chunk 4)
volatile float target_frequency_hz  =  0.0f;  // volatile: written by core 0, read by core 1
volatile float new_received_freq_hz =  0.0f;  // needed to compute phase shift
volatile float phase_shift_rad      =  0.0f;  // volatile: written by core 0 callback, read by core 1 loop
volatile float sinusoidTimeOffset_s =  0.0f;  // time reference reset on each mode switch

enum ControlMode { MODE_FREQ_SWEEP = 0, MODE_POSITION = 1 };
volatile ControlMode controlMode             = MODE_FREQ_SWEEP;
float integralError                          = 0.0f;  // accumulated position error for integral term
volatile bool ros_connected                  = false; // true once micro-ROS agent is up
TaskHandle_t spinTaskHandle                  = NULL;  // handle for the micro-ROS spin task
TaskHandle_t controlTaskHandle               = NULL;  // handle for the 1kHz control + ADC task
TaskHandle_t watchdogTaskHandle               = NULL;  // handle for the ping watchdog task - see watchdog_task()


#ifdef ENABLE_TELEOP_SUBSCRIPTION
rclc_executor_t executor;
#endif

#ifdef ENABLE_TELEOP_SUBSCRIPTION
void micro_ros_spin_task(void* /*arg*/)
{
  for (;;) {
    if (teleop_subscription_ready) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    vTaskDelay(1);  // yield to FreeRTOS scheduler
  }
}
#endif
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//separate debug serial output

// Guards TimestampedSerial's writes below - loop()'s default task and
// micro_ros_spin_task (subscription callbacks like setpoint_callback) both
// call into SerialDebug from core 0 with no prior synchronization, and
// concurrent unsynchronized writes into the same Stream were confirmed
// (real captured logs) to garble output character-by-character and, at
// least once, crash the board outright (Guru Meditation / LoadProhibited
// right after ROS_Connect() - a near-null pointer read, consistent with
// corrupted internal USB CDC driver state from a torn concurrent write).
// Created at the very start of setup(), before anything could log.
// RECURSIVE: write(buf, size) below calls write(c) internally in a loop, so
// a plain (non-recursive) mutex would deadlock a task against itself the
// moment it tried to take a lock it already held.
SemaphoreHandle_t serialDebugMutex = NULL;

// Define TX and RX pins for UART (change if needed)
#define TXD1 D8
#define RXD1 D9

// Use Serial1 for UART communication. Reverted here from an attempt to wrap
// native Serial (USB/HWCDC) directly instead - that path hit a genuine,
// never-fully-resolved bug in the ESP32 core's HWCDC driver under sustained
// high-rate logging (confirmed via captured serial logs across several
// rounds: task-interleaving corruption, fixed by the mutex below; an
// indefinite-hang-until-a-monitor-connects bug in HWCDC::write() at
// setTxTimeoutMs(0), root-caused to a uint32 underflow in the core itself;
// and, even after fixing both plus adding per-byte retry logic, STILL
// random dropped/corrupted characters under a rapid burst). Not worth
// continuing to chase - this UART1 approach was already proven reliable
// before that whole detour, at the cost of needing a physical relay
// connection to actually see the output instead of just the board's own
// USB port.
HardwareSerial SerialDebugRaw(1);

// Wraps SerialDebugRaw to prepend a ROS-epoch timestamp at the start of every
// line, so debug output can be correlated against ROS time (rosbags, dumped
// buffer timestamps, etc). No call sites need to change — every
// SerialDebug.print/println/printf still works as-is.
class TimestampedSerial : public Stream {
  public:
    void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin) {
      SerialDebugRaw.begin(baud, config, rxPin, txPin);
    }
    size_t write(uint8_t c) override {
      // Recursive mutex guards against loop()'s default task and
      // micro_ros_spin_task (subscription callbacks) both writing here
      // concurrently with no synchronization - confirmed via captured logs
      // to garble output character-by-character, and at least once crash
      // the board outright, before this was added. Kept even after
      // reverting away from native Serial, since the same multi-task
      // access pattern applies regardless of which hardware is underneath.
      xSemaphoreTakeRecursive(serialDebugMutex, portMAX_DELAY);
      if (_atLineStart) {
        _printTimestamp();
        _atLineStart = false;
      }
      if (c == '\n') _atLineStart = true;
      size_t n = SerialDebugRaw.write(c);
      xSemaphoreGiveRecursive(serialDebugMutex);
      return n;
    }
    size_t write(const uint8_t* buf, size_t size) override {
      // Locked around the WHOLE buffer, not just delegated to the per-byte
      // write(c) above - otherwise another task could still interleave
      // between two bytes of this same call.
      xSemaphoreTakeRecursive(serialDebugMutex, portMAX_DELAY);
      size_t n = 0;
      for (size_t i = 0; i < size; i++) n += write(buf[i]);
      xSemaphoreGiveRecursive(serialDebugMutex);
      return n;
    }
    int available() override { return SerialDebugRaw.available(); }
    int read() override { return SerialDebugRaw.read(); }
    int peek() override { return SerialDebugRaw.peek(); }
    void flush() override { SerialDebugRaw.flush(); }

  private:
    bool _atLineStart = true;
    void _printTimestamp() {
      if (ros_time_synced) {
        int64_t ns = ros_time_anchor_ns + (int64_t)(micros() - esp_micros_at_anchor) * 1000LL;
        SerialDebugRaw.printf("[%lld.%09lld] ", ns / 1000000000LL, ns % 1000000000LL);
      } else {
        SerialDebugRaw.printf("[unsynced t=%lums] ", millis());
      }
    }
};
TimestampedSerial SerialDebug;

// Times one rmw_uros_sync_session() round trip and logs its duration plus the
// resulting epoch, so a constant ~0.4s offset seen in captured buffers can be
// diagnosed as either slow/asymmetric round-trip latency or a bias baked into
// the sync algorithm's offset calculation. If set_anchor is true, also updates
// ros_time_anchor_ns/esp_micros_at_anchor (the pair buffer index 0 is aligned to).
bool syncRosTimeDebug(int timeout_ms, bool set_anchor, const char* label)
{
  unsigned long t0 = millis();
  rcl_ret_t ret = rmw_uros_sync_session(timeout_ms);
  unsigned long round_trip_ms = millis() - t0;
  if (ret != RCL_RET_OK) {
    SerialDebug.printf("[sync:%s] FAILED after %lums\n", label, round_trip_ms);
    if (set_anchor) {
      time_synced_msg.data = false;
      rcl_publish(&time_synced_publisher, &time_synced_msg, NULL);
    }
    return false;
  }
  int64_t epoch_ns = rmw_uros_epoch_nanos();
  SerialDebug.printf("[sync:%s] round-trip %lums | epoch=%lld ns (%lld.%03lld)\n",
                      label, round_trip_ms, epoch_ns,
                      epoch_ns / 1000000000LL, (epoch_ns % 1000000000LL) / 1000000LL);
  if (set_anchor) {
    ros_time_anchor_ns   = epoch_ns;
    esp_micros_at_anchor = micros();
    // Published (not just logged) so Foxglove/`ros2 topic echo` can see sync
    // status immediately, instead of having to dig through serial debug output.
    time_synced_msg.data = true;
    rcl_publish(&time_synced_publisher, &time_synced_msg, NULL);
  }
  return true;
}


#ifdef ENABLE_TELEOP_SUBSCRIPTION
void teleop_callback(const void* msg_in)
{
  if (msg_in == NULL) return;

  const std_msgs__msg__Float32* in = (const std_msgs__msg__Float32*)msg_in;

  switch (controlMode) {
    case MODE_FREQ_SWEEP: {
      new_received_freq_hz = in->data;
      SerialDebug.printf("teleop freq received: %.3f Hz\n", new_received_freq_hz);
      // compute the phase shift for the new frequency to maintain continuity of the sine wave
      float elapsedTime = millis() / 1000.0 - sinusoidTimeOffset_s;
      phase_shift_rad = 2*PI
                        * (target_frequency_hz - new_received_freq_hz)
                        * elapsedTime
                        + phase_shift_rad;
      SerialDebug.printf("Computed phase shift: %.3f rad\n", phase_shift_rad);
      target_frequency_hz = new_received_freq_hz;
      break;
    }
    case MODE_POSITION: {
      desiredPosition = in->data;
      SerialDebug.printf("teleop position received: %.1f mDeg\n", desiredPosition);
      break;
    }
  }
}

// f(τ) = 0.11(amps) + 0.10(amps/kg/mm)*τ is the datasheet's current-as-a-
// function-of-torque relationship for the 6V HPCB 100:1 - inverted here
// (measured current -> torque) and converted from kg*mm (kilogram-force
// millimeters, the datasheet's unit) to micro-Newton-meters:
//   I - 0.11 = 0.10*tau                    [tau in kg*mm, I in amps]
//   1 kg*mm = 9.81N * 0.001m = 0.00981 N*m
//   0.10 A/(kg*mm) = 0.10 / 0.00981 = 10194 mA/(N*m)
//   I_mA - 110 = 10194 * tau[N*m]
//   tau[uN*m] = (I_mA - 110) * 1e6 / 10194 = 98.1*I_mA - 10791
// Fit to the datasheet's (positive current, positive torque) curve - pass
// fabsf(Current) in if the caller only has a signed value, and reapply
// direction afterward if needed.
//
// 110mA is this motor's no-load current - the current it draws just to
// spin itself against internal friction (brushes, bearings, iron losses)
// with ZERO output torque. The datasheet's linear model only means
// anything at or above that floor (I >= no-load current => tau >= 0); it
// was fit from steady-state measurements at various mechanical loads, not
// derived from - or valid for - current below what the motor needs just to
// idle. Below 110mA the raw line keeps going negative, but that's the fit
// extrapolating past where the physics it models even applies, not a real
// reverse-torque condition - as current approaches the no-load floor, real
// torque approaches zero, not negative. This shows up constantly, not as a
// rare edge case: every zero-crossing of a sinusoidal sweep and any
// lightly-loaded/coasting portion of motion passes through this regime, so
// clamp instead of trusting the extrapolation.
float current_to_torque_function(float current_milliamps) {
  float torque_microN_meters = 98.1f * current_milliamps - 10791.0f;
  return fmaxf(0.0f, torque_microN_meters);
}


void mode_callback(const void* msg_in)
{
  if (msg_in == NULL) return;
  const std_msgs__msg__Float32* in = (const std_msgs__msg__Float32*)msg_in;
  ControlMode newMode = (in->data >= 1.0f) ? MODE_POSITION : MODE_FREQ_SWEEP;

  // Zero the encoder on every mode command
  encoder.setCount(0);
  desiredPosition = 0.0f;
  currentPosition = 0.0f;
  previousPosition = 0.0f;
  velocityWindowCounter = 0;
  velocityWindowStartMicros = micros();
  // Judgment call, not an obvious "correct" answer: energy/charge reset per
  // mode switch (matching everything else reset here) rather than
  // accumulating since boot. Flag if you actually want since-boot totals
  // instead - it's a one-line removal each (just don't zero them here).
  cumulativeEnergy_J = 0.0f;
  cumulativeCharge_mAh = 0.0f;
  SerialDebug.println("Encoder zeroed on mode switch");

  
  // Re-sync and re-anchor on mode switch. The buffer resets below so it's safe
  // to block Core 0 here — control task on Core 1 keeps running unaffected.
  if (ros_time_synced) {
    // fresh sync; 500ms timeout, Core 1 unaffected
    syncRosTimeDebug(500, true, "mode_switch");
  }
  currentBufferIndex = 0;      // also reset the buffer here so index 0 = anchor time
  currentBufferFull  = false;
  posBufferFull      = false;

  // Reset sinusoid time and phase so it starts fresh from position 0
  sinusoidTimeOffset_s = micros() / 1e6f;
  phase_shift_rad = 0.0f;
  SerialDebug.println("Sinusoid time and phase reset");

  controlMode = newMode;
  integralError = 0.0f;  // reset integral on mode change
  SerialDebug.printf("Control mode set to: %s\n", controlMode == MODE_POSITION ? "POSITION" : "FREQ_SWEEP");
}

void setpoint_callback(const void* msg_in)
{
  if (msg_in == NULL) return;
  const std_msgs__msg__Float32* in = (const std_msgs__msg__Float32*)msg_in;
  desiredPosition = in->data;
  integralError = 0.0f;  // reset integral on new setpoint
  SerialDebug.printf("Setpoint: %.1f mDeg\n", desiredPosition);
}

void dump_trigger_callback(const void* msg_in)
{
  const std_msgs__msg__Float32* in = (const std_msgs__msg__Float32*)msg_in;
  if (in == NULL) return;
  if (in->data == 0.0f) {
    dumpInProgress = telemDumpInProgress = false;
    SerialDebug.println("All buffer dumps cancelled");
    return;
  }
  int which = (int)in->data;
  switch (which) {
    case 1:  // current ADC buffer only
      if (!dumpInProgress) {
        dumpReadIndex = 0; dumpEndIndex = currentBufferIndex; dumpInProgress = true;
        SerialDebug.printf("Current dump triggered: %u samples\n", dumpEndIndex);
      }
      break;
    case 2:  // telemetry: interleaved current + actual position + desired position
      if (!telemDumpInProgress) {
        telemDumpReadIndex  = 0;
        telemDumpEndIndex   = min((uint32_t)currentBufferIndex, (uint32_t)POSITION_BUFFER_SIZE);
        telemDumpInProgress = true;
        SerialDebug.printf("Telemetry dump triggered: %u samples\n", telemDumpEndIndex);
      }
      break;
  }
}

// Re-sends one chunk by seq number, for either dump type — see RETRANSMIT_TELEM_OFFSET
// comment above. chunk_data_buf/chunk_msg are shared with the loop() dump senders;
// only safe to call once the relevant sequential dump has finished.
void retransmit_current_chunk(uint32_t seqNum)
{
  if (!currentBuffer) return;
  uint32_t startIdx = seqNum * CHUNK_DATA_SAMPLES;
  if (startIdx >= dumpEndIndex) return;
  uint32_t count    = min((uint32_t)CHUNK_DATA_SAMPLES, dumpEndIndex - startIdx);
  if (seqNum == 0) {
    int64_t anchor = ros_time_anchor_ns;
    chunk_data_buf[0] = 0;
    chunk_data_buf[1] = (uint16_t)((anchor >> 48) & 0xFFFF);
    chunk_data_buf[2] = (uint16_t)((anchor >> 32) & 0xFFFF);
    chunk_data_buf[3] = (uint16_t)((anchor >> 16) & 0xFFFF);
    chunk_data_buf[4] = (uint16_t)( anchor        & 0xFFFF);
    chunk_data_buf[5] = (uint16_t)CONTROL_RATE;
    memcpy(&chunk_data_buf[6], &currentBuffer[startIdx], count * sizeof(uint16_t));
    chunk_msg.data.size = 6 + count;
  } else {
    chunk_data_buf[0] = (uint16_t)seqNum;
    memcpy(&chunk_data_buf[1], &currentBuffer[startIdx], count * sizeof(uint16_t));
    chunk_msg.data.size = count + 1;
  }
  rcl_publish(&current_chunk_publisher, &chunk_msg, NULL);
  SerialDebug.printf("Retransmit chunk %u\n", seqNum);
}

void retransmit_telem_chunk(uint32_t seqNum)
{
  if (!currentBuffer || !positionBuffer || !desiredBuffer) return;
  uint32_t startIdx = seqNum * TELEM_SAMPLES_PER_CHUNK;
  if (startIdx >= telemDumpEndIndex) return;
  uint32_t count = min((uint32_t)TELEM_SAMPLES_PER_CHUNK, telemDumpEndIndex - startIdx);
  uint32_t hdr;
  if (seqNum == 0) {
    int64_t anchor = ros_time_anchor_ns;
    chunk_data_buf[0] = 0;
    chunk_data_buf[1] = (uint16_t)((anchor >> 48) & 0xFFFF);
    chunk_data_buf[2] = (uint16_t)((anchor >> 32) & 0xFFFF);
    chunk_data_buf[3] = (uint16_t)((anchor >> 16) & 0xFFFF);
    chunk_data_buf[4] = (uint16_t)( anchor        & 0xFFFF);
    chunk_data_buf[5] = (uint16_t)CONTROL_RATE;
    hdr = 6;
  } else {
    chunk_data_buf[0] = (uint16_t)seqNum;
    hdr = 1;
  }
  for (uint32_t i = 0; i < count; i++) {
    uint32_t si = startIdx + i;
    chunk_data_buf[hdr + i*3 + 0] = currentBuffer[si];
    chunk_data_buf[hdr + i*3 + 1] = (uint16_t)positionBuffer[si];
    chunk_data_buf[hdr + i*3 + 2] = (uint16_t)desiredBuffer[si];
  }
  chunk_msg.data.size = hdr + count * 3;
  rcl_publish(&telemetry_chunk_publisher, &chunk_msg, NULL);
  SerialDebug.printf("Retransmit telem chunk %u\n", seqNum);
}

void retransmit_callback(const void* msg_in)
{
  const std_msgs__msg__Float32* in = (const std_msgs__msg__Float32*)msg_in;
  if (in == NULL) return;
  if (in->data >= (float)RETRANSMIT_TELEM_OFFSET) {
    retransmit_telem_chunk((uint32_t)(in->data - (float)RETRANSMIT_TELEM_OFFSET));
  } else {
    retransmit_current_chunk((uint32_t)in->data);
  }
}
#endif

void control_task(void* /*arg*/)
{
  const TickType_t period = pdMS_TO_TICKS(1);  // 1 ms = 1000 Hz
  TickType_t lastWakeTime = xTaskGetTickCount();  // anchor to now, not boot

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, period);

    unsigned long currentMicros = micros();
    float dt = (currentMicros - previousControlMicros) / 1e6f;
    previousControlMicros = currentMicros;
    float controlTime = currentMicros / 1e6f;

    currentPosition = encoder.getCount() * TICKS_TO_mDEG;

    // Widened velocity window - see VELOCITY_WINDOW_SAMPLES declaration
    // comment. currentVelocity holds its last computed value between
    // window boundaries (a sample-and-hold), rather than being recomputed
    // every tick - correct here, since no new information about this
    // estimate actually arrives faster than once per window anyway.
    velocityWindowCounter++;
    if (velocityWindowCounter >= VELOCITY_WINDOW_SAMPLES) {
      float windowDt = (currentMicros - velocityWindowStartMicros) / 1e6f;
      if (windowDt > 0) {
        currentVelocity = (currentPosition - previousPosition) / windowDt;
      }
      previousPosition = currentPosition;
      velocityWindowStartMicros = currentMicros;
      velocityWindowCounter = 0;
    }

    // Based on smoothedCurrent_mA as of the END of last tick - see
    // current_limit_scale()'s declaration comment for why that lag is
    // expected. Computed once here (not once per switch case) since
    // exactly one case executes per tick anyway - avoids calling into the
    // hysteresis state machine twice for the same tick by accident.
    float currentLimitScale = current_limit_scale();

    switch (controlMode) {
      case MODE_FREQ_SWEEP: {
        desiredPosition = half_amp - half_amp * cosf(2.0f * PI * target_frequency_hz *
                          (controlTime - sinusoidTimeOffset_s) + phase_shift_rad);
        float errorPosition = desiredPosition - currentPosition;
        bool forward = (errorPosition > 0);
        float unclampedDuty = min(KP * fabsf(errorPosition), (float)MAX_WRITE);
        uint8_t duty = (uint8_t)(unclampedDuty * currentLimitScale);
        motorSet(duty, forward);
        break;
      }
      case MODE_POSITION: {
        float errorPosition = desiredPosition - currentPosition;
        integralError += errorPosition * dt;
        integralError = constrain(integralError, -(float)MAX_WRITE / KI, (float)MAX_WRITE / KI);
        float output = KP * errorPosition + KI * integralError;
        bool forward = (output > 0);
        float unclampedDuty = min(fabsf(output), (float)MAX_WRITE);
        uint8_t duty = (uint8_t)(unclampedDuty * currentLimitScale);
        motorSet(duty, forward);
        break;
      }
    }

    int rawCurrent = analogRead(CURRENT_PIN);
    // raw ADC ticks -> mV (ADC_TIKS_TO_MIllI_VOLTS) -> mA.
    // CURRENT_SENSOR_MILL_VOLTS_TO_MILLI_AMPS = 1/1.1 (the inverse of the CS
    // pin's 1.1 mV/mA sensitivity), so multiplying here gives the same
    // result dividing by 1.1 directly would - matches the "...TO_MILLI_AMPS"
    // name (mV in, mA out) instead of fighting it.
    float current_mA = (rawCurrent * ADC_TIKS_TO_MIllI_VOLTS) *
                       CURRENT_SENSOR_MILL_VOLTS_TO_MILLI_AMPS;
    Current = motorPhase ? -current_mA : current_mA;
    current_avg_push(Current);  // update smoothedCurrent_mA - see its declaration comment

    // Power/energy computed here (every 1kHz tick), not in the publish
    // block, so the energy integral uses the real ~1ms control-loop dt
    // instead of the much coarser ~33ms PUBLISH_PERIOD_MS - moved from
    // loop() when energy tracking was added. current_to_torque_function()
    // is fit to the datasheet's positive-current curve, so torque
    // magnitude comes from fabsf(smoothedCurrent_mA) with its sign
    // reapplied - see that function's declaration comment for the
    // conversion math and its low-current domain limit (it can extrapolate
    // to a NEGATIVE torque for small, normal, positive current below the
    // no-load threshold - a fit artifact, not a real sign flip). Uses the
    // smoothed value (not raw Current) so the published torque/power curves
    // aren't just noisy single-sample snapshots - see smoothedCurrent_mA's
    // declaration comment.
    float torque_uNm = current_to_torque_function(fabsf(smoothedCurrent_mA));
    // Writes the GLOBAL signed_torque_Nm directly (no "float" here - that
    // would shadow it with a local instead of actually updating it), since
    // it's now also published on its own in the energetics message.
    signed_torque_Nm = (smoothedCurrent_mA >= 0 ? torque_uNm : -torque_uNm) / 1e6f;  // uN*m -> N*m
    float omega_rad_s = (currentVelocity / 1000.0f) * (PI / 180.0f);      // mDeg/s -> rad/s
    // power_W uses fabsf(signed_torque_Nm), not signed_torque_Nm directly -
    // otherwise that low-current fit artifact above would leak into
    // power's sign/magnitude too. signed_torque_Nm itself is kept as-is
    // (not overwritten) since it's meaningful on its own.
    power_W = fabsf(signed_torque_Nm * omega_rad_s);
    cumulativeEnergy_J += power_W * dt;
    // Current[mA] * dt[s] -> mA*s, /3600 -> mAh. fabsf(Current), not signed
    // - same reasoning as power_W: forward/reverse motion shouldn't
    // partially cancel out actual battery charge drawn, which is what a
    // signed integral would do.
    cumulativeCharge_mAh += fabsf(Current) * dt / 3600.0f;

    uint32_t idx = currentBufferIndex;
    if (!currentBufferFull && currentBuffer)
      currentBuffer[idx] = (uint16_t)rawCurrent;
    if (!posBufferFull && positionBuffer)
      positionBuffer[idx] = (int16_t)encoder.getCount();
    if (!posBufferFull && desiredBuffer)
      desiredBuffer[idx] = (int16_t)roundf(desiredPosition / TICKS_TO_mDEG);
    currentBufferIndex++;
    if (currentBufferIndex >= CURRENT_BUFFER_SIZE)  currentBufferFull = true;
    if (currentBufferIndex >= POSITION_BUFFER_SIZE) posBufferFull     = true;
  }
}

// Own task (not called from loop()) so this can block without consequence.
// rmw_uros_ping_agent() is a SYNCHRONOUS call - 3 attempts at 100ms each,
// ~300ms worst case, see the comment on the call below - and it used to run
// directly inside loop(), before the PUBLISH_RATE publish block. Since
// loop() is single-threaded, every 5s watchdog tick delayed that iteration's
// telemetry/energetics publish by however long the ping took - a real,
// visible gap in Foxglove even when the ping succeeded, not just on
// failure. Moving it here means the worst that blocking can do is delay the
// NEXT ping, never the publisher.
void watchdog_task(void* /*arg*/)
{
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 3 attempts at 100ms each (~300ms worst case) instead of a single 20ms
    // ping — a lone slow/dropped UDP packet no longer looks identical to a
    // real disconnect. Reboot+reconnect already costs ~8s on its own, so
    // this adds negligible detection latency while filtering out transient
    // blips that aren't actual connection loss.
    rmw_ret_t pingResult = rmw_uros_ping_agent(100, 3);
    SerialDebug.printf("[watchdog] ping result: %d | ros_connected: %d | heap: %u\n",
                       (int)pingResult, (int)ros_connected, ESP.getFreeHeap());
    if (ros_connected && pingResult != RMW_RET_OK) {
      ros_connected = false;
      motorStop();
      SerialDebug.println("micro-ROS agent lost - rebooting to reconnect...");
      ledcWrite(LED_BUILTIN, 50);  // solid on to indicate lost connection
      delay(200);  // flush serial before reboot

      ESP.restart();
    }
  }
}

void ROS_Connect()
{
  static uint32_t callCount = 0;
  callCount++;
  SerialDebug.printf("\n=== ROS_Connect() call #%u | heap free: %u bytes ===\n", callCount, ESP.getFreeHeap());

  // Kill the spin task first so it stops using the transport before we reset it
  if (spinTaskHandle != NULL) {
    vTaskDelete(spinTaskHandle);
    spinTaskHandle = NULL;
    SerialDebug.println("Spin task deleted");
  }

  SerialDebug.println("Initializing micro-ROS...");
  SerialDebug.printf("WiFi status before transport set: %d\n", (int)WiFi.status());
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
  SerialDebug.printf("Transport set | WiFi status after: %d\n", (int)WiFi.status());

  // Wait for agent to be reachable before blocking on init
  SerialDebug.printf("Pinging agent at %s:%u ...\n", AGENT_IP, AGENT_PORT);
  SerialDebug.print("Waiting for micro-ROS agent");
  uint32_t pingCount = 0;
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    pingCount++;
    SerialDebug.print(".");
    if (pingCount % 10 == 0) {
      SerialDebug.printf(" [%u pings, heap: %u]\n", pingCount, ESP.getFreeHeap());
      // Refresh transport every 10 pings so a restarted agent can be detected.
      // rmw_uros_ping_agent sends to the stale socket from the previous
      // set_microros_wifi_transports call; re-calling it re-binds the UDP socket.
      set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
    }
    // Drive LED breathing while blocked waiting for agent
    float t = millis() / 1000.0f;
    ledcWrite(LED_BUILTIN, (uint8_t)((sinf(2.0f * PI * t) * 0.5f + 0.5f) * 255.0f));
    delay(500);
  }
  ledcWrite(LED_BUILTIN, 0);  // turn off when agent found
  SerialDebug.printf(" Agent found after %u pings!\n", pingCount);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  SerialDebug.println("Support initialized");

  rcl_ret_t ret = rclc_node_init_default(&node, "esp32_wifi_node", "", &support);
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Node init FAILED - check that the micro-XRC-DDS agent is running and accessible at " + String(AGENT_IP) + ":" + String(AGENT_PORT));
  } else {
    SerialDebug.println("Node created");
  }

  // Consolidated telemetry - encoder position, desired position, velocity
  // used to each be their own unstamped Float32 topic; folded into one
  // WrenchStamped so all three share a single synchronized timestamp per
  // sample instead of relying on receive-time correlation. Current/torque/
  // power/energy/charge used to live here too, moved out to their own
  // "energetics" message below - kept separate since they're a distinct
  // concern (electrical/mechanical accounting vs. motion state) and it
  // frees up field slots here for future motion-related additions:
  //   wrench.force.x  = currentPosition, encoder (mDeg)
  //   wrench.force.y  = desiredPosition (mDeg)
  //   wrench.force.z  = unused (spare)
  //   wrench.torque.x = currentVelocity (mDeg/s)
  //   wrench.torque.y = unused (spare)
  //   wrench.torque.z = unused (spare)
  ret = rclc_publisher_init_best_effort(
    &telemetry_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
    "micro_ros/telemetry"
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Telemetry publisher init FAILED");
  } else {
    SerialDebug.println("Telemetry publisher created");
  }
  // Allocates header.frame_id (rosidl_runtime_c__String) - called once, not
  // per-publish, same reason the loadcell board does this for its own
  // WrenchStamped messages. This board's first use of the type.
  geometry_msgs__msg__WrenchStamped__init(&telemetry_msg);

  // Energetics - current, torque, power, mechanical energy, electrical
  // charge. Moved out of micro_ros/telemetry (see that publisher's comment
  // above) into its own message:
  //   wrench.force.x  = smoothedCurrent_mA (mA) - rolling average, not
  //                      instantaneous Current - see its declaration comment
  //   wrench.force.y  = signed_torque_Nm (N*m)
  //   wrench.force.z  = power_W (W) - fabsf(signed_torque_Nm) * angular
  //                      velocity, so direction can't flip its sign
  //   wrench.torque.x = cumulativeEnergy_J (J, mechanical) - integral of
  //                      power_W over time; resets on mode switch
  //   wrench.torque.y = cumulativeCharge_mAh (mAh, electrical charge, NOT
  //                      energy - no voltage sensing on this board);
  //                      resets on mode switch
  //   wrench.torque.z = unused (spare)
  ret = rclc_publisher_init_best_effort(
    &energetics_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
    "micro_ros/energetics"
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Energetics publisher init FAILED");
  } else {
    SerialDebug.println("Energetics publisher created");
  }
  geometry_msgs__msg__WrenchStamped__init(&energetics_msg);

  // Initialize the current buffer chunk publisher (best_effort: reliable QoS crashes ESP32)
  ret = rclc_publisher_init_best_effort(
    &current_chunk_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    CURRENT_CHUNK_TOPIC
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Current chunk publisher init FAILED");
  } else {
    SerialDebug.println("Current chunk publisher created");
  }

  ret = rclc_publisher_init_best_effort(
    &telemetry_chunk_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "actuator_1/telemetry_buffer"
  );
  if (ret != RCL_RET_OK) SerialDebug.println("Telemetry chunk publisher init FAILED");
  else                    SerialDebug.println("Telemetry chunk publisher created");

  ret = rclc_publisher_init_best_effort(
    &time_synced_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "actuator_1/time_synced"
  );
  if (ret != RCL_RET_OK) SerialDebug.println("Time synced publisher init FAILED");
  else                    SerialDebug.println("Time synced publisher created");

  // Wire chunk_msg to chunk_data_buf (shared by all dumps — they never run concurrently)
  chunk_msg.data.data     = chunk_data_buf;
  chunk_msg.data.capacity = DUMP_CHUNK_SIZE + 6;
  chunk_msg.data.size     = 0;

#ifdef ENABLE_TELEOP_SUBSCRIPTION
  ret = rclc_subscription_init_best_effort(
    &teleop_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    TELEOP_TOPIC
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Teleop subscriber init FAILED");
  } else {
    teleop_subscription_ready = true;
    SerialDebug.print("Teleop subscriber created on topic: ");
    SerialDebug.println(TELEOP_TOPIC);
  }

  ret = rclc_subscription_init_best_effort(
    &mode_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    MODE_TOPIC
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Mode subscriber init FAILED");
  } else {
    SerialDebug.print("Mode subscriber created on topic: ");
    SerialDebug.println(MODE_TOPIC);
  }

  ret = rclc_subscription_init_best_effort(
    &setpoint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    SETPOINT_TOPIC
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Setpoint subscriber init FAILED");
  } else {
    SerialDebug.print("Setpoint subscriber created on topic: ");
    SerialDebug.println(SETPOINT_TOPIC);
  }

  ret = rclc_subscription_init_best_effort(
    &dump_trigger_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    DUMP_TRIGGER_TOPIC
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Dump trigger subscriber init FAILED");
  } else {
    SerialDebug.print("Dump trigger subscriber created on topic: ");
    SerialDebug.println(DUMP_TRIGGER_TOPIC);
  }

  ret = rclc_subscription_init_best_effort(
    &retransmit_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "actuator_1/retransmit_chunk"
  );
  if (ret != RCL_RET_OK) {
    SerialDebug.println("Retransmit subscriber init FAILED");
  } else {
    SerialDebug.println("Retransmit subscriber created");
  }

  rclc_executor_init(&executor, &support.context, 5, &allocator);
  if (teleop_subscription_ready) {
    rclc_executor_add_subscription(&executor, &teleop_subscriber,       &teleop_msg,       &teleop_callback,       ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &mode_subscriber,         &mode_msg,         &mode_callback,         ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &setpoint_subscriber,     &setpoint_msg,     &setpoint_callback,     ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &dump_trigger_subscriber, &dump_trigger_msg, &dump_trigger_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &retransmit_subscriber,   &retransmit_msg,   &retransmit_callback,   ON_NEW_DATA);
    SerialDebug.println("Executor started");
  } else {
    SerialDebug.println("Executor started without teleop subscription");
  }

  xTaskCreatePinnedToCore(
    micro_ros_spin_task,  // task function
    "micro_ros_spin",     // name
    8192,                 // stack bytes
    NULL,                 // arg
    1,                    // priority
    &spinTaskHandle,      // handle
    0                     // pin to core 0
  );
  SerialDebug.println("micro_ros_spin task pinned to core 0");

#else
  SerialDebug.println("Teleop subscription disabled");
#endif
  ros_connected = true;

  // Started here (not from boot) so the first vTaskDelay(5000) inside
  // watchdog_task() counts from a live connection, not from power-on -
  // same reasoning the old lastPingMs = millis() reset here used to cover.
  if (watchdogTaskHandle == NULL) {
    xTaskCreatePinnedToCore(
      watchdog_task,        // task function
      "watchdog",           // name
      4096,                 // stack bytes
      NULL,                 // arg
      1,                    // priority - matches micro_ros_spin, not time-critical
      &watchdogTaskHandle,  // handle
      0                     // pin to core 0, alongside micro_ros_spin
    );
    SerialDebug.println("watchdog task pinned to core 0");
  }

  SerialDebug.printf("micro-ROS connected | heap free: %u bytes\n", ESP.getFreeHeap());
  // Synchronize ESP32 clock to ROS agent
  // Timeout arg is in ms; 1000ms is generous but safe over WiFi
  // Warm-up round is discarded — a fresh UDP/DDS session's first round trip
  // can be slower/less consistent than later ones, so log both to see if
  // that's contributing to the constant ~0.4s offset seen in captures.
  syncRosTimeDebug(1000, false, "warmup");
  bool sync_ok = syncRosTimeDebug(1000, true, "connect");
  if (sync_ok) {
    ros_time_synced = true;
    lastSyncMs = millis();
    SerialDebug.printf("ROS time synced | anchor: %lld ns\n", ros_time_anchor_ns);
  } else {
    SerialDebug.println("WARNING: ROS time sync FAILED - timestamps will be relative");
  }

  // Create control task after sync so that buffer index 0 aligns with ros_time_anchor_ns.
  // Creating it before sync would fill ~1s of samples before the anchor is set, shifting
  // all reconstructed timestamps by the sync duration.
  if (controlTaskHandle == NULL) {
    currentBufferIndex = 0;
    currentBufferFull  = false;
    xTaskCreatePinnedToCore(
      control_task,       // task function
      "control",          // name
      4096,               // stack bytes
      NULL,               // arg
      5,                  // priority — above loop() so it always preempts
      &controlTaskHandle, // handle
      1                   // pin to core 1
    );
    SerialDebug.println("control task pinned to core 1 at priority 5");
  }
}

void setup()
{
  // Created before anything could possibly call SerialDebug/Serial - see
  // serialDebugMutex's declaration comment for why this exists at all.
  serialDebugMutex = xSemaphoreCreateRecursiveMutex();

  // Kept for system-level output (crash/panic dumps go through this
  // regardless of what our own code does) even though application logging
  // no longer goes through native Serial - see SerialDebugRaw's comment for
  // why. No buffer-size/timeout tuning here anymore; that was only needed
  // for our own high-rate logging over this transport, which reverted back
  // to UART1.
  Serial.begin(115200);

  SerialDebug.begin(921600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  delay(3000);
  SerialDebug.println("ESP32 micro-ROS Serial debug");

  WiFi.mode(WIFI_STA);
  SerialDebug.println("WiFi mode set");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  SerialDebug.println("WiFi begin");

  SerialDebug.print("WiFi connecting");
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    SerialDebug.print(".");
    if (millis() - wifi_start > 10000) {  // Timeout after 10s
      SerialDebug.println(" TIMEOUT!");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    SerialDebug.println(" WiFi connected");
  } else {
    SerialDebug.println("WiFi connection failed");
  }

  if (WiFi.status() == WL_CONNECTED) {
    SerialDebug.print("WiFi IP: ");
    SerialDebug.println(WiFi.localIP());
  } else {
    SerialDebug.println("WiFi FAILED - continuing anyway");
  }

  pinMode(CURRENT_PIN, INPUT);

  // INPUT_PULLUP since FAULT is open-drain - idles HIGH via the internal
  // pull-up, driver pulls it LOW on over-current/over-temp/under-voltage.
  pinMode(FAULT_PIN, INPUT_PULLUP);
  faultPinState = digitalRead(FAULT_PIN);  // real initial state, not the HIGH default guess
  attachInterrupt(digitalPinToInterrupt(FAULT_PIN), faultISR, CHANGE);

  currentBuffer = (uint16_t*)ps_malloc(CURRENT_BUFFER_SIZE * sizeof(uint16_t));
  if (!currentBuffer) SerialDebug.println("ERROR: PSRAM alloc for currentBuffer FAILED");
  else SerialDebug.printf("currentBuffer allocated in PSRAM: %u bytes\n", CURRENT_BUFFER_SIZE * sizeof(uint16_t));

  positionBuffer = (int16_t*)ps_malloc(POSITION_BUFFER_SIZE * sizeof(int16_t));
  if (!positionBuffer) SerialDebug.println("ERROR: PSRAM alloc for positionBuffer FAILED");
  else SerialDebug.printf("positionBuffer allocated in PSRAM: %u bytes\n", POSITION_BUFFER_SIZE * sizeof(int16_t));

  desiredBuffer = (int16_t*)ps_malloc(POSITION_BUFFER_SIZE * sizeof(int16_t));
  if (!desiredBuffer) SerialDebug.println("ERROR: PSRAM alloc for desiredBuffer FAILED");
  else SerialDebug.printf("desiredBuffer allocated in PSRAM: %u bytes\n", POSITION_BUFFER_SIZE * sizeof(int16_t));

  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
  encoder.setCount(0);
  SerialDebug.println("Encoder attached");

  motorSetup();
  motorStop();
  SerialDebug.println("Motor ready");

  ledcAttach(LED_BUILTIN, 1000, 8);  // PWM for LED breathing

  ROS_Connect();

  SerialDebug.println("Setup complete!");
}
unsigned long loop_count = 0;
void loop()
{
  loop_count++;

  // ── DRV8874 FAULT event (checked every pass - event-based, not polled on
  // a timer, so a brief chop/fault pulse gets reported promptly instead of
  // waiting for the next periodic block) ──────────────────────────────────
  if (faultEventPending) {
    faultEventPending = false;
    bool isFault = (faultPinState == LOW);  // active-low
    // SerialDebug's own wrapper already timestamps this line - the small
    // gap between the ISR capturing faultEventMicros and this print
    // running is normally sub-millisecond and not worth reconstructing a
    // separate, more "precise" timestamp for.
    SerialDebug.printf("[FAULT] DRV8874 %s (duty was %.0f at the time)\n",
                        isFault ? "FAULT ACTIVE (over-current/over-temp/under-voltage)" : "cleared",
                        commandedDuty);
  }

  // micro-ROS connection watchdog moved to its own task - see
  // watchdog_task() and its creation in ROS_Connect() - so a slow/
  // unresponsive ping (rmw_uros_ping_agent is blocking, up to ~300ms worst
  // case) can no longer delay this loop's publish block below.



  // ── LED status: breathe at 1 Hz when not connected to micro-ROS ─────────
  if (!ros_connected) {
    float t = millis() / 1000.0f;
    uint8_t brightness = (uint8_t)((sinf(2.0f * PI * t) * 0.5f + 0.5f) * 255.0f);
    ledcWrite(LED_BUILTIN, brightness);
  } else {
    ledcWrite(LED_BUILTIN, 0);
  }

  // ── PUBLISH_RATE Hz ROS publish ──────────────────────────────────────────
  unsigned long thisTime_msec = millis();
  if (thisTime_msec - last_data_time >= PUBLISH_PERIOD_MS) {
    last_data_time = thisTime_msec;
    float time = float(thisTime_msec) / 1000.0f;

    // Consolidated telemetry + energetics publish. See ROS_Connect()'s
    // field-mapping comments for which value lives in which WrenchStamped
    // field, and why they're two separate messages. Current/currentVelocity/
    // signed_torque_Nm/power_W/cumulativeEnergy_J/cumulativeCharge_mAh are
    // all written by control_task() on core 1 (every 1kHz tick, so the
    // energy/charge integrals use the real ~1ms dt, not this ~33ms publish
    // period), read here on core 0 - not volatile-synchronized beyond the
    // `volatile` qualifier itself, but the same established pattern
    // currentPosition already used safely before this consolidation.

    #ifdef DEBUG_ENABLE_POWER
    SerialDebug.printf(
      "[power] Current=%.1fmA smoothedCurrent_mA=%.1fmA torque_Nm=%.6f currentVelocity=%.1fmDeg/s power_W=%.4f "
      "cumulativeEnergy_J=%.4f cumulativeCharge_mAh=%.4f\n",
      Current, smoothedCurrent_mA, signed_torque_Nm, currentVelocity, power_W, cumulativeEnergy_J, cumulativeCharge_mAh);
    #endif

    int64_t telemetry_now_ns = ros_time_anchor_ns + (int64_t)(micros() - esp_micros_at_anchor) * 1000LL;

    telemetry_msg.header.stamp.sec     = (int32_t)(telemetry_now_ns / 1000000000LL);
    telemetry_msg.header.stamp.nanosec = (uint32_t)(telemetry_now_ns % 1000000000LL);
    telemetry_msg.wrench.force.x  = currentPosition;  // mDeg (encoder)
    telemetry_msg.wrench.force.y  = desiredPosition;  // mDeg
    telemetry_msg.wrench.torque.x = currentVelocity;  // mDeg/s
    uint32_t publish_start = millis();
    rcl_ret_t ret = rcl_publish(&telemetry_publisher, &telemetry_msg, NULL);
    uint32_t publish_duration = millis() - publish_start;

    energetics_msg.header.stamp.sec     = telemetry_msg.header.stamp.sec;
    energetics_msg.header.stamp.nanosec = telemetry_msg.header.stamp.nanosec;
    energetics_msg.wrench.force.x  = smoothedCurrent_mA;    // mA (smoothed, not instantaneous)
    energetics_msg.wrench.force.y  = signed_torque_Nm;      // N*m
    energetics_msg.wrench.force.z  = power_W;                // W
    energetics_msg.wrench.torque.x = cumulativeEnergy_J;     // J (mechanical)
    energetics_msg.wrench.torque.y = cumulativeCharge_mAh;   // mAh (electrical charge)
    rcl_publish(&energetics_publisher, &energetics_msg, NULL);

    // Republished every PUBLISH_PERIOD_MS (not just at sync events) so a client that
    // subscribes mid-session sees current status right away — best-effort
    // QoS has no durability/latching, so a late subscriber sees nothing
    // until the next publish.
    time_synced_msg.data = ros_time_synced;
    rcl_publish(&time_synced_publisher, &time_synced_msg, NULL);
    #ifdef DEBUG_ENABLE_CURRENT
    // cur is real calibrated milliamps now (see control_task()'s ADC->mV->mA
    // conversion). cmd is the signed PWM duty the controller decided to send
    // (see motorSet()) - raw duty, NOT current/amps, so don't compare its
    // magnitude directly against cur. Watch for cmd sitting at/near 0 while
    // err (desiredPosition-currentPosition) is clearly nonzero - that's the
    // controller genuinely deciding to send ~no current despite a real
    // position error, not the motor failing to respond to a real command.
    SerialDebug.printf("t:%.2fs enc:%.1f des:%.1f err:%.1f cur:%.1fmA cmd:%.0f | pub:%ums ret:%d lp:%lu\n",
                      time, currentPosition, desiredPosition, desiredPosition - currentPosition,
                      Current, commandedDuty, publish_duration, ret, loop_count);
    #endif

    loop_count = 0;
  }

  // ── Buffer dump: send one chunk every DUMP_PERIOD_MS ─────────────────────
  if (dumpInProgress && ros_connected) {
    unsigned long now = millis();
    if (now - lastDumpMs >= DUMP_PERIOD_MS) {
      lastDumpMs = now;
      uint32_t remaining = dumpEndIndex - dumpReadIndex;
      if (remaining == 0) {
        dumpInProgress = false;
        SerialDebug.println("Buffer dump complete");
      } else {
        uint32_t seqNum = dumpReadIndex / CHUNK_DATA_SAMPLES;
        uint32_t count  = min((uint32_t)CHUNK_DATA_SAMPLES, remaining);

        if (seqNum == 0) {
          int64_t anchor = ros_time_anchor_ns;
          chunk_data_buf[0] = 0;
          chunk_data_buf[1] = (uint16_t)((anchor >> 48) & 0xFFFF);
          chunk_data_buf[2] = (uint16_t)((anchor >> 32) & 0xFFFF);
          chunk_data_buf[3] = (uint16_t)((anchor >> 16) & 0xFFFF);
          chunk_data_buf[4] = (uint16_t)( anchor        & 0xFFFF);
          chunk_data_buf[5] = (uint16_t)CONTROL_RATE;
          memcpy(&chunk_data_buf[6], &currentBuffer[dumpReadIndex], count * sizeof(uint16_t));
          chunk_msg.data.size = 6 + count;
        } else {
          chunk_data_buf[0] = (uint16_t)seqNum;
          memcpy(&chunk_data_buf[1], &currentBuffer[dumpReadIndex], count * sizeof(uint16_t));
          chunk_msg.data.size = count + 1;
        }
        dumpReadIndex += count;
        rcl_ret_t pub_ret = rcl_publish(&current_chunk_publisher, &chunk_msg, NULL);
        SerialDebug.printf("Dump: chunk %u sent (%u / %u) pub_ret=%d\n", seqNum, dumpReadIndex, dumpEndIndex, (int)pub_ret);
      }
    }
  }

  // ── Telemetry dump: interleaved [current, actual_pos, desired_pos] triplets ──
  // Chunk 0 header: [0, ns48, ns32, ns16, ns0, rate] then TELEM_SAMPLES_PER_CHUNK triplets
  // Non-zero:       [seqNum] then TELEM_SAMPLES_PER_CHUNK triplets
  // Python decode:  data[0::3]=current(u16), data[1::3].view(i16)*TICKS_TO_mDEG=actual, data[2::3].view(i16)*TICKS_TO_mDEG=desired
  if (telemDumpInProgress && ros_connected && positionBuffer && desiredBuffer) {
    unsigned long now = millis();
    if (now - lastTelemDumpMs >= DUMP_PERIOD_MS) {
      lastTelemDumpMs = now;
      uint32_t remaining = telemDumpEndIndex - telemDumpReadIndex;
      if (remaining == 0) {
        telemDumpInProgress = false;
        SerialDebug.println("Telemetry dump complete");
      } else {
        uint32_t seqNum = telemDumpReadIndex / TELEM_SAMPLES_PER_CHUNK;
        uint32_t count  = min((uint32_t)TELEM_SAMPLES_PER_CHUNK, remaining);
        uint32_t hdr;
        if (seqNum == 0) {
          int64_t anchor = ros_time_anchor_ns;
          chunk_data_buf[0] = 0;
          chunk_data_buf[1] = (uint16_t)((anchor >> 48) & 0xFFFF);
          chunk_data_buf[2] = (uint16_t)((anchor >> 32) & 0xFFFF);
          chunk_data_buf[3] = (uint16_t)((anchor >> 16) & 0xFFFF);
          chunk_data_buf[4] = (uint16_t)( anchor        & 0xFFFF);
          chunk_data_buf[5] = (uint16_t)CONTROL_RATE;
          hdr = 6;
        } else {
          chunk_data_buf[0] = (uint16_t)seqNum;
          hdr = 1;
        }
        for (uint32_t i = 0; i < count; i++) {
          uint32_t si = telemDumpReadIndex + i;
          chunk_data_buf[hdr + i*3 + 0] = currentBuffer[si];
          chunk_data_buf[hdr + i*3 + 1] = (uint16_t)positionBuffer[si];
          chunk_data_buf[hdr + i*3 + 2] = (uint16_t)desiredBuffer[si];
        }
        chunk_msg.data.size = hdr + count * 3;
        telemDumpReadIndex += count;
        rcl_publish(&telemetry_chunk_publisher, &chunk_msg, NULL);
      }
    }
  }
}