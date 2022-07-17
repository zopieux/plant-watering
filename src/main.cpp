#include <AceRoutine.h>
#include <Arduino.h>
#include <RCSwitch.h>
#include <RH_ASK.h>
#include <RunningMedian.h>

constexpr uint32_t kDrynessAnalogPin = PA6;
constexpr uint32_t kRelayWritePin = PB5;
constexpr uint32_t kRtlRxPin = PA0;
constexpr uint32_t kRtlTxPin = PB4;
constexpr uint32_t kLedPin = LED_BUILTIN;

// Constants from experimenting.
constexpr uint32_t kFullyDry = 500;
constexpr uint32_t kFullyWatered = 300;
// We want 3 minutes of watering at minimum.
constexpr uint32_t kMinimumWateringMillis = 1000 * 60 * 3;
// Milliseconds.
constexpr uint32_t kPumpCycleOn = 3500, kPumpCyclePause = 250;
// RTL channel 1, button 4:
constexpr unsigned long kSwitchOff = 1397844ul;
constexpr unsigned long kSwitchOn = 1397845ul;

constexpr uint32_t kOn = LOW, kOff = HIGH;

RCSwitch rcSwitch;
RH_ASK driver(2000, 0 /* rx, unused */, kRtlTxPin, 0);
RunningMedian samples = RunningMedian(64);

enum class State { kWaitingForDry, kWateringOn, kWateringPause };
State state = State::kWaitingForDry;
uint32_t elapsed_watering;
uint32_t elapsed_watering_cycle;
uint32_t average_dryness = 0;

static const std::array<uint32_t, 3> kBlinkDelays{1000 * 2, 80, 250};

union TxPayload {
#pragma pack(push, 1)
  struct {
    uint16_t dryness;
    uint8_t relay;
  } data;
#pragma pack(pop)
  uint8_t u8[3];
};
static_assert(sizeof(TxPayload::data) == 3, "wrong size");
TxPayload payload;

void togglePump(uint32_t status) {
  digitalWrite(kRelayWritePin, status);
  payload.data.relay = static_cast<uint8_t>(status == kOn ? 1 : 0);
  Serial.print("Pump relay: ");
  Serial.println(status == kOn ? "on" : "off");
}

bool enoughTime(uint32_t now, uint32_t elapsed, uint32_t duration) {
  // Wrap-around. Just exit.
  if (now < elapsed) return true;
  return (now - elapsed) > duration;
}

void setup() {
  pinMode(kRelayWritePin, OUTPUT);
  pinMode(kLedPin, OUTPUT);
  pinMode(kDrynessAnalogPin, INPUT);

  digitalWrite(kLedPin, kOff);
  togglePump(kOff);

  Serial.begin();
  rcSwitch.enableReceive(digitalPinToInterrupt(kRtlRxPin));
  driver.init();
  driver.setHeaderId(0x42);
  driver.setHeaderFrom(0x99);
  driver.setModeTx();
}

COROUTINE(co_blinker) {
  COROUTINE_LOOP() {
    digitalWrite(kLedPin, kOn);
    COROUTINE_DELAY(10);
    digitalWrite(kLedPin, kOff);
    COROUTINE_DELAY(kBlinkDelays[static_cast<size_t>(state)]);
  }
}

COROUTINE(co_rc_receiver) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(rcSwitch.available());
    auto value = rcSwitch.getReceivedValue();
    Serial.print("RC received: ");
    Serial.println(value);
    rcSwitch.resetAvailable();
    switch (value) {
      case kSwitchOn:
        togglePump(kOn);
        break;
      case kSwitchOff:
        togglePump(kOff);
        break;
    }
  }
}

COROUTINE(co_dryness_reader) {
  payload.data.dryness = 0;
  COROUTINE_LOOP() {
    COROUTINE_DELAY(250);
    samples.add(static_cast<float>(analogRead(kDrynessAnalogPin)));
    if (samples.isFull()) {
      average_dryness = static_cast<uint32_t>(samples.getAverage());
    }
  }
}

COROUTINE(co_rc_sender) {
  COROUTINE_LOOP() {
    COROUTINE_DELAY_SECONDS(5);
    if (average_dryness == 0) continue;
    Serial.print("Dryness: ");
    Serial.println(average_dryness);
    payload.data.dryness = average_dryness;
    payload.data.relay =
        static_cast<uint8_t>(digitalRead(kRelayWritePin) == kOn);
    driver.send(payload.u8, 3);
    driver.waitPacketSent();
  }
}

COROUTINE(co_main) {
  COROUTINE_LOOP() {
    COROUTINE_DELAY(250);
    if (average_dryness == 0) continue;
    auto now = millis();
    switch (state) {
      case State::kWaitingForDry:
        if (average_dryness >= kFullyDry) {
          state = State::kWateringOn;
          togglePump(kOn);
          elapsed_watering = now;
          elapsed_watering_cycle = now;
        }
        break;
      case State::kWateringOn:
        if (enoughTime(now, elapsed_watering_cycle, kPumpCycleOn)) {
          state = State::kWateringPause;
          togglePump(kOff);
          elapsed_watering_cycle = now;
        }
        break;
      case State::kWateringPause:
        if (average_dryness <= kFullyWatered &&
            enoughTime(now, elapsed_watering, kMinimumWateringMillis)) {
          state = State::kWaitingForDry;
          togglePump(kOff);  // Just in case. Handled in previous state.
        } else if (enoughTime(now, elapsed_watering_cycle, kPumpCyclePause)) {
          state = State::kWateringOn;
          togglePump(kOn);
          elapsed_watering_cycle = now;
        }
        break;
    }
  }
}

void loop() {
  co_blinker.runCoroutine();
  co_dryness_reader.runCoroutine();
  co_rc_receiver.runCoroutine();
  co_rc_sender.runCoroutine();
  co_main.runCoroutine();
}
