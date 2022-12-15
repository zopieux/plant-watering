#include <AceRoutine.h>
#include <Arduino.h>
//#include <RCSwitch.h>
//#include <RH_ASK.h>
#include <RunningMedian.h>

#include <unordered_map>
//#include <Wire.h>

#include "protocol.h"

//constexpr uint32_t kDrynessAnalogPin = PA6;
//constexpr uint32_t kRelayWritePin = PB5;
// constexpr uint32_t kRtlRxPin = PA0;
// constexpr uint32_t kRtlTxPin = PB4;
constexpr uint32_t kLedPin = LED_BUILTIN;
// constexpr uint8_t kSdaPin = PIN_WIRE_SDA;
// constexpr uint8_t kSclPin = PIN_WIRE_SCL;

// Constants from experimenting.
constexpr uint32_t kFullyDry = 500;
constexpr uint32_t kFullyWatered = 350;
// We want 3 minutes of watering at minimum.
constexpr uint32_t kMinimumWateringMillis = 1000 * 60 * 3;
// Milliseconds.
constexpr uint32_t kPumpCycleOn = 3500, kPumpCyclePause = 250;
// RTL channel 1, button 4:
constexpr unsigned long kSwitchOff = 1397844ul;
constexpr unsigned long kSwitchOn = 1397845ul;

constexpr uint32_t kOn = LOW, kOff = HIGH;

static const std::array<uint32_t, 3> kBlinkDelays{1000 * 2, 80, 250};

// RCSwitch rcSwitch;
// RH_ASK driver(2000, 0 /* rx, unused */, kRtlTxPin, 0);
// RunningMedian samples = RunningMedian(32);

// State state = State::kWaitingForDry;
// uint32_t elapsed_watering;
// uint32_t elapsed_watering_cycle;
// uint32_t average_dryness = 0;

bool EnoughTime(uint32_t now, uint32_t elapsed, uint32_t duration) {
  // Wrap-around. Just exit.
  if (now < elapsed) return true;
  return (now - elapsed) > duration;
}

struct PlantState {
  const uint32_t relay_pin, sensor_pin;
  State state = State::kWaitingForDry;
  bool pump_on = false;
  mutable RunningMedian samples = RunningMedian(64);
  //  uint32_t average_dryness = 0;
  uint32_t elapsed_watering;
  //  uint32_t elapsed_watering_cycle;

 private:
  uint16_t average_dryness() const {
    return static_cast<uint16_t>(samples.getAverage());
  }

 public:
  bool can_publish() {
    return samples.isFull();
  }

  void WritePayload(PlantPayload::Data& payload) const {
    payload.state = state;
    payload.pump_on = pump_on;
    payload.dryness = average_dryness();
  }

  void Setup() const {
    pinMode(relay_pin, OUTPUT);
    pinMode(sensor_pin, INPUT);
  }

  void Toggle(uint32_t status) {
    digitalWrite(relay_pin, status);
    pump_on = status == kOn;
    Serial.print("Pump relay (pin ");
    Serial.print(relay_pin);
    Serial.print("): ");
    Serial.println(status == kOn ? "on" : "off");
  }

  void AccumulateHumidity() {
    samples.add(static_cast<float>(analogRead(sensor_pin)));
#if 0
    if (samples.isFull()) {
      Serial.print("dryness (pin ");
      Serial.print(sensor_pin);
      Serial.print("): ");
      Serial.println(average_dryness());
    }
#endif
  }

  void StateMachine(uint32_t now) {
    const auto& average = average_dryness();
    switch (state) {
      case State::kWaitingForDry:
        if (average >= kFullyDry) {
          state = State::kWateringOn;
          Toggle(kOn);
          elapsed_watering = now;
          //          elapsed_watering_cycle = now;
        }
        break;
      case State::kWateringOn:
        if (average <= kFullyWatered &&
            EnoughTime(now, elapsed_watering, kMinimumWateringMillis)) {
          state = State::kWaitingForDry;
          Toggle(kOff);
        }
        break;
#if 0
      case State::kWateringPause:
        if (average <= kFullyWatered &&
            EnoughTime(now, elapsed_watering, kMinimumWateringMillis)) {
          state = State::kWaitingForDry;
          Toggle(kOff);  // Just in case. Handled in previous state.
        } else if (EnoughTime(now, elapsed_watering_cycle, kPumpCyclePause)) {
          state = State::kWateringOn;
          Toggle(kOn);
          elapsed_watering_cycle = now;
        }
        break;
#endif
    }
  }
};

PlantState bonsai0{.relay_pin = PB3, .sensor_pin = PA6},
    bonsai1{.relay_pin = PB4, .sensor_pin = PA7};

static const std::unordered_map<std::string, PlantState*> plants = {
    {"old", &bonsai0}, {"new", &bonsai1}};

#if 0
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
#endif

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600, SERIAL_8N1);

  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, kOff);

  for (auto&& [name, plant] : plants) {
    plant->Setup();
    plant->Toggle(kOff);
  }

  //  rcSwitch.enableReceive(digitalPinToInterrupt(kRtlRxPin));
  //  driver.init();
  //  driver.setHeaderId(0x42);
  //  driver.setHeaderFrom(0x99);
  //  driver.setModeTx();
}

COROUTINE(co_blinker) {
  COROUTINE_LOOP() {
    digitalWrite(kLedPin, kOn);
    COROUTINE_DELAY(10);
    digitalWrite(kLedPin, kOff);
    int delay = 1000;
    for (const auto& [name, plant] : plants) {
      if (plant->pump_on) delay /= 4;
    }
    COROUTINE_DELAY(delay);
  }
}

#if 0
COROUTINE(co_rc_receiver) {
  COROUTINE_LOOP() {
    COROUTINE_AWAIT(rcSwitch.available());
    auto value = rcSwitch.getReceivedValue();
    Serial.print("RC received: ");
    Serial.println(value);
    rcSwitch.resetAvailable();
    switch (value) {
      case kSwitchOn:
        TogglePump(kOn);
        break;
      case kSwitchOff:
        TogglePump(kOff);
        break;
    }
  }
}

union StatePayload {
  struct __attribute__((packed, scalar_storage_order("little-endian"))) Data {
    uint8_t on0, on1;
    uint16_t hum0, hum1;
  } data;
  uint8_t buf[sizeof(Data)];
};
constexpr size_t StatePayloadSize = sizeof(StatePayload::Data);
static_assert(StatePayloadSize == 8 / 8 * 2 + 16 / 8 * 2);
StatePayload state_payload;
#endif

#if 0
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
#endif

COROUTINE(co_read_wifi) {
  SetPumpPayload pay{};
  COROUTINE_LOOP() {
    if (Serial3.available() >= SetPumpPayload::Size) {
      Serial3.readBytes(pay.buf, SetPumpPayload::Size);
      std::string name{pay.data.name};
      Serial.println(String("Received SetPumpPayload for ") + name.c_str() +
                     ": " + pay.data.pump_on);
      if (auto it = plants.find(name); it != plants.cend()) {
        it->second->Toggle(pay.data.pump_on ? kOn : kOff);
      }
    }
    COROUTINE_YIELD();
  }
}

COROUTINE(co_publish_wifi) {
  PlantPayload pay{};
  COROUTINE_LOOP() {
    COROUTINE_DELAY(2500);
    if (Serial3.availableForWrite()) {
      for (const auto& [name, plant] : plants) {
        if (!plant->can_publish()) continue;
        plant->WritePayload(pay.data);
        strncpy(pay.data.name, name.c_str(), NAME_SIZE);
        Serial3.write(pay.buf, PlantPayload::Size);
        Serial3.flush();
        Serial.println("published");
      }
    }
  }
}

COROUTINE(co_dryness_reader) {
  COROUTINE_LOOP() {
    COROUTINE_DELAY(250);
    for (auto&& [name, plant] : plants) plant->AccumulateHumidity();
  }
}

COROUTINE(co_main) {
  COROUTINE_LOOP() {
    COROUTINE_DELAY(250);
    auto now = millis();
    for (auto&& [name, plant] : plants) plant->StateMachine(now);
  }
}

void loop() {
  co_blinker.runCoroutine();
  co_dryness_reader.runCoroutine();
  co_read_wifi.runCoroutine();
  co_publish_wifi.runCoroutine();
  co_main.runCoroutine();
}
