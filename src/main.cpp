#include <Arduino.h>
#include <RCSwitch.h>
#include <RunningMedian.h>
#include <RH_ASK.h>

constexpr uint32_t kDrynessAnalogPin = PA6;
constexpr uint32_t kRelayWritePin = PB5;
constexpr uint32_t kRtlRxPin = PA0;
constexpr uint32_t kRtlTxPin = PB4;

constexpr uint32_t kLedPin = LED_BUILTIN;

// Constants from experimenting.
constexpr uint32_t kFullyDry = 500;
constexpr uint32_t kFullyWet = 300;
// We want 3 minutes of watering at minimum.
constexpr uint32_t kMinimumWateringMillis = 1000 * 60 * 3;

constexpr uint32_t kOn = LOW, kOff = HIGH;

// RTL channel 1, button 4:
constexpr unsigned long kSwitchOff = 1397844ul;
constexpr unsigned long kSwitchOn = 1397845ul;

enum class State {
    kWaitingForWet, kWaitingForDry
};

RCSwitch rcSwitch;
RH_ASK driver(2000, 0 /* rx, unused */, kRtlTxPin, 0);
RunningMedian samples = RunningMedian(64);

State state = State::kWaitingForDry;
unsigned long init_measures = 0;
unsigned long elapsed_measure, elapsed_send;
uint32_t elapsed_watering;

union Payload {
#pragma pack(push, 1)
    struct { ;
        uint16_t dryness;
        uint8_t relay;
    } data;
#pragma pack(pop)
    uint8_t u8[3];
};

Payload payload;
static_assert(sizeof(Payload::data) == 3, "wrong size");

void setup() {
    Serial.begin();

    pinMode(kRelayWritePin, OUTPUT);
    pinMode(kLedPin, OUTPUT);
    pinMode(kDrynessAnalogPin, INPUT);

    rcSwitch.enableReceive(digitalPinToInterrupt(kRtlRxPin));
    Serial.println(driver.init());
    driver.setHeaderId(0x42);
    driver.setHeaderFrom(0x99);
    driver.setModeTx();

    delay(1000);
    digitalWrite(kRelayWritePin, kOn);
    delay(700);
    digitalWrite(kRelayWritePin, kOff);

    elapsed_measure = millis();
    elapsed_send = elapsed_measure;
}

void act(uint8_t repeat = 1) {
    for (int i = 0; i < repeat; ++i) {
        digitalWrite(kLedPin, kOn);
        delay(10);
        digitalWrite(kLedPin, kOff);
        delay(10);
    }
}

unsigned long receive() {
    if (rcSwitch.available()) {
        auto value = rcSwitch.getReceivedValue();
        Serial.print("Received ");
        Serial.println(value);
        rcSwitch.resetAvailable();
        return value;
    }
    return 0;
}

void set_relay(uint32_t status) {
    digitalWrite(kRelayWritePin, status);
    payload.data.relay = static_cast<uint8_t>(status == kOn ? 1 : 0);
}

void send() {
    driver.send(payload.u8, 3);
    driver.waitPacketSent();
}

bool enoughWatering() {
    const uint32_t now = millis();
    // Wrap-around. Just exit.
    if (now < elapsed_watering) return true;
    return (now - elapsed_watering) > kMinimumWateringMillis;
}

void loop() {
    if (millis() - elapsed_measure > 250) {
        const uint32_t dryness = analogRead(kDrynessAnalogPin);
        samples.add(static_cast<float>(dryness));
        elapsed_measure = millis();
        act(1);

        const uint32_t average = static_cast<uint32_t>(samples.getAverage());
        Serial.println("average: ");
        Serial.println(average);
        payload.data.dryness = average;

        if (samples.isFull() && init_measures == 10) {
            init_measures++;
            // Firs time.
            Serial.print("init assessment: ");
            if (average <= kFullyWet) {
                state = State::kWaitingForDry;
                set_relay(kOff);
                Serial.println("wet! stopping pump");
            } else if (average >= kFullyDry) {
                state = State::kWaitingForWet;
                Serial.println("dry! starting pump");
                elapsed_watering = millis();
                set_relay(kOn);
            } else {
                state = State::kWaitingForDry;
                Serial.println("???");
            }
        } else if (samples.isFull() && init_measures < 10) {
            init_measures++;
        } else if (samples.isFull()) {
            if (state == State::kWaitingForDry && average >= kFullyDry) {
                state = State::kWaitingForWet;
                elapsed_watering = millis();
                set_relay(kOn);
                send();
                Serial.println("starting pump, waiting for fully wet");
            } else if (state == State::kWaitingForWet && average <= kFullyWet && enoughWatering()) {
                state = State::kWaitingForDry;
                set_relay(kOff);
                send();
                Serial.println("stopping pump, waiting for fully dry");
            }
        }
    }

    if (millis() - elapsed_send > 1000 * 5) {
        send();
        Serial.print("sent dryness: ");
        Serial.println(payload.data.dryness);
        elapsed_send = millis();
        act(3);
    }

    switch (receive()) {
        case kSwitchOn:
            Serial.println("switch ON");
            set_relay(kOn);
            send();
            break;
        case kSwitchOff:
            Serial.println("switch OFF");
            set_relay(kOff);
            send();
            break;
    }
}
