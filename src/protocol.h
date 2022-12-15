#pragma once

constexpr size_t NAME_SIZE = 32;

enum State : uint8_t { kWaitingForDry = 0xab, kWateringOn = 0xef };

union PlantPayload {
  struct __attribute__((packed, scalar_storage_order("little-endian"))) Data {
    char name[NAME_SIZE];
    uint8_t pump_on;
    uint8_t state;
    uint16_t dryness;
  } data;
  uint8_t buf[sizeof(Data)];
  static constexpr size_t Size = sizeof(Data);
};
static_assert(PlantPayload::Size == NAME_SIZE + 8 / 8 + 8 / 8 + 16 / 8);

union SetPumpPayload {
  struct __attribute__((packed, scalar_storage_order("little-endian"))) Data {
    char name[NAME_SIZE];
    uint8_t pump_on;
  } data;
  uint8_t buf[sizeof(Data)];
  static constexpr size_t Size = sizeof(Data);
};
static_assert(SetPumpPayload::Size == NAME_SIZE + 8 / 8);
