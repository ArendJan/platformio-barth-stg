#include <Arduino.h>
#include <hal/can_hal.hpp>
#include <hal/io_hal.hpp>

auto const LED_RED = 1; // PC_15

void cb(uint32_t id, uint8_t const *data, size_t size) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // fdcan1.send(id, data, size); // NOTE: this will trigger the InfiniteLoop,
  // as the callback is ran in the interrupt context
}

void setup() {
  io_init();

  fdcan1.on_receive(&cb);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

void loop() {
  delay(100);
  static uint8_t i = 0;
  uint8_t data[] = {1, 2, 3, i++};
  fdcan1.send(101, data, 4);
  // can::fdcan2.send(102, data, 4);
  digitalWrite(LED_RED, i % 2);
}