#pragma once

#include <hal/can_hal.h>
#include <Arduino.h>
void can_callback(uint32_t id, uint8_t const* data, size_t size) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
};

void init_can()
{
    can::init(can::Baudrate::Rate500k);
    can::on_receive(can_callback);
}