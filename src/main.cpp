#include <Arduino.h>
#include <can.hpp>
#include <pins.hpp>
#include <state_machine.hpp>
void setup()
{
    init_pins();
    init_can();
    init_state_machine();
    pinMode(LED_BUILTIN, OUTPUT);
}
void loop()
{
    run_state_machine();
    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000 / i);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000 / i);
    }
}
