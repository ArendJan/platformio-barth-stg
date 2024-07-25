#include <Arduino.h>
// #include <can.hpp>
// #include <pins.hpp>
// #include <state_machine.hpp>
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
}
void WriteLED_Green(bool State)
{
    digitalWrite(LED_BUILTIN, State);
}

void loop()
{
    digitalWrite(LED_BUILTIN, 1);

    delay(100);
    digitalWrite(LED_BUILTIN, 0);
    delay(600);
}
