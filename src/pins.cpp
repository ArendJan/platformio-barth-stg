#include "pins.hpp"

void init_pins()
{
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(CHARGER_SAFETY, INPUT);
    pinMode(USER_ABORT_CHARGING, INPUT);
    pinMode(IGN_KL15, INPUT);
    pinMode(CHARGE_INTERLOCK, INPUT);
    pinMode(CONTACTOR_ENABLE, INPUT);
    pinMode(BMS_CHARGE_ENABLE, INPUT);
    pinMode(DISCHARGE_ENABLE, INPUT);
    pinMode(STARTING, INPUT);
    pinMode(BRAKE_LIGHTS, INPUT);
    pinMode(UNUSED, INPUT);
    pinMode(BMS_READY_POWER, OUTPUT);
    pinMode(MAIN_MIN_CONTACTOR_REQUEST, OUTPUT);
    pinMode(PRECHARGE_CONTACTOR_REQUEST, OUTPUT);
    pinMode(TESLA_CHARGE_ENABLE, OUTPUT);
    pinMode(DISCHARGE_CONTACTOR_REQUEST, OUTPUT);
    pinMode(STARTED, OUTPUT);
    pinMode(BMS_ON, OUTPUT);
    pinMode(CHARGE_POWER_BMS, OUTPUT);
    pinMode(MAIN_PLUS_CONTACTOR_REQUEST, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(BMS_READY_POWER, LOW);
    digitalWrite(MAIN_MIN_CONTACTOR_REQUEST, LOW);
    digitalWrite(PRECHARGE_CONTACTOR_REQUEST, LOW);
    digitalWrite(TESLA_CHARGE_ENABLE, LOW);
    digitalWrite(DISCHARGE_CONTACTOR_REQUEST, LOW);
    digitalWrite(STARTED, LOW);
    digitalWrite(BMS_ON, LOW);
    digitalWrite(CHARGE_POWER_BMS, LOW);
    digitalWrite(MAIN_PLUS_CONTACTOR_REQUEST, HIGH);  // TODO: check that this not quickly switches it on boot
}

bool ignition_on()
{
    return digitalRead(IGN_KL15) == HIGH;
}

bool starting_on()
{
    return digitalRead(STARTING) == HIGH;
}

void set_bms_ready_power(bool on)
{
    digitalWrite(BMS_READY_POWER, on ? HIGH : LOW);
}

bool get_bms_ready_power()
{
    return digitalRead(BMS_READY_POWER) == HIGH;  // reading back outputs should work
}

void set_bms_on(bool on)
{
    digitalWrite(BMS_ON, on ? HIGH : LOW);
}

bool get_bms_on()
{
    return digitalRead(BMS_ON) == HIGH;  // reading back outputs should work
}

void set_main_min_contactor_request(bool on)
{
    digitalWrite(MAIN_MIN_CONTACTOR_REQUEST, on ? HIGH : LOW);
}

bool get_main_min_contactor_request()
{
    return digitalRead(MAIN_MIN_CONTACTOR_REQUEST) == HIGH;  // reading back outputs should work
}

void set_precharge_contactor_request(bool on)
{
    digitalWrite(PRECHARGE_CONTACTOR_REQUEST, on ? HIGH : LOW);
}

bool get_precharge_contactor_request()
{
    return digitalRead(PRECHARGE_CONTACTOR_REQUEST) == HIGH;  // reading back outputs should work
}