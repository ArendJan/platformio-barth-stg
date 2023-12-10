#pragma once
#include <Arduino.h>
#include <pins_stg_850.h>

auto constexpr CHARGER_SAFETY = IN1;       //
auto constexpr USER_ABORT_CHARGING = IN2;  //
auto constexpr IGN_KL15 = IN3;             // IGNITION ON
auto constexpr CHARGE_INTERLOCK = IN4;     // WE WANT TO CHARGE, PLUG IN SOCKET
auto constexpr CONTACTOR_ENABLE = IN5;     // MULTIPURPOSE ENABLE ACTIVE LOW
auto constexpr BMS_CHARGE_ENABLE = IN6;        // BMS: READY TO RECEIVE CHARGE
auto constexpr DISCHARGE_ENABLE = IN7;     // BMS: OK TO DRIVE
auto constexpr STARTING = IN8;             // IGNITION 2ND CLICK, STARTER
auto constexpr BRAKE_LIGHTS = IN9;         // BRAKE PEDAL PRESSED
auto constexpr UNUSED = IN10;
auto constexpr BMS_READY_POWER = OUT1;
auto constexpr MAIN_MIN_CONTACTOR_REQUEST = OUT2;
auto constexpr PRECHARGE_CONTACTOR_REQUEST = OUT3;
auto constexpr TESLA_CHARGE_ENABLE = OUT4;
auto constexpr DISCHARGE_CONTACTOR_REQUEST = OUT5;
auto constexpr STARTED = OUT6;  // SEVCON: PULSE RELAY, BUZZER RELAY, GREEN LIGHT DASHBOARD
auto constexpr BMS_ON = OUT7;   // MIDPACK REQUEST, SEVCON, FRONT BATTERY BOX
auto constexpr CHARGE_POWER_BMS = OUT8;
auto constexpr MAIN_PLUS_CONTACTOR_REQUEST = OUT9;  // FRONT BATT BOX, LOW SIDE SWITCH

void init_pins();

bool ignition_on();
bool starting_on();


void set_bms_ready_power(bool on);
bool get_bms_ready_power();

void set_bms_on(bool on);
bool get_bms_on();

void set_main_min_contactor_request(bool on);
bool get_main_min_contactor_request();

void set_precharge_contactor_request(bool on);
bool get_precharge_contactor_request();