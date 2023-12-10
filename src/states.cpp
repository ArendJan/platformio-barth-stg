#include <state_machine.hpp>
#include <pins.hpp>
void States::state_hibernate()
{
    if (ignition_on()) {
        state = State::PREPARE_TO_DRIVE;
    }
}
void States::not_hibernating() {}

auto const prepare_to_drive_time_interval = 100;
auto const prepare_to_drive_time_precharge = 5000;
void States::prepare_to_drive()
{
    if (!ignition_on()) {
        state = State::SHUTTING_DOWN;
    }
    if (starting_on() && this->prepare_to_drive_done) {
        state = State::STARTING;
    }
    // steps for prepare to drive
    // Auto is bijna aan
    //      Contactslot 1e positie (dus radio etc.)
    // BMS ready power pin (als eerste??)
    // Probeert HV aan te zetten, dus midpack, bms on
    // Main min
    // Precharge
    // Check voltage sevcon
    //      T = 5 sec
    // Main plus
    if (this->prepare_to_drive_step == 0 || !get_bms_ready_power()) {
        set_bms_ready_power(true);
        this->prepare_to_drive_time = millis();
        this->prepare_to_drive_step = 1;
    } else if (this->prepare_to_drive_step == 1) {
        if (millis() - this->prepare_to_drive_time > prepare_to_drive_time_interval) {
            set_bms_on(true);  // also midpack request
            this->prepare_to_drive_time = millis();
            this->prepare_to_drive_step = 2;
        }
    } else if (this->prepare_to_drive_step == 2 || !get_bms_on()) {
        if (millis() - this->prepare_to_drive_time > prepare_to_drive_time_interval) {
            set_main_min_contactor_request(true);
            this->prepare_to_drive_time = millis();
            this->prepare_to_drive_step = 3;
        }
    } else if (this->prepare_to_drive_step == 3 || !get_main_min_contactor_request()) {
        if (millis() - this->prepare_to_drive_time > prepare_to_drive_time_precharge) {
            set_precharge_contactor_request(true);
            this->prepare_to_drive_time = millis();
            this->prepare_to_drive_step = 4;
        }
    }
}
void States::not_preparing_to_drive()
{
    this->prepare_to_drive_step = 0;
    this->prepare_to_drive_time = millis();
}