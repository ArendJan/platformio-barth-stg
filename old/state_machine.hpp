#pragma once
#include <Arduino.h>

void init_state_machine();
void run_state_machine();

enum class State {
    HIBERNATE,            // CAR OFF
    PREPARE_TO_DRIVE,     //
    STARTING,             // IGN ON
    STARTED,              // DRIVING
    STOPPING,             // TURN OFF DRIVING
    SHUTTING_DOWN,        // TURN OFF HV
    PREPARE_CHARGING_HV,  // TURN ON HV
    PREPARE_CHARGING,     // TURN ON CHARGERS?
    CHARGING,             // CHARGING
    STOP_CHARGING,        // TURN OF CHARGER
    ERROR                 // ERROR
};

class States {
public:
    void state_hibernate();
    void not_hibernating();

    void prepare_to_drive();
    void not_preparing_to_drive();
    bool prepare_to_drive_done = false;
    int prepare_to_drive_step = 0;
    decltype(millis()) prepare_to_drive_time = 0;
};
extern State state;
extern States states;