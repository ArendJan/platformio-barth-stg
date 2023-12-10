#include <state_machine.hpp>

State state;
States states;
bool updated_state = false;
void init_state_machine()
{
    // TODO: check if rebooting from a crash
    // shouldn't be

    state = State::HIBERNATE;
    updated_state = true;
}

void run_state_machine()
{
    switch (state) {
        case State::HIBERNATE:
            states.state_hibernate();
            break;

        case State::PREPARE_TO_DRIVE:
            states.prepare_to_drive();
            break;
        case State::STARTING:
            break;
    }
    if (updated_state) { // only reset state variables when state changes
        updated_state = false;
        if (state != State::HIBERNATE) {
            states.not_hibernating();
        }
        if (state != State::PREPARE_TO_DRIVE) {
            states.not_preparing_to_drive();
        }
    }
}

void next_state(State next)
{
    state = next;
    updated_state = true;
}