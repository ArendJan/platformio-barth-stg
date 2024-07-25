/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

#include "pins_arduino.h"

// Digital PinName array
const PinName digitalPin[] = {
    PA_0, // led green
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
    // D28,  // A0 - IN1
    // D29,  // A1 - IN2
    // D30,  // A2 - IN3
    // D31,  // A3 - IN4
    // D32,  // A4 - IN5
    // D33,  // A5 - IN6
};
