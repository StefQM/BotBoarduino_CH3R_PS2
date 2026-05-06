#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include "../mocks/Arduino.h"
#include "../mocks/SoftwareSerial.h"
#include "../mocks/PS2X_lib.h"
#include "../../Hex_Globals.h"
#include "../../InputController.h"
#include "../../ServoDriver.h"

// Prevent setup/loop collision
#define setup robot_setup
#define loop robot_loop

// We need to handle the fact that .ino files don't have function prototypes 
// for functions defined later in the same file when compiled as standard C++.
// But for now, let's just include the logic.

#endif
