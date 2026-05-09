//==============================================================================
// InputController.h - Abstract API for robot input control
//==============================================================================
#ifndef _INPUT_CONTROLLER_h_
#define _INPUT_CONTROLLER_h_

#include "Hex_Cfg.h"
#include "ControlState.h"

class InputController {
public:
    // Initialize the input hardware
    void Init(void);
    
    // Poll for new input and update the global INCONTROLSTATE
    void ControlInput(void);
    
    // Manage hardware-specific interrupts (if applicable)
    void AllowControllerInterrupts(boolean fAllow);

    // Check if a hardware diagnostic mode (like SSC Forwarder) is requested at boot
    bool FIsDiagnosticModeRequested(void);
};   

#endif //_INPUT_CONTROLLER_h_
