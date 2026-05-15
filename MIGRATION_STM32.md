# STM32F411 (BlackPill) Migration Guide

This document outlines the necessary and recommended steps to port the BotBoarduino CH3R PS2 codebase from the 8-bit ATmega328P (16MHz) to the 32-bit STM32F411CEU6 "BlackPill" (100MHz+, Hardware FPU).

## ⚠️ Hardware Prerequisites
1. **Physical Footprint:** The Arduino Pro Mini is a 24-pin board; the BlackPill is a 40-pin board. To plug the BlackPill into the same Bot Board II socket (= BotBoarduino board with socket), the following adaptations are made in hardware [Bot Board II - Arduino Pro Mini - Blackpill v3.1 -> Function]:
   - Left side of pinheader socket:
      SOUT - TXD   - B12  -> Not used (reserved for RS232-Rx-out)
      SIN  - RXD   - B13  -> Not used (reserved for RS232-Tx-in)
      ATN  - RST   - B14  -> Not used (reserved for RS232-RTS-in)
      GND  - GND   - nc   -> Not used (reserved for GND)
      P0   - D2    - A8   -> Not used
      P1   - D3    - A9   -> Not used
      P2   - D4    - A10  -> Not used
      P3   - D5    - A11  -> Not used
      P4   - D6    - A12  -> Not used
      P5   - D7    - A15  -> Not used
      P6   - D8    - B3   -> Not used
      P7   - D9    - B4   -> SSC-IN 
      P16  - nc    - B5   -> Not used (reserved for AIN Vservo)
      P17  - nc    - B6   -> Not used (reserved for AIN Vservo)
   - Right side of pinheader socket:
      Vin  - 5Vraw - 5Vin -> 5V supply
	  GND  - GND   - GND  -> GND
	  RSTn - RSTn  - nc   -> Not used (reserved for RSTn)
	  Vcc  - 5Vdc  - nc   -> Not used (reserved for 5V supply)
	  P15  - A3    - B2   -> PS2-CLK
	  P14  - A2    - B1   -> PS2-ATT/SEL
	  P13  - A1    - B0   -> PS2-CMD
	  P12  - A0    - A7   -> PS2-DAT
	  P11  - D13   - A6   -> Not used (reserved for NEO_OUT)
	  P10  - D12   - A5   -> Not used (reserver for NEO_IN)
	  P9   - D11   - A4   -> SOUND
	  P8   - D10   - A3   -> SSC-OUT
	  P19  - nc    - A2   -> Not used
	  P18  - nc    - A1   -> Not used
   - Legend:
      nc = not connected
	  Not used = not in use at this moment
2. **Logic Levels:** The BotBoarduino uses **5V logic**, while the STM32 uses **3.3V logic**.
   - *PS2 Receiver:* Generally runs better on 3.3V (safe).
   - *SSC-32:* The RX pin on the SSC-32 will reliably register a 3.3V signal as a logic HIGH (safe).
   - *Sensors:* Ensure no 5V analog sensors push voltage back into the STM32 analog pins without level shifting.

## Recommended Toolchain: VSCode + PlatformIO
While you can use the standard Arduino IDE with the STM32duino core, **VSCode + PlatformIO** is highly recommended for this migration.

* **Framework Compatibility:** PlatformIO uses the exact same STM32duino core under the hood. You keep `setup()`, `loop()`, and all Arduino libraries without needing to rewrite the hardware layer from scratch (which native STM32CubeIDE/HAL would require).
* **Hardware Debugging:** Moving to 32-bit ARM introduces new complexities. With PlatformIO and the available **ST-LINK v2** dongle, you gain real-time hardware debugging capabilities (setting breakpoints, stepping through kinematics math, and inspecting memory).
* **Development Setup:** Use the **USB cable** for secondary serial monitoring (e.g., `TerminalMonitor`), while the **ST-LINK v2** handles high-speed code uploads and debugging. (Note: USB is not used for power in this configuration).
* **Easy Configuration:** Board and library management is handled entirely by a single configuration file.

**Sample `platformio.ini`:**
```ini
[env:blackpill_f411ce]
platform = ststm32
board = blackpill_f411ce
framework = arduino
upload_protocol = stlink
debug_tool = stlink
```

---

## 1. Minimum Required Changes (To Compile and Walk)
These changes are required to establish basic functionality while keeping the logic as close to the original Arduino implementation as possible.

### Pin Mapping
- **File:** `Hex_Cfg.h`
- **Action:** Update all Arduino pin numbers to the STM32 Port names defined in the **Hardware Prerequisites Table** above.
- **Key Mappings:**
  - `PS2_CLK`  -> `PB2`
  - `PS2_SEL`  -> `PB1`
  - `PS2_CMD`  -> `PB0`
  - `PS2_DAT`  -> `PA7`
  - `SOUND_PIN` -> `PA4`
  - `cSSC_OUT` -> `PA3`
  - `cSSC_IN`  -> `PB4`

### Serial Communication (Phase 1: Bit-Bang)
- **Context:** To ensure the smoothest transition, continue using the `SoftwareSerial` library provided by the STM32duino core. This avoids having to reroute pins to hardware UARTs immediately.
- **Files:** `Hex_Globals.h`, `phoenix_driver_ssc32.cpp`
- **Action:**
  - Keep `#include <SoftwareSerial.h>`.
  - Ensure `cSSC_BAUD` is set to `38400` (low enough for bit-banging on STM32).
  - **🚩 RISK FLAG:** If software bit-banging on the STM32 causes serial jitter, corrupted bytes, or excessive CPU blocking, stop and implement **Hardware Serial** (USART) immediately instead of waiting for Section 2.

### PS2 Controller Timing
- **Context:** The standard `PS2X_lib` bit-bangs SPI. Because the STM32 is ~6x faster than the Pro Mini, the internal logic timing of the library may need adjustment.
- **Action:** 
  - Use an STM32-compatible version of the `PS2X_lib` (which uses `delayMicroseconds` for consistent timing regardless of CPU clock).
  - Verify initialization: The robot will sit down automatically if the PS2 controller fails to initialize.
  - **🚩 RISK FLAG:** If the software SPI driver remains unstable or fails to initialize after basic timing adjustments, pivot immediately to the **Hardware SPI** implementation using the BlackPill's dedicated SPI pins.

---

## 2. Recommended Changes (To Utilize the 32-bit Architecture)
These changes remove 8-bit legacy hacks and improve code readability.

### Remove PROGMEM Macros
- **Context:** The ATmega uses Harvard architecture, requiring `PROGMEM` and `pgm_read_word()` to read tables from Flash memory instead of RAM. The STM32 uses a unified memory map.
- **Action:** 
  - While the STM32duino core provides macros to make `pgm_read_word` compile, it is an unnecessary abstraction.
  - Delete all `pgm_read_word(&cCoxaPin[i])` calls across the codebase and replace them with standard array access: `cCoxaPin[i]`.
  - The `const` arrays in `Hex_Tables.h` will automatically be stored in Flash on the STM32.

### Implement Hardware PWM for Sound
- **Context:** The current `SoundNoTimer` function bit-bangs the buzzer pin and uses `delayMicroseconds()`. This is blocking and wastes massive amounts of 100MHz CPU time.
- **Action:** 
  - Replace the internals of the `MSound()` function in `BotBoarduino_CH3R_PS2.ino` to use the standard, non-blocking `tone(pin, frequency, duration)` function, which utilizes the STM32's hardware timers.

---

## 3. Nice to Have (The Ultimate Upgrade)
If you are moving to the STM32F411, you have a **Hardware Floating Point Unit (FPU)**. This allows you to upgrade the kinematic engine.

### Switch to Native Floating-Point Math
- **Context:** The current code uses 16-bit scaled integers (`c1DEC`, `c2DEC`, `c4DEC`) to avoid the massive performance penalty of software floating-point emulation on the AVR. The STM32 executes floating-point math in 1 clock cycle in hardware.
- **Action:**
  - **Delete Tables:** Remove the massive `GetACos` and `GetSin` lookup tables from `Hex_Tables.h`.
  - **Refactor Engine:** Go into `Leg.cpp` and `Hexapod.cpp` and remove the scaled integer math. 
  - **Use Standard Math:** Replace the integer calculations with standard `float` variables and built-in `<math.h>` functions (`sinf()`, `cosf()`, `acosf()`, `atan2f()`).
- **Benefit:** The code becomes highly readable, mathematically perfect (removing 1-degree LUT rounding errors), eliminates integer overflow risks, and paves the way for integrating modern robotics libraries (like ROS).