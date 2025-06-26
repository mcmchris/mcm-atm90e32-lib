# MCM ATM90E32 Library for Arduino

The MCM_ATM90E32 Library simplifies the integration of the ATM90E32AS Energy Meter IC. It provides an easy-to-use interface and API to monitor 3-Phase or Split-Phase electrical systems with Arduino boards.

## Hardware Compatibility

While the MCM Split-Phase Energy Monitor is designed for the **Nano** form factor, this library is compatible with all Arduino boards that feature an SPI interface.

## Library Initialization

To initialize the MCM ATM90E32 library, include the header file and call the `begin()` method passing the IC register configurations. This will set up the following:

- **SPI communication:** The first parameter sets the **Chip Select (CS)** pin to be used alongside with the default SPI bus of the host board.
- **Grid frequency:** The second parameter sets the grid frequency, letting you select between:
  - **60 Hz** = 60
  - **50 Hz** = 50
- **Energy accumulation:** The third parameter sets the energy accumulation method, letting you select between:
  - **Arithmetic Sum** = (0)
  - **Absolute Sum** = (1)
- **Current channels gain:** The fourth, fifth and sixth parameters set the current sensor inputs gain, letting you select between for each current channel:
  - **1x** = 1 (CTs up to 60mA/720mV)
  - **2x** = 2 (CTs up to 30mA/360mV)
  - **4x** = 4 (CTs up to 15mA/180mV)
- **Voltage and current calibration:** The seventh, eighth, ninth and tenth parameters fine tune the voltage and current measurements. These values are obtained with the calibration workflow.

```arduino
#include <SPI.h>
#include <MCM_ATM90E32.h>

/***** CALIBRATION SETTINGS *****/
/* 
 * 60 Hz (America) or 50 Hz (rest of the world)
 * Energy Accumulation Method
 */
unsigned short LineFreq = 60;  //
unsigned short SumMode = 1;    // 0: Aritmethic Energy Sum, 1: Absolute Energy Sum

/* 
 * 1x (CTs up to 60mA/720mV)
 * 2x (CTs up to 30mA/360mV)
 * 4x (CTs up to 15mA/180mV)
 */
unsigned short IAGain = 2;
unsigned short IBGain = 2;
unsigned short ICGain = 2;

/*
* Calibration values obtained with a known load
*/
unsigned short VoltageCal = 25256; // Value obtained with Calibration Workflow
unsigned short CT1Cal = 32303; // Value obtained with Calibration Workflow
unsigned short CT2Cal = 32401; // Value obtained with Calibration Workflow

const int CS_pin = SS;  // Use default SS pin for unknown Arduino

ATM90E32 mcm{};  //initialize the IC class

void setup() {
  mcm.begin(CS_pin, LineFreq, SumMode, IAGain, IBGain, ICGain, VoltageCal, CT1Cal, 0, CT2Cal);
}

...
```

# MCM ATM90E32 Library API

## Summary

 Members                                    | Descriptions
--------------------------------------------|------------------------------------------
| `class` [`ModulinoClass`](#modulinoclass) | The base class for all Modulino components, providing essential functionality and structure for all subsequent Modulino modules.                                  |
| `class` [`ModulinoButtons`](#modulinobuttons) | Handles the functionality of Modulino Buttons, enabling detection of presses and handling input events.                                                |
| `class` [`ModulinoBuzzer`](#modulinobuttons) |Handles the functionality of Modulino Buzzer, enabling the sound generation for feedback or alerts.                                                    |
| `class` [`ModulinoPixels`](#modulinopixels) | Handles the functionality of Modulino Pixels, managing LEDs strip color, status and brightness.                                          |
| `class` [`ModulinoKnob`](#modulinoknob) | Handles the functionality of Modulino Knob, interfacing with the rotary knob position.                                                     |
| `class` [`ModulinoMovement`](#modulinomovement) | Handles the functionality of Modulino Movement,interfacing with the IMU sensor to get acceleration readings. |
| `class` [`ModulinoThermo`](#modulinothermo) | Handles the functionality of Modulino Thermo, managing temperature sensors to provide real-time temperature and humidity readings.                                |
| `class` [`ModulinoDistance`](#modulinodistance) | Handles the functionality of Modulino Distance, enabling distance measurement using ToF (Time-of-Flight) sensors for precise range detection. |
