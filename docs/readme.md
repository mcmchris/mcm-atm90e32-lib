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

 | Members                         | Descriptions                                                                                                                       |
 | ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
 | `class` [`ATM90E32`](#ATM90E32) | The base class for the MCM Energy Monitor, it provides essential functionality and structure for all subsequent methods. |

### Electrical Parameters Methods

- `GetLineVoltageA()`: Returns the RMS voltage value of channel A 
- `GetLineVoltageB()`: Returns the RMS voltage value of channel B 
- `GetLineVoltageC()`: Returns the RMS voltage value of channel C

- `GetLineCurrentA()`: Returns the RMS current value of channel A
- `GetLineCurrentB()`: Returns the RMS current value of channel B
- `GetLineCurrentC()`: Returns the RMS current value of channel C
- `GetLineCurrentN()`: Returns the RMS current value of Neutral

- `GetActivePowerA()`: Returns the Active Power (W) of channel A
- `GetActivePowerB()`: Returns the Active Power (W) of channel B
- `GetActivePowerC()`: Returns the Active Power (W) of channel C

- `GetTotalActivePower()`: Returns the Active Power (W) of all channels
- `GetTotalActiveFundPower()`: Returns the total active fundamental power 
- `GetTotalActiveHarPower()`: Returns the total active harmonical power

- `GetReactivePowerA()`: Returns the Reactive Power (VAR) of channel A
- `GetReactivePowerB()`: Returns the Reactive Power (VAR) of channel B
- `GetReactivePowerC()`: Returns the Reactive Power (VAR) of channel C
- `GetTotalReactivePower()`: Returns the Reactive Power (VAR) of all channels

- `GetApparentPowerA()`: Returns the Apparent Power (VA) of channel A
- `GetApparentPowerB()`: Returns the Apparent Power (VA) of channel B
- `GetApparentPowerC()`: Returns the Apparent Power (VA) of channel C
- `GetTotalApparentPower()`: Returns the Apparent Power (VA) of all channels

- `GetFrequency()`: Returns the grid frequency (Hz)

- `GetPowerFactorA()`: Returns the Power Factor of channel A
- `GetPowerFactorB()`: Returns the Power Factor of channel B
- `GetPowerFactorC()`: Returns the Power Factor of channel C
- `GetTotalPowerFactor()`: Returns the mean Power Factor of all channels

- `GetPhaseA()`: Returns the Phase Angle of channel A
- `GetPhaseB()`: Returns the Phase Angle of channel B
- `GetPhaseC()`: Returns the Phase Angle of channel C

- `GetImportEnergy()`: Returns the active energy imported in (Wh)
- `GetImportReactiveEnergy()`: Returns the reactive energy imported in (VARh)
- `GetImportApparentEnergy()`: Returns the apparent energy imported in (VAh)
- `GetExportEnergy()`: Returns the active energy exported in (Wh)
- `GetExportReactiveEnergy()`: Returns the reactive energy exported in (VARh)

### Calibration Methods

- `CalculateVIOffset(unsigned short regh_addr, unsigned short regl_addr, unsigned short offset_reg)`: Returns the offset of the voltage and current channels to be used within the `setVIOffset()` function.

This function admits several parameters, see the following example as reference:

```arduino
void VIoffsetCal()
{
  // Voltage Offset Calibration
  mcm.CalculateVIOffset(UrmsA, UrmsALSB, UoffsetA);
  mcm.CalculateVIOffset(UrmsB, UrmsBLSB, UoffsetB);
  mcm.CalculateVIOffset(UrmsC, UrmsCLSB, UoffsetC);

  // Current Offset Calibration
  mcm.CalculateVIOffset(IrmsA, IrmsALSB, IoffsetA);
  mcm.CalculateVIOffset(IrmsB, IrmsBLSB, IoffsetB);
  mcm.CalculateVIOffset(IrmsC, IrmsCLSB, IoffsetC);
}
```

The `VIoffsetCal()` function from above will calculate and return the offset of each input channel of the system. Take notes of the results because will be needed for the next method explained.

**Note:** The offset must be calculated without voltage and current input. No grid voltage and no powered loads connected to the CTs.

- `setVIOffset(unsigned short VoffA, unsigned short VoffB, unsigned short VoffC, unsigned short IoffA, unsigned short IoffB, unsigned short IoffC)`: Configures the offset for each channel to compensate the initial error of the ADC input stages. 

This function admits several parameters, see the following example as reference:

```arduino
mcm.setVIOffset(64608, 64608, 64608, 64606, 64608, 64606); // Values obtained with VIoffsetCal()
```

**Note:** Enter the offset values in the same order the `VIoffsetCal()` function prints them.

- `CalculatePowerOffset(unsigned short regh_addr, unsigned short regl_addr, unsigned short offset_reg)`: Returns the offset of the power registers. Optional depending on your setup.

- `CalibrateVI(unsigned short reg, float actualVal)`: Returns the calibration factor of your voltage and current readings using a known voltage and current.

This function admits the register to be calibrated and the voltage or current value measured by a reference tool. See the following example for reference:

- I measured a voltage of **121.54 VAC** with my lab multimeter. (The MCM Split-Phase Energy Monitor only uses channels A and C for voltage).
- I measured a current of **0.4815 Amps** with my lab multimeter. (The MCM Split-Phase Energy Monitor only uses channels A and C for current).

With those known values currently being present in the system input, run the following functions:

```arduino
  VCalibration(121.54, 0, 121.54); // Voltage in A, Voltage in B, Voltage in C

  ...

  void VCalibration(float VoltA, float VoltB, float VoltC)
{
  // Voltage Offset Calibration
  mcm.CalibrateVI(UrmsA, VoltA); // Voltage Channel A, Actual Reference Voltage
  mcm.CalibrateVI(UrmsB, VoltB); // Voltage Channel B, Actual Reference Voltage
  mcm.CalibrateVI(UrmsC, VoltC); // Voltage Channel C, Actual Reference Voltage
  delay(5000);
}
```

```arduino
  ICalibration(0.4815, 0, 0.4815); // Current in A, Current in B, Current in C

  ...

  void ICalibration(float CurrA, float CurrB, float CurrC)
{
  // Voltage Offset Calibration
  mcm.CalibrateVI(IrmsA, CurrA); // Voltage Channel A, Actual Reference Voltage
  mcm.CalibrateVI(IrmsB, CurrB); // Voltage Channel B, Actual Reference Voltage
  mcm.CalibrateVI(IrmsC, CurrC); // Voltage Channel C, Actual Reference Voltage
  delay(1000);
}
```
  
The functions from above will print in the Serial Monitor the calibration factors of each channel. Write them down and use them to populate the `begin()` function of the ATM90E32 class as explained in the [beginning](#library-initialization).


