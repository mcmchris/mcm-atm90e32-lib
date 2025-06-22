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
- 