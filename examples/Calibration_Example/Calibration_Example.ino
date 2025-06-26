/*CALIBRATION EXAMPLE

- Run the program with no Voltage and Current input during the setup() function initial execution.
This will calibrate the voltage and current offset.

- With a reference voltage and current meter, measure the respective parameters on your bench setup with a known load.
Then, update the VCalibration() and ICalibration() functions respectively.

- Reupload the code to the board with the new parameters off Vcalibration() and ICalibration().

- Press the board user button for 4 seconds to get the calibration parameters to match the real measured values by your
reference tool.

- Take notes of the calibration parameters and replace the VoltageCal, CT1Cal and CT2Cal variables to use them in the
.begin() function of your final code.
*/

#include <SPI.h>
#include <MCM_ATM90E32.h>

#define USR_BTN A1
#define LED_MCM A0

unsigned long previousMillis = 0;

// constants won't change:
const long interval = 1000;

/***** CALIBRATION SETTINGS *****/
/*
 * 60 Hz (America) or 50 Hz (rest of the world)
 * Energy Accumulation Method
 */
unsigned short LineFreq = 60; //
unsigned short SumMode = 1;   // 0: Aritmethic Energy Sum, 1: Absolute Energy Sum

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
unsigned short VoltageCal = 25256; // Value obtained with Calibration Workflow (Update with yours)
unsigned short CT1Cal = 32303;     // Value obtained with Calibration Workflow (Update with yours)
unsigned short CT2Cal = 32401;     // Value obtained with Calibration Workflow (Update with yours)

const int CS_pin = SS; // Use default SS pin for unknown Arduino

ATM90E32 mcm{}; // initialize the IC class

double act_energy_wh, apa_energy_wh;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  // while (!Serial)
  //   ;

  pinMode(USR_BTN, INPUT);
  pinMode(LED_MCM, OUTPUT);

  Serial.println("Start ATM90E32");
  mcm.begin(CS_pin, LineFreq, SumMode, IAGain, IBGain, ICGain, VoltageCal, CT1Cal, 0, CT2Cal);
  mcm.setVIOffset(64608, 64608, 64608, 64606, 64608, 64606); // Values obtained with VIoffsetCal()
  delay(1000);
  unsigned short sys0 = mcm.GetSysStatus0(); // EMMState0

  if (sys0 == 65535 || sys0 == 0)
  {
    Serial.println("Error: Not receiving data from energy meter - check your connections");
    while (1)
    {
    }
  }
  delay(250);

  // The Voltage and Current inputs must be 0 when running the Offset calibration - Grid voltage = 0 and CT sensors not hooked to cables.
  
  //VIoffsetCal();    // to obtain the offset values to use in setVIoffset()
  //PowerOffsetCal(); // not necessary, but possible
}

void loop()
{

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    unsigned short sys0 = mcm.GetSysStatus0(); // EMMState0

    if (sys0 == 65535 || sys0 == 0)
    {
      Serial.println("Error: Not receiving data from energy meter - check your connections");
    }
    else
    {
      float voltage = mcm.GetLineVoltageA();
      Serial.print(voltage, 3);
      Serial.println("[V]");

      float currentA = mcm.GetLineCurrentA();
      float currentB = mcm.GetLineCurrentC();
      Serial.print("IA: ");
      Serial.print(currentA, 4);
      Serial.print(" [A],");
      Serial.print(" IB: ");
      Serial.print(currentB, 4);
      Serial.println(" [A]");

      float pf_a = mcm.GetPowerFactorA();
      float pf_b = mcm.GetPowerFactorC();
      Serial.print("[PF A]: ");
      Serial.print(pf_a, 2);
      Serial.print(", [PF B]: ");
      Serial.println(pf_b, 2);

      float frequency = mcm.GetFrequency();
      Serial.print("Frequency: ");
      Serial.print(frequency, 2);
      Serial.println(" [Hz]");

      float powerA = mcm.GetActivePowerA();
      float powerB = mcm.GetActivePowerC();
      Serial.print("A: ");
      Serial.print(powerA);
      Serial.print(" [W], ");
      Serial.print("B: ");
      Serial.print(powerB);
      Serial.println(" [W]");

      act_energy_wh += mcm.GetImportEnergy();
      apa_energy_wh += mcm.GetImportApparentEnergy();

      Serial.print("Active Energy: ");
      Serial.print(act_energy_wh / 1000.0, 6);
      Serial.print(" [kWh], ");
      Serial.print("Aparent Energy: ");
      Serial.print(apa_energy_wh / 1000.0, 6);
      Serial.println(" [kVAh]");

      float temperature = mcm.GetTemperature();
      Serial.print("IC Temperature: ");
      Serial.print(temperature, 2);
      Serial.println(" [C]");

      float t_pow = mcm.GetTotalActivePower();
      float t_aparent_pow = mcm.GetTotalApparentPower();

      Serial.println();
    }
  }

  button_handler();
}

/***** BUTTON FUNCTIONS *****/
/*
 * 1s Press: Custom Free Action
 * 4s Press: Reset ESP32
 * 8s Press: Reset Energy Count
 */

void button_handler()
{
  if (digitalRead(USR_BTN) == LOW)
  { // Push button pressed

    // measures time pressed
    int startTime = millis();
    int elapsedTime = 0;
    while (digitalRead(USR_BTN) == LOW)
    {
      elapsedTime = (millis() - startTime) / 1000.0;
    }

    if (elapsedTime >= 8)
    { // 8 second press
      Serial.println("Reseting the Energy count and clearing Registers");
      delay(200);
      act_energy_wh = 0;
      apa_energy_wh = 0;
    }
    else if (elapsedTime >= 4)
    { // 4 second press
      Serial.println("Calibrating Voltage and Current!");

      VCalibration(121.54, 0, 121.54); // Voltage in A, Voltage in B, Voltage in C
      ICalibration(0.4815, 0, 0.4815); // Current in A, Current in B, Current in C

      Serial.println("Take notes of the new gain values and replace them on the .begin() function of your final code");
    }
    else if (elapsedTime >= 1)
    { // 1 second press
      Serial.println("Custom Action Here!");
    }
    elapsedTime = 0;
  }
}

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


/*
  - Calibrate the Voltage Readings
  - VoltA: Enter the real A voltage reference value
  - VoltB: Enter the real B voltage reference value
  - VoltC: Enter the real C voltage reference value
  - In the MCM Split-Phase Energy Monitor only A and C are used
*/
void VCalibration(float VoltA, float VoltB, float VoltC)
{
  // Voltage Offset Calibration
  mcm.CalibrateVI(UrmsA, VoltA); // Voltage Channel A, Actual Reference Voltage
  mcm.CalibrateVI(UrmsB, VoltB); // Voltage Channel B, Actual Reference Voltage
  mcm.CalibrateVI(UrmsC, VoltC); // Voltage Channel C, Actual Reference Voltage
  delay(5000);
}

/*
  - Calibrate the Current Readings
  - CurrA: Enter the real A current reference value
  - CurrB: Enter the real B current reference value
  - CurrC: Enter the real C current reference value
  - In the MCM Split-Phase Energy Monitor only A and C are used
*/
void ICalibration(float CurrA, float CurrB, float CurrC)
{
  // Voltage Offset Calibration
  mcm.CalibrateVI(IrmsA, CurrA); // Voltage Channel A, Actual Reference Voltage
  mcm.CalibrateVI(IrmsB, CurrB); // Voltage Channel B, Actual Reference Voltage
  mcm.CalibrateVI(IrmsC, CurrC); // Voltage Channel C, Actual Reference Voltage
  delay(1000);
}

void PowerOffsetCal()
{

  // Power Offset Calibration
  mcm.CalculatePowerOffset(PmeanA, PmeanALSB, PoffsetA);
  mcm.CalculatePowerOffset(QmeanA, QmeanALSB, QoffsetA);

  mcm.CalculatePowerOffset(PmeanB, PmeanBLSB, PoffsetB);
  mcm.CalculatePowerOffset(QmeanB, QmeanBLSB, QoffsetB);

  mcm.CalculatePowerOffset(PmeanC, PmeanCLSB, PoffsetC);
  mcm.CalculatePowerOffset(QmeanC, QmeanCLSB, QoffsetC);

  // Fundamental Power Offset Calibration
  mcm.CalculatePowerOffset(PmeanC, PmeanCLSB, PoffsetAF);
  mcm.CalculatePowerOffset(PmeanC, PmeanCLSB, PoffsetBF);
  mcm.CalculatePowerOffset(PmeanC, PmeanCLSB, PoffsetCF);
}
