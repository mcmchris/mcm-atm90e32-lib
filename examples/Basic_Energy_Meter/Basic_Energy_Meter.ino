#include <SPI.h>
#include <MCM_ATM90E32.h>

#include <SimpleTimer.h>

SimpleTimer Timer(10000);
SimpleTimer Timer2(60000);

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
unsigned short VoltageCal = 25256;
unsigned short CT1Cal = 33010;
unsigned short CT2Cal = 33010;

const int CS_pin = SS;  // Use default SS pin for unknown Arduino

ATM90E32 mcm{};  //initialize the IC class

#define ONLY_ENERGY 0
#define ALL_VALUES 1

char act_energy_buff[32];
char apa_energy_buff[32];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  //while (!Serial)
  //  ;

  pinMode(USR_BTN, INPUT);
  pinMode(LED_MCM, OUTPUT);

  Serial.println("Start ATM90E32");
  mcm.begin(CS_pin, LineFreq, SumMode, IAGain, IBGain, ICGain, VoltageCal, CT1Cal, 0, CT2Cal);
  delay(1000);
  unsigned short sys0 = mcm.GetSysStatus0();  //EMMState0

  if (sys0 == 65535 || sys0 == 0) {
    Serial.println("Error: Not receiving data from energy meter - check your connections");
    while (1) {
    }
  }
  delay(250);
}

float voltage;
float currentA, currentB;
float frequency;
float pf_a, pf_b;
float powerA, powerB;
float t_pow, t_aparent_pow;
float temperature;
double act_energy_wh, apa_energy_wh;

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    unsigned short sys0 = mcm.GetSysStatus0();  //EMMState0

    if (sys0 == 65535 || sys0 == 0) {
      Serial.println("Error: Not receiving data from energy meter - check your connections");

    } else {
      voltage = mcm.GetLineVoltageA();
      Serial.print(voltage, 2);
      Serial.println("[V]");

      currentA = mcm.GetLineCurrentA();
      currentB = mcm.GetLineCurrentC();
      Serial.print("IA: ");
      Serial.print(currentA, 4);
      Serial.print(" [A],");
      Serial.print(" IB: ");
      Serial.print(currentB, 4);
      Serial.println(" [A]");

      pf_a = mcm.GetPowerFactorA();
      pf_b = mcm.GetPowerFactorC();
      Serial.print("[PF A]: ");
      Serial.print(pf_a, 2);
      Serial.print(", [PF B]: ");
      Serial.println(pf_b, 2);

      frequency = mcm.GetFrequency();
      Serial.print("Frequency: ");
      Serial.print(frequency, 2);
      Serial.println(" [Hz]");

      powerA = mcm.GetActivePowerA();
      powerB = mcm.GetActivePowerC();
      Serial.print("A: ");
      Serial.print(powerA);
      Serial.print(" [W], ");
      Serial.print("B: ");
      Serial.print(powerB);
      Serial.println(" [W]");


      Serial.print("Active Energy: ");
      Serial.print(act_energy_wh / 1000.0, 6);
      Serial.print(" [kWh], ");
      Serial.print("Aparent Energy: ");
      Serial.print(apa_energy_wh / 1000.0, 6);
      Serial.println(" [kVAh]");


      temperature = mcm.GetTemperature();
      Serial.print("IC Temperature: ");
      Serial.print(temperature, 2);
      Serial.println(" [C]");

      t_pow = mcm.GetTotalActivePower();
      t_aparent_pow = mcm.GetTotalApparentPower();
    }

    //Serial.println();
  }

  if (Timer2.isReady()) {
    // Remember what was saved in the last energy reading


    Serial.println("Checking for new energy values:");
    // I add the current energy to reading
    act_energy_wh += mcm.GetImportEnergy();
    apa_energy_wh += mcm.GetImportApparentEnergy();

    Serial.print("Active Energy: ");
    Serial.print(act_energy_wh / 1000.0, 6);
    Serial.print(" [kWh], ");
    Serial.print("Aparent Energy: ");
    Serial.print(apa_energy_wh / 1000.0, 6);
    Serial.println(" [kVAh]");

    snprintf(act_energy_buff, sizeof(act_energy_buff), "%.10f", act_energy_wh);
    snprintf(apa_energy_buff, sizeof(apa_energy_buff), "%.10f", apa_energy_wh);

    // I store the new data in memory for further reading


    Timer2.reset();
  }


  button_handler();
}


/***** BUTTON FUNCTIONS *****/
/* 
 * 1s Press: Custom Free Action
 * 4s Press: Reset ESP32 
 * 8s Press: Reset Energy Count
 */

void button_handler() {
  if (digitalRead(USR_BTN) == LOW) {  //Push button pressed

    // measures time pressed
    int startTime = millis();
    int elapsedTime = 0;
    while (digitalRead(USR_BTN) == LOW) {

      elapsedTime = (millis() - startTime) / 1000.0;

      if (elapsedTime <= 7) {

      } else {

        delay(500);
        break;
      }
    }

    if (elapsedTime >= 8) {  // 8 second press
      Serial.println("Reseting the Energy count and clearing Registers");
      delay(200);

      act_energy_wh = 0;
      apa_energy_wh = 0;

    } else if (elapsedTime >= 4) {  // 4 second press
      Serial.println("MCU Resetting Started");
      //ESP.restart();
    } else if (elapsedTime >= 1) {  // 1 second press

      Serial.println("Custom Action Here!");
    }
    elapsedTime = 0;
  }
}
