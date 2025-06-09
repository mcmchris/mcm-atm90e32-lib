#define MODULINO
#define HELPER_IMPLEMENTATION

#include "helper.h"

#include <SPI.h>
#include <MCM_ATM90E32.h>

#include <SimpleTimer.h>

SimpleTimer Timer(10000);
SimpleTimer Timer2(60000);

#define USR_BTN A1
#define LED_MCM A0

#define FORMAT_SPIFFS_IF_FAILED false

unsigned long previousMillis = 0;

// constants won't change:
const long interval = 1000;

/***** CALIBRATION SETTINGS *****/
/* 
 * 60 Hz (America) or 50 Hz (rest of the world)
 * Energy Accumulation Method
 */
unsigned short LineFreq = 60;  // 
unsigned short SumMode = 1; // 0: Aritmethic Energy Sum, 1: Absolute Energy Sum

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

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    delay(10000);
    ESP.restart();
    return;
  }

  listDir(SPIFFS, "/", 0);

  if (file_names[0] == "activeEnergy.txt" && file_names[1] == "aparentEnergy.txt") {
    Serial.println("Files already there");
  } else {
    Serial.println("Creating files");
    writeFile(SPIFFS, "/activeEnergy.txt", "0");
    writeFile(SPIFFS, "/aparentEnergy.txt", "0");
  }

  // Remember what was saved in the last energy reading
  readActEnergy(SPIFFS, "/activeEnergy.txt");
  readApaEnergy(SPIFFS, "/aparentEnergy.txt");

#ifdef MODULINO

  Modulino.begin();
  leds.begin();

#endif

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

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    unsigned short sys0 = mcm.GetSysStatus0();  //EMMState0

    if (sys0 == 65535 || sys0 == 0) {
      Serial.println("Error: Not receiving data from energy meter - check your connections");
      ERROR_PIXEL(0);
    } else {
      voltage = mcm.GetLineVoltageA();
      Serial.printf("%.2f [V]\n", voltage);

      currentA = mcm.GetLineCurrentA();
      currentB = mcm.GetLineCurrentC();
      Serial.printf("IA: %.4f [A], IB: %.4f [A], \n", currentA, currentB);

      pf_a = mcm.GetPowerFactorA();
      pf_b = mcm.GetPowerFactorC();
      Serial.printf("%.2f [PF A], %.2f [PF B]\n", pf_a, pf_b);

      frequency = mcm.GetFrequency();
      Serial.printf("%.2f [Hz]\n", frequency);

      powerA = mcm.GetActivePowerA();
      powerB = mcm.GetActivePowerC();
      Serial.printf("A: %.2f [W], B: %.2f [W]\n", powerA, powerB);
#ifdef MODULINO
      Feedback(powerA, powerB);
#endif
      Serial.printf("Active: %.6f [kWh], Apparent: %.6f [kVAh]\n", act_energy_wh / 1000.0, apa_energy_wh / 1000.0);

      temperature = mcm.GetTemperature();
      Serial.printf("IC Temperature: %.2f [C]\n", temperature);

      t_pow = mcm.GetTotalActivePower();
      t_aparent_pow = mcm.GetTotalApparentPower();
    }

    //Serial.println();
  }

  if (Timer2.isReady()) {
    // Remember what was saved in the last energy reading
    readActEnergy(SPIFFS, "/activeEnergy.txt");
    readApaEnergy(SPIFFS, "/aparentEnergy.txt");

    Serial.println("Checking for new energy values:");
    // I add the current energy to reading
    act_energy_wh += mcm.GetImportEnergy();
    Serial.printf("%.10f [Wh]\n", act_energy_wh);
    apa_energy_wh += mcm.GetImportApparentEnergy();
    Serial.printf("%.10f [VAh]\n", apa_energy_wh);

    snprintf(act_energy_buff, sizeof(act_energy_buff), "%.10f", act_energy_wh);
    snprintf(apa_energy_buff, sizeof(apa_energy_buff), "%.10f", apa_energy_wh);

    // I store the new data in file system for further reading
    writeFile(SPIFFS, "/activeEnergy.txt", act_energy_buff);
    writeFile(SPIFFS, "/aparentEnergy.txt", apa_energy_buff);

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
#ifdef MODULINO
    for (int i = 0; i < 8; i++) {
      leds.set(i, 0, 0, 0, bright);
      leds.show();
    }
#endif
    // measures time pressed
    int startTime = millis();
    int elapsedTime = 0;
    while (digitalRead(USR_BTN) == LOW) {

      elapsedTime = (millis() - startTime) / 1000.0;

      //Serial.printf("Button pressed for: %d seconds\r\n", elapsedTime);

      if (elapsedTime <= 7) {
#ifdef MODULINO
        leds.set(elapsedTime, BLUE, bright);
        //leds.set(i, colorPower, 127);
        leds.show();
#endif
      } else {
#ifdef MODULINO
        for (int i = 0; i < 8; i++) {
          leds.set(i, VIOLET, bright);
          leds.show();
        }
#endif
        delay(500);
        break;
      }
    }

    if (elapsedTime >= 8) {  // 8 second press
      Serial.println("Reseting the Energy count and clearing Registers");
      delay(200);
      act_energy_wh = 0;
      apa_energy_wh = 0;
      writeFile(SPIFFS, "/activeEnergy.txt", "0");
      writeFile(SPIFFS, "/aparentEnergy.txt", "0");
      delay(200);
      // Remember what was saved in the last energy reading
      readActEnergy(SPIFFS, "/activeEnergy.txt");
      readApaEnergy(SPIFFS, "/aparentEnergy.txt");

    } else if (elapsedTime >= 4) {  // 4 second press
      Serial.println("ESP32 Resetting Started");
      ERROR_PIXEL(2);
      ESP.restart();

    } else if (elapsedTime >= 1) {  // 1 second press
 
      ERROR_PIXEL(3);

      Serial.println("Custom Action Here!");

    }
    elapsedTime = 0;
#ifdef MODULINO
    for (int i = 0; i < 8; i++) {
      leds.set(i, 0, 0, 0, bright);
      leds.show();
    }
#endif
  }
}
