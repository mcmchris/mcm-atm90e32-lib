#pragma once
#include <Arduino.h>
#include <FS.h>
#include "SPIFFS.h"

#ifdef MODULINO
#include <Modulino.h>
extern ModulinoPixels leds;
extern int bright;
extern ModulinoColor OFF;
extern ModulinoColor YELLOW;
#endif

extern String file_names[2];
extern String act_energy_str, apa_energy_str;
extern double act_energy_wh, apa_energy_wh;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void readActEnergy(fs::FS &fs, const char *path);
void readApaEnergy(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void ERROR_PIXEL(byte error);

#ifdef MODULINO
void Feedback(int powerA, int powerB);
void Modulino_blink(ModulinoColor rgb);
#endif

// ========== IMPLEMENTATION ==========

#define HELPER_IMPLEMENTATION

#ifdef HELPER_IMPLEMENTATION

#ifdef MODULINO
ModulinoPixels leds;
int bright = 5;
ModulinoColor OFF(0, 0, 0);
ModulinoColor YELLOW(255, 255, 0);
#endif

String file_names[2] = {};
String act_energy_str, apa_energy_str;
double act_energy_wh = 0;
double apa_energy_wh = 0;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) {
    Serial.println("- failed to open or invalid directory");
    return;
  }

  File file = root.openNextFile();
  byte cnt = 0;
  while (file) {
    if (!file.isDirectory()) file_names[cnt++] = file.name();
    file = root.openNextFile();
  }

  for (int i = 0; i < cnt; i++) {
    Serial.printf("File #%d: %s\n", i, file_names[i].c_str());
  }
}

void readActEnergy(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }

  act_energy_str = "";
  while (file.available()) {
    act_energy_str += (char)file.read();
  }
  file.close();

  char *endptr;
  act_energy_wh = strtod(act_energy_str.c_str(), &endptr);
  if (*endptr != '\0') Serial.println("Warning: Invalid characters in active energy");
  Serial.printf("Active WH: %.10f\r\n", act_energy_wh);
}

void readApaEnergy(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }

  apa_energy_str = "";
  while (file.available()) {
    apa_energy_str += (char)file.read();
  }
  file.close();

  char *endptr;
  apa_energy_wh = strtod(apa_energy_str.c_str(), &endptr);
  if (*endptr != '\0') Serial.println("Warning: Invalid characters in apparent energy");
  Serial.printf("Apparent VAh: %.10f\r\n", apa_energy_wh);
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void ERROR_PIXEL(byte error) {
#ifdef MODULINO
  switch (error) {
    case 0: Modulino_blink(RED); break;
    case 1: Modulino_blink(GREEN); break;
    case 2: Modulino_blink(YELLOW); break;
    case 3: Modulino_blink(BLUE); break;
    default: break;
  }
#endif
}

#ifdef MODULINO
void Feedback(int powerA, int powerB) {
  int t_power = powerA + powerB;
  int count = map(t_power, 0, 3200, 1, 8);
  if (count > 8) count = 8;

  for (int i = 0; i < count; i++) {
    if (i < 3) leds.set(i, GREEN, bright);
    else if (i < 6) leds.set(i, 255, 255, 0, bright);
    else leds.set(i, RED, bright);
    leds.show();
  }
  for (int i = count; i < (9 - count); i++) {
    leds.set(i, 0, 0, 0, bright);
    leds.show();
  }
}

void Modulino_blink(ModulinoColor rgb) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 8; j++) {
      leds.set(j, rgb, bright);
      leds.show();
    }
    delay(200);
    for (int j = 0; j < 8; j++) {
      leds.set(j, OFF, bright);
      leds.show();
    }
    delay(200);
  }
}
#endif

#endif // HELPER_IMPLEMENTATION
