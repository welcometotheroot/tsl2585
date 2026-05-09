/**
 * basic_reading.ino — Minimal TSL2585 usage example
 *
 * Reads Photopic, UV, and IR irradiance from the TSL2585 using interrupt-driven
 * operation and prints the results to Serial once per ALS cycle (~28ms).
 *
 * Wiring (adjust pin numbers for your board):
 *   SDA  → board SDA
 *   SCL  → board SCL
 *   INT  → INT_PIN (active-low, requires pull-up — check board datasheet)
 *   VDD  → 3.3V
 *   GND  → GND
 *
 * Expected Serial output (115200 baud):
 *   Photopic: 0.0123 mW/cm²  (gain 128x)
 *   UV:       0.0045 mW/cm²  (gain 128x)
 *   IR:       0.0089 mW/cm²  (gain 128x)
 */

#include <Wire.h>
#include <tsl2585.h>

static constexpr uint8_t INT_PIN = 2;  // GPIO connected to TSL2585 INT (active-low)

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  if (!TSL2585::begin(Wire, INT_PIN)) {
    Serial.println("TSL2585 not found — check wiring.");
    while (true) {}
  }

  Serial.println("TSL2585 ready.");
}

void loop() {
  if (!TSL2585::isDataReady()) return;

  TSL2585Data d;
  if (!TSL2585::read(d)) {
    Serial.println("Read failed.");
    return;
  }

  Serial.print("Photopic: ");
  Serial.print(d.photopicIrradiance, 4);
  Serial.print(" mW/cm²  (gain ");
  Serial.print(d.photopicGain == 0 ? "0.5" : String(d.photopicGain).c_str());
  Serial.println("x)");

  Serial.print("UV:       ");
  Serial.print(d.uvIrradiance, 4);
  Serial.print(" mW/cm²  (gain ");
  Serial.print(d.uvGain == 0 ? "0.5" : String(d.uvGain).c_str());
  Serial.println("x)");

  Serial.print("IR:       ");
  Serial.print(d.irIrradiance, 4);
  Serial.print(" mW/cm²  (gain ");
  Serial.print(d.irGain == 0 ? "0.5" : String(d.irGain).c_str());
  Serial.println("x)");

  Serial.println();
}
