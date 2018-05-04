/*
  Using the BH1749NUC -- RGB/IR Monitoring
  By: Jim Lindblom
  SparkFun Electronics
  Date: May 4, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/TODO
  This example demonstrates how to initialize and configure the BH1749NUC color sensor.
  Then check for new data and print the RGB and IR color data to the Serial Monitor at 115200
  
  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the Qwiic RGB Sensor onto the shield
  Watch each LED turn on one-at-a-time
*/
#include <SparkFun_BH1749NUC_Arduino_Library.h>

BH1749NUC rgb;

void setup() {
  Serial.begin(115200);

  if (rgb.begin() != BH1749NUC_SUCCESS)
  {
    Serial.println("Error initializing the rgb sensor.");
    while (1) ;
  }
  
  // IR and RGB gain can be set to either BH1749NUC_GAIN_X1 or
  // BH1749NUC_GAIN_X32
  rgb.setIRGain(BH1749NUC_GAIN_X1);
  rgb.setRGBGain(BH1749NUC_GAIN_X1);
  // Measurement mode can be set to either BH1749NUC_MEASUREMENT_MODE_35_MS,
  // BH1749NUC_MEASUREMENT_MODE_120_MS, or BH1749NUC_MEASUREMENT_MODE_240_MS
  // (35ms, 120ms, 240ms between measurements).
  rgb.setMeasurementMode(BH1749NUC_MEASUREMENT_MODE_240_MS);
}

void loop() {
  if (rgb.ready())
  {
    if (rgb.update())
    {
      Serial.println("Red: " + String(rgb.colors.red));
      Serial.println("Green: " + String(rgb.colors.green));
      Serial.println("Blue: " + String(rgb.colors.blue));
      Serial.println("IR: " + String(rgb.colors.ir));
      Serial.println("Green2: " + String(rgb.colors.green2));
      Serial.println();
    }
  }
}
