/*
  Using the BH1749NUC -- RGB/IR I2C Polling
  By: Jim Lindblom
  SparkFun Electronics
  Date: May 4, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14733
  This example demonstrates how to more efficiently poll the sensor for new data,
  then read and print all data when updates become available. 
  
  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the Qwiic RGB Sensor onto the shield
  Watch each LED turn on one-at-a-time
*/
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

PCA9536 io;
BH1749NUC rgb;

#define WHITE_LED 0
#define RED_LED   1
#define GREEN_LED 2
#define BLUE_LED  3

void setup() {
  Serial.begin(115200);

  if (io.begin() != PCA9536_SUCCESS)
  {
    Serial.println("Error: initializing the IO expander");
    while (1);
  }
  
  // IO Expander set up:
  for (int i = 0; i < 4; i++)
  {
    io.pinMode(i, OUTPUT); // Set all LED pins to output
    io.write(i, HIGH);   // Turn all LED's off
  }
  
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
  if (rgb.available())
  {
    Serial.println("" 
                   + String(rgb.colors.red) + "," 
                   + String(rgb.colors.green) + "," 
                   + String(rgb.colors.blue) + ","
                   + String(rgb.colors.ir));
  }
}

void serialEvent() {
  while (Serial.available() < 2) ;
  char c = Serial.read();
  char v = Serial.read();
  uint8_t pin = 0xFF;
  uint8_t value = 0xFF;
  switch (c)
  {
  case 'w':
    pin = WHITE_LED;
    break;
  case 'r':
    pin = RED_LED;
    break;
  case 'g':
    pin = GREEN_LED;
    break;
  case 'b':
    pin = BLUE_LED;
    break;
  }
  
  if (v == '1') value = LOW;
  else if (v == '0') value = HIGH;

  if ((pin != 0xFF) && (value != 0xFF)) {
    io.write(pin, value);
  }
}

