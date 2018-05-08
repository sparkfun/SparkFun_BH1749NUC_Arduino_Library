/*
  BH1749NUC Interrupt -- New Data
  By: Jim Lindblom
  SparkFun Electronics
  Date: May 4, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/TODO
  This example demonstrates how to use the BH1749NUC's interrupt output to generate a falling edge every time
  new data becomes available.
  
  Hardware Connections:
  - Assuming a SparkX BlackBoard or similar Arduino Uno variant
  (Connect Qwiic RGB Sensor to Arduino via Qwiic header
  -or- 
  Match SDA, SCL, 3.3V, and GND pins on RGB breakout to Arduino)
  -and-
  Connect INT pin on RGB breakout to Arduino pin 2
*/
#include <SparkFun_BH1749NUC_Arduino_Library.h>

BH1749NUC rgb;

#define INTERRUPT_PIN 2

static boolean intActive = false;
void setup() 
{
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rgbInterrupt, FALLING);
  
  if (rgb.begin() != BH1749NUC_SUCCESS)
  {
    Serial.println("Error: initializing the rgb sensor.");
    while (1) ;
  }
  // Configure the color sensor:
  rgb.setIRGain(BH1749NUC_GAIN_X1); 
  rgb.setRGBGain(BH1749NUC_GAIN_X1);
  rgb.setMeasurementMode(BH1749NUC_MEASUREMENT_MODE_240_MS);

  // Enable interrupt -- set it for new data
  rgb.enableInterrupt(true);
  rgb.setInterruptSource(BH1749NUC_INT_SOURCE_NEW);
}

void loop() 
{
  if (intActive)
  {
    Serial.println(String(rgb.red()) + ", " + 
                   String(rgb.green()) + ", " + 
                   String(rgb.blue()) + ", " + 
                   String(rgb.ir()));
    intActive = false;
  }
}

void rgbInterrupt()
{
  intActive = true;
}

