/*
  BH1749NUC Interrupt -- New Data
  By: Jim Lindblom
  SparkFun Electronics
  Date: May 4, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14733
  This example demonstrates how to use the BH1749NUC's interrupt output to generate a falling edge every time
  data exceeds high or low thresholds.

  If you don't see data, try covering the sensor with your finger to get the red channel reading below the
  low threshold.
  
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

#define HIGH_THRESHOLD 250
#define LOW_THRESHOLD 50

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

  // Enable interrupt
  rgb.enableInterrupt(true);
  
  // Set interrupt source -- can be either BH1749NUC_RED, BH1749NUC_GREEN or, BH1749NUC_BLUE.
  // Also set interrupt persistence -- can be either:
  // BH1749NUC_INT_PERSISTENCE_1, BH1749NUC_INT_PERSISTENCE_4, or BH1749NUC_INT_PERSISTENCE_8
  // Triggers an interrupt on 1, 4, or 8 consecutive readings out of the set thresholds.
  rgb.setInterruptSource(BH1749NUC_RED, BH1749NUC_INT_PERSISTENCE_4);
  
  // Set high and low thresholds. If red value is >HIGH_THRESHOLD or <LOW_THRESHOLD, then an
  // interrupt will be triggered
  //rgb.setThresholdHigh(HIGH_THRESHOLD);
  //rgb.setThresholdLow(LOW_THRESHOLD);
  // Alternatively, setThresholds can be used to set high and low thresholds simultaneously
  rgb.setThresholds(LOW_THRESHOLD, HIGH_THRESHOLD);
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

