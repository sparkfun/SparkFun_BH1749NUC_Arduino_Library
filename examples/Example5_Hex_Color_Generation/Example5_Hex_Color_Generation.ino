/*
  BH1749NUC Calibration -- Gather min/max RGB values then generate hex color code
  By: Jim Lindblom
  SparkFun Electronics
  Date: May 4, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14733
  This example demonstrates how "calibrate" the RGB sensor for min/max RGB values,
  then uses those scales to generate dynamic RGB hex values.
  
  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the Qwiic RGB Sensor onto the shield
  Watch each LED turn on one-at-a-time
*/
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

PCA9536 io;
BH1749NUC rgb;

#define NUM_SAMPLES 10
#define MAX_SAMPLES 65535

typedef enum {
  RED,
  GREEN,
  BLUE,
  NUM_COLORS
} rgb_colors;

const String colorNames[NUM_COLORS] = {
  "RED",
  "GREEN",
  "BLUE"
};

uint16_t minColorValues[NUM_COLORS] = {53, 128, 37};
uint16_t maxColorValues[NUM_COLORS] = {206, 408, 213};

//uint16_t maxColorValues[NUM_COLORS] = {0, 0, 0};

void setup() 
{
  Serial.begin(115200);
  
  if (rgb.begin(BH1749NUC_ADDRESS_CLOSED) != BH1749NUC_SUCCESS)
  {
    Serial.println("Error initializing the rgb sensor.");
    while (1) ;
  }
  // Set up RGB sensor:
  rgb.setIRGain(BH1749NUC_GAIN_X1);
  rgb.setRGBGain(BH1749NUC_GAIN_X1);
  rgb.setMeasurementMode(BH1749NUC_MEASUREMENT_MODE_240_MS);

  if ((maxColorValues[RED] == 0) || (maxColorValues[GREEN] == 0) || (maxColorValues[BLUE] == 0) ||
    (minColorValues[RED] == 65535) || (minColorValues[GREEN] == 65535) || (minColorValues[BLUE] == 65535))
  {
    calibrateAll();
    Serial.println("Calibration:");
    Serial.println("uint16_t minColorValues[NUM_COLORS] = {" + 
                   String(minColorValues[RED]) + ", " +
                   String(minColorValues[GREEN]) + ", " +
                   String(minColorValues[BLUE]) + "};");
    Serial.println("uint16_t maxColorValues[NUM_COLORS] = {" + 
                   String(maxColorValues[RED]) + ", " +
                   String(maxColorValues[GREEN]) + ", " +
                   String(maxColorValues[BLUE]) + "};\r\n");
  }
  Serial.println("Press any key to sample a color...\r\n");
}

void loop() 
{
  if (Serial.available())
  {
    uint16_t rawColor[NUM_COLORS] = {0, 0, 0};
    float calibratedColor[NUM_COLORS] = {0, 0, 0};
    uint16_t hexColor[NUM_COLORS] = {0, 0, 0};
    char rgbBuffer[8];
    
    Serial.read();
    
    while (!rgb.available()) ;
    for (int color = 0; color < NUM_COLORS; color++)
    {
      rawColor[color] = rgb.read((BH1749NUC_color_t) color);
      //calibratedColor[color] = map(rawColor[color], 0, 65535, minColorValues[color], maxColorValues[color]);//= (float)rawColor[color] / (float) maxColorValues[color];
      calibratedColor[color] = map(rawColor[color], minColorValues[color], maxColorValues[color], 0, 255);//= (float)rawColor[color] / (float) maxColorValues[color];        
      if (rawColor[color] > maxColorValues[color])
      {
        calibratedColor[color] = 255;
      }
      else if (rawColor[color] <= minColorValues[color])
      {
        calibratedColor[color] = 0;
      }
      else
      {
        calibratedColor[color] = map(rawColor[color], minColorValues[color], maxColorValues[color], 0, 255);//= (float)rawColor[color] / (float) maxColorValues[color];        
      }      
      
      hexColor[color] = calibratedColor[color];//(uint16_t)(calibratedColor[color] * 255.0);
      if (hexColor[color] > 255) hexColor[color] = 255;
      else if (hexColor[color] < 0) hexColor[color] = 0;
      if (rawColor[color] > maxColorValues[color])
      {
        Serial.println("Warning: " + colorNames[color] + " above MAX");
      }
      else if (rawColor[color] < minColorValues[color])
      {
        Serial.println("Warning: " + colorNames[color] + " below MIN");        
      }
    }
    
    Serial.println("Raw: " + String(rawColor[RED]) + ", "
                 + String(rawColor[GREEN]) + ", "
                 + String(rawColor[BLUE]));
    
    Serial.println("Calibrated: " + String(calibratedColor[RED]) + ", "
                 + String(calibratedColor[GREEN]) + ", "
                 + String(calibratedColor[BLUE]));

    sprintf(rgbBuffer, "#%02x%02x%02x", hexColor[RED], hexColor[GREEN], hexColor[BLUE]);
    Serial.println("RGB: " + String(rgbBuffer));
    Serial.println();
  }
}

void calibrateAll(void)
{
  for (uint8_t color = 0; color < NUM_COLORS; color++)
  {
    Serial.println("Hold sensor up to a pure " + 
            colorNames[color] + " source and press ANY KEY");
    while (!Serial.available()) ;
    Serial.read();
    Serial.println("Sampling " + colorNames[color] + "...");
    sampleColor(color, NUM_SAMPLES);
    Serial.println("Max " + colorNames[color] + ": " + 
            String(maxColorValues[color]) + "\r\n");
  }
}

void sampleColor(rgb_colors color, uint16_t samples)
{
  uint32_t total[NUM_COLORS] = {0, 0, 0};
  uint16_t numSamples = samples;
  int samplesTaken = 0;

  if (numSamples > MAX_SAMPLES) numSamples = MAX_SAMPLES;
  while (samplesTaken < numSamples)
  {
    if (rgb.available())
    {
      for (uint8_t i = 0; i < NUM_COLORS; i++)
      {
        total[i] += rgb.read((BH1749NUC_color_t) i); // Sample color;
      }
      samplesTaken += 1;
      // Print status . every 10% of readings
      if (samplesTaken % (numSamples / 10) == 0) {
        Serial.print('.');
      }
    }
  }
  Serial.println();
  
  // Iterate through all three colors after samples are taken to generate
  // min/max averages.
  for (uint8_t i = 0; i < NUM_COLORS; i++)
  {
    total[i] = total[i] / numSamples;
    if ((BH1749NUC_color_t)i == color)
    {
      maxColorValues[i] = total[i];
    }
    else
    {
      uint16_t tempMin = total[i];
      if (tempMin < minColorValues[i])
      {
        minColorValues[i] = tempMin;
      }
    }
  }
}
