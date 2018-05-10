/*
  Qwiic RGB Sensor Test Sketch
  By: Jim Lindblom
  SparkFun Electronics
  Date: May 4, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14733
  This sketch tests the I2C pins of the BH1749NUC RGB sensor.
  It blinks all four LEDs, then reads the RGB sensor and continues blinking the LEDs
  
  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the Qwiic RGB Sensor onto the shield
  Watch each LED turn on one-at-a-time
*/
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

PCA9536 io;
BH1749NUC rgb;

/*#define WHITE_LED 0
#define RED_LED   1
#define GREEN_LED 2
#define BLUE_LED  3*/

enum led_pins {
  WHITE_LED,
  RED_LED,
  GREEN_LED,
  BLUE_LED,
  NUM_LEDS
};

#define LED_INTERVAL 200
#define LOOP_LED BLUE_LED

void setup() {
  Serial.begin(115200);

  if (io.begin() != PCA9536_SUCCESS) {
    Serial.println("Error: initializing the IO expander");
    while (1);
  }

  // Initialize all LEDs as output and off (HIGH):
  for (int i = 0; i < NUM_LEDS; i++) {
    io.pinMode(i, OUTPUT);
    io.write(i, HIGH);
  }
  
  // Blink all four LEDs connected to I/O expander:
  for (int i = NUM_LEDS; i >= 0; i--) {
    io.write(i, LOW);   // Turn LED on
    delay(LED_INTERVAL);
    io.write(i, HIGH);   // Turn LED off
  }
  
  if (rgb.begin() != BH1749NUC_SUCCESS) {
    Serial.println("Error initializing the rgb sensor.");
    while (1) ;
  }
}

void loop() {
  static uint8_t ledValue = LOW;
  static uint8_t activeLED = 0;
  
  if (rgb.available()) {
    io.write(activeLED, ledValue++);
    if (ledValue > HIGH) {
      ledValue = LOW;
      activeLED += 1;
      if (activeLED >= NUM_LEDS) {
        activeLED = 0;
      }
    }
    Serial.println(String(rgb.red()) + ", " +
      String(rgb.blue()) + ", " + String(rgb.green()));
  }
}