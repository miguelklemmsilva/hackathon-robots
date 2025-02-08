#include <Arduino.h>
#include <PDM.h>
#include <Arduino.h>
#include "sensorFunctions.h"
#include <Wire.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// put function declarations here:
void playAlarm() {
    for (int i = 0; i < 10; i++) { // Repeat the alarm sound and light 10 times
        // Alarm sound
        tone(buzzer, 800); // First frequency
        setLEDbrightness(255); // Turn on the red light
        changeLedcolour(0, 255, 0, 0);
        changeLedcolour(1, 255, 0, 0);
        changeLedcolour(2, 255, 0, 0);
        delay(200); // Duration of the first frequency
        
        tone(buzzer, 600); // Second frequency
        delay(200); // Duration of the second frequency
        
        // Turn off the lights
        changeLedcolour(0, 0, 0, 0);
        changeLedcolour(1, 0, 0, 0);
        changeLedcolour(2, 0, 0, 0);
        delay(200); // Short pause
    }

    noTone(buzzer); // Stop the sound
}

void setup() {
  delay(3000);
  const int buzzer = 7; //buzzer to arduino pin 9
  Wire.begin();
  // Serial.begin(115200);
  Serial.print("Here is data: ");
  ledStripON();
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output
  initClimateSensor();
  
}

void loop() {
  delay(1000);
  float climate = getSingleClimateValue('h');
  // Serial.println(climate);
  if (climate>50.0)
  {
    playAlarm();
  }
  
}
