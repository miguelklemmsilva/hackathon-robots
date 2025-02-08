#ifndef sensorFunctions_H
#define sensorFunctions_H

#include <Arduino.h>
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#define baudRate 115200

void initSerial()
{
    Serial.begin(baudRate);
    Serial.read();
}

#define fanControl 1
#if fanControl

const int fanPin = 26;
const int incStep = 30;

void initFan()
{
    pinMode(fanPin, OUTPUT);
    analogWriteFreq(100);
    analogWriteRange(255);
    analogWriteResolution(8);
}

void deinitFan()
{
    analogWrite(fanPin, 0);
    pinMode(fanPin, INPUT);
}

void runFan(int speed)
{
    analogWrite(fanPin, speed);
}
void runFan(int speed, unsigned long durationMS)
{
    analogWrite(fanPin, speed);  // Set fan speed
    delay(durationMS);              // Wait for the specified duration
    analogWrite(fanPin, 0);       // Turn off the fan after the duration
}

#endif

#define fuelGauge 1
#if fuelGauge
#include "Adafruit_MAX1704X.h"
Adafruit_MAX17048 maxlipo;

void gaugeInit()
{
    if (!maxlipo.begin())
    {
        Serial.println(F("Couldnt find Adafruit MAX17048!"));
        delay(2000);
    }
}

float getFuelgaugeMeasurement()
{
    float cellVoltage = maxlipo.cellVoltage(); // reads cell voltage
    if (isnan(cellVoltage))
    {
        Serial.println("Failed to read cell voltage, check battery is connected!");
        delay(1000);
        return -1;
    }
    return cellVoltage;
}

float getFuelpercentage()
{
    
    float percent = (maxlipo.cellVoltage()-2.28)/(4.8-2.28)*100;
    return percent;
}


#endif

#define Led_Screen 1
#if Led_Screen
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin #
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// create OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// the library initializes this with an Adafruit splash screen.

void displayInit()
{
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.display();
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
}
void displayScreenFormat(int size, int column, int row)
{
    display.setTextSize(size);
    display.setCursor(column, row);
}

// To print on the LED display, do as you would usually but prefacing it with "display."

#endif
#define led_Functions 1
#if led_Functions
#define PIN_WS2812B 6 // The ESP8266 pin that connects to WS2812B
#define NUM_PIXELS 3  // The number of LEDs (pixels) on WS2812B
#define setLedBrightness
// Instantiate LEDs class
Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

// change Lumen levels between 1-255
void setLEDbrightness(int Brightness)
{
    WS2812B.setBrightness(Brightness);
}

void ledStripON()
{
    WS2812B.begin(); // initializes WS2812B strip object (REQUIRED)
}
void changeLedcolour(int ledNum, int red, int green, int blue, int delayTime)
{
    WS2812B.setPixelColor(ledNum, WS2812B.Color(red, green, blue)); // it only takes effect if pixels.show() is called
    WS2812B.show();                                                 // send the updated pixel colors to the WS2812B hardware.
    delay(delayTime);                                               // pause between each pixel
}
void changeLedcolour(int ledNum, int red, int green, int blue)
{
    WS2812B.setPixelColor(ledNum, WS2812B.Color(red, green, blue));
    WS2812B.show();
}
void ledOff()
{
    WS2812B.clear();
    WS2812B.show();
}
bool LEDstateCheck (){
    for (uint16_t i = 0; i < WS2812B.numPixels(); i++) {
        if (WS2812B.getPixelColor(i) != 0) {  // Check if the pixel color is not zero
            return true;
        }
    }
    return false;
}
void blinkID(int deviceID)
{
    switch (deviceID)
    {
    case 0:
        ledOff();
        setLEDbrightness(10);
        changeLedcolour(0, 255, 0, 0, 2500);
        ledOff();
        delay(2500);
        setLEDbrightness(10);
        changeLedcolour(0, 255, 0, 0, 2500);
        ledOff();
        delay(2500);

        break;

    case 1:
        ledOff();
        setLEDbrightness(10);
        changeLedcolour(1, 0, 255, 0, 2500);
        ledOff();
        delay(2500);
        setLEDbrightness(10);
        changeLedcolour(1, 0, 255, 0, 2500);
        ledOff();
        delay(2500);
        break;

    case 2:
        ledOff();
        setLEDbrightness(10);
        changeLedcolour(0, 0, 0, 255);
        setLEDbrightness(10);
        changeLedcolour(1, 0, 0, 255, 2500);
        ledOff();
        delay(2500);
        setLEDbrightness(10);
        changeLedcolour(0, 0, 0, 255);
        setLEDbrightness(10);
        changeLedcolour(1, 0, 0, 255, 2500);

        ledOff();
        break;

    default:
        break;
    }
}

#endif

#define lightSensor 1
#if lightSensor
#include "BH1745NUC.h" //light measurement sensor library

#include <Adafruit_GFX.h>     //OLED display support library
#include <Adafruit_SSD1306.h> //OLED display library

#define BH1745NUC_DEVICE_ADDRESS_38 0x38 // light measurement sensor I2C address
BH1745NUC bh1745nuc = BH1745NUC();

void initLightsensors()
{
    // start light level measurements
    bh1745nuc.begin(0x38);
    bh1745nuc.startMeasurement();
    PinMode OUTPUT;
}

unsigned short getLightValue(char initialOfColor)
{
    if (!bh1745nuc.read()) // attempts to read light measurement values
    {
        Serial.println("Light sensor failed to read measurement values!");
        delay(500);
        return -1;
    }
    switch (initialOfColor)
    {
    case 'r':
        return bh1745nuc.red; // Red LED
    case 'g':
        return bh1745nuc.blue; // Green LED
    case 'b':
        return bh1745nuc.green; // Blue LED
    case 'c':
        return bh1745nuc.clear; // Custom LED
    default:

        return -1; // Error code
    }
}

void getLightvalueBuffer(short unsigned *rgbc)
{
    short unsigned values[4];
    values[0] = bh1745nuc.red;
    values[1] = bh1745nuc.green;
    values[2] = bh1745nuc.blue;
    values[3] = bh1745nuc.clear;

    memcpy(rgbc, values, sizeof(values));
}

unsigned short getLumen()
{
    if (!bh1745nuc.read()) // attempts to read light measurement values
    {
        Serial.println("Light sensor failed to read measurement values!");
        delay(500);
        return -1;
    }

    unsigned short lumen = (bh1745nuc.red * .2126 + bh1745nuc.green * .7152 + bh1745nuc.blue);

    return lumen;
}

#endif

#define climateSensor 1
#if climateSensor
#include "bme68xLibrary.h"
// initialize Climate sensor object
#include "bme68xLibrary.h" 

Bme68x bme;


void initClimateSensor()
{
    // Initialize the BME68x sensor on the I2C bus
    bme.begin(0x76, Wire);

    // Check sensor status
    if (bme.checkStatus())
    {
        if (bme.checkStatus() == BME68X_ERROR)
        {
            Serial.println("Sensor error:" + bme.statusString());
            return;
        }
        else if (bme.checkStatus() == BME68X_WARNING)
        {
            Serial.println("Sensor Warning:" + bme.statusString());
        }
    }

    // Set the default configuration for temperature measurement
    bme.setTPH();

    // Optional: Configure sensor heating (not necessary for temperature only)
    uint16_t tempProf[10] = {100, 200, 320};
    uint16_t durProf[10] = {150, 150, 150};
    bme.setSeqSleep(BME68X_ODR_250_MS);
    bme.setHeaterProf(tempProf, durProf, 3);
    bme.setOpMode(BME68X_SEQUENTIAL_MODE);
}

/**
 * @param initialOfclimateData : "t"emperature","h"umidity,"p"ressure
 */
float getSingleClimateValue(char initialOfclimateData)
{
    bme68xData data;
    uint8_t nFieldsLeft = 0;

    // Fetch data from the BME68x sensor
    while (!bme.fetchData())
    {}
        do
        {
            nFieldsLeft = bme.getData(data);
            switch (initialOfclimateData)
            {
            case 't':
                return data.temperature; // temperature data
            case 'h':
                return data.humidity; // humidity data
            case 'p':
                return data.pressure; // pressure data
            default:

                return -1; // Error code
            }

            // Return the temperature (adjusted if necessary)
            return data.temperature - 4.49; // Adjust as needed
        } while (nFieldsLeft);
    

    // If no data available, return an invalid value (or handle as needed)
    return -999.0;
}

void getClimateValueBuffer(float *thp)
{
    bme68xData data;
    uint8_t nFieldsLeft = 0;

    while (!bme.fetchData())
    {}
        printf("failed to read climate sensor");
    

        nFieldsLeft = bme.getData(data);
        float values[3];
        values[0] = data.temperature;
        values[1] = data.humidity;
        values[2] = data.pressure;
        memcpy(thp, values, sizeof(values));
    
}
#endif

#define noiseSensor 1
#if noiseSensor

#include <PDM.h>
// Variables for PDM microphone
// Variables for PDM microphone
static const char channels = 1;     // Mono mode
static const int frequency = 16000; // 16 kHz sample rate
short sampleBuffer[512];            // Buffer to store PDM samples
volatile int samplesRead = 0;       // Number of samples read
// Initialize the PDM microphone

// Callback function for the PDM microphone data
void onPDMdata()
{
    int bytesAvailable = PDM.available();
    PDM.read(sampleBuffer, bytesAvailable);
    samplesRead = bytesAvailable / 2; // Each sample is 2 bytes
}
// Initialize the PDM microphone
void initNoiseSensor()
{
    PDM.onReceive(onPDMdata); // Set callback for PDM data

    // Begin PDM microphone with specified channels and frequency
    if (!PDM.begin(channels, frequency))
    {
        Serial.println("Failed to start PDM!");
        return; 
             // Loop forever if PDM fails to start
    }
}

// Function to read and calculate noise level

int readNoiseLevel()
{
    if (samplesRead > 0)
    {
        long long sumSquares = 0;

        // Calculate the sum of squares of the samples
        for (int i = 0; i < samplesRead; i++)
        {
            sumSquares += sampleBuffer[i] * sampleBuffer[i];
        }

        // Calculate RMS (Root Mean Square)
        float rms = sqrt(sumSquares / samplesRead);

        // Convert RMS to decibels (dB)
        if (rms <= 0)
            return -100;                      // Return a very low value if no sound detected
        float noiseLevelDB = 20 * log10(rms); // Convert RMS to dB

        samplesRead = 0;          // Reset sample count
        return (int)noiseLevelDB; // Return the noise level in dB as an integer
    }
    return -100; // Return a very low value if no samples available
}

#endif

#define motionDetector 1
#if motionDetector

#define detectionLED LED_BUILTIN // the pin that the LED is atteched to

int buzzer = 7;     // buzzer to arduino pin 9
int sensorPIN = 11; // the pin that the sensor is atteched to
int pirState = LOW; // keeps track of PIR state, LOW (0) when no motion is detected and HIGH (1) when
                    // motion is detected. Initialised to no motion detected
int value = 0;      // variable to store the sensor value

int getMotion(int _pirState)
{
    pirState = _pirState;
    value = digitalRead(sensorPIN); // read sensor value
    // check if the sensorPIN is HIGH
    if (value == HIGH)
    {
        digitalWrite(detectionLED, HIGH); // turn LED ON
        if (pirState == LOW)
        {
            Serial.println("Motion detected!");
            delay(1000); // ...for 1sec
        }
        return 1;
    }
    else
    {
        digitalWrite(detectionLED, LOW); // turn LED OFF
        if (pirState == HIGH)
        {
            Serial.println("Motion stopped!");
        }
        return 0;
    }
}
#endif
#endif
