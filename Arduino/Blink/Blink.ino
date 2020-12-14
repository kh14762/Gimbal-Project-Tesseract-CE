/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "MPU6050.h"
#include <Thread.h>
#include <ThreadController.h>
#include <I2Cdev.h>
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int led = 13;
String myName;
String videoTitle;
void turnOnTurnOffL();
void init_BMPsensor();
void printBMPSensorDetails();
void init_IMUSensor();
void i2c_Scanner();

#define LED_PIN 11
bool blinkState = false;

// the setup routine runs once when you press reset:
void setup() {
 init_BMPsensor();
 init_MPUsensor();
// accelgyro.initialize();
//  Serial.println(accelgyro.testConnection() ? "Found it! MPU6050 connection successful." : "MPU6050 connection failed :(");
  }

  


// the loop routine runs over and over again forever:
void loop() {
  
   
  //turnOnTurnOffL();
  printBMPSensorDetails();
  //i2c_Scanner();
  

  
}

void i2c_Scanner() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan

}

void turnOnTurnOffL() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  
}

void init_BMPsensor() {
   // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  //Initialize & begin Serial
  Serial.begin(9600);
  Serial.println(F("BMP280 Sensor event test"));
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);

      /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  }
}

void initBMPsensor() {
  
}

void printBMPSensorDetails() {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");
  Serial.println(" *F");

  Serial.print(F("Pressure = "));
  Serial.print((pressure_event.pressure - 1018) * 100);
  Serial.println(" hPa");

  Serial.println();

  delay(30);
}
