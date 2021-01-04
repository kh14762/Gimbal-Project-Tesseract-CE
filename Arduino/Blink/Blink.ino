/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <I2Cdev.h>
#include <SD.h>
#include <SPIFlash.h>
#include <Servo.h>
#include <MPU6050_tockn.h>

//set up LEDs and buzzer hello ...Kev7n...Kev?n...ㅏㄸ퍄ㅜ 
//define system led pins
int R_LED = 9;
int G_LED = 2;
int B_LED = 6;
int Buzzer = 10;

//SD card setup
Sd2Card card;
SdVolume volume;
SdFile root;
const int SDchipSelect = 0;

//This for the SPI Flash chip
#define CHIPSIZE MB64
SPIFlash flash(1);
uint8_t pageBuffer[256];
char printBuffer[128];
uint16_t page;
uint8_t offset, dataByte;
uint16_t dataInt;
String inputString, outString;

//initilize pyro channel values
int Pyro1 = 20;
int Pyro2 = 21;
int Pyro3 = 22;
int Pyro4 = 23;

//initialize TVC servos
int TVCXpin = 3;
int TVCYpin = 4;
Servo TVCXservp;
Servo TVCYservp;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

//create and initialized MPU object
MPU6050 mpu6050(Wire);

float PID, pvmLeft, pvmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
//PID constants
double kp = 3.44;
double ki = 0.048;
double kd =1.92; //2.6

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int led = 13;
void turnOnTurnOffL();
void init_BMPsensor();
void init_LEDlights();
void printMPUAngles();
void printBMPSensorDetails();
void i2c_Scanner();
void PIDfunc();


#define LED_PIN 11
bool blinkState = false;

// the setup routine runs once when you press reset:
void setup() {
  //Initialize & begin Serial
  Serial.begin(9600);
  //Initialize & begin Wire
  Wire.begin();
  //begin and set calcGyroOffsets to true
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  //setup BMP sensor
  init_BMPsensor();
  //setup LED Lights
  init_LEDlights();
  
}




// the loop routine runs over and over again forever:
void loop() {


  turnOnTurnOffL();
  printBMPSensorDetails();
  printMPUAngles();



}

void i2c_Scanner() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
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

void init_LEDlights() {
  //set sytem led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

//using common anode LED, so write LOW to turn it on and High to turn if off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);
  
}

void init_BMPsensor() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
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
}

void printMPUAngles() {
  mpu6050.update();
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
}
