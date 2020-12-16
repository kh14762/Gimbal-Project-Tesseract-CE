/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Thread.h>
#include <ThreadController.h>
#include <I2Cdev.h>
#include <AutoPID.h>
#include <SD.h>
#include <SPIFlash.h>
#include <Servo.h>
#include "MPU6050.h"

//set up LEDs and buzzer
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

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

//Cal

//Create a new AutoPID object
AutoPID myPID(double *input, double *setpoint, double *output,
              double outputMin, double outputMax,
              double Kp, double Ki, double Kd);

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
void init_LEDlights();
void init_IMUSensor();
void printBMPSensorDetails();
void i2c_Scanner();
void PIDfunc();


#define LED_PIN 11
bool blinkState = false;

// the setup routine runs once when you press reset:
void setup() {
  //setup BMP sensor
  init_BMPsensor();
  //setup LED Lights
  init_LEDlights();
  
}




// the loop routine runs over and over again forever:
void loop() {


  //turnOnTurnOffL();
  printBMPSensorDetails();
  printMPUSensorReadout();
  //i2c_Scanner();



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
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  /*
    Serial.println("Updating internal sensor offsets...");
    // -76 -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
  */

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
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

void printMPUSensorReadout() {
  //read raw accel/gyro measurements from this device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //other available methods
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  //display tab-sepaerated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
#endif

  //blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(100);
}



void PIDfunc() {
  /*
     previous_error := 0
    integral := 0

    loop:
      error := setpoint − measured_value
      integral := integral + error × dt
      derivative := (error − previous_error) / dt
      output := Kp × error + Ki × integral + Kd × derivative
      previous_error := error
      wait(dt)
      goto loop
  */
}
