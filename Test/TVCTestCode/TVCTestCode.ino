
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 11
bool blinkState = true;

Servo servoX;
Servo servoY;
int servoXpin = 3;
int servoYpin = 4;
int servoXPos = 0;
int servoYPos = 0;

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;

MPU6050 mpu;

//MPU control/status variables
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//  orientation/mation variables
Quaternion q;              //[w, x, y, z]
VectorInt16 aa;             //[x, y, z]
VectorInt16 aaReal;         //[x, y, z]
VectorInt16 aaWorld;        //[x, y, z]
VectorFloat gravity;        //[x, y, z]
float ypr[3];               //[yaw, pitch, roll] yaw/pitch/roll container and gravity vector

#define PITCH  1
#define ROLL   2
#define YAW    0

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{

servoX.attach(servoXpin);
servoY.attach(servoYpin);
delay(50);
servoX.write(0);
servoY.write(60);
delay(500);
servoX.write(180);
servoY.write(120);
delay(500);
servoX.write(0);
servoY.write(90);
delay(500);

//  join I2C bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

Serial.begin(115200);

//  initialize device
Serial.println(F("Initializing I2C devices..."));
mpu.initialize();

//  verify connection
Serial.println(F("Testing device connections..."));
Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// load and configure the DMP
Serial.println(F("Initializing DMP"));
devStatus = mpu.dmpInitialize();

//Input Calibrated Offsets Here: Specific for each unit and each mounting configuration
mpu.setXGyroOffset(118);
mpu.setYGyroOffset(-44);
mpu.setZGyroOffset(337);
mpu.setXAccelOffset(-651);
mpu.setYAccelOffset(670);
mpu.setZAccelOffset(1895);

//  make sure it worked (returned 0 if it did)
if (devStatus == 0) {
  //turn on dmp
  Serial.println(F("Enabling DMP"));
  mpu.setDMPEnabled(true);

  //enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
  mpuIntStatus = mpu.getIntStatus();

  //  get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
} else {
  //ERROR
  //1 = initial memory load failed, 2 = DMP coniguration updates failed
  Serial.print(F("DMP initialization failed code = "));
  Serial.println(devStatus);
}
  //configure LED for output
  pinMode(LED_PIN, OUTPUT);
} //setup()

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
processAccelGyro();
}   //loop()

// ================================================================
// ===                    PROCESS ACCEL/GYRO IF AVAILABLE       ===
// ================================================================

void processAccelGyro()
{

  //Get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  //get current FIFO count
  fifoCount = mpu.getFIFOCount();

  //  Check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    //reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02) //otherwise continue processing
  {
    //check for correct available data length
    if (fifoCount < packetSize)
      return; // fifoCount = mpu.getFIFOCount();

    //read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    //track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    //flush buffer to prevent overflow
    mpu.resetFIFO();

    //  display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    mpuYaw = ypr[YAW] * 180 / M_PI;
  
    //flush buffer to prevent overflow
    mpu.resetFIFO();

    //bink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    //flush buffer to prevent overflow
    mpu.resetFIFO();

    servoX.write(-mpuPitch + 90);
    servoY.write(mpuRoll + 90);

    //flush buffer to prevent overflow
    mpu.resetFIFO();
  }
}
