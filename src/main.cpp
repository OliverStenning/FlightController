#include "Arduino.h"

#include "Servo.h"
#include "Wire.h"


#define AD0_VAL 1

#define IMU_ADDR 0x69

enum RocketStates {CALIBRATION, STANDBY, FLIGHT, APOGEE, RECOVERY, LOGGING};
RocketStates rocketState = CALIBRATION;

Servo servoX;
Servo servoY;

const int servo1Pin = 20;
const int servo2Pin = 21;

const int redPin = 2;
const int greenPin = 3;
const int bluePin = 4;

const int buzzerPin = 33;

/* States

- Calibration
- Ready for Launch
- Powered flight (control system active)
- Apogee
- Recovery system
- Data logging
*/

void enableBuzzer()
{
  // 50% duty cycle when enabled
  analogWrite(buzzerPin, 127);
}

void disableBuzzer()
{
  // 0% duty cycle when disabled
  analogWrite(buzzerPin, 0);
}

void setRGB(int r, int g, int b)
{
  if (r > 0) 
  {
    // Enable LED
    digitalWrite(redPin, LOW);
  } else {
    // Disable
    digitalWrite(redPin, HIGH);
  }

  if (g > 0)
  {
    // Enable
    digitalWrite(greenPin, LOW);
  } else {
    // Disable
    digitalWrite(greenPin, HIGH);
  }
  
  if (b > 0)
  {
    // Enable
    digitalWrite(bluePin, LOW);
  } else {
    // Disable
    digitalWrite(bluePin, HIGH);
  }
  
  
}

void switchMachine()
{
  switch (rocketState)
  {
  case CALIBRATION:
    /* code */
    break;

  case STANDBY:
    /* code */
    break;

  case FLIGHT:
    /* code */
    break;

  case APOGEE:
    /* code */
    break;

  case RECOVERY:
    /* code */
    break;

  case LOGGING:
    /* code */
    break;

  default:
    break;
  }
}

void readID()
{
  int id = 0;
  Serial.print("Reading device ID: ");
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x75);
  Wire1.endTransmission();
  Wire1.requestFrom(IMU_ADDR, 1);

  if (Wire1.available() <= 2) 
  {
    id = Wire1.read();
  }

  Serial.println(id);

}

void readAccX()
{

  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x3B);
  Wire1.endTransmission(false);
  Wire1.requestFrom(IMU_ADDR, 6, true);
  
  int16_t accX = ((int16_t) Wire1.read() << 8 | Wire1.read());
  int16_t accY = ((int16_t) Wire1.read() << 8 | Wire1.read());
  int16_t accZ = ((int16_t) Wire1.read() << 8 | Wire1.read());

  float scaledAccX = ((float) accX) / 16384.0;
  float scaledAccY = ((float) accY) / 16384.0;
  float scaledAccZ = ((float) accZ) / 16384.0;

  Serial.print("X: ");
  Serial.print(scaledAccX);
  Serial.print("  Y: ");
  Serial.print(scaledAccY);
  Serial.print("  Z: ");
  Serial.println(scaledAccZ);

}

void setup()
{

  // Setup IMU
  Serial.begin(115200);
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  Wire1.begin();
  Wire1.setClock(400000);

  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x6B);
  Wire1.write(0);
  Wire1.endTransmission(true);

  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x1A);
  Wire1.write(0);
  Wire1.endTransmission(true);

  // Attach gimbal servos to correct pins
  //servoX.attach(servo1Pin);
  //servoY.attach(servo2Pin);

  // Set RGB LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Set Teensy LED pin as output
  pinMode(LED_BUILTIN,OUTPUT);

  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
  // Set PWM frequency for piezo buzzer according to datasheet
  analogWriteFrequency(buzzerPin, 4000);
}

void loop()
{

  //readID();
  readAccX();
  delay(100);

}

