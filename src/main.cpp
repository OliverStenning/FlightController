#include "Arduino.h"

#include "Servo.h"

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

void setup()
{

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

  switchMachine();

}

