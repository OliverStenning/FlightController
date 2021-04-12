#include "Arduino.h"

#include "Servo.h"
#include "IMU.h"

enum RocketStates {CALIBRATION, STANDBY, FLIGHT, APOGEE, RECOVERY, LOGGING};
RocketStates rocketState = CALIBRATION;

Servo servoX;
Servo servoY;

IMU imu;

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

void enableBuzzer() {
  // 50% duty cycle when enabled
  analogWrite(buzzerPin, 127);
}

void disableBuzzer() {
  // 0% duty cycle when disabled
  analogWrite(buzzerPin, 0);
}

void setRGB(int r, int g, int b) {
  if (r > 0) {
    // Enable LED
    digitalWrite(redPin, LOW);
  } else {
    // Disable
    digitalWrite(redPin, HIGH);
  }

  if (g > 0) {
    // Enable
    digitalWrite(greenPin, LOW);
  } else {
    // Disable
    digitalWrite(greenPin, HIGH);
  }
  
  if (b > 0) {
    // Enable
    digitalWrite(bluePin, LOW);
  } else {
    // Disable
    digitalWrite(bluePin, HIGH);
  }
}

void switchMachine() {
  switch (rocketState) {
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

void setup() {

  // Setup serial for USB monitoring
  Serial.begin(115200);

  // Enable IMU by changing registers
  imu.enable();
  imu.enableLPF(20.0, 0.01);

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

void loop() {

  imu.update();

  char outputBuf[100];
  //sprintf(outputBuf, "X: % 6.3f  Y: % 6.3f  Z: % 6.3f  X: % 6.3f  Y: % 6.3f  Z: % 6.3f  T: % 6.3f", accX, accY, accZ, gyroX, gyroY, gyroZ, temp);
  //sprintf(outputBuf, "% 6.3f,% 6.3f,% 6.3f,% 6.3f,% 6.3f,% 6.3f", accX, accY, accZ, gyroX, gyroY, gyroZ);
  //sprintf(outputBuf, "% 4.2f,% 4.2f,% 4.2f", accX, accY, accZ);
  sprintf(outputBuf, "% 4.2f,% 4.2f", imu.acc[0], imu.filtAcc[0][0]);

  Serial.println(outputBuf);


  delay(100);

}

