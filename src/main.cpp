#include "Arduino.h"

#include "Servo.h"
#include "IMU.h"

#define MAX_GIMBAL 20

#define Kp 0.5
#define Ki 0.5
#define Kd 0.5

#define TARGET_PITCH 0

enum RocketStates {STANDBY, FLIGHT, COAST, RECOVERY};
RocketStates rocketState = STANDBY;

Servo servoX;
Servo servoY;

IMU imu;

const int servo1Pin = 20;
const int servo2Pin = 21;

const int redPin = 2;
const int greenPin = 3;
const int bluePin = 4;

const int buzzerPin = 33;

float currentTime, previousTime, elapsedTime;
float integrator, differentiator, previousError, previousPitch;

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

void setServoAngle(int servo, float angle) {
  angle = angle * (180 / PI);
  int command = (int) round(angle);

  if (command > MAX_GIMBAL) {
    command = MAX_GIMBAL;
  } else if (command < -MAX_GIMBAL) {
    command = -MAX_GIMBAL;
  }

  command += 90;

  Serial.println(command);

  switch (servo) {
  case 0:
    servoX.write(command);
    break;
  
  case 1:
    servoY.write(command);
    break;

  default:
    break;
  }

}

void switchRocket() {
  switch (rocketState) {
  case STANDBY:
    
    // Detect liftoff
    // Update accelerometer values
    imu.updateAcc();

    // If positive accelerating in Z axis then thrust is being applied
    if (imu.getAcc(2) > 0) {
      rocketState = FLIGHT;
    }

    break;

  case FLIGHT:
    
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000; // Convert to seconds

    // Get data from IMU
    imu.update();

    // Third euler angle is roll of rocket
    float roll = imu.euler[2];

    // Calculate error signal
    float error = TARGET_PITCH - imu.euler[1];

    // Integral term
    integrator = integrator + (elapsedTime * error);

    // Derivative term
    differentiator = (error - previousError) / elapsedTime;

    // Calculate output
    float output = (Kp * error) + (Ki * integrator) + (Kd * differentiator);

    // Restrict to max gimbal angle
    if (output > MAX_GIMBAL) {
      output = MAX_GIMBAL;
    } else if (output < -MAX_GIMBAL) {
      output = -MAX_GIMBAL;
    }

    // Update previous error for next iteration
    previousError = error;

    // Split output angle into XY angles based on roll angle
    // Move roll into range 0 to Pi/2 for trig
    int quadrant = 1;
    if (roll < 0) {
      roll = roll * -1;
      quadrant = quadrant * -1;
    }

    if (roll > (PI/2)) {
      roll = PI - roll;
      quadrant = quadrant * 2;
    }
    
    float xAngle = output * sin(roll);
    float yAngle = output * cos(roll);

    // Adjust magnitude of angle depending on quadrant
    switch (quadrant) {
    case 1:
      yAngle = yAngle * -1;
      break;

    case 2:
      xAngle = xAngle * -1;
      break;
    
    case -1:
      xAngle = xAngle * -1;
      yAngle = yAngle * -1; 
      break;

    case -2:
      break;
    
    default:
      break;
    }

    // Update servo positions
    setServoAngle(0, xAngle);
    setServoAngle(1, yAngle);

    // Detect apogee
    float acceleration = sqrt(pow(imu.getAcc(0), 2) + pow(imu.getAcc(1), 2) + pow(imu.getAcc(3), 2));
    
    // If only acceleration is gravity
    if (acceleration > 0.95 && acceleration < 1.05) {
      rocketState = COAST;
    }

    break;

  case COAST:
    /* code */
    break;

  case RECOVERY:
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
  
  servoX.attach(servo1Pin);
  servoY.attach(servo2Pin);
  setServoAngle(0, 0);
  setServoAngle(1, 0);

  // Set RGB LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  setRGB(0, 0, 0);

  // Set Teensy LED pin as output
  pinMode(LED_BUILTIN,OUTPUT);

  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
  // Set PWM frequency for piezo buzzer according to datasheet
  analogWriteFrequency(buzzerPin, 4000);
}

void loop() {

  rocketState = FLIGHT;
  switchRocket();

  char outputBuf[100];
  sprintf(outputBuf, "A1: % 4.2f A2: % 4.2f A3: % 4.2f", imu.getEuler(0), imu.getEuler(1), imu.getEuler(2));

  Serial.println(outputBuf);
  delay(10);

}

