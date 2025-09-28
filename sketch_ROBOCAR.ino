
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN  150   // Min pulse length out of 4096
#define SERVO_MAX  600   // Max pulse length out of 4096

// Joystick pins
#define JOY_X A0
#define JOY_Y A1
#define JOY_SW 2

// Servo channels
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define GRIPPER_SERVO 3

int baseAngle = 90;
int shoulderAngle = 90;
int elbowAngle = 90;
int gripperAngle = 90;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz update

  pinMode(JOY_SW, INPUT_PULLUP);
}

int angleToPulse(int ang) {
  return map(ang, 0, 180, SERVO_MIN, SERVO_MAX);
}

void loop() {
  int xVal = analogRead(JOY_X);
  int yVal = analogRead(JOY_Y);
  int swVal = digitalRead(JOY_SW);

  // Map joystick movement to servo angles
  if (xVal < 400) baseAngle -= 1;
  if (xVal > 600) baseAngle += 1;
  if (yVal < 400) shoulderAngle -= 1;
  if (yVal > 600) shoulderAngle += 1;

  // Use joystick button to control gripper open/close
  if (swVal == LOW) {
    gripperAngle = (gripperAngle == 90) ? 20 : 90; // toggle open/close
    delay(300); // debounce
  }

  // Limit angles to servo safe range
  baseAngle = constrain(baseAngle, 0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle = constrain(elbowAngle, 0, 180);
  gripperAngle = constrain(gripperAngle, 0, 180);

  // Send signals to PCA9685
  pwm.setPWM(BASE_SERVO, 0, angleToPulse(baseAngle));
  pwm.setPWM(SHOULDER_SERVO, 0, angleToPulse(shoulderAngle));
  pwm.setPWM(ELBOW_SERVO, 0, angleToPulse(elbowAngle));
  pwm.setPWM(GRIPPER_SERVO, 0, angleToPulse(gripperAngle));

  delay(20); // smooth movement
}