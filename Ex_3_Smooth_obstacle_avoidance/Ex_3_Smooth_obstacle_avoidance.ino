// Arduino Smart Car - Closed-Loop Control Demo
// This code makes the robot move more smoothly.
// If it sees an obstacle in front, it slows down, looks left and right using a servo,
// compares the distances, and turns toward the clearer (more open) side to continue driving.


#include <ServoTimer2.h>

// Pin definitions
#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10

#define TRIG_PIN 12
#define ECHO_PIN 13
#define SERVO_PIN 11

//Servo declaration
ServoTimer2 servo;

// Constants
#define TURN_RATIO 0.25
#define OBSTACLE_THRESHOLD 30  // Stop distance (cm)
#define SLOW_DOWN_DISTANCE 50  // Slow down threshold (cm)

// Speed declarations
int driveSpeed = 200;
int turnSpeed = 55;
int currentSpeed = 0;


void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  analogWrite(ENA, currentSpeed);
  analogWrite(ENB, currentSpeed);

  servo.attach(SERVO_PIN);
  servo.write(1500);

  Serial.println("Ready...");
  delay(1000);

  // Start moving forward from 0 to driveSpeed
  gradualSpeedChange(currentSpeed, driveSpeed, goForward);
  currentSpeed = driveSpeed;

  delay(100);
}


void loop() {

  // Exercise:
  // 1. Get front distance using ultrasonic sensor reading

  

  // 2. Check if an obstacle is close:

  // 2a. If distance is below SLOW_DOWN_DISTANCE but above OBSTACLE_THRESHOLD, Slow down.



  // 2b. If obstacle is very close (distance below OBSTACLE_THRESHOLD), Avoid obstacle



  // 2c. Else (no obstacle close), gradually move at driveSpeed



  
  delay(50);

}



/*
Try out this code to see if it works for the exercise:
// Just copy and paste it in the main loop before the delay function.

  int frontDistance = getDistance();

  if (frontDistance < SLOW_DOWN_DISTANCE && frontDistance > OBSTACLE_THRESHOLD) {
    // int newSpeed = max(0, currentSpeed - 10);
    // gradualSpeedChange(currentSpeed, newSpeed, goForward);
    // currentSpeed = newSpeed;
    // delay(100);
    currentSpeed = slowDown(currentSpeed);
  }

  else if (frontDistance < OBSTACLE_THRESHOLD) {
    obstacleAvoidance();
    currentSpeed = driveSpeed;
  }

  else {
    gradualSpeedChange(currentSpeed, driveSpeed, goForward);
    currentSpeed = driveSpeed;
  }

*/





// Get Distance
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  int distance = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
  return distance;
}


// Look Right
int lookRight() {
  servo.write(544);  //Turn 90 degrees to the right
  delay(500);
  int distance = getDistance();
  servo.write(1500);
  delay(100);
  return distance;
}

// Look Left
int lookLeft() {
  servo.write(2400); //Turn 90 degrees to the left
  delay(500);
  int distance = getDistance();
  servo.write(1500);
  delay(100);
  return distance;
}



void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  currentSpeed = 0;
}


void goForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void goBackward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft(int speed) {
  int innerSpeed = speed * TURN_RATIO;
  analogWrite(ENA, speed);
  analogWrite(ENB, innerSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight(int speed) {
  int innerSpeed = speed * TURN_RATIO;
  analogWrite(ENA, innerSpeed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


void gradualSpeedChange(int startSpeed, int targetSpeed, void (*speedFunc)(int)) {
  int s = startSpeed;
  int step = (targetSpeed > startSpeed) ? 5 : -5;

  while ((step > 0 && s < targetSpeed) || (step < 0 && s > targetSpeed)) {
    speedFunc(abs(s));
    s += step;
    delay(25);
  }
  speedFunc(abs(targetSpeed));  // Ensure we land on exact target
}


int slowDown(int currentSpeed) {
  int newSpeed = max(0, currentSpeed - 10);
  gradualSpeedChange(currentSpeed, newSpeed, goForward);
  delay(100);
  return newSpeed;
}


void obstacleAvoidance() {
  int distanceLeft = lookLeft();
  delay(200);
  int distanceRight = lookRight();
  delay(200);

  Serial.print("Left Distance: ");
  Serial.println(distanceLeft);
  Serial.print("Right Distance: ");
  Serial.println(distanceRight);

  if (distanceRight > distanceLeft) {
    Serial.println("Turning Right...");
    gradualSpeedChange(currentSpeed, turnSpeed, turnRight);
    currentSpeed = turnSpeed;
  } else {
    Serial.println("Turning Left...");
    gradualSpeedChange(currentSpeed, turnSpeed, turnLeft);
    currentSpeed = turnSpeed;
  }

  // Resume forward motion gradually
  gradualSpeedChange(currentSpeed, driveSpeed, goForward);
  currentSpeed = driveSpeed;
}


