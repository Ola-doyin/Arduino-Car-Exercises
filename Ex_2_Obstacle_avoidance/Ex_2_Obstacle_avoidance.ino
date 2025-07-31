// Arduino Smart Car - Closed-Loop Control Demo
// This code makes the robot move forward at a constant speed.
// If it sees an obstacle in front, it stops, looks left and right using a servo,
// compares the distances, and turns toward the clearer (more open) side to continue driving.


#include <ServoTimer2.h>

#define ENA 5   // Enable pin for Motor A
#define IN1 6   // Control pin 1 for Motor A (Right)
#define IN2 7   // Control pin 2 for Motor A (Right)
#define IN3 8   // Control pin 1 for Motor B (Left)
#define IN4 9   // Control pin 2 for Motor B (Left)
#define ENB 10  // Enable pin for Motor B

#define TRIG_PIN 12
#define ECHO_PIN 13
#define SERVO_PIN 11

#define OBSTACLE_THRESHOLD 30  // Distance limit to detect obstacles (in cm)

int speedValue = 150;  // Motor speed (0-255)
ServoTimer2 servo;

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

  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);

  servo.attach(SERVO_PIN);  
  servo.write(1500); // Face forward

  moveForward();
  delay(2000);
}

void loop() {

  // Exercise: 
  // 1. Get front distance using ultrasonic sensor reading


  // 2. Check if an obstacle is close:
  
  
  
  // 2a. If obstacle is closer than threshold, Avoid Obstacle  


  
  // 2b. If not (Else), keep moving forward



  delay(50);
}




// Get Distance
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Add timeout (30ms)
  
  if (duration == 0) {
    return 999;  // Return a high value if no echo received
  }
  
  int distance = duration * 0.034 / 2;
  Serial.println("Front Distance: " + String(distance));

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

// Move Forward
void moveForward() {
  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Move Backward
void moveBackward() {
  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Turn Right (right motor stops, left motor moves forward)
void turnRight() {
  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Turn Left (left motor stops, right motor moves forward)
void turnLeft() {
  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Stop Motors
void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Avoid Obstacle
void avoidObstacle(){
  stopCar();
  delay(500);

  int distanceLeft = lookLeft();
  delay(100);
  int distanceRight = lookRight();

  Serial.print("Left: ");
  Serial.print(distanceLeft);
  Serial.print(" | Right: ");
  Serial.println(distanceRight);

  if (distanceRight > distanceLeft) {
    moveBackward();
    delay(500);
    stopCar();
    turnRight();
    delay(1000);
    stopCar();
    delay(500);
  } else {
    moveBackward();
    delay(500);
    stopCar();
    turnLeft();
    delay(1000);
    stopCar();
    delay(500);
  }
}
