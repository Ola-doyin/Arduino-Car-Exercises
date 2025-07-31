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

#define OBSTACLE_THRESHOLD 30  // Stop distance (cm)
#define SLOW_DOWN_DISTANCE 45  // Slow down threshold (cm)

int normalSpeed = 120;  // Normal speed (0-255)
int slowSpeed = 60;     // Slowed speed when approaching an obstacle
ServoTimer2 servo;

// Function Prototypes
void moveForward(int speed);
void turnRight();
void turnLeft();
void moveBackward();
void gradualSpeedChange(int startSpeed, int targetSpeed);
int getDistance();
int lookRight();
int lookLeft();

void setup() {
  Serial.begin(9600);  // Start Serial Monitor for debugging

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  analogWrite(ENA, normalSpeed);
  analogWrite(ENB, normalSpeed);

  servo.attach(SERVO_PIN);
  servo.write(1500);  // Face forward

  Serial.println("Robot starting...");
  moveForward(normalSpeed);
  delay(2000);
}

void loop() {
  int distanceFront = getDistance();
  Serial.print("Front Distance: ");
  Serial.println(distanceFront);

  // Automatically adjust speed based on distance
  int currentSpeed = (distanceFront < SLOW_DOWN_DISTANCE) ? slowSpeed : normalSpeed;
  moveForward(currentSpeed);

  if (distanceFront < OBSTACLE_THRESHOLD) {
    Serial.println("Obstacle detected! Slowing down...");
    
    // Look left and right while still moving
    int distanceLeft, distanceRight;

    // Look left
    distanceLeft = lookLeft();
    delay(200);

   // Look right
    distanceRight = lookRight();
    delay(200);

    Serial.print("Left Distance: ");
    Serial.println(distanceLeft);
    Serial.print("Right Distance: ");
    Serial.println(distanceRight);

    if (distanceRight > distanceLeft) {
      Serial.println("Turning Right...");
      turnRight();
    } else {
      Serial.println("Turning Left...");
      turnLeft();
    }
  }

  delay(50);
}


// Get Distance
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  int distance = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
  Serial.print("Measured Distance: ");
  Serial.println(distance);
  return distance;
}

// Look Right
int lookRight() {
  Serial.println("Looking Right...");
  servo.write(544);
  delay(500);
  int distance = getDistance();
  servo.write(1500);
  delay(100);
  return distance;
}

// Look Left
int lookLeft() {
  Serial.println("Looking Left...");
  servo.write(2400);
  delay(500);
  int distance = getDistance();
  servo.write(1500);
  delay(100);
  return distance;
}

// Move Forward
void moveForward(int speed) {
  Serial.print("Moving Forward at speed: ");
  Serial.println(speed);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Turn Right
void turnRight() {
  Serial.println("Executing Right Turn...");
  analogWrite(ENA, normalSpeed);
  analogWrite(ENB, normalSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(1000); // Time for turning

  moveForward(slowSpeed);  // Resume movement after turning
}

// Turn Left
void turnLeft() {
  Serial.println("Executing Left Turn...");
  analogWrite(ENA, normalSpeed);
  analogWrite(ENB, normalSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(1000); // Time for turning

  moveForward(slowSpeed);  // Resume movement after turning
}


// Move Backward
void moveBackward() {
  Serial.println("Moving Backward...");
  analogWrite(ENA, normalSpeed);
  analogWrite(ENB, normalSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Gradual Speed Change
void gradualSpeedChange(int startSpeed, int targetSpeed) {
  Serial.print("Changing speed from ");
  Serial.print(startSpeed);
  Serial.print(" to ");
  Serial.println(targetSpeed);

  int step = (startSpeed < targetSpeed) ? 10 : -10;
  for (int i = startSpeed; i != targetSpeed; i += step) {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    delay(50);
  }
}
