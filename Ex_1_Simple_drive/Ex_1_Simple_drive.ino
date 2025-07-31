// Arduino Robot Car - Open Loop Control Demo
// This code moves the robot forward, backward, and makes it turn

// Motor Pins
#define ENA 5   // Speed control for right motor
#define IN1 6  
#define IN2 7
#define IN3 8 
#define IN4 9
#define ENB 10  // Speed control for left motor

void setup() {
  Serial.begin(9600);  

  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Robot starting in 2 seconds...");
  delay(2000);
}

void loop() {

  // Write your driving sequence here. 
  //Speed ranges from 0 - 255

  // 1. Drive forward at a speed of 120 for 5 seconds



  // 2. Turn left at a speed of 60 for 2 seconds
  


  // 3. Reverse at a speed of 85 for 4 seconds



  // 4. Turn right at a speed of 40 for 4 seconds



  // 5. Drive forward at a speed of 120 for 2 seconds



  // 6. Stop.
  
  while (true); // End program (stays still till restart)
}

// Move forward
void goForward(int speed) {
  Serial.println("Driving forward...");
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Move backward
void goBackward(int speed) {
  Serial.println("Reversing...");
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Turn right
void turnRight(int speed) {
  Serial.println("Turning right...");
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Turn left
void turnLeft(int speed) {
  Serial.println("Turning left...");
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Stop
void stopCar() {
  Serial.println("Stopping...");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
