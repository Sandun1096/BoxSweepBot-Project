#define IN1 8   // Left motor forward
#define IN2 9   // Left motor backward
#define ENA 3   // Left motor speed control
#define IN3 10  // Right motor forward
#define IN4 11  // Right motor backward
#define ENB 5   // Right motor speed control

// IR Sensors
#define FRONT_LEFT_IR 6  // Front left IR sensor
#define FRONT_RIGHT_IR 4 // Front right IR sensor
#define BACK_LEFT_IR 7   // Back left IR sensor
#define BACK_RIGHT_IR 2  // Back right IR sensor

// Mosfet Pins
#define BLOWER_PIN 13   // Blower control pin
#define ROTOR_PIN 12

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(FRONT_LEFT_IR, INPUT);
  pinMode(FRONT_RIGHT_IR, INPUT);
  pinMode(BACK_LEFT_IR, INPUT);
  pinMode(BACK_RIGHT_IR, INPUT);

  pinMode(BLOWER_PIN, OUTPUT);
  pinMode(ROTOR_PIN, OUTPUT);

  // Turn on blower and rotor
  digitalWrite(BLOWER_PIN, HIGH);
  digitalWrite(ROTOR_PIN, HIGH);
}

void loop() {
  int rotationCount = 0;

  // Rotate and detect
  while (rotationCount < 8) {
    rotateRobot();

    // Continuously monitor all IR sensors during rotation
    if (detectSignal()) {
      avoidDetectedDirection();
    }
    rotationCount++;
  }

  // Move forward after rotation
  moveForwardWithDetection(1000); // Move forward for 2 seconds with IR monitoring

  // Resume rotation and detection
  rotationCount = 0;
  while (rotationCount < 8) {
    rotateRobot();

    // Continuously monitor all IR sensors during rotation
    if (detectSignal()) {
      avoidDetectedDirection();
    }
    rotationCount++;
  }
}

// Rotate the robot 360° once with IR monitoring
void rotateRobot() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 150); // Moderate speed
  analogWrite(ENB, 150);

  unsigned long startTime = millis();
  while (millis() - startTime < 1125) { // Adjust timing for 360° divided into 8 segments
    if (detectSignal()) {
      avoidDetectedDirection();
      break;
    }
  }
  stopMotors();
}

// Detect any signal from all IR sensors
bool detectSignal() {
  return digitalRead(FRONT_LEFT_IR) == HIGH || digitalRead(FRONT_RIGHT_IR) == HIGH ||
         digitalRead(BACK_LEFT_IR) == HIGH || digitalRead(BACK_RIGHT_IR) == HIGH;
}

// Stop and move away in a random direction
void avoidDetectedDirection() {
  stopMotors();

  int randomDir = random(0, 4);

  switch (randomDir) {
    case 0: moveForward(1000); break;  // Move forward
    case 1: moveBackward(1000); break; // Move backward
    case 2: moveLeft(1000); break;     // Move left
    case 3: moveRight(1000); break;    // Move right
  }
}

// Move forward for a specified duration with IR monitoring
void moveForwardWithDetection(int duration) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    if (detectSignal()) {
      avoidDetectedDirection();
      break;
    }
  }
  stopMotors();
}

// Move forward for a specified duration
void moveForward(int duration) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  delay(duration);
  stopMotors();
}

// Move backward for a specified duration
void moveBackward(int duration) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  delay(duration);
  stopMotors();
}

// Move left for a specified duration
void moveLeft(int duration) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  delay(duration);
  stopMotors();
}

// Move right for a specified duration
void moveRight(int duration) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  delay(duration);
  stopMotors();
}

// Stop all motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
