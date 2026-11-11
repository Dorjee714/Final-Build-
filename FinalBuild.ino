#include <QTRSensors.h>
#include <SPI.h>
#include <SD.h>


// QTR LINE SENSOR

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

const uint8_t qtrPins[SensorCount] = {
  A0, A1, A2, A3
};


// ULTRASONIC SENSOR


const int trigPin = 3;
const int echoPin = 2;

long duration;
int distance;

// MOTOR

const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;
const int ENB = 5;
const int IN3 = 6;
const int IN4 = 4;


// TCS230 COLOUR SENSOR


const int S2 = A4;
const int S3 = A5;
const int colourOut = 0;  // D0


// SD CARD


const int SD_CS = 10;


// ROBOT SETTINGS
int motorSpeed = 130;
int searchSpeed = 90;
int pushSpeed = 110;

const int objectDistance = 10;

const int GREEN_RATIO_PERCENT = 85;

int nonGreenCount = 0;


// ROBOT STATUS


bool greenZoneFound = false;
bool pushingObject = false;
bool rescueComplete = false;
bool danceDone = false;



void setup() {

  // Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Colour sensor
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colourOut, INPUT);

  // SD card
  pinMode(SD_CS, OUTPUT);

  // QTR line sensor
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setTimeout(2500);

  stopMotors();

  // Calibrate line sensor
  // Move the sensor over black and white
  // during this calibration period.

  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(20);
  }
}


void loop() {

  // 1. Follow the line until green zone
  if (!greenZoneFound) {

    followLine();

    if (isGreen()) {
      greenZoneFound = true;
      stopMotors();
      delay(500);
    }
  }

  // 2. Search for the object
  else if (greenZoneFound &&
           !pushingObject &&
           !rescueComplete) {

    distance = getDistance();

    if (distance > 0 && distance <= objectDistance) {

      stopMotors();
      pushingObject = true;
      delay(300);

    } else {

      searchForObject();
    }
  }

  // 3. Push object outside green zone
  else if (pushingObject && !rescueComplete) {

    moveForward(pushSpeed);

    delay(200);

    if (isGreen()) {

      nonGreenCount = 0;

    } else {

      nonGreenCount++;
    }

    if (nonGreenCount >= 3) {

      stopMotors();

      pushingObject = false;
      rescueComplete = true;

      delay(500);
    }
  }

  // 4. Check SD card and perform dance
  else if (rescueComplete && !danceDone) {

    stopMotors();

    if (SD.begin(SD_CS)) {

      if (performDance()) {
        danceDone = true;
      }

    }

    delay(500);
  }

  // 5. Stop after dancing
  else if (danceDone) {

    stopMotors();
  }
}

// LINE FOLLOWING


void followLine() {

  uint16_t position = qtr.readLineBlack(sensorValues);

  int leftError = (int)position - 1500;

  int correction = leftError / 8;

  int leftSpeed = motorSpeed + correction;
  int rightSpeed = motorSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}


// ULTRASONIC DISTANCE


int getDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return -1;
  }

  distance = duration * 0.034 / 2;

  return distance;
}


// COLOUR DETECTION


unsigned long readColour(bool s2Value, bool s3Value) {

  digitalWrite(S2, s2Value);
  digitalWrite(S3, s3Value);

  delay(10);

  return pulseIn(colourOut, LOW, 50000);
}

bool isGreen() {

  // Read red
  unsigned long red = readColour(LOW, LOW);

  // Read green
  unsigned long green = readColour(HIGH, HIGH);

  // Read blue
  unsigned long blue = readColour(LOW, HIGH);

  if (red == 0 || green == 0 || blue == 0) {
    return false;
  }

  if (green * 100 <
      red * GREEN_RATIO_PERCENT &&
      green * 100 <
      blue * GREEN_RATIO_PERCENT) {

    return true;
  }

  return false;
}


// SEARCH FOR OBJECT


void searchForObject() {

  // Turn in place
  setLeftMotor(searchSpeed);
  setRightMotor(-searchSpeed);
}


// MOTOR CONTROL


void setRightMotor(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

  } else if (speed < 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

  } else {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, abs(speed));
}


void setLeftMotor(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

  } else if (speed < 0) {

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

  } else {

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENB, abs(speed));
}


// MOVEMENT FUNCTIONS


void moveForward(int speed) {

  setLeftMotor(speed);
  setRightMotor(speed);
}

void moveBackward(int speed) {

  setLeftMotor(-speed);
  setRightMotor(-speed);
}

void stopMotors() {

  setLeftMotor(0);
  setRightMotor(0);
}


// DANCE MOVEMENTS


void danceLeft() {

  setLeftMotor(-100);
  setRightMotor(100);

  delay(500);

  stopMotors();
  delay(150);
}

void danceRight() {

  setLeftMotor(100);
  setRightMotor(-100);

  delay(500);

  stopMotors();
  delay(150);
}

void danceForward() {

  moveForward(100);

  delay(500);

  stopMotors();
  delay(150);
}

void danceBackward() {

  moveBackward(100);

  delay(500);

  stopMotors();
  delay(150);
}


// READ DANCE INSTRUCTIONS FROM SD CARD


bool performDance() {

  File danceFile = SD.open("DANCE.TXT");

  if (!danceFile) {
    return false;
  }

  char moves[64];

  int moveCount = 0;

  while (danceFile.available() && moveCount < 63) {

    char instruction = danceFile.read();

    if (instruction == 'L' ||
        instruction == 'R' ||
        instruction == 'F' ||
        instruction == 'B') {

      moves[moveCount] = instruction;
      moveCount++;
    }
  }

  moves[moveCount] = '\0';

  danceFile.close();

  if (moveCount == 0) {
    return false;
  }

  // Perform instructions
  for (int i = 0; i < moveCount; i++) {

    if (moves[i] == 'L') {
      danceLeft();
    }

    else if (moves[i] == 'R') {
      danceRight();
    }

    else if (moves[i] == 'F') {
      danceForward();
    }

    else if (moves[i] == 'B') {
      danceBackward();
    }
  }

  stopMotors();

  return true;
}
