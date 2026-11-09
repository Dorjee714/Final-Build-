#include <QTRSensors.h>
#include <SPI.h>
#include <SD.h>


// QTR-MD-04RC LINE SENSOR


QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

const uint8_t qtrPins[SensorCount] = {
  A0, A1, A2, A3
};


// HC-SR04 ULTRASONIC SENSOR


const int trigPin = 3;
const int echoPin = 2;

long duration;
int distance;


// L298N MOTOR DRIVER


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

unsigned long redValue;
unsigned long greenValue;
unsigned long blueValue;


// SD CARD


const int chipSelect = 10;



int motorSpeed = 130;
int obstacleDistance = 10;




void setup() {

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Colour sensor pins
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colourOut, INPUT);

  // QTR sensor
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);

  // SD card
  if (!SD.begin(chipSelect)) {
    // SD card could not initialise
  }

  // Stop motors initially
  stopMotors();
}


// MAIN LOOP
// =====================================

void loop() {

  distance = getDistance();

  // Check for an obstacle
  if (distance > 0 && distance <= obstacleDistance) {

    stopMotors();

    delay(300);

    // Read colour sensor
    readColour();

    delay(300);

  } else {

    followLine();

  }

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


// LINE FOLLOWING


void followLine() {

  uint16_t position =
    qtr.readLineBlack(sensorValues);

  int leftError = position - 1500;

  int correction = leftError / 8;

  int leftSpeed = motorSpeed + correction;
  int rightSpeed = motorSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  moveForward(leftSpeed, rightSpeed);
}


// MOTOR CONTROL


void moveForward(int leftSpeed, int rightSpeed) {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void stopMotors() {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// COLOUR SENSOR


unsigned long readColourFrequency(
  int s2Value,
  int s3Value
) {

  digitalWrite(S2, s2Value);
  digitalWrite(S3, s3Value);

  delay(50);

  unsigned long frequency =
    pulseIn(colourOut, LOW, 100000);

  return frequency;
}

void readColour() {

  // Red filter
  redValue = readColourFrequency(LOW, LOW);

  // Blue filter
  blueValue = readColourFrequency(LOW, HIGH);

  // Green filter
  greenValue = readColourFrequency(HIGH, HIGH);
}


// SD CARD INSTRUCTIONS


void readInstructions() {

  File instructionFile =
    SD.open("ROBOT.TXT");

  if (instructionFile) {

    while (instructionFile.available()) {

      char instruction =
        instructionFile.read();

      // Instructions can be processed here

    }

    instructionFile.close();
  }
}

