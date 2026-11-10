#include <QTRSensors.h>
#include <SPI.h>
#include <SD.h>

// LINE SENSOR

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


// MOTOR DRIVER

const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

const int ENB = 5;
const int IN3 = 6;
const int IN4 = 4;

int motorSpeed = 130;
int searchSpeed = 90;
int pushSpeed = 110;

int danceDuration = 500;


// COLOUR SENSOR


const int S2 = A4;
const int S3 = A5;
const int colourOut = 0;

unsigned long redValue;
unsigned long greenValue;
unsigned long blueValue;

const int GREEN_RATIO_PERCENT = 85;


// SD CARD


const int chipSelect = 10;
bool sdReady = false;




const int obstacleDistance = 10;

unsigned long searchTimeout = 15000;

unsigned long searchStartTime = 0;
unsigned long lastColourCheck = 0;

int nonGreenCount = 0;

// Robot stages
int robotStage = 0;

// 0 = Follow line
// 1 = Search for object
// 2 = Push object
// 3 = Perform dance
// 4 = Finished





void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colourOut, INPUT);

  stopMotors();

  // Set up line sensor
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);

  // Calibrate line sensor
  for (int i = 0; i < 100; i++) {

    qtr.calibrate();
    delay(20);
  }

  // Start SD card
  sdReady = SD.begin(chipSelect);

  stopMotors();
}




void loop() {

  // STAGE 0: FOLLOW THE LINE

  if (robotStage == 0) {

    followLine();

    if (millis() - lastColourCheck >= 250) {

      lastColourCheck = millis();

      if (isGreen()) {

        stopMotors();

        nonGreenCount = 0;

        searchStartTime = millis();

        robotStage = 1;
      }
    }
  }


  // STAGE 1: SEARCH FOR OBJECT

  else if (robotStage == 1) {

    if (millis() - searchStartTime >= searchTimeout) {

      stopMotors();

      robotStage = 4;
    }

    else {

      if (millis() - lastColourCheck >= 250) {

        lastColourCheck = millis();

        if (isGreen()) {
          nonGreenCount = 0;
        }

        else {
          nonGreenCount++;
        }

        if (nonGreenCount >= 3) {

          stopMotors();

          robotStage = 4;
        }
      }

      if (robotStage == 1) {

        distance = getDistance();

        if (distance > 0 &&
            distance <= obstacleDistance) {

          stopMotors();

          nonGreenCount = 0;

          robotStage = 2;
        }

        else {

          moveForward(searchSpeed, searchSpeed);
        }
      }
    }
  }


  // STAGE 2: PUSH THE OBJECT

  else if (robotStage == 2) {

    moveForward(pushSpeed, pushSpeed);

    if (millis() - lastColourCheck >= 250) {

      lastColourCheck = millis();

      if (isGreen()) {

        nonGreenCount = 0;
      }

      else {

        nonGreenCount++;
      }

      if (nonGreenCount >= 3) {

        stopMotors();

        delay(500);

        robotStage = 3;
      }
    }
  }


  // STAGE 3: PERFORM DANCE

  else if (robotStage == 3) {

    stopMotors();

    performDance();

    robotStage = 4;
  }


  // STAGE 4: FINISHED

  else if (robotStage == 4) {

    stopMotors();
  }
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


void moveBackward(int leftSpeed, int rightSpeed) {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

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



// LINE FOLLOWING


void followLine() {

  uint16_t position = qtr.readLineBlack(sensorValues);

  int leftError = position - 1500;

  int correction = leftError / 8;

  int leftSpeed = motorSpeed + correction;
  int rightSpeed = motorSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  moveForward(leftSpeed, rightSpeed);
}



// ULTRASONIC SENSOR


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



// COLOUR SENSOR


unsigned long readColourFrequency(int s2Value, int s3Value) {

  digitalWrite(S2, s2Value);
  digitalWrite(S3, s3Value);

  delay(50);

  unsigned long frequency = pulseIn(colourOut, LOW, 50000);

  return frequency;
}


bool isGreen() {

  redValue = readColourFrequency(LOW, LOW);

  blueValue = readColourFrequency(LOW, HIGH);

  greenValue = readColourFrequency(HIGH, HIGH);

  if (redValue == 0 ||
      greenValue == 0 ||
      blueValue == 0) {

    return false;
  }

  if (greenValue * 100UL <
      redValue * GREEN_RATIO_PERCENT &&
      greenValue * 100UL <
      blueValue * GREEN_RATIO_PERCENT) {

    return true;
  }

  else {

    return false;
  }
}



// DANCE MOVEMENTS

void left() {

  // Turn left

  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(danceDuration);

  stopMotors();
  delay(150);
}


void right() {

  // Turn right

  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(danceDuration);

  stopMotors();
  delay(150);
}


void forward() {

  moveForward(motorSpeed, motorSpeed);

  delay(danceDuration);

  stopMotors();
  delay(150);
}


void backward() {

  moveBackward(motorSpeed, motorSpeed);

  delay(danceDuration);

  stopMotors();
  delay(150);
}


// READ DANCE FROM SD CARD


void performDance() {

  stopMotors();

  if (sdReady == true) {

    File danceFile = SD.open("DANCE.TXT");

    if (danceFile) {

      String dance = "";

      // Read the dance instructions

      while (danceFile.available()) {

        char character = danceFile.read();

        if (character == 'L' ||
            character == 'R' ||
            character == 'F' ||
            character == 'B') {

          dance = dance + character;
        }
      }

      danceFile.close();

      // Convert String into character array

      char moves[64];

      dance.toCharArray(moves, 64);

      // Read each dance instruction

      for (int i = 0; i < dance.length() && i < 63; i++) {

        if (moves[i] == 'L') {

          left();
        }

        else if (moves[i] == 'R') {

          right();
        }

        else if (moves[i] == 'F') {

          forward();
        }

        else if (moves[i] == 'B') {

          backward();
        }
      }
    }
  }

  stopMotors();
}

