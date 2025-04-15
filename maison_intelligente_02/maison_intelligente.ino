#include <AccelStepper.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <Buzzer.h>

// Define Pins
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11
#define MOTOR_INTERFACE_TYPE 4
const int PIN_RED   = 7;
const int PIN_GREEN = 6;
const int PIN_BLUE  = 5;



AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
HCSR04 distanceSensor(TRIGGER_PIN, ECHO_PIN);
Buzzer buzzer(11, 13);

// States
enum DoorState { CLOSED,
                 OPENING,
                 OPEN,
                 CLOSING };
DoorState state = CLOSED;

// Global variables
unsigned long lastMeasureTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastAlarmTriggerTime = 0;
unsigned long lastColorSwitch = 0;


// Constant variables
const int distanceThresholdOpen = 30;
const int distanceThresholdClose = 60;
const int distanceThresoldAlarm = 15;
const long closedPosition = 0;
const long openPosition = 1000;
const unsigned long measureInterval = 50;
const unsigned long serialInterval = 100;
const unsigned long minAngle = 10;
const unsigned long maxAngle = 170;
const unsigned long alarmTimeout = 3000; // 3 secondes
const unsigned long colorInterval = 250; // changer de couleur toutes les 250 ms

bool alarmActive = false;
bool ledRGBState = false;

#pragma region

//int getCurrentAngle() {  // return map angle
  //return map(myStepper.currentPosition(), closedPosition, openPosition, minAngle, maxAngle);
//}

double measureDistance() {  // return distance
  return distanceSensor.dist();
}

void displayLCD(double distance) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm");

  lcd.setCursor(0, 1);

  lcd.print("Alarme: ");
  if (alarmActive) {
    lcd.print("on");
  } else {
    lcd.print("off");
  }

  //switch (state) {
    //case CLOSED:
      //lcd.print("Porte: Closed");
      //break;
    //case OPEN:
      //lcd.print("Porte: Open");
      //break;
    //default:
      //lcd.print("Porte: ");
      //lcd.print(getCurrentAngle());
      //lcd.print(" deg");
      //break;
  //}
}

void displaySerial(double distance) {
  Serial.print("etd:2206160");
  Serial.print(",dist:");
  Serial.print(distance);
  //Serial.print(",deg:");
  //Serial.println(getCurrentAngle());
  Serial.print(",alarme:");
  Serial.println(alarmActive);
}

void redColor() {
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_BLUE, HIGH);
}

void blueColor() {
  digitalWrite(PIN_BLUE, LOW);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void noColors() {
  digitalWrite(PIN_BLUE, HIGH);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

#pragma endregion

void updateAlarm(double distance) {
  unsigned long currentTime = millis();
  
  if (distance <= distanceThresoldAlarm) {
    alarmActive = true;
    lastAlarmTriggerTime = currentTime;
    buzzer.sound(500, 100); // fréquence de 500 Hz pendant 100 ms
  }

  if (alarmActive) {
    if (currentTime - lastColorSwitch >= colorInterval) {
      lastColorSwitch = currentTime;
      ledRGBState = !ledRGBState;
      if (ledRGBState) {
        redColor();
      } else {
        blueColor();
      }
    }

    if (currentTime - lastAlarmTriggerTime >= alarmTimeout) {
      alarmActive = false;
      buzzer.sound(0, 0); // stop sound
      noColors();
    }
  } else {
    buzzer.sound(0, 0); // stop sound
    noColors();
  }
}



void updateState(double distance) {
  switch (state) {
    case CLOSED:
      if (distance < distanceThresholdOpen) {
        state = OPENING;
        myStepper.enableOutputs();
        myStepper.moveTo(openPosition);
      }
      break;

    case OPENING:
      if (myStepper.distanceToGo() == 0) {
        state = OPEN;
        myStepper.disableOutputs();
      }
      break;

    case OPEN:
      if (distance > distanceThresholdClose) {
        state = CLOSING;
        myStepper.enableOutputs();
        myStepper.moveTo(closedPosition);
      }
      break;

    case CLOSING:
      if (myStepper.distanceToGo() == 0) {
        state = CLOSED;
        myStepper.disableOutputs();
      }
      break;
  }
}

#pragma region setup - loop

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  

  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);

  //myStepper.setMaxSpeed(500);
  //myStepper.setAcceleration(250);
  //myStepper.setCurrentPosition(closedPosition);
  //myStepper.disableOutputs();

  while (millis() <= 2000) {

    lcd.setCursor(0, 0);
    lcd.print("2206160      ");
    lcd.setCursor(0, 1);
    lcd.print("Smart Home   ");
  }
}

void loop() {
  unsigned long currentTime = millis();
  static double distance = 0;

  if (currentTime - lastMeasureTime >= measureInterval) {
    lastMeasureTime = currentTime;
    distance = measureDistance();
    
  }
  //updateState(distance);
  updateAlarm(distance);
  //myStepper.run();

  if (currentTime - lastSerialTime >= serialInterval) {
    lastSerialTime = currentTime;
    displayLCD(distance);
    displaySerial(distance);
  }
}

#pragma endregion
