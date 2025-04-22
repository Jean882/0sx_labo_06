#include <AccelStepper.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <Buzzer.h>
#include <Button.h>

// Define Pins
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11
#define MOTOR_INTERFACE_TYPE 4
#define BTN_PIN 12
const int PIN_RED   = 7;
const int PIN_GREEN = 6;
const int PIN_BLUE  = 5;


AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
HCSR04 distanceSensor(TRIGGER_PIN, ECHO_PIN);
Buzzer buzzer(11, 13);

// States
enum DoorState { CLOSED, OPENING, OPEN, CLOSING };
DoorState state = CLOSED;

// Timing
unsigned long lastMeasureTime = 0;
unsigned long lastAlarmTriggerTime = 0;
unsigned long lastColorSwitch = 0;

// Constantes fixes
const long closedPosition = 0;
const long openPosition = 1000;
const unsigned long measureInterval = 50;
const unsigned long alarmTimeout = 3000;
const unsigned long colorInterval = 250;
const unsigned long minAngle = 10;
const unsigned long maxAngle = 170;

// Paramètres configurables
int alarmLimit = 15;
int limInf = 20;
int limSup = 80;

bool alarmActive = false;
bool ledRGBState = false;

String tempMessage = "";
unsigned long tempMessageStartTime = 0;
const unsigned long tempMessageDuration = 3000;

byte checkSymbol[8] = {
  B00000,
  B00001,
  B00010,
  B10100,
  B01000,
  B00000,
  B00000,
  B00000
};

byte crossSymbol[8] = {
  B00000,
  B10001,
  B01010,
  B00100,
  B01010,
  B10001,
  B00000,
  B00000
};

byte banSymbol[8] = {
  B11111,
  B10001,
  B01010,
  B00100,
  B01010,
  B10001,
  B11111,
  B00000
};

int getCurrentAngle() {  // return map angle
  return map(myStepper.currentPosition(), closedPosition, openPosition, minAngle, maxAngle);
}

double measureDistance() {
  return distanceSensor.dist();
}

void displayLCDDoor(double distance) {
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  switch (state) {
    case CLOSED:
      lcd.print("Porte: Closed");
      break;
    case OPEN:
      lcd.print("Porte: Open   ");
      break;
    default:
      lcd.print("Porte: ");
      lcd.print(getCurrentAngle());
      lcd.print(" deg"       );
      break;
  }
}

void displayLCDAlarm(double distance) {
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm   ");

  lcd.setCursor(0, 1);
  lcd.print("Alarme: ");
  if (alarmActive) {
    lcd.print("on      ");
  } else {
    lcd.print("off     ");
  }
}

void displayCheck() {
  tempMessage = "[OK]";
  tempMessageStartTime = millis();
}

void displayCross() {
  tempMessage = "[ERREUR]";
  tempMessageStartTime = millis();
}

void displayErrorSymbol() {
  tempMessage = "[!!!]";
  tempMessageStartTime = millis();
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

void updateAlarm(double distance) {
  unsigned long currentTime = millis();
  
  if (distance <= alarmLimit) {
    alarmActive = true;
    lastAlarmTriggerTime = currentTime;
    buzzer.sound(500, 100);
  }

  if (alarmActive) {
    if (currentTime - lastColorSwitch >= colorInterval) {
      lastColorSwitch = currentTime;
      ledRGBState = !ledRGBState;
      ledRGBState ? redColor() : blueColor();
    }

    if (currentTime - lastAlarmTriggerTime >= alarmTimeout) {
      alarmActive = false;
      buzzer.sound(0, 0);
      noColors();
    }
  } else {
    buzzer.sound(0, 0);
    noColors();
  }
}

void updateState(double distance) {
  switch (state) {
    case CLOSED:
      if (distance < limInf) {
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
      if (distance > limSup) {
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

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);

  lcd.createChar(0, checkSymbol); // ✔️
  lcd.createChar(1, crossSymbol); // ❌
  lcd.createChar(2, banSymbol);   // 🚫


  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(250);
  myStepper.setCurrentPosition(closedPosition);
  myStepper.disableOutputs();

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

  if (tempMessage != "") {
  if (millis() - tempMessageStartTime <= tempMessageDuration) {
    lcd.setCursor(0, 0);
    lcd.print("     " + tempMessage + "      ");
    lcd.setCursor(0, 1);
    lcd.print("                ");  // effacer ligne 1
  } else {
    tempMessage = "";
    lcd.clear();  // on vide juste après la fin du message
  }
} else {
  if (estClic(currentTime)) {
    updateState(distance); 
    displayLCDDoor(distance);
  } else {
    displayLCDAlarm(distance);
    updateAlarm(distance);
  }
}



  if (currentTime - lastMeasureTime >= measureInterval) {
    lastMeasureTime = currentTime;
    distance = measureDistance();
  }

  

  myStepper.run();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "gdist") {
      Serial.println((double)distance);
      return;
    }

    if (input.startsWith("cfg;alm;")) {
      int newVal = input.substring(8).toInt();
      alarmLimit = newVal;
      Serial.print("Configure la distance de détection de l’alarme à ");
      Serial.print(alarmLimit);
      Serial.println(" cm");
      displayCheck();
      return;
    }

    if (input.startsWith("cfg;lim_inf;")) {
      int newVal = input.substring(12).toInt();
      if (newVal >= limSup) {
        Serial.println("Erreur – Limite inférieure plus grande que limite supérieure");
        displayErrorSymbol();
      } else {
        limInf = newVal;
        Serial.print("Il configure sa limite inférieure du moteur à ");
        Serial.print(limInf);
        Serial.println(" cm.");
        displayCheck();
      }
      return;
    }

    if (input.startsWith("cfg;lim_sup;")) {
      int newVal = input.substring(12).toInt();
      if (limInf >= newVal) {
        Serial.println("Erreur – Limite inférieure plus grande que limite supérieure");
        displayErrorSymbol();
      } else {
        limSup = newVal;
        Serial.print("Il configure sa limite supérieure du moteur à ");
        Serial.print(limSup);
        Serial.println(" cm.");
        displayCheck();
      }
      return;
    }

    displayCross(); // commande inconnue
  }
}

int estClic(unsigned long ct) {
  static unsigned long lastTime = 0;
  static int lastState = HIGH;
  const int rate = 50;
  static int clic = 0;

  if (ct - lastTime < rate) {
    return clic; // Trop rapide
  }

  lastTime = ct;

  int state = digitalRead(BTN_PIN);

  if (state == LOW) {
    if (state != lastState) {
      clic = !clic;
    }
  }

  lastState = state;

  return clic;
} // end of estClic
