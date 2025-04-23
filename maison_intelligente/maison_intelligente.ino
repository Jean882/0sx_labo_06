// Librairies
#include <AccelStepper.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <Buzzer.h>
#include <Button.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define Pins
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11
#define MOTOR_INTERFACE_TYPE 4
#define BTN_PIN 12

// RGB LED PINS
const int PIN_RED   = 7;
const int PIN_GREEN = 6;
const int PIN_BLUE  = 5;

// SSD1306 0x3C
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// AccepStepper motor
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
int distanceThresholdOpen = 30;
int distanceThresholdClose = 60;
int alarmLimit = 15;
int limInf = 30;
int limSup = 60;

bool alarmActive = false;
bool ledRGBState = false;

String tempMessage = "";
unsigned long tempMessageStartTime = 0;
const unsigned long tempMessageDuration = 3000;

#pragma region

// return map angle
int getCurrentAngle() { 
  return map(myStepper.currentPosition(), closedPosition, openPosition, minAngle, maxAngle);
}

// return distance
double measureDistance() {
  return distanceSensor.dist();
}

// LCD Doors
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
      lcd.print(" deg       ");
      break;
  }
}

// LCD Alarm
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

// Dessin check
void drawCheckSymbol() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.drawLine(48, 32, 64, 48, SSD1306_WHITE);
  display.drawLine(64, 48, 80, 16, SSD1306_WHITE);
  
  display.display();
}

// Dessin du X
void drawCrossSymbol() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.drawLine(48, 16, 80, 48, SSD1306_WHITE); // Diagonale \
  
  display.drawLine(48, 48, 80, 16, SSD1306_WHITE); // Diagonale /
  
  display.display();
}

// Dessin du cercle
void drawErrorSymbol() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.drawCircle(64, 32, 25, SSD1306_WHITE); // Cercle
  display.drawLine(48, 16, 80, 48, SSD1306_WHITE); // Diagonale \ 
  display.drawLine(48, 48, 80, 16, SSD1306_WHITE); // Diagonale /
  
  display.display();
}

void displayCheck() {
  drawCheckSymbol();
  tempMessageStartTime = millis();
  tempMessage = "check"; 
}

void displayCross() {
  drawCrossSymbol();
  tempMessageStartTime = millis();
  tempMessage = "cross"; 
}

void displayError() {
  drawErrorSymbol();
  tempMessageStartTime = millis();
  tempMessage = "error"; 
}

void updateTempMessage() {
  if (tempMessage != "" && millis() - tempMessageStartTime >= tempMessageDuration) {
    tempMessage = "";
    display.clearDisplay(); // Efface l'écran après 3 secondes
    display.display();
  }
}

// RBG LED RED
void redColor() {
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_BLUE, HIGH);
}

// RBG LED BLUE
void blueColor() {
  digitalWrite(PIN_BLUE, LOW);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

// RBG LED NO COLORS
void noColors() {
  digitalWrite(PIN_BLUE, HIGH);
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

#pragma endregion

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

  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);

  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(250);
  myStepper.setCurrentPosition(closedPosition);
  myStepper.disableOutputs();

  // Initialisation OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // boucle bloquante si échec
  }
  display.clearDisplay();
  display.display();

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

  if (estClic(currentTime)) {
    updateState(distance); 
    displayLCDDoor(distance);
  } else {
    displayLCDAlarm(distance);
    updateAlarm(distance);
  }

  myStepper.run();

  updateTempMessage();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Enlever les espaces autour de la chaîne

    if (input == "gdist") {
      Serial.println((double)distance);
      displayCheck();
      return;
    }

    // Commande de configuration de l'alarme
    if (input.startsWith("cfg;alm;")) {
      int newVal = input.substring(8).toInt();
      alarmLimit = newVal;
      Serial.print("Configure la distance de détection de l’alarme à ");
      Serial.print(alarmLimit);
      Serial.println(" cm");
      displayCheck();
      return;
    }

    // Commande de configuration du seuil d'ouverture
    if (input.startsWith("cfg;lim_inf;")) {
      int newVal = input.substring(12).toInt();
      if (newVal >= distanceThresholdClose) {
        Serial.println("Erreur – Seuil d'ouverture supérieur ou égal au seuil de fermeture.");
        displayCross();
      } else {
        distanceThresholdOpen = newVal;
        Serial.print("Seuil d’ouverture de porte configuré à ");
        Serial.print(distanceThresholdOpen);
        Serial.println(" cm.");
        displayCheck();
      }
      return;
    }

    // Commande de configuration du seuil de fermeture
    if (input.startsWith("cfg;lim_sup;")) {
      int newVal = input.substring(12).toInt();
      if (distanceThresholdOpen >= newVal) {
        Serial.println("Erreur – Seuil de fermeture inférieur ou égal au seuil d'ouverture.");
        displayCross();
      } else {
        distanceThresholdClose = newVal;
        Serial.print("Seuil de fermeture de porte configuré à ");
        Serial.print(distanceThresholdClose);
        Serial.println(" cm.");
        displayCheck();
      }
      return;
    }

    // Si la commande est inconnue, afficher une erreur
    Serial.println("Commande inconnue");
    drawErrorSymbol();
    tempMessageStartTime = millis();
    tempMessage = "Commande inconnue";
  }
}

#pragma endregion

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
}
