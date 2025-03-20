#include <LiquidCrystal_I2C.h>  // Biblioteca pentru ecranul LCD cu I2C
#include <Servo.h>               // Biblioteca pentru controlul servomotoarelor
#include <Arduino_FreeRTOS.h>    // Biblioteca pentru utilizarea FreeRTOS (multithreading)
#include <semphr.h>              // Biblioteca pentru semafoare (folosită pentru sincronizare)

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Inițializarea obiectului LCD cu adresa 0x27, 16 coloane și 2 linii
Servo inComingServo;                 // Obiect pentru controlul servomotorului barieră de intrare
Servo outGoingServo;                 // Obiect pentru controlul servomotorului barieră de ieșire

const int servo1 = 51;               // Pinul pentru servomotorul de intrare
const int servo2 = 53;               // Pinul pentru servomotorul de ieșire

// Variabilele pentru starea barierelor
bool inComingBarrierClose = false;   // Stare barieră intrare (închisă sau deschisă)
bool outGoingBarrierClose = false;   // Stare barieră ieșire (închisă sau deschisă)
String slotsAvailability[4] = {"1", "2", "3", "4"};  // Sloturile de parcare disponibile (1 - disponibil, - - ocupat)

// Senzorii pentru sloturi și bariere
const int inComingSensor = 49;       // Senzorul pentru detectarea vehiculului la intrare
const int outGoingSensor = 47;       // Senzorul pentru detectarea vehiculului la ieșire

// Definirea pinilor RGB pentru fiecare slot de parcare
// Slot 1
const int r1 = 22;
const int g1 = 24;
const int b1 = 26;
const int sensor1 = 42;

// Slot 2
const int r2 = 28;
const int g2 = 30;
const int b2 = 32;
const int sensor2 = 44;

// Slot 3
const int r3 = 39;
const int g3 = 36;
const int b3 = 38;
const int sensor3 = 46;

// Slot 4
const int r4 = 40;
const int g4 = 35;
const int b4 = 37;
const int sensor4 = 48;

// Semafore pentru sincronizarea accesului la resurse partajate
SemaphoreHandle_t servoSemaphore;
SemaphoreHandle_t ledSemaphore;
SemaphoreHandle_t displaySemaphore;

void setup() {
  lcd.init();           // Inițializează LCD-ul
  lcd.backlight();      // Activează iluminarea de fundal a LCD-ului
  Serial.begin(9600);   // Deschide comunicarea serială la 9600 bps

  // Atașează servomotoarele la pini
  inComingServo.attach(servo1);
  outGoingServo.attach(servo2);

 // Setează pinurile pentru senzori și LED-uri ca input/output
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(inComingSensor, INPUT);
  pinMode(outGoingSensor, INPUT);

  pinMode(r1, OUTPUT);
  pinMode(g1, OUTPUT);
  pinMode(b1, OUTPUT);

  pinMode(r2, OUTPUT);
  pinMode(g2, OUTPUT);
  pinMode(b2, OUTPUT);

  pinMode(r3, OUTPUT);
  pinMode(g3, OUTPUT);
  pinMode(b3, OUTPUT);

  pinMode(r4, OUTPUT);
  pinMode(g4, OUTPUT);
  pinMode(b4, OUTPUT);

  // Crearea semafoarelor
  servoSemaphore = xSemaphoreCreateBinary();
  ledSemaphore = xSemaphoreCreateBinary();
  displaySemaphore = xSemaphoreCreateBinary();

  // Inițializare semafoare
  xSemaphoreGive(servoSemaphore);
  xSemaphoreGive(ledSemaphore);
  xSemaphoreGive(displaySemaphore);
}

void loop() {
  // Sarcini concurrente pentru actualizarea sloturilor, barierei de intrare/ieșire și LED-urilor
  taskUpdateSlots();
  taskManageIncomingBarrier();
  taskManageOutgoingBarrier();
  taskUpdateLEDs();
}

// Actualizează starea sloturilor de parcare și afișează pe LCD
void taskUpdateSlots() {
  if (xSemaphoreTake(displaySemaphore, portMAX_DELAY)) {
    static unsigned long lastDisplayUpdate = 0;
    const unsigned long displayInterval = 500;

    // Verifică intervalul de actualizare al LCD-ului
    if (millis() - lastDisplayUpdate >= displayInterval) {
      lastDisplayUpdate = millis();

      // Citește statusul senzorilor pentru fiecare slot
      int slotSensorStatus_1 = digitalRead(sensor1);
      int slotSensorStatus_2 = digitalRead(sensor2);
      int slotSensorStatus_3 = digitalRead(sensor3);
      int slotSensorStatus_4 = digitalRead(sensor4);

      // Actualizează disponibilitatea sloturilor
      slotsAvailability[0] = (slotSensorStatus_1 == 1) ? "1" : "-";
      slotsAvailability[1] = (slotSensorStatus_2 == 1) ? "2" : "-";
      slotsAvailability[2] = (slotSensorStatus_3 == 1) ? "3" : "-";
      slotsAvailability[3] = (slotSensorStatus_4 == 1) ? "4" : "-";

      lcd.clear();// Curăță LCD-ul
      if (slotsAvailability[0] != "-" || slotsAvailability[1] != "-" || slotsAvailability[2] != "-" || slotsAvailability[3] != "-") {
        lcd.setCursor(2, 0); // Setează cursorul la prima linie, a doua coloană
        lcd.print("Locuri libere");
        lcd.setCursor(0, 1); // Setează cursorul pe a doua linie
        lcd.print(slotsAvailability[0] + ", " + slotsAvailability[1] + ", " + slotsAvailability[2] + ", " + slotsAvailability[3]);
      } else {
        lcd.setCursor(2, 0);
        lcd.print("Parcare plina!");
      }
    }
    xSemaphoreGive(displaySemaphore); // Eliberează semaforul pentru alte taskuri
  }
}

// Gestionează bariera de intrare
void taskManageIncomingBarrier() {
  if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
    static unsigned long lastServoMove = 0;
    const unsigned long servoInterval = 20;

    int inComingSensorStatus = digitalRead(inComingSensor);  // Citește statusul senzorului de intrare

    // Verifică dacă parcarea este plină
    bool parkingFull = true;
    for (int i = 0; i < 4; i++) {
      if (slotsAvailability[i] != "-") {
        parkingFull = false; // Dacă există cel puțin un loc liber
        break;
      }
    }

    if (parkingFull) {
      // Dacă parcarea este plină, afișează mesajul și ține bariera închisă
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("Parcare plina! ");
      xSemaphoreGive(servoSemaphore);
      return; // Ieșire din funcție fără a acționa bariera
    }

    // Dacă parcarea nu este plină, acționăm bariera
    if (inComingSensorStatus == 1) { // Senzorul detectează un vehicul
      if (millis() - lastServoMove >= servoInterval) {
        lastServoMove = millis();
        inComingServo.write(max(inComingServo.read() - 1, 0)); // Coboară bariera
        if (inComingServo.read() == 0) {
          inComingBarrierClose = false; // Bariera este complet coborâtă
        }
      }
    } else { // Senzorul nu mai detectează vehicul
      if (millis() - lastServoMove >= servoInterval) {
        lastServoMove = millis();
        inComingServo.write(min(inComingServo.read() + 1, 90)); // Ridică bariera
        if (inComingServo.read() == 90) {
          inComingBarrierClose = true; // Bariera este complet ridicată
        }
      }
    }

    xSemaphoreGive(servoSemaphore); // Eliberează semaforul
  }
}

// Gestionează bariera de ieșire
void taskManageOutgoingBarrier() {
  if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
    static unsigned long lastServoMove = 0;
    const unsigned long servoInterval = 20;

    int outGoingSensorStatus = digitalRead(outGoingSensor); // Citește statusul senzorului de ieșire

    if (outGoingSensorStatus == 1) { // Senzorul detectează un vehicul
      if (millis() - lastServoMove >= servoInterval) {
        lastServoMove = millis();
        outGoingServo.write(max(outGoingServo.read() - 1, 0)); // Coboară bariera
        if (outGoingServo.read() == 0) {
          outGoingBarrierClose = false; // Bariera este complet coborâtă
        }
      }
    } else { // Senzorul nu mai detectează vehicul
      if (millis() - lastServoMove >= servoInterval) {
        lastServoMove = millis();
        outGoingServo.write(min(outGoingServo.read() + 1, 90)); // Ridică bariera
        if (outGoingServo.read() == 90) {
          outGoingBarrierClose = true; // Bariera este complet ridicată
        }
      }
    }

    xSemaphoreGive(servoSemaphore); // Eliberează semaforul
  }
}


// Actualizează LED-urile pentru sloturi pe baza disponibilității
void taskUpdateLEDs() {
  if (xSemaphoreTake(ledSemaphore, portMAX_DELAY)) {
    static unsigned long lastLEDUpdate = 0;
    const unsigned long ledInterval = 100;

    if (millis() - lastLEDUpdate >= ledInterval) {
      lastLEDUpdate = millis();

      // Verifică disponibilitatea fiecărui slot și setează culoarea corespunzătoare a LED-urilor
      if (slotsAvailability[0] == "-") {
        color(255, 0, 0, 1);  // Slotul 1 ocupat (roșu)
      } else {
        color(0, 255, 0, 1);  // Slotul 1 liber (verde)
      }

      if (slotsAvailability[1] == "-") {
        color(255, 0, 0, 2);  // Slotul 2 ocupat (roșu)
      } else {
        color(0, 255, 0, 2);  // Slotul 2 liber (verde)
      }

      if (slotsAvailability[2] == "-") {
        color(255, 0, 0, 3);  // Slotul 3 ocupat (roșu)
      } else {
        color(0, 255, 0, 3);  // Slotul 3 liber (verde)
      }

      if (slotsAvailability[3] == "-") {
        color(255, 0, 0, 4);  // Slotul 4 ocupat (roșu)
      } else {
        color(0, 255, 0, 4);  // Slotul 4 liber (verde)
      }
    }
    xSemaphoreGive(ledSemaphore);  // Eliberează semaforul
  }
}

// Setează culoarea LED-urilor RGB pentru fiecare slot
void color(unsigned char red, unsigned char green, unsigned char blue, int light) {
  if (light == 1) {
    analogWrite(r1, red);
    analogWrite(b1, blue);
    analogWrite(g1, green);
  } else if (light == 2) {
    analogWrite(r2, red);
    analogWrite(b2, blue);
    analogWrite(g2, green);
  } else if (light == 3) {
    analogWrite(r3, red);
    analogWrite(b3, blue);
    analogWrite(g3, green);
  } else if (light == 4) {
    analogWrite(r4, red);
    analogWrite(b4, blue);
    analogWrite(g4, green);
  }
}
