#include <AccelStepper.h>
#include <ESP32Servo.h>

// ====================================================
// KONFIGURACJA PINÓW I PARAMETRÓW
// ====================================================

// --- Serwo ---
Servo servo1;

// --- Piny silników ATD5833 ---
#define X_EN_PIN   26
#define X_STEP_PIN 25
#define X_DIR_PIN  17

#define Y_EN_PIN   16
#define Y_STEP_PIN 27
#define Y_DIR_PIN  14

// --- LED / Laser ---
#define LED_PIN 12

// --- Parametry silników ---
const int Steps_per_rev = 200 * 8;  // 1/8 mikrokrok
const int maxSpeed = 2000;
const int accel = 800;
const int deadzone = 20;

// --- Silniki ---
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// --- Stany systemu ---
bool motorXEnabled = false;
bool motorYEnabled = false;
bool autoAimMode = false;  

long posX = 0;
const long posLimit = Steps_per_rev / 2;

// --- Zmienne dla klawiatury (Watchdog) ---
unsigned long lastKeyboardCmdTime = 0;
bool keyboardActive = false;

// ====================================================
// FUNKCJA: Odbiór danych z Pythona (AI oraz Klawiatura)
// ====================================================
void processSerialTracking() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        data.trim(); // Usuwamy białe znaki z końca

        // ------------------------------------------------
        // 1. OBSŁUGA KLAWIATURY (K:...)
        // ------------------------------------------------
        if (data.startsWith("K:")) {
            
            // Włączanie / Wyłączanie osi X (K:TX)
            if (data.indexOf("TX") != -1) { 
                motorXEnabled = !motorXEnabled;
                digitalWrite(X_EN_PIN, motorXEnabled ? LOW : HIGH); // LOW zazwyczaj włącza sterownik
                Serial.println(motorXEnabled ? "ESP32: SILNIK X ON" : "ESP32: SILNIK X OFF");
                return;
            }
            // Włączanie / Wyłączanie osi Y (K:TY)
            else if (data.indexOf("TY") != -1) { 
                motorYEnabled = !motorYEnabled;
                digitalWrite(Y_EN_PIN, motorYEnabled ? LOW : HIGH);
                Serial.println(motorYEnabled ? "ESP32: SILNIK Y ON" : "ESP32: SILNIK Y OFF");
                return;
            }
            // Włączanie / Wyłączanie AI (K:A)
            else if (data.indexOf("A") != -1) {
                autoAimMode = !autoAimMode;
                Serial.println(autoAimMode ? "ESP32: TRYB AI ON" : "ESP32: TRYB AI OFF");
                stepperX.setSpeed(0);
                stepperY.setSpeed(0);
                return;
            }

            // Każda komenda ruchu z klawiatury wymusza wyłączenie AI dla bezpieczeństwa
            autoAimMode = false; 

            // Sterowanie jazdą ze strzałek
            if (data.indexOf("L") != -1) stepperX.setSpeed(maxSpeed / 1.5);
            else if (data.indexOf("R") != -1) stepperX.setSpeed(-maxSpeed / 1.5);
            else if (data.indexOf("U") != -1) stepperY.setSpeed(maxSpeed / 1.5);
            else if (data.indexOf("D") != -1) stepperY.setSpeed(-maxSpeed / 1.5);
            else if (data.indexOf("S") != -1) {  // Strzał (K:S)
                 servo1.write(0);
                 delay(150);
                 servo1.write(160);
                 Serial.println("ESP32: STRZAL Z SERWA");
            }
            
            // Zapisujemy czas ostatniego wciśnięcia strzałki (dla Watchdoga)
            if (data.indexOf("L") != -1 || data.indexOf("R") != -1 || data.indexOf("U") != -1 || data.indexOf("D") != -1) {
                keyboardActive = true;
                lastKeyboardCmdTime = millis(); 
            }
            return; // Kończymy, to była komenda z klawiatury
        }

        // ------------------------------------------------
        // 2. OBSŁUGA AUTO-AIM AI (X:...)
        // ------------------------------------------------
        if (data.startsWith("X:") && autoAimMode) {
            int commaIndex = data.indexOf(',');
            if (commaIndex > 0) {
                int targetX = data.substring(2, commaIndex).toInt();
                int targetY = data.substring(commaIndex + 3).toInt();

                // Błąd względem PIONOWEJ ROZDZIELCZOŚCI (240x320)
                int errorX = targetX - 120; 
                int errorY = targetY - 160;

                float autoSpeedX = 0;
                float autoSpeedY = 0;

                // Minimalna prędkość pokonująca tarcie fizyczne mechanizmu
                int minSpd = 600;  
                int maxSpd = maxSpeed / 1.5;

                // Obliczanie prędkości Oś X
                if (abs(errorX) > 15) { 
                    if (errorX > 0) autoSpeedX = map(errorX, 16, 120, -minSpd, -maxSpd); 
                    else autoSpeedX = map(errorX, -16, -120, minSpd, maxSpd);
                }
                
                // Obliczanie prędkości Oś Y
                if (abs(errorY) > 15) {
                    if (errorY > 0) autoSpeedY = map(errorY, 16, 160, -minSpd, -maxSpd);
                    else autoSpeedY = map(errorY, -16, -160, minSpd, maxSpd);
                }

                // Aplikowanie prędkości (tylko jeśli silniki są fizycznie włączone)
                if (motorXEnabled) {
                    if ((autoSpeedX > 0 && posX < posLimit) || (autoSpeedX < 0 && posX > -posLimit)) {
                        stepperX.setSpeed(autoSpeedX);
                        posX += (autoSpeedX > 0) ? 1 : -1;
                    } else {
                        stepperX.setSpeed(0);
                    }
                }
                if (motorYEnabled) {
                    stepperY.setSpeed(autoSpeedY);
                }
            }
        }
    }
}

// ====================================================
// SETUP / LOOP
// ====================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10); 

    pinMode(X_EN_PIN, OUTPUT);
    pinMode(Y_EN_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(X_EN_PIN, HIGH); // Domyślnie wyłączone zasilanie silników po starcie
    digitalWrite(Y_EN_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);

    stepperX.setMaxSpeed(maxSpeed);
    stepperX.setAcceleration(accel);
    stepperY.setMaxSpeed(maxSpeed);
    stepperY.setAcceleration(accel);

    servo1.attach(13);
    servo1.write(160); 

    Serial.println("ESP32: SYSTEM GOTOWY. STEROWANIE TYLKO PRZEZ USB/AI.");
}

void loop() {
    // 1. Odczyt komend z USB (Klawiatura i AI z Pythona)
    processSerialTracking();

    // 2. WATCHDOG KLAWIATURY (Bezpiecznik)
    // Jeśli włączyliśmy jazdę z klawiatury, ale od 150ms nie przyszła 
    // żadna nowa litera (bo puściliśmy klawisz), to zatrzymaj robota.
    if (keyboardActive && (millis() - lastKeyboardCmdTime > 150)) {
        stepperX.setSpeed(0);
        stepperY.setSpeed(0);
        keyboardActive = false;
    }

    // 3. Bezwarunkowe odświeżanie kroków silników (Kluczowe dla AccelStepper!)
    stepperX.runSpeed();
    stepperY.runSpeed();
}