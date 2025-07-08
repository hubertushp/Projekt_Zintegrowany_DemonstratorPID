// bibloteki i pliki nagłówkowe
#include "LittleFS.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

#include <WiFi.h>
#include <Wire.h>

// -- KONFIGURACJA ---

// --- Ustawienia Pinów ---
const int ECHO_PIN = 26; // echo pin
const int TRIG_PIN = 25; // Trigger pin
const int SERVO_PIN = 12; // Servo Pin

Servo myservo; // Initialize Servo.

// -- Wyświetlacz LCD I2C ---
const int LCD_ADDRESS = 0x27; // Adres I2C dla wyświetlacza LCD
const int LCD_COLS = 20;
const int LCD_ROWS = 4;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS); // Inicjalizacja LCD 20x4

// --- Konfiguracja klawiatury ---
const byte KEYPAD_ROWS = 4; // Liczba wierszy
const byte KEYPAD_COLS = 4; // Liczba kolumn
// Definicja pinów dla klawiatury
const char hexaKeys[KEYPAD_ROWS][KEYPAD_COLS] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '*', '0', '#', 'D' }
};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

byte rowPins[KEYPAD_ROWS] = { 19, 18, 5, 17 }; // Piny wierszy
byte colPins[KEYPAD_COLS] = { 2, 0, 4, 16 }; // Piny kolumn - tu musi być rosnąco

// --- Stałe PID w postaci standardowej (z stałymi czasowymi) ---
// --- Wartości startowe PID ---
float kp = 0.75; // Wzmocnienie proporcjonalne (bez zmian)
float Ti = 1.5; // Czas zdwojenia (całkowania) w sekundach.
float Td = 0.25; // Czas wyprzedzenia (różniczkowania) w sekundach.
float distance_setpoint = 280.0; // Wartość zadana odległości w mm.

float PID_p, PID_i = 0.0, PID_d, PID_total; // PID_i musi być inicjalizowane

// --- Ustawienia serwa ---
float servo_angle_min = 40; // Minimalny kąt serwa (bezpieczny zakres)
float servo_angle_mid = 90; // Kąt serwa, gdy piłka jest na setpoincie
float servo_angle_max = 140; // Maksymalny kąt serwa (bezpieczny zakres)
float pid_output_min_expected = -200.0; // Oczekiwany minimalny output PID
float pid_output_max_expected = 200.0; // Oczekiwany maksymalny output PID

// --- Ustawienia Access Point --- do połączenia z ESP32 jako AP i wyświetlenie wykresów w przeglądarce
const char* ssid_ap = "DemonstratorPID"; // Ustaw nazwę SSID swojego AP
const char* password_ap = "123456789"; // Ustaw hasło (min. 8 znaków) lub NULL dla otwartej sieci

// --- Obiekty serwera ---
AsyncWebServer server(80); // Serwer na porcie 80
AsyncWebSocket ws("/ws"); // WebSocket na ścieżce /ws

// --- Zmienne do obsługi WebSocket ---
static unsigned long lastWsSendTime = 0;
unsigned long wsSendInterval = 500; // Wysyłaj dane co 500 ms
// --- Zmienne do obsługi LCD ---
unsigned long previousLCDMillis = 0; // Zmienna do pomiaru czasu
const long lcdUpdateInterval = 1500; // Czas odświeżania LCD w ms

// --- Zmienna do przechowywania stanu edycji parametrów PID ---
enum EditingParameter {
    PARAM_NONE, // Dodajemy stan, gdy nic nie jest wybrane do edycji
    PARAM_KP,
    PARAM_TI,
    PARAM_TD,
    PARAM_SETPOINT
};

EditingParameter currentEditingParam = PARAM_NONE;
String currentEditingParamName = ""; // Nazwa edytowanego parametru do wyświetlenia

char numericInputBuffer[12]; // Bufor na wprowadzane cyfry (np. "-123.45" + null)
byte bufferIndex = 0;

// Zmienna pomocnicza do śledzenia, czy użytkownik jest w trakcie wprowadzania
// wartości (po wybraniu parametru, ale przed wciśnięciem cyfry)
bool isAwaitingInputAfterParamSelect = false;

volatile unsigned long echoStartTime_us = 0;
volatile unsigned long echoDuration_us = 0;
volatile bool newDistanceDataAvailable = false;
int echoPinForISR_global;

// --- Zmienne do zarządzania pomiarami i uśrednianiem ---
const int NUM_SAMPLES_FOR_AVERAGE = 15; // Ile próbek do uśrednienia
float distanceSamples[NUM_SAMPLES_FOR_AVERAGE];
int currentSampleIndex = 0;
unsigned long lastMeasurementTriggerTime = 0;
// Kiedy ostatnio wyzwolono pomiar Odstęp między pomiarami - powinien być
// nieco dłuższy niż maksymalny czas echa Max dystans HC-SR04 to ~4m.
// Czas echa: (4000mm * 2) / 0.343 mm/us = ~23323 us = ~24 ms. Dajmy
// margines na przetwarzanie.
const unsigned long MEASUREMENT_INTERVAL_MS = 10;

float averaged_distance_mm = 0.0; // Uśredniona odległość w mm, używana przez PID

// --- Zmienne dla pętli i logiki PID ---
unsigned long previousPIDMillis = 0; // Czas ostatniego wykonania pętli PID
float distance_previous_error = 0.0;
float distance_error = 0.0;
int pid_period = 10; // Okres pętli PID w ms (np. 20Hz)

// -- FUNCJE I PROCEDURY ---

// --- Procedura Obsługi Przerwania (ISR) dla HC-SR04 ---
// IRAM_ATTR umieszcza funkcję w szybkiej pamięci IRAM, zalecane dla ISR na
// ESP32
void IRAM_ATTR handleEchoInterrupt()
{
    if (digitalRead(echoPinForISR_global) == HIGH) { // Zbocze narastające - początek echa
        echoStartTime_us = micros();
        newDistanceDataAvailable = false; // Resetuj flagę na początku nowego echa
    } else { // Zbocze opadające - koniec echa
        if (echoStartTime_us > 0) { // Upewnij się, że echoStartTime_us zostało ustawione
            echoDuration_us = micros() - echoStartTime_us;
            newDistanceDataAvailable = true; // Ustaw flagę - nowe dane są gotowe
            echoStartTime_us = 0; // Zresetuj na wypadek "luźnych" zboczy
        }
    }
}
// --- Funkcje pomocnicze dla HC-SR04 ---
// --- Funkcja do konfiguracji HC-SR04 i przerwań ---
void setup_hc_sr04_interrupts()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW); // Upewnij się, że trigger jest na początku niski
    echoPinForISR_global = ECHO_PIN;
    // Użyj digitalPinToInterrupt() aby uzyskać właściwy numer przerwania dla pinu
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), handleEchoInterrupt, CHANGE);
}
// --- Funkcja do wyzwalania impulsu HC-SR04 ---
void trigger_hc_sr04_pulse()
{
    // Resetuj flagi i czasy przed nowym pomiarem
    newDistanceDataAvailable = false;
    echoStartTime_us = 0;
    echoDuration_us = 0;

    unsigned long startTime;

    // Wyślij impuls wyzwalający
    digitalWrite(TRIG_PIN, LOW);
    startTime = micros();

    // Aktywne czekanie na 2 mikrosekundy
    // Ta pętla będzie się wykonywać bardzo niewiele razy na ESP32
    while (micros() - startTime < 2)
        digitalWrite(TRIG_PIN, HIGH);

    startTime = micros();

    // Aktywne czekanie na 10 mikrosekund
    while (micros() - startTime < 10)
        digitalWrite(TRIG_PIN, LOW);
}
// --- Główna funkcja aktualizująca pomiar odległości (wywoływana w loop) ---
void update_distance_reading()
{
    // 1. Okresowo wyzwalaj nowy pomiar HC-SR04
    if (millis() - lastMeasurementTriggerTime >= MEASUREMENT_INTERVAL_MS) {
        trigger_hc_sr04_pulse();
        lastMeasurementTriggerTime = millis();
    }

    // 2. Sprawdź, czy ISR zarejestrował nowe dane
    if (newDistanceDataAvailable) {
        newDistanceDataAvailable = false; // Skonsumuj flagę, aby nie przetwarzać
            // tych samych danych wielokrotnie
        unsigned long duration = echoDuration_us; // Skopiuj wartość, bo ISR może ją zmienić

        // Sprawdź, czy czas trwania jest sensowny (unikaj błędów i timeoutów)
        // Max czas dla HC-SR04 (np. 4m) to ok. 23323 µs. Min czas dla 2cm to ok.
        // 116 µs.
        if (duration > 100 && duration < 25000) {
            float current_distance_mm = (duration / 2.0) * 0.343; // Przelicz czas na mm
            if (current_distance_mm > 580) {
                current_distance_mm = 580; // Ustaw maksymalną wartość, jeśli pomiar jest poza zakresem
            }

            // Dodaj nowy odczyt do bufora uśredniającego
            distanceSamples[currentSampleIndex] = current_distance_mm;
            currentSampleIndex = (currentSampleIndex + 1) % NUM_SAMPLES_FOR_AVERAGE;

            // Oblicz nową średnią kroczącą
            float sum = 0;
            for (int i = 0; i < NUM_SAMPLES_FOR_AVERAGE; i++) {
                sum += distanceSamples[i];
            }
            averaged_distance_mm = sum / NUM_SAMPLES_FOR_AVERAGE;

            // Debug:
            // Serial.print("Raw dist: "); Serial.print(current_distance_mm);
            // Serial.print(" Avg dist: "); Serial.println(averaged_distance_mm);
        }
    }
}

// --- Funkcja do aktualizacji wyświetlacza LCD ---
void updateDisplay()
{
    char buffer[LCD_COLS + 1];

    // Linia 0: Kp, Ki, Kd
    lcd.setCursor(0, 0);
    if (currentEditingParam == PARAM_KP) {
        snprintf(buffer, sizeof(buffer), ">Kp:%s", numericInputBuffer);
    } else {
        snprintf(buffer, sizeof(buffer), "P:%.2f", kp);
    }
    lcd.print(buffer);
    for (int i = strlen(buffer); i < 9; i++) {
        lcd.print(" ");
    } // Wyczyść do następnego pola

    lcd.setCursor(7, 0);
    if (currentEditingParam == PARAM_TI) {
        snprintf(buffer, sizeof(buffer), ">Ti:%s", numericInputBuffer);
    } else {
        snprintf(buffer, sizeof(buffer), "I:%.2f", Ti);
    }
    lcd.print(buffer);
    for (int i = 7 + strlen(buffer); i < LCD_COLS; i++) {
        lcd.print(" ");
    }

    lcd.setCursor(14, 0);
    if (currentEditingParam == PARAM_TD) {
        snprintf(buffer, sizeof(buffer), ">Td:%s", numericInputBuffer);
    } else {
        snprintf(buffer, sizeof(buffer), "D:%.2f", Td);
    }
    lcd.print(buffer);
    for (int i = strlen(buffer); i < 9; i++) {
        lcd.print(" ");
    }
    // Linia
    lcd.setCursor(0, 1);
    if (currentEditingParam == PARAM_SETPOINT) {
        snprintf(buffer, sizeof(buffer), ">Set:%s", numericInputBuffer);
    } else {
        snprintf(buffer, sizeof(buffer), "Set:%.1f", distance_setpoint);
    }
    lcd.print(buffer);
    for (int i = strlen(buffer); i < 9; i++) {
        lcd.print(" ");
    }

    // Linia 2: Aktualna odległość i Błąd
    lcd.setCursor(0, 2);
    snprintf(buffer, sizeof(buffer), "Dist:%.1f Err:%.1f", averaged_distance_mm, distance_error);
    lcd.print(buffer);
    for (int i = strlen(buffer); i < LCD_COLS; i++) {
        lcd.print(" ");
    }

    // Linia 3: Kąt serwa i wyjście PID
    lcd.setCursor(0, 3);
    snprintf(buffer, sizeof(buffer), "Ang:%.0f PIDtot:%.1f", (float)myservo.read(), PID_total);
    lcd.print(buffer);
    for (int i = strlen(buffer); i < LCD_COLS; i++) {
        lcd.print(" ");
    }

    // Jeśli edytujemy i bufor jest pusty, można wyświetlić podpowiedź
    if (currentEditingParam != PARAM_NONE && bufferIndex == 0) {
        lcd.setCursor(0, currentEditingParam == PARAM_KP || currentEditingParam == PARAM_TI || currentEditingParam == PARAM_TD ? 0 : 1); // Ustaw kursor na linii edytowanego parametru
        // Znajdź pozycję za nazwą parametru
        int colOffset = 0;
        if (currentEditingParam == PARAM_KP)
            colOffset = 3;
        else if (currentEditingParam == PARAM_TI)
            colOffset = 10;
        else if (currentEditingParam == PARAM_TD)
            colOffset = 17;
        else if (currentEditingParam == PARAM_SETPOINT)
            colOffset = 5;

        lcd.setCursor(colOffset, (currentEditingParam == PARAM_KP || currentEditingParam == PARAM_TI || currentEditingParam == PARAM_TD) ? 0 : 1);
        lcd.print("_"); // Wskaźnik, że można pisać
    }
}

void startEditing(EditingParameter param, const String& paramName)
{
    currentEditingParam = param;
    currentEditingParamName = paramName;
    isAwaitingInputAfterParamSelect = true; // Czekamy na pierwszą cyfrę
    bufferIndex = 0; // Wyczyść bufor
    numericInputBuffer[0] = '\0'; // Upewnij się, że bufor jest pusty
    updateDisplay();
}

// Funkcja appendToBuffer (z obsługą kropki, jeśli klawiatura ją ma lub jest
// emulowana)
void appendToBuffer(char key)
{
    if (bufferIndex < sizeof(numericInputBuffer) - 1) {
        // Prosta obsługa kropki - pozwól tylko na jedną
        if (key == '.' && String(numericInputBuffer).indexOf('.') != -1) {
            return; // Już jest kropka, nie dodawaj kolejnej
        }
        numericInputBuffer[bufferIndex++] = key;
        numericInputBuffer[bufferIndex] = '\0';
        updateDisplay(); // Zaktualizuj LCD z nową wartością w buforze
    }
}

// Funkcja applyParameterChange -- wywoływana po wciśnięciu '#' -- zatwierdza wpisane nastawy
void applyParameterChange()
{
    if (bufferIndex == 0 && currentEditingParam != PARAM_NONE) { // Jeśli bufor jest pusty, nic nie zmieniaj
        currentEditingParam = PARAM_NONE;
        updateDisplay();
        return;
    }
    if (currentEditingParam != PARAM_NONE) {
        float newValue = atof(numericInputBuffer); // Konwertuj string na float
        bool changed = false;

        switch (currentEditingParam) {
        case PARAM_KP:
            newValue /= 100.0;
            if (kp != newValue) {
                kp = newValue;
                changed = true;
            }
            break;
        case PARAM_TI:
            newValue /= 100.0;
            if (Ti != newValue) {
                Ti = newValue;
                PID_i = 0;
                changed = true;
            } // Resetuj składową I przy zmianie Ki
            break;
        case PARAM_TD:
            newValue /= 100.0;
            if (Td != newValue) {
                Td = newValue;
                changed = true;
            }
            break;
        case PARAM_SETPOINT:
            newValue = constrain(newValue, 0.0, 600.0); // Ogranicz setpoint do sensownego zakresu
            if (distance_setpoint != newValue) {
                distance_setpoint = newValue;
                PID_i = 0;
                changed = true;
            } // Reset I przy zmianie setpointu
            break;
        default:
            break;
        }
        currentEditingParam = PARAM_NONE;
        numericInputBuffer[0] = '\0';
        bufferIndex = 0;
        updateDisplay(); // Zaktualizuj LCD po zmianie
    }
}

// Funkcja handleBackspace (wywoływana przez '*' gdy currentEditingParam !=
// PARAM_NONE)
void handleBackspace()
{
    if (bufferIndex > 0) {
        bufferIndex--;
        numericInputBuffer[bufferIndex] = '\0';
        updateDisplay(); // Zaktualizuj LCD
    } else { // Jeśli bufor jest pusty, a chcemy cofnąć, to anuluj edycję
        currentEditingParam = PARAM_NONE;
        numericInputBuffer[0] = '\0';
        bufferIndex = 0;
        updateDisplay();
    }
}

void processKeypadInput()
{
    char key = customKeypad.getKey();

    if (key) { // Jeśli wciśnięto jakiś klawisz
        Serial.print("Klawisz: ");
        Serial.println(key);

        if (currentEditingParam == PARAM_NONE) { // Jeśli nie jesteśmy w trybie edycji
            switch (key) {
            case 'A':
                startEditing(PARAM_KP, "Kp");
                break;
            case 'B':
                startEditing(PARAM_TI, "Ki");
                break;
            case 'C':
                startEditing(PARAM_TD, "Kd");
                break;
            case 'D':
                startEditing(PARAM_SETPOINT, "SetP");
                break;
            default:
                break; // Ignoruj inne klawisze, gdy nie edytujemy
            }
        } else { // Jesteśmy w trybie edycji
            if ((key >= '0' && key <= '9') || key == '.') {
                appendToBuffer(key);
            } else if (key == '#') { // Enter/Zatwierdź
                applyParameterChange();
            } else if (key == '*') { // Backspace lub Anuluj
                handleBackspace();
            }
            // Można dodać obsługę A,B,C,D aby anulować bieżącą edycję i zacząć nową
            // np. jeśli wciśnięto 'A' podczas edycji Ki, anuluj Ki i zacznij edytować Kp.
            else if (key == 'A') {
                applyParameterChange();
                startEditing(PARAM_KP, "Kp");
            } else if (key == 'B') {
                applyParameterChange();
                startEditing(PARAM_TI, "Ki");
            } else if (key == 'C') {
                applyParameterChange();
                startEditing(PARAM_TD, "Kd");
            } else if (key == 'D') {
                applyParameterChange();
                startEditing(PARAM_SETPOINT, "SetP");
            }
        }
    }
}
// --- Obsługa połączenia WebSocket ---
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len)
{
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // Możesz wysłać początkowe dane lub komunikat powitalny
        // client->text("Connected to ESP32 BallBeam");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        // Na razie nie oczekujemy danych od klienta, ale można tu dodać obsługę
        Serial.printf("WebSocket data from client #%u: %s\n", client->id(), (char*)data);
    } else if (type == WS_EVT_ERROR) {
        Serial.printf("WebSocket client #%u error #%u: %s\n", client->id(), *((uint16_t*)arg), (char*)data);
    }
}
// --- Funkcja setup() ---
// Inicjalizacja ESP32, serwera, LCD, WebSocket i HC-SR04
void setup()
{
    Serial.begin(9600);
    Wire.begin(); // Inicjalizacja I2C dla LCD

    // --- Inicjalizacja LittleFS ---
    if (!LittleFS.begin(true)) { // true = formatuj jeśli nie można zamontować
        Serial.println("An Error has occurred while mounting LittleFS");
        // Możesz dodać jakąś obsługę błędu, np. restart ESP
        return; // lub ESP.restart();
    }
    Serial.println("LittleFS mounted successfully");

    // List directory (opcjonalne, do debugowania)
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.print("FILE: ");
        Serial.println(file.name());
        file.close();
        file = root.openNextFile();
    }
    // --- Koniec inicjalizacji LittleFS ---

    Serial.println("Konfiguracja Access Point...");
    WiFi.softAP(ssid_ap, password_ap);
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(apIP);

    myservo.attach(SERVO_PIN);
    myservo.write(servo_angle_mid); // Ustaw serwo w pozycji środkowej na start

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // --- Serwowanie plików ze SPIFFS ---
    // Serwuj plik index.html dla żądania do roota "/"
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) { request->send(LittleFS, "/index.html", "text/html"); });

    server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/chart.js", "application/javascript");
    });

    server.serveStatic("/", LittleFS, "/");

    server.onNotFound([](AsyncWebServerRequest* request) {
        if (request->method() == HTTP_OPTIONS) {
            request->send(200);
        } else {
            request->send(404, "text/plain", "Strona nie znaleziona");
        }
    });

    server.begin();
    Serial.println("Serwer HTTP uruchomiony");

    setup_hc_sr04_interrupts(); // Skonfiguruj HC-SR04 i przerwania

    // Wypełnij bufor próbek wartością początkową (np. setpoint lub środek zakresu
    // czujnika) aby uniknąć zer lub dziwnych wartości na starcie w średniej.
    for (int i = 0; i < NUM_SAMPLES_FOR_AVERAGE; ++i) {
        distanceSamples[i] = distance_setpoint;
    }
    averaged_distance_mm = distance_setpoint; // Ustaw początkową średnią

    PID_i = 0.0; // Zawsze inicjuj składową całkową
    distance_previous_error = 0.0; // Inicjuj poprzedni błąd
    previousPIDMillis = millis(); // Inicjalizacja timera pętli PID
    Serial.println("System gotowy. Rozpoczynanie pętli PID...");

    // Inicjalizacja LCD
    lcd.init(); // Inicjalizacja LCD
    lcd.backlight(); // Włącz podświetlenie LCD
    lcd.clear(); // Wyczyść ekran LCD
    lcd.setCursor(0, 0); // Ustaw kursor na początek
    lcd.print("BALL & BEAM PID REG.");
    lcd.setCursor(0, 1); // Ustaw kursor na drugą linię
    lcd.print("URUCHAM SIE...");
    delay(2000); // Poczekaj chwilę, aby użytkownik mógł zobaczyć komunikat
    lcd.clear(); // Wyczyść ekran LCD

    // Wysyłanie danych przez WebSocket do wszystkich podłączonych klientów
    // Robimy to np. co sekundę lub rzadziej, aby nie przeciążać
    // komunikacji/wykresu

    if (millis() - lastWsSendTime > wsSendInterval) {
        if (ws.count() > 0) { // Jeśli są jacyś podłączeni klienci WebSocket
            // Przekształć float na string
            char buffer[10];
            dtostrf(averaged_distance_mm, 4, 1,
                buffer); // 4 znaki szerokości, 1 po przecinku
            ws.textAll(buffer); // Wyślij aktualną uśrednioną odległość
        }
        lastWsSendTime = millis();
    }

    ws.cleanupClients(); // Ważne do usuwania nieaktywnych klientów
}

// --- Główna pętla programu ---
// W tej pętli będziemy odczytywać klawiaturę, aktualizować pomiary i wykonywać
// logikę PID
void loop()
{
    processKeypadInput(); // Odczytaj klawiaturę i zaktualizuj stany edycji

    // Krok 1: Zaktualizuj odczyt odległości (nieblokująco)
    update_distance_reading();

    // Krok 2: Wykonaj logikę PID w regularnych odstępach czasu (`pid_period`)
    if (millis() - previousPIDMillis >= pid_period) {
        previousPIDMillis = millis(); // Zresetuj timer pętli PID

        // Aktualna odległość dla PID to nasza uśredniona wartość
        float current_measured_distance = averaged_distance_mm;
        distance_error = distance_setpoint - current_measured_distance;

        if (abs(distance_error) < 5) { // Jeśli błąd jest bardzo mały, zresetuj PID_i
            PID_i = 0.0; // Reset całki, aby uniknąć narastania błędu
            PID_d = 0.0; // Reset składowej różniczkującej

            distance_previous_error = distance_error; // Zresetuj poprzedni błąd
        } else {
            // Składowa Proporcjonalna
            PID_p = kp * distance_error;

            // Składowa Różniczkująca (Derivative)
            PID_d = kp * Td * (distance_error - distance_previous_error) / ((float)pid_period / 1000.0);

            // Składowa Całkująca (Integral) z Anti-Windup
            // Całkujemy tylko, gdy system jest blisko setpointu, aby uniknąć
            // nadmiernego narastania całki. Np. jeśli błąd jest mniejszy niż 20%
            // setpointu (lub stała wartość w mm)
            if (abs(distance_error) < (distance_setpoint * 0.9)) { // Przykładowy próg, dostosuj!
                if (Ti > 1e-6) { // Sprawdzenie, czy Ti nie jest zerem
                    PID_i += (kp / Ti) * distance_error * ((float)pid_period / 1000.0); // Całkujemy przez czas w sekundach
                } else {
                    PID_i = 0; // Jeśli Ti jest zerem, resetujemy całkę
                }

            } else {
                // Poza strefą całkowania - można powoli redukować całkę lub ją zatrzymać
                PID_i *= 0.99; // Powolne wygaszanie całki
                // Lub po prostu nie zmieniać: PID_i = PID_i;
            }
        }

        // Dodatkowe ograniczenie dla składowej całkującej (ważny anti-windup)
        // Te wartości powinny być związane z maksymalnym oczekiwanym wkładem od I.
        float max_integral_contribution = 50.0; // PRZYKŁAD - dostosuj
        PID_i = constrain(PID_i, -max_integral_contribution, max_integral_contribution);

        // Suma sygnałów PID
        PID_total = PID_p + PID_i + PID_d;

        // Mapowanie sygnału PID na kąt serwa
        // Chcemy, aby PID_total = 0 dało servo_angle_mid.
        // Maksymalne/minimalne odchylenie PID daje maksymalne/minimalne odchylenie
        // kąta.
        float angle_offset = map(
            PID_total, pid_output_min_expected, pid_output_max_expected, servo_angle_min - servo_angle_mid, servo_angle_max - servo_angle_mid);

        float target_servo_angle = servo_angle_mid + angle_offset;

        // Ostateczne ograniczenie kąta serwa do bezpiecznego zakresu
        target_servo_angle = constrain(target_servo_angle, servo_angle_min, servo_angle_max);

        myservo.write(target_servo_angle);

        // Zapisz aktualny błąd jako poprzedni dla następnej iteracji
        distance_previous_error = distance_error;

        // Debugowanie wartości PID (opcjonalne)

        //Serial.print("Dist: ");
        //Serial.print(current_measured_distance, 1);
        // Serial.print(" Err: ");
        // Serial.print(distance_error, 1);
        // Serial.print(" P: "); Serial.print(PID_p, 1);
        // Serial.print(" I: "); Serial.print(PID_i, 1);
        // Serial.print(" D: "); Serial.print(PID_d, 1);
        // Serial.print(" Total: ");
        // Serial.print(PID_total, 1);
        //Serial.print(" Angle: ");
        //Serial.println(target_servo_angle, 0);
    }

    // Okresowo aktualizuj wyświetlacz LCD
    if (millis() - previousLCDMillis >= lcdUpdateInterval) {
        previousLCDMillis = millis(); // Zresetuj timer aktualizacji LCD
        updateDisplay();
    }

    // Okresowo wysyłaj dane przez WebSocket do wszystkich klientów
    // Jeśli nie ma klientów, to nie wysyłaj nic
    // Jeśli są klienci, to wysyłaj co wsSendInterval ms
    if (millis() - lastWsSendTime > wsSendInterval) {
        if (ws.count() > 0) {
            char buffer[10];
            dtostrf(averaged_distance_mm, 4, 1, buffer); // Konwertuj float na string
            ws.textAll(buffer); // Wyślij do wszystkich klientów
        }
        lastWsSendTime = millis();
    }
    ws.cleanupClients();
}
