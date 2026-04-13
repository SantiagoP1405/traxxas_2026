#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- CONFIGURACIÓN PANTALLA OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- PINES ADC BATERÍA (ESP32) ---
#define PIN_C1 34
#define PIN_C2 35
#define PIN_C3 32

// --- PINES ULTRASÓNICOS (ESP32) ---
const int TRIG_L = 26; const int ECHO_L = 25;
const int TRIG_C = 13; const int ECHO_C = 19;
const int TRIG_R = 14; const int ECHO_R = 27;

// --- PIN INFRARROJO ---
const int PIN_IR = 33; 

// --- VARIABLES BATERÍA ---
const float FACTOR = 4.0f;
float CAL1 = 0.9457f; 
float CAL2 = 1.0437f; 
float CAL3 = 1.0973f;
const int NUM_SAMPLES = 20;
const int DELAY_US = 100;

// --- TIMERS PARA MULTITAREA ---
unsigned long lastSensorTime = 0;
unsigned long lastBatteryTime = 0;

// ==============================================================
// FUNCIÓN PARA CORREGIR LA NO LINEALIDAD DEL ADC (y = mx + b)
// ==============================================================
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Función para leer batería con filtro
float leerVoltajeADC(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(DELAY_US);
  }
  float raw = sum / (float)NUM_SAMPLES;
  return (raw / 4095.0f) * 3.3f;
}

// Función para leer ultrasonicos
long leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  long duracion = pulseIn(echo, HIGH, 25000); // Timeout 25ms
  if (duracion == 0) return 500; // Si falla, decimos que esta lejos
  return duracion * 0.034 / 2;
}

void setup() {
  Serial.begin(115200); 

  // --- SETUP BATERÍA ---
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_C1, ADC_11db);
  analogSetPinAttenuation(PIN_C2, ADC_11db);
  analogSetPinAttenuation(PIN_C3, ADC_11db);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Fallo al iniciar SSD1306"));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();
  }

  // --- SETUP SENSORES ---
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(PIN_IR, INPUT); 
  
  delay(200);
}

void loop() {
  unsigned long currentMillis = millis();

  // ==============================================================
  // TAREA 1: LECTURA DE SENSORES PARA ROS (Alta velocidad ~30Hz)
  // ==============================================================
  if (currentMillis - lastSensorTime >= 30) {
    lastSensorTime = currentMillis;

    long distL = leerDistancia(TRIG_L, ECHO_L);
    delay(2); // Pequeña pausa para evitar eco cruzado
    long distC = leerDistancia(TRIG_C, ECHO_C);
    delay(2);
    long distR = leerDistancia(TRIG_R, ECHO_R);
    
    int estadoIR = digitalRead(PIN_IR);
    
    // Formato exacto que espera tu nodo de Python (4 valores)
    Serial.print(distL);
    Serial.print(",");
    Serial.print(distC);
    Serial.print(",");
    Serial.print(distR);
    Serial.print(",");
    Serial.println(estadoIR);
  }

  // ==============================================================
  // TAREA 2: LECTURA BATERÍA Y PANTALLA OLED (Baja velocidad ~2Hz)
  // ==============================================================
  if (currentMillis - lastBatteryTime >= 500) {
    lastBatteryTime = currentMillis;

    float u1 = leerVoltajeADC(PIN_C1);
    float u2 = leerVoltajeADC(PIN_C2);
    float u3 = leerVoltajeADC(PIN_C3);

    // --- Convertir a taps crudos con la calibración original ---
    float tap1_crudo = u1 * FACTOR * CAL1;  // T1 es el Total (~12.20V)
    float tap2_crudo = u2 * FACTOR * CAL2;  // T2 es C1 + C2 (~8.12V)
    float tap3_crudo = u3 * FACTOR * CAL3;  // T3 es solo C1 (~4.06V)

    // --- Aplicar la corrección de la recta (Interpolación) ---
    // Sintaxis: mapFloat(valor_crudo, lectura_esp_baja, lectura_esp_alta, real_multimetro_bajo, real_multimetro_alto)
    float tap1 = mapFloat(tap1_crudo, 10.96f, 12.20f, 11.40f, 12.20f);
    
    // Corrección con tus nuevos datos reales (T2 y T3)
    float tap2 = mapFloat(tap2_crudo, 7.65f, 8.12f, 7.60f, 8.12f);
    float tap3 = mapFloat(tap3_crudo, 3.85f, 4.06f, 3.80f, 4.06f);

    // --- Cálculo real de celdas (¡Adiós negativos!) ---
    float cell1 = tap3;             // C1 directa
    float cell2 = tap2 - tap3;      // C2 = (C1+C2) - C1
    float cell3 = tap1 - tap2;      // C3 = Total - (C1+C2)
    float total = tap1;             // El total es directo T1

    float porcentaje = (total - 9.6f) / (12.6f - 9.6f);
    porcentaje = constrain(porcentaje, 0.0f, 1.0f) * 100.0f;

    const char *warningStr = "OK";
    if (cell1 < 3.3f || cell2 < 3.3f || cell3 < 3.3f) warningStr = "LOW";
    if (cell1 < 3.0f || cell2 < 3.0f || cell3 < 3.0f) warningStr = "DANGER";

    float diff = fabs(cell1 - cell3);

    // CSV de batería (7 valores, Python lo ignorará sin problemas)
    Serial.printf("%.3f,%.3f,%.3f,%.3f,%.1f,%s,%.3f\n",
                  cell1, cell2, cell3, total, porcentaje, warningStr, diff);

    // Actualizar Pantalla OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);

    display.printf("T1: %.2f  T2: %.2f\n", tap1, tap2);
    display.printf("T3: %.2f\n\n", tap3);
    display.printf("C1: %.2f  C2: %.2f\n", cell1, cell2);
    display.printf("C3: %.2f\n", cell3);
    display.printf("Tot: %.2fV  %4.1f%%\n", total, porcentaje);

    display.display();
  }
}
