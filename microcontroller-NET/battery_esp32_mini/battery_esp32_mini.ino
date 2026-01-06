#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define PIN_C1 0
#define PIN_C2 1
#define PIN_C3 2

#define I2C_SDA 8
#define I2C_SCL 9

const float FACTOR = 4.0f;

float CAL1 = 0.903f;
float CAL2 = 0.889f;
float CAL3 = 0.905f;

const int NUM_SAMPLES = 20;
const int DELAY_US = 100;

float leerVoltajeADC(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(DELAY_US);
  }
  float raw = sum / (float)NUM_SAMPLES;
  return (raw / 4095.0f) * 3.3f;
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.display();

  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_C1, ADC_11db);
  analogSetPinAttenuation(PIN_C2, ADC_11db);
  analogSetPinAttenuation(PIN_C3, ADC_11db);
}

void loop() {
  float u1 = leerVoltajeADC(PIN_C1);
  float u2 = leerVoltajeADC(PIN_C2);
  float u3 = leerVoltajeADC(PIN_C3);

  float tap1 = u1 * FACTOR * CAL1;
  float tap2 = u2 * FACTOR * CAL2;
  float tap3 = u3 * FACTOR * CAL3;

  float cell1 = tap1;
  float cell2 = tap2 - tap1;
  float cell3 = tap3 - tap2;

  float total = tap3;

  float porcentaje = (total - 9.6f) / (12.6f - 9.6f);
  porcentaje = constrain(porcentaje, 0.0f, 1.0f) * 100.0f;

  const char *warningStr = "OK";
  if (cell1 < 3.3f || cell2 < 3.3f || cell3 < 3.3f) warningStr = "LOW";
  if (cell1 < 3.0f || cell2 < 3.0f || cell3 < 3.0f) warningStr = "DANGER";

  float diff = fabs(cell1 - cell3);

  Serial.printf("%.3f,%.3f,%.3f,%.3f,%.1f,%s,%.3f\n",
                cell1, cell2, cell3,
                total,
                porcentaje,
                warningStr,
                diff);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.printf("T1: %.2f  T2: %.2f\n", tap1, tap2);
  display.printf("T3: %.2f\n\n", tap3);

  display.printf("C1: %.2f  C2: %.2f\n", cell1, cell2);
  display.printf("C3: %.2f\n", cell3);

  display.printf("Tot: %.2fV  %4.1f%%\n", total, porcentaje);

  display.display();

  delay(300);
}
