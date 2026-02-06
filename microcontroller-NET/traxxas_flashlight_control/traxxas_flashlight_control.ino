#include <Arduino.h>
#include <Wire.h>

/* ================= I2C ================= */
#define SDA_PIN 5
#define SCL_PIN 6
#define I2C_SLAVE_ADDR 0x05

/* ================= RGB RIGHT ================= */
#define R_PIN_R 0
#define R_PIN_G 1
#define R_PIN_B 2

/* ================= RGB LEFT ================= */
#define L_PIN_R 7
#define L_PIN_G 8
#define L_PIN_B 9

#define BLINK_INTERVAL 30

/* ================= PWM ================= */
#define PWM_FREQ 5000
#define PWM_RES  8   // 0–255

// Canales PWM
#define CH_RR 0
#define CH_RG 1
#define CH_RB 2
#define CH_LR 3
#define CH_LG 4
#define CH_LB 5

/* ================= Buffer ================= */
#define STRING_BUFFER_SIZE 100
volatile char receivedString[STRING_BUFFER_SIZE];
volatile bool newData = false;

void receiveEvent(int howMany);

/* ================= Helpers ================= */
void setRightRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(CH_RR, r);
  ledcWrite(CH_RG, g);
  ledcWrite(CH_RB, b);
}

void setLeftRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(CH_LR, r);
  ledcWrite(CH_LG, g);
  ledcWrite(CH_LB, b);
}

void allOff() {
  setRightRGB(0, 0, 0);
  setLeftRGB(0, 0, 0);
}

/* ================= SETUP ================= */
void setup() {
  USBSerial.begin(115200);
  delay(2000);

  USBSerial.println("\n=== I2C Slave RGB LED ===");

  if (!Wire.setPins(SDA_PIN, SCL_PIN)) {
    USBSerial.println("ERROR I2C pins");
    while (1);
  }

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

  /* ===== PWM setup ===== */
  ledcSetup(CH_RR, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RG, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RB, PWM_FREQ, PWM_RES);
  ledcSetup(CH_LR, PWM_FREQ, PWM_RES);
  ledcSetup(CH_LG, PWM_FREQ, PWM_RES);
  ledcSetup(CH_LB, PWM_FREQ, PWM_RES);

  ledcAttachPin(R_PIN_R, CH_RR);
  ledcAttachPin(R_PIN_G, CH_RG);
  ledcAttachPin(R_PIN_B, CH_RB);
  ledcAttachPin(L_PIN_R, CH_LR);
  ledcAttachPin(L_PIN_G, CH_LG);
  ledcAttachPin(L_PIN_B, CH_LB);

  allOff();

  USBSerial.println("✓ RGB PWM listo");
}

/* ================= LOOP ================= */
void loop() {
  if (newData) {
    USBSerial.print("String recibido: ");
    USBSerial.println((char*)receivedString);

    char cmd = receivedString[0];

    switch (cmd) {
      case 'R':   // Direccional derecha
        setRightRGB(255, 100, 0);   // Ámbar
        setLeftRGB(0, 0, 0);
        delay(500);
        allOff();
        delay(30);
        break;

      case 'L':   // Direccional izquierda
        setLeftRGB(255, 100, 0);
        setRightRGB(0, 0, 0);
        delay(500);
        allOff();
        delay(30);
        break;

      case 'S':   // Intermitentes
        setRightRGB(255, 0, 0);
        setLeftRGB(255, 0, 0);
        break;

      case 'F':   // Apagar todo
        allOff();
        break;
    }

    newData = false;
  }

  delay(20);
}

/* ================= I2C CALLBACK ================= */
void receiveEvent(int howMany) {
  int i = 0;

  while (Wire.available() && i < STRING_BUFFER_SIZE - 1) {
    receivedString[i++] = Wire.read();
  }

  receivedString[i] = '\0';   // Null termination
  newData = true;

  // Limpiar bytes sobrantes
  while (Wire.available()) {
    Wire.read();
  }
}
