#include <SPI.h>
#include <Wire.h>
#include "BNO055_support.h"

// ===================== Pines ESP32-C3 =====================
#define PIN_SCK    8
#define PIN_MISO   9
#define PIN_MOSI  10
#define PIN_CS     5

#define I2C_SDA    6
#define I2C_SCL    7

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

// ===================== AS5048A =====================
static const uint16_t REG_ANGLE = 0x3FFF;
static const int32_t CPR = 16384;
static const int32_t WRAP_TH = CPR / 2;

// ===================== Parametros =====================
static const unsigned long PRINT_PERIOD_MS = 20;
static const unsigned long OMEGA_WINDOW_MS = 100;
static const int WARMUP_SAMPLES = 15;
static const int DELTA_DEADBAND_COUNTS = 1;
static const float OMEGA_ALPHA = 0.18f;
static const float OMEGA_MAX_RAD_S = 200.0f;

// ===================== IMU =====================
struct bno055_t myBNO;
struct bno055_euler myEuler;
struct bno055_quaternion myQuat;
struct bno055_gyro myGyro;
struct bno055_linear_accel myAccel;

// ===================== Estado encoder =====================
uint16_t lastCnt = 0;
int32_t accumDelta = 0;
float omegaRaw = 0.0f;
float omegaFilt = 0.0f;

unsigned long lastPrint = 0;
unsigned long omegaWindowStart = 0;
int warmupLeft = WARMUP_SAMPLES;

// ===================== Utilidades =====================
static uint8_t parity16(uint16_t v) {
  uint8_t c = 0;
  for (int i = 0; i < 16; i++) {
    c += (v & 1);
    v >>= 1;
  }
  return c & 1;
}

static uint16_t xfer16(uint16_t tx) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
  uint16_t rx = SPI.transfer16(tx);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return rx;
}

static uint16_t readAngle() {
  uint16_t cmd = 0x4000 | REG_ANGLE;
  cmd |= (uint16_t)(parity16(cmd) << 15);
  xfer16(cmd);
  uint16_t rx = xfer16(0x0000);
  return rx & 0x3FFF;
}

static float clampFloat(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(600);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  Wire.begin(I2C_SDA, I2C_SCL);
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(100);

  lastCnt = readAngle();
  omegaWindowStart = millis();

  Serial.println("# t,omega,yaw,pitch,roll,qw,qx,qy,qz,gx,gy,gz,ax,ay,az");
}

// ===================== Loop =====================
void loop() {
  unsigned long now = millis();

  // ---------------- Encoder ----------------
  uint16_t current = readAngle();
  int32_t delta = (int32_t)current - (int32_t)lastCnt;

  if (delta > WRAP_TH)  delta -= CPR;
  if (delta < -WRAP_TH) delta += CPR;

  if (abs(delta) <= DELTA_DEADBAND_COUNTS) {
    delta = 0;
  }

  lastCnt = current;

  if (warmupLeft > 0) {
    warmupLeft--;
    accumDelta = 0;
    omegaRaw = 0.0f;
    omegaFilt = 0.0f;
    omegaWindowStart = now;
  } else {
    accumDelta += delta;

    float dtWindow = (now - omegaWindowStart) / 1000.0f;
    if (dtWindow >= (OMEGA_WINDOW_MS / 1000.0f)) {
      omegaRaw = ((float)accumDelta / CPR) * (2.0f * PI) / dtWindow;
      omegaRaw = clampFloat(omegaRaw, -OMEGA_MAX_RAD_S, OMEGA_MAX_RAD_S);

      omegaFilt = (1.0f - OMEGA_ALPHA) * omegaFilt + OMEGA_ALPHA * omegaRaw;

      accumDelta = 0;
      omegaWindowStart = now;
    }
  }

  // ---------------- IMU ----------------
  bno055_read_euler_hrp(&myEuler);
  bno055_read_quaternion_wxyz(&myQuat);
  bno055_read_gyro_xyz(&myGyro);
  bno055_read_linear_accel_xyz(&myAccel);

  float yaw   = myEuler.h / 16.0f;
  float pitch = myEuler.p / 16.0f;
  float roll  = myEuler.r / 16.0f;

  float qw = myQuat.w / 16384.0f;
  float qx = myQuat.x / 16384.0f;
  float qy = myQuat.y / 16384.0f;
  float qz = myQuat.z / 16384.0f;

  float gx = (myGyro.x / 16.0f) * (PI / 180.0f);
  float gy = (myGyro.y / 16.0f) * (PI / 180.0f);
  float gz = (myGyro.z / 16.0f) * (PI / 180.0f);

  float ax = myAccel.x / 100.0f;
  float ay = myAccel.y / 100.0f;
  float az = myAccel.z / 100.0f;

  // ---------------- Output ----------------
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    Serial.print(now); Serial.print(",");
    Serial.print(omegaFilt, 6); Serial.print(",");
    Serial.print(yaw, 2); Serial.print(",");
    Serial.print(pitch, 2); Serial.print(",");
    Serial.print(roll, 2); Serial.print(",");
    Serial.print(qw, 6); Serial.print(",");
    Serial.print(qx, 6); Serial.print(",");
    Serial.print(qy, 6); Serial.print(",");
    Serial.print(qz, 6); Serial.print(",");
    Serial.print(gx, 6); Serial.print(",");
    Serial.print(gy, 6); Serial.print(",");
    Serial.print(gz, 6); Serial.print(",");
    Serial.print(ax, 4); Serial.print(",");
    Serial.print(ay, 4); Serial.print(",");
    Serial.print(az, 4);
    Serial.println();

    lastPrint = now;
  }
}