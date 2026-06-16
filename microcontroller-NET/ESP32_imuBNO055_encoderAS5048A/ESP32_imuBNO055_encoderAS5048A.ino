#include <SPI.h>
#include <Wire.h>
#include "BNO055_support.h"

// ---------------- Pins ----------------
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_CS     5

#define I2C_SDA   21
#define I2C_SCL   22

// ---------------- SPI ----------------
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE1);

// ---------------- AS5048A registers ----------------
static const uint16_t REG_CLEAR = 0x0001;
static const uint16_t REG_ANGLE = 0x3FFF;

// ---------------- Encoder constants ----------------
static const int32_t CPR = 16384;
static const int32_t WRAP_TH = CPR / 2;

// ---------------- Mechanical parameters ----------------
static const float ENCODER_SIGN = 1.0f;
static const float WHEEL_RADIUS_M = 0.0475f;
static const float GEAR_RATIO = 10.48583f;

static const float METERS_PER_COUNT =
  (2.0f * PI * WHEEL_RADIUS_M) / (CPR * GEAR_RATIO);

// ---------------- Timing ----------------
static const uint32_t ENCODER_PERIOD_US = 500;    // 2 kHz lectura interna
static const uint32_t IMU_PERIOD_US     = 10000;  // 100 Hz IMU
static const uint32_t PRINT_PERIOD_US   = 20000;  // 50 Hz publicación terminal/ROS2
static const int WARMUP_SAMPLES = 20;

// ---------------- Noise handling ----------------
static const int DELTA_DEADBAND_COUNTS = 1;
static const int MAX_STEP_COUNTS = 2500;
static const float DS_ZERO_THRESH_M = 0.00001f;

// ---------------- IMU ----------------
struct bno055_t myBNO;
struct bno055_euler myEuler;
struct bno055_quaternion myQuat;
struct bno055_gyro myGyro;
struct bno055_linear_accel myAccel;

// ---------------- Encoder state ----------------
uint16_t lastCnt = 0;
uint16_t angleCnt = 0;
float angleDeg = 0.0f;
int32_t lastDelta = 0;

volatile int32_t dsAccum = 0;   // counts acumulados entre publicaciones
float dsStep = 0.0f;            // metros publicados en el último frame
float sTotal = 0.0f;            // metros totales

int warmupLeft = WARMUP_SAMPLES;

// ---------------- IMU cached values ----------------
float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
float ax = 0.0f, ay = 0.0f, az = 0.0f;

// ---------------- Timers ----------------
uint32_t lastEncoderUs = 0;
uint32_t lastImuUs = 0;
uint32_t lastPrintUs = 0;

// =====================================================
// Utilities
// =====================================================
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

static uint16_t makeReadCommand(uint16_t reg) {
  uint16_t cmd = 0x4000 | (reg & 0x3FFF);
  cmd |= ((uint16_t)parity16(cmd) << 15);
  return cmd;
}

static uint16_t readRegisterRaw(uint16_t reg) {
  uint16_t cmd = makeReadCommand(reg);
  xfer16(cmd);
  return xfer16(0x0000);
}

static uint16_t readRegisterData(uint16_t reg) {
  return readRegisterRaw(reg) & 0x3FFF;
}

static void clearErrorFlag() {
  uint16_t cmd = makeReadCommand(REG_CLEAR);
  xfer16(cmd);
  xfer16(0x0000);
}

static inline uint16_t readAngleFast() {
  return readRegisterData(REG_ANGLE);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(700);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  delay(50);
  clearErrorFlag();
  delay(5);

  Wire.begin(I2C_SDA, I2C_SCL);
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(100);

  angleCnt = readAngleFast();
  angleDeg = (360.0f * angleCnt) / CPR;
  lastCnt = angleCnt;

  uint32_t nowUs = micros();
  lastEncoderUs = nowUs;
  lastImuUs = nowUs;
  lastPrintUs = nowUs;

  Serial.println("# ESP32 FAST ENCODER READY");
  Serial.println("# ROS2: t,ds,yaw,pitch,roll,qw,qx,qy,qz,gx,gy,gz,ax,ay,az");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  uint32_t nowUs = micros();

  // --------- Encoder ----------
  if ((uint32_t)(nowUs - lastEncoderUs) >= ENCODER_PERIOD_US) {
    lastEncoderUs += ENCODER_PERIOD_US;

    angleCnt = readAngleFast();
    angleDeg = (360.0f * angleCnt) / CPR;

    int32_t delta = (int32_t)angleCnt - (int32_t)lastCnt;

    if (delta >  WRAP_TH) delta -= CPR;
    if (delta < -WRAP_TH) delta += CPR;

    if (abs(delta) <= DELTA_DEADBAND_COUNTS) delta = 0;
    if (abs(delta) >  MAX_STEP_COUNTS)       delta = 0;

    lastCnt = angleCnt;
    lastDelta = delta;

    if (warmupLeft > 0) {
      warmupLeft--;
    } else {
      dsAccum += delta;
    }
  }

  // --------- IMU aparte ----------
  if ((uint32_t)(nowUs - lastImuUs) >= IMU_PERIOD_US) {
    lastImuUs += IMU_PERIOD_US;

    bno055_read_euler_hrp(&myEuler);
    bno055_read_quaternion_wxyz(&myQuat);
    bno055_read_gyro_xyz(&myGyro);
    bno055_read_linear_accel_xyz(&myAccel);

    yaw   = myEuler.h / 16.0f;
    pitch = myEuler.p / 16.0f;
    roll  = myEuler.r / 16.0f;

    qw = myQuat.w / 16384.0f;
    qx = myQuat.x / 16384.0f;
    qy = myQuat.y / 16384.0f;
    qz = myQuat.z / 16384.0f;

    gx = (myGyro.x / 16.0f) * (PI / 180.0f);
    gy = (myGyro.y / 16.0f) * (PI / 180.0f);
    gz = (myGyro.z / 16.0f) * (PI / 180.0f);

    ax = myAccel.x / 100.0f;
    ay = myAccel.y / 100.0f;
    az = myAccel.z / 100.0f;
  }

  // --------- Publicación a terminal / ROS2 ----------
  if ((uint32_t)(nowUs - lastPrintUs) >= PRINT_PERIOD_US) {
    lastPrintUs += PRINT_PERIOD_US;

    int32_t countsToPublish = dsAccum;
    dsAccum = 0;

    dsStep = ENCODER_SIGN * ((float)countsToPublish * METERS_PER_COUNT);
    if (fabs(dsStep) < DS_ZERO_THRESH_M) {
      dsStep = 0.0f;
    } else {
      sTotal += dsStep;
    }

    Serial.print(millis()); Serial.print(",");
    Serial.print(dsStep, 8); Serial.print(",");
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
  }
}