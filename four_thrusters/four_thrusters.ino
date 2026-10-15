// Four thrusters + impeller pump relay + MPU-6050 gyro on one Arduino Uno.
//
// Serial command (115200 baud, one line per command):
//   LF,LR,RF,RR,pump\n      e.g.  1550,1550,1550,1550,1
// Thruster values are ESC pulse widths in microseconds (1500 = stop,
// clamped to 1000-2000). pump is 1 = on, 0 = off.
//   CAL\n                   re-measure the gyro bias (keep the robot still)
//
// If no valid command arrives for FAILSAFE_TIMEOUT_MS, all thrusters go to
// 1500 and the pump turns off. The sketch prints "PUMP ON" / "PUMP OFF" when
// the pump changes, and "FAILSAFE" when the failsafe trips.
//
// MPU-6050 on I2C (SDA = A4, SCL = A5, VCC = 5V, GND = GND). Every
// TELEMETRY_PERIOD_MS it prints:
//   IMU,<yaw rate deg/s>,<yaw deg>,<ax g>,<ay g>,<az g>
// yaw rate is the gyro axis YAW_AXIS (bias removed); yaw is its integral.
// Status lines: "IMU OK", "IMU MISSING", "IMU LOST", "CAL START", "CAL DONE".
#include <Servo.h>
#include <Wire.h>

const int NUM_MOTORS = 4;
// Order matches the command: left front, left rear, right front, right rear
const int MOTOR_PINS[NUM_MOTORS] = {9, 10, 5, 11};
Servo motors[NUM_MOTORS];

const int PUMP_RELAY_PIN = 7;
const int NEUTRAL_US = 1500;
const unsigned long FAILSAFE_TIMEOUT_MS = 500;

const int NUM_FIELDS = NUM_MOTORS + 1;
const int LINE_MAX = 48;
char line[LINE_MAX];
int lineLen = 0;
bool lineOverflow = false;

int pumpState = LOW;
unsigned long lastCommandTime = 0;
bool failsafeActive = true;

// ------------------------------------------------------------------- IMU
const uint8_t MPU_ADDR = 0x68;              // AD0 pin low/unconnected
const int YAW_AXIS = 2;                     // 0 = x, 1 = y, 2 = z (vertical when mounted flat)
const float GYRO_LSB_PER_DPS = 65.5;        // +/-500 deg/s range
const float ACCEL_LSB_PER_G = 16384.0;      // +/-2 g range
const unsigned long IMU_PERIOD_MS = 10;     // read + integrate at 100 Hz
const unsigned long TELEMETRY_PERIOD_MS = 50;
const int CAL_SAMPLES = 200;                // 2 s of samples
const int IMU_MAX_FAILS = 10;

bool imuOk = false;
bool calibrating = false;
int calCount = 0;
float calSum = 0;
float gyroBias = 0;
float yawRate = 0;                          // deg/s, bias removed
float yawDeg = 0;                           // integrated yaw
float accelG[3] = {0, 0, 0};
int imuFails = 0;
unsigned long lastImuMs = 0;
unsigned long lastImuMicros = 0;
unsigned long lastTelemetryMs = 0;

void writeMpu(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads accel x/y/z, temperature, gyro x/y/z (7 values). Returns false on I2C error.
bool readMpu(int16_t raw[7]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) {
    return false;
  }
  for (int i = 0; i < 7; i++) {
    uint8_t high = Wire.read();   // read high byte first (evaluation order matters)
    uint8_t low = Wire.read();
    raw[i] = (int16_t)((high << 8) | low);
  }
  return true;
}

void startCalibration() {
  calibrating = true;
  calCount = 0;
  calSum = 0;
  Serial.println("CAL START");
}

void setupImu() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);          // never hang if the sensor is unplugged

  Wire.beginTransmission(MPU_ADDR);
  imuOk = (Wire.endTransmission() == 0);
  if (!imuOk) {
    Serial.println("IMU MISSING");
    return;
  }
  writeMpu(0x6B, 0x01);                     // wake up, gyro clock
  writeMpu(0x1A, 0x03);                     // low-pass filter ~44 Hz
  writeMpu(0x1B, 0x08);                     // gyro +/-500 deg/s
  writeMpu(0x1C, 0x00);                     // accel +/-2 g
  delay(100);
  Serial.println("IMU OK");
  startCalibration();
  lastImuMicros = micros();
}

void updateImu() {
  int16_t raw[7];
  if (!readMpu(raw)) {
    if (++imuFails >= IMU_MAX_FAILS) {
      imuOk = false;
      Serial.println("IMU LOST");
    }
    return;
  }
  imuFails = 0;

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastImuMicros) * 1e-6;
  lastImuMicros = nowMicros;

  float rate = raw[4 + YAW_AXIS] / GYRO_LSB_PER_DPS;
  for (int i = 0; i < 3; i++) {
    accelG[i] = raw[i] / ACCEL_LSB_PER_G;
  }

  if (calibrating) {
    calSum += rate;
    if (++calCount >= CAL_SAMPLES) {
      gyroBias = calSum / calCount;
      calibrating = false;
      yawDeg = 0;
      Serial.println("CAL DONE");
    }
    yawRate = 0;
    return;
  }
  yawRate = rate - gyroBias;
  yawDeg += yawRate * dt;
}

void sendTelemetry() {
  Serial.print("IMU,");
  Serial.print(yawRate, 2);
  Serial.print(',');
  Serial.print(yawDeg, 1);
  for (int i = 0; i < 3; i++) {
    Serial.print(',');
    Serial.print(accelG[i], 3);
  }
  Serial.println();
}

// ------------------------------------------------------------ thrusters
void setPump(int state) {
  if (state != pumpState) {
    pumpState = state;
    digitalWrite(PUMP_RELAY_PIN, state);
    Serial.println(state == HIGH ? "PUMP ON" : "PUMP OFF");
  }
}

void allStop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(NEUTRAL_US);
  }
  setPump(LOW);
}

// Parses exactly NUM_FIELDS comma-separated integers. Rejects anything else.
bool parseLine(char *text, long values[]) {
  char *p = text;
  int count = 0;
  while (count < NUM_FIELDS) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) {
      return false;
    }
    values[count++] = v;
    p = end;
    if (*p == ',') {
      p++;
    } else {
      break;
    }
  }
  while (*p == ' ' || *p == '\r') {
    p++;
  }
  return count == NUM_FIELDS && *p == '\0';
}

void handleLine(char *text) {
  if (strncmp(text, "CAL", 3) == 0) {
    if (imuOk) {
      startCalibration();
    } else {
      Serial.println("IMU MISSING");
    }
    return;
  }
  long values[NUM_FIELDS];
  if (!parseLine(text, values)) {
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(constrain(values[i], 1000, 2000));
  }
  setPump(values[NUM_MOTORS] > 0 ? HIGH : LOW);
  lastCommandTime = millis();
  failsafeActive = false;
}

void setup() {
  Serial.begin(115200);

  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, LOW);

  // ESCs arm when they power up (main switch) and see 1500 us
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(MOTOR_PINS[i]);
    motors[i].writeMicroseconds(NEUTRAL_US);
  }

  setupImu();
}

void loop() {
  // Read characters without blocking; act on each complete line
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      if (!lineOverflow) {
        line[lineLen] = '\0';
        handleLine(line);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < LINE_MAX - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }

  // FAILSAFE: no valid command for FAILSAFE_TIMEOUT_MS -> stop everything
  if (!failsafeActive && millis() - lastCommandTime > FAILSAFE_TIMEOUT_MS) {
    allStop();
    failsafeActive = true;
    Serial.println("FAILSAFE");
  }

  unsigned long now = millis();
  if (imuOk && now - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = now;
    updateImu();
  }
  if (imuOk && !calibrating && now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
