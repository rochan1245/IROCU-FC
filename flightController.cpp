#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP3XX.h>
#include "bmm150.h"
#include "bmm150_defs.h"

// ================== RECEIVER PINS ==================
#define CH1_PIN 16  // YAW
#define CH2_PIN 15  // ROLL
#define CH3_PIN 7  // THROTTLE
#define CH4_PIN 6   // PITCH
#define CH5_PIN 5  // ARM/DISARM
#define CH6_PIN 16

// ================== MOTOR PINS ==================
#define M1_PIN 48
#define M2_PIN 35
#define M3_PIN 36
#define M4_PIN 37

// ================== MPU ==================
#define MPU_ADDR 0x68

// ================== CALIBRATION OFFSETS ==================
// You can edit these manually
float ACC_OFFSET_X = 0;
float ACC_OFFSET_Y = 0;
float ACC_OFFSET_Z = 0;

float GYRO_OFFSET_X = 0;
float GYRO_OFFSET_Y = 0;
float GYRO_OFFSET_Z = 0;

#define MAG_OFFSET_X -50
#define MAG_OFFSET_Y -15
#define MAG_OFFSET_Z -9

// ================== OBJECTS ==================
Servo M1, M2, M3, M4;
Adafruit_BMP3XX bmp;
BMM150 bmm;

// ================== PID CONSTANTS ==================
float Kp = 1.2;
float Ki = 0.001;
float Kd = 0.01;

/* try ************************************************************************
Kp = 2.0
Ki = 0.0
Kd = 0.8
*/

float Kp_yaw = 3.0;
float Kp_alt = 2.0;

// ================== VARIABLES ==================
float roll, pitch, yaw;
float rollPrevErr, pitchPrevErr;
float rollInt, pitchInt;

float altitudeBase = 0;
float currentAlt = 0;

bool armed = false;

unsigned long lastTime;
float dt;

// ================== READ MPU ==================
int16_t readMPU(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

// ================== RECEIVER ==================
uint16_t readChannel(uint8_t pin) {
  // timeout 25 ms (one RC frame)
  return pulseIn(pin, HIGH, 25000);
}

// ================== BARO CALIBRATION ==================
void calibrateBaro()
{
  float sum = 0;
  for (int i = 0; i < 100; i++)
  {
    bmp.performReading();
    sum += bmp.readAltitude(1013.25);
    delay(20);
  }
  altitudeBase = sum / 100.0;
}

// ================== SETUP ==================
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Wake MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Init BMP390
  bmp.begin_I2C(0x77);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  calibrateBaro();

  // Init BMM150
  bmm.initialize();

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  // Attach Motors
  M1.attach(M1_PIN, 1000, 2000);
  M2.attach(M2_PIN, 1000, 2000);
  M3.attach(M3_PIN, 1000, 2000);
  M4.attach(M4_PIN, 1000, 2000);

  delay(5000);

  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);

  delay(5000);
  lastTime = micros();

}

void loop()
{
  // ---- Receiver ----
  uint16_t ch1 = readChannel(CH1_PIN);
  uint16_t ch2 = readChannel(CH2_PIN);
  uint16_t ch3 = readChannel(CH3_PIN);
  uint16_t ch4 = readChannel(CH4_PIN);
  uint16_t ch5 = readChannel(CH5_PIN);
  uint16_t ch6 = readChannel(CH6_PIN);

  if (ch5 > 1500)
  {
    armed = 1;
  }
  else
  {
    armed = 0;
  }

  float yawSet   = map(ch1, 1000, 2000, -100, 100);
  float rollSet  = map(ch2, 1000, 2000, -30, 30);
  int throttle   = constrain(ch3, 1050, 1700);
  float pitchSet = map(ch4, 1000, 2000, -30, 30);

  // ---- Time ----
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // ---- ACCEL ----
  float ax = (readMPU(0x3B) / 16384.0) - ACC_OFFSET_X;
  float ay = (readMPU(0x3D) / 16384.0) - ACC_OFFSET_Y;
  float az = (readMPU(0x3F) / 16384.0) - ACC_OFFSET_Z;

  // ---- GYRO ----
  float gx = (readMPU(0x43) / 131.0) - GYRO_OFFSET_X;
  float gy = (readMPU(0x45) / 131.0) - GYRO_OFFSET_Y;
  float gz = (readMPU(0x47) / 131.0) - GYRO_OFFSET_Z;

  float accRoll  = atan2(ay, az) * 180 / PI;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  roll  = 0.98 * (roll + gx * dt) + 0.02 * accRoll;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * accPitch;

  // ---- YAW (Tilt Compensated) ----
  bmm.read_mag_data();
  float mx = bmm.raw_mag_data.raw_datax - MAG_OFFSET_X;
  float my = bmm.raw_mag_data.raw_datay - MAG_OFFSET_Y;
  float mz = bmm.raw_mag_data.raw_dataz - MAG_OFFSET_Z;

  float mx_comp = mx * cos(pitch * DEG_TO_RAD) + mz * sin(pitch * DEG_TO_RAD);
  float my_comp = mx * sin(roll * DEG_TO_RAD) * sin(pitch * DEG_TO_RAD)
                + my * cos(roll * DEG_TO_RAD)
                - mz * sin(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);

  yaw = atan2(-my_comp, mx_comp) * 180 / PI;

  // ---- BAROMETER ----
  bmp.performReading();
  currentAlt = (bmp.readAltitude(1013.25) - altitudeBase) * 100;

  // ---- PID ----
  float rollErr  = rollSet - roll;
  float pitchErr = pitchSet - pitch;

  float rollPID  = Kp * rollErr;
  float pitchPID = Kp * pitchErr;
  float yawPID   = Kp_yaw * yawSet;

  // ---- MOTOR MIXING ----
  int M1_out = throttle + pitchPID - rollPID - yawPID;
  int M2_out = throttle + pitchPID + rollPID + yawPID;
  int M3_out = throttle - pitchPID + rollPID - yawPID;
  int M4_out = throttle - pitchPID - rollPID + yawPID;

  
  if (M1_out > 2000)
  {
    M1_out = 1999;
  }

  if (M2_out > 2000)
  {
    M2_out = 1999;
  }

  if (M3_out > 2000)
  {
    M3_out = 1999;
  }

  if (M4_out > 2000)
  {
    M4_out = 1999;
  }


  M1_out = constrain(M1_out, 1000, 2000);
  M2_out = constrain(M2_out, 1000, 2000);
  M3_out = constrain(M3_out, 1000, 2000);
  M4_out = constrain(M4_out, 1000, 2000);

    //option 2 *************************************************************
// ----- Find max & min -----
float maxMotor = max(max(M1_out, M2_out), max(M3_out, M4_out));
float minMotor = min(min(M1_out, M2_out), min(M3_out, M4_out));

// ----- Scale if needed -----
if (maxMotor > 2000)
{
  float diff = maxMotor - 2000;
  M1_out -= diff;
  M2_out -= diff;
  M3_out -= diff;
  M4_out -= diff;
}

if (minMotor < 1000)
{
  float diff = 1000 - minMotor;
  M1_out += diff;
  M2_out += diff;
  M3_out += diff;
  M4_out += diff;
}
 



  if (armed)
  {
    M1.writeMicroseconds(M1_out);
    M2.writeMicroseconds(M2_out);
    M3.writeMicroseconds(M3_out);
    M4.writeMicroseconds(M4_out);
  }
  else
  {
    M1.writeMicroseconds(1000);
    M2.writeMicroseconds(1000);
    M3.writeMicroseconds(1000);
    M4.writeMicroseconds(1000);
   M1_out = 1000;
   M2_out = 1000;
   M3_out = 1000;
   M4_out = 1000;
  }

  // ================= SERIAL PRINT =================
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100)   // print every 100ms
  {
    lastPrint = millis();

    Serial.println("------------ FC DATA ------------");

    Serial.print("ARMED: "); Serial.println(armed);

    Serial.print("RX -> ");
    Serial.print("CH1(YAW): "); Serial.print(ch1);
    Serial.print("  CH2(ROLL): "); Serial.print(ch2);
    Serial.print("  CH3(THR): "); Serial.print(ch3);
    Serial.print("  CH4(PITCH): "); Serial.print(ch4);
    Serial.print("  CH5(ARM): "); Serial.println(ch5);

    Serial.print("Setpoints -> ");
    Serial.print("RollSet: "); Serial.print(rollSet);
    Serial.print("  PitchSet: "); Serial.print(pitchSet);
    Serial.print("  YawSet: "); Serial.println(yawSet);

    Serial.print("IMU -> ");
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print("  Pitch: "); Serial.print(pitch);
    Serial.print("  Yaw: "); Serial.println(yaw);

    Serial.print("Altitude (cm): ");
    Serial.println(currentAlt);

    Serial.print("Motors -> ");
    Serial.print("M1: "); Serial.print(M1_out);
    Serial.print("  M2: "); Serial.print(M2_out);
    Serial.print("  M3: "); Serial.print(M3_out);
    Serial.print("  M4: "); Serial.println(M4_out);

    Serial.println("----------------------------------");
  }
 
}

