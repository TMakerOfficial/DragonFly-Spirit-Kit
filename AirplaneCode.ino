// *************************************************************  Library  ************************************************************* //

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_WE.h>
#define MPU9250_ADDR 0x68

// *************************************************************  Parameters & Variables  ************************************************************* //

// Motors 
#define MA 5  
#define MB 3 
#define MC 1 
#define MD 7 
#define LEDC_TIMER_10_BIT 10
#define LEDC_BASE_FREQ 20000

// Objects 
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
Adafruit_BMP280 bmp;
Madgwick MadgwickFilter;

// Timer 
unsigned long lastInnerTime = 0;
unsigned long lastOuterTime = 0;
unsigned long lastBaroTime  = 0;
unsigned long lastDNSTime = 0;

const float innerHz = 400.0;    // Inner loop rate (Madgwick + inner PID)
const float outerHz = 100.0;    // Outer loop rate (angle PID / altitude)
const float baroHz  = 25.0;     // Barometer update rate

const float innerDt = 1.0 / innerHz;
const float outerDt = 1.0 / outerHz;
const float baroDt  = 1.0 / baroHz;

// Initials
float targetRoll = 0 ,targetPitch = 0 ,targetYaw = 0;
float currentRoll = 0 ,currentPitch = 0 ,currentYaw = 0;
float yaw_setpoint = 0.0; // deg
float yaw_rate_target = 0;
float baroAltitude = 0;
float currentAltitude = 0;
float altitude_setpoint = 0.0; // meter
float altitude_rate_target = 0.0;
float altitude_baseline = 0.0;
float max_altitude = 0.0;
float min_altitude = 0.0;
float g = 9.80665; // m/s^2

int baseSpeed = 480;
int integralLimit = 10;
float range_altitude = 15;
float maxRateChange = 1; // m/s
float kff_roll = 0.5; // FeedForward Roll
float kff_pitch = 0.5; // FeedForward Pitch
float kff_yaw = 0.5; // FeedForward Yaw
float kff_altitude = 0.9; // FeedForward Altitude 

float altitude = 0;
float velocityZ = 0;
float alpha = 0.95;
float beta = 0.75;
float previousBaro = 0;

float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
float mx_bias = 0, my_bias = 0, mz_bias = 0;
float mx_scale = 1, my_scale = 1, mz_scale = 1;
float magXmin =  1000, magXmax = -1000;
float magYmin =  1000, magYmax = -1000;
float magZmin =  1000, magZmax = -1000;

// PID Control
struct PID_t {
  float P;
  float I;
  float D;
};

PID_t pidRoll_rate, pidPitch_rate, pidYaw_rate, pidAltitude_rate, pidRoll_angle, pidPitch_angle, pidYaw_angle, pidAltitude_m, pidAltitude_height;
float trimRoll, trimPitch, trimYaw, trimAltitude;

// Outer loop PID (deg and m control)
float rollKp_angle = 10.0 ,rollKi_angle = 0.0 ,rollKd_angle = 0.0;  
float rollError_angle, rollPrevError_angle = 0, rollIntegral_angle = 0;
float rollOutput_angle; 

float pitchKp_angle = 9.0 ,pitchKi_angle = 0.5 ,pitchKd_angle = 0.0; 
float pitchError_angle, pitchPrevError_angle = 0, pitchIntegral_angle = 0;
float pitchOutput_angle;

float yawKp_angle = 4.0 ,yawKi_angle = 0.0 ,yawKd_angle = 0.0; 
float yawError_angle, yawPrevError_angle = 0, yawIntegral_angle = 0;
float yawOutput_angle;

float altitudeKp_m = 1.0 ,altitudeKi_m = 0.0 ,altitudeKd_m = 0.4;
float altitudeError_m, altitudePrevError_m = 0, altitudeIntegral_m = 0;
float altitudeOutput_m;

// Inner loop PID (rate control ==> deg/s and m/s)
float rollKp_rate = 1.2 ,rollKi_rate = 0.0 ,rollKd_rate = 0.03; 
float rollError_rate, rollPrevError_rate = 0, rollIntegral_rate = 0;
float rollOutput_rate; 

float pitchKp_rate = 1.2 ,pitchKi_rate = 0.0 ,pitchKd_rate = 0.04; 
float pitchError_rate, pitchPrevError_rate = 0, pitchIntegral_rate = 0;
float pitchOutput_rate;

float yawKp_rate = 3.0 ,yawKi_rate = 0.0 ,yawKd_rate = 0.03; 
float yawError_rate, yawPrevError_rate = 0, yawIntegral_rate = 0;
float yawOutput_rate;

float altitudeKp_rate = 45.0 ,altitudeKi_rate = 0.00 ,altitudeKd_rate = 1.5;
float altitudeError_rate, altitudePrevError_rate = 0, altitudeIntegral_rate = 0;
float altitudeOutput_rate;

// Filters
float gyroX_filtered = 0; 
float gyroY_filtered = 0;
float gyroZ_filtered = 0;
float accX_filtered  = 0;
float accY_filtered  = 0;
float accZ_filtered  = 0;
float magX_filtered  = 0;
float magY_filtered  = 0;
float magZ_filtered  = 0;
float alt_filtered  = 0;

// EMA filter
float emaGyroX = 0;
float emaGyroY = 0;
float emaGyroZ = 0;
float emaAccX = 0;
float emaAccY = 0;
float emaAccZ = 0;
float emaMagX = 0;
float emaMagY = 0;
float emaMagZ = 0;
float emaAlt = 0;
float emaVZ = 0;

float alphaGyro = 0.8f; 
float alphaAcc  = 0.7f;  
float alphaMag  = 0.9f; 
float alphaAlt  = 0.5f;
float alphaVZ  = 0.6f; 


// Median filter 
float gyroX_buf[9] = {0}, gyroY_buf[9] = {0}, gyroZ_buf[9] = {0};
float accX_buf[9]  = {0}, accY_buf[9]  = {0}, accZ_buf[9]  = {0};
float magX_buf[3]  = {0}, magY_buf[3]  = {0}, magZ_buf[3]  = {0};
float alt_buf[7]  = {0};
float vz_buf[5]  = {0};
size_t gyroX_idx = 0, gyroY_idx = 0, gyroZ_idx = 0;
size_t accX_idx  = 0, accY_idx  = 0, accZ_idx  = 0;
size_t magX_idx  = 0, magY_idx  = 0, magZ_idx  = 0;
size_t alt_idx  = 0;
size_t vz_idx  = 0;

// State
bool swlPressed = false;
bool swrPressed = false;
bool lastArmButton = false;
bool Armed = false;
bool OnFlying = false;
bool initial_altitude = false;
bool initial_yaw = false;
volatile bool calibrateAccelGyroRequested = false;
volatile bool calibrateMagRequested = false;
bool AccelGyroisCalibrating = false;
bool AccelGyrocalibrationDone = false;
bool MagisCalibrating = false;
bool MagcalibrationDone = false;

// ESP Now 
String getMacAddress() {
  return WiFi.macAddress();
}

typedef struct JoyData {
  int XL, YL;
  bool SWL;
  int XR, YR;
  bool SWR;
  uint8_t webCommand;
} JoyData;

JoyData incomingJoystickData;
uint8_t lastWebCommand = 255;

// *************************************************************  Functions  ************************************************************* //

void readJoystickData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(incomingJoystickData)) {
    memcpy(&incomingJoystickData, data, sizeof(incomingJoystickData));
  } else {
    Serial.printf("Received invalid data size: %d bytes\n", len);
  }
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_err_t result = esp_now_register_recv_cb(readJoystickData);
  if (result == ESP_OK) {
    Serial.println("Receive callback registered successfully");
  } else {
    Serial.printf("Failed to register receive callback: %d\n", result);
  }
}

float angleError(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// ---------------- Median Filter ----------------
template <size_t N>
float medianFilter(float input, float (&buffer)[N], size_t &index) {
  buffer[index] = input;
  index = (index + 1) % N;

  float temp[N];
  memcpy(temp, buffer, sizeof(temp));

  // sort ค่า
  for (size_t i = 0; i < N - 1; i++) {
    for (size_t j = i + 1; j < N; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[N / 2];
}

// ---------------- EMA Filter ----------------
float emaFilter(float input, float &emaPrev, float alpha) {
  emaPrev = alpha * input + (1.0f - alpha) * emaPrev;
  return emaPrev;
}

// ====== Complementary filter =====
void Complementary_Alt_Vz(float accZ, float altitudeBaro, float dt) {
    // integrate acceleration
    velocityZ += accZ * dt;
    altitude += velocityZ * dt;

    // complementary filter สำหรับ altitude
    altitude = alpha * altitude + (1 - alpha) * altitudeBaro;

    // complementary filter สำหรับ velocityZ
    float baroVz = (altitudeBaro - previousBaro) / dt;
    velocityZ = beta * velocityZ + (1 - beta) * baroVz;

    previousBaro = altitudeBaro;
}

void updateSensorsAndMadgwick(float dt) {
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr    = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();

  // Read sensor
  float accX = gValue.x;
  float accY = gValue.y;
  float accZ = gValue.z;
  float gyroX = gyr.x;
  float gyroY = gyr.y;
  float gyroZ = gyr.z;
  float magX = magValue.x - mx_bias;
  float magY = magValue.y - my_bias;
  float magZ = magValue.z - mz_bias; 

  // ----------- Median filter -------------
  float gyroX_med = medianFilter(gyroX, gyroX_buf, gyroX_idx);
  float gyroY_med = medianFilter(gyroY, gyroY_buf, gyroY_idx);
  float gyroZ_med = medianFilter(gyroZ, gyroZ_buf, gyroZ_idx);
  float accX_med  = medianFilter(accX,  accX_buf,  accX_idx);
  float accY_med  = medianFilter(accY,  accY_buf,  accY_idx);
  float accZ_med  = medianFilter(accZ,  accZ_buf,  accZ_idx);
  float magX_med  = medianFilter(magX,  magX_buf,  magX_idx);
  float magY_med  = medianFilter(magY,  magY_buf,  magY_idx);
  float magZ_med  = medianFilter(magZ,  magZ_buf,  magZ_idx);
  float alt_med  = medianFilter(baroAltitude,  alt_buf,  alt_idx);

  // ----------- EMA filter ------------------
  gyroX_filtered = emaFilter(gyroX_med, emaGyroX, alphaGyro); 
  gyroY_filtered = emaFilter(gyroY_med, emaGyroY, alphaGyro);
  gyroZ_filtered = emaFilter(gyroZ_med, emaGyroZ, alphaGyro);
  accX_filtered  = emaFilter(accX_med,  emaAccX,  alphaAcc);
  accY_filtered  = emaFilter(accY_med,  emaAccY,  alphaAcc);
  accZ_filtered  = emaFilter(accZ_med,  emaAccZ,  alphaAcc);
  magX_filtered  = emaFilter(magX_med,  emaMagX,  alphaMag);
  magY_filtered  = emaFilter(magY_med,  emaMagY,  alphaMag);
  magZ_filtered  = emaFilter(magZ_med,  emaMagZ,  alphaMag);
  alt_filtered  = emaFilter(alt_med,  emaAlt,  alphaAlt);

  float checkMag = magValue.x + magValue.y + magValue.z;

  if (checkMag == 0) {
    MadgwickFilter.updateIMU(gyroX_filtered, gyroY_filtered, gyroZ_filtered,
                             accX_filtered, accY_filtered, accZ_filtered);
  } 
  else if (checkMag != 0) {
    float avgScale = (mx_scale + my_scale + mz_scale) / 3.0;
    MadgwickFilter.update(gyroX_filtered, gyroY_filtered, gyroZ_filtered,
                          accX_filtered, accY_filtered, accZ_filtered,
                          magX_filtered, magY_filtered, magZ_filtered);
  }

  currentRoll  = -MadgwickFilter.getRoll();
  currentPitch =  MadgwickFilter.getPitch();
  currentYaw   = 360 - MadgwickFilter.getYaw();

  // g -> m/s^2
  float accX_ms2 = accX_filtered * g; 
  float accY_ms2 = accY_filtered * g; 
  float accZ_ms2 = accZ_filtered * g; 

  // deg -> rad
  float roll  = currentRoll  * DEG_TO_RAD;
  float pitch = currentPitch * DEG_TO_RAD;
  float yaw   = currentYaw   * DEG_TO_RAD;

  // transform body -> world
  float accZ_world = (-accX_ms2 * sin(pitch))
                      + (accY_ms2 * sin(roll) * cos(pitch))
                      + (accZ_ms2 * cos(roll) * cos(pitch));

  // vertical acceleration
  float accZ_true = accZ_world - g;

  Complementary_Alt_Vz(accZ_true, baroAltitude, innerDt);
  
  currentAltitude = altitude;
  velocityZ  = medianFilter(velocityZ,  vz_buf,  vz_idx);
  velocityZ  = emaFilter(velocityZ,  emaVZ,  alphaVZ);
}

void updateParameters(float dt) {
  targetRoll  = constrain(incomingJoystickData.XR + trimRoll, -30, 30); // deg
  targetPitch = constrain(incomingJoystickData.YR + trimPitch, -30, 30); // deg
  targetYaw   = constrain(incomingJoystickData.XL + trimYaw, -90, 90); // deg/s
  // --- Yaw control ---
  yaw_rate_target = targetYaw;
  yaw_setpoint += yaw_rate_target * dt;

  float rawAltitudeRate = constrain(incomingJoystickData.YL * 0.01, -5, 5); // m/s

  // Update altitude setpoint 
  altitude_setpoint += rawAltitudeRate * dt;

  // Clamp setpoint
  altitude_setpoint = constrain(altitude_setpoint, min_altitude, max_altitude);
}

// =================== Inner PID (rate) ===================
void innerPID(float dt) {
  float gyroRoll  = -gyroX_filtered;
  float gyroPitch =  gyroY_filtered;
  float gyroYaw   = -gyroZ_filtered;

  // Roll rate
  float rollError = rollOutput_angle - gyroRoll;
  rollIntegral_rate += rollError * dt;
  rollIntegral_rate = constrain(rollIntegral_rate, -integralLimit, integralLimit);
  rollOutput_rate = pidRoll_rate.P * rollError +
                    pidRoll_rate.I * rollIntegral_rate +
                    pidRoll_rate.D * ((rollError - rollPrevError_rate)/dt);
  rollOutput_rate = rollOutput_rate + (kff_roll * rollOutput_angle);  
  rollPrevError_rate = rollError;

  // Pitch rate
  float pitchError = pitchOutput_angle - gyroPitch;
  pitchIntegral_rate += pitchError * dt;
  pitchIntegral_rate = constrain(pitchIntegral_rate, -integralLimit, integralLimit);
  pitchOutput_rate = pidPitch_rate.P * pitchError +
                     pidPitch_rate.I * pitchIntegral_rate +
                     pidPitch_rate.D * ((pitchError - pitchPrevError_rate)/dt);
  pitchOutput_rate = pitchOutput_rate + (kff_pitch * pitchOutput_angle);
  pitchPrevError_rate = pitchError;

  // Yaw rate
  float yawError = yawOutput_angle - gyroYaw;
  yawIntegral_rate += yawError * dt;
  yawIntegral_rate = constrain(yawIntegral_rate, -integralLimit, integralLimit);
  yawOutput_rate = pidYaw_rate.P * yawError +
                   pidYaw_rate.I * yawIntegral_rate +
                   pidYaw_rate.D * ((yawError - yawPrevError_rate)/dt);
  yawOutput_rate = yawOutput_rate + (kff_yaw * yawOutput_angle);
  yawPrevError_rate = yawError;

  // Altitude rate
  float altitudeError = altitudeOutput_m - velocityZ;
  altitudeIntegral_rate += altitudeError * dt;
  altitudeIntegral_rate = constrain(altitudeIntegral_rate, -integralLimit, integralLimit);
  altitudeOutput_rate = pidAltitude_rate.P * altitudeError +
                        pidAltitude_rate.I * altitudeIntegral_rate +
                        pidAltitude_rate.D * ((altitudeError - altitudePrevError_rate)/dt);      
  altitudeOutput_rate = altitudeOutput_rate + (kff_altitude * altitudeOutput_m);                                  
  altitudePrevError_rate = altitudeError;
}

// =================== Outer PID (angle / altitude) ===================
void outerPID(float dt) {
  // Roll angle
  float rollError_angle = targetRoll - currentRoll;
  rollIntegral_angle += rollError_angle * dt;
  rollIntegral_angle = constrain(rollIntegral_angle, -integralLimit, integralLimit);
  rollOutput_angle = pidRoll_angle.P * rollError_angle +
                     pidRoll_angle.I * rollIntegral_angle +
                     pidRoll_angle.D * ((rollError_angle - rollPrevError_angle)/dt);
  rollPrevError_angle = rollError_angle;

  // Pitch angle
  float pitchError_angle = targetPitch - currentPitch;
  pitchIntegral_angle += pitchError_angle * dt;
  pitchIntegral_angle = constrain(pitchIntegral_angle, -integralLimit, integralLimit);
  pitchOutput_angle = pidPitch_angle.P * pitchError_angle +
                      pidPitch_angle.I * pitchIntegral_angle +
                      pidPitch_angle.D * ((pitchError_angle - pitchPrevError_angle)/dt);
  pitchPrevError_angle = pitchError_angle;

  // Yaw angle
  float yawError_angle = angleError(yaw_setpoint, currentYaw);
  yawIntegral_angle += yawError_angle * dt;
  yawIntegral_angle = constrain(yawIntegral_angle, -integralLimit, integralLimit);
  yawOutput_angle = pidYaw_angle.P * yawError_angle +
                    pidYaw_angle.I * yawIntegral_angle +
                    pidYaw_angle.D * ((yawError_angle - yawPrevError_angle)/dt);
  yawPrevError_angle = yawError_angle;

  // Altitude
  float altitudeError_m = altitude_setpoint - currentAltitude;
  altitudeIntegral_m += altitudeError_m * dt;
  altitudeIntegral_m = constrain(altitudeIntegral_m, -integralLimit, integralLimit);
  altitudeOutput_m = pidAltitude_m.P * altitudeError_m +
                          pidAltitude_m.I * altitudeIntegral_m +
                          pidAltitude_m.D * ((altitudeError_m - altitudePrevError_m)/dt);                                           
  altitudePrevError_m = altitudeError_m;
}

// =================== Motor output ===================
void driveMotors() {
  float m1 = baseSpeed  + pitchOutput_rate + rollOutput_rate - yawOutput_rate + altitudeOutput_rate;
  float m2 = baseSpeed  + pitchOutput_rate - rollOutput_rate + yawOutput_rate + altitudeOutput_rate;
  float m3 = baseSpeed  - pitchOutput_rate - rollOutput_rate - yawOutput_rate + altitudeOutput_rate;
  float m4 = baseSpeed  - pitchOutput_rate + rollOutput_rate + yawOutput_rate + altitudeOutput_rate;

  m1 = constrain(m1, 0, 1023);
  m2 = constrain(m2, 0, 1023);
  m3 = constrain(m3, 0, 1023);
  m4 = constrain(m4, 0, 1023);
}

void FlightController() {
  unsigned long now = micros();

  // -------- Inner loop: 400 Hz --------
  if (now - lastInnerTime >= 1000000UL / innerHz) {
    lastInnerTime += 1000000UL / innerHz;   // ใช้ period คงที่
    updateSensorsAndMadgwick(innerDt);      // innerDt คงที่ = 1/400
    innerPID(innerDt);
  }

  // -------- Outer loop: 100 Hz --------
  if (now - lastOuterTime >= 1000000UL / outerHz) {
    lastOuterTime += 1000000UL / outerHz;
    outerPID(outerDt);
  }

  // -------- Barometer: 50 Hz --------
  if (now - lastBaroTime >= 1000000UL / baroHz) {
    lastBaroTime += 1000000UL / baroHz;
    baroAltitude = bmp.readAltitude(1013.25);
  }
}

void setupSensors() {
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_2);
  myMPU9250.setSampleRateDivider(0);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_2);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  MadgwickFilter.begin(innerHz);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  Wire.setClock(400000); // 400 kHz Fast Mode

  ledcAttach(MA, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(MB, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(MC, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(MD, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);

  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  if (!bmp.begin(0x76)) {
    while (1) {
      Serial.println("BMP280 connection failed. Please check your connection");
      delay(10);
    }
  }
  
  setupSensors();
  setupESPNow();

  incomingJoystickData.webCommand = 1;
  Serial.println("Setup done, waiting for joystick data...");
}

void loop() {
  updateParameters(innerDt);
  FlightController();
  driveMotors();
}
