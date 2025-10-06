// *************************************************************  Library  ************************************************************* //

#include <WiFi.h>
#include <esp_now.h>

// *************************************************************  Parameters & Variables  ************************************************************* //

// Motors 
#define MA 5  
#define MB 3 
#define MC 1 
#define MD 7 
#define LEDC_TIMER_10_BIT 10
#define LEDC_BASE_FREQ 20000

// State
bool swlPressed = false;
bool swrPressed = false;
bool lastArmButton = false;
bool Armed = false;
bool OnFlying = false;

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

void Arm() {
  ledcWrite(MA, 100);
  ledcWrite(MB, 100);
  ledcWrite(MC, 100);
  ledcWrite(MD, 100);
}

void Disarm() {
  ledcWrite(MA, 0);
  ledcWrite(MB, 0);
  ledcWrite(MC, 0);
  ledcWrite(MD, 0);
}

// =================== Motor output ===================
void driveMotors() {
  if (incomingJoystickData.YR > 5) {
    ledcWrite(MA, incomingJoystickData.YR);
    ledcWrite(MB, incomingJoystickData.YR);
    ledcWrite(MC, incomingJoystickData.YR);
    ledcWrite(MD, incomingJoystickData.YR);
  }
  else if (incomingJoystickData.XR > 5) {
    ledcWrite(MA, incomingJoystickData.XR);
    ledcWrite(MB, 0);
    ledcWrite(MC, 0);
    ledcWrite(MD, incomingJoystickData.XR);
  }
  else if (incomingJoystickData.XR < -5) {
    ledcWrite(MA, 0);
    ledcWrite(MB, abs(incomingJoystickData.XR));
    ledcWrite(MC, abs(incomingJoystickData.XR));
    ledcWrite(MD, 0);
  }
  else {
    ledcWrite(MA, 0);
    ledcWrite(MB, 0);
    ledcWrite(MC, 0);
    ledcWrite(MD, 0);
  }
}

void setup() {
  Serial.begin(115200);

  ledcAttach(MA, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(MB, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(MC, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(MD, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);

  setupESPNow();
  Serial.println(WiFi.macAddress());
}

void loop() {
  driveMotors();
}
