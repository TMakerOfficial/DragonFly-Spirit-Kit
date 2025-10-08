#include <WiFi.h>
#include <esp_now.h>

// ==== PIN Definitions ====
#define XL_PIN 3 //3 34
#define YL_PIN 4  //4 35
#define SWL_PIN 6 //6 25
#define XR_PIN 1 //1 32
#define YR_PIN 2 //2 33
#define SWR_PIN 5 //5 26

// ==== Joystick Data ====
int XL, YL, XR, YR;
bool SWL, SWR;
uint8_t webCommand = 0;


// ==== ESP-NOW ====
uint8_t ReceiverMAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
struct JoyData {
  int XL, YL;
  bool SWL;
  int XR, YR;
  bool SWR;
  uint8_t webCommand;
};

void readJoystick() {
  XL = readAverage(XL_PIN);
  YL = readAverage(YL_PIN);
  XR = readAverage(XR_PIN);
  YR = readAverage(YR_PIN);
  SWL = !digitalRead(SWL_PIN);
  SWR = !digitalRead(SWR_PIN);
}

void sendESPNow() {
  JoyData data = {XL, YL, SWL, XR, YR, SWR, webCommand};
  esp_now_send(ReceiverMAC, (uint8_t*)&data, sizeof(data));
  
  Serial.print("  Sending -> ");
  Serial.print("XL: "); Serial.print(data.XL); Serial.print("  ");
  Serial.print("YL: "); Serial.print(data.YL); Serial.print("  ");
  Serial.print("SWL: "); Serial.print(data.SWL); Serial.print("  ");
  Serial.print("XR: "); Serial.print(data.XR); Serial.print("  ");
  Serial.print("YR: "); Serial.print(data.YR); Serial.print("  ");
  Serial.print("SWR: "); Serial.print(data.SWR); Serial.print("  ");
  Serial.print("webCommand: "); Serial.print(data.webCommand);
}

void OnSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(68); // 17 dBm
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  prefs.begin("config", true);
  String macStr = prefs.getString("ReceiverMAC", "00:00:00:00:00:00");
  prefs.end();

  sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
         &ReceiverMAC[0], &ReceiverMAC[1], &ReceiverMAC[2],
         &ReceiverMAC[3], &ReceiverMAC[4], &ReceiverMAC[5]);

  esp_now_register_send_cb(OnSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ReceiverMAC, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.print("📡 Receiver MAC Set: ");
  Serial.println(macStr);
}

void setup() {
  Serial.begin(115200);
  pinMode(SWL_PIN, INPUT_PULLUP);
  pinMode(SWR_PIN, INPUT_PULLUP);
  setupESPNow();
}

void loop() {
  readJoystick();
  sendESPNow();
  delay(50);
} 