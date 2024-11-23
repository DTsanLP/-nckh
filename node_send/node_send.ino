// #include <LiquidCrystal_I2C.h>
// #include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <DHT.h>
#define DHT11_PIN 14

DHT dht(DHT11_PIN, DHT11);
// uint8_t broadcastAddress[] = {0x30, 0xC9, 0x22, 0x32, 0x5D, 0x30};
// uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x00, 0x01, 0x00};
// uint8_t broadcastAddress[] = {0xD0,0xEF,0x76,0x57,0xF2,0x50};
uint8_t broadcastAddress[] = { 0x24, 0xDC, 0xC3, 0x44, 0xFF, 0x18 };  //TFT
// LiquidCrystal_I2C lcd(0x27, 16, 2);

typedef struct struct_message {
  uint8_t add[6];
  unsigned int seq;
  unsigned int mq_data;
  bool fl_data;
  float temp_data;
  uint8_t humi_data;
} struct_message;

struct_message myData;

typedef struct control_message {
  unsigned int seq;
  bool buz_ctrl;
  bool dcm_ctrl;
} control_message;

control_message ctrlData;

esp_now_peer_info_t peerInfo;
char macAddressStr[30];
uint8_t add_mac[6];
const int MQ = 36;
const int FL = 26;
const int dcm = 23;
const int buzz = 12;

const int freq = 5000;
const int Channel = 0;
const int resolution = 8;

bool buz_value = false;
bool dcm_value = false;
bool fl_value;
unsigned int mq_value;
float temp_value;
uint8_t humi_value;
void readMacAddress(char *macStr, size_t size) {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    // Format the MAC address into the provided char array
    snprintf(macStr, size, "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X",
             baseMac[0], baseMac[1], baseMac[2],
             baseMac[3], baseMac[4], baseMac[5]);
    for (int i = 0; i < 6; i++) {
      add_mac[i] = baseMac[i];
    }

  } else {
    // Handle the error by setting the macStr to an error message
    snprintf(macStr, size, "Failed to read MAC address");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&ctrlData, incomingData, sizeof(ctrlData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Info: ");
  Serial.println(ctrlData.seq);
  Serial.print("BUZZ: ");
  Serial.println(ctrlData.buz_ctrl);
  Serial.print("DCM: ");
  Serial.println(ctrlData.dcm_ctrl);
  Serial.println("-------");
}
void setup() {
  // lcd.init();
  // lcd.backlight();

  // ledcAttach(buzz, freq, resolution);

  dht.begin();
  pinMode(MQ, INPUT);
  pinMode(FL, INPUT);
  pinMode(dcm, OUTPUT);
  pinMode(buzz, OUTPUT);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  readMacAddress(macAddressStr, sizeof(macAddressStr));

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  readSensorShow();
  delay(500);
  for (int i = 0; i < 6; i++) {
    myData.add[i] = add_mac[i];
  }
  myData.seq = 201;
  myData.mq_data = mq_value;
  myData.fl_data = fl_value;
  myData.temp_data = temp_value;
  myData.humi_data = humi_value;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void readSensorShow() {
  fl_value = digitalRead(FL);
  mq_value = analogRead(MQ);

  humi_value = dht.readHumidity();
  temp_value = dht.readTemperature();

  // Serial.print("FL: ");
  // Serial.println(fl_value);
  // Serial.print("MQ: " );
  // Serial.println(mq_value);
  // Serial.print("Temp: ");
  // Serial.println(temp_value);
  // Serial.print("Humi: " );
  // Serial.println(humi_value);
  Serial.println("-------");
  if (fl_value == 0 || mq_value >= 500 || temp_value >= 45) {
    digitalWrite(buzz, HIGH);
  }
  if (ctrlData.buz_ctrl == false) {
    digitalWrite(buzz, LOW);
    digitalWrite(dcm, LOW);
  }
  if (ctrlData.dcm_ctrl == true){
    digitalWrite(dcm, HIGH);
  }
  
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void OnOffBuzzer(bool On) {
  if (On == true) {
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
      // changing the LED brightness with PWM
      ledcWrite(buzz, dutyCycle);
      delay(15);
    }
  } else {
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
      // changing the LED brightness with PWM
      ledcWrite(buzz, dutyCycle);
      delay(15);
    }
  }
}