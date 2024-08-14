// #include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>
#define DHT11_PIN 14

DHT dht(DHT11_PIN, DHT11);
// uint8_t broadcastAddress[] = {0x30, 0xC9, 0x22, 0x32, 0x5D, 0x30};
// uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x00, 0x01, 0x00};
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0x57, 0xF2, 0x50};
// LiquidCrystal_I2C lcd(0x27, 16, 2);

typedef struct struct_message {
  // char a[32];
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

const int MQ = 36;
const int FL = 26;
const int dcm = 25;
const int buzz = 17;

bool buz_value = false;
bool dcm_value = false;
bool fl_value;
unsigned int mq_value;
float temp_value;
uint8_t humi_value;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
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
  dht.begin();
  pinMode(MQ, INPUT);
  pinMode(FL, INPUT);
  pinMode(dcm, OUTPUT);
  pinMode(buzz, OUTPUT);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  readSensorShow();
  delay(500);

  myData.seq = 101;
  myData.mq_data = mq_value;
  myData.fl_data = fl_value;
  myData.temp_data = temp_value;
  myData.humi_data = humi_value;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void readSensorShow()
{
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
  if (fl_value == 0 || mq_value >= 1000 || temp_value >= 45)
  {
    dcm_value = true;
    buz_value = true;
  }
  if (ctrlData.buz_ctrl == false)
  {
    dcm_value = false;
    buz_value = false;
  }

  digitalWrite(buzz, buz_value == true? HIGH : LOW);
  digitalWrite(dcm, dcm_value == true? HIGH : LOW);

  // lcd.clear();
  // lcd.setCursor(0, 0); 
  // lcd.print("FL: ");
  // lcd.print(fl_value);
  
  // lcd.setCursor(8, 0);
  // lcd.print("BUZ: ");
  // lcd.print(buz_value == true? "HI":"LO");

  // lcd.setCursor(0, 1); 
  // lcd.print("MQ: ");
  // lcd.print(mq_value);

  // lcd.setCursor(8, 1);
  // lcd.print("DCM: ");
  // lcd.print(dcm_value == true? "HI":"LO");
  
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}