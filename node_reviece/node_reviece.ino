#include <esp_now.h>
#include <WiFi.h>

#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

typedef struct struct_message {
  char a[32];
  int seq;
  int mq_data;
  bool fl_data;
} struct_message;

// Create a struct_message called myData
struct_message myData;
struct_message listdata[10];
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  // Start the tft display and set it to black
  tft.init();
  tft.setRotation(1); //This is the display in landscape
  tft.invertDisplay(1);
  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int x = 5;
  int y = 10;
  int fontNum = 2; 
  tft.drawString("Hello", x, y, fontNum); // Left Aligned
  x = 320 /2;
  y += 16;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawCentreString("World", x, y, fontNum);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // String mq_str = String(myData.mq_data);
  // String fl_str = String(myData.fl_data);
  // int x = myData.seq == 1? 20 : 100;


  if (myData.seq == 1)
    listdata[0] = myData;
  if (myData.seq == 2)
    listdata[1] = myData;
  
  // tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // tft.drawString(listdata[myData.seq == 1? 0 : 1].a, x, 50, 2);
  // tft.drawNumber(listdata[myData.seq == 1? 0 : 1].mq_data, x, 100, 2);
  // tft.drawNumber(listdata[myData.seq == 1? 0 : 1].fl_data, x, 150, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(listdata[0].a, 20, 50, 2);
  char buffer[5]; 
  sprintf(buffer, "%04d", listdata[0].mq_data);
  tft.drawString(buffer, 20, 80, 2);
  tft.drawNumber(listdata[0].fl_data, 20, 100, 2);
   
  sprintf(buffer, "%04d", listdata[1].mq_data);
  tft.drawString(listdata[1].a, 20, 150, 2);
  tft.drawString(buffer, 20, 180, 2);
  tft.drawNumber(listdata[1].fl_data, 20, 200, 2);

}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Info: ");
  Serial.println(myData.a);
  Serial.print("MQ: ");
  Serial.println(myData.mq_data);
  Serial.print("Flame: ");
  Serial.println(myData.fl_data);
  Serial.println("-------");
}