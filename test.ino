/*******************************************************************
    A touch screen test for the ESP32 Cheap Yellow Display.

    https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display

    If you find what I do useful and would like to support me,
    please consider becoming a sponsor on Github
    https://github.com/sponsors/witnessmenow/

    Written by Brian Lough
    YouTube: https://www.youtube.com/brianlough
    Twitter: https://twitter.com/witnessmenow
 *******************************************************************/

// Make sure to copy the UserSetup.h file into the library as
// per the Github Instructions. The pins are defined in there.

// ----------------------------
// Standard Libraries
// ----------------------------
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>

// ----------------------------
// Additional Libraries - each one of these will need to be installed.
// ----------------------------
#include <XPT2046_Touchscreen.h>
// A library for interfacing with the touch screen
//
// Can be installed from the library manager (Search for "XPT2046")
// https://github.com/PaulStoffregen/XPT2046_Touchscreen

#include <TFT_eSPI.h>
// A library for interfacing with LCD displays
//
// Can be installed from the library manager (Search for "TFT_eSPI")
// https://github.com/Bodmer/TFT_eSPI

// MESSAGE
typedef struct struct_message {
  char a[32];
  int seq;
  int mq_data;
  bool fl_data;
} struct_message;

// Create a struct_message called myData
struct_message myData;
struct_message listdata[10];

// Flag to track new data
bool newDataReceived = false;

// ----------------------------
// Touch Screen pins
// ----------------------------

// The CYD touch uses some non-default
// SPI pins
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// ----------------------------

SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

TFT_eSPI tft = TFT_eSPI();

// Constants for grid layout
const int rows = 3;
const int cols = 5;
const int cellWidth = 60;
const int cellHeight = 60;
const int gridTop = 50;
const int gridLeft = 10;

// Constants for back button
const int buttonWidth = 50;
const int buttonHeight = 30;
const int buttonX = 10;
const int buttonY = 10;

// State of each cell
uint16_t cellColors[rows][cols];
bool cellAlerts[rows][cols];

// Variables for touch debounce
unsigned long lastTouchTime = 0;
const unsigned long debounceDelay = 300; // 300 milliseconds debounce delay

bool isGridDisplayed = true; // Flag to indicate if the grid screen is displayed

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Start the SPI for the touch screen and init the TS library
  mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(mySpi);
  ts.setRotation(1);

  // Start the tft display and set it to black
  tft.init();
  tft.setRotation(1); // This is the display in landscape

  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);
  tft.invertDisplay(1);

  // Initialize the grid
  initializeGrid();
}

void initializeGrid() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("DEPARTMENT", 160, 20); // Display the title
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      cellColors[i][j] = TFT_LIGHTGREY;
      cellAlerts[i][j] = false;
    }
  }
  drawGrid();
}

void drawGrid() {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      int x = gridLeft + j * cellWidth;
      int y = gridTop + i * cellHeight;
      tft.fillRect(x, y, cellWidth, cellHeight, cellColors[i][j]);
      tft.drawRect(x, y, cellWidth, cellHeight, TFT_BLACK);
    }
  }
}

void drawButton() {
  tft.fillRect(buttonX, buttonY, buttonWidth, buttonHeight, TFT_BLUE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Back", buttonX + buttonWidth / 2, buttonY + buttonHeight / 2);
}

void updateCell(int row, int col, uint16_t color) {
  int x = gridLeft + col * cellWidth;
  int y = gridTop + row * cellHeight;
  tft.fillRect(x, y, cellWidth, cellHeight, color);
  tft.drawRect(x, y, cellWidth, cellHeight, TFT_BLACK);
}

void switchToDifferentScreen() {
  tft.fillScreen(TFT_BLACK); // Clear the screen
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("New Screen", 160, 60); // Display some text on the new screen
  
  // Draw green rectangle
  tft.fillRoundRect(60, 100, 100, 50, 10, TFT_GREEN);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(String(listdata[1].mq_data), 110, 125);

  // Draw red rectangle
  tft.fillRoundRect(180, 100, 100, 50, 10, TFT_RED);
  tft.setTextColor(TFT_WHITE); 
  tft.setTextSize(1);
  tft.drawString(String(listdata[1].fl_data), 230, 125);
  
  drawButton(); // Draw the back button
  isGridDisplayed = false;
}

void updateScreenData() {
  // Update the displayed data on the screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("New Screen", 160, 60); // Display some text on the new screen
  
  // Draw green rectangle
  tft.fillRoundRect(60, 100, 100, 50, 10, TFT_GREEN);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(String(listdata[1].mq_data), 110, 125);

  // Draw red rectangle
  tft.fillRoundRect(180, 100, 100, 50, 10, TFT_RED);
  tft.setTextColor(TFT_WHITE); 
  tft.setTextSize(1);
  tft.drawString(String(listdata[1].fl_data), 230, 125);
  
  drawButton(); // Draw the back button
}

void handleTouch(int x, int y) {
  if (isGridDisplayed) {
    int col = (x - gridLeft) / cellWidth;
    int row = (y - gridTop) / cellHeight;
    if (col >= 0 && col < cols && row >= 0 && row < rows) {
      switchToDifferentScreen();
    }
  } else {
    // Check if the back button is pressed
    if (x >= buttonX && x <= (buttonX + buttonWidth) && y >= buttonY && y <= (buttonY + buttonHeight)) {
      tft.fillScreen(TFT_BLACK);
      initializeGrid();
      isGridDisplayed = true;
    }
  }
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

  // Set new data flag
  newDataReceived = true;
}

void loop() {
  // No longer simulating alert data
  unsigned long currentTime = millis();
  
  // Update screen data if new data is received
  if (!isGridDisplayed && newDataReceived) {
    listdata[myData.seq - 1] = myData;
    updateScreenData();
    newDataReceived = false;
  }

  // Touch handling
  if (ts.tirqTouched() && ts.touched()) {
    TS_Point p = ts.getPoint();
    long x_map = map(p.x, 200, 3800, 0, 320);
    long y_map = map(p.y, 200, 3800, 0, 240);

    // Only process the touch if debounce delay has passed
    if (currentTime - lastTouchTime > debounceDelay) {
      handleTouch(x_map, y_map);
      lastTouchTime = currentTime; // Update the last touch time
    }
  
    delay(100);
  }
}