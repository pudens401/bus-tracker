/*******************************************************
 LCD + KEYPAD + EEPROM + SIM800 MQTT + GPS
 
 MQTT TOPICS USED :
 - wjp/bus/{SERIAL}/location          : GPS coordinates (publish every 30s)
 - wjp/bus/{SERIAL}/route/update      : Route updates (publish on save)
 - wjp/bus/{SERIAL}/route/req         : Request current route (publish "1" on startup)
 - wjp/bus/{SERIAL}/route/current     : Receive current route (subscribe)
********************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>

// ---------------- DEVICE SERIAL NUMBER ----------------
const char* DEVICE_SERIAL = "GPS001"; 

// ---------------- LCD + KEYPAD ----------------
#define BACKLIGHT_PIN 15
#define BACKLIGHT_TIMEOUT 10000UL

LiquidCrystal_I2C lcd(0x27, 20, 4);

const byte ROWS = 4, COLS = 4;
char keyMap[ROWS][COLS] = {
  {'1','4','7','*'}, {'2','5','8','0'},
  {'3','6','9','#'}, {'A','B','C','D'}
};
uint8_t rowPins[ROWS] = {13, 12, 14, 25};
uint8_t colPins[COLS] = {33, 32, 5, 4};
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);

bool editMode = false;
char route[4] = "---";
char bufferIn[4] = "---";
byte bufIdx = 0;

bool blinkOn = true;
unsigned long lastBlink = 0, lastKey = 0, msgStart = 0, editStart = 0;
bool showMsg = false;
const char* msg = "";

// Forward declaration
void cancelEdit();
void publishRoute();
void mqttCallback(char* topic, byte* payload, unsigned int length);

// ---------------- SIM800 + MQTT + GPS -----------------
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>

#define MODEM_RX 16
#define MODEM_TX 17

#define GPS_RX 26
#define GPS_TX 27

TinyGPSPlus gps;
HardwareSerial GPS(1);

HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);
TinyGsmClient net(modem);
PubSubClient mqtt(net);

// SIM APN
const char apn[]  = "internet.mtn";
const char user[] = "";
const char pass[] = "";

// MQTT
const char* mqttServer = "broker.gihangadynamics.com";
const uint16_t mqttPort = 1883;

const char* mqttUser = nullptr;
const char* mqttPassword = nullptr;

// Dynamic MQTT Topics based on device serial
char locationTopic[50];       // wjp/bus/GPS001/location
char routeUpdateTopic[50];    // wjp/bus/GPS001/route/update
char routeReqTopic[50];       // wjp/bus/GPS001/route/req
char routeCurrentTopic[50];   // wjp/bus/GPS001/route/current

String clientId;

double latitude = 0.0;
double longitude = 0.0;

const unsigned long publishInterval = 30000UL;
unsigned long lastPublish = 0;

// --------------------------------------------------------------

void showCurrentFile(){
  const char *path = __FILE__;
  const char *file = strrchr(path, '/');
  if (!file) file = strrchr(path, '\\'); // Windows path
  if (file) file++; // Skip separator

  Serial.print("Current file: ");
  Serial.println(file);
}



void setup() {
  Serial.begin(115200);

  showCurrentFile(); // Show the current filename

  // Build dynamic MQTT topics using device serial number
  snprintf(locationTopic, sizeof(locationTopic), "wjp/bus/%s/location", DEVICE_SERIAL);
  snprintf(routeUpdateTopic, sizeof(routeUpdateTopic), "wjp/bus/%s/route/update", DEVICE_SERIAL);
  snprintf(routeReqTopic, sizeof(routeReqTopic), "wjp/bus/%s/route/req", DEVICE_SERIAL);
  snprintf(routeCurrentTopic, sizeof(routeCurrentTopic), "wjp/bus/%s/route/current", DEVICE_SERIAL);

  Serial.println("=== Device Configuration ===");
  Serial.print("Serial Number: ");
  Serial.println(DEVICE_SERIAL);
  Serial.print("Location Topic: ");
  Serial.println(locationTopic);
  Serial.print("Route Update Topic: ");
  Serial.println(routeUpdateTopic);
  Serial.print("Route Request Topic: ");
  Serial.println(routeReqTopic);
  Serial.print("Route Current Topic: ");
  Serial.println(routeCurrentTopic);
  Serial.println("============================");

  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

  lcd.init();
  lcd.backlight();
  EEPROM.begin(3);

  // Load stored route
  for (byte i = 0; i < 3; i++) {
    char c = EEPROM.read(i);
    route[i] = (c >= '0' && c <= '9') ? c : '-';
  }

  render();
  lastKey = millis();

  // --------- Setup GPS --------------
  GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // --------- Setup SIM800 -----------
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(500);

  modem.restart();
  delay(500);

  // Wait for SIM
  unsigned long start = millis();
  while (modem.getSimStatus() != SIM_READY) {
    if (millis() - start > 15000UL) break;
    delay(500);
  }

  // Wait network
  modem.waitForNetwork();

  // GPRS
  modem.gprsConnect(apn, user, pass);

  // Set MQTT callback before connecting
  mqtt.setServer(mqttServer, mqttPort);
  mqtt.setCallback(mqttCallback);

  // Use device serial in client ID for uniqueness
  clientId = "esp32-" + String(DEVICE_SERIAL) + "-" + String(random(0xffff), HEX);
  mqttConnect();
  
  // Request current route from server
  requestCurrentRoute();
}

void loop() {
  unsigned long now = millis();

  // ---------------------------------------------------------
  // BACKLIGHT handling
  // ---------------------------------------------------------
  if (now - lastKey >= BACKLIGHT_TIMEOUT) {
    digitalWrite(BACKLIGHT_PIN, LOW);
  }

  // AUTO cancel edit
  if (editMode && now - editStart >= BACKLIGHT_TIMEOUT)
    cancelEdit();

  // BLINK cursor
  if (editMode && bufIdx < 3 && now - lastBlink >= 500) {
    lastBlink = now;
    blinkOn = !blinkOn;
    renderRow1();
  }

  // Clear message
  if (showMsg && now - msgStart >= 1200) {
    showMsg = false;
    lcd.setCursor(0, 3);
    if (editMode)
      lcd.print("Save (D), Clear (C) ");
    else
      centerText("Press A to edit", 3);
  }

  // ---------------------------------------------------------
  // KEYPAD input
  // ---------------------------------------------------------
  char key = keypad.getKey();
  if (key) {
    lastKey = now;
    digitalWrite(BACKLIGHT_PIN, HIGH);

    if (editMode) editStart = now;

    if (key == 'A' && !editMode) {
      editMode = true;
      bufIdx = 0;
      memcpy(bufferIn, "---", 4);
      editStart = now;
      render();
    }
    else if (key == 'B' && editMode) {
      cancelEdit();
    }
    else if (key >= '0' && key <= '9' && bufIdx < 3) {
      bufferIn[bufIdx++] = key;
      renderRow1();
    }
    else if (key == 'C') {
      bufIdx = 0;
      memcpy(bufferIn, "---", 4);
      renderRow1();
    }
    else if (key == 'D') {
      if (bufIdx == 3) {
        memcpy(route, bufferIn, 3);
        for (byte i = 0; i < 3; i++) EEPROM.write(i, route[i]);
        EEPROM.commit();
        editMode = false;
        render();
        showMessage("Saved");

        // -------------------------------
        // PUBLISH ROUTE JSON HERE
        // -------------------------------
        publishRoute();

      } else {
        showMessage("Fill all 3 digits");
      }
    }
  }

  // ---------------------------------------------------------
  // MQTT housekeeping
  // ---------------------------------------------------------
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  // ---------------------------------------------------------
  // GPS reading
  // ---------------------------------------------------------
  while (GPS.available()) gps.encode(GPS.read());

  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }

  // ---------------------------------------------------------
  // Publish GPS JSON every 30s
  // ---------------------------------------------------------
  if (millis() - lastPublish >= publishInterval) {
    lastPublish = millis();

    StaticJsonDocument<200> doc;
    doc["lt"] = latitude;
    doc["ln"] = longitude;

    char buf[200];
    size_t n = serializeJson(doc, buf);

    mqtt.publish(locationTopic, buf, n);
  }

  delay(5);
}

// -------------------------------------------------------------------
// MQTT CONNECT helper
// -------------------------------------------------------------------
bool mqttConnect() {
  if (!mqtt.connect(clientId.c_str())) return false;
  
  // Subscribe to route/current topic
  mqtt.subscribe(routeCurrentTopic);
  
  Serial.println("MQTT Connected!");
  Serial.print("Client ID: ");
  Serial.println(clientId);
  Serial.print("Subscribed to: ");
  Serial.println(routeCurrentTopic);
  
  return true;
}

// -------------------------------------------------------------------
// Request current route from server on startup
// -------------------------------------------------------------------
void requestCurrentRoute() {
  mqtt.publish(routeReqTopic, "1");
  
  Serial.print("Requested current route on topic: ");
  Serial.println(routeReqTopic);
}

// -------------------------------------------------------------------
// MQTT Callback - Handle incoming messages
// -------------------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  
  // Check if it's the route/current topic
  if (strcmp(topic, routeCurrentTopic) == 0) {
    // Parse JSON payload
    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Extract route
    const char* newRoute = doc["route"];
    
    if (newRoute && strlen(newRoute) == 3) {
      // Validate that all characters are digits
      bool valid = true;
      for (int i = 0; i < 3; i++) {
        if (newRoute[i] < '0' || newRoute[i] > '9') {
          valid = false;
          break;
        }
      }
      
      if (valid) {
        // Update route in memory
        memcpy(route, newRoute, 3);
        
        // Save to EEPROM
        for (byte i = 0; i < 3; i++) {
          EEPROM.write(i, route[i]);
        }
        EEPROM.commit();
        
        // Update display
        render();
        showMessage("Route synced");
        
        Serial.print("Route updated to: ");
        Serial.println(newRoute);
      } else {
        Serial.println("Invalid route format - must be 3 digits");
      }
    } else {
      Serial.println("Invalid route data");
    }
  }
}

// -------------------------------------------------------------------
// Publish saved route immediately after saving
// -------------------------------------------------------------------
void publishRoute() {
  StaticJsonDocument<100> doc;
  String s = "";
  s += route[0];
  s += route[1];
  s += route[2];

  doc["route"] = s;

  char out[100];
  size_t len = serializeJson(doc, out);

  mqtt.publish(routeUpdateTopic, (uint8_t*)out, len, true); // retained
  
  Serial.print("Published route to topic: ");
  Serial.println(routeUpdateTopic);
  Serial.print("Payload: ");
  Serial.println(out);
}

// ---------------- LCD FUNCTIONS ------------------------

void render() {
  lcd.clear();

  if (editMode) {
    lcd.setCursor(0, 0); lcd.print("Enter new route code");
    renderRow1();
    lcd.setCursor(0, 2); lcd.print("Cancel (B)");
    lcd.setCursor(0, 3); lcd.print("Save (D), Clear (C)");
  } else {
    centerText("Current route:", 0);
    renderRow1();
    lcd.setCursor(0, 2); lcd.print("                    ");
    centerText("Press A to edit", 3);
  }
}

void renderRow1() {
  char display[8];
  const char* src = editMode ? bufferIn : route;

  display[0] = src[0];
  display[1] = ' ';
  display[2] = src[1];
  display[3] = ' ';
  display[4] = src[2];
  display[5] = '\0';

  if (editMode && blinkOn && bufIdx < 3) {
    display[bufIdx * 2] = '_';
  }

  byte col = (20 - 5) / 2;
  lcd.setCursor(col, 1);
  lcd.print(display);
}

void showMessage(const char* txt) {
  msg = txt;
  showMsg = true;
  msgStart = millis();
  centerText(txt, 3);
}

void centerText(const char* txt, byte row) {
  byte len = strlen(txt);
  byte col = (20 - len) / 2;
  lcd.setCursor(col, row);
  lcd.print(txt);
  for (byte i = col + len; i < 20; i++) lcd.print(' ');
}

void cancelEdit() {
  editMode = false;
  bufIdx = 0;
  memcpy(bufferIn, "---", 4);
  render();
}