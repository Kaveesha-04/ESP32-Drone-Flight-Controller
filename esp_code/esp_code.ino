#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- Configuration ---
#define CE_PIN 4
#define CSN_PIN 5
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

// Radio Setup
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// OLED Setup
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// Data Structure (Must match Drone side exactly)
struct __attribute__((packed)) DataPacket {
  int throttle;
  int yaw;
  int roll;
  int pitch;
};

DataPacket JS;
bool isArmed = false;
unsigned long lastDisplayTime = 0;

void setup() {
  // 1. Initialize Serial Communication
  Serial.begin(115200);        // USB Debugging
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Nano Communication (RX=16, TX=17)

  // 2. Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); 
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println("BOOTING SYSTEM...");
  display.display();

  // 3. Initialize Radio
  SPI.begin(18, 19, 23, 5); // Force SPI Pins
  if (!radio.begin()) {
    Serial.println("RADIO ERROR: Hardware not found!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("RADIO ERROR!");
    display.println("Check Wiring/Power");
    display.display();
  } else {
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);  // Low power to prevent EMI glitches
    radio.setDataRate(RF24_1MBPS);  // Standard speed for stability
    radio.stopListening();          // Transmitter mode
    Serial.println("RADIO SUCCESS: Linked.");
  }
  
  delay(1000);
}

void loop() {
  // --- 1. RECEIVE ARM/DISARM FROM NANO ---
  // Look for the "!!!" header to prevent electrical noise glitches
  while (Serial2.available() > 0) {
    if (Serial2.peek() == '!') { 
      if (Serial2.available() >= 4) {
        Serial2.read(); Serial2.read(); Serial2.read(); // Discard '!!!'
        char cmd = Serial2.read();
        if (cmd == 'A') isArmed = true;
        if (cmd == 'D') isArmed = false;
      } else {
        break; // Wait for the full packet to arrive
      }
    } else {
      Serial2.read(); // Clear buffer of non-'!' noise
    }
  }

  // --- 2. PROCESS JOYSTICKS ---
  if (isArmed) {
    // Read and apply Exponential Smoothing (0.7 old, 0.3 new)
    // This removes the "twitchy" movement from electrical noise
    int rawT = map(analogRead(36), 0, 4095, 100, 0); // Throttle
    JS.throttle = (JS.throttle * 0.7) + (rawT * 0.3);

    int rawY = map(analogRead(39), 0, 4095, 0, 100); // Yaw
    JS.yaw = (JS.yaw * 0.7) + (rawY * 0.3);

    int rawP = map(analogRead(32), 0, 4095, 100, 0); // Pitch
    JS.pitch = (JS.pitch * 0.7) + (rawP * 0.3);

    int rawR = map(analogRead(33), 0, 4095, 0, 100); // Roll
    JS.roll = (JS.roll * 0.7) + (rawR * 0.3);
  } else {
    // Safety Neutrals when Disarmed
    JS.throttle = 0;
    JS.yaw = 50;
    JS.pitch = 50;
    JS.roll = 50;
  }

  // --- 3. SEND DATA TO DRONE ---
  // This sends the packet over the air as fast as possible
  radio.write(&JS, sizeof(DataPacket));

  // --- 4. ASYNC DISPLAY UPDATE ---
  // Refresh screen only every 250ms to keep the loop fast
  if (millis() - lastDisplayTime > 250) {
    updateOLED();
    lastDisplayTime = millis();
  }
}

void updateOLED() {
  display.clearDisplay();
  display.setCursor(0, 0);
  
  if (isArmed) {
    display.setTextSize(1);
    display.println("--- [ ARMED ] ---");
    display.printf("\nTHR:%d  YAW:%d", JS.throttle, JS.yaw);
    display.printf("\nROL:%d  PIT:%d", JS.roll, JS.pitch);
  } else {
    display.setTextSize(2);
    display.setCursor(15, 25);
    display.print("DISARMED");
  }
  display.display();
}