#include <SoftwareSerial.h>

const int buttonPin = 15; // A0
SoftwareSerial mySerial(2, 3); // RX=2, TX=3

bool armed = false;
bool lastButtonState = HIGH;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  mySerial.begin(9600);
}

void loop() {
  bool currentButtonState = digitalRead(buttonPin);

  if (currentButtonState == LOW && lastButtonState == HIGH) {
    armed = !armed;
    
    // We send a "Packet" instead of a byte. 
    // Noise might look like 'A', but it rarely looks like '!!!A'
    if (armed) {
      for(int i=0; i<3; i++) mySerial.print('!'); // Header
      mySerial.print('A'); // Command
    } else {
      for(int i=0; i<3; i++) mySerial.print('!'); 
      mySerial.print('D');
    }
    
    delay(300); // Lockout
  }
  lastButtonState = currentButtonState;
}