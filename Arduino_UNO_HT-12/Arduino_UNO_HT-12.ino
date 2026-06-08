#include <SoftwareSerial.h>

// Assign non-conflicting pins for communication
const int HC12_RX = 4; // Connects to HC-12 TXD
const int HC12_TX = 5; // Connects to HC-12 RXD

SoftwareSerial HC12(HC12_RX, HC12_TX);

void setup() {
  Serial.begin(9600);   
  HC12.begin(9600);     
  
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char choice = Serial.read();
    
    delay(5);
    while(Serial.available()) Serial.read(); 

    bool success = false;

    switch (choice) {
      case '1':
        readSettings();
        break;
      case '2':
        success = setConfiguration("9600", "003", "3", "FU1");
        printStatus(success);
        break;
      default:
        Serial.println(F("\nInvalid selection. Try again."));
        printMenu();
        break;
    }
  }
}

void printMenu() {
  Serial.println(F("\n===================================="));
  Serial.println(F("    HC-12 PROGRAMMER   "));
  Serial.println(F("===================================="));
  Serial.println(F("1) Read Current Settings"));
  Serial.println(F("2) Program HC-12 modules (9600, C003, 5dBm, FU1)"));
  Serial.println(F("===================================="));
  Serial.print(F("Enter selection (1 or 2): "));
}

void readSettings() {
  Serial.println(F("\n--- Reading HC-12 Current Settings ---"));
  sendATCommand("AT+RX", "OK"); // Just looks for a general response
  printMenu();
}

// Programs settings and checks if ALL components succeeded
bool setConfiguration(String baud, String channel, String power, String mode) {
  Serial.println(F("\n--- Programming HC-12 Settings ---"));
  
  bool bOK = sendATCommand("AT+B" + baud, "OK+B" + baud);
  bool cOK = sendATCommand("AT+C" + channel, "OK+C" + channel);
  bool pOK = sendATCommand("AT+P" + power, "OK+P" + power);
  bool mOK = sendATCommand("AT+" + mode, "OK+" + mode);
  
  // Return true only if all 4 settings were successfully applied
  return (bOK && cOK && pOK && mOK);
}

void printStatus(bool success) {
  if (success) {
    Serial.println(F("Programming Complete: SUCCESS"));
  } else {
    Serial.println(F("Programming Complete: FAILURE"));
  }
  printMenu();
}

// Sends command, displays text, and verifies the incoming response
bool sendATCommand(String command, String expectedResponse) {
  Serial.print(F("Sending: "));
  Serial.println(command);
  
  HC12.print(command);
  delay(250); // Wait for HC-12 to process and send response buffer
  
  String response = "";
  while (HC12.available()) {
    char c = HC12.read();
    response += c;
  }
  
  // Show what the module answered back
  Serial.print(F("Response: "));
  if (response.length() > 0) {
    Serial.print(response);
  } else {
    Serial.println(F("[No response/Timeout]"));
  }
  
  // Clean up responses strings to prevent carriage return issues during comparison
  response.replace("\r", "");
  response.replace("\n", "");
  
  return (response.indexOf(expectedResponse) >= 0);
}
