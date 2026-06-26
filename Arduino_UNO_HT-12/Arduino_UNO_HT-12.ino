/*
 * HC-12 Master Diagnostic, Recovery, & Compatibility Tool
 * Copyright (c) 2026 Alexander Fuchs
 * Licensed under the MIT License.
 */

/*
 * Hardware Wiring Diagram
 * 
 * To automate testing and individual programming, you must connect the SET pins 
 * of both modules to the Arduino. This allows the code to pull them LOW for 
 * programming and push them HIGH for communication.
 *
 * Module: ARDUINO UNO
 * Adapt for any other microcontroller development boards
 * 
 * HC-12 Module A:
 *   VCC -> 5V (or 3.3V with a capacitor)
 *   GND -> GND
 *   RXD -> Arduino Pin 3 (TX_A)
 *   TXD -> Arduino Pin 2 (RX_A)
 *   SET -> Arduino Pin 6 (SET_A)
 * 
 * HC-12 Module B:
 *   VCC -> 5V (or 3.3V with a capacitor)
 *   GND -> GND
 *   RXD -> Arduino Pin 5 (TX_B)
 *   TXD -> Arduino Pin 4 (RX_B)
 *   SET -> Arduino Pin 7 (SET_B)
 */

#include <SoftwareSerial.h>

// Module A Pins
const int RX_A = 2;
const int TX_A = 3;
const int SET_A = 6;

// Module B Pins
const int RX_B = 4;
const int TX_B = 5;
const int SET_B = 7;

SoftwareSerial hc12A(RX_A, TX_A);
SoftwareSerial hc12B(RX_B, TX_B);

// --- Timing & config constants ---
const long DEFAULT_BAUD          = 9600;   // standard HC-12 operating baud
const long FU4_BAUD              = 1200;   // HC-12 FU4 only supports 1200bps UART
const int  SET_PIN_SETTLE_MS     = 100;    // delay after toggling SET pin
const int  AT_CMD_DELAY_MS       = 250;    // time to wait for an AT command reply
const int  CONFIG_READ_DELAY_MS  = 250;    // time to wait for AT+RX full config dump
const int  RECOVERY_SETTLE_MS    = 200;    // SET settle time during emergency recovery
const int  RECOVERY_CMD_DELAY_MS = 300;    // AT command delay during emergency recovery
const int  PING_TIMEOUT_MS       = 500;    // ping timeout for FU1 (9600 bps air rate)
// FIX 1: FU4 uses 250 bps over-the-air. A 9-char packet takes ~360 ms to transmit
// alone, leaving no margin at 500 ms. 2000 ms gives comfortable headroom.
const int  PING_TIMEOUT_FU4_MS   = 2000;  // ping timeout for FU4 (250 bps air rate)

const String DEFAULT_CHANNEL = "C003";
const String DEFAULT_POWER   = "P3";

void setup() {
  pinMode(SET_A, OUTPUT);
  pinMode(SET_B, OUTPUT);

  digitalWrite(SET_A, HIGH);
  digitalWrite(SET_B, HIGH);

  Serial.begin(9600);
  hc12A.begin(DEFAULT_BAUD);
  hc12B.begin(DEFAULT_BAUD);

  delay(1000);
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char choice = Serial.read();

    delay(5);
    while (Serial.available()) Serial.read();

    switch (choice) {
      case '1':
        readIndividualConfiguration("A", hc12A, SET_A);
        break;
      case '2':
        readIndividualConfiguration("B", hc12B, SET_B);
        break;
      case '3':
        compareConfigurations();
        break;
      case '4':
        programIndividualModule("A", hc12A, SET_A, "FU1");
        printMenu();
        break;
      case '5':
        programIndividualModule("B", hc12B, SET_B, "FU1");
        printMenu();
        break;
      case '6':
        programBothModules();
        break;
      case '7':
        runEmergencyBaudRecovery();
        break;
      case '8':
        runAdvancedSoftwareDiagnostic();
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
  Serial.println(F("    HC-12 MASTER DIAGNOSTIC TOOL    "));
  Serial.println(F("===================================="));
  Serial.println(F("1) Read Configuration: Module A"));
  Serial.println(F("2) Read Configuration: Module B"));
  Serial.println(F("3) Compare Module Settings & Compatibility"));
  Serial.println(F("4) Program Module A (9600, C003, P3, FU1)"));
  Serial.println(F("5) Program Module B (9600, C003, P3, FU1)"));
  Serial.println(F("6) Program BOTH Modules sequentially"));
  Serial.println(F("7) Emergency Recovery (Force Stuck 1200 -> 9600)"));
  Serial.println(F("8) Run Advanced Auto-Fault Detection (FU1/FU4)"));
  Serial.println(F("===================================="));
  Serial.print(F("Enter selection (1-8): "));
}

String fetchConfigString(SoftwareSerial &module, int setPin) {
  digitalWrite(setPin, LOW);
  delay(SET_PIN_SETTLE_MS);
  module.listen();

  module.print("AT+RX");
  delay(CONFIG_READ_DELAY_MS);

  String response = "";
  while (module.available()) {
    response += (char)module.read();
  }

  digitalWrite(setPin, HIGH);
  delay(SET_PIN_SETTLE_MS);

  response.replace("\r", "");
  response.replace("\n", " ");
  response.trim();
  return response;
}

void readIndividualConfiguration(String label, SoftwareSerial &module, int setPin) {
  Serial.print(F("\n--- Reading Configuration Module "));
  Serial.print(label);
  Serial.println(F(" ---"));

  String config = fetchConfigString(module, setPin);

  Serial.print(F("Raw Response: "));
  if (config.length() > 0) {
    Serial.println(config);
  } else {
    Serial.println(F("[No response/Timeout]"));
  }
  printMenu();
}

void compareConfigurations() {
  Serial.println(F("\n--- Running Configuration Comparison ---"));

  String configA = fetchConfigString(hc12A, SET_A);
  String configB = fetchConfigString(hc12B, SET_B);

  Serial.print(F("Module A Settings: ")); Serial.println(configA.length() > 0 ? configA : F("TIMEOUT"));
  Serial.print(F("Module B Settings: ")); Serial.println(configB.length() > 0 ? configB : F("TIMEOUT"));
  Serial.println(F("---------------------------------------------------"));

  if (configA.length() == 0 || configB.length() == 0) {
    Serial.println(F("COMPATIBILITY VERDICT: UNKNOWN (One or both modules timed out)"));
    Serial.println(F("-> If a module is stuck in FU4 mode, run Option 7 (Emergency Recovery) first."));
    printMenu();
    return;
  }

  bool baudMatch = false;
  if (configA.indexOf("B9600") >= 0 && configB.indexOf("B9600") >= 0) baudMatch = true;
  else if (configA.indexOf("B1200") >= 0 && configB.indexOf("B1200") >= 0) baudMatch = true;
  else if (configA.indexOf("B19200") >= 0 && configB.indexOf("B19200") >= 0) baudMatch = true;
  else if (configA.indexOf("B38400") >= 0 && configB.indexOf("B38400") >= 0) baudMatch = true;
  else if (configA.indexOf("B115200") >= 0 && configB.indexOf("B115200") >= 0) baudMatch = true;

  bool modeMatch = false;
  if (configA.indexOf("FU1") >= 0 && configB.indexOf("FU1") >= 0) modeMatch = true;
  else if (configA.indexOf("FU2") >= 0 && configB.indexOf("FU2") >= 0) modeMatch = true;
  else if (configA.indexOf("FU3") >= 0 && configB.indexOf("FU3") >= 0) modeMatch = true;
  else if (configA.indexOf("FU4") >= 0 && configB.indexOf("FU4") >= 0) modeMatch = true;

  int idxA = configA.indexOf("C");
  int idxB = configB.indexOf("C");
  bool channelMatch = false;
  if (idxA >= 0 && idxB >= 0) {
    String chanA = configA.substring(idxA, idxA + 4);
    String chanB = configB.substring(idxB, idxB + 4);
    if (chanA == chanB) channelMatch = true;
  }

  if (baudMatch && modeMatch && channelMatch) {
    Serial.println(F("COMPATIBILITY VERDICT: PERFECT SETTINGS MATCH!"));
    Serial.println(F("-> Both modules are on the same channel, baud rate, and mode."));
    Serial.println(F("-> Wireless over-the-air communication SHOULD work perfectly."));
  } else {
    Serial.println(F("COMPATIBILITY VERDICT: MISMATCH DETECTED! Communication impossible."));
    if (!baudMatch)    Serial.println(F("   [!] Serial Baud Rates do not match."));
    if (!channelMatch) Serial.println(F("   [!] Working Wireless Channels (Cxxx) do not match."));
    if (!modeMatch)    Serial.println(F("   [!] Air Transmission Modes (FUx) do not match."));
    Serial.println(F("-> Run Option 6 to re-sync parameters to a matching baseline."));
  }
  printMenu();
}

bool programIndividualModule(String label, SoftwareSerial &module, int setPin, String mode) {
  Serial.print(F("\n--- Programming Module "));
  Serial.print(label);
  Serial.println(F(" ---"));

  String targetBaud = (mode == "FU4") ? String(FU4_BAUD) : String(DEFAULT_BAUD);

  digitalWrite(setPin, LOW);
  delay(SET_PIN_SETTLE_MS);
  module.listen();

  bool bOK = sendATCommand(module, "AT+B" + targetBaud, "OK+B" + targetBaud);
  bool cOK = sendATCommand(module, "AT+" + DEFAULT_CHANNEL, "OK+" + DEFAULT_CHANNEL);
  bool pOK = sendATCommand(module, "AT+" + DEFAULT_POWER, "OK+" + DEFAULT_POWER);
  bool mOK = sendATCommand(module, "AT+" + mode, "OK+" + mode);

  digitalWrite(setPin, HIGH);
  delay(SET_PIN_SETTLE_MS);

  bool success = (bOK && cOK && pOK && mOK);
  Serial.print(F("Module "));
  Serial.print(label);
  Serial.println(success ? F(": SUCCESS") : F(": FAILURE"));
  return success;
}

void programBothModules() {
  Serial.println(F("\n--- Programming BOTH Modules sequentially ---"));
  programIndividualModule("A", hc12A, SET_A, "FU1");
  programIndividualModule("B", hc12B, SET_B, "FU1");
  printMenu();
}

void runEmergencyBaudRecovery() {
  Serial.println(F("\n--- Executing Emergency 1200-Baud Recovery ---"));

  digitalWrite(SET_A, LOW);
  digitalWrite(SET_B, LOW);
  delay(RECOVERY_SETTLE_MS);

  hc12A.begin(FU4_BAUD); // 1200bps - the universal fallback for stuck modules
  hc12B.begin(FU4_BAUD);
  delay(SET_PIN_SETTLE_MS);

  hc12A.listen();
  Serial.println(F("\nResetting Module A Memory registers..."));
  hc12A.print("AT+B9600");  delay(RECOVERY_CMD_DELAY_MS);  clearRecoveryBuffer(hc12A);
  hc12A.print("AT+FU1");    delay(RECOVERY_CMD_DELAY_MS);  clearRecoveryBuffer(hc12A);

  hc12B.listen();
  Serial.println(F("\nResetting Module B Memory registers..."));
  hc12B.print("AT+B9600");  delay(RECOVERY_CMD_DELAY_MS);  clearRecoveryBuffer(hc12B);
  hc12B.print("AT+FU1");    delay(RECOVERY_CMD_DELAY_MS);  clearRecoveryBuffer(hc12B);

  digitalWrite(SET_A, HIGH);
  digitalWrite(SET_B, HIGH);
  delay(SET_PIN_SETTLE_MS);

  hc12A.begin(DEFAULT_BAUD);
  hc12B.begin(DEFAULT_BAUD);

  Serial.println(F("\nRecovery pass execution sequence ended."));
  printMenu();
}

void clearRecoveryBuffer(SoftwareSerial &module) {
  while (module.available()) {
    Serial.print((char)module.read());
  }
  Serial.println();
}

void runAdvancedSoftwareDiagnostic() {
  Serial.println(F("\n--- Starting Advanced Auto-Fault Detection ---"));

  hc12A.begin(DEFAULT_BAUD);
  hc12B.begin(DEFAULT_BAUD);
  Serial.println(F("Configuring baseline profiles (FU1)..."));
  bool progA1 = programIndividualModule("A", hc12A, SET_A, "FU1");
  bool progB1 = programIndividualModule("B", hc12B, SET_B, "FU1");

  bool baselineAtoB = false;
  bool baselineBtoA = false;
  if (progA1 && progB1) {
    baselineAtoB = verifyWirelessLink(hc12A, hc12B, "PING_FU1_A", PING_TIMEOUT_MS);
    baselineBtoA = verifyWirelessLink(hc12B, hc12A, "PING_FU1_B", PING_TIMEOUT_MS);
  } else {
    Serial.println(F("Skipping FU1 link test: baseline programming failed."));
  }

  Serial.println(F("\nShifting to FU4 Stress Profiles..."));
  bool progA4 = programIndividualModule("A", hc12A, SET_A, "FU4");
  bool progB4 = programIndividualModule("B", hc12B, SET_B, "FU4");

  // FU4 only supports 1200bps on the UART side. Switch the local
  // SoftwareSerial objects to match now that the modules have exited
  // command mode and adopted the new setting.
  hc12A.begin(FU4_BAUD);
  hc12B.begin(FU4_BAUD);
  delay(SET_PIN_SETTLE_MS);

  bool stressAtoB = false;
  bool stressBtoA = false;
  if (progA4 && progB4) {
    // FIX 1: Use the longer FU4 timeout — 250 bps air rate needs ~360 ms just
    // to transmit a 9-char packet; PING_TIMEOUT_FU4_MS gives safe headroom.
    stressAtoB = verifyWirelessLink(hc12A, hc12B, "PING_FU4_A", PING_TIMEOUT_FU4_MS);
    stressBtoA = verifyWirelessLink(hc12B, hc12A, "PING_FU4_B", PING_TIMEOUT_FU4_MS);
  } else {
    Serial.println(F("Skipping FU4 link test: stress-profile programming failed."));
  }

  // FIX 2: Revert sequence — call listen() before sending AT commands to each
  // module. Without this, SoftwareSerial may still be listening on the other
  // module, causing AT commands to be silently dropped and responses to bleed
  // across, producing the spurious "ERROR / OK+FU1" output.
  Serial.println(F("\nReverting to stable baseline..."));
  digitalWrite(SET_A, LOW);
  digitalWrite(SET_B, LOW);
  delay(SET_PIN_SETTLE_MS);

  hc12A.listen();
  hc12A.print("AT+B9600"); delay(AT_CMD_DELAY_MS); clearRecoveryBuffer(hc12A);
  hc12A.print("AT+FU1");   delay(AT_CMD_DELAY_MS); clearRecoveryBuffer(hc12A);

  hc12B.listen();
  hc12B.print("AT+B9600"); delay(AT_CMD_DELAY_MS); clearRecoveryBuffer(hc12B);
  hc12B.print("AT+FU1");   delay(AT_CMD_DELAY_MS); clearRecoveryBuffer(hc12B);

  digitalWrite(SET_A, HIGH);
  digitalWrite(SET_B, HIGH);

  hc12A.begin(DEFAULT_BAUD);
  hc12B.begin(DEFAULT_BAUD);
  delay(SET_PIN_SETTLE_MS);

  Serial.println(F("\n================ DIAGNOSTIC REPORT ================"));
  Serial.print(F("Link A -> B (FU1): ")); Serial.println(baselineAtoB ? F("PASS") : F("FAIL"));
  Serial.print(F("Link B -> A (FU1): ")); Serial.println(baselineBtoA ? F("PASS") : F("FAIL"));
  Serial.print(F("Link A -> B (FU4): ")); Serial.println(stressAtoB ? F("PASS") : F("FAIL"));
  Serial.print(F("Link B -> A (FU4): ")); Serial.println(stressBtoA ? F("PASS") : F("FAIL"));
  Serial.println(F("---------------------------------------------------"));

  if (!(progA1 && progB1 && progA4 && progB4)) {
    Serial.println(F("AUTOMATIC VERDICT: INCONCLUSIVE - one or more AT programming steps failed."));
    Serial.println(F("-> Check wiring/SET pins, or run Option 7 (Emergency Recovery) first."));
  } else if (!baselineAtoB && baselineBtoA) {
    Serial.println(F("AUTOMATIC VERDICT: MODULE A HAS A DEAD TRANSMITTER (FAULTY)."));
  } else if (baselineAtoB && !baselineBtoA) {
    Serial.println(F("AUTOMATIC VERDICT: MODULE B HAS A DEAD TRANSMITTER (FAULTY)."));
  } else if (!baselineAtoB && !baselineBtoA) {
    Serial.println(F("AUTOMATIC VERDICT: TOTAL RF TRANSMISSION LINK REJECTION."));
  } else {
    Serial.println(F("AUTOMATIC VERDICT: LOGICAL ASSESSMENT CONCLUDED. SEE MATRIX ABOVE."));
  }
  printMenu();
}

// FIX 1: timeout parameter added so FU1 and FU4 pings can use different wait times.
bool verifyWirelessLink(SoftwareSerial &txModule, SoftwareSerial &rxModule, String packet, int timeout) {
  rxModule.listen();
  delay(10);
  txModule.print(packet);
  delay(timeout);

  String buffer = "";
  while (rxModule.available()) {
    buffer += (char)rxModule.read();
  }
  return (buffer.indexOf(packet) >= 0);
}

bool sendATCommand(SoftwareSerial &module, String command, String expectedResponse) {
  Serial.print(F("Sending: "));
  Serial.println(command);

  module.print(command);
  delay(AT_CMD_DELAY_MS);

  String response = "";
  while (module.available()) {
    response += (char)module.read();
  }

  Serial.print(F("Response: "));
  if (response.length() > 0) {
    Serial.println(response);
  } else {
    Serial.println(F("[No response/Timeout]"));
  }

  response.replace("\r", "");
  response.replace("\n", "");
  return (response.indexOf(expectedResponse) >= 0);
}
