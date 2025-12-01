/*

  Connection check - Code to check if there is a connection between Arduino and Bronkhorst

  Created by AI, Copied by Emils Kukojs, December 1, 2025.
  
*/
#include <SoftwareSerial.h>
#include <ModbusMaster.h>

// --- PIN CONFIGURATION ---
// Based on your previous setup
#define MAX485_DE_RE 4  
#define RX_PIN 10       
#define TX_PIN 11       

SoftwareSerial rs485Serial(RX_PIN, TX_PIN);
ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH); // TALK
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW);  // LISTEN
}

void setup() {
  // 1. Setup Control Pin
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  // 2. Setup PC Serial
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for monitor to open
  Serial.println("--- MODBUS CONNECTION TESTER ---");

  // 3. Setup RS485 Serial (TRY 38400 FIRST)
  rs485Serial.begin(38400); 

  // 4. Initialize Modbus
  node.begin(1, rs485Serial); // Slave ID 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  Serial.print("Sending Ping to ID=1 at 38400 baud... ");

  // Try to read just 1 register at address 1 (Safe Read)
  uint8_t result = node.readHoldingRegisters(1, 1);

  if (result == node.ku8MBSuccess) {
    Serial.println("SUCCESS! Device Found.");
    Serial.print("Response: ");
    Serial.println(node.getResponseBuffer(0));
    Serial.println("-----------------------------------");
  } else {
    Serial.print("FAILED. Error Code: 0x");
    Serial.println(result, HEX);
    
    if (result == 0xE2) Serial.println(" -> (E2) Timeout: Check Wiring, Power, or Baud Rate.");
    if (result == 0xE0) Serial.println(" -> (E0) Invalid ID: Device heard us but ignored us.");
  }

  delay(2000); // Try again every 2 seconds
}