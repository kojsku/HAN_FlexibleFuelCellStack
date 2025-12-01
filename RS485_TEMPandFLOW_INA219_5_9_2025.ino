/*

  Data coms - Code to receive data from the INA219 and RS485-Bronkhorst Flexiflow

  Created by Kristupas Liudvikas Naudolaitis & Emils Kukojs, December 1, 2025.
  
*/
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// ----------- Modbus (Sketch 1) ------------------
const int rs485Rx = 10; // Receive Pin 
const int rs485Tx = 5; // Transmit Pin

SoftwareSerial rs485(rs485Rx, rs485Tx); // Creates Virtual serial object

#define MAX485_DE_RE 4 // Write/Read switch for communication
ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, 1); // Transmit mode
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, 0); // Receiving mode
}

// ----------- INA219 (Sketch 2) ------------------
Adafruit_INA219 ina219_B(0x40); //(default 0x40)

// No clue what these are:
const uint16_t ticks[] = {5000, 1000, 2000, 1000}; // purgevalve behaviour, i.e. 5000ms off, 1000ms on, 2000ms off, 1000 ms on 
const uint8_t nbTicks = sizeof(ticks) / sizeof(ticks[0]);
uint8_t currentTick = 0;
uint32_t previousMillis = 0; // millis is used for energy calculations

unsigned long previousTime = 0;
const unsigned long eventInterval = 1000;

float INA219_B_shuntvoltage = 0;
float INA219_B_busvoltage = 0;
float INA219_B_current_mA = 0;
float INA219_B_loadvoltage = 0;
float INA219_B_power_mW = 0;
float INA219_B_power_batt_mW = 0;
float INA219_B_energy = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1); // Wait for serial port to open

  // ----- Modbus Setup -----
  pinMode(MAX485_DE_RE, OUTPUT); // Makes this digital pin an output
  digitalWrite(MAX485_DE_RE, 0); // Turns on Receiving mode
  rs485.begin(19200); // Baud rate for the Bronkhorst - default 19200
  node.begin(1, rs485); // Slave ID - default 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // ----- INA219 Setup -----
  if (!ina219_B.begin()) { 
    Serial.println(F("Failed to find INA219 chip B"));
    while (1) delay(10);
  }
  Serial.println(F("INA219 Connected..."));
}

void loop() {
  // ---------- Read Modbus Temperature ----------
  uint8_t result;
  float temperature = 0.0;
  float flow = 0.0;

  result = node.readHoldingRegisters(0xA138, 2); // Modbus command (adress, 2 registers)
  if (result == node.ku8MBSuccess) {
    uint16_t r0 = node.getResponseBuffer(0) & 0x7FFF; // no fucking clue what 0x7FFF is
    uint16_t r1 = node.getResponseBuffer(1) & 0x7FFF; // AI says it forces the first bit to be 0, idk why we do this here but fuck it we ball
    uint32_t bits = ((uint32_t)r0 << 16) | r1;  // combines the two registers or smth like that idrk
    memcpy(&temperature, &bits, sizeof(float)); // memory copy
  } else {
    Serial.print("Temp read error: ");
    Serial.println(result, HEX);
  }

  delay(50); // Was 10, put it higher to prevent errors if the sys is too slow

  // ---------- Read Modbus Flow ----------
  result = node.readHoldingRegisters(0x0020, 1); // Sends a modbus command (hex adress, registers to read)
  if (result == node.ku8MBSuccess) {
    uint16_t rawFlow = node.getResponseBuffer(0);
    flow = float(rawFlow) / 16.0; // IDK why its divided by 16, its just a % where 0-32000 is 0 to 100% 
  } else {
    Serial.print("Flow read error: ");
    Serial.println(result, HEX);
  }

  delay(50);

  // ---------- Read INA219 Values ----------
  INA219_B_shuntvoltage = ina219_B.getShuntVoltage_mV();
  INA219_B_busvoltage = ina219_B.getBusVoltage_V();
  INA219_B_current_mA = ina219_B.getCurrent_mA();
  INA219_B_power_mW = ina219_B.getPower_mW();
  INA219_B_loadvoltage = INA219_B_busvoltage + (INA219_B_shuntvoltage / 1000);
  INA219_B_power_batt_mW = INA219_B_loadvoltage * INA219_B_current_mA;


  unsigned long currentTime = millis(); // Calculates total energy consumed (+=)
  if (currentTime - previousTime >= eventInterval) {
    INA219_B_energy += ((currentTime - previousTime) * INA219_B_power_batt_mW) / 1000000.0; 
    previousTime = currentTime;
  }

  // ---------- Output All Data ----------
  Serial.print("Temperature: ");
  Serial.print(temperature, 2); // 2 for decimal spaces
  Serial.print(" °C\t");

  Serial.print("Flow: ");
  Serial.print(flow, 2);
  Serial.print(" mln/min\t");
  Serial.print("H2 power in ");
  Serial.print((flow*12.7)/60, 2);
  Serial.println(" W");

  Serial.print("Battery Voltage: ");
  Serial.print(INA219_B_loadvoltage); Serial.print(" V\t");

  Serial.print("Battery Current: ");
  Serial.print(INA219_B_current_mA/1000); Serial.print(" A\t");

  Serial.print("Battery Power: ");
  Serial.print(INA219_B_power_batt_mW/1000); Serial.print(" W\t");

  Serial.print("Battery Energy: ");
  Serial.print(INA219_B_energy); Serial.println(" J");

  Serial.print(currentTick); Serial.println(" ms");


  delay(1000);
  rs485.flush();
}
