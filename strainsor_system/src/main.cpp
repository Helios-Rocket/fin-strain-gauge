#include <Arduino.h>
#include <SPI.h>
#include <cmath>

const int cs = 15; 
const int drdy = 5; 
const int clk = 2; 
// Command definitions
#define NULL_COMMAND    0x000000
#define RESET_COMMAND   0x000011
#define STANDBY_COMMAND 0x000022
#define WAKEUP_COMMAND  0x000033
#define LOCK_COMMAND    0x000555
#define UNLOCK_COMMAND  0x000655

// Register addresses for read commands
#define STATUS_REG_READ 0x200100  // Read STATUS register (0x01)

// Register Addresses for write commands 
#define SET_CH1_WRITE_COMMAND 0x668000 //0110 0110 1000 0000 0000 0000 
#define SET_CH1_DATA 0x000200 // 0000 0000 0000 0010 0000 0000 

void resetChip(); 
void readADCChannels(); 
uint32_t sendCommand(uint32_t command); 
float convertADC2Temp(int adc_value); 

void setup() {

  Serial.begin(115200); 

  while(!Serial){}

  pinMode(cs, OUTPUT); 
  digitalWrite(cs, HIGH); 

  if (drdy != -1) {
    pinMode(drdy, INPUT);
  }

  // Set up ADC CLK 
  pinMode(clk, OUTPUT); 
  analogWrite(clk, 128); 
  analogWriteFrequency(clk, 4000000); 

  SPI.begin(); 
  delay(100); // Give some time for chip to power 

  resetChip(); 
  delay(10); 

  Serial.println("ADS131M03 Initialized (theoretically)");
  
}

void loop() {

  // Read status register
  sendCommand(STATUS_REG_READ);
  delay(1);
  uint32_t status = sendCommand(NULL_COMMAND);
  Serial.print("Status Register: 0x");
  Serial.println(status, BIN);

  // Set ADC test signals on CH 1
  // sendCommand(SET_CH1_WRITE_COMMAND); 
  // delay(1); 
  // sendCommand(SET_CH1_DATA); 
  // delay(1);

  // Read ADC channels
  while(true){
    readADCChannels();
    delay(100);
  }
 
}

uint32_t sendCommand(uint32_t command) { 
  uint32_t response = 0;
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 
  digitalWrite(cs, LOW);
  
  // Send 24-bit command (3 bytes)
  uint8_t byte2 = (command >> 16) & 0xFF;
  uint8_t byte1 = (command >> 8) & 0xFF;
  uint8_t byte0 = command & 0xFF;
  
  uint8_t resp2 = SPI.transfer(byte2);
  uint8_t resp1 = SPI.transfer(byte1);
  uint8_t resp0 = SPI.transfer(byte0);
  
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  
  // Combine response bytes
  response = ((uint32_t)resp2 << 16) | ((uint32_t)resp1 << 8) | resp0;
  
  return response;
}

void resetChip() {
  Serial.println("Resetting ADS131M03...");
  sendCommand(RESET_COMMAND);
  delay(10); // Wait for reset to complete
}

void readADCChannels() {
  // Wait for data ready (if DRDY pin is connected)
  if (drdy != -1) {
    while (digitalRead(drdy) == HIGH) {
      // Wait for DRDY to go low
    }
  }
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  
  uint32_t words[4]; // STATUS, CH0, CH1, CH2
  
  for (int i = 0; i < 4; i++) {
    // Send NULL command (0x000000) for each word
    uint8_t resp2 = SPI.transfer(0x00);
    // Serial.print("Resp2: "); 
    // Serial.println(resp2, BIN); 
    uint8_t resp1 = SPI.transfer(0x00);
    // Serial.print("Resp1: "); 
    // Serial.println(resp2, BIN); 
    uint8_t resp0 = SPI.transfer(0x00);
    // Serial.print("Resp0: "); 
    // Serial.println(resp2, BIN); 
    
    words[i] = ((uint32_t)resp2 << 16) | ((uint32_t)resp1 << 8) | resp0;
    delay(1); 
  }
  
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  
  Serial.println("=== ADC Reading ===");
  Serial.print("Status: 0x");
  Serial.println(words[0], BIN);
  
  for (int i = 1; i < 4; i++) {
    // Convert 24-bit two's complement to signed 32-bit
    int32_t adcValue = words[i];
    if (adcValue & 0x800000) { // Check sign bit
      adcValue |= 0xFF000000; // Sign extend
    }
    
    Serial.print("Channel ");
    Serial.print(i - 1);
    Serial.print(": ");
    Serial.print(adcValue);
    Serial.print(" (0x");
    Serial.print(words[i], HEX);
    Serial.println(")");

    if(i == 1){
      float temp = convertADC2Temp(adcValue); 
      Serial.print("Temperature Value in C: "); 
      Serial.println(temp); 
      Serial.println(); 
    }
  }
}

float convertADC2Temp(int adc_value){
  // adc_value = (float) adc_value * 2.4/pow(2, 24); 
  float adc_voltage = (1.2/(pow(2, 24)-1))*adc_value; 
  Serial.print("Ref voltage: " ); 
  Serial.println(adc_voltage); 
  return (adc_voltage - 0.5)*100; 
}


