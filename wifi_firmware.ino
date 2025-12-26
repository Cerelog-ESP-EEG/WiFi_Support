#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// ===  USER CONFIGURATION                ===
// ==========================================
String WIFI_SSID = ""; 
String WIFI_PASS = "";

// CHANGED: Using Port 4545 to avoid Mac OS conflicts with 8080
const int TCP_PORT = 4545;  
const int UDP_PORT = 4445;  
// ==========================================

WiFiServer server(TCP_PORT);
WiFiClient client; 
WiFiUDP udp;

// --- DEBUG MACROS ---
// #define DEBUG_ENABLED // Uncomment to debug

#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__) 
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// --- Packet/Protocol Global Variables ---
const uint8_t ADS1299_NUM_STATUS_BYTES = 3;
const uint8_t ADS1299_NUM_CHANNELS = 8;
const uint8_t ADS1299_BYTES_PER_CHANNEL = 3;
const uint8_t ADS1299_TOTAL_DATA_BYTES = ADS1299_NUM_STATUS_BYTES + (ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL);
const uint8_t PACKET_TIMESTAMP_BYTES = 4;
const uint8_t PACKET_START_MARKER_BYTES = 2;
const uint8_t PACKET_END_MARKER_BYTES = 2;
const uint8_t PACKET_LENGTH_FIELD_BYTES = 1;
const uint8_t PACKET_CHECKSUM_BYTES = 1;
const uint8_t PACKET_MSG_LENGTH = PACKET_TIMESTAMP_BYTES + ADS1299_TOTAL_DATA_BYTES;
const uint8_t PACKET_TOTAL_SIZE = PACKET_START_MARKER_BYTES + PACKET_LENGTH_FIELD_BYTES + PACKET_MSG_LENGTH + PACKET_CHECKSUM_BYTES + PACKET_END_MARKER_BYTES;
const uint8_t PACKET_IDX_START_MARKER = 0;
const uint8_t PACKET_IDX_LENGTH = PACKET_IDX_START_MARKER + PACKET_START_MARKER_BYTES;
const uint8_t PACKET_IDX_TIMESTAMP = PACKET_IDX_LENGTH + PACKET_LENGTH_FIELD_BYTES;
const uint8_t PACKET_IDX_ADS1299_DATA = PACKET_IDX_TIMESTAMP + PACKET_TIMESTAMP_BYTES;
const uint8_t PACKET_IDX_CHECKSUM = PACKET_IDX_ADS1299_DATA + ADS1299_TOTAL_DATA_BYTES;
const uint8_t PACKET_IDX_END_MARKER = PACKET_IDX_CHECKSUM + PACKET_CHECKSUM_BYTES;

// --- Pin Mapping ---
static const uint8_t pin_MOSI_NUM = 23;
static const uint8_t pin_CS_NUM = 5;
static const uint8_t pin_MISO_NUM = 19;
static const uint8_t pin_SCK_NUM = 18;
static const uint8_t pin_PWDN_NUM = 13;
static const uint8_t pin_RST_NUM = 12;
static const uint8_t pin_START_NUM = 14;
static const uint8_t pin_DRDY_NUM = 27;
static const uint8_t pin_LED_DEBUG = 17;

// --- SPI instance ---
SPIClass *vspi = NULL;

// --- ADS1299 State Management ---
int _ADS1299_MODE = -2;
int ADS1299_MODE_SDATAC = 1;
int ADS1299_MODE_RDATAC = 2;
int _ADS1299_PREV_CMD = -1;
int _CMD_ADC_WREG = 3;
int _CMD_ADC_RREG = 4;
int _CMD_ADC_SDATAC = 17;
int _CMD_ADC_RDATAC = 16;
int _CMD_ADC_START = 8;
static const int SPI_FREQ = 4000000;

// --- Interrupt Flag & Timestamp ---
volatile bool dataReady = false;
unsigned long _unix_timestamp_reference = 0;
unsigned long _millis_reference = 0;
bool _timestamp_initialized = false;

// --- Function Prototypes ---
void ADS1299_WREG(uint8_t regAdd, uint8_t *values, uint8_t numRegs);
void ADS1299_RREG(uint8_t regAdd, uint8_t *buffer, uint8_t numRegs);
void ADS1299_SETUP(void);
void ADS1299_SDATAC(void);
void ADS1299_RDATAC(void);
void ADS1299_START(void);
byte SPI_SendByte(byte data_byte, bool cont);
void read_ADS1299_data(byte *buffer);
void IRAM_ATTR onDRDYFalling(void);

// --- Register Setup ---
typedef struct Deez { int add; int reg_val; } regVal_pair;
const int size_reg_ls = 24;
static const regVal_pair ADS1299_REGISTER_LS[size_reg_ls] = {
  {0x01, 0b10110110}, {0x02, 0b11010000}, {0x03, 0b11101100}, {0x04, 0}, {-2, -2},
  {0x05, 0b01100000}, {0x06, 0b01100000}, {0x07, 0b01100000}, {0x08, 0b01100000},
  {0x09, 0b01100000}, {0x0A, 0b01100000}, {0x0B, 0b01100000}, {0x0C, 0b01100000},
  {0x0D, 0b11111111}, {0x0E, 0b00000000}, {0x0F, 0}, {0x10, 0}, {0x11, 0}, {-2, -2},
  {0x15, 0b00100000}, {0x16, 0}, {0x17, 0}
};

// --- Handshake from Computer ---
int handshake_packet_size = 12;
const uint8_t MSG_TYPE_TIMESTAMP = 0x02;
const uint8_t HANDSHAKE_START_MARKER_1 = 0xAA;
const uint8_t HANDSHAKE_START_MARKER_2 = 0xBB;
const uint8_t RING_BUFFER_SIZE = 24;
uint8_t ring_buffer[RING_BUFFER_SIZE];
uint8_t ring_head = 0, ring_tail = 0, ring_counter = 0;

// REPLACED Serial with client
bool waitForTimestamp() {
  if (!client || !client.connected()) return false;

  while (client.available() > 0 && ring_counter < RING_BUFFER_SIZE) {
      ring_buffer[ring_head] = client.read(); 
      ring_head = (ring_head + 1) % RING_BUFFER_SIZE; 
      ring_counter++;
  }
  if (ring_counter < handshake_packet_size) return false;
  
  for (uint8_t i = 0; i < handshake_packet_size + 3; i++) {
      uint8_t ring_index = (ring_tail + i) % RING_BUFFER_SIZE;
      if (ring_buffer[ring_index] == HANDSHAKE_START_MARKER_1 && ring_buffer[(ring_index + 1) % RING_BUFFER_SIZE] == HANDSHAKE_START_MARKER_2 && ring_buffer[(ring_index + 2) % RING_BUFFER_SIZE] == MSG_TYPE_TIMESTAMP) {
          uint32_t received_timestamp = (uint32_t)ring_buffer[(ring_index + 3) % RING_BUFFER_SIZE] << 24 | (uint32_t)ring_buffer[(ring_index + 4) % RING_BUFFER_SIZE] << 16 | (uint32_t)ring_buffer[(ring_index + 5) % RING_BUFFER_SIZE] << 8 | (uint32_t)ring_buffer[(ring_index + 6) % RING_BUFFER_SIZE];
          
          _unix_timestamp_reference = received_timestamp; 
          _millis_reference = millis(); 
          _timestamp_initialized = true;
          ring_counter = 0; ring_head = 0; ring_tail = 0;
          return true;
      }
  }
  return false;
}

void IRAM_ATTR onDRDYFalling(void) { dataReady = true; }

byte SPI_SendByte(byte data_byte, bool cont) {
  if (!cont) { digitalWrite(pin_CS_NUM, LOW); delayMicroseconds(1); }
  byte received = vspi->transfer(data_byte);
  if (!cont) { delayMicroseconds(1); digitalWrite(pin_CS_NUM, HIGH); }
  return received;
}

void ADS1299_WREG(uint8_t regAdd, uint8_t *values, uint8_t numRegs) {
  if (_ADS1299_MODE != ADS1299_MODE_SDATAC) ADS1299_SDATAC();
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); 
  SPI_SendByte(0b01000000 | regAdd, true);
  SPI_SendByte(numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) SPI_SendByte(values[i], true);
  delayMicroseconds(1); 
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_WREG;
}

void ADS1299_RREG(uint8_t regAdd, uint8_t *buffer, uint8_t numRegs) {
  if (_ADS1299_MODE != ADS1299_MODE_SDATAC) ADS1299_SDATAC();
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); 
  SPI_SendByte(0b00100000 | regAdd, true);
  SPI_SendByte(numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) buffer[i] = SPI_SendByte(0x00, true);
  delayMicroseconds(1); 
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_RREG;
}

void ADS1299_SDATAC(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); 
  SPI_SendByte(_CMD_ADC_SDATAC, true);
  delayMicroseconds(1); 
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_MODE = ADS1299_MODE_SDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_SDATAC;
}

void ADS1299_RDATAC(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); 
  SPI_SendByte(_CMD_ADC_RDATAC, true);
  delayMicroseconds(1); 
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_MODE = ADS1299_MODE_RDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_RDATAC;
}

void ADS1299_START(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); 
  SPI_SendByte(_CMD_ADC_START, true);
  delayMicroseconds(1); 
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_START;
}

void ADS1299_SETUP(void) {
  digitalWrite(pin_PWDN_NUM, LOW);
  digitalWrite(pin_RST_NUM, LOW);
  delay(100);
  digitalWrite(pin_PWDN_NUM, HIGH);
  digitalWrite(pin_RST_NUM, HIGH);
  delay(1000);
  ADS1299_SDATAC();
  uint8_t refbuf[] = {0b11101100};
  ADS1299_WREG(0x03, refbuf, 1);
  delay(10);
  uint8_t value[1];
  uint8_t i = 0;
  while (i < size_reg_ls) {
      const regVal_pair temp = ADS1299_REGISTER_LS[i];
      if (temp.add == -2) { i++; continue; }
      value[0] = {(uint8_t)temp.reg_val};
      ADS1299_WREG(temp.add, value, 1);
      delayMicroseconds(10);
      i++;
  }
}

void read_ADS1299_data(byte *buffer) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); 
  for (int i = 0; i < ADS1299_TOTAL_DATA_BYTES; i++) {
      buffer[i] = SPI_SendByte(0x00, true);
  }
  delayMicroseconds(1); 
  digitalWrite(pin_CS_NUM, HIGH);
}

void setup() {
  Serial.begin(9600); 
  delay(2000);
  Serial.println("\n\n--- CERELOG ESP32 WIFI START ---");

  if (WIFI_SSID == "") {
    Serial.println("!!! NO WIFI CREDENTIALS DETECTED !!!");
    Serial.println("Please type your WiFi SSID (Name) in the box above and hit Enter:");
    while (Serial.available() == 0) { delay(100); }
    WIFI_SSID = Serial.readStringUntil('\n');
    WIFI_SSID.trim(); 
    Serial.print("SSID set to: "); Serial.println(WIFI_SSID);

    Serial.println("Please type your WiFi PASSWORD in the box above and hit Enter:");
    while (Serial.available() == 0) { delay(100); }
    WIFI_PASS = Serial.readStringUntil('\n');
    WIFI_PASS.trim();
    Serial.println("Password set.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());
  WiFi.setSleep(false); 

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("SUCCESS! IP Address: ");
  Serial.println(WiFi.localIP()); 
  Serial.print("Discovery Port (UDP): "); Serial.println(UDP_PORT);
  Serial.print("Stream Port (TCP): "); Serial.println(TCP_PORT);
  
  server.begin();
  udp.begin(UDP_PORT); 
  
  pinMode(pin_PWDN_NUM, OUTPUT);
  pinMode(pin_RST_NUM, OUTPUT);
  pinMode(pin_START_NUM, OUTPUT);
  pinMode(pin_CS_NUM, OUTPUT);
  pinMode(pin_DRDY_NUM, INPUT_PULLUP);
  pinMode(pin_LED_DEBUG, OUTPUT);
  digitalWrite(pin_CS_NUM, HIGH);
  delay(2000);
  digitalWrite(pin_LED_DEBUG, LOW);

  vspi = new SPIClass(VSPI);
  vspi->begin(pin_SCK_NUM, pin_MISO_NUM, pin_MOSI_NUM, pin_CS_NUM);
  vspi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE1));
  delay(500);

  ADS1299_SETUP();

  _unix_timestamp_reference = 0;
  _millis_reference = millis();
  _timestamp_initialized = true;
 
  attachInterrupt(digitalPinToInterrupt(pin_DRDY_NUM), onDRDYFalling, FALLING);

  DEBUG_PRINTLN("Setup complete.");
  digitalWrite(pin_START_NUM, HIGH);
 
  delay(10);
  ADS1299_START();

  delay(20);
  ADS1299_RDATAC();

  digitalWrite(pin_LED_DEBUG, HIGH);
  Serial.println("System Ready. Waiting for Python connection...");
}

void loop() {
  // 1. Handle UDP Auto-Discovery (Port 4445)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    if (strstr(packetBuffer, "CERELOG_FIND_ME")) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print("CERELOG_HERE");
        udp.endPacket();
        Serial.println("Discovery Packet Received & Replied");
    }
  }

  // 2. Handle TCP Client (Port 4545)
  if (!client || !client.connected()) {
    client = server.available(); 
    if (client) {
      Serial.println("Client Connected via TCP!");
      client.setNoDelay(true); 
    }
  }

  // 3. Main Data Stream
  if (client && client.connected()) {
      if (client.available() >= 12) {
          if (waitForTimestamp()) {
             // Synced
          }     
      }
      
      if (dataReady) {
          dataReady = false;
     
          byte raw_data[ADS1299_TOTAL_DATA_BYTES];
          read_ADS1299_data(raw_data);

          const uint16_t START_MARKER = 0xABCD;
          const uint16_t END_MARKER = 0xDCBA;
          byte packet[PACKET_TOTAL_SIZE];

          packet[PACKET_IDX_START_MARKER] = (START_MARKER >> 8) & 0xFF;
          packet[PACKET_IDX_START_MARKER + 1] = START_MARKER & 0xFF;
          packet[PACKET_IDX_LENGTH] = PACKET_MSG_LENGTH;

         uint32_t millis_since_sync = millis() - _millis_reference;

         packet[PACKET_IDX_TIMESTAMP]     = (millis_since_sync >> 24) & 0xFF;
         packet[PACKET_IDX_TIMESTAMP + 1] = (millis_since_sync >> 16) & 0xFF;
         packet[PACKET_IDX_TIMESTAMP + 2] = (millis_since_sync >> 8) & 0xFF;
         packet[PACKET_IDX_TIMESTAMP + 3] =  millis_since_sync & 0xFF;

          for (uint8_t i = 0; i < ADS1299_TOTAL_DATA_BYTES; i++) {
              packet[PACKET_IDX_ADS1299_DATA + i] = raw_data[i];
          }
          uint8_t checksum = 0;
          for (uint8_t i = PACKET_IDX_LENGTH; i < PACKET_IDX_CHECKSUM; i++) {
              checksum += packet[i];
          }
          packet[PACKET_IDX_CHECKSUM] = checksum;
          packet[PACKET_IDX_END_MARKER] = (END_MARKER >> 8) & 0xFF;
          packet[PACKET_IDX_END_MARKER + 1] = END_MARKER & 0xFF;

          client.write(packet, sizeof(packet));
      }
  }
  // IMPORTANT FIX: Small delay to prevent WiFi stack starvation
  delay(1); 
}