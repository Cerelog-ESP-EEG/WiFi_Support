#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// ===  USER CONFIGURATION                ===
// ==========================================
String WIFI_SSID = ""; 
String WIFI_PASS = "";

const int TCP_PORT = 4545;
const int UDP_PORT = 4445;
// ==========================================

WiFiServer server(TCP_PORT);
WiFiClient client;
WiFiUDP udp;

// --- Pin Mapping ---
static const uint8_t pin_MOSI = 23;
static const uint8_t pin_MISO = 19;
static const uint8_t pin_SCK  = 18;
static const uint8_t pin_CS   = 5;
static const uint8_t pin_PWDN = 13;
static const uint8_t pin_RST  = 12;
static const uint8_t pin_START= 14;
static const uint8_t pin_DRDY = 27;
static const uint8_t pin_LED  = 17;

SPIClass *vspi = NULL;

// --- State Flags ---
volatile bool streaming_allowed = false;
volatile bool data_ready = false;
unsigned long _millis_reference = 0;

// --- Interrupt ---
void IRAM_ATTR onDRDY() {
    // If we aren't connected, do NOT run the interrupt logic.
    // This prevents the CPU from freezing.
    if (!streaming_allowed) return; 
    data_ready = true;
}

void setup() {
    Serial.begin(115200);
    
    // 1. Safe Pin Setup
    pinMode(pin_PWDN, OUTPUT); pinMode(pin_RST, OUTPUT);
    pinMode(pin_START, OUTPUT); pinMode(pin_CS, OUTPUT);
    pinMode(pin_DRDY, INPUT_PULLUP); pinMode(pin_LED, OUTPUT);

    // 2. Hardware Silence (Force OFF)
    digitalWrite(pin_CS, HIGH);
    digitalWrite(pin_START, LOW); // LOW = STOP
    digitalWrite(pin_PWDN, HIGH);
    digitalWrite(pin_RST, HIGH);

    delay(1000);
    Serial.println("\n--- CERELOG START ---");

    // 3. WiFi Setup
    if (WIFI_SSID == "") {
        Serial.println("Type SSID:");
        while (!Serial.available()) delay(10);
        WIFI_SSID = Serial.readStringUntil('\n'); WIFI_SSID.trim();
        Serial.println("Type PASSWORD:");
        while (!Serial.available()) delay(10);
        WIFI_PASS = Serial.readStringUntil('\n'); WIFI_PASS.trim();
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // CRITICAL: Turns off power save to reduce latency
    WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.print("\nIP: "); Serial.println(WiFi.localIP());

    server.begin(); // Open TCP Port
    udp.begin(UDP_PORT); // Open UDP Port

    // 4. SPI Setup
    vspi = new SPIClass(VSPI);
    vspi->begin(pin_SCK, pin_MISO, pin_MOSI, pin_CS);
    vspi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    
    // 5. ADS1299 Soft Reset
    delay(100);
    digitalWrite(pin_RST, LOW); delay(100);
    digitalWrite(pin_RST, HIGH); delay(500);
    
    // Stop Data Read Mode immediately
    digitalWrite(pin_CS, LOW); delayMicroseconds(2);
    vspi->transfer(0x11); // SDATAC command
    delayMicroseconds(2); digitalWrite(pin_CS, HIGH);

    attachInterrupt(digitalPinToInterrupt(pin_DRDY), onDRDY, FALLING);
    Serial.println("Ready. Waiting for TCP Connection...");
}

void loop() {
    // ============================================================
    // PART 1: ZOMBIE KILLER (TCP Connection Handling)
    // ============================================================
    
    // Check if a NEW client is knocking, even if we think we have one.
    WiFiClient newClient = server.available();
    
    if (newClient) {
        Serial.println(">>> New Client Request!");
        
        // If we already have a client, it's likely a Zombie or an old session.
        if (client && client.connected()) {
            Serial.println(">>> Disconnecting Old Client...");
            client.stop();
        }
        
        // Accept the new one
        client = newClient;
        client.setNoDelay(true); // Faster transmission
        Serial.println(">>> Connected!");
        
        // Reset State
        streaming_allowed = true;
        _millis_reference = millis();
        
        // Start Hardware
        digitalWrite(pin_START, HIGH);
        delay(1);
        digitalWrite(pin_CS, LOW); 
        vspi->transfer(0x10); // RDATAC (Start Continuous Read)
        digitalWrite(pin_CS, HIGH);
        
        digitalWrite(pin_LED, HIGH);
    }

    // ============================================================
    // PART 2: UDP Discovery (Always Active)
    // ============================================================
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char buf[64];
        int len = udp.read(buf, 63);
        buf[len] = 0;
        if (strstr(buf, "CERELOG_FIND_ME")) {
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.print("CERELOG_HERE");
            udp.endPacket();
            // Serial.println("UDP Ping Replied"); 
        }
    }

    // ============================================================
    // PART 3: Data Streaming
    // ============================================================
    if (client && client.connected()) {
        if (data_ready) {
            data_ready = false;
            
            byte raw[27]; // 3 status + 24 data
            digitalWrite(pin_CS, LOW);
            for(int i=0; i<27; i++) raw[i] = vspi->transfer(0x00);
            digitalWrite(pin_CS, HIGH);

            // Simple Packet Protocol
            uint8_t packet[37];
            uint32_t t = millis() - _millis_reference;
            
            packet[0] = 0xAB; packet[1] = 0xCD;     // Header
            packet[2] = 31;                         // Payload Len (4 time + 27 data)
            packet[3] = (t >> 24) & 0xFF;           // Time
            packet[4] = (t >> 16) & 0xFF;
            packet[5] = (t >> 8) & 0xFF;
            packet[6] = t & 0xFF;
            
            memcpy(&packet[7], raw, 27);            // Data
            
            uint8_t sum = 0;                        // Checksum
            for(int i=2; i<34; i++) sum += packet[i];
            packet[34] = sum;
            
            packet[35] = 0xDC; packet[36] = 0xBA;   // Footer

            client.write(packet, 37);
        }
    } else {
        // If client dropped, STOP HARDWARE immediately
        if (streaming_allowed) {
            Serial.println("Client Lost. Stopping Hardware.");
            streaming_allowed = false;
            digitalWrite(pin_START, LOW);
            digitalWrite(pin_LED, LOW);
            if (client) client.stop();
        }
    }
}