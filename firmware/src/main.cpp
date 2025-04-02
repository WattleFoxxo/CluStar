#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

#define SERVER_PORT 3969

#define LED_PIN 14 // D5

WiFiUDP udp;
char packetBuffer[1024];

void setup() {
    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WIFI...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WIFI connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    udp.begin(SERVER_PORT);
    Serial.print("UDP Server running on port ");
    Serial.println(SERVER_PORT);

    pinMode(LED_PIN, OUTPUT);

    digitalWrite(LED_PIN, HIGH);
}

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, sizeof(packetBuffer));
        if (len > 0) {
            packetBuffer[len] = '\0';
            
            uint8_t command = packetBuffer[0];
            uint8_t arg = packetBuffer[1];

            switch (command) {
                case 0x00:
                    digitalWrite(LED_PIN, arg);
                    break;
                default:
                    Serial.println("Unknown command :(");
                    break;
            }
        }
    }
}
