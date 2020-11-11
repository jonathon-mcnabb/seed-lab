#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Set WiFi credentials
#define WIFI_SSID "CSMwireless"
// #define WIFI_PASS "44651268"
#define UDP_PORT 4210

// UDP
WiFiUDP UDP;
char packet[255];
char reply[] = "Packet received!";

void setup() {
  // Setup serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println(WiFi.macAddress());

  // Begin WiFi
  WiFi.begin(WIFI_SSID);

  // Connecting to WiFi...
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }

  // Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Begin listening to UDP port
  UDP.begin(UDP_PORT);
  Serial.print("Listening on UDP port ");
  Serial.println(UDP_PORT);

}

void loop() {

  // If packet received...
  int packetSize = UDP.parsePacket();
  if (packetSize) {
    Serial.print("Received packet! Size: ");
    Serial.println(packetSize);
    int len = UDP.read(packet, 255);
    if (len > 0)
    {
      packet[len] = '\0';
    }
    Serial.print("Packet received: ");
    Serial.println(packet);

    // Send return packet
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(reply);
    UDP.endPacket();

  }

}
