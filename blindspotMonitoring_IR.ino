/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <String.h>
#include <SharpIR.h>
#define IRPin A0
#define model 100500

int distance_cm;
int buzzPin = 12;

SharpIR mySensor = SharpIR(IRPin, model);

int status = WL_IDLE_STATUS;

#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on
IPAddress ip(192, 168, 0, 34);

char packetBuffer[256]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);//displaying wifi status
  pinMode(7, OUTPUT); //displaying sensor status
  pinMode(10, OUTPUT); //Displaying less than distance
  Wire.begin();
    
  unsigned status;
  
  digitalWrite(7, HIGH);

  /*WIFI STUFF*/
  //WiFi.config(ip);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    // WiFi.disconnect();
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();
  digitalWrite(13, HIGH);
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  int udpStat = Udp.begin(localPort);
  if(udpStat == 1) {
    Serial.println("Success starting UDP server");
  } else {
    Serial.println("Error! ");
    Serial.println(udpStat);
  }
}

void loop() {

  char buffer[1024];
  //getDistance() returns the distance reading in cm
  distance_cm = mySensor.distance();
  
  //printWifiStatus();
  //Serial.println(distance_cm);
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);
  
  // send a reply, to the IP address and port that sent us the packet we received
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  buffer[0] = 'F';
  buffer[1] = ',';
  itoa(distance_cm, &buffer[2], 10);
  Serial.println(buffer);
  
  Udp.write(buffer);
  Udp.endPacket();
  }

  if(distance_cm < 0.5) {
      digitalWrite(10, HIGH);
  } else {
      digitalWrite(10, LOW);
  }
  
  delay(500);
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
