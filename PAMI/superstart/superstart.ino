#include <WiFi.h>
#include <ESP32Servo.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>

// Replace with your credentials
const char* ssid = "LaMasseBox_2.4GHz_EXT";
const char* password = "mastercraft";

Servo myServo;
const int servoPin = 18; // Changed from 18 because 18 is now LED
const int ledPin = 19;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC

WebServer server(80);

unsigned long targetTimestamp = 0;
bool hasRun = false;

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // LED OFF

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2500);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: " + WiFi.localIP().toString());
  digitalWrite(ledPin, HIGH); // LED ON

  timeClient.begin();

  server.on("/timestamp", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      targetTimestamp = body.toInt();
      Serial.println("Received timestamp: " + String(targetTimestamp));
      hasRun = false;
      server.send(200, "text/plain", "Timestamp received");
    } else {
      server.send(400, "text/plain", "Missing timestamp");
    }
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  timeClient.update();

  unsigned long currentTime = timeClient.getEpochTime();

  // Debug info if waiting for trigger
  if (targetTimestamp > 0 && !hasRun) {
    long diff = (long)(targetTimestamp - currentTime);
    Serial.println("Current time: " + String(currentTime) +
                   " | Target: " + String(targetTimestamp) +
                   " | Countdown: " + String(diff) + "s");

    if (diff <= 0) {
      Serial.println("==> Time reached! Triggering servo!");

      myServo.write(0);   // Clockwise
      delay(5000);

      myServo.write(90);  // Stop
      delay(500);

      myServo.write(180); // Counterclockwise
      delay(5000);

      myServo.write(90);  // Stop
      delay(500);

      hasRun = true; // Only run once
    }
  }

  delay(1000); // Avoid spamming Serial Monitor
}
