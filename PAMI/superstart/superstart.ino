#include <WiFi.h>
#include <ESP32Servo.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>

// Replace with your credentials
const char* ssid = "LaMasseBox_2.4GHz_EXT";
const char* password = "mastercraft";

Servo myServo0;
Servo myServo1;
const int servo0 = 18;
const int servo1 = 19;
const int ledPin = 23;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC

WebServer server(80);

unsigned long targetTimestamp = 0;
bool hasRun = false;

void stopMotors() {
  myServo0.write(90);
  myServo1.write(90);
  delay(500);
}

void forward(int seconds) {
  myServo0.write(0);
  myServo1.write(180);
  delay(seconds * 1000);
  stopMotors();
}

void rotateRight(int seconds) {
  myServo0.write(0);
  myServo1.write(0);
  delay(seconds * 1000);
  stopMotors();
}

void rotateLeft(int seconds) {
  myServo0.write(180);
  myServo1.write(180);
  delay(seconds * 1000);
  stopMotors();
}

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // LED OFF

  myServo0.setPeriodHertz(50);
  myServo1.setPeriodHertz(50);
  myServo0.attach(servo0, 500, 2500);
  myServo1.attach(servo1, 500, 2500);

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

  if (targetTimestamp > 0 && !hasRun) {
    long diff = (long)(targetTimestamp - currentTime);
    Serial.println("Current time: " + String(currentTime) +
                   " | Target: " + String(targetTimestamp) +
                   " | Countdown: " + String(diff) + "s");

    if (diff <= 0) {
      Serial.println("==> Time reached! Executing movement sequence");

      forward(5);
      rotateRight(2);
      rotateLeft(2);
      forward(5);

      hasRun = true;
    }
  }

  delay(1000); // Avoid spamming Serial Monitor
}
