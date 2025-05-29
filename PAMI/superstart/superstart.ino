#include <WiFi.h>
#include <ESP32Servo.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>

// Replace with your credentials
const char* ssid = "BeaconEdog";
const char* password = "1pass4u!";

Servo myServo0;
Servo myServo1;
Servo myServo2;

const int servo0 = 18;
const int servo1 = 19;
const int servo2 = 23;

const int ledPin = 32;

// Ultrasonic sensor HC-SR04 pins
const int trigPin = 27;
const int echoPin = 14;
const float obstacleThreshold = 10.0; // 10cm threshold

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC

WebServer server(80);

unsigned long targetTimestamp = 0;
bool hasRun = false;

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  if (duration == 0) return -1;

  float distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}

void stopMotors() {
  myServo0.write(90);
  myServo1.write(90);
  delay(500);
}

void dance() {
  myServo2.write(0);
}

void forward(int seconds) {
  unsigned long startTime = millis();
  unsigned long duration = seconds * 1000;

  myServo0.write(180);
  myServo1.write(0);

  while (millis() - startTime < duration) {
    float distance = getDistance();
    Serial.println("Distance: " + String(distance) + " cm");

    if (distance > 0 && distance <= obstacleThreshold) {
      Serial.println("Obstacle detected! Stopping motors.");
      stopMotors();
      return;
    }

    delay(100); // Check distance every 100ms
  }

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

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo0.setPeriodHertz(50);
  myServo1.setPeriodHertz(50);
  myServo2.setPeriodHertz(50);

  myServo0.attach(servo0, 500, 2500);
  myServo1.attach(servo1, 500, 2500);
  myServo2.attach(servo2, 500, 2500);

  myServo0.write(90);
  myServo1.write(90);
  myServo2.write(90);

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
      targetTimestamp = strtoull(body.c_str(), nullptr, 10);
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
      Serial.println("==> Time reached! Moving forward");

      forward(5);
      dance();

      hasRun = true;
    }
  }

  delay(1000); // Avoid spamming Serial Monitor
}
