#include "WiFi.h"
#include <HTTPClient.h>

// SSID and PASSWORD of your WiFi network
const char* ssid = "Korn";  // Your WiFi name
const char* password = "12345678"; // Your WiFi password
String URL = "http://172.20.10.14/dht11_project/test_data.php";

int proxPin = 34;
float t_start = 0;
float t_end = 0;
bool timing = false;
bool lastState = false;
int name = 1;

void setup() {
  Serial.begin(9600);
  pinMode(proxPin, INPUT);

  Serial.println();
  Serial.println("-------------");
  Serial.println("WIFI mode: STA");
  WiFi.mode(WIFI_STA);
  Serial.println("-------------");

  Serial.println();
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int connecting_process_timed_out = 20; // 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      delay(1000);
    }
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("------------");
  delay(2000);
}

void loop() {
  float time_ori = 0;
  bool currentState = digitalRead(proxPin);
  if (!timing && lastState == LOW && currentState == HIGH) {
    t_start = millis() / 1000.0;
    timing = true;
    Serial.println("Start timing");
  }

  if (timing && lastState == HIGH && currentState == LOW) {
    t_end = millis() / 1000.0;
  }

  if (timing && lastState == HIGH && currentState == LOW) {
    time_ori = t_end - t_start;
    Serial.print("Elapsed time: ");
    Serial.print(time_ori);
    Serial.println(" seconds");
    Serial.println("-----------------------------------------------");
    timing = false;
  }

  lastState = currentState;
  if (WiFi.status() == WL_CONNECTED) {
    String postData = "Name=" + String(name) + "&Time=" + String(time_ori);
    HTTPClient http;
    http.begin(URL);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpCode = http.POST(postData);
    String payload = http.getString();
    //Serial.print("HTTP Status Code: ");
    //Serial.println(httpCode);
    //Serial.print("payload: ");
    //Serial.println(payload);
  }
  delay(10);
}
