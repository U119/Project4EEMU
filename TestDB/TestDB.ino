#include "WiFi.h"
#include <HTTPClient.h>

// SSID and PASSWORD of your WiFi network
const char* ssid = "Korn";  // Your WiFi name
const char* password = "12345678"; // Your WiFi password
String URL = "http://172.20.10.14/dht11_project/test_data.php";
float X, Y, Z = 0;

void setup() {
  Serial.begin(9600);

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
  if (WiFi.status() == WL_CONNECTED) {
    String postData = "X=" + String(X) + "&Y=" + String(Y) + "&Z=" + String(Z);
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
  delay(1000);
}
