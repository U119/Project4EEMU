#include "WiFi.h"
#include <HTTPClient.h>
#include <driver/ledc.h> // Include the LEDC library

// SSID and PASSWORD of your WiFi network
const char* ssid = "Korn";  // Your WiFi name
const char* password = "12345678"; // Your WiFi password
String URL = "http://172.20.10.14/dht11_project/test_data.php";

int proxPin = 34;
float t_start = 0;
float t_end = 0;
bool timing = false;
bool lastState = false;
int name = 3;

int RPWM_Output = 22;
int LPWM_Output = 23;
int motorSpeed = 128; // Set motor speed (assuming 64 is the desired RPM)
int i = 0;

TaskHandle_t Task1 = NULL;    
TaskHandle_t Task2 = NULL;
TaskHandle_t Task3 = NULL;
TaskHandle_t Task4 = NULL;
TaskHandle_t Task5 = NULL;
int passValue = 0;

void setup() {
  Serial.begin(9600);
  pinMode(proxPin, INPUT);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  // Configure LEDC
  ledcSetup(0, 5000, 8); // LEDC channel 0, 5000 Hz frequency, 8-bit resolution
  ledcAttachPin(RPWM_Output, 0); // Attach RPWM_Output to LEDC channel 0
  xTaskCreatePinnedToCore(Motor_Task,"Task1",1000,NULL,2,&Task1,0);

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

void Motor_Task(void *pvParam) {
  while(1){
    while (i > motorSpeed) {
      i++;
      ledcWrite(0, i);  // Set motor speed using LEDC
      digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
      vTaskDelay(pdMS_TO_TICKS(40));
    }
    vTaskDelay(pdMS_TO_TICKS(55000));
    ledcWrite(0, 0);
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
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
