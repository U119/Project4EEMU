#include <driver/ledc.h> // Include the LEDC library
#include <Wire.h>

int RPWM_Output = 22;
int LPWM_Output = 23;
int motorSpeed = 64; // Set motor speed (assuming 64 is the desired RPM)

int proxPin = 34;
float t_start = 0;
float t_end = 0;
bool timing = false;
bool lastState = false;
int name = 1;
bool currentState;
//set paramrter
float velocity;
int i = 0;
float time_ori = 0;
float G = 2.44;
float Gm = 0.61;
float R = 0.3;
float p = 0.17;
float I = 12.9;
float F, Ph, Pm, rpm;
float Torque = 253.098;

const TickType_t xDelay2000ms = pdMS_TO_TICKS(2000);
TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;
TaskHandle_t Task3 = NULL;
int passValue = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);
  xTaskCreatePinnedToCore(Proxit_Task,"Task1",1000,NULL,1,&Task1,0);
  xTaskCreatePinnedToCore(Velocity_Task,"Task1",1000,NULL,1,&Task2,0);
  // Setup motor
  pinMode(proxPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(proxPin), proximitInterrupt, CHANGE);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  // Configure LEDC
  ledcSetup(0, 5000, 8); // LEDC channel 0, 5000 Hz frequency, 8-bit resolution
  ledcAttachPin(RPWM_Output, 0); // Attach RPWM_Output to LEDC channel 0
}

void Proxit_Task(void *pvParam) {
  while(1) {
    currentState = digitalRead(proxPin);
  }
}

void Velocity_Task(void *pvParam) {
  velocity = 0.05/time_ori;
  F = (velocity * I * G) / (time_ori * p * R);
  Ph = F * velocity;
  Pm = 0.5 * Ph;
  rpm = (Pm * 9.55) / Torque;
}

void loop() {

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
    Serial.print(time_ori); // ได้เวลา และระยะห่าง 0.05 m
    Serial.println(" seconds");
    Serial.println("-----------------------------------------------");
    timing = false;
    if ( time_ori > 5) {
      // Move forward for 5 seconds
      ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
      digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
      while (i < 60000) { // Continue loop until i reaches 50
        int buttonState = digitalRead(BUTTON_PIN);
        if (buttonState == 0) {
          break;
        }
        i++; // Increment i by 1 in each iteration
        delay(1);
      }  // 5 seconds delay
      // Stop the motor
      ledcWrite(0, 0); // Set PWM to 0 for stopping
      delay(1000); // Ensure the motor stops completely before next movement
    }
  }
  lastState = currentState;
  delay(10);
}
