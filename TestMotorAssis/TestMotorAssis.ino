#include <driver/ledc.h> // Include the LEDC library
#include <Wire.h>
#include <Arduino.h>
//#include "WiFi.h"
//#include <HTTPClient.h>

const unsigned long TIMER_INTERVAL = 500; // 0.5 second

// Timer object
hw_timer_t *timer = NULL;

// Function prototypes
void IRAM_ATTR onTimer();
void setupProximityDetection();
void proximityDetected();

int RPWM_Output = 22;
int LPWM_Output = 23;
int motorSpeed ; // Set motor speed (assuming 64 is the desired RPM)

// ตัวแปรสำหรับคำนวณหาความเร็ว
float velocity;
int i = 0;
float time_ori;
float G = 2.44;
float Gm = 0.61;
float R = 0.3;
float p = 0.17;
float I = 12.9;
float F, Ph, Pm, rpm;
float Torque = 253.098;
float Vm;

int proxPin = 34;
float t_start = 0;
float t_end = 0;
bool timing = false;
bool lastState = false;
int name = 1;
bool currentState;

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
float LastAccX = 0, CurrentAccX;

#define BUTTON_PIN 33

const TickType_t xDelay2000ms = pdMS_TO_TICKS(2000);
TaskHandle_t Task1 = NULL;    
TaskHandle_t Task2 = NULL;
TaskHandle_t Task3 = NULL;
TaskHandle_t Task4 = NULL;
TaskHandle_t Task5 = NULL;
TaskHandle_t Task6 = NULL;
TaskHandle_t Task7 = NULL;    
TaskHandle_t Task8 = NULL;
TaskHandle_t Task9 = NULL;
TaskHandle_t Task10 = NULL;
TaskHandle_t Task11 = NULL;
TaskHandle_t Task12 = NULL;
int passValue = 0;

bool SwitchState;

void Switch_Task(void *pvParam) {
  while(1) {
    /*
    1 = ยังไม่กด
    0 = กดแล้ว
    */
    SwitchState = digitalRead(BUTTON_PIN);
    //Serial.println(SwitchState);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Start_Task(void *pvParam) {
  while (1) {
    if (!timing && lastState == LOW && currentState == HIGH) {
      t_start = millis() / 1000.0;
      timing = true;
      //Serial.println("Start timing");
    }

    if (timing && lastState == HIGH && currentState == LOW) {
      t_end = millis() / 1000.0;
    }

    if (timing && lastState == HIGH && currentState == LOW) {
      time_ori = t_end - t_start;
      //Serial.print("Elapsed time: ");
      //Serial.print(time_ori); // ได้เวลา และระยะห่าง 0.05 m
      //Serial.println(" seconds");
      //Serial.println("-----------------------------------------------");
      timing = false;
      if ( time_ori > 5) {
        // Move forward for 5 seconds
        motorSpeed = 64;

        //ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
        //digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
        vTaskDelay(pdMS_TO_TICKS(100));
        
        while (i < 30000) { // Continue loop until i reaches 50
          int buttonState = digitalRead(BUTTON_PIN);
          if(buttonState == 1) {
            break; // Exit the loop
          }
          i++; // Increment i by 1 in each iteration
          vTaskDelay(pdMS_TO_TICKS(1));
        }  // 5 seconds delay
        // Stop the motor
        //ledcWrite(0, 0); // Set PWM to 0 for stopping
        vTaskDelay(pdMS_TO_TICKS(10000)); // Ensure the motor stops completely before next movement
      }
    }
    lastState = currentState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  xTaskCreatePinnedToCore(Switch_Task,"Task1",1000,NULL,5,&Task1,0);
  xTaskCreatePinnedToCore(Velocity_Task,"Task2",1000,NULL,3,&Task2,0);
  //xTaskCreatePinnedToCore(Angle_Task,"Task3",2000,NULL,2,&Task3,0);
  xTaskCreatePinnedToCore(Start_Task,"Task4",2000,NULL,1,&Task4,0);
  //xTaskCreatePinnedToCore(Ramp_Task,"Task5",2000,NULL,0,&Task5,0);
  //xTaskCreatePinnedToCore(inputMag_Task,"Task6",2000,NULL,4,&Task6,0);
  //attachInterrupt(digitalPinToInterrupt(proxPin), &Proxit_Task, FALLING);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), &Switch_Task, CHANGE);
  // Setup motor
  //pinMode(proxPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(proxPin), proximitInterrupt, CHANGE);
  //pinMode(RPWM_Output, OUTPUT);
  //pinMode(LPWM_Output, OUTPUT);

  // Configure LEDC
  ledcSetup(0, 5000, 8); // LEDC channel 0, 5000 Hz frequency, 8-bit resolution
  ledcAttachPin(RPWM_Output, 0); // Attach RPWM_Output to LEDC channel 0
  // Setup MPU6050
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  setupProximityDetection();

  // Start the timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true); // Attach the timer interrupt
  timerAlarmWrite(timer, TIMER_INTERVAL * 1000, true); // Set the alarm interval in microseconds
  timerAlarmEnable(timer); // Enable the timer alarm
}

void Velocity_Task(void *pvParam) {
  while(1) {
    velocity = 0.05/time_ori;
    F = (velocity * I * G) / (time_ori * p * R);
    Ph = F * velocity;
    Pm = 0.5 * Ph;
    rpm = (Pm * 9.55) / Torque;
    Vm = 0.5 * velocity;
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// Timer interrupt handler
void IRAM_ATTR onTimer() {
  // Call function to check proximity
  proximityDetected();
  delay(500);
}

// Function to check proximity
void proximityDetected() {
  // Perform proximity detection here
  bool currentState = digitalRead(proxPin);
  Serial.println(currentState);
  delay(500);
}

// Function to set up proximity detection (replace with actual setup code)
void setupProximityDetection() {
  pinMode(proxPin, INPUT);
  // Set up proximity sensor pins, interrupts, etc.
  // This function should contain the necessary code to initialize proximity detection
}

void loop() {
  //
}