#include <driver/ledc.h> // Include the LEDC library
#include <Wire.h>
//#include "WiFi.h"
//#include <HTTPClient.h>

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

void Angle_Task(void *pvParam) {
  while(1){
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); 
    Wire.write(0x8);
    Wire.endTransmission();                                                   
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();
    RateRoll=(float)GyroX/65.5;
    RatePitch=(float)GyroY/65.5;
    RateYaw=(float)GyroZ/65.5;
    AccX=(float)AccXLSB/4096 - 0.03;
    AccY=(float)AccYLSB/4096;
    AccZ=(float)AccZLSB/4096 - 0.17;
    AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
    AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
    //Serial.println(AngleRoll); //หน้าหลัง
    //Serial.println(AnglePitch); //ซ้ายขวา
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void IRAM_ATTR Proxit_Task() {
  currentState = true;
}

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
      Serial.print("Elapsed time: ");
      Serial.print(time_ori); // ได้เวลา และระยะห่าง 0.05 m
      //Serial.println(" seconds");
      //Serial.println("-----------------------------------------------");
      timing = false;
      if ( time_ori > 5) {
        // Move forward for 5 seconds
        motorSpeed = 64;

        ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
        digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
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
        ledcWrite(0, 0); // Set PWM to 0 for stopping
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
  xTaskCreatePinnedToCore(Switch_Task,"Task1",1000,NULL,3,&Task1,0);
  xTaskCreatePinnedToCore(Velocity_Task,"Task2",1000,NULL,2,&Task2,0);
  xTaskCreatePinnedToCore(Angle_Task,"Task3",2000,NULL,1,&Task3,0);
  xTaskCreatePinnedToCore(Start_Task,"Task4",2000,NULL,1,&Task4,0);
  xTaskCreatePinnedToCore(Ramp_Task,"Task5",2000,NULL,1,&Task5,0);
  xTaskCreatePinnedToCore(inputMag_Task,"Task6",2000,NULL,2,&Task6,0);
  attachInterrupt(digitalPinToInterrupt(proxPin), &Proxit_Task, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), &Switch_Task, CHANGE);
  // Setup motor
  pinMode(proxPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(proxPin), proximitInterrupt, CHANGE);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

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

void Ramp_Task(void *pvParam) {
  while (1) {
    if (AngleRoll > 8 && velocity != 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (AngleRoll > 8 && velocity != 0) {
        //ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
        //digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
        while (i < 60000) { // Continue loop until i reaches 50
          int buttonState = digitalRead(BUTTON_PIN);
          if(buttonState == 1) {
            break; // Exit the loop
          }
          i++; // Increment i by 1 in each iteration
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        ledcWrite(0, 0);
      }
    }
  }
}

void inputMag_Task(void *pvParam) {
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (currentState) {
      //Serial.println(currentState);
      currentState = false;
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      //Serial.println(currentState);
      currentState = false;
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Acc_Task(void *pvParam) {
  while(1) {
    CurrentAccX = AccX;
    if(CurrentAccX - LastAccX > 1.5*LastAccX) {
      //ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
      //digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
    }
  }
}

void loop() {
  //
}