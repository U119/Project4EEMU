#include <driver/ledc.h>

#define RPWM_Output 22
#define LPWM_Output 23
#define BUTTON_PIN 33

TaskHandle_t motorTaskHandle;
TaskHandle_t buttonTaskHandle;

void motorTask(void *pvParameters) {
  int motorSpeed = 64; // Set motor speed (assuming 64 is the desired RPM)

  ledcSetup(0, 5000, 8); // LEDC channel 0, 5000 Hz frequency, 8-bit resolution
  ledcAttachPin(RPWM_Output, 0); // Attach RPWM_Output to LEDC channel 0

  while (true) {
    // Move forward
    ledcWrite(0, motorSpeed);
    digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
    vTaskDelay(pdMS_TO_TICKS(5000)); // Run motor for 5 seconds
    // Stop the motor
    ledcWrite(0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Ensure the motor stops completely before next movement
  }
}

void buttonTask(void *pvParameters) {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  while (true) {
    // Monitor button state
    if (digitalRead(BUTTON_PIN) == LOW) {
      // Button pressed, stop the motor
      //vTaskDelete(motorTaskHandle); // Stop the motor task
      ledcWrite(0, 0); // Set PWM to 0 for stopping
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Check button state every 100 milliseconds
  }
}

void setup() {
  Serial.begin(250000);

  xTaskCreatePinnedToCore(
    motorTask,      // Function to implement the task
    "Motor Task",   // Name of the task
    10000,          // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    &motorTaskHandle, // Task handle
    0               // Core number (0 or 1) - use core 0
  );

  xTaskCreatePinnedToCore(
    buttonTask,     // Function to implement the task
    "Button Task",  // Name of the task
    10000,          // Stack size in words
    NULL,           // Task input parameter
    0,              // Priority of the task
    &buttonTaskHandle, // Task handle
    0               // Core number (0 or 1) - use core 0
  );
}

void loop() {
  // Empty, loop is not used in FreeRTOS-based applications
}
