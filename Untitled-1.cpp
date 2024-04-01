#include <driver/ledc.h> // Include the LEDC library

int RPWM_Output = 22;
int LPWM_Output = 23;
int motorSpeed = 64; // Set motor speed (assuming 64 is the desired RPM)
//New
int i = 0;
#define BUTTON_PIN 33

void setup() {
  Serial.begin(115200);
  // Setup motor
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  ledcSetup(0, 5000, 8); // LEDC channel 0, 5000 Hz frequency, 8-bit resolution
  ledcAttachPin(RPWM_Output, 0); // Attach RPWM_Output to LEDC channel 0
  //New
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {

  // Move forward for 5 seconds
  ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
  digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
  while (i < 5000) { // Continue loop until i reaches 50
      int buttonState = digitalRead(BUTTON_PIN);
      Serial.println(buttonState);
      if(buttonState == 0) {
        //Serial.println("stop");
        break; // Exit the loop
      }
      i++; // Increment i by 1 in each iteration
      delay(1);
  }  // 5 seconds delay
  // Stop the motor
  ledcWrite(0, 0); // Set PWM to 0 for stopping
  delay(10000); // Ensure the motor stops completely before next movement
}
