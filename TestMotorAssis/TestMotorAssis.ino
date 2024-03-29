#include <driver/ledc.h> // Include the LEDC library

int RPWM_Output = 22;
int LPWM_Output = 23;
int motorSpeed = 64; // Set motor speed (assuming 64 is the desired RPM)

void setup() {
  Serial.begin(115200);
  pinMode(proxPin, INPUT);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  // Configure LEDC
  ledcSetup(0, 5000, 8); // LEDC channel 0, 5000 Hz frequency, 8-bit resolution
  ledcAttachPin(RPWM_Output, 0); // Attach RPWM_Output to LEDC channel 0
}

void loop() {
  // Move forward for 10 seconds
  ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
  digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
  delay(10000);  // 10 seconds delay

  // Stop the motor
  ledcWrite(0, 0); // Set PWM to 0 for stopping
  delay(1000); // Ensure the motor stops completely before next movement
}
