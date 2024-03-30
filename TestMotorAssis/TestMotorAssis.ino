#include <driver/ledc.h> // Include the LEDC library

int RPWM_Output = 22;
int LPWM_Output = 23;
int motorSpeed = 64; // Set motor speed (assuming 64 is the desired RPM)

int proxPin = 34;
float t_start = 0;
float t_end = 0;
bool timing = false;
bool lastState = false;
int name = 1;

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

void gyro_signals(void) {
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
}

void setup() {
  Serial.begin(9600);
  // Setup motor
  pinMode(proxPin, INPUT);
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
}

void loop() {
  float time_ori = 0;
  bool currentState = digitalRead(proxPin);
  gyro_signals();
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
    if ( time_ori > 5 && AnglePitch > -10 ) {
      // Move forward for 5 seconds
      ledcWrite(0, motorSpeed);  // Set motor speed using LEDC
      digitalWrite(LPWM_Output, LOW); // Assuming LOW is forward direction
      delay(5000);  // 5 seconds delay

      // Stop the motor
      ledcWrite(0, 0); // Set PWM to 0 for stopping
      delay(1000); // Ensure the motor stops completely before next movement
    }
  }
  lastState = currentState;
  delay(10);
}
