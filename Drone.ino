// Motor 1 and Motor 2 Pin Definitions
#define MOTOR1_IN1 18 // GPIO3
#define MOTOR1_IN2 19 // GPIO4
void setup() {
  Serial.begin(115200);
  // Set motor pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
}
void loop() {
  // Motor 1: Forward (speed control with PWM)
  analogWrite(MOTOR1_IN2, motorSpeed); // Speed: between 0-255
  Serial.println(analogRead(4));
}