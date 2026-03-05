#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float accX, accY, accZ;
float velX, velY, velZ;
float accAngX, accAngY;
float velAngX, velAngY;
float pitchAngle, rollAngle;
uint32_t LoopTimer;

float PRateRoll = 20; float PRatePitch = PRateRoll; float PRateYaw=2;
float IRateRoll = 0; float IRatePitch = IRateRoll; float IRateYaw=12;
float DRateRoll = 0; float DRatePitch = DRateRoll; float DRateYaw=0;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

void reset_pid(void) {
  PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
}

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  Wire.begin(5, 43); //Change to corresponding SDA/SCL pins
  mpu.begin(0x68);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  getValues();

  accAngY = -atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));
  accAngX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2)));
  velAngY = pitchAngle + velY * 0.004;
  velAngX = rollAngle + velX * 0.004;

  pitchAngle = (0.98 * velAngY) + (0.02 * accAngY);
  rollAngle = (0.98 * velAngX) + (0.02 * accAngX);

  ErrorRatePitch = 0 - pitchAngle;
  ErrorRateRoll = 0 - rollAngle;
  ErrorRateYaw = DesiredRateYaw - velZ;
  
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0]; 
    PrevErrorRatePitch = PIDReturn[1]; 
    PrevItermRatePitch = PIDReturn[2];
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1]; 
    PrevItermRateRoll = PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0]; 
    PrevErrorRateYaw = PIDReturn[1]; 
    PrevItermRateYaw = PIDReturn[2];

  printValues();

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004/2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm <-400) Iterm = -400;
  float Dterm = D * (Error - PrevError)/0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput <-400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void getValues() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x - 0.17;
  accY = a.acceleration.y;
  accZ = a.acceleration.z - 0.44;
  velX = g.gyro.x;
  velY = g.gyro.y - 0.085;
  velZ = g.gyro.z;
}

void printValues() {
  /*Serial.print("AccX:"); // m/s^2
  Serial.print(accX);
  Serial.print(",");
  Serial.print("AccY:");
  Serial.print(accY);
  Serial.print(",");
  Serial.print("AccZ:");
  Serial.print(accZ);
  Serial.println(",");*/

  /*Serial.print("RotX:"); // rad/s
  Serial.print(velX);
  Serial.print(",");
  Serial.print("RotY:");
  Serial.print(velY);
  Serial.print(",");
  Serial.print("RotZ:");
  Serial.print(velZ);
  Serial.println(",");*/

  /*Serial.print("AccAngX:");
  Serial.print(accAngX);
  Serial.print(",");
  Serial.print("AccAngY:");
  Serial.print(accAngY);
  Serial.print(",");
  Serial.print("VelAngX:");
  Serial.print(velAngX);
  Serial.print(",");
  Serial.print("VelAngY:");
  Serial.print(velAngY);
  Serial.println(",");
  Serial.print("PitchAng:");
  Serial.print(pitchAngle);
  Serial.print(",");
  Serial.print("RollAng:");
  Serial.println(rollAngle);
  Serial.print(",");*/

  Serial.print("Roll:");
  Serial.print(InputRoll);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(InputPitch);
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.println(InputYaw);
}
