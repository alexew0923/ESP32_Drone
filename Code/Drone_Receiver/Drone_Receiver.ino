#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define M1_PIN 8
#define M2_PIN 7
#define M3_PIN 1
#define M4_PIN 4

int M1_strength;
int M2_strength;
int M3_strength;
int M4_strength;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float roll;
  float pitch;
  float yaw;
  int thrust;
} struct_message;

// Create a struct_message called droneData
struct_message droneData;

Adafruit_MPU6050 mpu;

float accX, accY, accZ;
float velX, velY, velZ;
float accAngX, accAngY;
float velAngX, velAngY;
float pitchAngle, rollAngle;
uint32_t LoopTimer;

float PRateRoll = 5; float PRatePitch = PRateRoll; float PRateYaw=2;
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

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&droneData, incomingData, sizeof(droneData));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  Wire.begin(5, 43);
  mpu.begin(0x68);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  if (Serial.available() > 0) {
    PRateRoll = Serial.read();
    PRatePitch = Serial.read();
    Serial.println(PRateRoll);
  }
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

  M1_strength = droneData.thrust - InputRoll - InputPitch; //- InputYaw
  M2_strength = droneData.thrust - InputRoll + InputPitch; //+ InputYaw
  M3_strength = droneData.thrust + InputRoll + InputPitch; //- InputYaw
  M4_strength = droneData.thrust + InputRoll - InputPitch; //+ InputYaw
  
  M1_strength = constrain(M1_strength, 0, 250);
  M2_strength = constrain(M2_strength, 0, 250);
  M3_strength = constrain(M3_strength, 0, 250);
  M4_strength = constrain(M4_strength, 0, 250);

  if (droneData.thrust < 50) {
    M1_strength = 0;
    M2_strength = 0;
    M3_strength = 0;
    M4_strength = 0;
    reset_pid();
  }

  /*analogWrite(M1_PIN, M1_strength);
  analogWrite(M2_PIN, M2_strength);
  analogWrite(M3_PIN, M3_strength);
  analogWrite(M4_PIN, M4_strength);*/

  printValues();

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
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

  /*Serial.print("Roll:");
  Serial.print(InputRoll);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(InputPitch);
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.println(InputYaw);*/

  /*Serial.print("Roll:");
  Serial.print(droneData.roll);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(droneData.pitch);
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.print(droneData.yaw);
  Serial.print(",");
  Serial.print("Thrust:");
  Serial.println(droneData.thrust);*/

  Serial.print("M1:");
  Serial.print(M1_strength);
  Serial.print(",");
  Serial.print("M2:");
  Serial.print(M2_strength);
  Serial.print(",");
  Serial.print("M3:");
  Serial.print(M3_strength);
  Serial.print(",");
  Serial.print("M4:");
  Serial.println(M4_strength);
}
