#include <esp_now.h>
#include <WiFi.h>

#define M1_PIN 1
#define M2_PIN 4
#define M3_PIN 7
#define M4_PIN 8

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

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&droneData, incomingData, sizeof(droneData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Roll: ");
  Serial.println(droneData.roll);
  Serial.print("Pitch: ");
  Serial.println(droneData.pitch);
  Serial.print("Yaw: ");
  Serial.println(droneData.yaw);
  Serial.print("Thrust: ");
  Serial.println(droneData.thrust);
  Serial.println();

  int M1_strength = droneData.thrust; //+ droneData.thrust*-1*droneData.yaw + droneData.thrust*-1*droneData.roll + droneData.thrust*-1*droneData.pitch;
  int M2_strength = droneData.thrust; //+ droneData.thrust*droneData.yaw + droneData.thrust*droneData.roll + droneData.thrust*-1*droneData.pitch;
  int M3_strength = droneData.thrust; //+ droneData.thrust*droneData.yaw + droneData.thrust*-1*droneData.roll + droneData.thrust*droneData.pitch;
  int M4_strength = droneData.thrust; //+ droneData.thrust*-1*droneData.yaw + droneData.thrust*droneData.roll + droneData.thrust*droneData.pitch;
  M1_strength = round(M1_strength);
  M1_strength = constrain(M1_strength, 0, 255);
  M2_strength = round(M2_strength);
  M2_strength = constrain(M2_strength, 0, 255);
  M3_strength = round(M3_strength);
  M3_strength = constrain(M3_strength, 0, 255);
  M4_strength = round(M4_strength);
  M4_strength = constrain(M4_strength, 0, 255);

  Serial.print("M1: ");
  Serial.println(M1_strength);
  Serial.print("M2: ");
  Serial.println(M2_strength);
  Serial.print("M3: ");
  Serial.println(M3_strength);
  Serial.print("M4: ");
  Serial.println(M4_strength);
  Serial.println();
  
  analogWrite(M1_PIN, M1_strength);
  analogWrite(M2_PIN, M2_strength);
  analogWrite(M3_PIN, M3_strength);
  analogWrite(M4_PIN, M4_strength);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

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

}
