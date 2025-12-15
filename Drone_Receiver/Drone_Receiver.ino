#include <esp_now.h>
#include <WiFi.h>

#define M1_PIN 19
#define M2_PIN 21
#define M3_PIN 22
#define M4_PIN 23

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int M1;
  int M2;
  int M3;
  int M4;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("M1: ");
  Serial.println(myData.M1);
  Serial.print("M2: ");
  Serial.println(myData.M2);
  Serial.print("M3: ");
  Serial.println(myData.M3);
  Serial.print("M4: ");
  Serial.println(myData.M4);
  Serial.println();

  analogWrite(M1_PIN, myData.M4);
  analogWrite(M2_PIN, myData.M4);
  analogWrite(M3_PIN, myData.M4);
  analogWrite(M4_PIN, myData.M4);
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
