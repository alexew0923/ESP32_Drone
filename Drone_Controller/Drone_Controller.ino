#include <esp_now.h>
#include <WiFi.h>

#define M1_PIN 32 // ESP32 ADC1 Pins 32~39
#define M2_PIN 33
#define M3_PIN 34
#define M4_PIN 35

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xf0,0x24,0xf9,0x7a,0xe5,0x7c};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int M1;
  int M2;
  int M3;
  int M4;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  myData.M1 =  map(analogRead(M1_PIN), 0, 4095, 0, 255);
  myData.M2 =  map(analogRead(M2_PIN), 0, 4095, 255, 0);
  myData.M3 =  map(analogRead(M3_PIN), 0, 4095, 0, 255);
  myData.M4 =  map(analogRead(M4_PIN), 0, 4095, 255, 0);
  
  
  Serial.print("m1 = ");
  Serial.print(myData.M1);
  Serial.print(", m2 = ");
  Serial.println(myData.M2);

  Serial.print("m3 = ");
  Serial.print(myData.M3);
  Serial.print(", m4 = ");
  Serial.println(myData.M4);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10);
}
