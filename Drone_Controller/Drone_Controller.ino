#include <esp_now.h>
#include <WiFi.h>

#define X1_PIN 32 // ESP32 ADC1 Pins 32~39
#define Y1_PIN 33
#define X2_PIN 34
#define Y2_PIN 35

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xf0,0x24,0xf9,0x7a,0xe5,0x7c};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int X1;
  int Y1;
  int X2;
  int Y2;
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
  myData.X1 = analogRead(X1_PIN);
  myData.Y1 = analogRead(Y1_PIN);
  myData.X2 = analogRead(X2_PIN);
  myData.Y2 = analogRead(Y2_PIN);
  
  Serial.print("x1 = ");
  Serial.print(valueX1);
  Serial.print(", y1 = ");
  Serial.println(valueY1);

  Serial.print("x2 = ");
  Serial.print(valueX2);
  Serial.print(", y2 = ");
  Serial.println(valueY2);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}