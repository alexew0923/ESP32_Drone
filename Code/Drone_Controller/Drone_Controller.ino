#include <esp_now.h>
#include <WiFi.h>

#define roll_pin 4 // ESP32 ADC1 Pins 32~39
#define pitch_pin 3
#define yaw_pin 2
#define thrust_pin 1

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x10, 0xb4, 0x1d, 0xe8, 0x1a, 0x34};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float roll;
  float pitch;
  float yaw;
  int thrust;
} struct_message;

// Create a struct_message called droneData
struct_message droneData;

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

float joystickValue(int analogValue) {
  float joystick;
  if (analogValue > 1854) {
    joystick = map (analogValue, 1855, 4095, 0, 50);
  } else {
    joystick = map (analogValue, 0, 1854, -50, 0);
  }
  return joystick;
}

void loop() {
  // Set values to send
  droneData.roll = joystickValue(analogRead(roll_pin))/100;
  droneData.pitch = joystickValue(analogRead(pitch_pin))/100;
  droneData.yaw = joystickValue(analogRead(yaw_pin))/100;
  droneData.thrust = map(analogRead(thrust_pin), 0, 4095, 0, 255);

  Serial.print("Roll = ");
  Serial.print(droneData.roll);
  Serial.print(", Pitch = ");
  Serial.println(droneData.pitch);
  Serial.print("Yaw = ");
  Serial.print(droneData.yaw);
  Serial.print(", Thrust = ");
  Serial.println(droneData.thrust);
  /*Serial.print("Raw Roll = ");
  Serial.print(analogRead(roll_pin));
  Serial.print(", Raw Pitch = ");
  Serial.println(analogRead(pitch_pin));
  Serial.print("Raw Yaw = ");
  Serial.print(analogRead(yaw_pin));
  Serial.print(", Raw Thrust = ");
  Serial.println(analogRead(thrust_pin));*/
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &droneData, sizeof(droneData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
}
