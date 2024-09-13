#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include<stdio.h>
#include<stdlib.h>

uint8_t Slaveaddress[] = {0x94, 0xe6, 0x86, 0x3a, 0xc2, 0x34};
int stm32Pin1 =12 ; // Chân 12 của ESP32 kết nối với STM32
int stm32Pin2 = 13; // Chân 13 của ESP32 kết nối với STM32
int stm32Pin3 = 14; // Chân 14 của ESP32 kết nối với STM32
int stm32Pin4 = 27; // Chân 27 của ESP32 kết nối với STM32
HardwareSerial MySerial(1);
typedef struct struct_message {
  int a;
  int b;
  int c;
  int d;
} struct_message;
struct_message myData;

esp_now_peer_info_t slaveaddress;

void setup() {
  // Thiết lập các chân làm ngõ vào với điện trở kéo lên (pull-up)
  pinMode(stm32Pin1, INPUT);
  pinMode(stm32Pin2, INPUT_PULLUP);
  pinMode(stm32Pin3, INPUT_PULLUP);
  pinMode(stm32Pin4, INPUT_PULLUP);

  // Khởi động Serial để theo dõi mức logic
  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, 16, 17); 
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(slaveaddress.peer_addr, Slaveaddress, 6);
slaveaddress.channel = 0;  
slaveaddress.encrypt = false;
esp_now_add_peer(&slaveaddress); 
}

void loop() {
  // Đọc mức logic của các chân
  myData.a = digitalRead(stm32Pin1);
  myData.b= digitalRead(stm32Pin2);
  myData.c = digitalRead(stm32Pin3);
  myData.d = digitalRead(stm32Pin4);
  Serial.println(myData.c);
  delay(100);
  
  esp_now_send(Slaveaddress, (uint8_t *) &myData, sizeof(myData)); 
  //hoang anh tuan
  
}
