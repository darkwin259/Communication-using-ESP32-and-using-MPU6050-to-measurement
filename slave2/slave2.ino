
#include "I2Cdev.h"
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include<stdio.h>
#include<stdlib.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <LiquidCrystal_I2C.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define RX_PIN 16
#define TX_PIN 17
char truyendi[200];
MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27,16,2); 


#define OUTPUT_READABLE_YAWPITCHROLL




#define INTERRUPT_PIN 2 
#define LED_PIN 13
int a=0;
typedef struct struct_message {
    int a;
    int b;
    int c;
    int d;

} struct_message;
struct_message myData;
 HardwareSerial MySerial(1);
 bool dataReceived = false;
int yarnew;
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  
  if (len == sizeof(myData)){
  memcpy(&myData, incomingData, sizeof(myData)); // chú ý đảm bảo là cùng kích thước
  dataReceived = true; 
  Serial.println(myData.b);
 }
}

// MPU control/status vars
bool dmpReady = false; 
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  
  MySerial.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN); 
  WiFi.mode(WIFI_STA);

 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

 
  Serial.begin(115200);
  while (!Serial); 
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

//   // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  
  devStatus = mpu.dmpInitialize();

 
mpu.setXGyroOffset(201);
  mpu.setYGyroOffset(-47);
  mpu.setZGyroOffset(-41);
  mpu.setXAccelOffset(2429);
  mpu.setYAccelOffset(-1906);
  mpu.setZAccelOffset(1585);
 
  if (devStatus == 0) {
   
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
   
    mpu.PrintActiveOffsets();
   
    mpu.setDMPEnabled(true);

    
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    
    dmpReady = true;

    
    packetSize = mpu.dmpGetFIFOPacketSize();
 
}
   lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();
 

esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}


void loop() {
 
  if (!dmpReady) return;
 
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {


#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   
   yarnew= round (ypr[0] * 180 / M_PI);
   
   if (dataReceived) {
        
        sprintf(truyendi, "%d,%d,%d,%d,%d,d",myData.a,myData.b,myData.c,myData.d,yarnew);
        // sprintf(truyendi,"%d,d",yarnew);
        MySerial.write(truyendi);
         Serial.println(truyendi);
          
        if(yarnew<2 && yarnew > -2)
          a=0;
        else
          a=yarnew;
        
        lcd.setCursor(0,0);
        lcd.print("Goc hien thi la");
        lcd.setCursor(0,1);
        lcd.print(a);
        
        
        dataReceived = false; // Reset flag after processing
    }
#endif

  }
  
}

