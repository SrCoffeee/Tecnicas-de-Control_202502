/* spin_down_id.ino — Firma: Nadia */
#include <Wire.h>
#define MPU6050_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_GYRO_ZOUT_H 0x47
const float GYRO_SENS=131.0f, DEG2RAD=3.14159265f/180.0f;
const uint8_t ENA=9, IN1=7, IN2=8;
float TS=0.01f; float Tseg=6.0f;
String ibuf;
void mpuInit(){ Wire.begin(); Wire.beginTransmission(MPU6050_ADDR); Wire.write(MPU_PWR_MGMT_1); Wire.write(0); Wire.endTransmission(); delay(50); }
float gyroZ(){ Wire.beginTransmission(MPU6050_ADDR); Wire.write(MPU_GYRO_ZOUT_H); Wire.endTransmission(false); Wire.requestFrom(MPU6050_ADDR,(uint8_t)2);
  int16_t gz=((int16_t)Wire.read()<<8)|Wire.read(); return ((float)gz/GYRO_SENS)*DEG2RAD;}
void setup(){ Serial.begin(115200); pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); analogWrite(ENA,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); mpuInit(); Serial.println("READY"); }
void loop(){
  while(Serial.available()){ char c=Serial.read(); if(c=='\n'||c=='\r'){ if(ibuf=="RUN"){ runTests(); } ibuf=""; } else ibuf+=c; }
}
void runTests(){
  // Pide al usuario "impulsar" el brazo a mano a una velocidad moderada
  Serial.println("COAST"); delay(500);
  unsigned long t0=millis(); while( (millis()-t0) < (unsigned long)(Tseg*1000) ){
    float t=0.001f*(millis()-t0); float w=gyroZ(); Serial.print(t,3); Serial.print(','); Serial.println(w,5); delay((int)(TS*1000));
  }
  Serial.println("BRAKE"); // short
  analogWrite(ENA,255); digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH); delay(10);
  t0=millis(); while( (millis()-t0) < (unsigned long)(Tseg*1000) ){
    float t=0.001f*(millis()-t0); float w=gyroZ(); Serial.print(t,3); Serial.print(','); Serial.println(w,5); delay((int)(TS*1000));
  }
  analogWrite(ENA,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  Serial.println("DONE");
}
