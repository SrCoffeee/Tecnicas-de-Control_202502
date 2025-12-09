/* =====================================================================
 *  spin_down_damping_encoder.ino — Amortiguamiento con SOLO AS5600
 *  Ensayos: COAST (rueda libre) y BRAKE (cortocircuito).
 *  Emite CSV: t,theta_deg,omega_deg_s para cada bloque.
 *  L298N: ENA=9, IN1=7, IN2=8.  AS5600 I2C: A4(SDA), A5(SCL).
 * ===================================================================== */
#include <Wire.h>
#include <math.h>

const uint8_t PIN_ENA=9, PIN_IN1=7, PIN_IN2=8;
#define AS5600_ADDR 0x36
#define REG_RAW_ANGLE_H 0x0C
#define REG_RAW_ANGLE_L 0x0D

// Muestreo/tiempos
float TS=0.01f;            // s
float Tcoast=6.0f;         // s
float Tbrake=6.0f;         // s
float spinDuty=0.60f;      // duty para acelerar antes del coast
float spinTime=0.7f;       // s

// Estimación ω con derivada sucia
float offset_rad=0, theta_prev=0, omega_est=0;
const float alpha=0.25f;         // 0.15–0.30 típico
const float omega_dead_deg_s=1.0f;

String cmd;

uint16_t as5600Raw(){
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_RAW_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR,(uint8_t)2);
  if(Wire.available()<2) return 0;
  uint8_t msb=Wire.read(), lsb=Wire.read();
  return (((uint16_t)msb<<8)|lsb) & 0x0FFF;
}
float as5600Rad(){ return (float)as5600Raw()*(2.0f*M_PI/4096.0f); }
static inline float unwrapDelta(float d){
  if(d>M_PI) d-=2.0f*M_PI; if(d<-M_PI) d+=2.0f*M_PI; return d;
}

void driveDuty(float d){
  d = fmaxf(fminf(d,1.0f),-1.0f);
  if(d>=0){ digitalWrite(PIN_IN1,HIGH); digitalWrite(PIN_IN2,LOW);  analogWrite(PIN_ENA,(int)(255*d)); }
  else    { digitalWrite(PIN_IN1,LOW);  digitalWrite(PIN_IN2,HIGH); analogWrite(PIN_ENA,(int)(-255*d)); }
}
void hbridge_freewheel(){ analogWrite(PIN_ENA,0); digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW); }
void hbridge_brake(){ analogWrite(PIN_ENA,255); digitalWrite(PIN_IN1,HIGH); digitalWrite(PIN_IN2,HIGH); }

void spin_up(){
  driveDuty(spinDuty);
  unsigned long t0=millis();
  while((millis()-t0) < (unsigned long)(1000.0f*spinTime)) delay(1);
  hbridge_freewheel();
}

void phase_capture(const char* label, float Tsec, void (*prep)()){
  prep();
  float t0=millis()*0.001f;
  theta_prev = as5600Rad()-offset_rad;
  omega_est  = 0.0f;

  Serial.println(label);
  Serial.println("t,theta_deg,omega_deg_s");

  unsigned long nextTick=millis();
  while((millis()*0.001f - t0) < Tsec){
    if(millis()<nextTick){ delay(1); continue; }
    nextTick += (unsigned long)(TS*1000.0f);

    float t = millis()*0.001f - t0;
    float th= as5600Rad()-offset_rad;
    float dth= unwrapDelta(th - theta_prev);
    float omega_raw = dth/TS;              // rad/s
    omega_est = (1.0f-alpha)*omega_est + alpha*omega_raw;
    float omega_deg_s = omega_est*180.0f/M_PI;
    if(fabs(omega_deg_s) < omega_dead_deg_s) omega_deg_s=0.0f;

    Serial.print(t,3); Serial.print(',');
    Serial.print(th*180.0f/M_PI,4); Serial.print(',');
    Serial.println(omega_deg_s,4);

    theta_prev=th;
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_ENA,OUTPUT); pinMode(PIN_IN1,OUTPUT); pinMode(PIN_IN2,OUTPUT);
  hbridge_freewheel();
  // offset rápido
  float acc=0; for(int k=0;k<50;k++){ acc+=as5600Rad(); delay(5); }
  offset_rad=acc/50.0f;
  Serial.println("READY (send RUN)");
}

void loop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){
      if(cmd=="RUN"){
        // Arranque y COAST
        spin_up();
        phase_capture("COAST", Tcoast, hbridge_freewheel);
        // BRAKE
        phase_capture("BRAKE", Tbrake, hbridge_brake);
        hbridge_freewheel();
        Serial.println("DONE");
      }
      cmd="";
    } else cmd+=c;
  }
}
