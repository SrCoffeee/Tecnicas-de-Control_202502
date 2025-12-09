/* =====================================================================
 *  control_as5600_mpu6050_adaptive.ino
 *  Firma: Nadia
 *  Hardware: Arduino UNO + AS5600 + MPU6050 + L298N (IN1=7, IN2=8, ENA=9)
 *  Control: PID | HINF (discreto fijo) | SMC | ADAPTIVE_PID (PID + FF con RLS)
 *  Perfiles: STEP | RAMP | PULSET  (desde comandos serial CFG,...)
 *  STOP: comando serie "STOP" para abortar y apagar
 * ===================================================================== */
#include <Wire.h>
#include <math.h>

/*** Pines L298N ***/
const uint8_t PIN_ENA = 9;   // PWM
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;

/*** I2C direcciones ***/
#define AS5600_ADDR 0x36
#define MPU6050_ADDR 0x68

/*** AS5600 ***/
#define REG_RAW_ANGLE_H 0x0C
#define REG_RAW_ANGLE_L 0x0D

/*** MPU6050 registros mínimos ***/
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_ZOUT_H 0x47

/*** Conversión giroscopio ***/
const float GYRO_SENS = 131.0f;                 // LSB/(deg/s)
const float DEG2RAD   = 3.14159265f/180.0f;

/*** Config desde PC ***/
float TS = 0.01f;         // s
float TEST_TIME = 15.0f;  // s
enum Mode {M_PID, M_HINF, M_SMC, M_APID};
Mode mode = M_PID;
enum Prof {P_STEP, P_RAMP, P_PULSET};
Prof prof = P_STEP;
float ref_deg = 60.0f, ramp_deg_s=10.0f, pulse_amp_deg=15.0f, pulse_T=1.0f;

bool stopFlag = false;

/*** Físico (ajustable) ***/
struct Phys {
  float J   = 1.08e-3f;
  float B   = 2.61e-3f;
  float MgL = 4.999e-3f;
  float Ku  = 1.13e-2f;
  float tauC= 0.0f;
} phys;

/*** PID ***/
struct PIDP { float Kp=2.141f, Ki=2.290f, Kd=0.685f, tau_d=0.05f, Tt=0.5f; } pid;
float I_int=0, D_f=0, e_prev=0;

/*** HINF discreto fijo: u = a1 u(z-1) + b0 e + b1 e(z-1) ***/
struct KH { float a1=0.85f, b0=0.15f, b1=-0.14f, u1=0, e1=0; } kh;

/*** SMC ***/
struct SMC { float lambda=3.6f, k_torque=3.0e-3f, phi=0.02f, alphaVel=0.85f; } smc;

/*** Adaptativo (RLS sobre Ku y B) ***/
struct RLS {
  // theta = [Ku, B]^T
  float Ku_hat=1.13e-2f;
  float B_hat =2.61e-3f;
  float P11=10.0f, P12=0.0f, P22=10.0f;  // matriz P inicial grande
  float lam=0.995f;                      // factor olvido
} rls;

/*** Estado sensores ***/
float offset_rad = 0.0f;
float theta_prev = 0.0f;
float omega_est  = 0.0f;
float gyroZ_prev = 0.0f;

/*** Helpers ***/
static inline float wrapToPi(float x){
  while (x>M_PI) x-=2*M_PI;
  while (x<-M_PI) x+=2*M_PI;
  return x;
}

/*** I2C utils ***/
uint16_t as5600Raw(){
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_RAW_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR,(uint8_t)2);
  if (Wire.available()<2) return 0;
  uint8_t msb=Wire.read(), lsb=Wire.read();
  uint16_t v = ((uint16_t)msb<<8)|lsb; v&=0x0FFF; return v;
}
float as5600Rad(){
  return (float)as5600Raw()*(2.0f*M_PI/4096.0f);
}

void mpuInit(){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_PWR_MGMT_1); Wire.write(0x00); // wake up
  Wire.endTransmission();
  delay(50);
}
float mpuGyroZ_rad(){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,(uint8_t)2);
  if(Wire.available()<2) return 0.0f;
  int16_t gz = ((int16_t)Wire.read()<<8) | Wire.read();
  float degs = (float)gz / GYRO_SENS;    // deg/s
  return degs*DEG2RAD;                   // rad/s
}

/*** Actuación ***/
void driveDuty(float duty){
  duty = fmaxf(fminf(duty,1.0f),-1.0f);
  if (duty >= 0){
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, (int)(duty*255.0f+0.5f));
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, (int)((-duty)*255.0f+0.5f));
  }
}

/*** Parser simple de CFG,... y STOP ***/
String inbuf;
void parseLine(const String& s){
  if (s=="STOP"){ stopFlag=true; return; }
  if (!s.startsWith("CFG")) return;
  // Formato: CFG,TS=x,T=y,MODE=PID/HINF/SMC/ADAPTIVE_PID,PROF=STEP/RAMP/PULSET,REF=,RAMPS=,PAMP=,PT=
  if (s.indexOf("TS=")>0){ TS = s.substring(s.indexOf("TS=")+3).toFloat(); }
  int iT = s.indexOf("T="); if(iT>0){ TEST_TIME = s.substring(iT+2).toFloat(); }
  if (s.indexOf("MODE=PID")>0) mode=M_PID;
  else if (s.indexOf("MODE=HINF")>0) mode=M_HINF;
  else if (s.indexOf("MODE=SMC")>0) mode=M_SMC;
  else if (s.indexOf("MODE=ADAPTIVE_PID")>0) mode=M_APID;
  if (s.indexOf("PROF=STEP")>0) prof=P_STEP;
  else if (s.indexOf("PROF=RAMP")>0) prof=P_RAMP;
  else if (s.indexOf("PROF=PULSET")>0) prof=P_PULSET;
  int ir = s.indexOf("REF=");   if(ir>0){ ref_deg = s.substring(ir+4).toFloat(); }
  int irs= s.indexOf("RAMPS="); if(irs>0){ ramp_deg_s = s.substring(irs+6).toFloat(); }
  int ipa= s.indexOf("PAMP=");  if(ipa>0){ pulse_amp_deg = s.substring(ipa+5).toFloat(); }
  int ipt= s.indexOf("PT=");    if(ipt>0){ pulse_T = s.substring(ipt+3).toFloat(); }
}

/*** PID con derivada filtrada ***/
float pid_step(float e){
  float a = pid.tau_d / (pid.tau_d + TS);
  float b = pid.Kd    / (pid.tau_d + TS);
  D_f = a*D_f + b*(e - e_prev);
  I_int += pid.Ki*TS*e;
  float u = pid.Kp*e + I_int + D_f;
  return u;
}

/*** Hinf discreto ***/
float hinf_step(float e){
  float u = kh.a1*kh.u1 + kh.b0*e + kh.b1*kh.e1;
  kh.u1=u; kh.e1=e; return u;
}

/*** SMC ***/
float smc_step(float e, float omega, float theta){
  float s = omega + smc.lambda*e;
  float u_eq = (phys.J*(-smc.lambda*omega) + phys.B*omega + phys.MgL*sinf(theta)) / phys.Ku;
  float kDuty = rls.Ku_hat>0 ? (smc.k_torque/rls.Ku_hat) : 0.25f;
  float sat = fmaxf(-1.0f,fminf(1.0f, s/smc.phi));
  return u_eq - kDuty*sat;
}

/*** RLS (estima Ku y B) usando J*dw = Ku*u - B*w - MgL*sin(theta) ***/
void rls_update(float u, float omega, float omegadot, float theta){
  // y = J*omegadot + MgL*sin(theta)  = [u, -omega] * [Ku; B]
  float y = phys.J*omegadot + phys.MgL*sinf(theta);
  float phi1 = u;
  float phi2 = -omega;

  // P * phi
  float P11=rls.P11, P12=rls.P12, P22=rls.P22;
  float Pf1 = P11*phi1 + P12*phi2;
  float Pf2 = P12*phi1 + P22*phi2;

  float denom = rls.lam + phi1*Pf1 + phi2*Pf2;
  float k1 = Pf1/denom;
  float k2 = Pf2/denom;

  float err = y - (rls.Ku_hat*phi1 + rls.B_hat*phi2);

  rls.Ku_hat += k1*err;
  rls.B_hat  += k2*err;

  // P = (P - k*phi^T*P)/lam
  rls.P11 = (P11 - k1*Pf1)/rls.lam;
  rls.P12 = (P12 - k1*Pf2)/rls.lam;
  rls.P22 = (P22 - k2*Pf2)/rls.lam;
}

/*** Perfil de referencia ***/
float gen_ref(float t){
  if (prof==P_STEP)  return ref_deg*DEG2RAD;
  if (prof==P_RAMP)  return (ramp_deg_s*DEG2RAD)*t;
  if (prof==P_PULSET){
    float phase = fmodf(t, pulse_T);
    float A = pulse_amp_deg*DEG2RAD;
    return (phase < pulse_T/2)? A : -A;
  }
  return 0.0f;
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  // L298N
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_ENA, OUTPUT);
  driveDuty(0.0f);

  // AS5600 offset
  float acc=0; for(int k=0;k<50;k++){ acc+=as5600Rad(); delay(5); }
  offset_rad = acc/50.0f;

  // MPU6050
  mpuInit();
  gyroZ_prev = mpuGyroZ_rad();

  Serial.println("READY");
}

void loop(){
  // Espera configuración
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n' || c=='\r'){ parseLine(inbuf); inbuf=""; }
    else inbuf += c;
  }
  if (stopFlag){ driveDuty(0); stopFlag=false; Serial.println("DONE,STOP"); return; }

  // Si no hay ejecución, duerme corto
  static bool running=false;
  static unsigned long t0ms=0;
  if (!running){
    // Inicia al recibir 'CFG,' (ya parseado)
    if (TS>0 && TEST_TIME>0){
      running=true; t0ms=millis();
      I_int=0; D_f=0; e_prev=0; kh.u1=0; kh.e1=0;
      theta_prev=0; omega_est=0;
    } else { delay(10); return; }
  }

  // Lazo de control
  unsigned long tms = millis() - t0ms;
  float t = 0.001f * (float)tms;
  static unsigned long nextTick = 0;
  if (millis() < nextTick){ delay(1); return; }
  nextTick = millis() + (unsigned long)(TS*1000.0f);

  // Lecturas
  float theta = wrapToPi(as5600Rad() - offset_rad);
  float gyroZ = mpuGyroZ_rad();
  // filtro vel: combinación simple (puedes ajustar pesos)
  float dtheta = (theta - theta_prev)/TS;
  omega_est = 0.6f*omega_est + 0.4f*gyroZ;     // usa más gyro para rapidez
  float omegadot = (gyroZ - gyroZ_prev)/TS;

  // Referencia
  float theta_ref = gen_ref(t);
  float e = wrapToPi(theta_ref - theta);

  // Feed-forward gravitacional (usa estimados si modo adaptativo)
  float Ku_ff = (mode==M_APID)? rls.Ku_hat : phys.Ku;
  float B_ff  = (mode==M_APID)? rls.B_hat  : phys.B;
  float u_ff = (Ku_ff>0)? (phys.MgL*sinf(theta_ref))/Ku_ff : 0.0f;

  // Control
  float u_ctrl=0;
  if (mode==M_PID || mode==M_APID){
    float u_pid = pid_step(e);
    u_ctrl = u_ff + u_pid;
  } else if (mode==M_HINF){
    u_ctrl = u_ff + hinf_step(e);
  } else if (mode==M_SMC){
    u_ctrl = smc_step(e, omega_est, theta);
  }

  // Saturación
  float u_cmd = fmaxf(fminf(u_ctrl,1.0f), -1.0f);

  // RLS (si adaptativo o siempre para telemetría)
  rls_update(u_cmd, omega_est, omegadot, theta);

  // Seguridad simple
  if (fabs(theta)*180.0f/M_PI > 95.0f){ u_cmd=0; }

  // Actuar
  driveDuty(u_cmd);

  // Telemetría: t,theta_deg,ref_deg,e_deg,omega_deg_s,u,KuHat,BHat,mode
  Serial.print(t,3); Serial.print(',');
  Serial.print(theta*180.0f/M_PI,1); Serial.print(',');
  Serial.print(theta_ref*180.0f/M_PI,1); Serial.print(',');
  Serial.print(e*180.0f/M_PI,1); Serial.print(',');
  Serial.print(omega_est*180.0f/M_PI,1); Serial.print(',');
  Serial.print(u_cmd,3); Serial.print(',');
  Serial.print(rls.Ku_hat,6); Serial.print(',');
  Serial.print(rls.B_hat,6); Serial.print(',');
  if      (mode==M_PID)  Serial.println("PID");
  else if (mode==M_HINF) Serial.println("HINF");
  else if (mode==M_SMC)  Serial.println("SMC");
  else                   Serial.println("ADP");

  // Fin de prueba
  if (t >= TEST_TIME || stopFlag){
    driveDuty(0); running=false; stopFlag=false;
    Serial.println("DONE");
  }

  // Estados
  e_prev=e; theta_prev=theta; gyroZ_prev=gyroZ;
}
