/* =====================================================================
 *  control_as5600_mpu6050_adaptive.ino
 *  SP en [85°,90°], PWM máx=17, anti-windup >90°, ref relativa a ángulo inicial
 *  HW: Arduino UNO + AS5600 + MPU6050 + L298N (IN1=7, IN2=8, ENA=9)
 *  Controles: PID | HINF | SMC | ADAPTIVE_PID
 * ===================================================================== */
#include <Wire.h>
#include <math.h>

/*** Pines L298N ***/
const uint8_t PIN_ENA = 9;   // PWM
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;

/*** LÍMITES PWM ***/
const uint8_t PWM_MAX_COUNTS   = 50;   // tope duro (visto 16–17)
const uint8_t PWM_IMP_RECOM    = 25;   // sugerencia para impulsos abiertos

/*** I2C direcciones ***/
#define AS5600_ADDR 0x36
#define MPU6050_ADDR 0x68

/*** AS5600 ***/
#define REG_RAW_ANGLE_H 0x0C
#define REG_RAW_ANGLE_L 0x0D

/*** MPU6050 (mínimo) ***/
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_ZOUT_H 0x47

/*** Escalas ***/
const float GYRO_SENS = 131.0f;                 // LSB/(deg/s)
const float DEG2RAD   = 3.14159265f/180.0f;

/*** Config desde PC ***/
float TS = 0.01f;         // s
float TEST_TIME = 15.0f;  // s
enum Mode {M_PID, M_HINF, M_SMC, M_APID};
Mode mode = M_HINF;
enum Prof {P_STEP, P_RAMP, P_PULSET};
Prof prof = P_STEP;

/*** SP forzado al rango [85°,90°] ***/
float ref_deg = 88.0f;                    // nominal
const float REF_MIN_DEG = 85.0f;
const float REF_MAX_DEG = 90.0f;

/*** ¡REAPARECEN ESTAS TRES! (eran la causa del error) ***/
float ramp_deg_s    = 5.0f;   // velocidad de rampa en deg/s
float pulse_amp_deg = 2.0f;   // amplitud del pulso en deg (±)
float pulse_T       = 1.0f;   // periodo del pulso en s
// -------------------------------------------------------

bool stopFlag = false;

/*** Modelo físico (ajustable) ***/
struct Phys { float J=1.08e-3f, B=2.61e-3f, MgL=4.999e-3f, Ku=1.13e-2f, tauC=0.0f; } phys;

/*** PID ***/
struct PIDP { float Kp=2.141f, Ki=2.290f, Kd=0.685f, tau_d=0.05f, Tt=0.5f; } pid;
float I_int=0, D_f=0, e_prev=0;

/*** H∞ discreto fijo ***/
struct KH { float a1=0.85f, b0=0.15f, b1=-0.14f, u1=0, e1=0; } kh;

/*** Modo deslizante ***/
struct SMC { float lambda=3.6f, k_torque=3.0e-3f, phi=0.02f, alphaVel=0.85f; } smc;

/*** Adaptativo (RLS sobre Ku y B) ***/
struct RLS {
  float Ku_hat=1.13e-2f, B_hat=2.61e-3f;
  float P11=10.0f, P12=0.0f, P22=10.0f;
  float lam=0.995f;
} rls;

/*** Estado sensores ***/
float offset_rad = 0.0f;
float theta_prev = 0.0f;
float omega_est  = 0.0f;
float gyroZ_prev = 0.0f;

/*** Ángulo inicial de la PRUEBA (para referencia relativa) ***/
float theta_run0 = 0.0f;

/*** Helpers ***/
static inline float wrapToPi(float x){ while (x>M_PI) x-=2*M_PI; while (x<-M_PI) x+=2*M_PI; return x; }
static inline float clampf(float x, float a, float b){ return (x<a)?a:((x>b)?b:x); }

/*** I2C utils ***/
uint16_t as5600Raw(){
  Wire.beginTransmission(AS5600_ADDR); Wire.write(REG_RAW_ANGLE_H);
  Wire.endTransmission(false); Wire.requestFrom(AS5600_ADDR,(uint8_t)2);
  if (Wire.available()<2) return 0;
  uint8_t msb=Wire.read(), lsb=Wire.read(); uint16_t v=((uint16_t)msb<<8)|lsb; return v&0x0FFF;
}
float as5600Rad(){ return (float)as5600Raw()*(2.0f*M_PI/4096.0f); }

void mpuInit(){ Wire.beginTransmission(MPU6050_ADDR); Wire.write(MPU_PWR_MGMT_1); Wire.write(0x00);
  Wire.endTransmission(); delay(50); }
float mpuGyroZ_rad(){
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(MPU_GYRO_ZOUT_H); Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,(uint8_t)2); if(Wire.available()<2) return 0.0f;
  int16_t gz=((int16_t)Wire.read()<<8)|Wire.read(); return (gz/GYRO_SENS)*DEG2RAD;
}

/*** Actuación: duty [-1,1] -> PWM LIMITADO A 17 CUENTAS ***/
void driveDuty(float duty){
  duty = fmaxf(fminf(duty,1.0f),-1.0f);
  int counts = (int)(fabsf(duty) * PWM_MAX_COUNTS + 0.5f);  // <= 17
  if (duty >= 0){ digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);  analogWrite(PIN_ENA, counts); }
  else           { digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH); analogWrite(PIN_ENA, counts); }
}

/*** Parser simple de CFG,... y STOP ***/
String inbuf;
void parseLine(const String& s){
  if (s=="STOP"){ stopFlag=true; return; }
  if (!s.startsWith("CFG")) return;
  if (s.indexOf("TS=")>0){ TS = s.substring(s.indexOf("TS=")+3).toFloat(); }
  int iT = s.indexOf("T="); if(iT>0){ TEST_TIME = s.substring(iT+2).toFloat(); }
  if (s.indexOf("MODE=PID")>0) mode=M_PID;
  else if (s.indexOf("MODE=HINF")>0) mode=M_HINF;
  else if (s.indexOf("MODE=SMC")>0) mode=M_SMC;
  else if (s.indexOf("MODE=ADAPTIVE_PID")>0) mode=M_APID;
  if (s.indexOf("PROF=STEP")>0) prof=P_STEP;
  else if (s.indexOf("PROF=RAMP")>0) prof=P_RAMP;
  else if (s.indexOf("PROF=PULSET")>0) prof=P_PULSET;

  int ir = s.indexOf("REF="); if(ir>0){ ref_deg = s.substring(ir+4).toFloat(); }
  ref_deg = clampf(ref_deg, REF_MIN_DEG, REF_MAX_DEG);

  int irs= s.indexOf("RAMPS="); if(irs>0){ ramp_deg_s    = s.substring(irs+6).toFloat(); }
  int ipa= s.indexOf("PAMP=");  if(ipa>0){ pulse_amp_deg = s.substring(ipa+5).toFloat(); }
  int ipt= s.indexOf("PT=");    if(ipt>0){ pulse_T       = s.substring(ipt+3).toFloat(); }
}

/*** PID con derivada filtrada; I se ajusta con anti-windup después de saturar ***/
float pid_PDpart(float e){
  float a = pid.tau_d / (pid.tau_d + TS);
  float b = pid.Kd    / (pid.tau_d + TS);
  D_f = a*D_f + b*(e - e_prev);
  return pid.Kp*e + D_f;  // P + D filtrada
}

/*** H∞ discreto ***/
float hinf_step(float e){ float u = kh.a1*kh.u1 + kh.b0*e + kh.b1*kh.e1; kh.u1=u; kh.e1=e; return u; }

/*** SMC ***/
float smc_step(float e, float omega, float theta){
  float s = omega + smc.lambda*e;
  float u_eq = (phys.J*(-smc.lambda*omega) + phys.B*omega + phys.MgL*sinf(theta)) / phys.Ku;
  float kDuty = rls.Ku_hat>0 ? (smc.k_torque/rls.Ku_hat) : 0.25f;
  float sat = fmaxf(-1.0f,fminf(1.0f, s/smc.phi));
  return u_eq - kDuty*sat;
}

/*** RLS (estima Ku y B) ***/
void rls_update(float u, float omega, float omegadot, float theta){
  float y = phys.J*omegadot + phys.MgL*sinf(theta);
  float phi1 = u, phi2 = -omega;
  float P11=rls.P11, P12=rls.P12, P22=rls.P22;
  float Pf1 = P11*phi1 + P12*phi2, Pf2 = P12*phi1 + P22*phi2;
  float denom = rls.lam + phi1*Pf1 + phi2*Pf2;
  float k1 = Pf1/denom, k2 = Pf2/denom;
  float err = y - (rls.Ku_hat*phi1 + rls.B_hat*phi2);
  rls.Ku_hat += k1*err; rls.B_hat += k2*err;
  rls.P11 = (P11 - k1*Pf1)/rls.lam; rls.P12 = (P12 - k1*Pf2)/rls.lam; rls.P22 = (P22 - k2*Pf2)/rls.lam;
}

/*** Referencia ABSOLUTA respecto al ángulo inicial; clamp a [85°,90°] ***/
float gen_ref_abs(float t, float theta0){
  float lo = theta0 + REF_MIN_DEG*DEG2RAD, hi = theta0 + REF_MAX_DEG*DEG2RAD;
  float cand = theta0;
  if (prof==P_STEP){
    cand = theta0 + clampf(ref_deg, REF_MIN_DEG, REF_MAX_DEG)*DEG2RAD;
  } else if (prof==P_RAMP){
    float off_deg = ramp_deg_s*t;                      // desde 0°
    off_deg = clampf(off_deg, REF_MIN_DEG, REF_MAX_DEG);
    cand = theta0 + off_deg*DEG2RAD;
  } else if (prof==P_PULSET){
    float center = (REF_MIN_DEG+REF_MAX_DEG)*0.5f;     // 87.5°
    float phase = fmodf(t, pulse_T);
    float off = (phase < pulse_T*0.5f) ? +pulse_amp_deg : -pulse_amp_deg;
    cand = theta0 + (center + off)*DEG2RAD;
  }
  return clampf(cand, lo, hi);
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_ENA, OUTPUT);
  driveDuty(0.0f);

  float acc=0; for(int k=0;k<50;k++){ acc+=as5600Rad(); delay(5); } offset_rad = acc/50.0f;
  mpuInit(); gyroZ_prev = mpuGyroZ_rad();
  Serial.println("READY");
}

void loop(){
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n' || c=='\r'){ parseLine(inbuf); inbuf=""; }
    else inbuf += c;
  }
  if (stopFlag){ driveDuty(0); stopFlag=false; Serial.println("DONE,STOP"); return; }

  static bool running=false; static unsigned long t0ms=0;
  if (!running){
    if (TS>0 && TEST_TIME>0){
      theta_run0 = wrapToPi(as5600Rad() - offset_rad);   // toma θ0
      running=true; t0ms=millis();
      I_int=0; D_f=0; e_prev=0; kh.u1=0; kh.e1=0; theta_prev=0; omega_est=0;
      ref_deg = clampf(ref_deg, REF_MIN_DEG, REF_MAX_DEG);
    } else { delay(10); return; }
  }

  // Tick
  static unsigned long nextTick = 0;
  if (millis() < nextTick){ delay(1); return; }
  nextTick = millis() + (unsigned long)(TS*1000.0f);
  float t = 0.001f * (millis() - t0ms);

  // Sensores
  float theta = wrapToPi(as5600Rad() - offset_rad);
  float gyroZ = mpuGyroZ_rad();
  omega_est = 0.6f*omega_est + 0.4f*gyroZ;
  float omegadot = (gyroZ - gyroZ_prev)/TS;

  // Referencia relativa a θ0 con clamp [85°,90°]
  float theta_ref = gen_ref_abs(t, theta_run0);
  float e = wrapToPi(theta_ref - theta);
  float e_for_I = e;

  // Feed-forward (usa estimados en adaptativo)
  float Ku_ff = (mode==M_APID)? rls.Ku_hat : phys.Ku;
  float u_ff  = (Ku_ff>0)? (phys.MgL*sinf(theta_ref))/Ku_ff : 0.0f;

  // --- Control y ANTI-WINDUP ---
  float u_cmd = 0.0f;
  if (mode==M_PID || mode==M_APID){
    float u_PD = pid_PDpart(e);
    float I_before = I_int;

    // Congelar integrador si |theta - theta0| > 90°
    float theta_rel = wrapToPi(theta - theta_run0);
    bool over90 = fabsf(theta_rel) > (90.0f*DEG2RAD);
    if (!over90) I_int = I_int + pid.Ki*TS*e_for_I;

    float u_unsat = u_ff + (u_PD + I_int);
    float u_sat = fmaxf(fminf(u_unsat, 1.0f), -1.0f);

    // Anti-windup por retrocálculo
    I_int += (TS/pid.Tt) * (u_sat - u_unsat);

    if (over90) { // si congelado, sólo aplica el término AW
      I_int = I_before + (TS/pid.Tt) * (u_sat - (u_ff + u_PD + I_before));
    }
    u_cmd = u_sat;
  }
  else if (mode==M_HINF){
    u_cmd = u_ff + hinf_step(e);
    u_cmd = fmaxf(fminf(u_cmd, 1.0f), -1.0f);
  }
  else if (mode==M_SMC){
    u_cmd = smc_step(e, omega_est, theta);
    u_cmd = fmaxf(fminf(u_cmd, 1.0f), -1.0f);
  }

  // Seguridad dura
  float theta_rel_now = wrapToPi(theta - theta_run0);
  if (fabs(theta_rel_now)*180.0f/M_PI > 95.0f){ u_cmd=0; }

  // Actuar (máx 17 cuentas reales)
  driveDuty(u_cmd);

  // Adaptativo (telemetría)
  rls_update(u_cmd, omega_est, omegadot, theta);

  // Telemetría
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

  if (t >= TEST_TIME || stopFlag){
    driveDuty(0); running=false; stopFlag=false; Serial.println("DONE");
  }

  e_prev=e; theta_prev=theta; gyroZ_prev=gyroZ;
}
