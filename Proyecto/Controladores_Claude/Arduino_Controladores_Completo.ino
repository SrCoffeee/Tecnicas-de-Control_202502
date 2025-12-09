/*
 * SISTEMA DE CONTROL AVANZADO - AEROBALANCÍN
 * ===========================================
 * Controladores: PID, H∞, SMC, ADAPTATIVO
 * Sensores: AS5600 (ángulo) + MPU6050 (IMU)
 * Comunicación: Serial con MATLAB/Python
 * 
 * COMANDOS SERIAL:
 * - START:1  → Iniciar control PID
 * - START:2  → Iniciar control H∞
 * - START:3  → Iniciar control SMC
 * - START:4  → Iniciar control ADAPTATIVO
 * - STOP     → Detener control
 * - SET:45.5 → Cambiar setpoint
 * - REF:0    → Referencia escalón
 * - REF:1    → Referencia rampa
 * - REF:2    → Referencia tren de impulsos
 * - IDENT    → Modo identificación de parámetros
 * - STATUS   → Estado actual del sistema
 * - CALIB    → Calibrar sensores
 */

#include <Wire.h>
#include <MPU6050.h>

// =============== PINES DEL L298N ===============
const int ENA = 9;   // PWM
const int IN1 = 7;   // Dirección 1
const int IN2 = 8;   // Dirección 2

// =============== SENSORES ===============
const int AS5600_ADDRESS = 0x36;
const byte RAW_ANGLE_HIGH = 0x0C;
const byte RAW_ANGLE_LOW = 0x0D;

MPU6050 mpu;

// =============== ENUMERACIONES ===============
enum TipoControlador {
  CTRL_NINGUNO = 0,
  CTRL_PID = 1,
  CTRL_HINF = 2,
  CTRL_SMC = 3,
  CTRL_ADAPTATIVO = 4
};

enum TipoReferencia {
  REF_ESCALON = 0,
  REF_RAMPA = 1,
  REF_IMPULSOS = 2,
  REF_MANUAL = 3
};

enum ModoOperacion {
  MODO_DETENIDO = 0,
  MODO_CONTROL = 1,
  MODO_IDENTIFICACION = 2
};

// =============== VARIABLES GLOBALES ===============
TipoControlador controladorActual = CTRL_NINGUNO;
TipoReferencia tipoReferencia = REF_ESCALON;
ModoOperacion modoActual = MODO_DETENIDO;

// =============== PARÁMETROS COMUNES ===============
const float Ts = 0.020;     // 20ms - 50Hz (mejor para control)
const float PWM_MIN = 0.0;
const float PWM_MAX = 38.0;

float setpoint = 0.0;
float anguloInicial = 0.0;
float setpointBase = 45.0;  // Setpoint por defecto

// =============== CONTROL PID ===============
float Kp = 2.14;
float Ki = 2.29;
float Kd = 0.685;
const float N = 10.0;

float Ki_discrete = Ki * Ts / 2.0;
float Kd_num = (2.0 * Kd) / (2.0 + N * Ts);
float Kd_den = (2.0 - N * Ts) / (2.0 + N * Ts);

float pid_error_anterior = 0.0;
float pid_integral = 0.0;
float pid_derivada_anterior = 0.0;

// =============== CONTROL H∞ ===============
const float hinf_num[4] = {1.850, -4.920, 4.350, -1.270};
const float hinf_den[4] = {1.000, -2.650, 2.310, -0.660};

float hinf_u[3] = {0.0, 0.0, 0.0};
float hinf_y[3] = {0.0, 0.0, 0.0};

const float Ku = 0.011;
const float M = 0.150;
const float g = 9.81;
const float L = 0.50;

// =============== CONTROL SMC ===============
float lambda_smc = 3.6;
float phi_smc = 0.02;
float k_smc = 0.27;

const float J = 0.0625;
float B = 0.015;  // No const para adaptativo

float smc_error_anterior = 0.0;
float smc_angulo_anterior = 0.0;
float smc_velocidad = 0.0;

// =============== CONTROL ADAPTATIVO ===============
// Modelo de referencia: sistema de 2do orden
float omega_ref = 3.43;  // Frecuencia natural deseada
float zeta_ref = 0.7;    // Factor de amortiguamiento deseado

// Ganancias adaptativas (MIT rule)
float gamma1 = 0.5;  // Ganancia de adaptación proporcional
float gamma2 = 0.3;  // Ganancia de adaptación derivativa
float gamma3 = 0.1;  // Ganancia de adaptación integral

// Parámetros del controlador adaptativo
float theta1 = 2.0;  // Ganancia proporcional adaptativa
float theta2 = 0.5;  // Ganancia derivativa adaptativa
float theta3 = 1.0;  // Ganancia integral adaptativa

// Estados del modelo de referencia
float xm1 = 0.0;  // Posición del modelo
float xm2 = 0.0;  // Velocidad del modelo

// Error de adaptación
float adapt_integral = 0.0;

// Variables MPU6050
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// Filtro complementario
float alpha_filter = 0.98;
float angulo_filtrado = 0.0;

// =============== REFERENCIAS ===============
unsigned long tiempoReferenciaInicio = 0;
float amplitudRampa = 0.5; // grados/segundo
float amplitudImpulso = 30.0; // grados
float periodoImpulso = 2000; // ms

// =============== IDENTIFICACIÓN ===============
bool modoIdentificacion = false;
float pwmIdentificacion = 25.0;
unsigned long tiempoIdentInicio = 0;
const unsigned long TIEMPO_IDENT = 10000; // 10 segundos

// =============== TIEMPO ===============
unsigned long tiempoAnterior = 0;
unsigned long tiempoInicio = 0;
unsigned long tiempoActual = 0;

// =============== BUFFER SERIAL ===============
String comandoSerial = "";
bool comandoCompleto = false;

// =============== SETUP ===============
void setup() {
  // Configurar pines
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Inicializar comunicación
  Serial.begin(115200);
  Wire.begin();
  
  // Configurar dirección ANTIHORARIA
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Motor detenido
  detenerMotor();
  
  // Inicializar MPU6050
  mpu.initialize();
  
  if (mpu.testConnection()) {
    enviarMensaje("MPU6050_OK");
  } else {
    enviarMensaje("MPU6050_ERROR");
  }
  
  // Calibrar MPU6050 (rápido)
  mpu.setXAccelOffset(-1343);
  mpu.setYAccelOffset(-1155);
  mpu.setZAccelOffset(1165);
  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(-7);
  mpu.setZGyroOffset(24);
  
  // Esperar estabilización
  delay(500);
  
  // Calibrar ángulo inicial
  calibrarSensores();
  
  enviarMensaje("SYSTEM_READY");
  
  tiempoAnterior = micros();
}

// =============== LOOP PRINCIPAL ===============
void loop() {
  // Procesar comandos seriales
  if (Serial.available()) {
    procesarComandoSerial();
  }
  
  // Control en tiempo real
  unsigned long tiempoActualMicros = micros();
  float dt = (tiempoActualMicros - tiempoAnterior) / 1000000.0;
  
  if (dt >= Ts) {
    tiempoAnterior = tiempoActualMicros;
    tiempoActual = millis();
    
    // Leer sensores
    float anguloAS5600 = leerAngulo();
    leerMPU6050();
    float anguloActual = fusionarSensores(anguloAS5600);
    
    // Generar referencia
    if (modoActual == MODO_CONTROL) {
      setpoint = generarReferencia();
    }
    
    // Calcular control
    float pwmPorcentaje = 0.0;
    
    if (modoActual == MODO_CONTROL && controladorActual != CTRL_NINGUNO) {
      switch (controladorActual) {
        case CTRL_PID:
          pwmPorcentaje = controlPID(setpoint, anguloActual);
          break;
          
        case CTRL_HINF:
          pwmPorcentaje = controlHInf(setpoint, anguloActual);
          break;
          
        case CTRL_SMC:
          pwmPorcentaje = controlSMC(setpoint, anguloActual);
          break;
          
        case CTRL_ADAPTATIVO:
          pwmPorcentaje = controlAdaptativo(setpoint, anguloActual, dt);
          break;
      }
      
      aplicarPWM(pwmPorcentaje);
    } 
    else if (modoActual == MODO_IDENTIFICACION) {
      aplicarPWM(pwmIdentificacion);
      
      // Finalizar identificación después del tiempo
      if (millis() - tiempoIdentInicio >= TIEMPO_IDENT) {
        detenerMotor();
        modoActual = MODO_DETENIDO;
        enviarMensaje("IDENT_COMPLETE");
      }
    }
    
    // Enviar datos
    enviarDatos(anguloActual, pwmPorcentaje);
  }
}

// ═══════════════════════════════════════════════════════
// CONTROLADORES
// ═══════════════════════════════════════════════════════

float controlPID(float ref, float medicion) {
  float error = ref - medicion;
  
  float P = Kp * error;
  float integral_temp = pid_integral + Ki_discrete * (error + pid_error_anterior);
  float D = Kd_num * (error - pid_error_anterior) + Kd_den * pid_derivada_anterior;
  
  float u_temp = P + integral_temp + D;
  float u = constrain(u_temp, PWM_MIN, PWM_MAX);
  
  // Anti-windup
  if ((u_temp >= PWM_MIN && u_temp <= PWM_MAX) || 
      (u_temp < PWM_MIN && error > 0) || 
      (u_temp > PWM_MAX && error < 0)) {
    pid_integral = integral_temp;
  }
  
  pid_error_anterior = error;
  pid_derivada_anterior = D;
  
  return u;
}

float controlHInf(float ref, float medicion) {
  float error = ref - medicion;
  
  float u_feedback = hinf_num[0] * error 
                   + hinf_num[1] * hinf_u[0] 
                   + hinf_num[2] * hinf_u[1] 
                   + hinf_num[3] * hinf_u[2]
                   - hinf_den[1] * hinf_y[0]
                   - hinf_den[2] * hinf_y[1]
                   - hinf_den[3] * hinf_y[2];
  
  float angulo_rad = medicion * PI / 180.0;
  float u_ff = (M * g * L * sin(angulo_rad)) / Ku;
  u_ff = (u_ff / 0.011) * 100.0 / 255.0;
  
  float u_temp = u_feedback + u_ff;
  float u = constrain(u_temp, PWM_MIN, PWM_MAX);
  
  hinf_u[2] = hinf_u[1];
  hinf_u[1] = hinf_u[0];
  hinf_u[0] = error;
  
  hinf_y[2] = hinf_y[1];
  hinf_y[1] = hinf_y[0];
  hinf_y[0] = u_feedback;
  
  return u;
}

float controlSMC(float ref, float medicion) {
  float error = ref - medicion;
  float error_dot = (error - smc_error_anterior) / Ts;
  float s = error_dot + lambda_smc * error;
  
  float angulo_rad = medicion * PI / 180.0;
  float angulo_rad_ant = smc_angulo_anterior * PI / 180.0;
  float omega = (angulo_rad - angulo_rad_ant) / Ts;
  
  float theta_ddot_ref = 0.0;
  float torque_eq = J * (theta_ddot_ref - lambda_smc * error_dot)
                  + B * omega
                  + M * g * L * sin(angulo_rad);
  
  float u_eq = torque_eq / Ku;
  float u_rob = -k_smc * sat(s, phi_smc);
  float u_duty = u_eq + u_rob;
  float u_pwm = u_duty * 100.0;
  float u = constrain(u_pwm, PWM_MIN, PWM_MAX);
  
  smc_error_anterior = error;
  smc_angulo_anterior = medicion;
  smc_velocidad = omega;
  
  return u;
}

float controlAdaptativo(float ref, float medicion, float dt) {
  // Error de seguimiento
  float error = ref - medicion;
  
  // Derivada del error (aproximación)
  static float error_prev = 0.0;
  float error_dot = (error - error_prev) / dt;
  error_prev = error;
  
  // Integración del error
  adapt_integral += error * dt;
  
  // Modelo de referencia (sistema de 2do orden)
  // ẍm + 2ζωₙẋm + ωₙ²xm = ωₙ²r
  float xm1_dot = xm2;
  float xm2_dot = -2.0 * zeta_ref * omega_ref * xm2 
                  - omega_ref * omega_ref * xm1 
                  + omega_ref * omega_ref * ref;
  
  // Integración (Euler)
  xm1 += xm1_dot * dt;
  xm2 += xm2_dot * dt;
  
  // Error entre modelo y planta
  float e1 = medicion - xm1;
  
  // Señal de control adaptativo
  float u_adapt = theta1 * error + theta2 * error_dot + theta3 * adapt_integral;
  
  // Ley de adaptación (MIT rule con normalización)
  float epsilon = 0.01; // Factor de normalización
  float norm_factor = 1.0 / (1.0 + error * error + epsilon);
  
  // Actualizar parámetros
  theta1 -= gamma1 * e1 * error * norm_factor * dt;
  theta2 -= gamma2 * e1 * error_dot * norm_factor * dt;
  theta3 -= gamma3 * e1 * adapt_integral * norm_factor * dt;
  
  // Limitar parámetros (proyección)
  theta1 = constrain(theta1, 0.5, 5.0);
  theta2 = constrain(theta2, 0.1, 2.0);
  theta3 = constrain(theta3, 0.1, 3.0);
  
  // Compensación gravitacional
  float angulo_rad = medicion * PI / 180.0;
  float u_ff = (M * g * L * sin(angulo_rad)) / Ku;
  u_ff = (u_ff / 0.011) * 100.0 / 255.0;
  
  // Control total
  float u_total = u_adapt + u_ff;
  
  return constrain(u_total, PWM_MIN, PWM_MAX);
}

// ═══════════════════════════════════════════════════════
// GENERACIÓN DE REFERENCIAS
// ═══════════════════════════════════════════════════════

float generarReferencia() {
  unsigned long tiempoTranscurrido = millis() - tiempoReferenciaInicio;
  float t = tiempoTranscurrido / 1000.0; // segundos
  
  switch (tipoReferencia) {
    case REF_ESCALON:
      return setpointBase;
      
    case REF_RAMPA:
      return min(setpointBase, amplitudRampa * t);
      
    case REF_IMPULSOS:
      {
        unsigned long periodo = tiempoTranscurrido % periodoImpulso;
        if (periodo < periodoImpulso / 2) {
          return amplitudImpulso;
        } else {
          return -amplitudImpulso;
        }
      }
      
    case REF_MANUAL:
    default:
      return setpointBase;
  }
}

// ═══════════════════════════════════════════════════════
// SENSORES
// ═══════════════════════════════════════════════════════

float leerAngulo() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(RAW_ANGLE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int rawAngle = (highByte << 8) | lowByte;
    rawAngle &= 0x0FFF;
    float angulo = (rawAngle * 360.0) / 4096.0;
    
    // Calcular relativo
    float diferencia = angulo - anguloInicial;
    if (diferencia > 180.0) diferencia -= 360.0;
    if (diferencia < -180.0) diferencia += 360.0;
    
    return diferencia;
  }
  
  return 0.0;
}

void leerMPU6050() {
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
  
  // Convertir a unidades físicas
  accel_x = ax_raw / 16384.0;
  accel_y = ay_raw / 16384.0;
  accel_z = az_raw / 16384.0;
  
  gyro_x = gx_raw / 131.0;
  gyro_y = gy_raw / 131.0;
  gyro_z = gz_raw / 131.0;
}

float fusionarSensores(float anguloAS5600) {
  // Ángulo del acelerómetro (pitch)
  float angulo_acel = atan2(accel_y, accel_z) * 180.0 / PI;
  
  // Filtro complementario
  angulo_filtrado = alpha_filter * (angulo_filtrado + gyro_x * Ts) 
                  + (1.0 - alpha_filter) * angulo_acel;
  
  // Fusión: priorizar AS5600 (más preciso en posición)
  // MPU6050 para dinámica rápida
  float angulo_fusionado = 0.8 * anguloAS5600 + 0.2 * angulo_filtrado;
  
  return angulo_fusionado;
}

void calibrarSensores() {
  delay(1000);
  
  // Promedio de múltiples lecturas
  float suma = 0.0;
  for (int i = 0; i < 20; i++) {
    suma += leerAngulo() + anguloInicial; // Leer absoluto
    delay(50);
  }
  
  anguloInicial = suma / 20.0;
  
  // Resetear MPU
  leerMPU6050();
  angulo_filtrado = atan2(accel_y, accel_z) * 180.0 / PI;
}

// ═══════════════════════════════════════════════════════
// COMANDOS SERIAL
// ═══════════════════════════════════════════════════════

void procesarComandoSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (comandoSerial.length() > 0) {
        ejecutarComando(comandoSerial);
        comandoSerial = "";
      }
    } else {
      comandoSerial += c;
    }
  }
}

void ejecutarComando(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("START:")) {
    int tipo = cmd.substring(6).toInt();
    iniciarControl(tipo);
  }
  else if (cmd == "STOP") {
    detenerControl();
  }
  else if (cmd.startsWith("SET:")) {
    float nuevo_setpoint = cmd.substring(4).toFloat();
    setpointBase = nuevo_setpoint;
    enviarMensaje("SETPOINT:" + String(setpointBase, 2));
  }
  else if (cmd.startsWith("REF:")) {
    int tipo_ref = cmd.substring(4).toInt();
    tipoReferencia = (TipoReferencia)tipo_ref;
    tiempoReferenciaInicio = millis();
    enviarMensaje("REF_TYPE:" + String(tipo_ref));
  }
  else if (cmd == "IDENT") {
    iniciarIdentificacion();
  }
  else if (cmd == "STATUS") {
    enviarEstado();
  }
  else if (cmd == "CALIB") {
    calibrarSensores();
    enviarMensaje("CALIBRATED");
  }
  else if (cmd.startsWith("KP:")) {
    Kp = cmd.substring(3).toFloat();
    recalcularPID();
  }
  else if (cmd.startsWith("KI:")) {
    Ki = cmd.substring(3).toFloat();
    recalcularPID();
  }
  else if (cmd.startsWith("KD:")) {
    Kd = cmd.substring(3).toFloat();
    recalcularPID();
  }
}

void iniciarControl(int tipo) {
  controladorActual = (TipoControlador)tipo;
  modoActual = MODO_CONTROL;
  tiempoInicio = millis();
  tiempoReferenciaInicio = millis();
  
  // Resetear estados
  resetearEstados();
  
  String nombre = "";
  switch (controladorActual) {
    case CTRL_PID: nombre = "PID"; break;
    case CTRL_HINF: nombre = "HINF"; break;
    case CTRL_SMC: nombre = "SMC"; break;
    case CTRL_ADAPTATIVO: nombre = "ADAPTIVE"; break;
  }
  
  enviarMensaje("STARTED:" + nombre);
}

void detenerControl() {
  modoActual = MODO_DETENIDO;
  controladorActual = CTRL_NINGUNO;
  detenerMotor();
  enviarMensaje("STOPPED");
}

void iniciarIdentificacion() {
  modoActual = MODO_IDENTIFICACION;
  tiempoIdentInicio = millis();
  enviarMensaje("IDENT_STARTED");
}

void resetearEstados() {
  pid_error_anterior = 0.0;
  pid_integral = 0.0;
  pid_derivada_anterior = 0.0;
  
  for (int i = 0; i < 3; i++) {
    hinf_u[i] = 0.0;
    hinf_y[i] = 0.0;
  }
  
  smc_error_anterior = 0.0;
  smc_angulo_anterior = 0.0;
  smc_velocidad = 0.0;
  
  adapt_integral = 0.0;
  xm1 = 0.0;
  xm2 = 0.0;
}

void recalcularPID() {
  Ki_discrete = Ki * Ts / 2.0;
  Kd_num = (2.0 * Kd) / (2.0 + N * Ts);
  Kd_den = (2.0 - N * Ts) / (2.0 + N * Ts);
}

// ═══════════════════════════════════════════════════════
// COMUNICACIÓN
// ═══════════════════════════════════════════════════════

void enviarDatos(float angulo, float pwm) {
  // Formato: DATA,tiempo,setpoint,angulo,error,pwm,gyro_x,theta1,theta2,theta3
  Serial.print("DATA,");
  Serial.print(millis() - tiempoInicio);
  Serial.print(",");
  Serial.print(setpoint, 3);
  Serial.print(",");
  Serial.print(angulo, 3);
  Serial.print(",");
  Serial.print(setpoint - angulo, 3);
  Serial.print(",");
  Serial.print(pwm, 3);
  Serial.print(",");
  Serial.print(gyro_x, 3);
  Serial.print(",");
  Serial.print(theta1, 4);
  Serial.print(",");
  Serial.print(theta2, 4);
  Serial.print(",");
  Serial.println(theta3, 4);
}

void enviarMensaje(String mensaje) {
  Serial.println("MSG," + mensaje);
}

void enviarEstado() {
  Serial.print("STATUS,");
  Serial.print("MODE:");
  Serial.print(modoActual);
  Serial.print(",CTRL:");
  Serial.print(controladorActual);
  Serial.print(",REF:");
  Serial.print(tipoReferencia);
  Serial.print(",SP:");
  Serial.println(setpointBase, 2);
}

// ═══════════════════════════════════════════════════════
// UTILIDADES
// ═══════════════════════════════════════════════════════

void aplicarPWM(float pwmPorcentaje) {
  int pwmValue = int((pwmPorcentaje / 100.0) * 255.0);
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(ENA, pwmValue);
}

void detenerMotor() {
  analogWrite(ENA, 0);
}

float sat(float x, float limite) {
  if (x > limite) return 1.0;
  if (x < -limite) return -1.0;
  return x / limite;
}
