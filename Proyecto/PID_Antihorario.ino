/*
 * CONTROLADOR PID - AEROBALANCÍN 
 * ====================================================
 * Sistema: Motor DC con L298N + Sensor AS5600
 * 
 * Parámetros PID (continuos):
 * - Kp = 2.14
 * - Ki = 2.29
 * - Kd = 0.685
 * 
 * Discretización: Tustin (Ts = 0.1 s)
 * Punto de operación: ~25% PWM
 * 
 * ANTI-WINDUP: Fuga exponencial del integrador
 */

#include <Wire.h>

// =============== PINES DEL L298N ===============
const int ENA = 9;   // PWM
const int IN1 = 7;   // Dirección 1
const int IN2 = 8;   // Dirección 2

// =============== AS5600 (Sensor de ángulo) ===============
const int AS5600_ADDRESS = 0x36;
const byte RAW_ANGLE_HIGH = 0x0C;
const byte RAW_ANGLE_LOW = 0x0D;

// =============== PARÁMETROS DEL PID ===============
const float Kp = 2.14;      // Ganancia proporcional
const float Ki = 3.29;      // Ganancia integral
const float Kd = 0.685;     // Ganancia derivativa

const float Ts = 0.100;     // Tiempo de muestreo [s] (100 ms)
const float N = 10.0;       // Coeficiente de filtro derivativo

// Coeficientes discretos (método Tustin)
const float Ki_discrete = Ki * Ts / 2.0;
const float Kd_num = (2.0 * Kd) / (2.0 + N * Ts);
const float Kd_den = (2.0 - N * Ts) / (2.0 + N * Ts);

// =============== ANTI-WINDUP POR FUGA ===============
const float LEAK_RATE = 0.95;        // Factor de fuga (0-1)
                                      // 0.95 = retiene 95% por ciclo
                                      // 0.90 = fuga más agresiva
const float INTEGRAL_MAX = 20.0;     // Límite absoluto del integrador

// =============== LÍMITES Y SATURACIÓN ===============
const float PWM_MIN = 0.0;      // PWM mínimo [%]
const float PWM_MAX = 15.0;    // PWM máximo [%]

// =============== VARIABLES DEL CONTROLADOR ===============
float setpoint = 75.0;         // Ángulo deseado [grados]
float anguloInicial = 0.0;     // Ángulo de referencia inicial

// Variables del PID
float error_anterior = 0.0;
float integral = 0.0;
float derivada_anterior = 0.0;

// Variables de tiempo
unsigned long tiempoAnterior = 0;
unsigned long tiempoInicio = 0;

// =============== CONFIGURACIÓN DE PRUEBA ===============
const int TIEMPO_PRUEBA = 120*1000;      // Duración [ms]
bool controlActivo = false;

void setup() {
  // Configurar pines
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Inicializar I2C y Serial
  Wire.begin();
  Serial.begin(9600);
  
  // Motor detenido
  detenerMotor();
  
  // Mensaje de bienvenida
  Serial.println(F("========================================"));
  Serial.println(F("  CONTROLADOR PID - AEROBALANCÍN"));
  Serial.println(F("  Sentido: ANTIHORARIO"));
  Serial.println(F("  Anti-Windup: FUGA EXPONENCIAL"));
  Serial.println(F("========================================"));
  Serial.println();
  
  // Mostrar parámetros
  Serial.println(F("PARÁMETROS:"));
  Serial.print(F("  Kp = ")); Serial.println(Kp, 3);
  Serial.print(F("  Ki = ")); Serial.println(Ki, 3);
  Serial.print(F("  Kd = ")); Serial.println(Kd, 3);
  Serial.print(F("  Ts = ")); Serial.print(Ts * 1000); Serial.println(F(" ms"));
  Serial.print(F("  Leak Rate = ")); Serial.println(LEAK_RATE, 3);
  Serial.print(F("  Integral Max = ")); Serial.println(INTEGRAL_MAX, 2);
  Serial.println();
  
  // Calibración
  Serial.println(F(">> Calibrando ángulo inicial..."));
  delay(2000);
  
  anguloInicial = leerAngulo();
  Serial.print(F(">> Ángulo inicial: "));
  Serial.print(anguloInicial, 2);
  Serial.println(F("°"));
  
  Serial.print(F(">> Setpoint: "));
  Serial.print(setpoint, 2);
  Serial.println(F("°"));
  Serial.println();
  
  delay(1000);
  
  // Configurar dirección ANTIHORARIA (fija)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Iniciar control
  controlActivo = true;
  tiempoInicio = millis();
  tiempoAnterior = tiempoInicio;
  
  // Encabezado CSV
  Serial.println(F("===CSV_PID_START==="));
  Serial.println(F("Tiempo_ms,Setpoint_deg,Angulo_deg,Error_deg,Integral,PWM_%,PWM_val"));
  
  Serial.println(F(">> Control PID activado (ANTIHORARIO)"));
  Serial.println();
}

void loop() {
  unsigned long tiempoActual = millis();
  unsigned long tiempoTranscurrido = tiempoActual - tiempoInicio;
  
  // Verificar fin de prueba
  if (tiempoTranscurrido >= TIEMPO_PRUEBA && controlActivo) {
    controlActivo = false;
    detenerMotor();
    
    Serial.println(F("===CSV_PID_END==="));
    Serial.println();
    Serial.println(F(">> Prueba finalizada"));
    
    while (true) { delay(1000); }
  }
  
  // Ejecutar control cada Ts
  if (controlActivo && (tiempoActual - tiempoAnterior >= Ts * 1000)) {
    tiempoAnterior = tiempoActual;
    
    // Leer ángulo
    float anguloAbsoluto = leerAngulo();
    float anguloActual = calcularAnguloRelativo(anguloAbsoluto, anguloInicial);
    
    // Calcular control PID
    float pwmPorcentaje = controlPID(setpoint, anguloActual);
    
    // Aplicar al motor (ANTIHORARIO)
    aplicarPWM(pwmPorcentaje);
    
    // Registrar datos
    registrarDatos(tiempoTranscurrido, anguloActual, pwmPorcentaje);
  }
}

// ═══════════════════════════════════════════════════════
// CONTROLADOR PID DISCRETO CON ANTI-WINDUP POR FUGA
// ═══════════════════════════════════════════════════════
float controlPID(float referencia, float medicion) {
  // Error
  float error = referencia - medicion;
  
  // ---- ANTI-WINDUP: FUGA EXPONENCIAL ----
  // Aplica decaimiento exponencial al integrador antes de actualizar
  integral *= LEAK_RATE;
  
  // Término Proporcional
  float P = Kp * error;
  
  // Término Integral (Tustin/Trapezoidal)
  float integral_temp = integral + Ki_discrete * (error + error_anterior);
  
  // Limitar el integrador para evitar acumulación infinita
  integral_temp = constrain(integral_temp, -INTEGRAL_MAX, INTEGRAL_MAX);
  
  // Término Derivativo (con filtro)
  float D = Kd_num * (error - error_anterior) + Kd_den * derivada_anterior;
  
  // Señal de control total
  float u_temp = P + integral_temp + D;
  
  // Saturación
  float u = constrain(u_temp, PWM_MIN, PWM_MAX);
  
  // Anti-windup condicional adicional (opcional, pero recomendado)
  // Solo actualiza el integrador si no está saturado O si el error ayuda a salir
  if ((u_temp >= PWM_MIN && u_temp <= PWM_MAX) || 
      (u_temp < PWM_MIN && error > 0) || 
      (u_temp > PWM_MAX && error < 0)) {
    integral = integral_temp;
  }
  // Si está saturado y el error empeora la saturación, NO actualizar
  // (el integrador ya tiene fuga aplicada)
  
  // Actualizar estados
  error_anterior = error;
  derivada_anterior = D;
  
  return u;
}

// ═══════════════════════════════════════════════════════
// APLICAR PWM AL MOTOR (ANTIHORARIO)
// ═══════════════════════════════════════════════════════
void aplicarPWM(float pwmPorcentaje) {
  // Convertir porcentaje a valor PWM (0-255)
  int pwmValue = int((pwmPorcentaje / 100.0) * 255.0);
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Aplicar PWM (dirección ya configurada en setup)
  analogWrite(ENA, pwmValue);
}

// ═══════════════════════════════════════════════════════
// REGISTRO DE DATOS
// ═══════════════════════════════════════════════════════
void registrarDatos(unsigned long tiempo, float angulo, float pwmPorcent) {
  float error = setpoint - angulo;
  int pwmValue = int((pwmPorcent / 100.0) * 255.0);
  
  // Formato: Tiempo,Setpoint,Angulo,Error,Integral,PWM_%,PWM_val
  Serial.print(tiempo);
  Serial.print(F(","));
  Serial.print(setpoint, 2);
  Serial.print(F(","));
  Serial.print(angulo, 2);
  Serial.print(F(","));
  Serial.print(error, 2);
  Serial.print(F(","));
  Serial.print(integral, 3);  // Registrar el valor del integrador
  Serial.print(F(","));
  Serial.print(pwmPorcent, 2);
  Serial.print(F(","));
  Serial.println(pwmValue);
}
 
// ═══════════════════════════════════════════════════════
// FUNCIONES DEL AS5600
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
    return angulo;
  }
  
  return -1.0;
}

float calcularAnguloRelativo(float anguloActual, float anguloRef) {
  float diferencia = anguloActual - anguloRef;
  
  if (diferencia > 180.0) {
    diferencia -= 360.0;
  } else if (diferencia < -180.0) {
    diferencia += 360.0;
  }
  
  return diferencia;
}

// ═══════════════════════════════════════════════════════
// DETENER MOTOR
// ═══════════════════════════════════════════════════════
void detenerMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}