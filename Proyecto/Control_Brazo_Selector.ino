/*
 * CONTROL DE BRAZO ROBOTICO - SELECTOR DE CONTROLADORES
 * Controladores: PID / H-infinity / SMC (Sliding Mode Control)
 * Hardware: Arduino Uno + AS5600 + L298N + Micromotor
 * 
 * Selección por Serial Monitor:
 * Enviar '1' para PID
 * Enviar '2' para H-infinity  
 * Enviar '3' para SMC
 * Enviar 'r' para cambiar referencia (ej: r45 para 45 grados)
 */

#include <Wire.h>

// ==================== CONFIGURACION HARDWARE ====================
// AS5600 (Sensor de posición magnético)
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_HIGH 0x0C
#define AS5600_RAW_ANGLE_LOW 0x0D

// L298N (Driver de motor)
#define PIN_IN1  5   // Control dirección motor
#define PIN_IN2  6   // Control dirección motor
#define PIN_ENA  9   // PWM velocidad motor (debe ser pin PWM)

// ==================== PARAMETROS DEL SISTEMA ====================
const float J   = 1.08e-3;      // Inercia [kg*m^2]
const float Bv  = 2.61e-3;      // Fricción viscosa [N*m*s/rad]
const float MgL = 4.999e-3;     // Torque gravitacional [N*m]
const float Ku  = 1.13e-2;      // Constante de torque [N*m/duty]
const float th0 = 50.0 * PI/180.0; // Punto de operación [rad]

const float Ts = 0.01;          // Periodo de muestreo [s] = 10ms
const unsigned long Ts_ms = 10; // 10 milisegundos

// ==================== PARAMETROS PID ====================
const float Kp_pid = 5.088;     // Ganancia proporcional
const float Ki_pid = 7.0707;    // Ganancia integral
const float Kd_pid = 0.486;     // Ganancia derivativa
const float tau_d  = 0.05;      // Constante filtro derivativo
const float Tt     = 0.35;      // Constante anti-windup

// ==================== PARAMETROS H-INFINITY ====================
// Coeficientes del controlador discretizado (obtener de MATLAB)
// Forma: u[k] = a1*u[k-1] + b0*e[k] + b1*e[k-1]
float kh_a1 = 0.0;  // Coeficiente recursivo (actualizar con valores de MATLAB)
float kh_b0 = 1.5;  // Coeficiente error actual
float kh_b1 = 0.0;  // Coeficiente error previo

// ==================== PARAMETROS SMC ====================
const float lambda_smc   = 3.0;   // Pendiente superficie deslizante [rad/s]
const float k_torque_smc = 0.02;  // Ganancia conmutación [N*m]
const float phi_smc      = 0.5;   // Capa límite [rad/s]

// ==================== VARIABLES GLOBALES ====================
// Controlador seleccionado: 1=PID, 2=HINF, 3=SMC
int controllerMode = 1;

// Variables de estado
float theta = 0.0;      // Posición angular actual [rad]
float omega = 0.0;      // Velocidad angular actual [rad/s]
float theta_prev = 0.0; // Posición anterior para calcular velocidad
float ref = th0;        // Referencia angular [rad]

// Variables PID
float Iint_pid = 0.0;   // Integrador
float Df_pid = 0.0;     // Filtro derivativo
float eprev_pid = 0.0;  // Error anterior

// Variables H-infinity
float z_state1 = 0.0;   // Estado del filtro IIR
float eprev_hinf = 0.0; // Error anterior

// Variables SMC
float omega_filt = 0.0; // Velocidad filtrada

// Control
float u_control = 0.0;  // Señal de control [-1, 1]
unsigned long lastTime = 0;

// ==================== SETUP ====================
void setup() {
  // Inicializar Serial
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println(F("==========================================="));
  Serial.println(F("CONTROL BRAZO ROBOTICO - SELECTOR"));
  Serial.println(F("==========================================="));
  Serial.println(F("Controladores disponibles:"));
  Serial.println(F("  1 - PID"));
  Serial.println(F("  2 - H-infinity"));
  Serial.println(F("  3 - SMC (Sliding Mode Control)"));
  Serial.println(F(""));
  Serial.println(F("Comandos:"));
  Serial.println(F("  1, 2, 3  -> Seleccionar controlador"));
  Serial.println(F("  r<valor> -> Cambiar referencia (ej: r45)"));
  Serial.println(F("==========================================="));
  Serial.println();
  
  // Inicializar I2C para AS5600
  Wire.begin();
  Wire.setClock(400000); // 400kHz
  
  // Configurar pines L298N
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  
  // Motor apagado inicialmente
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);
  
  // Leer posición inicial
  theta = readAngleAS5600();
  theta_prev = theta;
  
  delay(100);
  Serial.print(F("Controlador inicial: PID"));
  Serial.print(F(" | Ref: "));
  Serial.print(ref * 180.0/PI);
  Serial.println(F(" grados"));
  
  lastTime = millis();
}

// ==================== LOOP PRINCIPAL ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Control a frecuencia fija (10ms)
  if (currentTime - lastTime >= Ts_ms) {
    lastTime = currentTime;
    
    // Leer posición del sensor
    theta = readAngleAS5600();
    
    // Estimar velocidad (derivada numérica con filtro simple)
    omega = (theta - theta_prev) / Ts;
    omega_filt = 0.9 * omega_filt + 0.1 * omega; // Filtro pasa-bajos
    theta_prev = theta;
    
    // Calcular señal de control según modo seleccionado
    switch (controllerMode) {
      case 1:
        u_control = computePID();
        break;
      case 2:
        u_control = computeHInfinity();
        break;
      case 3:
        u_control = computeSMC();
        break;
      default:
        u_control = 0.0;
    }
    
    // Aplicar control al motor
    applyControl(u_control);
    
    // Enviar datos por serial cada 100ms
    static unsigned long printTime = 0;
    if (currentTime - printTime >= 100) {
      printTime = currentTime;
      printData();
    }
  }
  
  // Procesar comandos del Serial
  processSerialCommands();
}

// ==================== CONTROLADOR PID ====================
float computePID() {
  float e = ref - theta;
  
  // Feedforward (compensación gravedad)
  float u_ff = (MgL / Ku) * sin(ref);
  
  // Filtro derivativo de primer orden
  float a = tau_d / (tau_d + Ts);
  float b = Kd_pid / (tau_d + Ts);
  Df_pid = a * Df_pid + b * (e - eprev_pid);
  
  // Señal de control sin saturar
  float u_unsat = u_ff + (Kp_pid * e + Df_pid + Iint_pid);
  
  // Saturación
  float u = constrain(u_unsat, -1.0, 1.0);
  
  // Anti-windup (back-calculation)
  Iint_pid = Iint_pid + Ki_pid * Ts * e + (Ts / Tt) * (u - u_unsat);
  
  eprev_pid = e;
  
  return u;
}

// ==================== CONTROLADOR H-INFINITY ====================
float computeHInfinity() {
  float e = ref - theta;
  
  // Feedforward
  float u_ff = (MgL / Ku) * sin(ref);
  
  // Filtro IIR discreto (Direct Form II Transposed)
  // y[k] = b0*x[k] + z1[k-1]
  // z1[k] = b1*x[k] - a1*y[k]
  
  float x = e;
  float y = kh_b0 * x + z_state1;
  
  z_state1 = kh_b1 * x - kh_a1 * y;
  
  // Control total con saturación
  float u = constrain(u_ff + y, -1.0, 1.0);
  
  return u;
}

// ==================== CONTROLADOR SMC ====================
float computeSMC() {
  // Error de posición (theta - ref) según convención del MATLAB
  float e_theta = theta - ref;
  
  // Superficie deslizante: s = omega + lambda * e_theta
  float s = omega_filt + lambda_smc * e_theta;
  
  // Control equivalente
  float u_eq = (J * (-lambda_smc * omega_filt) + Bv * omega_filt + MgL * sin(theta)) / Ku;
  
  // Término de conmutación con función tanh (suavizado)
  float u_sw = -(k_torque_smc / Ku) * tanh(s / phi_smc);
  
  // Ley de control total
  float u = u_eq + u_sw;
  
  // Saturación
  u = constrain(u, -1.0, 1.0);
  
  return u;
}

// ==================== LECTURA AS5600 ====================
float readAngleAS5600() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_RAW_ANGLE_HIGH);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() == 2) {
    uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
    // Convertir de 0-4095 a radianes (0-2π)
    float angle = (rawAngle * 2.0 * PI) / 4096.0;
    return angle;
  }
  
  return theta; // Si falla, mantener valor anterior
}

// ==================== CONTROL MOTOR L298N ====================
void applyControl(float u) {
  // u está en rango [-1, 1]
  // Convertir a PWM [0-255]
  int pwm = abs(u) * 255.0;
  pwm = constrain(pwm, 0, 255);
  
  if (u > 0.01) {
    // Giro horario
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, pwm);
  } else if (u < -0.01) {
    // Giro antihorario
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, pwm);
  } else {
    // Zona muerta - motor apagado
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  }
}

// ==================== IMPRIMIR DATOS ====================
void printData() {
  Serial.print(F("Mode:"));
  switch (controllerMode) {
    case 1: Serial.print(F("PID")); break;
    case 2: Serial.print(F("HINF")); break;
    case 3: Serial.print(F("SMC")); break;
  }
  
  Serial.print(F(" | Ref:"));
  Serial.print(ref * 180.0/PI, 2);
  
  Serial.print(F(" | Theta:"));
  Serial.print(theta * 180.0/PI, 2);
  
  Serial.print(F(" | Error:"));
  Serial.print((ref - theta) * 180.0/PI, 2);
  
  Serial.print(F(" | Omega:"));
  Serial.print(omega_filt, 3);
  
  Serial.print(F(" | u:"));
  Serial.println(u_control, 3);
}

// ==================== PROCESAR COMANDOS SERIAL ====================
void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Cambiar controlador
    if (cmd == '1' || cmd == '2' || cmd == '3') {
      controllerMode = cmd - '0';
      
      // Resetear variables de cada controlador
      Iint_pid = 0.0;
      Df_pid = 0.0;
      eprev_pid = 0.0;
      z_state1 = 0.0;
      eprev_hinf = 0.0;
      omega_filt = omega;
      
      Serial.println();
      Serial.print(F(">>> Controlador cambiado a: "));
      switch (controllerMode) {
        case 1: Serial.println(F("PID")); break;
        case 2: Serial.println(F("H-infinity")); break;
        case 3: Serial.println(F("SMC")); break;
      }
    }
    
    // Cambiar referencia
    else if (cmd == 'r' || cmd == 'R') {
      delay(10); // Esperar más datos
      if (Serial.available() > 0) {
        float refDegrees = Serial.parseFloat();
        if (refDegrees >= 0 && refDegrees <= 180) {
          ref = refDegrees * PI / 180.0;
          Serial.println();
          Serial.print(F(">>> Nueva referencia: "));
          Serial.print(refDegrees);
          Serial.println(F(" grados"));
        }
      }
    }
    
    // Limpiar buffer
    while (Serial.available() > 0) Serial.read();
  }
}

// ==================== FUNCIÓN TANH (para SMC) ====================
float tanh(float x) {
  if (x > 5.0) return 1.0;
  if (x < -5.0) return -1.0;
  float exp2x = exp(2.0 * x);
  return (exp2x - 1.0) / (exp2x + 1.0);
}
