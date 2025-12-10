/*
* Control de Motor DC con L298N + Lectura de Ángulo AS5600
* 3 Fases: IMPULSO -> STEP -> BARRIDO INCREMENTAL
* Salida por Serial para capturar con PuTTY
*/

#include <Wire.h>

// Pines del L298N
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Dirección I2C del AS5600
const int AS5600_ADDRESS = 0x36;
const byte RAW_ANGLE_HIGH = 0x0C;
const byte RAW_ANGLE_LOW = 0x0D;

// =============== PARÁMETROS DE PRUEBA ===============
// FASE 1: IMPULSO INICIAL
const float PWM_IMPULSO = 20.0;        // Potencia del impulso
const int TIEMPO_IMPULSO = 500;        // Duración del impulso
const int MUESTRAS_IMPULSO = 50;       // Lecturas durante el impulso
const int INTERVALO_IMPULSO = 10;      // ms entre lecturas

// FASE 2: STEP (escalón)
const float PWM_STEP = 20.0;           // Potencia del step
const int TIEMPO_STEP = 5000;          // Duración total del step
const int MUESTRAS_STEP = 50;          // Número de lecturas
const int INTERVALO_STEP = 100;        // ms entre lecturas

// FASE 3: BARRIDO INCREMENTAL
const float PWM_MINIMO = 5.0;         // Inicio del barrido
const float PWM_MAXIMO = 45.0;         // Fin del barrido
const float INCREMENTO_PWM = 1.0;      // Incremento por paso
const int TIEMPO_ESTABILIZACION = 3000; // Tiempo por paso

// Variable para ángulo inicial
float anguloInicial = 0.0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
 
  Wire.begin();
  Serial.begin(9600);
 
  detenerMotor();
 
  Serial.println(F("========================================"));
  Serial.println(F("  SISTEMA DE CONTROL AEROBALANCÍN"));
  Serial.println(F("  L298N + AS5600"));
  Serial.println(F("  SENTIDO HORARIO - 3 FASES"));
  Serial.println(F("========================================"));
  Serial.println();
 
  // Calibrar ángulo inicial
  delay(2000);
  anguloInicial = leerAngulo();
  Serial.print(F(">> Ángulo inicial calibrado: "));
  Serial.print(anguloInicial, 2);
  Serial.println(F("°"));
  Serial.println();
  delay(1000);
}

void loop() {
  // =======================================
  // SENTIDO HORARIO - 3 FASES
  // =======================================
  Serial.println(F("\n╔═══════════════════════════════════════╗"));
  Serial.println(F("║     INICIANDO SENTIDO HORARIO        ║"));
  Serial.println(F("╚═══════════════════════════════════════╝\n"));
 
  ejecutarFaseImpulso();
  delay(2000);
 
  ejecutarFaseStep();
  delay(2000);
 
  ejecutarFaseBarrido();
  delay(5000);
 
  // Fin del ciclo
  Serial.println(F("\n╔═══════════════════════════════════════╗"));
  Serial.println(F("║   CICLO COMPLETADO - PAUSA 10s       ║"));
  Serial.println(F("╚═══════════════════════════════════════╝\n"));
  delay(10000);
}

// ═══════════════════════════════════════════════════════
// FASE 1: IMPULSO INICIAL
// ═══════════════════════════════════════════════════════
void ejecutarFaseImpulso() {
  Serial.println(F("┌───────────────────────────────────────┐"));
  Serial.println(F("│  FASE 1: IMPULSO INICIAL             │"));
  Serial.println(F("└───────────────────────────────────────┘"));
 
  Serial.println(F("Dirección: Horario"));
  Serial.print(F("PWM Impulso: "));
  Serial.print(PWM_IMPULSO, 1);
  Serial.print(F("% | Duración: "));
  Serial.print(TIEMPO_IMPULSO);
  Serial.println(F("ms"));
  Serial.println();
 
  // Configurar dirección horaria
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
 
  // Encabezado CSV
  Serial.println(F("===CSV_IMPULSO_START==="));
  Serial.println(F("Velocidad%,PWM,AnguloRelativo"));
 
  int pwmValue = mapFloat(PWM_IMPULSO, 0.0, 100.0, 0.0, 255.0);
  analogWrite(ENA, pwmValue);
 
  // Tomar múltiples lecturas durante el impulso
  for (int i = 0; i < MUESTRAS_IMPULSO; i++) {
    float anguloActual = leerAngulo();
    float anguloRelativo = calcularAnguloRelativo(anguloActual, anguloInicial);
   
    // Formato CSV
    Serial.print(PWM_IMPULSO, 1);
    Serial.print(F(","));
    Serial.print(pwmValue);
    Serial.print(F(","));
    Serial.println(anguloRelativo, 2);
   
    delay(INTERVALO_IMPULSO);
  }
 
  Serial.println(F("===CSV_IMPULSO_END==="));
  Serial.println();
 
  detenerMotor();
  Serial.println(F("✓ Impulso completado\n"));
}

// ═══════════════════════════════════════════════════════
// FASE 2: STEP (ESCALÓN)
// ═══════════════════════════════════════════════════════
void ejecutarFaseStep() {
  Serial.println(F("┌───────────────────────────────────────┐"));
  Serial.println(F("│  FASE 2: STEP (ESCALÓN)              │"));
  Serial.println(F("└───────────────────────────────────────┘"));
 
  Serial.println(F("Dirección: Horario"));
  Serial.print(F("PWM Step: "));
  Serial.print(PWM_STEP, 1);
  Serial.print(F("% | Duración: "));
  Serial.print(TIEMPO_STEP);
  Serial.println(F("ms"));
  Serial.println();
 
  // Configurar dirección horaria
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
 
  // Encabezado CSV
  Serial.println(F("===CSV_STEP_START==="));
  Serial.println(F("Velocidad%,PWM,AnguloRelativo"));
 
  int pwmValue = mapFloat(PWM_STEP, 0.0, 100.0, 0.0, 255.0);
  analogWrite(ENA, pwmValue);
 
  // Tomar múltiples lecturas durante el step
  for (int i = 0; i < MUESTRAS_STEP; i++) {
    float anguloActual = leerAngulo();
    float anguloRelativo = calcularAnguloRelativo(anguloActual, anguloInicial);
   
    // Formato CSV
    Serial.print(PWM_STEP, 1);
    Serial.print(F(","));
    Serial.print(pwmValue);
    Serial.print(F(","));
    Serial.println(anguloRelativo, 2);
   
    delay(INTERVALO_STEP);
  }
 
  Serial.println(F("===CSV_STEP_END==="));
  Serial.println();
 
  detenerMotor();
  Serial.println(F("✓ Step completado\n"));
}

// ═══════════════════════════════════════════════════════
// FASE 3: BARRIDO INCREMENTAL
// ═══════════════════════════════════════════════════════
void ejecutarFaseBarrido() {
  Serial.println(F("┌───────────────────────────────────────┐"));
  Serial.println(F("│  FASE 3: BARRIDO INCREMENTAL         │"));
  Serial.println(F("└───────────────────────────────────────┘"));
 
  Serial.println(F("Dirección: Horario"));
  Serial.print(F("Rango: "));
  Serial.print(PWM_MINIMO, 1);
  Serial.print(F("% a "));
  Serial.print(PWM_MAXIMO, 1);
  Serial.print(F("% | Incremento: "));
  Serial.print(INCREMENTO_PWM, 1);
  Serial.println(F("%"));
  Serial.println();
 
  // Configurar dirección horaria
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
 
  // Encabezado CSV
  Serial.println(F("===CSV_BARRIDO_START==="));
  Serial.println(F("Velocidad%,PWM,AnguloRelativo"));
 
  for (float porcentaje = PWM_MINIMO; porcentaje <= PWM_MAXIMO; porcentaje += INCREMENTO_PWM) {
    int pwmValue = mapFloat(porcentaje, 0.0, 100.0, 0.0, 255.0);
    analogWrite(ENA, pwmValue);
   
    // Esperar estabilización
    delay(TIEMPO_ESTABILIZACION);
   
    // Leer ángulo estabilizado
    float anguloActual = leerAngulo();
    float anguloRelativo = calcularAnguloRelativo(anguloActual, anguloInicial);
   
    // Formato CSV
    Serial.print(porcentaje, 1);
    Serial.print(F(","));
    Serial.print(pwmValue);
    Serial.print(F(","));
    Serial.println(anguloRelativo, 2);
  }
 
  Serial.println(F("===CSV_BARRIDO_END==="));
  Serial.println();
 
  detenerMotor();
  Serial.println(F("✓ Barrido completado\n"));
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
// FUNCIONES AUXILIARES
// ═══════════════════════════════════════════════════════
int mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void detenerMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}