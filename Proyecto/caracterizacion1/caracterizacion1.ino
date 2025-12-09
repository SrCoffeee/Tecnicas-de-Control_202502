/*
* Control de Motor DC con L298N + Lectura de Ángulo AS5600
* Incremento gradual de velocidad con medición de posición angular
*/

#include <Wire.h>

// Pines del L298N
const int ENA = 9;  // Pin PWM para control de velocidad
const int IN1 = 7;  // Pin de dirección 1
const int IN2 = 8;  // Pin de dirección 2

// Dirección I2C del AS5600
const int AS5600_ADDRESS = 0x36;

// Registros del AS5600
const byte RAW_ANGLE_HIGH = 0x0C;
const byte RAW_ANGLE_LOW = 0x0D;

// Variable para almacenar ángulo inicial
float anguloInicial = 0.0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
 
  Wire.begin();
  Serial.begin(9600);
 
  detenerMotor();
 
  Serial.println(F("=== Control de Motor L298N + AS5600 ==="));
  Serial.println(F("Barrido horario: 0% a 25% | Antihorario: 0% a 25%"));
  Serial.println(F("--------------------------------------------------------"));
 
  // Calibrar ángulo inicial
  delay(1000);
  anguloInicial = leerAngulo();
  Serial.print(F("Ángulo inicial calibrado: "));
  Serial.print(anguloInicial, 2);
  Serial.println(F("°"));
  Serial.println(F("--------------------------------------------------------"));
}

void loop() {
  // ---- Sentido Horario ----
  Serial.println(F("\n=== INCREMENTO SENTIDO HORARIO ==="));
  incrementarHorario();
  delay(4000);
 
  // ---- Sentido Antihorario ----
  Serial.println(F("\n=== INCREMENTO SENTIDO ANTIHORARIO ==="));
  incrementarAntihorario();
  delay(4000);
 
  // Repetir ciclo
  Serial.println(F("\n=== Ciclo completado, reiniciando ==="));
  delay(10000);
}

// ===========================================================
// FUNCIÓN 1: BARRIDO EN SENTIDO HORARIO (0% a 25%)
// ===========================================================
void incrementarHorario() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  Serial.println(F("Dirección: Horario"));
  Serial.println(F("Velocidad% | PWM | Ángulo° | Ángulo Relativo°"));
 
  for (float porcentaje = 0.0; porcentaje <= 50; porcentaje += 1) {
    int pwmValue = mapFloat(porcentaje, 0.0, 100.0, 0.0, 255.0);
    analogWrite(ENA, pwmValue);
   
    delay(100); // Breve pausa para estabilización
    float anguloActual = leerAngulo();
    float anguloRelativo = calcularAnguloRelativo(anguloActual, anguloInicial);
   
    // Formato tabular
    Serial.print(porcentaje, 1);
    Serial.print(F("% | "));
    Serial.print(pwmValue);
    Serial.print(F(" | "));
    Serial.print(anguloActual, 2);
    Serial.print(F("° | "));
    Serial.print(anguloRelativo, 2);
    Serial.println(F("°"));
   
    delay(2000); // Completar los 2 segundos totales
  }
 
  detenerMotor();
  Serial.println(F("Motor detenido (fin sentido horario)"));
}

// ===========================================================
// FUNCIÓN 2: BARRIDO EN SENTIDO ANTIHORARIO (0% a 25%)
// ===========================================================
void incrementarAntihorario() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  Serial.println(F("Dirección: Antihorario"));
  Serial.println(F("Velocidad% | PWM | Ángulo° | Ángulo Relativo°"));
 
  for (float porcentaje = 0.0; porcentaje <= 25; porcentaje += 1) {
    int pwmValue = mapFloat(porcentaje, 0.0, 100.0, 0.0, 255.0);
    analogWrite(ENA, pwmValue);
   
    delay(100); // Breve pausa para estabilización
    float anguloActual = leerAngulo();
    float anguloRelativo = calcularAnguloRelativo(anguloActual, anguloInicial);
   
    // Formato tabular
    Serial.print(porcentaje, 1);
    Serial.print(F("% | "));
    Serial.print(pwmValue);

    Serial.print(F(" | "));
    Serial.print(anguloActual, 2);
    Serial.print(F("° | "));
    Serial.print(anguloRelativo, 2);
    Serial.println(F("°"));
   
    delay(2000); // Completar los 2 segundos totales
  }
 
  detenerMotor();
  Serial.println(F("Motor detenido (fin sentido antihorario)"));
}

// ===========================================================
// FUNCIONES DEL AS5600
// ===========================================================
float leerAngulo() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(RAW_ANGLE_HIGH);
  Wire.endTransmission(false);
 
  Wire.requestFrom(AS5600_ADDRESS, 2);
 
  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
   
    int rawAngle = (highByte << 8) | lowByte;
    rawAngle &= 0x0FFF; // Máscara de 12 bits
   
    float angulo = (rawAngle * 360.0) / 4096.0;
    return angulo;
  }
 
  return -1.0; // Error en la lectura
}

// Calcula el ángulo relativo considerando el cruce por 0°/360°
float calcularAnguloRelativo(float anguloActual, float anguloRef) {
  float diferencia = anguloActual - anguloRef;
 
  // Normalizar a rango [-180, 180]
  if (diferencia > 180.0) {
    diferencia -= 360.0;
  } else if (diferencia < -180.0) {
    diferencia += 360.0;
  }
 
  return diferencia;
}

// ===========================================================
// FUNCIONES AUXILIARES
// ===========================================================
int mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void detenerMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}