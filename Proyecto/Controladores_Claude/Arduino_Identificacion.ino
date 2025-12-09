/*
 * IDENTIFICACIÓN DE PARÁMETROS - AEROBALANCÍN
 * ============================================
 * Determina: ωn, ζ, B (fricción)
 * Método: Respuesta a escalón con análisis espectral
 * 
 * PROTOCOLO:
 * 1. Aplicar escalón de PWM
 * 2. Registrar respuesta libre
 * 3. Enviar datos por serial
 * 4. Procesar en MATLAB/Python
 */

#include <Wire.h>

// =============== PINES ===============
const int ENA = 9;
const int IN1 = 7;
const int IN2 = 8;

// =============== AS5600 ===============
const int AS5600_ADDRESS = 0x36;
const byte RAW_ANGLE_HIGH = 0x0C;
const byte RAW_ANGLE_LOW = 0x0D;

// =============== PARÁMETROS ===============
const float Ts = 0.010;  // 10ms - Mayor resolución
const int NUM_MUESTRAS = 500;  // 5 segundos de datos

// Estados
enum Fase {
  ESPERA,
  ESCALON,
  DECAIMIENTO,
  COMPLETADO
};

Fase faseActual = ESPERA;

// Variables
float anguloInicial = 0.0;
float datos[NUM_MUESTRAS];
unsigned long tiempos[NUM_MUESTRAS];
int indiceMuestra = 0;

const float PWM_ESCALON = 30.0;  // Escalón de 30%
const int DURACION_ESCALON = 100;  // 1 segundo
const int DURACION_DECAIMIENTO = 400;  // 4 segundos

unsigned long tiempoFaseInicio = 0;
bool datosListos = false;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  Serial.begin(115200);
  Wire.begin();
  
  detenerMotor();
  
  Serial.println(F("=== IDENTIFICACIÓN DE PARÁMETROS ==="));
  Serial.println(F("Comandos:"));
  Serial.println(F("  START - Iniciar identificación"));
  Serial.println(F("  DATA  - Obtener datos"));
  Serial.println();
  
  delay(2000);
  calibrarAngulo();
  
  Serial.println(F("Sistema listo"));
}

void loop() {
  // Procesar comandos
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "START") {
      iniciarIdentificacion();
    } else if (cmd == "DATA" && datosListos) {
      enviarDatos();
    }
  }
  
  // Máquina de estados
  static unsigned long tiempoAnterior = 0;
  unsigned long tiempoActual = millis();
  
  if (faseActual != ESPERA && faseActual != COMPLETADO) {
    if (tiempoActual - tiempoAnterior >= Ts * 1000) {
      tiempoAnterior = tiempoActual;
      
      // Leer ángulo
      float angulo = leerAnguloRelativo();
      
      // Almacenar datos
      if (indiceMuestra < NUM_MUESTRAS) {
        datos[indiceMuestra] = angulo;
        tiempos[indiceMuestra] = tiempoActual - tiempoFaseInicio;
        indiceMuestra++;
      }
      
      // Control de fases
      unsigned long tiempoEnFase = tiempoActual - tiempoFaseInicio;
      
      switch (faseActual) {
        case ESCALON:
          aplicarPWM(PWM_ESCALON);
          if (tiempoEnFase >= DURACION_ESCALON * Ts * 1000) {
            faseActual = DECAIMIENTO;
            detenerMotor();
            Serial.println(F("Fase: Decaimiento libre"));
          }
          break;
          
        case DECAIMIENTO:
          // Motor apagado, solo registrar
          if (tiempoEnFase >= (DURACION_ESCALON + DURACION_DECAIMIENTO) * Ts * 1000) {
            faseActual = COMPLETADO;
            datosListos = true;
            Serial.println(F("Identificación completa"));
            Serial.println(F("Envíe 'DATA' para obtener resultados"));
          }
          break;
          
        default:
          break;
      }
    }
  }
}

void iniciarIdentificacion() {
  Serial.println(F("=== INICIANDO IDENTIFICACIÓN ==="));
  
  // Resetear
  indiceMuestra = 0;
  datosListos = false;
  faseActual = ESCALON;
  tiempoFaseInicio = millis();
  
  Serial.println(F("Fase: Escalón"));
}

void calibrarAngulo() {
  Serial.println(F("Calibrando..."));
  delay(1000);
  
  float suma = 0.0;
  for (int i = 0; i < 50; i++) {
    suma += leerAngulo();
    delay(20);
  }
  
  anguloInicial = suma / 50.0;
  Serial.print(F("Ángulo inicial: "));
  Serial.print(anguloInicial, 2);
  Serial.println(F("°"));
}

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
    return (rawAngle * 360.0) / 4096.0;
  }
  
  return 0.0;
}

float leerAnguloRelativo() {
  float angulo = leerAngulo();
  float diferencia = angulo - anguloInicial;
  
  if (diferencia > 180.0) diferencia -= 360.0;
  if (diferencia < -180.0) diferencia += 360.0;
  
  return diferencia;
}

void enviarDatos() {
  Serial.println(F("=== DATOS DE IDENTIFICACIÓN ==="));
  Serial.print(F("Muestras: "));
  Serial.println(indiceMuestra);
  Serial.print(F("Ts: "));
  Serial.print(Ts * 1000, 1);
  Serial.println(F(" ms"));
  Serial.println(F("DATA_START"));
  
  for (int i = 0; i < indiceMuestra; i++) {
    Serial.print(tiempos[i]);
    Serial.print(F(","));
    Serial.println(datos[i], 4);
  }
  
  Serial.println(F("DATA_END"));
}

void aplicarPWM(float pwm) {
  int valor = (int)((pwm / 100.0) * 255.0);
  valor = constrain(valor, 0, 255);
  analogWrite(ENA, valor);
}

void detenerMotor() {
  analogWrite(ENA, 0);
}
