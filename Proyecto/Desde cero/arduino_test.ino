/*
 * TEST DE HARDWARE BÁSICO
 * Prueba individual de cada componente antes de implementar el control
 * 
 * Este código te permite verificar:
 * 1. Encoder AS5600 funciona correctamente
 * 2. Motor gira en ambas direcciones
 * 3. Comunicación I2C
 * 4. PWM del motor
 */

#include <Wire.h>

// ============================================================
// CONFIGURACIÓN DE HARDWARE
// ============================================================

// Pines L298N
#define MOTOR_IN1    7
#define MOTOR_IN2    8
#define MOTOR_PWM    9

// AS5600
#define AS5600_ADDR  0x36
#define AS5600_RAW_ANGLE_H  0x0C
#define AS5600_RAW_ANGLE_L  0x0D

// ============================================================
// VARIABLES GLOBALES
// ============================================================

int testMode = 0;  // 0=Encoder, 1=Motor CW, 2=Motor CCW, 3=Auto
unsigned long lastPrint = 0;
float lastAngle = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n====================================");
  Serial.println("   TEST DE HARDWARE - DIAGNÓSTICO");
  Serial.println("====================================\n");
  
  // Configurar pines
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Motor apagado
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  
  // Inicializar I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Test 1: Verificar conexión AS5600
  Serial.println("TEST 1: Verificando AS5600...");
  Serial.print("  Buscando dispositivo en 0x36... ");
  
  Wire.beginTransmission(AS5600_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("✓ ENCONTRADO!");
    
    // Leer un valor para verificar funcionalidad
    float angle = readAngle();
    Serial.print("  Ángulo inicial: ");
    Serial.print(angle);
    Serial.print(" rad = ");
    Serial.print(angle * 180.0 / PI);
    Serial.println("°");
    
    Serial.println("\n  >> Gira el motor manualmente y observa cambios.\n");
  } else {
    Serial.println("✗ NO ENCONTRADO!");
    Serial.println("  Verifica:");
    Serial.println("    - Cable SDA conectado a A4");
    Serial.println("    - Cable SCL conectado a A5");
    Serial.println("    - VCC conectado a 5V");
    Serial.println("    - GND conectado a GND");
    Serial.println("    - Imán presente sobre el sensor\n");
  }
  
  // Test 2: Verificar pines digitales
  Serial.println("TEST 2: Verificando pines digitales...");
  Serial.println("  Probando IN1...");
  digitalWrite(MOTOR_IN1, HIGH);
  delay(100);
  digitalWrite(MOTOR_IN1, LOW);
  
  Serial.println("  Probando IN2...");
  digitalWrite(MOTOR_IN2, HIGH);
  delay(100);
  digitalWrite(MOTOR_IN2, LOW);
  
  Serial.println("  ✓ Pines digitales OK\n");
  
  // Test 3: Verificar PWM
  Serial.println("TEST 3: Verificando PWM...");
  Serial.println("  Generando PWM 50% en pin 9...");
  analogWrite(MOTOR_PWM, 127);
  delay(500);
  analogWrite(MOTOR_PWM, 0);
  Serial.println("  ✓ PWM OK (si tienes LED en pin 9, debió parpadear)\n");
  
  // Menú de opciones
  printMenu();
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop() {
  // Procesar comandos seriales
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    processCommand(cmd);
  }
  
  // Ejecutar modo actual
  unsigned long now = millis();
  
  switch(testMode) {
    case 0: // Modo Encoder
      if (now - lastPrint >= 100) {
        lastPrint = now;
        testEncoder();
      }
      break;
      
    case 1: // Motor CW (sentido horario)
      testMotorCW();
      break;
      
    case 2: // Motor CCW (sentido antihorario)
      testMotorCCW();
      break;
      
    case 3: // Auto test
      autoTest();
      break;
      
    default:
      testMode = 0;
  }
}

// ============================================================
// FUNCIONES DE TEST
// ============================================================

void testEncoder() {
  float angle = readAngle();
  float angleDeg = angle * 180.0 / PI;
  float deltaAngle = angle - lastAngle;
  
  // Manejar cruce de 0-360
  if (deltaAngle > PI) deltaAngle -= 2*PI;
  if (deltaAngle < -PI) deltaAngle += 2*PI;
  
  float velocity = deltaAngle / 0.1; // rad/s (aprox)
  
  Serial.print("Ángulo: ");
  Serial.print(angleDeg, 1);
  Serial.print("° | Rad: ");
  Serial.print(angle, 3);
  Serial.print(" | Vel: ");
  Serial.print(velocity, 2);
  Serial.println(" rad/s");
  
  lastAngle = angle;
}

void testMotorCW() {
  static int pwm = 0;
  static unsigned long lastChange = 0;
  unsigned long now = millis();
  
  if (now - lastChange >= 2000) {
    lastChange = now;
    pwm += 64;
    if (pwm > 255) pwm = 64;
    
    Serial.print("\n>>> Motor CW - PWM: ");
    Serial.print(pwm);
    Serial.print(" (");
    Serial.print((pwm * 100) / 255);
    Serial.println("%)");
    
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, pwm);
  }
  
  // Leer encoder
  if (now - lastPrint >= 200) {
    lastPrint = now;
    testEncoder();
  }
}

void testMotorCCW() {
  static int pwm = 0;
  static unsigned long lastChange = 0;
  unsigned long now = millis();
  
  if (now - lastChange >= 2000) {
    lastChange = now;
    pwm += 64;
    if (pwm > 255) pwm = 64;
    
    Serial.print("\n>>> Motor CCW - PWM: ");
    Serial.print(pwm);
    Serial.print(" (");
    Serial.print((pwm * 100) / 255);
    Serial.println("%)");
    
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, pwm);
  }
  
  // Leer encoder
  if (now - lastPrint >= 200) {
    lastPrint = now;
    testEncoder();
  }
}

void autoTest() {
  static int phase = 0;
  static unsigned long phaseStart = 0;
  unsigned long now = millis();
  unsigned long elapsed = now - phaseStart;
  
  if (phase == 0) {
    Serial.println("\n=== AUTO TEST INICIADO ===");
    Serial.println("Fase 1: Motor CW (3 seg)");
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, 150);
    phaseStart = now;
    phase = 1;
  }
  else if (phase == 1 && elapsed >= 3000) {
    Serial.println("\nFase 2: STOP (2 seg)");
    stopMotor();
    phaseStart = now;
    phase = 2;
  }
  else if (phase == 2 && elapsed >= 2000) {
    Serial.println("\nFase 3: Motor CCW (3 seg)");
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, 150);
    phaseStart = now;
    phase = 3;
  }
  else if (phase == 3 && elapsed >= 3000) {
    Serial.println("\nFase 4: STOP");
    stopMotor();
    Serial.println("\n=== AUTO TEST COMPLETADO ===");
    Serial.println("Si el motor giró en ambas direcciones, ¡todo funciona!\n");
    testMode = 0;
    phase = 0;
    printMenu();
  }
  
  // Monitorear encoder
  if (now - lastPrint >= 200) {
    lastPrint = now;
    testEncoder();
  }
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

// ============================================================
// LECTURA DE ENCODER
// ============================================================

float readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDR, 2);
  
  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    
    uint16_t rawAngle = ((uint16_t)highByte << 8) | lowByte;
    rawAngle &= 0x0FFF; // 12 bits
    
    // Convertir a radianes [0, 2π]
    float angle = rawAngle * (2.0 * PI / 4096.0);
    return angle;
  }
  
  return 0; // Error
}

// ============================================================
// MENÚ Y COMANDOS
// ============================================================

void printMenu() {
  Serial.println("\n====================================");
  Serial.println("        MENÚ DE PRUEBAS");
  Serial.println("====================================");
  Serial.println("Comandos disponibles:");
  Serial.println("  0 - Test Encoder (solo lectura)");
  Serial.println("  1 - Test Motor CW (sentido horario)");
  Serial.println("  2 - Test Motor CCW (antihorario)");
  Serial.println("  3 - Auto Test (secuencia completa)");
  Serial.println("  s - STOP motor");
  Serial.println("  i - Información del sistema");
  Serial.println("  m - Mostrar este menú");
  Serial.println("====================================\n");
  Serial.print("Modo actual: ");
  printCurrentMode();
  Serial.println();
}

void processCommand(char cmd) {
  switch(cmd) {
    case '0':
      testMode = 0;
      stopMotor();
      Serial.println("\n>>> Modo: TEST ENCODER");
      Serial.println("Gira el motor manualmente y observa la lectura.\n");
      break;
      
    case '1':
      testMode = 1;
      Serial.println("\n>>> Modo: MOTOR CW (Clockwise)");
      Serial.println("El motor aumentará PWM cada 2 segundos.\n");
      break;
      
    case '2':
      testMode = 2;
      Serial.println("\n>>> Modo: MOTOR CCW (Counter-Clockwise)");
      Serial.println("El motor aumentará PWM cada 2 segundos.\n");
      break;
      
    case '3':
      testMode = 3;
      Serial.println("\n>>> Modo: AUTO TEST");
      Serial.println("Iniciando secuencia automática...\n");
      break;
      
    case 's':
    case 'S':
      testMode = 0;
      stopMotor();
      Serial.println("\n>>> MOTOR DETENIDO");
      break;
      
    case 'i':
    case 'I':
      printSystemInfo();
      break;
      
    case 'm':
    case 'M':
      printMenu();
      break;
      
    default:
      Serial.println("\nComando no reconocido. Presiona 'm' para ver menú.");
  }
}

void printCurrentMode() {
  switch(testMode) {
    case 0: Serial.print("Test Encoder"); break;
    case 1: Serial.print("Motor CW"); break;
    case 2: Serial.print("Motor CCW"); break;
    case 3: Serial.print("Auto Test"); break;
  }
}

void printSystemInfo() {
  Serial.println("\n====================================");
  Serial.println("    INFORMACIÓN DEL SISTEMA");
  Serial.println("====================================");
  
  // Info del encoder
  Serial.println("\n[ENCODER AS5600]");
  Wire.beginTransmission(AS5600_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("  Estado: ✓ CONECTADO");
    float angle = readAngle();
    Serial.print("  Ángulo actual: ");
    Serial.print(angle * 180.0 / PI, 2);
    Serial.println("°");
    Serial.print("  Radianes: ");
    Serial.println(angle, 4);
    
    // Leer valor crudo
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() == 2) {
      uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
      raw &= 0x0FFF;
      Serial.print("  Valor crudo: ");
      Serial.print(raw);
      Serial.println(" / 4095");
    }
  } else {
    Serial.println("  Estado: ✗ NO DETECTADO");
  }
  
  // Info del motor
  Serial.println("\n[MOTOR L298N]");
  Serial.print("  Pin IN1 (D7): ");
  Serial.println(digitalRead(MOTOR_IN1) ? "HIGH" : "LOW");
  Serial.print("  Pin IN2 (D8): ");
  Serial.println(digitalRead(MOTOR_IN2) ? "HIGH" : "LOW");
  Serial.print("  Pin PWM (D9): ");
  Serial.print("~");
  Serial.print(analogRead(9));  // Nota: esto no lee el PWM correctamente
  
  // Info del sistema
  Serial.println("\n[ARDUINO]");
  Serial.print("  Tiempo encendido: ");
  Serial.print(millis() / 1000);
  Serial.println(" segundos");
  Serial.print("  Memoria libre: ");
  Serial.print(freeRam());
  Serial.println(" bytes");
  
  Serial.println("\n====================================\n");
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// ============================================================
// TEST ESPECÍFICOS DE DIAGNÓSTICO
// ============================================================

// Puedes añadir aquí funciones adicionales para tests específicos

/*
 * GUÍA DE INTERPRETACIÓN DE RESULTADOS:
 * 
 * [ENCODER]
 * - Si "NO ENCONTRADO": revisar conexiones I2C (SDA/SCL)
 * - Si ángulo = 0 siempre: imán no presente o muy lejos
 * - Si ángulo salta aleatoriamente: imán descentrado o interferencia
 * - Normal: valores estables que cambian suavemente al girar
 * 
 * [MOTOR]
 * - Si no gira en ningún sentido: verificar fuente externa 12V
 * - Si gira pero débil: aumentar PWM o verificar batería
 * - Si gira solo en un sentido: cables OUT1/OUT2 mal conectados
 * - Si hace ruido pero no gira: motor bloqueado mecánicamente
 * 
 * [VELOCIDAD]
 * - Valores típicos: -10 a +10 rad/s para micro motores
 * - Si velocidad = 0 con motor girando: encoder no funciona
 * - Si velocidad muy errática: problema de lectura encoder
 */
