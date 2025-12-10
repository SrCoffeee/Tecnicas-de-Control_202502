/*
 * TEST DE HARDWARE BÁSICO - VERSIÓN OPTIMIZADA
 * Reducido uso de memoria RAM usando F() macro
 */

#include <Wire.h>

// Pines L298N
#define MOTOR_IN1    7
#define MOTOR_IN2    8
#define MOTOR_PWM    9

// AS5600
#define AS5600_ADDR  0x36
#define AS5600_RAW_H 0x0C

// Variables globales (mínimas)
byte testMode = 0;
unsigned long lastPrint = 0;
float lastAngle = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== TEST HARDWARE ==="));
  
  // Configurar pines
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  stopMotor();
  
  // Inicializar I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Test AS5600
  Serial.print(F("AS5600: "));
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println(F("OK"));
    Serial.print(F("Angulo: "));
    Serial.println(readAngle() * 57.2958, 1);
  } else {
    Serial.println(F("NO DETECTADO"));
    Serial.println(F("Revisa SDA=A4, SCL=A5"));
  }
  
  printMenu();
}

void loop() {
  // Comandos seriales
  if (Serial.available()) {
    processCommand(Serial.read());
  }
  
  unsigned long now = millis();
  
  switch(testMode) {
    case 0: // Encoder
      if (now - lastPrint >= 100) {
        lastPrint = now;
        printEncoder();
      }
      break;
      
    case 1: // Motor CW
    case 2: // Motor CCW
      runMotorTest(now);
      break;
      
    case 3: // Auto test
      autoTest(now);
      break;
  }
}

// ============ FUNCIONES ============

float readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  
  if (Wire.available() == 2) {
    uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
    raw &= 0x0FFF;
    return raw * 0.00153398;  // 2*PI/4096
  }
  return 0;
}

void printEncoder() {
  float angle = readAngle();
  float vel = (angle - lastAngle) * 10;  // aprox rad/s
  
  // Manejar cruce 0-360
  if (vel > 31.4) vel -= 62.83;
  if (vel < -31.4) vel += 62.83;
  
  Serial. print(angle * 57.3, 1);
  Serial.print(F("deg "));
  Serial.print(vel, 1);
  Serial.println(F("r/s"));
  
  lastAngle = angle;
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

void setMotor(bool cw, byte pwm) {
  digitalWrite(MOTOR_IN1, cw ?  HIGH : LOW);
  digitalWrite(MOTOR_IN2, cw ? LOW : HIGH);
  analogWrite(MOTOR_PWM, pwm);
}

void runMotorTest(unsigned long now) {
  static byte pwm = 0;
  static unsigned long lastChange = 0;
  
  if (now - lastChange >= 2000) {
    lastChange = now;
    pwm += 64;
    if (pwm > 250) pwm = 64;
    
    Serial.print(testMode == 1 ? F("CW ") : F("CCW "));
    Serial.print(pwm);
    Serial.println(F(" PWM"));
    
    setMotor(testMode == 1, pwm);
  }
  
  if (now - lastPrint >= 200) {
    lastPrint = now;
    printEncoder();
  }
}

void autoTest(unsigned long now) {
  static byte phase = 0;
  static unsigned long phaseStart = 0;
  
  if (phase == 0) {
    Serial.println(F("Auto:  CW"));
    setMotor(true, 150);
    phaseStart = now;
    phase = 1;
  }
  else if (phase == 1 && now - phaseStart >= 3000) {
    Serial.println(F("Auto:  STOP"));
    stopMotor();
    phaseStart = now;
    phase = 2;
  }
  else if (phase == 2 && now - phaseStart >= 2000) {
    Serial.println(F("Auto: CCW"));
    setMotor(false, 150);
    phaseStart = now;
    phase = 3;
  }
  else if (phase == 3 && now - phaseStart >= 3000) {
    stopMotor();
    Serial.println(F("Auto: FIN"));
    testMode = 0;
    phase = 0;
    printMenu();
  }
  
  if (now - lastPrint >= 200) {
    lastPrint = now;
    printEncoder();
  }
}

void printMenu() {
  Serial. println(F("\n--- MENU ---"));
  Serial.println(F("0: Encoder 1:CW 2:CCW"));
  Serial.println(F("3:Auto s:Stop m:Menu"));
}

void processCommand(char cmd) {
  switch(cmd) {
    case '0': 
      testMode = 0;
      stopMotor();
      Serial.println(F("\nModo: Encoder"));
      break;
      
    case '1':
      testMode = 1;
      Serial.println(F("\nModo: Motor CW"));
      break;
      
    case '2':
      testMode = 2;
      Serial.println(F("\nModo: Motor CCW"));
      break;
      
    case '3':
      testMode = 3;
      Serial. println(F("\nModo: Auto Test"));
      break;
      
    case 's':
    case 'S':
      testMode = 0;
      stopMotor();
      Serial.println(F("\nSTOP"));
      break;
      
    case 'i':
    case 'I':
      printInfo();
      break;
      
    case 'm':
    case 'M': 
      printMenu();
      break;
  }
}

void printInfo() {
  Serial.println(F("\n--- INFO ---"));
  
  // Encoder
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.print(F("AS5600: "));
    Serial.print(readAngle() * 57.3, 1);
    Serial.println(F(" deg"));
  } else {
    Serial. println(F("AS5600: Error"));
  }
  
  // Motor pins
  Serial.print(F("IN1:"));
  Serial.print(digitalRead(MOTOR_IN1));
  Serial.print(F(" IN2:"));
  Serial.println(digitalRead(MOTOR_IN2));
  
  // RAM libre
  extern int __heap_start, *__brkval;
  int v;
  int ram = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  Serial.print(F("RAM: "));
  Serial.println(ram);
}