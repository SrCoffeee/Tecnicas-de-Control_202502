/*
 * IDENTIFICACIÓN PARÁMETROS - AEROBALANCÍN
 * Comandos: START, DATA
 */

#include <Wire.h>

#define ENA 9
#define IN1 7
#define IN2 8
#define AS5600 0x36
#define ANG_H 0x0C
#define ANG_L 0x0D

const byte Ts = 10;           // ms
const int N = 500;            // muestras
const byte PWM = 30;          // escalón %
const byte T_ESC = 100;       // duración escalón
const int T_DEC = 400;        // duración decaimiento

byte fase = 0;  // 0:espera, 1:escalón, 2:decay, 3:listo
float ang0 = 0;
float datos[N];
unsigned int t[N];
int idx = 0;
unsigned long t0 = 0;
bool listo = false;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  Serial.begin(115200);
  Wire.begin();
  analogWrite(ENA, 0);
  
  Serial.println(F("IDENT - START/DATA"));
  delay(2000);
  
  // Calibrar
  float s = 0;
  for (byte i = 0; i < 50; i++) {
    s += leerAng();
    delay(20);
  }
  ang0 = s / 50.0;
  Serial.print(F("Ang0: "));
  Serial.println(ang0, 1);
}

void loop() {
  // Comandos
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n');
    c.trim();
    
    if (c == "START") {
      idx = 0;
      listo = false;
      fase = 1;
      t0 = millis();
      Serial.println(F("Iniciado"));
    } else if (c == "DATA" && listo) {
      enviar();
    }
  }
  
  // Ejecutar
  static unsigned long tp = 0;
  unsigned long ta = millis();
  
  if (fase > 0 && fase < 3) {
    if (ta - tp >= Ts) {
      tp = ta;
      
      float a = leerAngRel();
      
      if (idx < N) {
        datos[idx] = a;
        t[idx] = (unsigned int)(ta - t0);
        idx++;
      }
      
      unsigned long tf = ta - t0;
      
      if (fase == 1) {
        analogWrite(ENA, (PWM * 255) / 100);
        if (tf >= T_ESC * (unsigned long)Ts) {
          fase = 2;
          analogWrite(ENA, 0);
          Serial.println(F("Decay"));
        }
      } else if (fase == 2) {
        if (tf >= (T_ESC + T_DEC) * (unsigned long)Ts) {
          fase = 3;
          listo = true;
          Serial.println(F("Listo->DATA"));
        }
      }
    }
  }
}

float leerAng() {
  Wire.beginTransmission(AS5600);
  Wire.write(ANG_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600, 2);
  
  if (Wire.available() >= 2) {
    int h = Wire.read();
    int l = Wire.read();
    int raw = ((h << 8) | l) & 0x0FFF;
    return (raw * 360.0) / 4096.0;
  }
  return 0;
}

float leerAngRel() {
  float a = leerAng();
  float d = a - ang0;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;
}

void enviar() {
  Serial.println(F("DATA_START"));
  for (int i = 0; i < idx; i++) {
    Serial.print(t[i]);
    Serial.print(',');
    Serial.println(datos[i], 3);
  }
  Serial.println(F("DATA_END"));
}
