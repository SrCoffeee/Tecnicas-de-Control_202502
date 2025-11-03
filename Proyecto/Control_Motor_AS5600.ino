/*
 * Control de Motor F1607 con L298N y lectura de ángulo con AS5600
 * 
 * Componentes:
 * - Arduino (Uno, Nano, Mega, etc.)
 * - Driver L298N
 * - Sensor AS5600 (magnético de ángulo)
 * - Micro motor F1607
 * 
 * Conexiones:
 * 
 * === L298N al Arduino ===
 * ENA  -> Pin 9 (PWM)
 * IN1  -> Pin 7
 * IN2  -> Pin 8
 * GND  -> GND Arduino
 * +12V -> Fuente externa 12V
 * +5V  -> No conectar (usa el regulador interno del L298N si lo necesitas)
 * 
 * === L298N al Motor F1607 ===
 * OUT1 -> Terminal positiva del motor
 * OUT2 -> Terminal negativa del motor
 * 
 * === AS5600 al Arduino ===
 * VCC  -> 5V Arduino
 * GND  -> GND Arduino
 * SDA  -> A4 (Arduino Uno/Nano) o Pin 20 (Mega)
 * SCL  -> A5 (Arduino Uno/Nano) o Pin 21 (Mega)
 * 
 * === Notas importantes ===
 * - El AS5600 requiere un imán diametral colocado sobre el eje del motor
 * - La distancia entre el sensor y el imán debe ser de 0.5-3mm
 * - Asegúrate de conectar una fuente externa para el L298N (6-12V)
 * - El jumper ENA debe estar removido para control PWM
 */

#include <Wire.h>

// Pines del L298N
const int ENA = 9;   // PWM para velocidad
const int IN1 = 7;   // Control dirección
const int IN2 = 8;   // Control dirección

// Dirección I2C del AS5600
const int AS5600_ADDRESS = 0x36;

// Registros del AS5600
const int ANGLE_HIGH = 0x0E;  // Registro alto del ángulo
const int ANGLE_LOW = 0x0F;   // Registro bajo del ángulo

// Variables globales
int velocidad = 0;
int direccion = 1; // 1 = adelante, -1 = atrás
float angulo = 0.0;

void setup() {
  // Inicializar comunicación serial
  Serial.begin(9600);
  
  // Configurar pines del L298N
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Inicializar I2C para AS5600
  Wire.begin();
  
  // Estado inicial: motor detenido
  detenerMotor();
  
  Serial.println("=== Sistema Iniciado ===");
  Serial.println("Comandos disponibles:");
  Serial.println("+ : Aumentar velocidad");
  Serial.println("- : Disminuir velocidad");
  Serial.println("d : Cambiar dirección");
  Serial.println("s : Detener motor");
  Serial.println("1-9: Velocidad directa (10%-90%)");
  Serial.println("=======================");
}

void loop() {
  // Leer ángulo del AS5600
  angulo = leerAngulo();
  
  // Mostrar información cada 500ms
  static unsigned long ultimoTiempo = 0;
  if (millis() - ultimoTiempo >= 500) {
    mostrarEstado();
    ultimoTiempo = millis();
  }
  
  // Procesar comandos del puerto serial
  if (Serial.available() > 0) {
    char comando = Serial.read();
    procesarComando(comando);
  }
}

// Función para leer el ángulo del AS5600
float leerAngulo() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(ANGLE_HIGH);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() == 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    
    // Combinar bytes (resolución de 12 bits)
    int rawAngle = (highByte << 8) | lowByte;
    
    // Convertir a grados (0-360)
    float angle = rawAngle * 0.087890625; // 360/4096
    
    return angle;
  }
  
  return -1; // Error en lectura
}

// Función para procesar comandos
void procesarComando(char cmd) {
  switch (cmd) {
    case '+':
      velocidad += 25;
      if (velocidad > 255) velocidad = 255;
      aplicarVelocidad();
      Serial.print("Velocidad aumentada: ");
      Serial.println(velocidad);
      break;
      
    case '-':
      velocidad -= 25;
      if (velocidad < 0) velocidad = 0;
      aplicarVelocidad();
      Serial.print("Velocidad reducida: ");
      Serial.println(velocidad);
      break;
      
    case 'd':
      direccion *= -1;
      aplicarVelocidad();
      Serial.print("Dirección cambiada: ");
      Serial.println(direccion == 1 ? "Adelante" : "Atrás");
      break;
      
    case 's':
      velocidad = 0;
      detenerMotor();
      Serial.println("Motor detenido");
      break;
      
    case '1': case '2': case '3': case '4': case '5':
    case '6': case '7': case '8': case '9':
      int nivel = cmd - '0';
      velocidad = map(nivel, 1, 9, 28, 255); // 10% a 100%
      aplicarVelocidad();
      Serial.print("Velocidad establecida a nivel ");
      Serial.println(nivel);
      break;
  }
}

// Función para aplicar velocidad y dirección
void aplicarVelocidad() {
  if (velocidad == 0) {
    detenerMotor();
    return;
  }
  
  analogWrite(ENA, velocidad);
  
  if (direccion == 1) {
    // Adelante
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    // Atrás
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

// Función para detener el motor
void detenerMotor() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// Función para mostrar el estado actual
void mostrarEstado() {
  Serial.println("------------------------");
  Serial.print("Ángulo: ");
  Serial.print(angulo, 2);
  Serial.println("°");
  
  Serial.print("Velocidad: ");
  Serial.print(velocidad);
  Serial.print(" (");
  Serial.print(map(velocidad, 0, 255, 0, 100));
  Serial.println("%)");
  
  Serial.print("Dirección: ");
  Serial.println(direccion == 1 ? "Adelante" : "Atrás");
  Serial.println("------------------------");
}
