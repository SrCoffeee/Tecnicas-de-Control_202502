# GUÍA COMPLETA DE USO - SISTEMA DE CONTROL AVANZADO

## 📚 Tabla de Contenidos

1. [Instalación Inicial](#instalación-inicial)
2. [Sistema de Control Principal](#sistema-de-control-principal)
3. [Sistema de Identificación](#sistema-de-identificación)
4. [Sincronización Arduino-MATLAB](#sincronización-arduino-matlab)
5. [Análisis de Resultados](#análisis-de-resultados)
6. [Troubleshooting](#troubleshooting)

---

## 🔧 Instalación Inicial

### Hardware Necesario

- **Arduino Uno/Mega**
- **L298N Motor Driver**
- **AS5600** (sensor magnético de ángulo)
- **MPU6050** (IMU - acelerómetro + giroscopio)
- **Motor DC** con reductor
- **Fuente de alimentación** 12V, 2A

### Conexiones

```
L298N → Arduino:
  ENA → Pin 9 (PWM)
  IN1 → Pin 7
  IN2 → Pin 8
  GND → GND

AS5600 → Arduino:
  SDA → A4 (I2C)
  SCL → A5 (I2C)
  VCC → 5V
  GND → GND

MPU6050 → Arduino:
  SDA → A4 (I2C) [compartido con AS5600]
  SCL → A5 (I2C) [compartido con AS5600]
  VCC → 5V
  GND → GND
```

### Software Necesario

**Arduino IDE:**
```
1. Descargar de: https://www.arduino.cc/en/software
2. Instalar librerías:
   - Wire (incluida)
   - MPU6050 by Electronic Cats
```

**MATLAB:**
```
Versión: R2020a o superior
Toolboxes requeridos:
  - Control System Toolbox
  - Signal Processing Toolbox
  - Instrument Control Toolbox (para serial)
```

**Python (alternativo):**
```
pip install pyserial numpy matplotlib scipy pandas
```

---

## 🎮 Sistema de Control Principal

### PASO 1: Cargar Código en Arduino

1. Abrir `Arduino_Controladores_Completo.ino`
2. Verificar puerto COM:
   - Windows: Herramientas > Puerto > COMx
   - Linux/Mac: Herramientas > Puerto > /dev/ttyUSBx
3. Cargar código (Ctrl+U)

### PASO 2: Verificar Comunicación Serial

Abrir Monitor Serial (Ctrl+Shift+M):
- Baudios: **115200**
- Line ending: **Nueva línea**

Debería aparecer:
```
=== SISTEMA LISTO ===
MPU6050_OK
SYSTEM_READY
```

### PASO 3: Ejecutar Interfaz MATLAB

```matlab
% En MATLAB Command Window:
cd 'ruta/a/archivos'
AerobalancinGUI
```

### PASO 4: Conectar Arduino

1. En la GUI, ingresar puerto COM (ej: COM3)
2. Click en **CONECTAR**
3. Estado cambia a: ● CONECTADO (verde)

### PASO 5: Configurar Experimento

**Seleccionar Tipo de Referencia:**
- 📊 **Escalón**: Referencia constante
- 📈 **Rampa**: Incremento lineal
- ⚡ **Impulsos**: Tren de pulsos cuadrados

**Configurar Setpoint:**
1. Ingresar valor (ej: 45°)
2. Click "Aplicar"

### PASO 6: Iniciar Control

Click en el botón del controlador deseado:
- **PID** → Control clásico
- **H∞** → Control robusto
- **SMC** → Modo deslizante
- **ADAPT** → Control adaptativo

### PASO 7: Monitorear en Tiempo Real

La GUI muestra **6 gráficas**:

1. **Respuesta Angular** (arriba izquierda)
   - Roja (--): Setpoint
   - Azul (—): Ángulo medido
   
2. **Error de Seguimiento** (arriba derecha)
   - Roja (—): Error = Setpoint - Ángulo
   
3. **Esfuerzo de Control** (centro izquierda)
   - Verde (—): PWM [%]
   
4. **Velocidad Angular** (centro derecha)
   - Magenta (—): ω del giroscopio
   
5. **Parámetros Adaptativos** (abajo izquierda)
   - Roja (—): θ₁ (ganancia proporcional)
   - Azul (—): θ₂ (ganancia derivativa)
   - Verde (—): θ₃ (ganancia integral)
   
6. **Métricas** (abajo derecha)
   - Error RMS
   - Error máximo
   - Error estado estacionario
   - Tiempo de establecimiento
   - PWM promedio

### PASO 8: Detener Control

Click en **⏹ DETENER**

### PASO 9: Guardar Datos

1. Click en **💾 Guardar**
2. Guardar como: `datos_experimento.mat`

### PASO 10: Exportar Gráficas

1. Click en **📊 Export**
2. Guardar como: `graficas_resultado.png`

---

## 🔍 Sistema de Identificación

### Objetivo

Determinar los parámetros físicos del sistema:
- **ωn**: Frecuencia natural [rad/s]
- **ζ**: Factor de amortiguamiento
- **B**: Coeficiente de fricción [N·m·s]

### PASO 1: Cargar Código de Identificación

1. Abrir `Arduino_Identificacion.ino`
2. Cargar en Arduino

### PASO 2: Ejecutar Script MATLAB

```matlab
identificacionParametros
```

### PASO 3: Seguir el Proceso

**El script preguntará:**

```
Puerto COM (ej: COM3): COM3
```

**Luego:**
```
Iniciando identificación...
Fase: Escalón
Fase: Decaimiento libre
Identificación completa
Obteniendo datos...
Recibiendo datos...
```

### PASO 4: Ingresar Parámetros Conocidos

```
Inercia J [kg·m²] (Enter para 0.0625): 0.0625
```

### PASO 5: Analizar Resultados

El script muestra:

```
=== RESULTADOS DE IDENTIFICACIÓN ===

--- Método del Decremento Logarítmico ---
Factor de amortiguamiento (ζ): 0.0352
Frecuencia natural (ωn): 3.426 rad/s

--- Método de Ajuste de Curva ---
Coeficiente de decaimiento: 0.121
Factor de amortiguamiento (ζ): 0.0353

--- Análisis FFT ---
Frecuencia dominante: 0.545 Hz
Frecuencia angular (ωd): 3.423 rad/s

=== PARÁMETROS FÍSICOS ===
--- Resultados Finales ---
Inercia (J): 0.0625 kg·m²
Fricción viscosa (B): 0.01503 N·m·s
Constante de tiempo (τ): 8.283 s
```

### PASO 6: Visualizar Gráficas

Se genera una figura con:
1. Respuesta completa
2. Decaimiento libre con envolvente
3. Análisis FFT
4. Diagrama de polos

### PASO 7: Guardar Resultados

```
¿Guardar resultados? (s/n): s
```

Guarda:
- `parametros_identificados.mat`
- `identificacion.png`
- `reporte_identificacion.txt`

### PASO 8: Usar Parámetros

Copiar valores al archivo principal:

```cpp
// En Arduino_Controladores_Completo.ino
const float omega_ref = 3.426;  // Actualizar con ωn identificado
const float zeta_ref = 0.0352;  // Actualizar con ζ identificado
float B = 0.01503;              // Actualizar con B identificado
```

---

## 🔗 Sincronización Arduino-MATLAB

### Protocolo de Comunicación

**Arduino → MATLAB:**

```
Formato de datos:
DATA,tiempo,setpoint,angulo,error,pwm,gyro_x,theta1,theta2,theta3

Ejemplo:
DATA,1250,45.000,43.567,1.433,26.450,12.345,2.1456,0.5234,1.0987

Mensajes de estado:
MSG,STARTED:PID
MSG,STOPPED
MSG,CALIBRATED
```

**MATLAB → Arduino:**

```
Comandos:
START:1     → Iniciar PID
START:2     → Iniciar H∞
START:3     → Iniciar SMC
START:4     → Iniciar Adaptativo
STOP        → Detener control
SET:45.5    → Cambiar setpoint a 45.5°
REF:0       → Referencia escalón
REF:1       → Referencia rampa
REF:2       → Referencia impulsos
IDENT       → Modo identificación
CALIB       → Calibrar sensores
KP:2.5      → Cambiar Kp del PID
KI:3.0      → Cambiar Ki del PID
KD:0.8      → Cambiar Kd del PID
```

### Ejemplo Manual (sin GUI)

```matlab
% Conectar
s = serialport('COM3', 115200);
configureTerminator(s, "LF");

% Iniciar PID
writeline(s, 'START:1');
pause(1);

% Cambiar setpoint
writeline(s, 'SET:60');
pause(0.5);

% Leer datos
for i = 1:100
    if s.NumBytesAvailable > 0
        linea = readline(s);
        disp(linea);
    end
    pause(0.02);
end

% Detener
writeline(s, 'STOP');

% Cerrar
delete(s);
```

---

## 📊 Análisis de Resultados

### Comparar Controladores

```matlab
% Cargar datos guardados
load('datos_pid.mat');
pid_data = data;

load('datos_hinf.mat');
hinf_data = data;

load('datos_smc.mat');
smc_data = data;

% Crear figura comparativa
figure('Position', [100 100 1200 800]);

% Respuesta angular
subplot(2,2,1);
plot(pid_data.tiempo, pid_data.angulo, 'b-', 'LineWidth', 1.5);
hold on;
plot(hinf_data.tiempo, hinf_data.angulo, 'r-', 'LineWidth', 1.5);
plot(smc_data.tiempo, smc_data.angulo, 'g-', 'LineWidth', 1.5);
plot(pid_data.tiempo, pid_data.setpoint, 'k--', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
title('Comparación de Respuesta Angular');
legend('PID', 'H∞', 'SMC', 'Setpoint', 'Location', 'best');

% Error
subplot(2,2,2);
plot(pid_data.tiempo, pid_data.error, 'b-', 'LineWidth', 1.5);
hold on;
plot(hinf_data.tiempo, hinf_data.error, 'r-', 'LineWidth', 1.5);
plot(smc_data.tiempo, smc_data.error, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Error [°]');
title('Comparación de Error');
legend('PID', 'H∞', 'SMC', 'Location', 'best');

% PWM
subplot(2,2,3);
plot(pid_data.tiempo, pid_data.pwm, 'b-', 'LineWidth', 1.5);
hold on;
plot(hinf_data.tiempo, hinf_data.pwm, 'r-', 'LineWidth', 1.5);
plot(smc_data.tiempo, smc_data.pwm, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('PWM [%]');
title('Comparación de Esfuerzo de Control');
legend('PID', 'H∞', 'SMC', 'Location', 'best');

% Métricas
subplot(2,2,4);
metricas = [
    rms(pid_data.error), rms(hinf_data.error), rms(smc_data.error);
    mean(pid_data.pwm), mean(hinf_data.pwm), mean(smc_data.pwm)
];

bar(metricas');
grid on;
xlabel('Controlador');
ylabel('Valor');
title('Métricas Comparativas');
legend('Error RMS [°]', 'PWM Promedio [%]', 'Location', 'best');
set(gca, 'XTickLabel', {'PID', 'H∞', 'SMC'});
```

### Calcular Índices de Desempeño

```matlab
function metricas = calcularMetricas(data)
    % ISE: Integral Square Error
    metricas.ISE = trapz(data.tiempo, data.error.^2);
    
    % IAE: Integral Absolute Error
    metricas.IAE = trapz(data.tiempo, abs(data.error));
    
    % ITAE: Integral Time Absolute Error
    metricas.ITAE = trapz(data.tiempo, data.tiempo .* abs(data.error));
    
    % Error RMS
    metricas.ERMS = sqrt(mean(data.error.^2));
    
    % Tiempo de establecimiento (2%)
    tol = 0.02 * abs(data.setpoint(end));
    idx = find(abs(data.error) <= tol, 1, 'first');
    if ~isempty(idx)
        metricas.ts = data.tiempo(idx);
    else
        metricas.ts = inf;
    end
    
    % Sobrepico
    [pico, idx_pico] = max(data.angulo);
    if pico > data.setpoint(end)
        metricas.Mp = ((pico - data.setpoint(end)) / data.setpoint(end)) * 100;
    else
        metricas.Mp = 0;
    end
    
    % Esfuerzo de control
    metricas.PWM_mean = mean(data.pwm);
    metricas.PWM_std = std(data.pwm);
    metricas.TV = sum(abs(diff(data.pwm)));  % Total Variation
end
```

---

## 🛠️ Troubleshooting

### Problema 1: Arduino no se conecta

**Síntomas:**
```
Error: No se pudo conectar al Arduino
```

**Soluciones:**
1. Verificar cable USB
2. Comprobar puerto COM en Administrador de Dispositivos (Windows)
3. Reinstalar drivers CH340/FTDI
4. Probar con otro puerto USB
5. Cerrar otras aplicaciones que usen el puerto (Monitor Serial)

### Problema 2: Datos no se reciben en MATLAB

**Síntomas:**
- Gráficas no se actualizan
- No aparecen mensajes en consola

**Soluciones:**
1. Verificar baudios: **115200** en ambos lados
2. Verificar terminador de línea: `configureTerminator(s, "LF")`
3. Limpiar buffer: `flush(arduino)`
4. Aumentar timeout
5. Revisar cable USB (puede tener fallas intermitentes)

### Problema 3: Motor no responde

**Síntomas:**
- PWM se muestra pero motor no gira
- Motor gira solo a un lado

**Soluciones:**
1. Verificar alimentación L298N (12V, 2A mínimo)
2. Comprobar conexiones ENA, IN1, IN2
3. Medir voltaje en bornes del motor
4. Verificar direcciones:
   ```cpp
   digitalWrite(IN1, LOW);   // Cambiar si necesario
   digitalWrite(IN2, HIGH);
   ```
5. Probar con PWM manual:
   ```cpp
   analogWrite(ENA, 128);  // 50%
   ```

### Problema 4: Sensor AS5600 no lee

**Síntomas:**
```
MSG,AS5600_ERROR
```

**Soluciones:**
1. Verificar conexión I2C (SDA=A4, SCL=A5)
2. Usar I2C Scanner:
   ```cpp
   #include <Wire.h>
   void setup() {
     Wire.begin();
     Serial.begin(9600);
     Serial.println("I2C Scanner");
   }
   void loop() {
     for(byte address = 1; address < 127; address++) {
       Wire.beginTransmission(address);
       if(Wire.endTransmission() == 0) {
         Serial.print("Dispositivo en 0x");
         Serial.println(address, HEX);
       }
     }
     delay(5000);
   }
   ```
3. Verificar imán: distancia 2-3mm, polo correcto
4. Comprobar voltaje (5V)

### Problema 5: MPU6050 da lecturas erráticas

**Síntomas:**
- Velocidad angular oscila violentamente
- Acelerómetro con ruido

**Soluciones:**
1. Recalibrar:
   ```
   Enviar comando: CALIB
   ```
2. Ajustar offsets en código:
   ```cpp
   mpu.setXAccelOffset(-1343);  // Modificar estos valores
   mpu.setYAccelOffset(-1155);
   mpu.setZAccelOffset(1165);
   ```
3. Aumentar filtro complementario:
   ```cpp
   float alpha_filter = 0.99;  // Más suave (era 0.98)
   ```
4. Verificar vibraciones mecánicas

### Problema 6: Control oscila mucho

**Síntomas:**
- Sistema no se estabiliza
- Oscilaciones persistentes

**Soluciones PID:**
```matlab
% En MATLAB, enviar comandos:
writeline(arduino, 'KP:1.5');  % Reducir Kp
writeline(arduino, 'KD:0.4');  % Reducir Kd
```

**Soluciones SMC:**
```cpp
// En código Arduino:
float phi_smc = 0.05;  // Aumentar capa límite (era 0.02)
float k_smc = 0.20;    // Reducir ganancia (era 0.27)
```

**Soluciones Adaptativo:**
```cpp
// Reducir ganancias de adaptación:
float gamma1 = 0.3;  // Era 0.5
float gamma2 = 0.2;  // Era 0.3
float gamma3 = 0.05; // Era 0.1
```

### Problema 7: Identificación da resultados incorrectos

**Síntomas:**
- ζ negativo o > 1
- ωn muy alto/bajo

**Soluciones:**
1. Verificar que sistema esté en reposo al iniciar
2. Aumentar PWM de escalón:
   ```cpp
   const float PWM_ESCALON = 35.0;  // Era 30.0
   ```
3. Aumentar número de muestras:
   ```cpp
   const int NUM_MUESTRAS = 1000;  // Era 500
   ```
4. Verificar fricción no sea muy alta (añadir lubricante)
5. Repetir identificación 3 veces y promediar

### Problema 8: MATLAB cierra inesperadamente

**Síntomas:**
- GUI se cierra sin aviso
- Error de memoria

**Soluciones:**
1. Limpiar workspace antes de ejecutar:
   ```matlab
   clear all; close all; clc;
   ```
2. Reducir frecuencia de actualización:
   ```matlab
   % En leerDatosArduino:
   if mod(length(tiempoData), 20) == 0  % Era 10
       actualizarGraficas();
   end
   ```
3. Aumentar memoria de MATLAB:
   ```
   Preferencias > General > Java Heap Memory > 2048 MB
   ```

---

## 📖 Recursos Adicionales

### Comandos Útiles MATLAB

```matlab
% Ver puertos disponibles
serialportlist("available")

% Limpiar puertos ocupados
delete(instrfindall)

% Ver variables cargadas
whos

% Limpiar todo
clear; close all; clc;
```

### Atajos de Teclado GUI

- **F5**: Iniciar control rápido (PID)
- **Esc**: Detener control
- **Ctrl+S**: Guardar datos
- **Ctrl+E**: Exportar gráficas
- **Ctrl+L**: Limpiar gráficas

### Scripts de Ejemplo

**Prueba automática de todos los controladores:**
```matlab
controladores = {'PID', 'HINF', 'SMC', 'ADAPTIVE'};

for i = 1:4
    fprintf('Probando %s...\n', controladores{i});
    
    % Iniciar
    writeline(arduino, sprintf('START:%d', i));
    pause(30);  % 30 segundos
    
    % Detener
    writeline(arduino, 'STOP');
    pause(5);
    
    % Guardar
    datos = obtenerDatos();
    save(sprintf('datos_%s.mat', lower(controladores{i})), 'datos');
end
```

---

**¡Sistema listo para usar! 🚀**

Para soporte adicional, consultar:
- `DOCUMENTO_TECNICO.md` - Fundamentos matemáticos
- `README.md` - Visión general del proyecto
- Comunidad Arduino: https://forum.arduino.cc/

