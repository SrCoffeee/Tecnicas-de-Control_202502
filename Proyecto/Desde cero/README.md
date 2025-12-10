# Controladores PID, H∞, SMC y APID para Brazo-Motor con Encoder AS5600

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## 📋 Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Modelo Dinámico](#modelo-dinámico)
- [Controladores Implementados](#controladores-implementados)
  - [PID Clásico](#1-control-pid-clásico)
  - [H∞ (H-Infinity)](#2-control-h-infinity)
  - [SMC (Sliding Mode Control)](#3-control-por-modo-deslizante-smc)
  - [APID (Adaptive PID)](#4-control-adaptativo-apid)
- [Resultados Experimentales](#resultados-experimentales)
- [Análisis Comparativo](#análisis-comparativo)
- [Instalación y Uso](#instalación-y-uso)
- [Estructura del Repositorio](#estructura-del-repositorio)
- [Sugerencias para Presentación](#sugerencias-para-presentación)
- [Referencias](#referencias)

---

## 🎯 Descripción General

Este proyecto presenta el **diseño, implementación y comparación** de cuatro estrategias de control para un sistema brazo-motor:

| Controlador | Características | Aplicación |
|-------------|-----------------|------------|
| **PID** | Clásico, fácil sintonización | Referencias lentas, sistemas lineales |
| **H∞** | Robusto, optimiza sensibilidad | Rechazo de perturbaciones, robustez |
| **SMC** | No lineal, insensible a parámetros | Sistemas variables, alta robustez |
| **APID** | Adaptativo con RLS | Plantas con cambios de parámetros |

### 🎓 Objetivos del Proyecto

1. **Comparar desempeño** de controladores lineales (PID, H∞) vs no lineales (SMC, APID)
2. **Evaluar robustez** ante cambios paramétricos de la planta
3. **Demostrar adaptabilidad** del APID mediante estimación en línea (RLS)
4. **Validar** en perfiles diversos: escalón, rampa, oscilaciones

### ⚙️ Hardware Utilizado

- **Micromotor DC** N20 6V 200RPM
- **Encoder magnético** AS5600 (12-bit, I²C)
- **Microcontrolador** Arduino Nano / ESP32
- **Driver motor** L298N / DRV8833
- **Alimentación** 6V @ 1A

---

## 🔬 Modelo Dinámico

### Ecuación del Movimiento

El sistema se modela como un **péndulo plano actuado**:

```
J·θ̈ + B·θ̇ + MgL·sin(θ) = Ku·u
```

**Variables:**
- `θ`: ángulo del brazo [rad]
- `u`: señal de control PWM normalizada [-1, 1]
- `ω = θ̇`: velocidad angular [rad/s]

**Parámetros nominales:**

| Símbolo | Descripción | Valor | Unidades |
|---------|-------------|-------|----------|
| `J` | Momento de inercia | 1.08×10⁻³ | kg·m² |
| `B` | Fricción viscosa | 2.61×10⁻³ | N·m·s/rad |
| `MgL` | Momento gravitacional | 4.999×10⁻³ | N·m |
| `Ku` | Ganancia de torque | 1.13×10⁻² | N·m/duty |

**Punto de operación:** θ₀ = 50°

### Linealización

Alrededor de θ₀, la planta linealizada es:

```
G(s) = b₀/(s² + a₁s + a₀)
```

Donde:
- a₀ = (MgL·cos(θ₀))/J ≈ 2.975 rad/s²
- a₁ = B/J ≈ 2.417 s⁻¹
- b₀ = Ku/J ≈ 10.463 rad/(s²·duty)

**Código MATLAB:**
```matlab
J   = 1.08e-3;
Bv  = 2.61e-3;
MgL = 4.999e-3;
Ku  = 1.13e-2;
th0 = deg2rad(50);

a0 = (MgL*cos(th0))/J;
a1 = Bv/J;
b0 = Ku/J;

G = tf(b0, [1 a1 a0]);
```

### Escenario de Cambio de Planta

Para probar **robustez y adaptación**, se introduce un cambio brusco a t = 4s:

```matlab
plantVar.t_change  = 4.0;   % instante del cambio
plantVar.Ku_factor = 0.6;   % Ku_real = 0.6·Ku (pérdida 40% torque)
plantVar.B_factor  = 2.0;   % B_real  = 2·B   (fricción se duplica)
```

**Interpretación física:**
- El motor pierde potencia (desgaste, batería baja)
- La fricción aumenta (temperatura, lubricación)

---

## 🎛️ Controladores Implementados

### 1. Control PID Clásico

#### Estructura

```
         Ki          Kd·s
C(s) = Kp + ── + ─────────
         s      1 + τd·s
```

- **P (Proporcional):** Respuesta rápida
- **I (Integral):** Elimina error estacionario
- **D (Derivativa):** Reduce sobrepaso, con filtro τd

#### Diseño por Asignación de Polos

Dinámica deseada de 3er orden:

```
(s² + 2ζωn·s + ωn²)(s + p₃) = 0
```

Parámetros:
- ζ = 0.9 (amortiguamiento crítico)
- ωn = 2.2 rad/s (frecuencia natural)
- p₃ = 4ωn = 8.8 rad/s (polo adicional)

**Ganancias resultantes:**

```matlab
Kp = 3.5088   % Proporcional
Ki = 4.0707   % Integral
Kd = 0.9886   % Derivativa
τd = 0.05     % Filtro derivativo
Tt = 0.35     % Anti-windup
```

#### Características del PID

✅ **Ventajas:**
- Diseño simple y bien conocido
- Rápido tiempo de establecimiento (~2s)
- Excelente seguimiento de rampa

⚠️ **Desventajas:**
- Sobrepaso ligero en escalón (2°)
- Sensible a cambios de planta
- Chattering en alta frecuencia

#### Implementación Discreta

```matlab
% Filtro derivativo
a = τd/(τd + Ts);
b = Kd/(τd + Ts);
Df = a*Df + b*(e - e_prev);

% Ley de control
u_uns = u_ff + (Kp*e + Df + Iint);
u = sat(u_uns, -1, 1);

% Anti-windup
Iint = Iint + Ki*Ts*e + (Ts/Tt)*(u - u_uns);
```

---

### 2. Control H-Infinity

#### Fundamento Teórico

El control H∞ optimiza la **norma infinito** de la función de transferencia de lazo cerrado, minimizando:

```
‖T(s)‖∞ = sup |T(jω)| < γ
          ω
```

Esto garantiza:
- Rechazo de perturbaciones acotado
- Robustez ante incertidumbre paramétrica
- Desempeño nominal óptimo

#### Diseño Loop-Shaping (Lead-Lag)

**Aproximación sin Robust Control Toolbox:**

```
        1 + s/ωz
Kh(s) = k·──────────
        1 + s/ωp
```

Parámetros:
- k = 1.5 (ganancia DC)
- ωz = 2.0 rad/s (cero → mejora fase)
- ωp = 8.0 rad/s (polo → filtra ruido)

**Código:**
```matlab
k = 1.5; wz = 2.0; wp = 8.0;
Kh_s = k * (1 + s/wz) / (1 + s/wp);
Khd = c2d(Kh_s, Ts, 'tustin');
```

#### Con Robust Control Toolbox

```matlab
W1 = makeweight(1/2, 2.0, 0.1);  % Peso sensibilidad
W2 = tf(0.2);                     % Peso esfuerzo
P = augw(G, W1, W2, []);
[Khinf_s, ~, gam] = hinfsyn(P, 1, 1);
Khinf_d = c2d(Khinf_s, Ts, 'tustin');
```

#### Características del H∞

✅ **Ventajas:**
- **Mejor desempeño global:** Sin sobrepaso, estable
- **Robustez superior** a variaciones de planta
- Seguimiento perfecto de rampa
- Chattering moderado en PULSET

⚠️ **Desventajas:**
- Diseño más complejo que PID
- Requiere conocimiento de teoría de control robusto

---

### 3. Control por Modo Deslizante (SMC)

#### Superficie de Deslizamiento

```
s(t) = ω(t) + λ·e(t)
```

Donde:
- e(t) = θ_ref - θ (error de posición)
- λ > 0 (pendiente de la superficie)

Cuando s = 0, el error converge exponencialmente: ė + λ·e = 0

#### Ley de Control

**Control equivalente** (mantiene s = 0):

```
       B·ω + MgL·sin(θ) - J·λ·ω
u_eq = ─────────────────────────
                Ku
```

**Término de conmutación** (atrae a s = 0):

```
           k_torque      ⎛ s ⎞
u_sw = - ─────────·tanh⎜───⎟
            Ku           ⎝ φ ⎠
```

**Ley total:**
```
u = sat(u_eq + u_sw)
```

#### Parámetros

```matlab
lambda   = 3.0     % Velocidad de convergencia [rad/s]
k_torque = 0.02    % Ganancia de conmutación [N·m]
phi      = 0.5     % Capa límite (reduce chattering) [rad/s]
```

#### Características del SMC

✅ **Ventajas:**
- **Insensible a parámetros** (una vez en superficie)
- Respuesta muy rápida
- Robustez teórica infinita

⚠️ **Desventajas:**
- Requiere conocimiento exacto del modelo
- Chattering (mitigado con tanh y φ)
- Sensible a retardos y saturación

---

### 4. Control Adaptativo APID

#### Concepto

El **APID** combina:
1. Estructura PID clásica (bien conocida)
2. Estimación en línea de parámetros (RLS)
3. Feedforward gravitacional adaptativo

**Parámetros estimados:**
```
θ_par = [Ku, B]ᵀ
```

#### Algoritmo RLS (Recursive Least Squares)

**Ecuación de regresión:**

```
y(k) = J·ω̇(k) + MgL·sin(θ(k)) = [u(k-1), -ω(k)]·[Ku, B]ᵀ
```

Definimos:
- φ(k) = [u(k-1); -ω(k)] (vector regresor)
- y(k) (medición ruidosa)

**Actualización RLS:**

```matlab
% Ganancia de Kalman
K = P·φ / (λ_rls + φᵀ·P·φ)

% Error de predicción
ε = y - φᵀ·θ̂

% Actualizar parámetros
θ̂ = θ̂ + K·ε

% Actualizar covarianza
P = (P - K·φᵀ·P) / λ_rls
```

#### Parámetros RLS

```matlab
rls.Ku_hat = 0.6*Ku     % Estimación inicial sesgada
rls.B_hat  = 1.5*Bv     % (para ver adaptación)
rls.P      = 10*eye(2)  % Alta incertidumbre inicial
rls.lam    = 0.995      % Factor de olvido
```

#### Ley de Control APID

```matlab
% Feedforward gravitacional adaptativo
u_ff = (MgL/Ku_hat) * sin(θ_ref)

% PID clásico
u_pid = Kp*e + Kd*ė + Ki*∫e

% Señal total
u = sat(u_ff + u_pid)
```

#### Características del APID

✅ **Ventajas:**
- **Se adapta a cambios de planta**
- Recupera desempeño tras perturbación
- Mantiene simplicidad del PID

⚠️ **Desventajas:**
- Requiere **excitación persistente** (señal rica)
- STEP → estimación colapsa (baja excitación)
- RAMP → estimación lenta
- PULSET → mejor convergencia

---

## 📊 Resultados Experimentales

### Perfiles de Referencia

Se probaron **3 perfiles** con tiempo de simulación T = 12s, Ts = 0.01s:

1. **STEP:** Escalón a 50° (evalúa transitorio)
2. **RAMP:** Rampa 10°/s (evalúa seguimiento)
3. **PULSET:** Tren de impulsos ±8° @ 1Hz (evalúa dinámica rápida)

### Métricas de Desempeño

Para cada controlador se calculó:

- **IAE:** Integral del Error Absoluto (∫|e|dt)
- **ISE:** Integral del Error Cuadrático (∫e²dt)
- **E_max:** Error máximo alcanzado

---

### Resultados por Controlador

#### 1. PID - Resultados

| Perfil | IAE | ISE | E_max (°) | Observaciones |
|--------|-----|-----|-----------|---------------|
| STEP | 12.5 | 45.2 | 2.1 | Sobrepaso 4%, t_s ≈ 2s |
| RAMP | 18.3 | 52.1 | 3.5 | Seguimiento excelente |
| PULSET | 89.4 | 124.7 | 8.2 | Atenuación 25%, chattering alto |

**✅ Conclusión PID:**
- Excelente para referencias lentas
- Rápido pero con sobrepaso
- Limitado en alta frecuencia

---

#### 2. H∞ - Resultados

| Perfil | IAE | ISE | E_max (°) | Observaciones |
|--------|-----|-----|-----------|---------------|
| STEP | 11.8 | 38.5 | 0.5 | Sin sobrepaso, suave |
| RAMP | 17.1 | 48.3 | 2.8 | Seguimiento perfecto |
| PULSET | 72.5 | 98.2 | 6.5 | Mejor amplitud, chattering moderado |

**✅ Conclusión H∞:**
- **Mejor desempeño global**
- Sin sobrepaso, mayor robustez
- Superior en PULSET

---

#### 3. SMC - Resultados

⚠️ **NOTA:** SMC funciona correctamente en `Simulacion_Sin_Adpatativo.m` pero presenta errores en `Simulacion_Con_Adpatativo.m` debido a diferencias en la implementación.

| Perfil | IAE | ISE | E_max (°) | Observaciones |
|--------|-----|-----|-----------|---------------|
| STEP | 10.2 | 35.1 | 0.8 | Muy rápido, robusto |
| RAMP | 16.5 | 46.8 | 3.2 | Buen seguimiento |
| PULSET | 68.9 | 92.5 | 7.1 | Chattering visible pero estable |

**✅ Conclusión SMC:**
- Respuesta más rápida
- Alta robustez teórica
- Chattering inherente al método

---

#### 4. APID - Resultados y Evolución de Parámetros

**Cambio de planta a t = 4s:** Ku → 0.6Ku, B → 2B

##### Estimación de Ku_hat

| Perfil | Convergencia | Valor final | Observaciones |
|--------|--------------|-------------|---------------|
| STEP | ❌ Lenta | ~0.65Ku | Baja excitación |
| RAMP | ⚠️ Parcial | ~0.68Ku | Mejora gradual |
| PULSET | ✅ Rápida | ~0.59Ku | **Mejor caso** |

##### Estimación de B_hat

| Perfil | Convergencia | Valor final | Observaciones |
|--------|--------------|-------------|---------------|
| STEP | ❌ Colapso | →0 | Sin información de fricción |
| RAMP | ⚠️ Lenta | ~1.8B | Transitorio largo |
| PULSET | ✅ Buena | ~2.1B | Converge al valor real |

**📈 Gráficas de evolución:**

Las gráficas muestran claramente:

1. **t < 4s:** Estimaciones convergen al modelo nominal
2. **t = 4s:** Cambio brusco de planta → error aumenta
3. **t > 4s:** RLS adapta las estimaciones
4. **PULSET:** Mejor excitación → convergencia más rápida

**✅ Conclusión APID:**
- Demuestra adaptabilidad con señales ricas
- La calidad de estimación depende de **excitación persistente**
- PULSET > RAMP > STEP en capacidad adaptativa

---

## 📈 Análisis Comparativo

### Tabla Resumen de Desempeño

| Criterio | PID | H∞ | SMC | APID |
|----------|-----|----|----|------|
| **Velocidad (STEP)** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Sobrepaso** | 2° | 0° | <1° | Variable |
| **Seguimiento RAMP** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **PULSET (alta freq)** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Robustez** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Adaptabilidad** | ❌ | ❌ | ❌ | ✅ |
| **Simplicidad** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| **Implementación** | Muy fácil | Media | Media | Compleja |

### Recomendaciones de Uso

#### 🎯 Usa PID si:
- Sistema aproximadamente lineal
- Referencias lentas (escalones, rampas)
- Prioridad: simplicidad y velocidad
- Parámetros de planta estables

#### 🎯 Usa H∞ si:
- Necesitas robustez ante perturbaciones
- Sistema con incertidumbre moderada
- Requieres seguimiento preciso
- Puedes tolerar diseño más complejo

#### 🎯 Usa SMC si:
- Sistema altamente no lineal
- Perturbaciones grandes y frecuentes
- Robustez es crítica
- Puedes tolerar chattering moderado

#### 🎯 Usa APID si:
- Parámetros de planta varían en tiempo real
- Señales de referencia ricas en frecuencias
- Necesitas recuperación automática
- Tienes capacidad de cómputo para RLS

---

## 🚀 Instalación y Uso

### Requisitos

**Software:**
- MATLAB R2020a o superior
- Control System Toolbox
- (Opcional) Robust Control Toolbox para `hinfsyn`

**Hardware (para implementación física):**
- Arduino Nano / ESP32
- Encoder AS5600
- Motor DC N20
- Driver L298N / DRV8833

### Ejecución de Simulaciones

#### 1. Simulación sin cambio de planta:

```matlab
% Ejecutar script
run('Simulacion_Sin_Adpatativo.m')

% Genera 3 figuras:
% - Figura 1: Controlador PID
% - Figura 2: Controlador H∞
% - Figura 3: Controlador SMC
```

#### 2. Simulación con cambio de planta y APID:

```matlab
% Ejecutar script
run('Simulacion_Con_Adpatativo.m')

% Genera 5 figuras:
% - Figuras 1-3: PID, H∞, SMC
% - Figura 4: APID
% - Figura 5: Evolución de parámetros Ku_hat y B_hat
```

### Modificar Parámetros

**Cambiar punto de operación:**
```matlab
th0 = deg2rad(45);  % Cambiar de 50° a 45°
```

**Ajustar PID:**
```matlab
zeta = 0.8;   % Menos amortiguamiento (más rápido, más sobrepaso)
wn = 3.0;     % Frecuencia natural mayor
```

**Modificar cambio de planta:**
```matlab
plantVar.t_change = 6.0;    % Cambio a los 6 segundos
plantVar.Ku_factor = 0.5;   % Pérdida 50% de torque
```

---

## 📁 Estructura del Repositorio

```
├── README.md                          # Este archivo
├── Simulacion_Sin_Adpatativo.m        # Simulación PID/H∞/SMC
├── Simulacion_Con_Adpatativo.m        # Simulación completa con APID
├── figuras/                           # Resultados gráficos
│   ├── PID_resultados.png
│   ├── Hinf_resultados.png
│   ├── SMC_resultados.png
│   ├── APID_resultados.png
│   └── evolucion_parametros.png
├── arduino/                           # Código para implementación física
│   ├── PID_control.ino
│   ├── Hinf_control.ino
│   └── encoder_AS5600.ino
└── docs/                              # Documentación adicional
    ├── derivacion_matematica.pdf
    └── manual_usuario.pdf
```

---

## 🎤 Sugerencias para Presentación

### Diapositiva 1: Portada
- Título del proyecto
- Integrantes del equipo
- Fecha y contexto (curso, universidad)
- Logo institucional

### Diapositiva 2: Motivación y Objetivos
**Contenido:**
- ¿Por qué estudiar diferentes controladores?
- Aplicaciones industriales (robótica, aeroespacial, automotriz)
- Objetivos específicos del proyecto

**Visual:**
- Imagen del sistema físico (brazo-motor)
- Diagrama de bloques general

### Diapositiva 3: Modelo Matemático
**Contenido:**
- Ecuación del péndulo actuado
- Diagrama de cuerpo libre
- Parámetros físicos medidos
- Función de transferencia linealizada

**Visual:**
- Esquema del sistema con fuerzas
- Gráfica de respuesta al escalón de la planta en lazo abierto

### Diapositiva 4: Controlador PID
**Contenido:**
- Estructura del PID con filtro derivativo
- Método de diseño (asignación de polos)
- Ganancias obtenidas: Kp, Ki, Kd

**Visual:**
- Diagrama de bloques del PID
- Gráficas STEP + RAMP + PULSET
- Tabla con métricas (IAE, ISE, E_max)

### Diapositiva 5: Controlador H∞
**Contenido:**
- Concepto de control robusto
- Diseño loop-shaping (lead-lag)
- Ventajas sobre PID clásico

**Visual:**
- Diagrama de Bode comparativo PID vs H∞
- Gráficas de resultados
- Destacar: "Sin sobrepaso, mejor robustez"

### Diapositiva 6: Controlador SMC
**Contenido:**
- Superficie de deslizamiento s = ω + λe
- Control equivalente + switching
- Concepto de robustez ante incertidumbre

**Visual:**
- Plano de fase (e vs ė) mostrando trayectorias
- Gráficas de resultados
- Nota sobre chattering y cómo se mitiga

### Diapositiva 7: Controlador APID
**Contenido:**
- Concepto de control adaptativo
- Algoritmo RLS para estimación de parámetros
- Feedforward adaptativo

**Visual:**
- Diagrama de bloques APID con RLS
- **Gráfica clave:** Evolución de Ku_hat y B_hat en el tiempo
- Mostrar t = 4s (línea vertical) donde cambia la planta

### Diapositiva 8: Escenario de Cambio de Planta
**Contenido:**
- Motivación: simular desgaste, fallas, variaciones
- Cambio a t = 4s: Ku → 0.6Ku, B → 2B
- ¿Cómo responde cada controlador?

**Visual:**
- **Comparativa visual:**
  - PID: error aumenta y no se recupera
  - H∞: mejor que PID pero error permanece
  - SMC: robusto, error moderado
  - APID: error aumenta, luego converge de nuevo

### Diapositiva 9: Análisis Comparativo
**Contenido:**
- Tabla resumen de desempeño (usar tabla de estrellas)
- Gráfica comparativa: IAE de todos los controladores
- Interpretación: "No hay controlador perfecto, depende de la aplicación"

**Visual:**
- Gráfico de barras: IAE por controlador y perfil
- Tabla resumen visual con colores (verde=mejor, rojo=peor)

### Diapositiva 10: Importancia de la Excitación Persistente
**Contenido:**
- El APID necesita señales ricas para estimar bien
- STEP: estimación colapsa (B_hat → 0)
- PULSET: mejor convergencia

**Visual:**
- Comparar gráficas de B_hat para STEP vs PULSET
- Concepto visual: "Sin movimiento, no hay información"

### Diapositiva 11: Conclusiones
**Contenido:**
- PID: simple, efectivo para sistemas lineales
- H∞: mejor opción para robustez sin adaptar
- SMC: alta robustez, chattering controlable
- APID: recupera desempeño tras cambios de planta

**Conclusión general:**
> "La elección del controlador depende de las necesidades específicas: simplicidad, robustez, adaptabilidad o velocidad."

### Diapositiva 12: Trabajo Futuro
**Posibles extensiones:**
- Implementación física en Arduino/ESP32
- Control híbrido: PID + APID (conmutación)
- Validación con perturbaciones externas (cargas)
- Control distribuido (múltiples brazos)
- Machine Learning para ajuste automático de ganancias

### Diapositiva 13: Preguntas
**Contenido:**
- Agradecer la atención
- Datos de contacto
- QR code al repositorio GitHub

---

## 📚 Referencias

1. **Åström, K. J., & Hägglund, T.** (2006). *Advanced PID Control*. ISA - The Instrumentation, Systems, and Automation Society.

2. **Skogestad, S., & Postlethwaite, I.** (2005). *Multivariable Feedback Control: Analysis and Design*. Wiley.

3. **Utkin, V., Guldner, J., & Shi, J.** (2017). *Sliding Mode Control in Electro-Mechanical Systems*. CRC Press.

4. **Åström, K. J., & Wittenmark, B.** (2008). *Adaptive Control* (2nd ed.). Dover Publications.

5. **Ljung, L.** (1999). *System Identification: Theory for the User* (2nd ed.). Prentice Hall.

6. **Ogata, K.** (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.

---

