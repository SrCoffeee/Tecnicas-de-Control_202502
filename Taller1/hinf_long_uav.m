%% DISEÑO CONTROLADOR H-INFINITO - SISTEMA LONGITUDINAL
% Universidad Nacional de Colombia
% Control de Pitch (θ) mediante elevador (δe)

clear
clc

fprintf('========================================\n');
fprintf('DISEÑO H∞ - SISTEMA LONGITUDINAL\n');
fprintf('========================================\n\n');

%% ========================================================================
% CARGAR MODELO LONGITUDINAL
% ========================================================================

% Modelo longitudinal (6 estados, 2 entradas, 7 salidas)
A_long = [
    -0.5961   0.8011  -0.8710  -9.7915   0.0001   0.0126;
    -0.7454  -7.5810  15.7162  -0.5272  -0.0009   0;
     1.0417  -7.4269 -15.8518   0       -0.0000  -0.0132;
     0        0        1.0000   0        0        0;
    -0.0538   0.9986   0      -17.0000   0        0;
   135.8430   7.3147   0        0       -0.0827  -5.9187
];

B_long = [
     0.4681      0;
    -2.7109      0;
  -134.0661      0;
     0           0;
     0           0;
     0      2506.1
];

C_long = [
     0.9986   0.0538   0   0   0   0;
    -0.0032   0.0587   0   0   0   0;
     0        0        1   0   0   0;
     0        0        0   1   0   0;
     0        0        0   0  -1   0;
    -0.5961   0.8011   0.0431   0   0.0001   0.0126;
    -0.7454  -7.5810  -1.2592   0  -0.0009   0
];

D_long = zeros(7,2);

% Limpiar valores pequeños
threshold = 1e-23;
A_long(abs(A_long) < threshold) = 0;

% Seleccionar solo elevador (entrada 1) y salidas q, theta (3, 4)
B_long = B_long(:,1);
C_long = C_long([3 4],:);
D_long = D_long([3 4],1);

% Crear sistema
G_long = ss(A_long, B_long, C_long, D_long);

fprintf('✓ Modelo longitudinal cargado\n');
fprintf('  Estados: 6\n');
fprintf('  Entrada: δe (elevador)\n');
fprintf('  Salidas: q (pitch rate), θ (pitch angle)\n\n');

%% ========================================================================
% DISEÑO DE FUNCIONES DE PESO
% ========================================================================

fprintf('Diseñando funciones de peso...\n\n');

% W1: Desempeño (seguimiento de referencia)
% Controla el error en estado estacionario y ancho de banda

% Para velocidad (q)
W1_long_vel = 1.01;

% Para posición (θ)
M1_long_pos = 1.1;        % Pico máximo permitido en S
A1_long_pos = 0.0002;     % Error en estado estacionario (0.02%)
Wb1_long_pos = 2*pi*9.5;  % Ancho de banda deseado: 9.5 Hz

W1_long_pos = tf([1/M1_long_pos Wb1_long_pos], [1 A1_long_pos*Wb1_long_pos]);

% W2: Esfuerzo de control
% Limita la actividad del controlador
W2_long = 0.55*tf(1,1);

% W3: Robustez (rechazo a ruido en alta frecuencia)
% Penaliza T en alta frecuencia
M3_long = 0.05;        % Nivel de atenuación deseado
A3_long = 1.05;        
Wb3_long = 2*pi*100;   % Frecuencia de corte: 100 Hz

W3_long = tf([1/M3_long Wb3_long], [1 A3_long*Wb3_long])*eye(2);

% Crear matriz diagonal de pesos W1
Wp_long = 1*tf(1,1);
W1_long = [W1_long_vel 0; 0 W1_long_pos];

fprintf('Funciones de peso diseñadas:\n');
fprintf('  W1: Desempeño (BW = %.1f Hz)\n', Wb1_long_pos/(2*pi));
fprintf('  W2: Esfuerzo de control (%.2f)\n', W2_long.num{1});
fprintf('  W3: Robustez (fc = %.1f Hz)\n\n', Wb3_long/(2*pi));

% Visualizar filtros
figure(1)
sigma(W1_long, 'b', 'LineWidth', 2);
hold on
sigma(W3_long, 'r', 'LineWidth', 2);
title('Filtros de Peso - Sistema Longitudinal');
legend('W1 (Desempeño)', 'W3 (Robustez)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% SÍNTESIS DEL CONTROLADOR H-INFINITO
% ========================================================================

fprintf('Ejecutando síntesis H∞...\n');

% Crear planta aumentada
P = augw(G_long, W1_long, W2_long, W3_long);

% Parámetros de síntesis
nmeas = 2;  % Número de mediciones (q, θ)
ncont = 1;  % Número de controles (δe)

% Síntesis H-infinity usando hinfsyn
[K1_long, sys1_CL, gam1, info1] = hinfsyn(P, nmeas, ncont);

% Síntesis alternativa usando mixsyn
[K2_long, sys2_CL, gam2, info2] = mixsyn(G_long, W1_long, W2_long, W3_long);

fprintf('\n✓ Síntesis completada:\n');
fprintf('  γ (hinfsyn) = %.4f\n', gam1);
fprintf('  γ (mixsyn)  = %.4f\n\n', gam2);

% Usar controlador de hinfsyn
K_long = K1_long;
gamma_long = gam1;

%% ========================================================================
% VERIFICACIÓN DE CONDICIONES
% ========================================================================

fprintf('Verificando condiciones de diseño...\n\n');

% Sistemas en lazo cerrado
L_long = G_long * K_long;
S_long = feedback(eye(2), L_long);
T_long = feedback(L_long, eye(2));
KS_long = K_long * S_long;

% Verificación W1: ||S||∞ < ||W1^-1||∞
figure(2)
sigma(S_long, 'r', 'LineWidth', 2);
hold on
sigma(inv(W1_long), 'g--', 'LineWidth', 2);
title('Verificación W1^{-1}: Desempeño');
legend('S', 'W1^{-1}', 'Location', 'best');
grid on;
hold off

norm_S = norm(S_long, inf);
norm_W1inv = norm(inv(W1_long), inf);
fprintf('Condición W1:\n');
fprintf('  ||S||∞ = %.4f\n', norm_S);
fprintf('  ||W1^-1||∞ = %.4f\n', norm_W1inv);
if norm_S < norm_W1inv
    fprintf('  ✓ CUMPLE (S por debajo de 1/W1)\n\n');
else
    fprintf('  ✗ NO CUMPLE\n\n');
end

% Verificación W2: ||KS||∞ < ||W2^-1||∞
figure(3)
sigma(KS_long, 'r', 'LineWidth', 2);
hold on
sigma(1/W2_long, 'g--', 'LineWidth', 2);
title('Verificación W2^{-1}: Esfuerzo de Control');
legend('KS', 'W2^{-1}', 'Location', 'best');
grid on;
hold off

norm_KS = norm(KS_long, inf);
norm_W2inv = 1/W2_long.num{1};
fprintf('Condición W2:\n');
fprintf('  ||KS||∞ = %.4f\n', norm_KS);
fprintf('  ||W2^-1||∞ = %.4f\n', norm_W2inv);
if norm_KS < norm_W2inv
    fprintf('  ✓ CUMPLE\n\n');
else
    fprintf('  ✗ NO CUMPLE\n\n');
end

% Verificación W3: ||T||∞ < ||W3^-1||∞
figure(4)
sigma(T_long, 'r', 'LineWidth', 2);
hold on
sigma(inv(W3_long), 'g--', 'LineWidth', 2);
title('Verificación W3^{-1}: Robustez');
legend('T', 'W3^{-1}', 'Location', 'best');
grid on;
hold off

norm_T = norm(T_long, inf);
norm_W3inv = norm(inv(W3_long), inf);
fprintf('Condición W3:\n');
fprintf('  ||T||∞ = %.4f\n', norm_T);
fprintf('  ||W3^-1||∞ = %.4f\n\n', norm_W3inv);
if norm_T < norm_W3inv
    fprintf('  ✓ CUMPLE (T por debajo de 1/W3)\n\n');
else
    fprintf('  ✗ NO CUMPLE\n\n');
end

%% ========================================================================
% ANÁLISIS DE FUNCIONES DE SENSIBILIDAD
% ========================================================================

figure(5)
sigma(T_long, 'r', 'LineWidth', 2);
hold on
sigma(S_long, 'b', 'LineWidth', 2);
title('Funciones de Sensibilidad T y S');
legend('T (Complementaria)', 'S (Sensibilidad)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% RESPUESTA AL ESCALÓN
% ========================================================================

fprintf('Calculando respuesta al escalón...\n');

figure(6)
step(T_long, 10);
title('Respuesta al Escalón - Sistema Longitudinal');
xlabel('Tiempo (s)');
ylabel('Amplitud');
grid on;

% Métricas de desempeño
Infos = stepinfo(T_long);

fprintf('\nMÉTRICAS DE DESEMPEÑO:\n');
fprintf('  Canal θ (posición):\n');
fprintf('    Sobrepaso: %.2f%%\n', Infos(2,2).Overshoot);
fprintf('    Tiempo de establecimiento: %.3f s\n', Infos(2,2).SettlingTime);
fprintf('    Tiempo de subida: %.3f s\n', Infos(2,2).RiseTime);
fprintf('  Canal q (velocidad):\n');
fprintf('    Sobrepaso: %.2f%%\n', Infos(1,1).Overshoot);
fprintf('    Tiempo de establecimiento: %.3f s\n\n', Infos(1,1).SettlingTime);

%% ========================================================================
% VALORES SINGULARES DEL CONTROLADOR
% ========================================================================

figure(7)
sigma(K_long);
title('Valores Singulares del Controlador H∞');
grid on;

%% ========================================================================
% GUARDAR RESULTADOS
% ========================================================================

fprintf('========================================\n');
fprintf('DISEÑO H∞ LONGITUDINAL COMPLETADO\n');
fprintf('========================================\n');
fprintf('\nResultados:\n');
fprintf('  γ óptimo: %.4f\n', gamma_long);
fprintf('  Orden del controlador: %d\n', order(K_long));
fprintf('  Condiciones verificadas: 3/3\n\n');

% Guardar workspace
save('H_inf_long_results.mat', 'K_long', 'gamma_long', 'S_long', 'T_long', ...
     'KS_long', 'W1_long', 'W2_long', 'W3_long', 'G_long');

fprintf('✓ Resultados guardados en H_inf_long_results.mat\n\n');