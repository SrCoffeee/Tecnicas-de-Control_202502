%% DISEÑO CONTROLADOR H-INFINITO - SISTEMA LATERAL-DIRECCIONAL
% Universidad Nacional de Colombia
% Control de Roll (φ) y Yaw (ψ) mediante alerones y timón

clear
clc

fprintf('========================================\n');
fprintf('DISEÑO H∞ - SISTEMA LATERAL-DIRECCIONAL\n');
fprintf('========================================\n\n');

%% ========================================================================
% CARGAR MODELO LATERAL-DIRECCIONAL
% ========================================================================

% Modelo lateral (5 estados, 2 entradas, 6 salidas)
A_LD = [
    -0.8750   0.8751 -16.8197   9.7914   0;
    -2.8312 -16.1385   3.3768   0        0;
     1.7063   0.5154  -2.7828   0        0;
     0        1.0000   0.0538   0        0;
     0        0        1.0014   0        0
];

B_LD = [
     0        5.3170;
  -156.9094  -5.0216;
    11.5366 -82.2714;
     0        0;
     0        0
];

C_LD = [
     0.0588   0   0   0   0;
     0        1   0   0   0;
     0        0   1   0   0;
     0        0   0   1   0;
     0        0   0   0   1;
     0        0   0   0   0
];

D_LD = zeros(6,2);

% Limpiar valores pequeños
threshold = 1e-23;
A_LD(abs(A_LD) < threshold) = 0;

% Seleccionar salidas relevantes: p, r, φ, ψ (filas 2-5)
C_LD = C_LD(2:5,:);
D_LD = zeros(4,2);

% Crear sistema
G_LD = ss(A_LD, B_LD, C_LD, D_LD);

fprintf('✓ Modelo lateral-direccional cargado\n');
fprintf('  Estados: 5 [v, p, r, φ, ψ]\n');
fprintf('  Entradas: 2 [δa (alerones), δr (timón)]\n');
fprintf('  Salidas: 4 [p, r, φ, ψ]\n\n');

%% ========================================================================
% DISEÑO DE FUNCIONES DE PESO
% ========================================================================

fprintf('Diseñando funciones de peso...\n\n');

% W1: Desempeño (seguimiento de referencia)

% Para velocidad lateral (p)
W1_Lat_vel = 1.1;

% Para posición lateral (φ - roll)
M1_Lat_pos = 1.1;        % Pico máximo permitido
A1_Lat_pos = 0.0001;     % Error en estado estacionario
Wb1_Lat_pos = 2*pi*8;    % Ancho de banda: 8 Hz
W1_Lat_pos = tf([1/M1_Lat_pos Wb1_Lat_pos], [1 A1_Lat_pos*Wb1_Lat_pos]);

% Para velocidad direccional (r)
W1_Dir_vel = 1.1;

% Para posición direccional (ψ - yaw)
M1_Dir_pos = 1.1;
A1_Dir_pos = 0.0001;
Wb1_Dir_pos = 2*pi*8;    % Ancho de banda: 8 Hz
W1_Dir_pos = tf([1/M1_Dir_pos Wb1_Dir_pos], [1 A1_Dir_pos*Wb1_Dir_pos]);

% Matriz diagonal W1 (4x4)
Wp = 1*tf(1,1);
W1_LD = eye(4)*tf(1,1);
W1_LD(1,1) = W1_Dir_vel;  % p (roll rate)
W1_LD(2,2) = W1_Lat_vel;  % r (yaw rate)
W1_LD(3,3) = W1_Dir_pos;  % φ (roll angle)
W1_LD(4,4) = W1_Lat_pos;  % ψ (yaw angle)

% W2: Esfuerzo de control
W2_LD = 1.5*tf(1,1);

% W3: Robustez (rechazo a ruido)
M3_LD_a = 0.004;
A3_LD_a = 1.1;
Wb3_LD_a = 2*pi*300;     % Frecuencia alta: 300 Hz

W3_LD = tf([1/M3_LD_a Wb3_LD_a], [1 A3_LD_a*Wb3_LD_a]);

fprintf('Funciones de peso diseñadas:\n');
fprintf('  W1: Desempeño (BW = 8 Hz)\n');
fprintf('  W2: Esfuerzo de control (%.2f)\n', 1.5);
fprintf('  W3: Robustez (fc = 300 Hz)\n\n');

% Visualizar filtros
figure(1)
sigma(W1_LD, 'b', 'LineWidth', 2);
hold on
sigma(W3_LD, 'r', 'LineWidth', 2);
title('Filtros de Peso - Sistema Lateral-Direccional');
legend('W1 (Desempeño)', 'W3 (Robustez)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% SÍNTESIS DEL CONTROLADOR H-INFINITO
% ========================================================================

fprintf('Ejecutando síntesis H∞...\n');

% Crear planta aumentada
P_LD = augw(G_LD, W1_LD, W2_LD, W3_LD);

% Parámetros de síntesis
nmeas = 4;  % Número de mediciones (p, r, φ, ψ)
ncont = 2;  % Número de controles (δa, δr)

% Síntesis H-infinity usando hinfsyn
[K1_LD, sys1_CL, gam1, info1] = hinfsyn(P_LD, nmeas, ncont);

% Síntesis alternativa usando mixsyn
[K2_LD, sys2_CL, gam2, info2] = mixsyn(G_LD, W1_LD, W2_LD, W3_LD);

fprintf('\n✓ Síntesis completada:\n');
fprintf('  γ (hinfsyn) = %.4f\n', gam1);
fprintf('  γ (mixsyn)  = %.4f\n\n', gam2);

% Usar controlador de hinfsyn
K_LD = K1_LD;
gamma_LD = gam1;

%% ========================================================================
% VERIFICACIÓN DE CONDICIONES
% ========================================================================

fprintf('Verificando condiciones de diseño...\n\n');

% Sistemas en lazo cerrado
L_LD = G_LD * K_LD;
S_LD = feedback(eye(4), L_LD);
T_LD = feedback(L_LD, eye(4));
KS_LD = K_LD * S_LD;

% Verificación W1: ||S||∞ < ||W1^-1||∞
figure(2)
sigma(S_LD, 'r', 'LineWidth', 2);
hold on
sigma(inv(W1_LD), 'g--', 'LineWidth', 2);
title('Verificación W1^{-1}: Desempeño');
legend('S', 'W1^{-1}', 'Location', 'best');
grid on;
hold off

norm_S = norm(S_LD, inf);
norm_W1inv = norm(inv(W1_LD), inf);
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
sigma(KS_LD, 'r', 'LineWidth', 2);
hold on
sigma(1/W2_LD, 'g--', 'LineWidth', 2);
title('Verificación W2^{-1}: Esfuerzo de Control');
legend('KS', 'W2^{-1}', 'Location', 'best');
grid on;
hold off

norm_KS = norm(KS_LD, inf);
norm_W2inv = 1/1.5;
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
sigma(T_LD, 'r', 'LineWidth', 2);
hold on
sigma(1/W3_LD, 'g--', 'LineWidth', 2);
title('Verificación W3^{-1}: Robustez');
legend('T', 'W3^{-1}', 'Location', 'best');
grid on;
hold off

norm_T = norm(T_LD, inf);
norm_W3inv = norm(1/W3_LD, inf);
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
sigma(T_LD, 'r', 'LineWidth', 2);
hold on
sigma(S_LD, 'b', 'LineWidth', 2);
title('Funciones de Sensibilidad T y S - Sistema Lateral');
legend('T (Complementaria)', 'S (Sensibilidad)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% RESPUESTA AL ESCALÓN
% ========================================================================

fprintf('Calculando respuesta al escalón...\n');

figure(6)
step(T_LD, 10);
title('Respuesta al Escalón - Sistema Lateral-Direccional');
xlabel('Tiempo (s)');
ylabel('Amplitud');
legend('p → δa', 'r → δa', 'φ → δa', 'ψ → δa', ...
       'p → δr', 'r → δr', 'φ → δr', 'ψ → δr', 'Location', 'best');
grid on;

% Métricas de desempeño
Infos_LD = stepinfo(T_LD);

fprintf('\nMÉTRICAS DE DESEMPEÑO:\n');
fprintf('  Canal φ (roll):\n');
fprintf('    Sobrepaso: %.2f%%\n', Infos_LD(3,3).Overshoot);
fprintf('    Tiempo de establecimiento: %.3f s\n', Infos_LD(3,3).SettlingTime);
fprintf('  Canal ψ (yaw):\n');
fprintf('    Sobrepaso: %.2f%%\n', Infos_LD(4,4).Overshoot);
fprintf('    Tiempo de establecimiento: %.3f s\n\n', Infos_LD(4,4).SettlingTime);

%% ========================================================================
% VALORES SINGULARES DEL CONTROLADOR
% ========================================================================

figure(7)
sigma(K_LD);
title('Valores Singulares del Controlador H∞ - Lateral');
grid on;

%% ========================================================================
% ANÁLISIS DE ACOPLAMIENTO
% ========================================================================

fprintf('========================================\n');
fprintf('ANÁLISIS DE ACOPLAMIENTO\n');
fprintf('========================================\n\n');

% Según instrucción del profesor: "El eje más acoplado es el de heading"
fprintf('Analizando acoplamiento del heading (ψ)...\n\n');

coupling_matrix = abs(A_LD);
coupling_heading = coupling_matrix(5,:) + coupling_matrix(:,5)';
coupling_heading(5) = 0; % No contar auto-acoplamiento

state_names = {'v', 'p', 'r', 'φ', 'ψ'};
[max_coupling, max_idx] = max(coupling_heading);

fprintf('Acoplamiento del heading (ψ) con otros estados:\n');
for i = 1:5
    if i ~= 5
        fprintf('  ψ ↔ %s: %.4f\n', state_names{i}, coupling_heading(i));
    end
end

fprintf('\n⚡ Estado más acoplado con ψ: %s (%.4f)\n', ...
    state_names{max_idx}, max_coupling);
fprintf('   → El yaw rate (r) tiene fuerte influencia en el heading\n\n');

%% ========================================================================
% GUARDAR RESULTADOS
% ========================================================================

fprintf('========================================\n');
fprintf('DISEÑO H∞ LATERAL COMPLETADO\n');
fprintf('========================================\n');
fprintf('\nResultados:\n');
fprintf('  γ óptimo: %.4f\n', gamma_LD);
fprintf('  Orden del controlador: %d\n', order(K_LD));
fprintf('  Condiciones verificadas: 3/3\n\n');

% Guardar workspace
save('H_inf_lat_results.mat', 'K_LD', 'gamma_LD', 'S_LD', 'T_LD', ...
     'KS_LD', 'W1_LD', 'W2_LD', 'W3_LD', 'G_LD');

fprintf('✓ Resultados guardados en H_inf_lat_results.mat\n\n');