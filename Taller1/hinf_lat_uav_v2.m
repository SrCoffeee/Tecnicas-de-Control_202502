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
% DISEÑO DE FUNCIONES DE PESO - MEJORADO
% ========================================================================

% W1: Desempeño (seguimiento de referencia) 

% Para velocidad lateral (p) 
W1_Lat_vel = 4.0;  

% Para posición lateral (φ - roll) -
M1_Lat_pos = 2.5;        
A1_Lat_pos = 0.00001;  
Wb1_Lat_pos = 2*pi*15;  
W1_Lat_pos = tf([1/M1_Lat_pos Wb1_Lat_pos], [1 A1_Lat_pos*Wb1_Lat_pos]);

% Para velocidad direccional (r) 
W1_Dir_vel = 4.0;  

% Para posición direccional (ψ - yaw) - 
M1_Dir_pos = 2.5;       
A1_Dir_pos = 0.00001;    
Wb1_Dir_pos = 2*pi*15; 
W1_Dir_pos = tf([1/M1_Dir_pos Wb1_Dir_pos], [1 A1_Dir_pos*Wb1_Dir_pos]);

% Matriz diagonal W1 (4x4)
W1_LD = eye(4)*tf(1,1);
W1_LD(1,1) = W1_Dir_vel;  % p (roll rate)
W1_LD(2,2) = W1_Lat_vel;  % r (yaw rate)
W1_LD(3,3) = W1_Dir_pos;  % φ (roll angle)
W1_LD(4,4) = W1_Lat_pos;  % ψ (yaw angle)

% W2: Esfuerzo de control - MENOS RESTRICTIVO (permite más control)
W2_LD = 0.8*tf(1,1);  % Antes: 1.5

% W3: Robustez (rechazo a ruido) - AJUSTADO
M3_LD_a = 0.008;         % Antes: 0.004
A3_LD_a = 1.1;
Wb3_LD_a = 2*pi*250;     % Antes: 2*pi*300 - Frecuencia alta: 250 Hz

W3_LD = tf([1/M3_LD_a Wb3_LD_a], [1 A3_LD_a*Wb3_LD_a]);

% Visualizar filtros
figure(1)
sigma(W1_LD);
hold on
sigma(W3_LD);
title('Filtros de Peso - Sistema Lateral-Direccional (MEJORADOS)');
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


% Sistemas en lazo cerrado
L_LD = G_LD * K_LD;
S_LD = feedback(eye(4), L_LD);
T_LD = feedback(L_LD, eye(4));
KS_LD = K_LD * S_LD;


%% ========================================================================
% ANÁLISIS DE FUNCIONES DE SENSIBILIDAD
% ========================================================================

figure(5)
sigma(T_LD);
hold on
sigma(S_LD);
title('Funciones de Sensibilidad T y S - Sistema Lateral');
legend('T (Complementaria)', 'S (Sensibilidad)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% RESPUESTA AL ESCALÓN - ESTILO MEJORADO CON DIAGNÓSTICO
% ========================================================================

fprintf('Calculando respuesta al escalón...\n');

% Parámetros de simulación
t_sim = 0:0.01:6;

% Respuesta al escalón para Roll (canal 3,3) y Yaw (canal 4,4)
[y_roll, t_roll] = step(T_LD(3,3), t_sim);
[y_yaw, t_yaw] = step(T_LD(4,4), t_sim);

% Figura para Roll
figure(6)
plot(t_roll, y_roll, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Step Response - Roll (φ)', 'FontSize', 12);
xlim([0 6]);
set(gca, 'FontSize', 10);

% Figura para Yaw
figure(7)
plot(t_yaw, y_yaw, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Step Response - Yaw (ψ)', 'FontSize', 12);
xlim([0 6]);
set(gca, 'FontSize', 10);

% Métricas de desempeño - Con diagnóstico
fprintf('\n=== DIAGNÓSTICO DE RESPUESTA AL ESCALÓN ===\n');

% Para Roll - simulación más larga
t_long = 0:0.01:30; % 30 segundos
[y_roll_long, t_roll_long] = step(T_LD(3,3), t_long);

fprintf('\nCanal Roll (φ):\n');
fprintf('  Valor inicial: %.6f\n', y_roll_long(1));
fprintf('  Valor a los 10s: %.6f\n', y_roll_long(find(t_roll_long >= 10, 1)));
fprintf('  Valor a los 20s: %.6f\n', y_roll_long(find(t_roll_long >= 20, 1)));
fprintf('  Valor final (30s): %.6f\n', y_roll_long(end));
fprintf('  Valor máximo: %.6f\n', max(y_roll_long));
fprintf('  Valor mínimo: %.6f\n', min(y_roll_long));

% Verificar si el sistema es estable
if abs(y_roll_long(end)) > 1e10 || isnan(y_roll_long(end)) || isinf(y_roll_long(end))
    fprintf('  ⚠️ ADVERTENCIA: Sistema posiblemente inestable o con problemas numéricos\n');
else
    % Intentar calcular stepinfo
    try
        % Método 1: stepinfo directo
        S_roll = stepinfo(T_LD(3,3), 'SettlingTimeThreshold', 0.02);
        
        fprintf('\n  Métricas calculadas:\n');
        fprintf('    Sobrepaso: %.2f%%\n', S_roll.Overshoot);
        fprintf('    Tiempo de establecimiento: %.3f s\n', S_roll.SettlingTime);
        fprintf('    Tiempo de subida: %.3f s\n', S_roll.RiseTime);
        fprintf('    Tiempo de pico: %.3f s\n', S_roll.PeakTime);
        
    catch ME
        fprintf('  ⚠️ Error en stepinfo automático: %s\n', ME.message);
        fprintf('  Calculando métricas manualmente...\n\n');
        
        % Cálculo manual
        valor_final = y_roll_long(end);
        
        % 1. Sobrepaso
        [pico, idx_pico] = max(y_roll_long);
        sobrepaso = ((pico - valor_final) / abs(valor_final)) * 100;
        tiempo_pico = t_roll_long(idx_pico);
        
        % 2. Tiempo de subida (10% a 90%)
        umbral_10 = 0.1 * valor_final;
        umbral_90 = 0.9 * valor_final;
        
        idx_10 = find(y_roll_long >= umbral_10, 1, 'first');
        idx_90 = find(y_roll_long >= umbral_90, 1, 'first');
        
        if ~isempty(idx_10) && ~isempty(idx_90)
            tiempo_subida = t_roll_long(idx_90) - t_roll_long(idx_10);
        else
            tiempo_subida = NaN;
        end
        
        % 3. Tiempo de establecimiento (2%)
        umbral_settling = 0.02 * abs(valor_final);
        banda_superior = valor_final + umbral_settling;
        banda_inferior = valor_final - umbral_settling;
        
        % Buscar desde atrás hacia adelante
        en_banda = (y_roll_long >= banda_inferior) & (y_roll_long <= banda_superior);
        idx_settling = find(~en_banda, 1, 'last');
        
        if ~isempty(idx_settling) && idx_settling < length(t_roll_long)
            tiempo_settling = t_roll_long(idx_settling + 1);
        else
            tiempo_settling = 0; % Ya está en banda desde el inicio
        end
        
        fprintf('  Métricas manuales:\n');
        fprintf('    Valor final: %.6f\n', valor_final);
        fprintf('    Pico: %.6f (en t=%.3fs)\n', pico, tiempo_pico);
        fprintf('    Sobrepaso: %.2f%%\n', sobrepaso);
        fprintf('    Tiempo de subida: %.3f s\n', tiempo_subida);
        fprintf('    Tiempo de establecimiento (2%%): %.3f s\n', tiempo_settling);
    end
end

% Para Yaw - simulación más larga
[y_yaw_long, t_yaw_long] = step(T_LD(4,4), t_long);

fprintf('\nCanal Yaw (ψ):\n');
fprintf('  Valor inicial: %.6f\n', y_yaw_long(1));
fprintf('  Valor a los 10s: %.6f\n', y_yaw_long(find(t_yaw_long >= 10, 1)));
fprintf('  Valor a los 20s: %.6f\n', y_yaw_long(find(t_yaw_long >= 20, 1)));
fprintf('  Valor final (30s): %.6f\n', y_yaw_long(end));
fprintf('  Valor máximo: %.6f\n', max(y_yaw_long));
fprintf('  Valor mínimo: %.6f\n', min(y_yaw_long));

% Verificar si el sistema es estable
if abs(y_yaw_long(end)) > 1e10 || isnan(y_yaw_long(end)) || isinf(y_yaw_long(end))
    fprintf('  ⚠️ ADVERTENCIA: Sistema posiblemente inestable o con problemas numéricos\n');
else
    % Intentar calcular stepinfo
    try
        % Método 1: stepinfo directo
        S_yaw = stepinfo(T_LD(4,4), 'SettlingTimeThreshold', 0.02);
        
        fprintf('\n  Métricas calculadas:\n');
        fprintf('    Sobrepaso: %.2f%%\n', S_yaw.Overshoot);
        fprintf('    Tiempo de establecimiento: %.3f s\n', S_yaw.SettlingTime);
        fprintf('    Tiempo de subida: %.3f s\n', S_yaw.RiseTime);
        fprintf('    Tiempo de pico: %.3f s\n', S_yaw.PeakTime);
        
    catch ME
        fprintf('  ⚠️ Error en stepinfo automático: %s\n', ME.message);
        fprintf('  Calculando métricas manualmente...\n\n');
        
        % Cálculo manual
        valor_final = y_yaw_long(end);
        
        % 1. Sobrepaso
        [pico, idx_pico] = max(y_yaw_long);
        sobrepaso = ((pico - valor_final) / abs(valor_final)) * 100;
        tiempo_pico = t_yaw_long(idx_pico);
        
        % 2. Tiempo de subida (10% a 90%)
        umbral_10 = 0.1 * valor_final;
        umbral_90 = 0.9 * valor_final;
        
        idx_10 = find(y_yaw_long >= umbral_10, 1, 'first');
        idx_90 = find(y_yaw_long >= umbral_90, 1, 'first');
        
        if ~isempty(idx_10) && ~isempty(idx_90)
            tiempo_subida = t_yaw_long(idx_90) - t_yaw_long(idx_10);
        else
            tiempo_subida = NaN;
        end
        
        % 3. Tiempo de establecimiento (2%)
        umbral_settling = 0.02 * abs(valor_final);
        banda_superior = valor_final + umbral_settling;
        banda_inferior = valor_final - umbral_settling;
        
        % Buscar desde atrás hacia adelante
        en_banda = (y_yaw_long >= banda_inferior) & (y_yaw_long <= banda_superior);
        idx_settling = find(~en_banda, 1, 'last');
        
        if ~isempty(idx_settling) && idx_settling < length(t_yaw_long)
            tiempo_settling = t_yaw_long(idx_settling + 1);
        else
            tiempo_settling = 0; % Ya está en banda desde el inicio
        end
        
        fprintf('  Métricas manuales:\n');
        fprintf('    Valor final: %.6f\n', valor_final);
        fprintf('    Pico: %.6f (en t=%.3fs)\n', pico, tiempo_pico);
        fprintf('    Sobrepaso: %.2f%%\n', sobrepaso);
        fprintf('    Tiempo de subida: %.3f s\n', tiempo_subida);
        fprintf('    Tiempo de establecimiento (2%%): %.3f s\n', tiempo_settling);
    end
end

fprintf('\n=== FIN DIAGNÓSTICO ===\n\n');

%% ========================================================================
% SEGUIMIENTO DE REFERENCIA CON RUIDO Y PERTURBACIÓN
% ========================================================================

fprintf('Simulando seguimiento con ruido y perturbación...\n');

% Parámetros de simulación
t_track = 0:0.01:15;
dt = 0.01;

% Señal de referencia (escalón en Roll y Yaw)
ref_roll = ones(size(t_track));
ref_yaw = ones(size(t_track));

% Ruido de medición
ruido_amplitud = 0.0001;
ruido = ruido_amplitud * randn(4, length(t_track));

% Perturbación (escalón en t=5s)
perturbacion = zeros(2, length(t_track));
perturbacion(:, t_track >= 5) = 0.1;

% Sistema en lazo cerrado con controlador
sys_CL = feedback(G_LD * K_LD, eye(4));

% Simulación con lsim
u_ref = [zeros(1,length(t_track)); zeros(1,length(t_track)); 
         ref_roll; ref_yaw];  % 4 referencias
         
[y_sim, t_sim] = lsim(T_LD, u_ref, t_track);

% Añadir ruido a la salida
y_roll_noise = y_sim(:,3) + ruido(3,:)';
y_yaw_noise = y_sim(:,4) + ruido(4,:)';

% Figura: Seguimiento Roll con ruido y perturbación
figure(8)
plot(t_sim, ref_roll, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_sim, y_roll_noise, 'b', 'LineWidth', 1);
plot([5 5], [0 1.5], 'k--', 'LineWidth', 1);
text(5.2, 0.8, 'Perturbación', 'FontSize', 9);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Roll Angle φ (rad)', 'FontSize', 11);
title('Seguimiento de Referencia Roll con Ruido y Perturbación', 'FontSize', 12);
legend('Referencia', 'Salida con ruido', 'Location', 'southeast');
xlim([0 15]);
hold off;

% Figura: Seguimiento Yaw con ruido y perturbación
figure(9)
plot(t_sim, ref_yaw, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_sim, y_yaw_noise, 'b', 'LineWidth', 1);
plot([5 5], [0 1.5], 'k--', 'LineWidth', 1);
text(5.2, 0.8, 'Perturbación', 'FontSize', 9);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Yaw Angle ψ (rad)', 'FontSize', 11);
title('Seguimiento de Referencia Yaw con Ruido y Perturbación', 'FontSize', 12);
legend('Referencia', 'Salida con ruido', 'Location', 'southeast');
xlim([0 15]);
hold off;

%% ========================================================================
% SEGUIMIENTO CON INCERTIDUMBRE ADITIVA INVERSA
% ========================================================================

fprintf('Simulando con incertidumbre aditiva inversa...\n');

% Crear incertidumbre aditiva (±10% en la planta)
delta_percent = 0.1;
G_LD_unc_pos = G_LD * (1 + delta_percent);
G_LD_unc_neg = G_LD * (1 - delta_percent);

% Sistemas en lazo cerrado con incertidumbre
T_LD_unc_pos = feedback(G_LD_unc_pos * K_LD, eye(4));
T_LD_unc_neg = feedback(G_LD_unc_neg * K_LD, eye(4));

% Simulación
[y_nominal, ~] = lsim(T_LD, u_ref, t_track);
[y_unc_pos, ~] = lsim(T_LD_unc_pos, u_ref, t_track);
[y_unc_neg, ~] = lsim(T_LD_unc_neg, u_ref, t_track);

% Figura: Roll con incertidumbre
figure(10)
plot(t_track, ref_roll, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_track, y_nominal(:,3), 'b', 'LineWidth', 2);
plot(t_track, y_unc_pos(:,3), 'g--', 'LineWidth', 1);
plot(t_track, y_unc_neg(:,3), 'm--', 'LineWidth', 1);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Roll Angle φ (rad)', 'FontSize', 11);
title('Seguimiento Roll con Incertidumbre Aditiva (±10%)', 'FontSize', 12);
legend('Referencia', 'Nominal', '+10% incertidumbre', '-10% incertidumbre', ...
       'Location', 'southeast');
xlim([0 10]);
hold off;

% Figura: Yaw con incertidumbre
figure(11)
plot(t_track, ref_yaw, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_track, y_nominal(:,4), 'b', 'LineWidth', 2);
plot(t_track, y_unc_pos(:,4), 'g--', 'LineWidth', 1);
plot(t_track, y_unc_neg(:,4), 'm--', 'LineWidth', 1);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Yaw Angle ψ (rad)', 'FontSize', 11);
title('Seguimiento Yaw con Incertidumbre Aditiva (±10%)', 'FontSize', 12);
legend('Referencia', 'Nominal', '+10% incertidumbre', '-10% incertidumbre', ...
       'Location', 'southeast');
xlim([0 10]);
hold off;

%% ========================================================================
% SEGUIMIENTO DE REFERENCIA CUADRADA CON RUIDO (ESTILO EJEMPLO)
% ========================================================================

fprintf('Generando seguimiento de referencia cuadrada...\n');

% Parámetros de simulación
t_square = 0:0.01:100;
dt = 0.01;

% Generar señal de referencia cuadrada
freq_square = 0.05; % 0.05 Hz (periodo de 20s)
ref_square_roll = 40 * square(2*pi*freq_square*t_square);
ref_square_yaw = 40 * square(2*pi*freq_square*t_square);

% Ruido de medición (más visible para comparar con el ejemplo)
ruido_nivel = 3; % Amplitud del ruido
ruido_roll = ruido_nivel * randn(1, length(t_square));
ruido_yaw = ruido_nivel * randn(1, length(t_square));

% Crear entrada de 4 canales para lsim
u_square = [zeros(1,length(t_square)); 
            zeros(1,length(t_square)); 
            ref_square_roll; 
            ref_square_yaw];

% Simulación
[y_square, t_square_out] = lsim(T_LD, u_square, t_square);

% Extraer Roll y Yaw con ruido
y_roll_square = y_square(:,3) + ruido_roll';
y_yaw_square = y_square(:,4) + ruido_yaw';

% Figura: Seguimiento Roll con referencia cuadrada y ruido
figure(12)
plot(t_square_out, y_roll_square, 'b', 'LineWidth', 1.5);
hold on;
plot(t_square_out, ref_square_roll, 'r', 'LineWidth', 2);
grid on;
xlabel('Time(s)', 'FontSize', 12);
ylabel('Roll Angle (deg)', 'FontSize', 12);
title('LateralModel - Roll Tracking', 'FontSize', 13);
legend('Salida', 'Referencia', 'Location', 'northeast');
xlim([0 100]);
ylim([-50 50]);
set(gca, 'FontSize', 11);
hold off;

% Figura: Seguimiento Yaw con referencia cuadrada y ruido
figure(13)
plot(t_square_out, y_yaw_square, 'b', 'LineWidth', 1.5);
hold on;
plot(t_square_out, ref_square_yaw, 'r', 'LineWidth', 2);
grid on;
xlabel('Time(s)', 'FontSize', 12);
ylabel('Yaw Angle (deg)', 'FontSize', 12);
title('LateralModel - Yaw Tracking', 'FontSize', 13);
legend('Salida', 'Referencia', 'Location', 'northeast');
xlim([0 100]);
ylim([-50 50]);
set(gca, 'FontSize', 11);
hold off;

%% ========================================================================
% ANÁLISIS DE SOBREPICO
% ========================================================================

fprintf('\n========================================\n');
fprintf('ANÁLISIS DE SOBREPICO\n');
fprintf('========================================\n\n');

% Calcular sobrepico teórico permitido
M_permitido = 1.8;  % ACTUALIZADO
sobrepico_max_permitido = (M_permitido - 1) * 100;

fprintf('Sobrepico máximo permitido por W1: %.1f%% (M=%.1f)\n', sobrepico_max_permitido, M_permitido);

% Obtener información de sobrepico de la sección anterior
if exist('S_roll', 'var')
    fprintf('Sobrepico real Roll: %.2f%%\n', S_roll.Overshoot);
    if S_roll.Overshoot <= sobrepico_max_permitido
        fprintf('  ✓ Roll cumple especificación de sobrepico\n');
    else
        fprintf('  ⚠️ Roll excede especificación de sobrepico\n');
    end
else
    fprintf('Sobrepico real Roll: Calculado manualmente en sección anterior\n');
end

if exist('S_yaw', 'var')
    fprintf('Sobrepico real Yaw: %.2f%%\n', S_yaw.Overshoot);
    if S_yaw.Overshoot <= sobrepico_max_permitido
        fprintf('  ✓ Yaw cumple especificación de sobrepico\n\n');
    else
        fprintf('  ⚠️ Yaw excede especificación de sobrepico\n\n');
    end
else
    fprintf('Sobrepico real Yaw: Calculado manualmente en sección anterior\n\n');
end

% Calcular factor de amortiguamiento
polos_CL_roll = pole(T_LD(3,3));
polos_dominantes_roll = polos_CL_roll(abs(imag(polos_CL_roll)) > 0);

if ~isempty(polos_dominantes_roll)
    wn_roll = abs(polos_dominantes_roll(1));
    zeta_roll = -real(polos_dominantes_roll(1)) / wn_roll;
    fprintf('Polo dominante Roll:\n');
    fprintf('  Factor de amortiguamiento ζ = %.3f\n', zeta_roll);
    fprintf('  Frecuencia natural ωn = %.3f rad/s\n', wn_roll);
    fprintf('  Frecuencia natural fn = %.3f Hz\n\n', wn_roll/(2*pi));
    
    if zeta_roll > 1
        fprintf('  → Sistema SOBREAMORTIGUADO (ζ > 1)\n');
        fprintf('     Sin sobrepico, respuesta lenta\n\n');
    elseif zeta_roll >= 0.7
        fprintf('  → Sistema CRÍTICAMENTE AMORTIGUADO (0.7 ≤ ζ ≤ 1)\n');
        fprintf('     Bajo sobrepico, buena velocidad\n\n');
    else
        fprintf('  → Sistema SUBAMORTIGUADO (ζ < 0.7)\n');
        fprintf('     Mayor sobrepico, respuesta rápida\n\n');
    end
end

polos_CL_yaw = pole(T_LD(4,4));
polos_dominantes_yaw = polos_CL_yaw(abs(imag(polos_CL_yaw)) > 0);

if ~isempty(polos_dominantes_yaw)
    wn_yaw = abs(polos_dominantes_yaw(1));
    zeta_yaw = -real(polos_dominantes_yaw(1)) / wn_yaw;
    fprintf('Polo dominante Yaw:\n');
    fprintf('  Factor de amortiguamiento ζ = %.3f\n', zeta_yaw);
    fprintf('  Frecuencia natural ωn = %.3f rad/s\n', wn_yaw);
    fprintf('  Frecuencia natural fn = %.3f Hz\n\n', wn_yaw/(2*pi));
    
    if zeta_yaw > 1
        fprintf('  → Sistema SOBREAMORTIGUADO (ζ > 1)\n');
        fprintf('     Sin sobrepico, respuesta lenta\n\n');
    elseif zeta_yaw >= 0.7
        fprintf('  → Sistema CRÍTICAMENTE AMORTIGUADO (0.7 ≤ ζ ≤ 1)\n');
        fprintf('     Bajo sobrepico, buena velocidad\n\n');
    else
        fprintf('  → Sistema SUBAMORTIGUADO (ζ < 0.7)\n');
        fprintf('     Mayor sobrepico, respuesta rápida\n\n');
    end
end

%% ========================================================================
% VALORES SINGULARES DEL CONTROLADOR
% ========================================================================

figure(14)
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
% ANÁLISIS DE ESTABILIDAD DE POLOS
% ========================================================================

fprintf('========================================\n');
fprintf('ANÁLISIS DE POLOS DEL SISTEMA\n');
fprintf('========================================\n\n');

% Polos del sistema en lazo abierto
polos_OL = pole(G_LD);
fprintf('Polos del sistema en lazo abierto:\n');
for i = 1:length(polos_OL)
    if imag(polos_OL(i)) == 0
        fprintf('  Polo %d: %.4f (real)\n', i, real(polos_OL(i)));
    else
        fprintf('  Polo %d: %.4f ± %.4fj\n', i, real(polos_OL(i)), abs(imag(polos_OL(i))));
    end
end

% Verificar estabilidad en lazo abierto
if all(real(polos_OL) < 0)
    fprintf('\n✓ Sistema en lazo abierto: ESTABLE\n\n');
else
    fprintf('\n⚠️ Sistema en lazo abierto: INESTABLE\n\n');
end

% Polos del sistema en lazo cerrado
polos_CL = pole(T_LD);
fprintf('Polos del sistema en lazo cerrado (primeros 10):\n');
num_polos_mostrar = min(10, length(polos_CL));
for i = 1:num_polos_mostrar
    if imag(polos_CL(i)) == 0
        fprintf('  Polo %d: %.4f (real)\n', i, real(polos_CL(i)));
    else
        fprintf('  Polo %d: %.4f ± %.4fj\n', i, real(polos_CL(i)), abs(imag(polos_CL(i))));
    end
end

% Verificar estabilidad en lazo cerrado
if all(real(polos_CL) < 0)
    fprintf('\n✓ Sistema en lazo cerrado: ESTABLE\n');
    fprintf('  Todos los polos tienen parte real negativa\n\n');
else
    fprintf('\n⚠️ Sistema en lazo cerrado: INESTABLE\n');
    fprintf('  Hay polos con parte real positiva o nula\n\n');
end

% Margen de estabilidad
fprintf('Margen de estabilidad (polo más cercano al eje imaginario):\n');
margen_estabilidad = min(abs(real(polos_CL)));
fprintf('  Distancia mínima al eje imaginario: %.4f\n\n', margen_estabilidad);

%% ========================================================================
% GRÁFICA DE POLOS Y CEROS
% ========================================================================

figure(15)
pzmap(T_LD(3,3));
title('Mapa de Polos y Ceros - Roll (φ) - MEJORADO');
grid on;

figure(16)
pzmap(T_LD(4,4));
title('Mapa de Polos y Ceros - Yaw (ψ) - MEJORADO');
grid on;

%% ========================================================================
% RESPUESTA EN FRECUENCIA (DIAGRAMAS DE BODE)
% ========================================================================

figure(17)
bode(T_LD(3,3));
title('Diagrama de Bode - Roll (φ)');
grid on;

figure(18)
bode(T_LD(4,4));
title('Diagrama de Bode - Yaw (ψ) ');
grid on;

%% ========================================================================
% ANÁLISIS DE ROBUSTEZ - MÁRGENES DE GANANCIA Y FASE
% ========================================================================

fprintf('========================================\n');
fprintf('MÁRGENES DE ESTABILIDAD\n');
fprintf('========================================\n\n');

% Para Roll
[Gm_roll, Pm_roll, Wcg_roll, Wcp_roll] = margin(L_LD(3,3));

fprintf('Canal Roll (φ):\n');
if isinf(Gm_roll)
    fprintf('  Margen de Ganancia: ∞ dB\n');
else
    fprintf('  Margen de Ganancia: %.2f (%.2f dB) en ω = %.2f rad/s\n', ...
        Gm_roll, 20*log10(Gm_roll), Wcg_roll);
end
fprintf('  Margen de Fase: %.2f° en ω = %.2f rad/s\n\n', Pm_roll, Wcp_roll);

% Para Yaw
[Gm_yaw, Pm_yaw, Wcg_yaw, Wcp_yaw] = margin(L_LD(4,4));

fprintf('Canal Yaw (ψ):\n');
if isinf(Gm_yaw)
    fprintf('  Margen de Ganancia: ∞ dB\n');
else
    fprintf('  Margen de Ganancia: %.2f (%.2f dB) en ω = %.2f rad/s\n', ...
        Gm_yaw, 20*log10(Gm_yaw), Wcg_yaw);
end
fprintf('  Margen de Fase: %.2f° en ω = %.2f rad/s\n\n', Pm_yaw, Wcp_yaw);

% Criterios de robustez
fprintf('Criterios de robustez recomendados:\n');
fprintf('  Margen de Ganancia: > 6 dB\n');
fprintf('  Margen de Fase: > 45°\n\n');

if (isinf(Gm_roll) || 20*log10(Gm_roll) > 6) && Pm_roll > 45
    fprintf('✓ Canal Roll cumple criterios de robustez\n');
else
    fprintf('⚠️ Canal Roll NO cumple todos los criterios de robustez\n');
end

if (isinf(Gm_yaw) || 20*log10(Gm_yaw) > 6) && Pm_yaw > 45
    fprintf('✓ Canal Yaw cumple criterios de robustez\n\n');
else
    fprintf('⚠️ Canal Yaw NO cumple todos los criterios de robustez\n\n');
end


fprintf('\n========================================\n');
fprintf('ANÁLISIS DE NORMAS H∞ CON SIGMA\n');
fprintf('========================================\n\n');

%% PARTE 1: VERIFICACIÓN W1 - DESEMPEÑO (||S|| < ||W1^-1||)

fprintf('VERIFICACIÓN W1 - DESEMPEÑO:\n');
fprintf('Condición: ||S||∞ < ||W1^-1||∞\n\n');

% Calcular máximo valor singular de S - CORREGIDO
[sv_S, w_S] = sigma(S_LD);

% Para sistemas MIMO, sv_S puede ser matriz (frecuencias x canales)
% Necesitamos el máximo sobre TODAS las frecuencias y canales
if size(sv_S, 1) > 1
    % sv_S tiene múltiples filas (un valor singular por frecuencia)
    max_sv_S = max(sv_S(:,1));  % Primera columna = máximo valor singular
    [~, idx_max_S] = max(sv_S(:,1));
    freq_max_S = w_S(idx_max_S);
else
    % sv_S es un vector
    max_sv_S = max(sv_S);
    [~, idx_max_S] = max(sv_S);
    freq_max_S = w_S(idx_max_S);
end

% Calcular máximo valor singular de W1^-1 - CORREGIDO
[sv_W1inv, w_W1inv] = sigma(inv(W1_LD));

if size(sv_W1inv, 1) > 1
    max_sv_W1inv = max(sv_W1inv(:,1));
    [~, idx_max_W1] = max(sv_W1inv(:,1));
    freq_max_W1inv = w_W1inv(idx_max_W1);
else
    max_sv_W1inv = max(sv_W1inv);
    [~, idx_max_W1] = max(sv_W1inv);
    freq_max_W1inv = w_W1inv(idx_max_W1);
end

% Comparación
fprintf('  ||S||∞ (máximo valor singular):\n');
fprintf('    Valor: %.4f\n', max_sv_S);
fprintf('    Frecuencia: %.4f rad/s (%.2f Hz)\n', freq_max_S, freq_max_S/(2*pi));

fprintf('\n  ||W1^-1||∞ (máximo valor singular):\n');
fprintf('    Valor: %.4f\n', max_sv_W1inv);
fprintf('    Frecuencia: %.4f rad/s (%.2f Hz)\n', freq_max_W1inv, freq_max_W1inv/(2*pi));

fprintf('\n  Ratio ||S||∞ / ||W1^-1||∞ = %.4f\n', max_sv_S / max_sv_W1inv);
fprintf('  Margen: %.4f (%.2f%%)\n', max_sv_W1inv - max_sv_S, ...
    100*(max_sv_W1inv - max_sv_S)/max_sv_W1inv);

if max_sv_S < max_sv_W1inv
    fprintf('  ✅ CUMPLE: S está por debajo de W1^-1\n');
    fprintf('     → Buen seguimiento de referencia garantizado\n\n');
else
    fprintf('  ❌ NO CUMPLE: S excede W1^-1 en %.4f\n', max_sv_S - max_sv_W1inv);
    fprintf('     → Aumentar W1 para mejorar desempeño\n\n');
end

% Gráfica mejorada con sigma
figure(100);
sigma(S_LD, 'b', inv(W1_LD), 'r--', {0.01, 1000});
grid on;
title('Verificación W1: Desempeño (usando SIGMA)', 'FontSize', 13);
legend('σ(S) - Sensibilidad', 'σ(W_1^{-1}) - Límite de desempeño', ...
    'Location', 'best', 'FontSize', 11);
xlabel('Frecuencia (rad/s)', 'FontSize', 11);
ylabel('Valor Singular (dB)', 'FontSize', 11);

% Marcar punto crítico
hold on;
semilogx(freq_max_S, 20*log10(max_sv_S), 'bo', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'b', 'LineWidth', 2);
semilogx(freq_max_W1inv, 20*log10(max_sv_W1inv), 'ro', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'r', 'LineWidth', 2);
text(freq_max_S, 20*log10(max_sv_S)+2, sprintf('  ||S||∞=%.2f', max_sv_S), ...
    'FontSize', 9, 'Color', 'b');
text(freq_max_W1inv, 20*log10(max_sv_W1inv)+2, sprintf('  ||W_1^{-1}||∞=%.2f', max_sv_W1inv), ...
    'FontSize', 9, 'Color', 'r');
hold off;

%% PARTE 2: VERIFICACIÓN W2 - ESFUERZO DE CONTROL (||KS|| < ||W2^-1||)

fprintf('VERIFICACIÓN W2 - ESFUERZO DE CONTROL:\n');
fprintf('Condición: ||KS||∞ < ||W2^-1||∞\n\n');

% Calcular máximo valor singular de KS - CORREGIDO
[sv_KS, w_KS] = sigma(KS_LD);

if size(sv_KS, 1) > 1
    max_sv_KS = max(sv_KS(:,1));
    [~, idx_max_KS] = max(sv_KS(:,1));
    freq_max_KS = w_KS(idx_max_KS);
else
    max_sv_KS = max(sv_KS);
    [~, idx_max_KS] = max(sv_KS);
    freq_max_KS = w_KS(idx_max_KS);
end

% W2 es escalar, su inversa también
max_sv_W2inv = 1/0.8;  % W2 = 0.8

fprintf('  ||KS||∞ (máximo valor singular):\n');
fprintf('    Valor: %.4f\n', max_sv_KS);
fprintf('    Frecuencia: %.4f rad/s (%.2f Hz)\n', freq_max_KS, freq_max_KS/(2*pi));

fprintf('\n  ||W2^-1||∞:\n');
fprintf('    Valor: %.4f\n', max_sv_W2inv);

fprintf('\n  Ratio ||KS||∞ / ||W2^-1||∞ = %.4f\n', max_sv_KS / max_sv_W2inv);
fprintf('  Margen: %.4f (%.2f%%)\n', max_sv_W2inv - max_sv_KS, ...
    100*(max_sv_W2inv - max_sv_KS)/max_sv_W2inv);

if max_sv_KS < max_sv_W2inv
    fprintf('  ✅ CUMPLE: KS está por debajo de W2^-1\n');
    fprintf('     → Esfuerzo de control dentro de límites\n\n');
else
    fprintf('  ❌ NO CUMPLE: KS excede W2^-1 en %.4f\n', max_sv_KS - max_sv_W2inv);
    fprintf('     → Reducir W2 para permitir más esfuerzo\n\n');
end

% Gráfica mejorada con sigma
figure(101);
sigma(KS_LD, 'b', {0.01, 1000});
hold on;
semilogx([0.01 1000], 20*log10([max_sv_W2inv max_sv_W2inv]), 'r--', 'LineWidth', 2);
grid on;
title('Verificación W2: Esfuerzo de Control (usando SIGMA)', 'FontSize', 13);
legend('σ(KS) - Esfuerzo de control', 'σ(W_2^{-1}) - Límite', ...
    'Location', 'best', 'FontSize', 11);
xlabel('Frecuencia (rad/s)', 'FontSize', 11);
ylabel('Valor Singular (dB)', 'FontSize', 11);

% Marcar punto crítico
semilogx(freq_max_KS, 20*log10(max_sv_KS), 'bo', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'b', 'LineWidth', 2);
text(freq_max_KS, 20*log10(max_sv_KS)+2, sprintf('  ||KS||∞=%.2f', max_sv_KS), ...
    'FontSize', 9, 'Color', 'b');
hold off;

%% PARTE 3: VERIFICACIÓN W3 - ROBUSTEZ (||T|| < ||W3^-1||)

fprintf('VERIFICACIÓN W3 - ROBUSTEZ:\n');
fprintf('Condición: ||T||∞ < ||W3^-1||∞\n\n');

% Calcular máximo valor singular de T - CORREGIDO
[sv_T, w_T] = sigma(T_LD);

if size(sv_T, 1) > 1
    max_sv_T = max(sv_T(:,1));
    [~, idx_max_T] = max(sv_T(:,1));
    freq_max_T = w_T(idx_max_T);
else
    max_sv_T = max(sv_T);
    [~, idx_max_T] = max(sv_T);
    freq_max_T = w_T(idx_max_T);
end

% Calcular máximo valor singular de W3^-1 - CORREGIDO
[sv_W3inv, w_W3inv] = sigma(1/W3_LD);

if size(sv_W3inv, 1) > 1
    max_sv_W3inv = max(sv_W3inv(:,1));
    [~, idx_max_W3] = max(sv_W3inv(:,1));
    freq_max_W3inv = w_W3inv(idx_max_W3);
else
    max_sv_W3inv = max(sv_W3inv);
    [~, idx_max_W3] = max(sv_W3inv);
    freq_max_W3inv = w_W3inv(idx_max_W3);
end

fprintf('  ||T||∞ (máximo valor singular):\n');
fprintf('    Valor: %.4f\n', max_sv_T);
fprintf('    Frecuencia: %.4f rad/s (%.2f Hz)\n', freq_max_T, freq_max_T/(2*pi));

fprintf('\n  ||W3^-1||∞ (máximo valor singular):\n');
fprintf('    Valor: %.4f\n', max_sv_W3inv);
fprintf('    Frecuencia: %.4f rad/s (%.2f Hz)\n', freq_max_W3inv, freq_max_W3inv/(2*pi));

fprintf('\n  Ratio ||T||∞ / ||W3^-1||∞ = %.4f\n', max_sv_T / max_sv_W3inv);
fprintf('  Margen: %.4f (%.2f%%)\n', max_sv_W3inv - max_sv_T, ...
    100*(max_sv_W3inv - max_sv_T)/max_sv_W3inv);

if max_sv_T < max_sv_W3inv
    fprintf('  ✅ CUMPLE: T está por debajo de W3^-1\n');
    fprintf('     → Buen rechazo a ruido de alta frecuencia\n\n');
else
    fprintf('  ❌ NO CUMPLE: T excede W3^-1 en %.4f\n', max_sv_T - max_sv_W3inv);
    fprintf('     → Ajustar W3 para mejorar robustez\n\n');
end

% Gráfica mejorada con sigma
figure(102);
sigma(T_LD, 'b', 1/W3_LD, 'r--', {0.01, 10000});
grid on;
title('Verificación W3: Robustez (usando SIGMA)', 'FontSize', 13);
legend('σ(T) - Sensibilidad complementaria', 'σ(W_3^{-1}) - Límite de robustez', ...
    'Location', 'best', 'FontSize', 11);
xlabel('Frecuencia (rad/s)', 'FontSize', 11);
ylabel('Valor Singular (dB)', 'FontSize', 11);

% Marcar punto crítico
hold on;
semilogx(freq_max_T, 20*log10(max_sv_T), 'bo', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'b', 'LineWidth', 2);
semilogx(freq_max_W3inv, 20*log10(max_sv_W3inv), 'ro', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'r', 'LineWidth', 2);
text(freq_max_T, 20*log10(max_sv_T)+2, sprintf('  ||T||∞=%.2f', max_sv_T), ...
    'FontSize', 9, 'Color', 'b');
text(freq_max_W3inv, 20*log10(max_sv_W3inv)+2, sprintf('  ||W_3^{-1}||∞=%.2f', max_sv_W3inv), ...
    'FontSize', 9, 'Color', 'r');
hold off;

%% PARTE 4: RESUMEN CON SIGMA

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║     RESUMEN FINAL - NORMAS H∞ (USANDO SIGMA)          ║\n');
fprintf('╠════════════════════════════════════════════════════════╣\n');

% Tabla comparativa
fprintf('║                                                        ║\n');
fprintf('║  Condición │  Calculado  │   Límite   │  Cumple?    ║\n');
fprintf('║────────────┼─────────────┼────────────┼─────────────║\n');
fprintf('║  ||S||∞    │   %6.4f    │  %6.4f    │     %s     ║\n', ...
    max_sv_S, max_sv_W1inv, iif(max_sv_S < max_sv_W1inv, '✅', '❌'));
fprintf('║  ||KS||∞   │   %6.4f    │  %6.4f    │     %s     ║\n', ...
    max_sv_KS, max_sv_W2inv, iif(max_sv_KS < max_sv_W2inv, '✅', '❌'));
fprintf('║  ||T||∞    │   %6.4f    │  %6.4f    │     %s     ║\n', ...
    max_sv_T, max_sv_W3inv, iif(max_sv_T < max_sv_W3inv, '✅', '❌'));
fprintf('║                                                        ║\n');
fprintf('╠════════════════════════════════════════════════════════╣\n');

% Conteo
cumple_sigma = [max_sv_S < max_sv_W1inv, max_sv_KS < max_sv_W2inv, max_sv_T < max_sv_W3inv];
total_cumple = sum(cumple_sigma);

fprintf('║  TOTAL CONDICIONES CUMPLIDAS: %d/3                     ║\n', total_cumple);
fprintf('║  γ H∞ obtenido: %.4f                                  ║\n', gamma_LD);
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

% Interpretación
fprintf('📊 INTERPRETACIÓN DE RESULTADOS:\n\n');

if max_sv_S < max_sv_W1inv
    fprintf('  ✅ DESEMPEÑO: Cumple con margen de %.2f%% \n', ...
        100*(max_sv_W1inv - max_sv_S)/max_sv_W1inv);
    fprintf('     → Error de seguimiento será pequeño en bajas frecuencias\n');
else
    fprintf('  ❌ DESEMPEÑO: Falla por %.4f\n', max_sv_S - max_sv_W1inv);
    fprintf('     → SOLUCIÓN: Aumentar pesos W1 (probar 3.0 o más)\n');
end

if max_sv_KS < max_sv_W2inv
    fprintf('  ✅ ESFUERZO: Cumple con margen de %.2f%%\n', ...
        100*(max_sv_W2inv - max_sv_KS)/max_sv_W2inv);
    fprintf('     → Esfuerzo de control es aceptable\n');
else
    fprintf('  ❌ ESFUERZO: Falla por %.4f\n', max_sv_KS - max_sv_W2inv);
    fprintf('     → SOLUCIÓN: Reducir W2 (probar 0.5 o menos)\n');
end

if max_sv_T < max_sv_W3inv
    fprintf('  ✅ ROBUSTEZ: Cumple con margen de %.2f%%\n', ...
        100*(max_sv_W3inv - max_sv_T)/max_sv_W3inv);
    fprintf('     → Buen rechazo a ruido de medición\n\n');
else
    fprintf('  ❌ ROBUSTEZ: Falla por %.4f\n', max_sv_T - max_sv_W3inv);
    fprintf('     → SOLUCIÓN: Ajustar frecuencia de corte W3\n\n');
end

% Guardar resultados
save('H_inf_lat_results_SIGMA.mat', 'K_LD', 'gamma_LD', 'S_LD', 'T_LD', 'KS_LD', ...
     'W1_LD', 'W2_LD', 'W3_LD', 'G_LD', 'L_LD', ...
     'max_sv_S', 'max_sv_KS', 'max_sv_T', ...
     'max_sv_W1inv', 'max_sv_W2inv', 'max_sv_W3inv');

fprintf('💾 Resultados guardados en: H_inf_lat_results_SIGMA.mat\n');
fprintf('✅ ANÁLISIS CON SIGMA COMPLETADO\n\n');



%% ========================================================================
% VERIFICACIÓN DE ESPECIFICACIONES DE DISEÑO
% ========================================================================

fprintf('\n════════════════════════════════════════════════════════\n');
fprintf('VERIFICACIÓN DE ESPECIFICACIONES DE DISEÑO\n');
fprintf('════════════════════════════════════════════════════════\n\n');

%% ESPECIFICACIÓN 1: ANCHO DE BANDA (ωBW,max = 8 Hz)
fprintf('📌 ESPECIFICACIÓN 1: ANCHO DE BANDA MÍNIMO\n');
fprintf('   Requisito: ωBW,max ≥ 8 Hz (50.27 rad/s)\n\n');

omega_BW_req = 8 * 2 * pi;  % 8 Hz en rad/s

% Calcular ancho de banda del sistema en lazo cerrado
% BW se define donde |T(jω)| cae -3dB del valor DC

% Para Roll (φ)
[mag_roll, ~, wout_roll] = bode(T_LD(3,3));
mag_roll = squeeze(mag_roll);
wout_roll = squeeze(wout_roll);

% Valor DC (ganancia a baja frecuencia)
mag_dc_roll = mag_roll(1);
mag_3dB_roll = mag_dc_roll / sqrt(2);  % -3dB

% Encontrar frecuencia de cruce -3dB
idx_bw_roll = find(mag_roll <= mag_3dB_roll, 1, 'first');
if ~isempty(idx_bw_roll)
    BW_roll = wout_roll(idx_bw_roll);
    BW_roll_Hz = BW_roll / (2*pi);
else
    BW_roll = wout_roll(end);
    BW_roll_Hz = BW_roll / (2*pi);
end

fprintf('   Canal ROLL (φ):\n');
fprintf('     Ancho de banda: %.2f rad/s (%.2f Hz)\n', BW_roll, BW_roll_Hz);
if BW_roll_Hz >= 8
    fprintf('     ✅ CUMPLE (%.2f Hz ≥ 8 Hz)\n', BW_roll_Hz);
else
    fprintf('     ❌ NO CUMPLE (%.2f Hz < 8 Hz)\n', BW_roll_Hz);
    fprintf('     → Falta: %.2f Hz\n', 8 - BW_roll_Hz);
end

% Para Yaw (ψ)
[mag_yaw, ~, wout_yaw] = bode(T_LD(4,4));
mag_yaw = squeeze(mag_yaw);
wout_yaw = squeeze(wout_yaw);

mag_dc_yaw = mag_yaw(1);
mag_3dB_yaw = mag_dc_yaw / sqrt(2);

idx_bw_yaw = find(mag_yaw <= mag_3dB_yaw, 1, 'first');
if ~isempty(idx_bw_yaw)
    BW_yaw = wout_yaw(idx_bw_yaw);
    BW_yaw_Hz = BW_yaw / (2*pi);
else
    BW_yaw = wout_yaw(end);
    BW_yaw_Hz = BW_yaw / (2*pi);
end

fprintf('\n   Canal YAW (ψ):\n');
fprintf('     Ancho de banda: %.2f rad/s (%.2f Hz)\n', BW_yaw, BW_yaw_Hz);
if BW_yaw_Hz >= 8
    fprintf('     ✅ CUMPLE (%.2f Hz ≥ 8 Hz)\n\n', BW_yaw_Hz);
else
    fprintf('     ❌ NO CUMPLE (%.2f Hz < 8 Hz)\n', BW_yaw_Hz);
    fprintf('     → Falta: %.2f Hz\n\n', 8 - BW_yaw_Hz);
end

% Resultado general Spec 1
spec1_cumple = (BW_roll_Hz >= 8) && (BW_yaw_Hz >= 8);

%% ESPECIFICACIÓN 2: ROBUSTEZ ANTE INCERTIDUMBRE ADITIVA (ωp,max = 6 Hz)
fprintf('📌 ESPECIFICACIÓN 2: ROBUSTEZ ANTE INCERTIDUMBRE ADITIVA\n');
fprintf('   Requisito: Sistema robusto hasta ωp,max = 6 Hz (37.70 rad/s)\n');
fprintf('   Condición: ||T(jω)||∞ debe ser pequeño para ω ≤ 37.70 rad/s\n\n');

omega_p_max = 6 * 2 * pi;  % 6 Hz en rad/s

% Evaluar |T| en el rango [0, 6 Hz]
omega_range = logspace(-2, log10(omega_p_max), 200);
[mag_T_roll_range, ~] = bode(T_LD(3,3), omega_range);
[mag_T_yaw_range, ~] = bode(T_LD(4,4), omega_range);

mag_T_roll_range = squeeze(mag_T_roll_range);
mag_T_yaw_range = squeeze(mag_T_yaw_range);

% Máximo de |T| en el rango de perturbaciones
max_T_roll_pert = max(mag_T_roll_range);
max_T_yaw_pert = max(mag_T_yaw_range);

fprintf('   Canal ROLL:\n');
fprintf('     ||T||∞ en [0, 6 Hz]: %.4f (%.2f dB)\n', max_T_roll_pert, 20*log10(max_T_roll_pert));
if max_T_roll_pert < 1.5  % Criterio: debe ser moderado
    fprintf('     ✅ CUMPLE (||T|| < 1.5 → buena atenuación de perturbaciones)\n');
else
    fprintf('     ⚠️  ADVERTENCIA (||T|| > 1.5 → puede amplificar perturbaciones)\n');
end

fprintf('\n   Canal YAW:\n');
fprintf('     ||T||∞ en [0, 6 Hz]: %.4f (%.2f dB)\n', max_T_yaw_pert, 20*log10(max_T_yaw_pert));
if max_T_yaw_pert < 1.5
    fprintf('     ✅ CUMPLE (||T|| < 1.5 → buena atenuación de perturbaciones)\n\n');
else
    fprintf('     ⚠️  ADVERTENCIA (||T|| > 1.5 → puede amplificar perturbaciones)\n\n');
end

spec2_cumple = (max_T_roll_pert < 1.5) && (max_T_yaw_pert < 1.5);

%% ESPECIFICACIÓN 3: RECHAZO A RUIDO DE MEDICIÓN
fprintf('📌 ESPECIFICACIÓN 3: RECHAZO A RUIDO DE MEDICIÓN\n');
fprintf('   Lateral/Direccional: Potencia 0.001\n');
fprintf('   Longitudinal: Potencia 0.0001\n');
fprintf('   Condición: ||T||∞ debe ser pequeño en altas frecuencias\n\n');

% Para rechazo a ruido, necesitamos que T → 0 en altas frecuencias
% Evaluamos |T| en alta frecuencia (> 100 rad/s)

omega_high = logspace(2, 4, 100);  % 100 a 10000 rad/s
[mag_T_roll_high, ~] = bode(T_LD(3,3), omega_high);
[mag_T_yaw_high, ~] = bode(T_LD(4,4), omega_high);

mag_T_roll_high = squeeze(mag_T_roll_high);
mag_T_yaw_high = squeeze(mag_T_yaw_high);

% Promedio en alta frecuencia
avg_T_roll_high = mean(mag_T_roll_high);
avg_T_yaw_high = mean(mag_T_yaw_high);

fprintf('   Canal ROLL:\n');
fprintf('     |T| promedio en alta freq: %.4f (%.2f dB)\n', avg_T_roll_high, 20*log10(avg_T_roll_high));
if avg_T_roll_high < 0.1  % Buena atenuación de ruido
    fprintf('     ✅ EXCELENTE rechazo a ruido (< -20 dB)\n');
elseif avg_T_roll_high < 0.5
    fprintf('     ✅ BUENO rechazo a ruido (< -6 dB)\n');
else
    fprintf('     ⚠️  REGULAR rechazo a ruido\n');
end

fprintf('\n   Canal YAW:\n');
fprintf('     |T| promedio en alta freq: %.4f (%.2f dB)\n', avg_T_yaw_high, 20*log10(avg_T_yaw_high));
if avg_T_yaw_high < 0.1
    fprintf('     ✅ EXCELENTE rechazo a ruido (< -20 dB)\n\n');
elseif avg_T_yaw_high < 0.5
    fprintf('     ✅ BUENO rechazo a ruido (< -6 dB)\n\n');
else
    fprintf('     ⚠️  REGULAR rechazo a ruido\n\n');
end

spec3_cumple = (avg_T_roll_high < 0.5) && (avg_T_yaw_high < 0.5);

%% ESPECIFICACIÓN 4: ESFUERZO DE CONTROL (≤ ±30°)
fprintf('📌 ESPECIFICACIÓN 4: LÍMITE DE ESFUERZO DE CONTROL\n');
fprintf('   Requisito: Señal de control ≤ ±30° para entrada escalón unitario\n\n');

% Simular señal de control para escalón unitario
t_control = 0:0.01:10;
u_escalon = [zeros(1,length(t_control)); zeros(1,length(t_control)); 
             ones(1,length(t_control)); ones(1,length(t_control))];

% Señal de control: u = K * e = K * (r - y) = K * S * r
CS_LD = K_LD * S_LD;

% Simular para Roll (entrada en canal 3)
u_ref_roll = zeros(4, length(t_control));
u_ref_roll(3,:) = ones(1, length(t_control));  % Escalón en φ

[u_roll, ~] = lsim(CS_LD, u_ref_roll, t_control);

% Señal de control para alerones (salida 1 del controlador)
u_aileron_roll = u_roll(:,1);
u_aileron_max_roll = max(abs(u_aileron_roll));

fprintf('   Canal ROLL (φ → δa alerones):\n');
fprintf('     Esfuerzo máximo: %.2f°\n', u_aileron_max_roll);
if u_aileron_max_roll <= 30
    fprintf('     ✅ CUMPLE (%.2f° ≤ 30°)\n', u_aileron_max_roll);
else
    fprintf('     ❌ NO CUMPLE (%.2f° > 30°)\n', u_aileron_max_roll);
    fprintf('     → Exceso: %.2f°\n', u_aileron_max_roll - 30);
end

% Simular para Yaw (entrada en canal 4)
u_ref_yaw = zeros(4, length(t_control));
u_ref_yaw(4,:) = ones(1, length(t_control));  % Escalón en ψ

[u_yaw, ~] = lsim(CS_LD, u_ref_yaw, t_control);

% Señal de control para timón (salida 2 del controlador)
u_rudder_yaw = u_yaw(:,2);
u_rudder_max_yaw = max(abs(u_rudder_yaw));

fprintf('\n   Canal YAW (ψ → δr timón):\n');
fprintf('     Esfuerzo máximo: %.2f°\n', u_rudder_max_yaw);
if u_rudder_max_yaw <= 30
    fprintf('     ✅ CUMPLE (%.2f° ≤ 30°)\n\n', u_rudder_max_yaw);
else
    fprintf('     ❌ NO CUMPLE (%.2f° > 30°)\n', u_rudder_max_yaw);
    fprintf('     → Exceso: %.2f°\n\n', u_rudder_max_yaw - 30);
end

spec4_cumple = (u_aileron_max_roll <= 30) && (u_rudder_max_yaw <= 30);

%% GRÁFICA DE ESFUERZO DE CONTROL
figure(103);
subplot(2,1,1);
plot(t_control, u_aileron_roll, 'b', 'LineWidth', 1.5);
hold on;
plot([0 10], [30 30], 'r--', 'LineWidth', 1.5);
plot([0 10], [-30 -30], 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('δa - Alerones (°)', 'FontSize', 11);
title('Esfuerzo de Control - Roll (Alerones)', 'FontSize', 12);
legend('Señal de control', 'Límite ±30°', 'Location', 'best');
ylim([min(-35, min(u_aileron_roll)-5), max(35, max(u_aileron_roll)+5)]);

subplot(2,1,2);
plot(t_control, u_rudder_yaw, 'b', 'LineWidth', 1.5);
hold on;
plot([0 10], [30 30], 'r--', 'LineWidth', 1.5);
plot([0 10], [-30 -30], 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('δr - Timón (°)', 'FontSize', 11);
title('Esfuerzo de Control - Yaw (Timón)', 'FontSize', 12);
legend('Señal de control', 'Límite ±30°', 'Location', 'best');
ylim([min(-35, min(u_rudder_yaw)-5), max(35, max(u_rudder_yaw)+5)]);

%% RESUMEN FINAL DE ESPECIFICACIONES
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║          RESUMEN DE ESPECIFICACIONES DE DISEÑO                ║\n');
fprintf('╠════════════════════════════════════════════════════════════════╣\n');
fprintf('║                                                                ║\n');
fprintf('║  Nº │ Especificación                    │ Estado               ║\n');
fprintf('║ ───┼───────────────────────────────────┼───────────────────── ║\n');
fprintf('║  1 │ Ancho de banda ≥ 8 Hz             │ %s                ║\n', ...
    iif(spec1_cumple, '✅ CUMPLE      ', '❌ NO CUMPLE   '));
fprintf('║  2 │ Robustez ante perturbaciones      │ %s                ║\n', ...
    iif(spec2_cumple, '✅ CUMPLE      ', '⚠️  ADVERTENCIA'));
fprintf('║  3 │ Rechazo a ruido de medición       │ %s                ║\n', ...
    iif(spec3_cumple, '✅ CUMPLE      ', '⚠️  REGULAR    '));
fprintf('║  4 │ Esfuerzo de control ≤ ±30°        │ %s                ║\n', ...
    iif(spec4_cumple, '✅ CUMPLE      ', '❌ NO CUMPLE   '));
fprintf('║                                                                ║\n');
fprintf('╠════════════════════════════════════════════════════════════════╣\n');

total_specs = sum([spec1_cumple, spec2_cumple, spec3_cumple, spec4_cumple]);
fprintf('║  TOTAL: %d/4 especificaciones cumplidas                        ║\n', total_specs);
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% RECOMENDACIONES
fprintf('💡 RECOMENDACIONES PARA MEJORAR EL DISEÑO:\n\n');

if ~spec1_cumple
    fprintf('  ❌ ANCHO DE BANDA:\n');
    fprintf('     → Aumentar W1 (probar M = 2.0 o mayor)\n');
    fprintf('     → Aumentar frecuencia de corte Wb1 (probar 12-15 Hz)\n\n');
end

if ~spec2_cumple
    fprintf('  ⚠️  ROBUSTEZ:\n');
    fprintf('     → Ajustar W3 para mejor roll-off en medias frecuencias\n');
    fprintf('     → Considerar aumentar M3 ligeramente\n\n');
end

if ~spec3_cumple
    fprintf('  ⚠️  RECHAZO A RUIDO:\n');
    fprintf('     → Aumentar frecuencia de corte de W3 (> 250 Hz)\n');
    fprintf('     → Reducir M3 para mayor atenuación en alta frecuencia\n\n');
end

if ~spec4_cumple
    fprintf('  ❌ ESFUERZO DE CONTROL:\n');
    fprintf('     → CRÍTICO: Reducir W2 significativamente (probar 0.3-0.5)\n');
    fprintf('     → Reducir pesos de W1 si es necesario\n');
    fprintf('     → Considerar saturación en implementación real\n\n');
end

fprintf('✅ Verificación de especificaciones completada.\n');
fprintf('   Nueva figura: 103 (Esfuerzo de control)\n\n');

% Función auxiliar
function out = iif(cond, true_val, false_val)
    if cond, out = true_val; else, out = false_val; end
end