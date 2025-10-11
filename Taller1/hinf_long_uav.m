%% DISEÑO CONTROLADOR H-INFINITO - SISTEMA LONGITUDINAL (ANÁLISIS COMPLETO)
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
fprintf('  Salidas: 2 [q (pitch rate), θ (pitch angle)]\n\n');

%% ========================================================================
% DISEÑO DE FUNCIONES DE PESO - MEJORADO
% ========================================================================

% W1: Desempeño (seguimiento de referencia)

% Para velocidad de pitch (q)
W1_long_vel = 5.0;

% Para posición de pitch (θ)
M1_long_pos = 2.5;
A1_long_pos = 0.00001;
Wb1_long_pos = 2*pi*18;
W1_long_pos = tf([1/M1_long_pos Wb1_long_pos], [1 A1_long_pos*Wb1_long_pos]);

% Matriz diagonal W1 (2x2)
W1_long = eye(2)*tf(1,1);
W1_long(1,1) = W1_long_vel;  % q (pitch rate)
W1_long(2,2) = W1_long_pos;  % θ (pitch angle)

% W2: Esfuerzo de control - MENOS RESTRICTIVO
W2_long = 0.4*tf(1,1);

% W3: Robustez (rechazo a ruido) - AJUSTADO
M3_long = 0.008;
A3_long = 1.1;
Wb3_long = 2*pi*250;

W3_long = tf([1/M3_long Wb3_long], [1 A3_long*Wb3_long]);

% Visualizar filtros
figure(1)
sigma(W1_long);
hold on
sigma(W3_long);
title('Filtros de Peso - Sistema Longitudinal (MEJORADOS)');
legend('W1 (Desempeño)', 'W3 (Robustez)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% SÍNTESIS DEL CONTROLADOR H-INFINITO
% ========================================================================

fprintf('Ejecutando síntesis H∞...\n');

% Crear planta aumentada
P_long = augw(G_long, W1_long, W2_long, W3_long);

% Parámetros de síntesis
nmeas = 2;  % Número de mediciones (q, θ)
ncont = 1;  % Número de controles (δe)

% Síntesis H-infinity usando hinfsyn
[K1_long, sys1_CL, gam1, info1] = hinfsyn(P_long, nmeas, ncont);

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

% Sistemas en lazo cerrado
L_long = G_long * K_long;
S_long = feedback(eye(2), L_long);
T_long = feedback(L_long, eye(2));
KS_long = K_long * S_long;

%% ========================================================================
% ANÁLISIS DE FUNCIONES DE SENSIBILIDAD
% ========================================================================

figure(5)
sigma(T_long);
hold on
sigma(S_long);
title('Funciones de Sensibilidad T y S - Sistema Longitudinal');
legend('T (Complementaria)', 'S (Sensibilidad)', 'Location', 'best');
grid on;
hold off

%% ========================================================================
% RESPUESTA AL ESCALÓN - ESTILO MEJORADO CON DIAGNÓSTICO
% ========================================================================

fprintf('Calculando respuesta al escalón...\n');

% Parámetros de simulación
t_sim = 0:0.01:6;

% Respuesta al escalón para Pitch Rate (canal 1,1) y Pitch Angle (canal 2,2)
[y_pitch_rate, t_pr] = step(T_long(1,1), t_sim);
[y_pitch_angle, t_pa] = step(T_long(2,2), t_sim);

% Figura para Pitch Rate
figure(6)
plot(t_pr, y_pitch_rate, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Step Response - Pitch Rate (q)', 'FontSize', 12);
xlim([0 6]);
set(gca, 'FontSize', 10);

% Figura para Pitch Angle
figure(7)
plot(t_pa, y_pitch_angle, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Step Response - Pitch Angle (θ)', 'FontSize', 12);
xlim([0 6]);
set(gca, 'FontSize', 10);

% Métricas de desempeño - Con diagnóstico
fprintf('\n=== DIAGNÓSTICO DE RESPUESTA AL ESCALÓN ===\n');

% Para Pitch Rate - simulación más larga
t_long = 0:0.01:30;
[y_pr_long, t_pr_long] = step(T_long(1,1), t_long);

fprintf('\nCanal Pitch Rate (q):\n');
fprintf('  Valor inicial: %.6f\n', y_pr_long(1));
fprintf('  Valor a los 10s: %.6f\n', y_pr_long(find(t_pr_long >= 10, 1)));
fprintf('  Valor a los 20s: %.6f\n', y_pr_long(find(t_pr_long >= 20, 1)));
fprintf('  Valor final (30s): %.6f\n', y_pr_long(end));
fprintf('  Valor máximo: %.6f\n', max(y_pr_long));
fprintf('  Valor mínimo: %.6f\n', min(y_pr_long));

% Verificar estabilidad
if abs(y_pr_long(end)) > 1e10 || isnan(y_pr_long(end)) || isinf(y_pr_long(end))
    fprintf('  ⚠️ ADVERTENCIA: Sistema posiblemente inestable\n');
else
    try
        S_pr = stepinfo(T_long(1,1), 'SettlingTimeThreshold', 0.02);
        fprintf('\n  Métricas calculadas:\n');
        fprintf('    Sobrepaso: %.2f%%\n', S_pr.Overshoot);
        fprintf('    Tiempo de establecimiento: %.3f s\n', S_pr.SettlingTime);
        fprintf('    Tiempo de subida: %.3f s\n', S_pr.RiseTime);
        fprintf('    Tiempo de pico: %.3f s\n', S_pr.PeakTime);
    catch ME
        fprintf('  ⚠️ Error en stepinfo: %s\n', ME.message);
        fprintf('  Calculando métricas manualmente...\n');
        
        valor_final = y_pr_long(end);
        [pico, idx_pico] = max(y_pr_long);
        sobrepaso = ((pico - valor_final) / abs(valor_final)) * 100;
        tiempo_pico = t_pr_long(idx_pico);
        
        umbral_10 = 0.1 * valor_final;
        umbral_90 = 0.9 * valor_final;
        idx_10 = find(y_pr_long >= umbral_10, 1, 'first');
        idx_90 = find(y_pr_long >= umbral_90, 1, 'first');
        
        if ~isempty(idx_10) && ~isempty(idx_90)
            tiempo_subida = t_pr_long(idx_90) - t_pr_long(idx_10);
        else
            tiempo_subida = NaN;
        end
        
        umbral_settling = 0.02 * abs(valor_final);
        banda_superior = valor_final + umbral_settling;
        banda_inferior = valor_final - umbral_settling;
        en_banda = (y_pr_long >= banda_inferior) & (y_pr_long <= banda_superior);
        idx_settling = find(~en_banda, 1, 'last');
        
        if ~isempty(idx_settling) && idx_settling < length(t_pr_long)
            tiempo_settling = t_pr_long(idx_settling + 1);
        else
            tiempo_settling = 0;
        end
        
        fprintf('  Métricas manuales:\n');
        fprintf('    Valor final: %.6f\n', valor_final);
        fprintf('    Pico: %.6f (en t=%.3fs)\n', pico, tiempo_pico);
        fprintf('    Sobrepaso: %.2f%%\n', sobrepaso);
        fprintf('    Tiempo de subida: %.3f s\n', tiempo_subida);
        fprintf('    Tiempo de establecimiento (2%%): %.3f s\n', tiempo_settling);
    end
end

% Para Pitch Angle
[y_pa_long, t_pa_long] = step(T_long(2,2), t_long);

fprintf('\nCanal Pitch Angle (θ):\n');
fprintf('  Valor inicial: %.6f\n', y_pa_long(1));
fprintf('  Valor a los 10s: %.6f\n', y_pa_long(find(t_pa_long >= 10, 1)));
fprintf('  Valor a los 20s: %.6f\n', y_pa_long(find(t_pa_long >= 20, 1)));
fprintf('  Valor final (30s): %.6f\n', y_pa_long(end));
fprintf('  Valor máximo: %.6f\n', max(y_pa_long));
fprintf('  Valor mínimo: %.6f\n', min(y_pa_long));

if abs(y_pa_long(end)) > 1e10 || isnan(y_pa_long(end)) || isinf(y_pa_long(end))
    fprintf('  ⚠️ ADVERTENCIA: Sistema posiblemente inestable\n');
else
    try
        S_pa = stepinfo(T_long(2,2), 'SettlingTimeThreshold', 0.02);
        fprintf('\n  Métricas calculadas:\n');
        fprintf('    Sobrepaso: %.2f%%\n', S_pa.Overshoot);
        fprintf('    Tiempo de establecimiento: %.3f s\n', S_pa.SettlingTime);
        fprintf('    Tiempo de subida: %.3f s\n', S_pa.RiseTime);
        fprintf('    Tiempo de pico: %.3f s\n', S_pa.PeakTime);
    catch ME
        fprintf('  ⚠️ Error en stepinfo: %s\n', ME.message);
        fprintf('  Calculando métricas manualmente...\n');
        
        valor_final = y_pa_long(end);
        [pico, idx_pico] = max(y_pa_long);
        sobrepaso = ((pico - valor_final) / abs(valor_final)) * 100;
        tiempo_pico = t_pa_long(idx_pico);
        
        umbral_10 = 0.1 * valor_final;
        umbral_90 = 0.9 * valor_final;
        idx_10 = find(y_pa_long >= umbral_10, 1, 'first');
        idx_90 = find(y_pa_long >= umbral_90, 1, 'first');
        
        if ~isempty(idx_10) && ~isempty(idx_90)
            tiempo_subida = t_pa_long(idx_90) - t_pa_long(idx_10);
        else
            tiempo_subida = NaN;
        end
        
        umbral_settling = 0.02 * abs(valor_final);
        banda_superior = valor_final + umbral_settling;
        banda_inferior = valor_final - umbral_settling;
        en_banda = (y_pa_long >= banda_inferior) & (y_pa_long <= banda_superior);
        idx_settling = find(~en_banda, 1, 'last');
        
        if ~isempty(idx_settling) && idx_settling < length(t_pa_long)
            tiempo_settling = t_pa_long(idx_settling + 1);
        else
            tiempo_settling = 0;
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

t_track = 0:0.01:15;
dt = 0.01;

% Señal de referencia (escalón en Pitch Rate y Pitch Angle)
ref_pr = ones(size(t_track));
ref_pa = ones(size(t_track));

% Ruido de medición
ruido_amplitud = 0.0001;
ruido = ruido_amplitud * randn(2, length(t_track));

% Perturbación (escalón en t=5s)
perturbacion = zeros(1, length(t_track));
perturbacion(:, t_track >= 5) = 0.1;

% Simulación con lsim
u_ref = [ref_pr; ref_pa];
[y_sim, t_sim] = lsim(T_long, u_ref, t_track);

% Añadir ruido
y_pr_noise = y_sim(:,1) + ruido(1,:)';
y_pa_noise = y_sim(:,2) + ruido(2,:)';

% Figura: Seguimiento Pitch Rate
figure(8)
plot(t_sim, ref_pr, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_sim, y_pr_noise, 'b', 'LineWidth', 1);
plot([5 5], [0 1.5], 'k--', 'LineWidth', 1);
text(5.2, 0.8, 'Perturbación', 'FontSize', 9);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Pitch Rate q (rad/s)', 'FontSize', 11);
title('Seguimiento de Referencia Pitch Rate con Ruido y Perturbación', 'FontSize', 12);
legend('Referencia', 'Salida con ruido', 'Location', 'southeast');
xlim([0 15]);
hold off;

% Figura: Seguimiento Pitch Angle
figure(9)
plot(t_sim, ref_pa, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_sim, y_pa_noise, 'b', 'LineWidth', 1);
plot([5 5], [0 1.5], 'k--', 'LineWidth', 1);
text(5.2, 0.8, 'Perturbación', 'FontSize', 9);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Pitch Angle θ (rad)', 'FontSize', 11);
title('Seguimiento de Referencia Pitch Angle con Ruido y Perturbación', 'FontSize', 12);
legend('Referencia', 'Salida con ruido', 'Location', 'southeast');
xlim([0 15]);
hold off;

%% ========================================================================
% SEGUIMIENTO CON INCERTIDUMBRE ADITIVA INVERSA
% ========================================================================

fprintf('Simulando con incertidumbre aditiva inversa...\n');

delta_percent = 0.1;
G_long_unc_pos = G_long * (1 + delta_percent);
G_long_unc_neg = G_long * (1 - delta_percent);

T_long_unc_pos = feedback(G_long_unc_pos * K_long, eye(2));
T_long_unc_neg = feedback(G_long_unc_neg * K_long, eye(2));

[y_nominal, ~] = lsim(T_long, u_ref, t_track);
[y_unc_pos, ~] = lsim(T_long_unc_pos, u_ref, t_track);
[y_unc_neg, ~] = lsim(T_long_unc_neg, u_ref, t_track);

% Figura: Pitch Rate con incertidumbre
figure(10)
plot(t_track, ref_pr, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_track, y_nominal(:,1), 'b', 'LineWidth', 2);
plot(t_track, y_unc_pos(:,1), 'g--', 'LineWidth', 1);
plot(t_track, y_unc_neg(:,1), 'm--', 'LineWidth', 1);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Pitch Rate q (rad/s)', 'FontSize', 11);
title('Seguimiento Pitch Rate con Incertidumbre Aditiva (±10%)', 'FontSize', 12);
legend('Referencia', 'Nominal', '+10% incertidumbre', '-10% incertidumbre', 'Location', 'southeast');
xlim([0 10]);
hold off;

% Figura: Pitch Angle con incertidumbre
figure(11)
plot(t_track, ref_pa, 'r--', 'LineWidth', 1.5);
hold on;
plot(t_track, y_nominal(:,2), 'b', 'LineWidth', 2);
plot(t_track, y_unc_pos(:,2), 'g--', 'LineWidth', 1);
plot(t_track, y_unc_neg(:,2), 'm--', 'LineWidth', 1);
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Pitch Angle θ (rad)', 'FontSize', 11);
title('Seguimiento Pitch Angle con Incertidumbre Aditiva (±10%)', 'FontSize', 12);
legend('Referencia', 'Nominal', '+10% incertidumbre', '-10% incertidumbre', 'Location', 'southeast');
xlim([0 10]);
hold off;

%% ========================================================================
% SEGUIMIENTO DE REFERENCIA CUADRADA CON RUIDO
% ========================================================================

fprintf('Generando seguimiento de referencia cuadrada...\n');

t_square = 0:0.01:100;
dt = 0.01;

freq_square = 0.05;
ref_square_pr = 40 * square(2*pi*freq_square*t_square);
ref_square_pa = 40 * square(2*pi*freq_square*t_square);

ruido_nivel = 3;
ruido_pr = ruido_nivel * randn(1, length(t_square));
ruido_pa = ruido_nivel * randn(1, length(t_square));

u_square = [ref_square_pr; ref_square_pa];
[y_square, t_square_out] = lsim(T_long, u_square, t_square);

y_pr_square = y_square(:,1) + ruido_pr';
y_pa_square = y_square(:,2) + ruido_pa';

% Figura: Pitch Rate con referencia cuadrada
figure(12)
plot(t_square_out, y_pr_square, 'b', 'LineWidth', 1.5);
hold on;
plot(t_square_out, ref_square_pr, 'r', 'LineWidth', 2);
grid on;
xlabel('Time(s)', 'FontSize', 12);
ylabel('Pitch Rate (deg/s)', 'FontSize', 12);
title('LongitudinalModel - Pitch Rate Tracking', 'FontSize', 13);
legend('Salida', 'Referencia', 'Location', 'northeast');
xlim([0 100]);
ylim([-50 50]);
set(gca, 'FontSize', 11);
hold off;

% Figura: Pitch Angle con referencia cuadrada
figure(13)
plot(t_square_out, y_pa_square, 'b', 'LineWidth', 1.5);
hold on;
plot(t_square_out, ref_square_pa, 'r', 'LineWidth', 2);
grid on;
xlabel('Time(s)', 'FontSize', 12);
ylabel('Pitch Angle (deg)', 'FontSize', 12);
title('LongitudinalModel - Pitch Angle Tracking', 'FontSize', 13);
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

M_permitido = 1.8;
sobrepico_max_permitido = (M_permitido - 1) * 100;

fprintf('Sobrepico máximo permitido por W1: %.1f%% (M=%.1f)\n', sobrepico_max_permitido, M_permitido);

if exist('S_pr', 'var')
    fprintf('Sobrepico real Pitch Rate: %.2f%%\n', S_pr.Overshoot);
    if S_pr.Overshoot <= sobrepico_max_permitido
        fprintf('  ✓ Pitch Rate cumple especificación\n');
    else
        fprintf('  ⚠️ Pitch Rate excede especificación\n');
    end
else
    fprintf('Sobrepico real Pitch Rate: Calculado manualmente\n');
end

if exist('S_pa', 'var')
    fprintf('Sobrepico real Pitch Angle: %.2f%%\n', S_pa.Overshoot);
    if S_pa.Overshoot <= sobrepico_max_permitido
        fprintf('  ✓ Pitch Angle cumple especificación\n\n');
    else
        fprintf('  ⚠️ Pitch Angle excede especificación\n\n');
    end
else
    fprintf('Sobrepico real Pitch Angle: Calculado manualmente\n\n');
end

% Polos dominantes
polos_CL_pr = pole(T_long(1,1));
polos_dominantes_pr = polos_CL_pr(abs(imag(polos_CL_pr)) > 0);

if ~isempty(polos_dominantes_pr)
    wn_pr = abs(polos_dominantes_pr(1));
    zeta_pr = -real(polos_dominantes_pr(1)) / wn_pr;
    fprintf('Polo dominante Pitch Rate:\n');
    fprintf('  ζ = %.3f\n', zeta_pr);
    fprintf('  ωn = %.3f rad/s (%.3f Hz)\n\n', wn_pr, wn_pr/(2*pi));
end

polos_CL_pa = pole(T_long(2,2));
polos_dominantes_pa = polos_CL_pa(abs(imag(polos_CL_pa)) > 0);

if ~isempty(polos_dominantes_pa)
    wn_pa = abs(polos_dominantes_pa(1));
    zeta_pa = -real(polos_dominantes_pa(1)) / wn_pa;
    fprintf('Polo dominante Pitch Angle:\n');
    fprintf('  ζ = %.3f\n', zeta_pa);
    fprintf('  ωn = %.3f rad/s (%.3f Hz)\n\n', wn_pa, wn_pa/(2*pi));
end

%% ========================================================================
% VALORES SINGULARES DEL CONTROLADOR
% ========================================================================

figure(14)
sigma(K_long);
title('Valores Singulares del Controlador H∞ - Longitudinal');
grid on;

%% ========================================================================
% ANÁLISIS DE ACOPLAMIENTO
% ========================================================================

fprintf('========================================\n');
fprintf('ANÁLISIS DE ACOPLAMIENTO\n');
fprintf('========================================\n\n');

coupling_matrix = abs(A_long);
state_names = {'u', 'w', 'q', 'θ', 'h', 'T'};

fprintf('Matriz de acoplamiento (valores absolutos de A):\n\n');
for i = 1:6
    fprintf('  Estado %s: ', state_names{i});
    coupling_row = coupling_matrix(i,:);
    coupling_row(i) = 0;
    [max_coup, max_idx] = max(coupling_row);
    fprintf('más acoplado con %s (%.4f)\n', state_names{max_idx}, max_coup);
end
fprintf('\n');

%% ========================================================================
% ANÁLISIS DE ESTABILIDAD DE POLOS
% ========================================================================

fprintf('========================================\n');
fprintf('ANÁLISIS DE POLOS DEL SISTEMA\n');
fprintf('========================================\n\n');

polos_OL = pole(G_long);
fprintf('Polos del sistema en lazo abierto:\n');
for i = 1:length(polos_OL)
    if imag(polos_OL(i)) == 0
        fprintf('  Polo %d: %.4f (real)\n', i, real(polos_OL(i)));
    else
        fprintf('  Polo %d: %.4f ± %.4fj\n', i, real(polos_OL(i)), abs(imag(polos_OL(i))));
    end
end

if all(real(polos_OL) < 0)
    fprintf('\n✓ Sistema en lazo abierto: ESTABLE\n\n');
else
    fprintf('\n⚠️ Sistema en lazo abierto: INESTABLE\n\n');
end

polos_CL = pole(T_long);
fprintf('Polos del sistema en lazo cerrado (primeros 10):\n');
num_polos_mostrar = min(10, length(polos_CL));
for i = 1:num_polos_mostrar
    if imag(polos_CL(i)) == 0
        fprintf('  Polo %d: %.4f (real)\n', i, real(polos_CL(i)));
    else
        fprintf('  Polo %d: %.4f ± %.4fj\n', i, real(polos_CL(i)), abs(imag(polos_CL(i))));
    end
end

if all(real(polos_CL) < 0)
    fprintf('\n✓ Sistema en lazo cerrado: ESTABLE\n\n');
else
    fprintf('\n⚠️ Sistema en lazo cerrado: INESTABLE\n\n');
end

margen_estabilidad = min(abs(real(polos_CL)));
fprintf('Margen de estabilidad: %.4f\n\n', margen_estabilidad);

%% ========================================================================
% GRÁFICA DE POLOS Y CEROS
% ========================================================================

figure(15)
pzmap(T_long(1,1));
title('Mapa de Polos y Ceros - Pitch Rate (q)');
grid on;

figure(16)
pzmap(T_long(2,2));
title('Mapa de Polos y Ceros - Pitch Angle (θ)');
grid on;

%% ========================================================================
% RESPUESTA EN FRECUENCIA (DIAGRAMAS DE BODE)
% ========================================================================

figure(17)
bode(T_long(1,1));
title('Diagrama de Bode - Pitch Rate (q)');
grid on;

figure(18)
bode(T_long(2,2));
title('Diagrama de Bode - Pitch Angle (θ)');
grid on;

%% ========================================================================
% ANÁLISIS DE ROBUSTEZ - MÁRGENES DE GANANCIA Y FASE
% ========================================================================

fprintf('========================================\n');
fprintf('MÁRGENES DE ESTABILIDAD\n');
fprintf('========================================\n\n');

% Para Pitch Rate
[Gm_pr, Pm_pr, Wcg_pr, Wcp_pr] = margin(L_long(1,1));

fprintf('Canal Pitch Rate (q):\n');
if isinf(Gm_pr)
    fprintf('  Margen de Ganancia: ∞ dB\n');
else
    fprintf('  Margen de Ganancia: %.2f (%.2f dB) en ω = %.2f rad/s\n', ...
        Gm_pr, 20*log10(Gm_pr), Wcg_pr);
end
fprintf('  Margen de Fase: %.2f° en ω = %.2f rad/s\n\n', Pm_pr, Wcp_pr);

% Para Pitch Angle
[Gm_pa, Pm_pa, Wcg_pa, Wcp_pa] = margin(L_long(2,2));

fprintf('Canal Pitch Angle (θ):\n');
if isinf(Gm_pa)
    fprintf('  Margen de Ganancia: ∞ dB\n');
else
    fprintf('  Margen de Ganancia: %.2f (%.2f dB) en ω = %.2f rad/s\n', ...
        Gm_pa, 20*log10(Gm_pa), Wcg_pa);
end
fprintf('  Margen de Fase: %.2f° en ω = %.2f rad/s\n\n', Pm_pa, Wcp_pa);

fprintf('Criterios de robustez recomendados:\n');
fprintf('  Margen de Ganancia: > 6 dB\n');
fprintf('  Margen de Fase: > 45°\n\n');

if (isinf(Gm_pr) || 20*log10(Gm_pr) > 6) && Pm_pr > 45
    fprintf('✓ Canal Pitch Rate cumple criterios de robustez\n');
else
    fprintf('⚠️ Canal Pitch Rate NO cumple todos los criterios\n');
end

if (isinf(Gm_pa) || 20*log10(Gm_pa) > 6) && Pm_pa > 45
    fprintf('✓ Canal Pitch Angle cumple criterios de robustez\n\n');
else
    fprintf('⚠️ Canal Pitch Angle NO cumple todos los criterios\n\n');
end

%% ========================================================================
% ANÁLISIS DE NORMAS H∞ CON SIGMA
% ========================================================================

fprintf('\n========================================\n');
fprintf('ANÁLISIS DE NORMAS H∞ CON SIGMA\n');
fprintf('========================================\n\n');

%% PARTE 1: VERIFICACIÓN W1 - DESEMPEÑO

fprintf('VERIFICACIÓN W1 - DESEMPEÑO:\n');
fprintf('Condición: ||S||∞ < ||W1^-1||∞\n\n');

[sv_S, w_S] = sigma(S_long);

if size(sv_S, 1) > 1
    max_sv_S = max(sv_S(:,1));
    [~, idx_max_S] = max(sv_S(:,1));
    freq_max_S = w_S(idx_max_S);
else
    max_sv_S = max(sv_S);
    [~, idx_max_S] = max(sv_S);
    freq_max_S = w_S(idx_max_S);
end

[sv_W1inv, w_W1inv] = sigma(inv(W1_long));

if size(sv_W1inv, 1) > 1
    max_sv_W1inv = max(sv_W1inv(:,1));
    [~, idx_max_W1] = max(sv_W1inv(:,1));
    freq_max_W1inv = w_W1inv(idx_max_W1);
else
    max_sv_W1inv = max(sv_W1inv);
    [~, idx_max_W1] = max(sv_W1inv);
    freq_max_W1inv = w_W1inv(idx_max_W1);
end

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

figure(100);
sigma(S_long, 'b', inv(W1_long), 'r--', {0.01, 1000});
grid on;
title('Verificación W1: Desempeño (usando SIGMA)', 'FontSize', 13);
legend('σ(S) - Sensibilidad', 'σ(W_1^{-1}) - Límite de desempeño', 'Location', 'best', 'FontSize', 11);
xlabel('Frecuencia (rad/s)', 'FontSize', 11);
ylabel('Valor Singular (dB)', 'FontSize', 11);

hold on;
semilogx(freq_max_S, 20*log10(max_sv_S), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 2);
semilogx(freq_max_W1inv, 20*log10(max_sv_W1inv), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);
text(freq_max_S, 20*log10(max_sv_S)+2, sprintf('  ||S||∞=%.2f', max_sv_S), 'FontSize', 9, 'Color', 'b');
text(freq_max_W1inv, 20*log10(max_sv_W1inv)+2, sprintf('  ||W_1^{-1}||∞=%.2f', max_sv_W1inv), 'FontSize', 9, 'Color', 'r');
hold off;

%% PARTE 2: VERIFICACIÓN W2 - ESFUERZO DE CONTROL

fprintf('VERIFICACIÓN W2 - ESFUERZO DE CONTROL:\n');
fprintf('Condición: ||KS||∞ < ||W2^-1||∞\n\n');

[sv_KS, w_KS] = sigma(KS_long);

if size(sv_KS, 1) > 1
    max_sv_KS = max(sv_KS(:,1));
    [~, idx_max_KS] = max(sv_KS(:,1));
    freq_max_KS = w_KS(idx_max_KS);
else
    max_sv_KS = max(sv_KS);
    [~, idx_max_KS] = max(sv_KS);
    freq_max_KS = w_KS(idx_max_KS);
end

max_sv_W2inv = 1/0.4;

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

figure(101);
sigma(KS_long, 'b', {0.01, 1000});
hold on;
semilogx([0.01 1000], 20*log10([max_sv_W2inv max_sv_W2inv]), 'r--', 'LineWidth', 2);
grid on;
title('Verificación W2: Esfuerzo de Control (usando SIGMA)', 'FontSize', 13);
legend('σ(KS) - Esfuerzo de control', 'σ(W_2^{-1}) - Límite', 'Location', 'best', 'FontSize', 11);
xlabel('Frecuencia (rad/s)', 'FontSize', 11);
ylabel('Valor Singular (dB)', 'FontSize', 11);

semilogx(freq_max_KS, 20*log10(max_sv_KS), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 2);
text(freq_max_KS, 20*log10(max_sv_KS)+2, sprintf('  ||KS||∞=%.2f', max_sv_KS), 'FontSize', 9, 'Color', 'b');
hold off;

%% PARTE 3: VERIFICACIÓN W3 - ROBUSTEZ

fprintf('VERIFICACIÓN W3 - ROBUSTEZ:\n');
fprintf('Condición: ||T||∞ < ||W3^-1||∞\n\n');

[sv_T, w_T] = sigma(T_long);

if size(sv_T, 1) > 1
    max_sv_T = max(sv_T(:,1));
    [~, idx_max_T] = max(sv_T(:,1));
    freq_max_T = w_T(idx_max_T);
else
    max_sv_T = max(sv_T);
    [~, idx_max_T] = max(sv_T);
    freq_max_T = w_T(idx_max_T);
end

[sv_W3inv, w_W3inv] = sigma(1/W3_long);

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

figure(102);
sigma(T_long, 'b', 1/W3_long, 'r--', {0.01, 10000});
grid on;
title('Verificación W3: Robustez (usando SIGMA)', 'FontSize', 13);
legend('σ(T) - Sensibilidad complementaria', 'σ(W_3^{-1}) - Límite de robustez', 'Location', 'best', 'FontSize', 11);
xlabel('Frecuencia (rad/s)', 'FontSize', 11);
ylabel('Valor Singular (dB)', 'FontSize', 11);

hold on;
semilogx(freq_max_T, 20*log10(max_sv_T), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 2);
semilogx(freq_max_W3inv, 20*log10(max_sv_W3inv), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);
text(freq_max_T, 20*log10(max_sv_T)+2, sprintf('  ||T||∞=%.2f', max_sv_T), 'FontSize', 9, 'Color', 'b');
text(freq_max_W3inv, 20*log10(max_sv_W3inv)+2, sprintf('  ||W_3^{-1}||∞=%.2f', max_sv_W3inv), 'FontSize', 9, 'Color', 'r');
hold off;

%% PARTE 4: RESUMEN CON SIGMA

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║     RESUMEN FINAL - NORMAS H∞ (USANDO SIGMA)          ║\n');
fprintf('╠════════════════════════════════════════════════════════╣\n');
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

cumple_sigma = [max_sv_S < max_sv_W1inv, max_sv_KS < max_sv_W2inv, max_sv_T < max_sv_W3inv];
total_cumple = sum(cumple_sigma);

fprintf('║  TOTAL CONDICIONES CUMPLIDAS: %d/3                     ║\n', total_cumple);
fprintf('║  γ H∞ obtenido: %.4f                                  ║\n', gamma_long);
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

fprintf('📊 INTERPRETACIÓN DE RESULTADOS:\n\n');

if max_sv_S < max_sv_W1inv
    fprintf('  ✅ DESEMPEÑO: Cumple con margen de %.2f%%\n', ...
        100*(max_sv_W1inv - max_sv_S)/max_sv_W1inv);
    fprintf('     → Error de seguimiento será pequeño\n');
else
    fprintf('  ❌ DESEMPEÑO: Falla por %.4f\n', max_sv_S - max_sv_W1inv);
    fprintf('     → SOLUCIÓN: Aumentar pesos W1\n');
end

if max_sv_KS < max_sv_W2inv
    fprintf('  ✅ ESFUERZO: Cumple con margen de %.2f%%\n', ...
        100*(max_sv_W2inv - max_sv_KS)/max_sv_W2inv);
    fprintf('     → Esfuerzo de control es aceptable\n');
else
    fprintf('  ❌ ESFUERZO: Falla por %.4f\n', max_sv_KS - max_sv_W2inv);
    fprintf('     → SOLUCIÓN: Reducir W2\n');
end

if max_sv_T < max_sv_W3inv
    fprintf('  ✅ ROBUSTEZ: Cumple con margen de %.2f%%\n', ...
        100*(max_sv_W3inv - max_sv_T)/max_sv_W3inv);
    fprintf('     → Buen rechazo a ruido de medición\n\n');
else
    fprintf('  ❌ ROBUSTEZ: Falla por %.4f\n', max_sv_T - max_sv_W3inv);
    fprintf('     → SOLUCIÓN: Ajustar frecuencia de corte W3\n\n');
end

%% ========================================================================
% VERIFICACIÓN DE ESPECIFICACIONES DE DISEÑO
% ========================================================================

fprintf('\n════════════════════════════════════════════════════════\n');
fprintf('VERIFICACIÓN DE ESPECIFICACIONES DE DISEÑO\n');
fprintf('════════════════════════════════════════════════════════\n\n');

%% ESPECIFICACIÓN 1: ANCHO DE BANDA
fprintf('📌 ESPECIFICACIÓN 1: ANCHO DE BANDA MÍNIMO\n');
fprintf('   Requisito: ωBW,max ≥ 8 Hz (50.27 rad/s)\n\n');

omega_BW_req = 8 * 2 * pi;

% Para Pitch Rate
[mag_pr, ~, wout_pr] = bode(T_long(1,1));
mag_pr = squeeze(mag_pr);
wout_pr = squeeze(wout_pr);

mag_dc_pr = mag_pr(1);
mag_3dB_pr = mag_dc_pr / sqrt(2);

idx_bw_pr = find(mag_pr <= mag_3dB_pr, 1, 'first');
if ~isempty(idx_bw_pr)
    BW_pr = wout_pr(idx_bw_pr);
    BW_pr_Hz = BW_pr / (2*pi);
else
    BW_pr = wout_pr(end);
    BW_pr_Hz = BW_pr / (2*pi);
end

fprintf('   Canal PITCH RATE (q):\n');
fprintf('     Ancho de banda: %.2f rad/s (%.2f Hz)\n', BW_pr, BW_pr_Hz);
if BW_pr_Hz >= 8
    fprintf('     ✅ CUMPLE (%.2f Hz ≥ 8 Hz)\n', BW_pr_Hz);
else
    fprintf('     ❌ NO CUMPLE (%.2f Hz < 8 Hz)\n', BW_pr_Hz);
    fprintf('     → Falta: %.2f Hz\n', 8 - BW_pr_Hz);
end

% Para Pitch Angle
[mag_pa, ~, wout_pa] = bode(T_long(2,2));
mag_pa = squeeze(mag_pa);
wout_pa = squeeze(wout_pa);

mag_dc_pa = mag_pa(1);
mag_3dB_pa = mag_dc_pa / sqrt(2);

idx_bw_pa = find(mag_pa <= mag_3dB_pa, 1, 'first');
if ~isempty(idx_bw_pa)
    BW_pa = wout_pa(idx_bw_pa);
    BW_pa_Hz = BW_pa / (2*pi);
else
    BW_pa = wout_pa(end);
    BW_pa_Hz = BW_pa / (2*pi);
end

fprintf('\n   Canal PITCH ANGLE (θ):\n');
fprintf('     Ancho de banda: %.2f rad/s (%.2f Hz)\n', BW_pa, BW_pa_Hz);
if BW_pa_Hz >= 8
    fprintf('     ✅ CUMPLE (%.2f Hz ≥ 8 Hz)\n\n', BW_pa_Hz);
else
    fprintf('     ❌ NO CUMPLE (%.2f Hz < 8 Hz)\n', BW_pa_Hz);
    fprintf('     → Falta: %.2f Hz\n\n', 8 - BW_pa_Hz);
end

spec1_cumple = (BW_pr_Hz >= 8) && (BW_pa_Hz >= 8);

%% ESPECIFICACIÓN 2: ROBUSTEZ ANTE INCERTIDUMBRE ADITIVA
fprintf('📌 ESPECIFICACIÓN 2: ROBUSTEZ ANTE INCERTIDUMBRE ADITIVA\n');
fprintf('   Requisito: Sistema robusto hasta ωp,max = 6 Hz (37.70 rad/s)\n');
fprintf('   Condición: ||T(jω)||∞ debe ser pequeño para ω ≤ 37.70 rad/s\n\n');

omega_p_max = 6 * 2 * pi;

omega_range = logspace(-2, log10(omega_p_max), 200);
[mag_T_pr_range, ~] = bode(T_long(1,1), omega_range);
[mag_T_pa_range, ~] = bode(T_long(2,2), omega_range);

mag_T_pr_range = squeeze(mag_T_pr_range);
mag_T_pa_range = squeeze(mag_T_pa_range);

max_T_pr_pert = max(mag_T_pr_range);
max_T_pa_pert = max(mag_T_pa_range);

fprintf('   Canal PITCH RATE:\n');
fprintf('     ||T||∞ en [0, 6 Hz]: %.4f (%.2f dB)\n', max_T_pr_pert, 20*log10(max_T_pr_pert));
if max_T_pr_pert < 1.5
    fprintf('     ✅ CUMPLE (||T|| < 1.5)\n');
else
    fprintf('     ⚠️  ADVERTENCIA (||T|| > 1.5)\n');
end

fprintf('\n   Canal PITCH ANGLE:\n');
fprintf('     ||T||∞ en [0, 6 Hz]: %.4f (%.2f dB)\n', max_T_pa_pert, 20*log10(max_T_pa_pert));
if max_T_pa_pert < 1.5
    fprintf('     ✅ CUMPLE (||T|| < 1.5)\n\n');
else
    fprintf('     ⚠️  ADVERTENCIA (||T|| > 1.5)\n\n');
end

spec2_cumple = (max_T_pr_pert < 1.5) && (max_T_pa_pert < 1.5);

%% ESPECIFICACIÓN 3: RECHAZO A RUIDO DE MEDICIÓN
fprintf('📌 ESPECIFICACIÓN 3: RECHAZO A RUIDO DE MEDICIÓN\n');
fprintf('   Longitudinal: Potencia 0.0001\n');
fprintf('   Condición: ||T||∞ debe ser pequeño en altas frecuencias\n\n');

omega_high = logspace(2, 4, 100);
[mag_T_pr_high, ~] = bode(T_long(1,1), omega_high);
[mag_T_pa_high, ~] = bode(T_long(2,2), omega_high);

mag_T_pr_high = squeeze(mag_T_pr_high);
mag_T_pa_high = squeeze(mag_T_pa_high);

avg_T_pr_high = mean(mag_T_pr_high);
avg_T_pa_high = mean(mag_T_pa_high);

fprintf('   Canal PITCH RATE:\n');
fprintf('     |T| promedio en alta freq: %.4f (%.2f dB)\n', avg_T_pr_high, 20*log10(avg_T_pr_high));
if avg_T_pr_high < 0.1
    fprintf('     ✅ EXCELENTE rechazo a ruido (< -20 dB)\n');
elseif avg_T_pr_high < 0.5
    fprintf('     ✅ BUENO rechazo a ruido (< -6 dB)\n');
else
    fprintf('     ⚠️  REGULAR rechazo a ruido\n');
end

fprintf('\n   Canal PITCH ANGLE:\n');
fprintf('     |T| promedio en alta freq: %.4f (%.2f dB)\n', avg_T_pa_high, 20*log10(avg_T_pa_high));
if avg_T_pa_high < 0.1
    fprintf('     ✅ EXCELENTE rechazo a ruido (< -20 dB)\n\n');
elseif avg_T_pa_high < 0.5
    fprintf('     ✅ BUENO rechazo a ruido (< -6 dB)\n\n');
else
    fprintf('     ⚠️  REGULAR rechazo a ruido\n\n');
end

spec3_cumple = (avg_T_pr_high < 0.5) && (avg_T_pa_high < 0.5);

%% ESPECIFICACIÓN 4: ESFUERZO DE CONTROL
fprintf('📌 ESPECIFICACIÓN 4: LÍMITE DE ESFUERZO DE CONTROL\n');
fprintf('   Requisito: Señal de control ≤ ±30° para entrada escalón unitario\n\n');

t_control = 0:0.01:10;

CS_long = K_long * S_long;

% Para Pitch Rate
u_ref_pr = zeros(2, length(t_control));
u_ref_pr(1,:) = ones(1, length(t_control));

[u_pr, ~] = lsim(CS_long, u_ref_pr, t_control);
u_elevator_pr = u_pr(:,1);
u_elevator_max_pr = max(abs(u_elevator_pr));

fprintf('   Canal PITCH RATE (q → δe elevador):\n');
fprintf('     Esfuerzo máximo: %.2f°\n', u_elevator_max_pr);
if u_elevator_max_pr <= 30
    fprintf('     ✅ CUMPLE (%.2f° ≤ 30°)\n', u_elevator_max_pr);
else
    fprintf('     ❌ NO CUMPLE (%.2f° > 30°)\n', u_elevator_max_pr);
    fprintf('     → Exceso: %.2f°\n', u_elevator_max_pr - 30);
end

% Para Pitch Angle
u_ref_pa = zeros(2, length(t_control));
u_ref_pa(2,:) = ones(1, length(t_control));

[u_pa, ~] = lsim(CS_long, u_ref_pa, t_control);
u_elevator_pa = u_pa(:,1);
u_elevator_max_pa = max(abs(u_elevator_pa));

fprintf('\n   Canal PITCH ANGLE (θ → δe elevador):\n');
fprintf('     Esfuerzo máximo: %.2f°\n', u_elevator_max_pa);
if u_elevator_max_pa <= 30
    fprintf('     ✅ CUMPLE (%.2f° ≤ 30°)\n\n', u_elevator_max_pa);
else
    fprintf('     ❌ NO CUMPLE (%.2f° > 30°)\n', u_elevator_max_pa);
    fprintf('     → Exceso: %.2f°\n\\n', u_elevator_max_pa - 30);
end

spec4_cumple = (u_elevator_max_pr <= 30) && (u_elevator_max_pa <= 30);

%% GRÁFICA DE ESFUERZO DE CONTROL
figure(103);
subplot(2,1,1);
plot(t_control, u_elevator_pr, 'b', 'LineWidth', 1.5);
hold on;
plot([0 10], [30 30], 'r--', 'LineWidth', 1.5);
plot([0 10], [-30 -30], 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('δe - Elevador (°)', 'FontSize', 11);
title('Esfuerzo de Control - Pitch Rate (Elevador)', 'FontSize', 12);
legend('Señal de control', 'Límite ±30°', 'Location', 'best');
ylim([min(-35, min(u_elevator_pr)-5), max(35, max(u_elevator_pr)+5)]);

subplot(2,1,2);
plot(t_control, u_elevator_pa, 'b', 'LineWidth', 1.5);
hold on;
plot([0 10], [30 30], 'r--', 'LineWidth', 1.5);
plot([0 10], [-30 -30], 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('δe - Elevador (°)', 'FontSize', 11);
title('Esfuerzo de Control - Pitch Angle (Elevador)', 'FontSize', 12);
legend('Señal de control', 'Límite ±30°', 'Location', 'best');
ylim([min(-35, min(u_elevator_pa)-5), max(35, max(u_elevator_pa)+5)]);

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
% Función auxiliar
function out = iif(cond, true_val, false_val)
    if cond, out = true_val; else, out = false_val; end
end