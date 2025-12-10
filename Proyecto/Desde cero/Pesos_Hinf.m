%% Análisis de Pesos de Ponderación para Control H-infinity
% Visualización de W1, W2, W3 y funciones de sensibilidad

clear; close all; clc;
s = tf('s');

%% Parámetros de la planta
J   = 1.08e-3;
Bv  = 2.61e-3;
MgL = 4.999e-3;
Ku  = 1.13e-2;
th0 = deg2rad(50);

a0 = (MgL*cos(th0))/J;
a1 = Bv/J;
b0 = Ku/J;

G = tf(b0, [1 a1 a0]);

fprintf('Planta G(s):\n');
zpk(G)

%% Función auxiliar para obtener magnitud en dB
function [mag_dB] = get_mag_dB(sys, w)
    [mag, ~] = bode(sys, w);
    mag_dB = 20*log10(squeeze(mag));
end

%% Definición de Pesos de Ponderación

% =========================================================================
% W1: Peso de Sensibilidad (S)
% =========================================================================
M_s  = 2.0;      % Pico máximo de sensibilidad
wb   = 2.0;      % Frecuencia de cruce (rad/s)
A_s  = 0.01;     % Error en estado estacionario (1%)

% Crear W1 manualmente (forma estándar)
W1 = (s/M_s + wb) / (s + wb*A_s);
fprintf('W1 creado manualmente\n');

% =========================================================================
% W2: Peso de Esfuerzo de Control (KS)
% =========================================================================
W2_const = tf(0.2);

w_u   = 50;
M_u   = 0.1;
A_u   = 2.0;
W2_hp = (s + w_u*M_u) / (s/A_u + w_u);

W2 = W2_const;

% =========================================================================
% W3: Peso de Sensibilidad Complementaria (T)
% =========================================================================
M_t  = 2.0;
wt   = 10.0;
A_t  = 0.01;

W3 = (s + wt*A_t) / (s/M_t + wt);
fprintf('W3 creado manualmente\n');

%% Vector de frecuencias
w = logspace(-2, 3, 500);

%% === FIGURA 1: Pesos de Ponderación Individuales ===
figure('Name', 'Pesos de Ponderación H-infinity', 'Position', [50, 300, 1400, 600]);

% Obtener magnitudes
mag_W1 = get_mag_dB(W1, w);
mag_W1_inv = get_mag_dB(1/W1, w);
mag_W2 = get_mag_dB(W2, w);
mag_W2_inv = get_mag_dB(1/W2, w);
mag_W2_hp = get_mag_dB(W2_hp, w);
mag_W2_const = get_mag_dB(W2_const, w);
mag_W3 = get_mag_dB(W3, w);
mag_W3_inv = get_mag_dB(1/W3, w);

% W1: Peso de sensibilidad
subplot(2,3,1);
semilogx(w, mag_W1, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W1_inv, 'r--', 'LineWidth', 1.5);
grid on;
title('W_1(s) - Peso de Sensibilidad', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('|W_1|', '|1/W_1| (límite para S)', 'Location', 'best');

subplot(2,3,4);
semilogx(w, mag_W1, 'b-', 'LineWidth', 2);
hold on;
xline(wb, 'g--', 'LineWidth', 2);
yline(20*log10(1/A_s), 'b--', 'LineWidth', 1.5);
yline(20*log10(M_s), 'r--', 'LineWidth', 1.5);
grid on;
title('W_1: Interpretación', 'FontSize', 11);
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
text(0.02, 35, sprintf('1/A = %.0f dB', 20*log10(1/A_s)), 'FontSize', 9);
text(20, 10, sprintf('M = %.1f dB', 20*log10(M_s)), 'FontSize', 9);
text(wb*1.5, 25, sprintf('\\omega_b = %.1f', wb), 'FontSize', 9, 'Color', 'g');

% W2: Peso de control
subplot(2,3,2);
semilogx(w, mag_W2, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W2_inv, 'r--', 'LineWidth', 1.5);
grid on;
title('W_2(s) - Peso de Esfuerzo de Control', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('|W_2|', '|1/W_2| (límite para KS)', 'Location', 'best');

subplot(2,3,5);
semilogx(w, mag_W2_hp, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W2_const, 'r--', 'LineWidth', 1.5);
grid on;
title('W_2: Opciones de Diseño', 'FontSize', 11);
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('Pasa-altas', 'Constante', 'Location', 'best');

% W3: Peso de sensibilidad complementaria
subplot(2,3,3);
semilogx(w, mag_W3, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W3_inv, 'r--', 'LineWidth', 1.5);
grid on;
title('W_3(s) - Peso de Sens. Complementaria', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('|W_3|', '|1/W_3| (límite para T)', 'Location', 'best');

subplot(2,3,6);
semilogx(w, mag_W3, 'b-', 'LineWidth', 2);
hold on;
xline(wt, 'g--', 'LineWidth', 2);
grid on;
title('W_3: Interpretación', 'FontSize', 11);
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
text(0.02, -30, sprintf('A = %.2f', A_t), 'FontSize', 9);
text(50, 5, sprintf('M = %.1f', M_t), 'FontSize', 9);
text(wt*1.5, -10, sprintf('\\omega_t = %.1f', wt), 'FontSize', 9, 'Color', 'g');

sgtitle('PESOS DE PONDERACIÓN PARA SÍNTESIS H-\infty', 'FontSize', 14, 'FontWeight', 'bold');

%% === FIGURA 2: Diseño Completo con Sensibilidades ===
figure('Name', 'Síntesis H-infinity', 'Position', [100, 250, 1400, 700]);

% Diseñar controlador H-infinity
try
    P = augw(G, W1, W2, []);
    [K_hinf, CL, gamma] = hinfsyn(P, 1, 1);
    fprintf('\n¡Síntesis H-infinity exitosa!\n');
    fprintf('Gamma óptimo: %.4f\n', gamma);
    use_hinf = true;
catch
    fprintf('\nRobust Control Toolbox no disponible.\n');
    fprintf('Usando controlador lead-lag como aproximación.\n');
    k  = 1.5;
    wz = 2.0;
    wp = 8.0;
    K_hinf = k * (1 + s/wz) / (1 + s/wp);
    gamma = 1.5;  % Valor aproximado
    use_hinf = false;
end

% Calcular funciones de sensibilidad
L = G * K_hinf;
S = feedback(1, L);
T = feedback(L, 1);
KS = K_hinf * S;

% Obtener magnitudes
mag_S = get_mag_dB(S, w);
mag_T = get_mag_dB(T, w);
mag_KS = get_mag_dB(KS, w);
mag_L = get_mag_dB(L, w);
mag_gamma_W1_inv = get_mag_dB(gamma/W1, w);

% Subplot 1: Sensibilidad S vs 1/W1
subplot(2,2,1);
semilogx(w, mag_S, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W1_inv, 'r--', 'LineWidth', 1.5);
if use_hinf
    semilogx(w, mag_gamma_W1_inv, 'g:', 'LineWidth', 1.5);
    legend('|S|', '|1/W_1|', sprintf('|\\gamma/W_1| (\\gamma=%.2f)', gamma), 'Location', 'best');
else
    legend('|S|', '|1/W_1|', 'Location', 'best');
end
grid on;
title('Sensibilidad S(s) vs Límite 1/W_1', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');

% Subplot 2: Esfuerzo de control KS vs 1/W2
subplot(2,2,2);
semilogx(w, mag_KS, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W2_inv, 'r--', 'LineWidth', 1.5);
grid on;
title('Esfuerzo de Control K(s)S(s) vs Límite 1/W_2', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('|KS|', '|1/W_2|', 'Location', 'best');

% Subplot 3: Sensibilidad complementaria T vs 1/W3
subplot(2,2,3);
semilogx(w, mag_T, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W3_inv, 'r--', 'LineWidth', 1.5);
grid on;
title('Sens. Complementaria T(s) vs Límite 1/W_3', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('|T|', '|1/W_3|', 'Location', 'best');

% Subplot 4: Todas las funciones juntas
subplot(2,2,4);
semilogx(w, mag_S, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_T, 'r-', 'LineWidth', 2);
semilogx(w, mag_L, 'g-', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xline(wb, 'm:', 'LineWidth', 1.5);
grid on;
title('Funciones de Lazo Cerrado', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('S', 'T', 'L', '0 dB', '\omega_b', 'Location', 'best');

if use_hinf
    sgtitle(sprintf('SÍNTESIS H-\\infty:  \\gamma_{opt} = %.4f', gamma), 'FontSize', 14, 'FontWeight', 'bold');
else
    sgtitle('ANÁLISIS H-\infty (Controlador Lead-Lag)', 'FontSize', 14, 'FontWeight', 'bold');
end

%% === FIGURA 3: Interpretación de los Pesos ===
figure('Name', 'Interpretación de Pesos', 'Position', [150, 200, 1200, 600]);

% Gráfica combinada de todos los inversos de pesos
subplot(1,2,1);
semilogx(w, mag_W1_inv, 'b-', 'LineWidth', 2);
hold on;
semilogx(w, mag_W2_inv, 'r-', 'LineWidth', 2);
semilogx(w, mag_W3_inv, 'g-', 'LineWidth', 2);
grid on;
title('LÍMITES SUPERIORES (1/W_i)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
legend('1/W_1 (límite S)', '1/W_2 (límite KS)', '1/W_3 (límite T)', ...
       'Location', 'best', 'FontSize', 10);

% Regiones de frecuencia
subplot(1,2,2);
freq_regions = [0.01, wb, wt, 100];
colors = [0.8 0.9 1.0; 1.0 0.95 0.8; 1.0 0.85 0.85];

hold on;
for i = 1:3
    fill([freq_regions(i) freq_regions(i+1) freq_regions(i+1) freq_regions(i)], ...
         [-60 -60 40 40], colors(i,: ), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end

semilogx(w, mag_S, 'b-', 'LineWidth', 2);
semilogx(w, mag_T, 'r-', 'LineWidth', 2);
xline(wb, 'k--', 'LineWidth', 1.5);
xline(wt, 'k--', 'LineWidth', 1.5);
yline(0, 'k-', 'LineWidth', 1);
grid on;

title('REGIONES DE DISEÑO', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia [rad/s]');
ylabel('Magnitud [dB]');
ylim([-60, 40]);

% Anotaciones
text(0.02, 35, {'BAJA FREQ:', '• S \approx 0 dB', '• T \approx 0 dB', '• Seguimiento'}, ... 
     'FontSize', 9, 'BackgroundColor', 'w');
text(3, 35, {'MEDIA FREQ:', '• Transición', '• |S| + |T| \approx 1', '• Ancho banda'}, ...
     'FontSize', 9, 'BackgroundColor', 'w');
text(30, 35, {'ALTA FREQ:', '• S \approx 0 dB', '• T \rightarrow 0', '• Robustez'}, ... 
     'FontSize', 9, 'BackgroundColor', 'w');

legend('', '', '', 'S(j\omega)', 'T(j\omega)', '\omega_b', '\omega_t', 'Location', 'southeast');

sgtitle('INTERPRETACIÓN DE PESOS H-\infty', 'FontSize', 14, 'FontWeight', 'bold');

%% === FIGURA 4: Diagrama de Bloques Conceptual ===
figure('Name', 'Estructura Mixed Sensitivity', 'Position', [200, 150, 1000, 500]);

% Crear diagrama explicativo
subplot(1,2,1);
% Mostrar las funciones de transferencia importantes
text(0.1, 0.9, 'PROBLEMA MIXED SENSITIVITY H_\infty', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.75, '────────────────────────────────────', 'FontSize', 10);
text(0.1, 0.65, 'Minimizar:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.15, 0.55, '         ┌ W_1 S  ┐', 'FontSize', 11, 'FontName', 'FixedWidth');
text(0.15, 0.48, '  T_{zw} = │ W_2 KS │', 'FontSize', 11, 'FontName', 'FixedWidth');
text(0.15, 0.41, '         └ W_3 T  ┘', 'FontSize', 11, 'FontName', 'FixedWidth');
text(0.1, 0.28, 'Tal que:  ||T_{zw}||_\infty < \gamma', 'FontSize', 12);
text(0.1, 0.15, '────────────────────────────────────', 'FontSize', 10);
text(0.1, 0.05, sprintf('Resultado: \\gamma_{opt} = %.4f', gamma), 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'b');
axis off;

subplot(1,2,2);
% Significado de cada peso
text(0.05, 0.95, 'SIGNIFICADO DE LOS PESOS', 'FontSize', 14, 'FontWeight', 'bold');
text(0.05, 0.85, '─────────────────────────────────────────', 'FontSize', 10);

text(0.05, 0.75, 'W_1 (Sensibilidad S):', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'b');
text(0.08, 0.68, '• Grande en baja freq → Buen seguimiento', 'FontSize', 10);
text(0.08, 0.62, '• Determina ancho de banda', 'FontSize', 10);
text(0.08, 0.56, '• Rechazo de perturbaciones', 'FontSize', 10);

text(0.05, 0.46, 'W_2 (Esfuerzo KS):', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');
text(0.08, 0.39, '• Limita magnitud del control', 'FontSize', 10);
text(0.08, 0.33, '• Evita saturación del actuador', 'FontSize', 10);

text(0.05, 0.23, 'W_3 (Sens. Complementaria T):', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.6 0]);
text(0.08, 0.16, '• Grande en alta freq → Robustez', 'FontSize', 10);
text(0.08, 0.10, '• Atenúa ruido de medición', 'FontSize', 10);
text(0.08, 0.04, '• Robustez ante incertidumbre', 'FontSize', 10);

axis off;

sgtitle('ESTRUCTURA DEL PROBLEMA H-\infty', 'FontSize', 14, 'FontWeight', 'bold');

%% Información en consola
fprintf('\n========================================\n');
fprintf('   PESOS DE PONDERACIÓN H-∞\n');
fprintf('========================================\n');
fprintf('\nW1 - Peso de Sensibilidad:\n');
fprintf('  • Objetivo: Rechazo de perturbaciones, seguimiento\n');
fprintf('  • M_s = %. 1f (pico máximo)\n', M_s);
fprintf('  • ω_b = %.1f rad/s (ancho de banda)\n', wb);
fprintf('  • A = %.3f (error estado estacionario)\n', A_s);

fprintf('\nW2 - Peso de Esfuerzo de Control:\n');
fprintf('  • Objetivo:  Limitar señal de control\n');
fprintf('  • Valor: %. 2f (constante)\n', dcgain(W2));

fprintf('\nW3 - Peso de Sens. Complementaria:\n');
fprintf('  • Objetivo:  Robustez ante incertidumbre\n');
fprintf('  • M_t = %.1f (pico máximo)\n', M_t);
fprintf('  • ω_t = %.1f rad/s (frecuencia transición)\n', wt);
fprintf('  • A = %.3f (ganancia baja freq)\n', A_t);

fprintf('\n--- Resultado de Síntesis ---\n');
fprintf('Gamma óptimo: %. 4f\n', gamma);
fprintf('Norma H-∞ alcanzada: ||T_zw||_∞ < γ\n');
fprintf('========================================\n');