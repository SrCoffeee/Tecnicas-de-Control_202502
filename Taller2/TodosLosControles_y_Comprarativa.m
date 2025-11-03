%% Control Robusto de Modo Deslizante: Análisis Completo y Comparativo
% Jorge Sofrony - Universidad Nacional de Colombia
% Análisis separado de SM Básico, SM+Equivalente y Comparación
clear all; close all; clc;

%% PARÁMETROS DEL SISTEMA
% Parámetros nominales del motor
R = 8;                          % Resistencia [Ohm]
J = 9.85e-3;                    % Momento de inercia [kg-m^2]
B = 2.52e-3;                    % Constante de fricción [Nm-s/rad]
Km = 1.57e-2;                   % Constante del motor [Nm/Amp]

% Parámetros de la carga
M_nom = 0.3;                    % Masa nominal [kg]
l_nom = 0.05;                   % Distancia nominal [m]
g = 9.81;                       % Gravedad [m/s^2]

% Variaciones paramétricas (20%)
variation = 0.2;                % 20% de variación
M = M_nom * (1 + variation*0.5); % Masa con variación
l = l_nom * (1 - variation*0.3); % Distancia con variación
J_real = J * (1 + variation*0.2);
B_real = B * (1 - variation*0.1);
Km_real = Km * (1 + variation*0.15);
R_real = R * (1 - variation*0.1);

%% PARÁMETROS DE DISEÑO DEL CONTROLADOR
a = 5;                          % Parámetro de la superficie de deslizamiento
beta1 = 35;                     % Ganancia para SM básico
beta2 = 15;                     % Ganancia para SM con control equivalente
phi = 0.01;                     % Parámetro de saturación (para reducir chattering)

%% CONDICIONES INICIALES Y REFERENCIA
x0 = [deg2rad(-20); 0];        % Posición inicial en -20 grados, velocidad 0
xr = deg2rad(30);               % Referencia: 30 grados
tf = 5;                         % Tiempo de simulación [s]

%% ========================================================================
%% PARTE 1: CONTROL SM BÁSICO (ANÁLISIS INDIVIDUAL)
%% ========================================================================
disp('╔════════════════════════════════════════════════════════════════╗');
disp('║         PARTE 1: CONTROL POR MODO DESLIZANTE BÁSICO           ║');
disp('╚════════════════════════════════════════════════════════════════╝');

% Sistema con control SM básico
[t1, x1, u1, s1] = simulate_sm_basic(x0, xr, tf, a, beta1, phi, ...
    J_real, B_real, Km_real, R_real, M, l, g);

% FIGURA 1: ANÁLISIS DEL CONTROL SM BÁSICO
figure('Position', [100, 100, 1400, 900], 'Name', 'Análisis del Control SM Básico', ...
    'Color', 'w');

% Subplot 1: Posición angular
subplot(3,3,1);
plot(t1, rad2deg(x1(:,1)), 'b-', 'LineWidth', 2.5);
hold on;
plot([0 tf], [rad2deg(xr) rad2deg(xr)], 'r--', 'LineWidth', 2);
plot([0 tf], [rad2deg(x0(1)) rad2deg(x0(1))], 'g:', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Posición θ [°]', 'FontSize', 11);
title('Seguimiento de Posición Angular', 'FontSize', 12, 'FontWeight', 'bold');
legend('Respuesta', 'Referencia', 'Inicial', 'Location', 'southeast');
ylim([-25, 35]);
set(gca, 'FontSize', 10);

% Subplot 2: Velocidad angular
subplot(3,3,2);
plot(t1, rad2deg(x1(:,2)), 'b-', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Velocidad ω [°/s]', 'FontSize', 11);
title('Velocidad Angular', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

% Subplot 3: Señal de control
subplot(3,3,3);
plot(t1, u1, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Señal de Control', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-40, 40]);
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% Subplot 4: Superficie de deslizamiento
subplot(3,3,4);
plot(t1, s1, 'b-', 'LineWidth', 2);
hold on;
plot([0 tf], [0 0], 'r--', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Superficie s', 'FontSize', 11);
title('Superficie de Deslizamiento', 'FontSize', 12, 'FontWeight', 'bold');
legend('s(t)', 's = 0', 'Location', 'best');
set(gca, 'FontSize', 10);

% Subplot 5: Error de posición
subplot(3,3,5);
error1 = rad2deg(x1(:,1) - xr);
plot(t1, error1, 'b-', 'LineWidth', 2);
hold on;
plot([0 tf], [0 0], 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Error e [°]', 'FontSize', 11);
title('Error de Posición', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

% Subplot 6: Plano de fase
subplot(3,3,6);
plot(rad2deg(x1(:,1)), rad2deg(x1(:,2)), 'b-', 'LineWidth', 2.5);
hold on;
plot(rad2deg(x0(1)), rad2deg(x0(2)), 'go', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');
plot(rad2deg(xr), 0, 'rx', 'MarkerSize', 15, 'LineWidth', 3, ...
    'DisplayName', 'Objetivo');
grid on;
xlabel('Posición θ [°]', 'FontSize', 11);
ylabel('Velocidad ω [°/s]', 'FontSize', 11);
title('Plano de Fase', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');
set(gca, 'FontSize', 10);

% Subplot 7: Zoom de control (transitorio)
subplot(3,3,7);
idx_zoom1 = find(t1 >= 0 & t1 <= 1);
plot(t1(idx_zoom1), u1(idx_zoom1), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Zoom: Control en Transitorio (0-1s)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-40, 40]);
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% Subplot 8: Zoom de control (estado estacionario)
subplot(3,3,8);
idx_zoom2 = find(t1 >= 4 & t1 <= 5);
plot(t1(idx_zoom2), u1(idx_zoom2), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Zoom: Control en Estado Estacionario (4-5s)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-40, 40]);
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% Subplot 9: Índice de chattering
subplot(3,3,9);
du1 = abs(diff(u1)./diff(t1));
plot(t1(2:end), du1, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('|du/dt| [V/s]', 'FontSize', 11);
title('Índice de Chattering', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0, 5000]);
set(gca, 'FontSize', 10);

% Agregar título general
sgtitle('CONTROL POR MODO DESLIZANTE BÁSICO (β = 35V)', ...
    'FontSize', 14, 'FontWeight', 'bold');

% Calcular métricas
metrics1 = calculate_metrics(t1, x1, u1, s1, x0, xr, tf);

% Mostrar métricas en consola
disp(' ');
disp('Métricas del Control SM Básico:');
disp('─────────────────────────────────────────────');
fprintf('Tiempo de establecimiento (2%%): %.3f s\n', metrics1.ts);
fprintf('Sobrepaso: %.2f %%\n', metrics1.overshoot);
fprintf('Esfuerzo de control: %.1f V²·s\n', metrics1.control_effort);
fprintf('Índice de chattering: %.1f V/s\n', metrics1.chattering);
fprintf('Voltaje máximo: %.1f V\n', metrics1.u_max);
fprintf('Error RMS final: %.3f °\n', metrics1.error_rms);
disp(' ');

%% ========================================================================
%% PARTE 2: CONTROL SM CON CONTROL EQUIVALENTE (ANÁLISIS INDIVIDUAL)
%% ========================================================================
disp('╔════════════════════════════════════════════════════════════════╗');
disp('║     PARTE 2: CONTROL SM + CONTROL EQUIVALENTE                 ║');
disp('╚════════════════════════════════════════════════════════════════╝');

% Sistema con control SM + equivalente
[t2, x2, u2, s2] = simulate_sm_equivalent(x0, xr, tf, a, beta2, phi, ...
    J, B, Km, R, M_nom, l_nom, g, ...  % Parámetros nominales para ueq
    J_real, B_real, Km_real, R_real, M, l, g);  % Parámetros reales

% FIGURA 2: ANÁLISIS DEL CONTROL SM + EQUIVALENTE
figure('Position', [150, 150, 1400, 900], 'Name', 'Análisis del Control SM + Equivalente', ...
    'Color', 'w');

% Subplot 1: Posición angular
subplot(3,3,1);
plot(t2, rad2deg(x2(:,1)), 'r-', 'LineWidth', 2.5);
hold on;
plot([0 tf], [rad2deg(xr) rad2deg(xr)], 'k--', 'LineWidth', 2);
plot([0 tf], [rad2deg(x0(1)) rad2deg(x0(1))], 'g:', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Posición θ [°]', 'FontSize', 11);
title('Seguimiento de Posición Angular', 'FontSize', 12, 'FontWeight', 'bold');
legend('Respuesta', 'Referencia', 'Inicial', 'Location', 'southeast');
ylim([-25, 35]);
set(gca, 'FontSize', 10);

% Subplot 2: Velocidad angular
subplot(3,3,2);
plot(t2, rad2deg(x2(:,2)), 'r-', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Velocidad ω [°/s]', 'FontSize', 11);
title('Velocidad Angular', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

% Subplot 3: Señal de control
subplot(3,3,3);
plot(t2, u2, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Señal de Control', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-40, 40]);
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% Subplot 4: Superficie de deslizamiento
subplot(3,3,4);
plot(t2, s2, 'r-', 'LineWidth', 2);
hold on;
plot([0 tf], [0 0], 'k--', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Superficie s', 'FontSize', 11);
title('Superficie de Deslizamiento', 'FontSize', 12, 'FontWeight', 'bold');
legend('s(t)', 's = 0', 'Location', 'best');
set(gca, 'FontSize', 10);

% Subplot 5: Error de posición
subplot(3,3,5);
error2 = rad2deg(x2(:,1) - xr);
plot(t2, error2, 'r-', 'LineWidth', 2);
hold on;
plot([0 tf], [0 0], 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Error e [°]', 'FontSize', 11);
title('Error de Posición', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

% Subplot 6: Plano de fase
subplot(3,3,6);
plot(rad2deg(x2(:,1)), rad2deg(x2(:,2)), 'r-', 'LineWidth', 2.5);
hold on;
plot(rad2deg(x0(1)), rad2deg(x0(2)), 'go', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');
plot(rad2deg(xr), 0, 'kx', 'MarkerSize', 15, 'LineWidth', 3, ...
    'DisplayName', 'Objetivo');
grid on;
xlabel('Posición θ [°]', 'FontSize', 11);
ylabel('Velocidad ω [°/s]', 'FontSize', 11);
title('Plano de Fase', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');
set(gca, 'FontSize', 10);

% Subplot 7: Zoom de control (transitorio)
subplot(3,3,7);
idx_zoom1 = find(t2 >= 0 & t2 <= 1);
plot(t2(idx_zoom1), u2(idx_zoom1), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Zoom: Control en Transitorio (0-1s)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-40, 40]);
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% Subplot 8: Zoom de control (estado estacionario)
subplot(3,3,8);
idx_zoom2 = find(t2 >= 4 & t2 <= 5);
plot(t2(idx_zoom2), u2(idx_zoom2), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Zoom: Control en Estado Estacionario (4-5s)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-40, 40]);
yline(0, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% Subplot 9: Índice de chattering
subplot(3,3,9);
du2 = abs(diff(u2)./diff(t2));
plot(t2(2:end), du2, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('|du/dt| [V/s]', 'FontSize', 11);
title('Índice de Chattering', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0, 5000]);
set(gca, 'FontSize', 10);

% Agregar título general
sgtitle('CONTROL SM + CONTROL EQUIVALENTE (β = 15V)', ...
    'FontSize', 14, 'FontWeight', 'bold');

% Calcular métricas
metrics2 = calculate_metrics(t2, x2, u2, s2, x0, xr, tf);

% Mostrar métricas en consola
disp(' ');
disp('Métricas del Control SM + Equivalente:');
disp('─────────────────────────────────────────────');
fprintf('Tiempo de establecimiento (2%%): %.3f s\n', metrics2.ts);
fprintf('Sobrepaso: %.2f %%\n', metrics2.overshoot);
fprintf('Esfuerzo de control: %.1f V²·s\n', metrics2.control_effort);
fprintf('Índice de chattering: %.1f V/s\n', metrics2.chattering);
fprintf('Voltaje máximo: %.1f V\n', metrics2.u_max);
fprintf('Error RMS final: %.3f °\n', metrics2.error_rms);
disp(' ');

%% ========================================================================
%% PARTE 3: COMPARACIÓN ENTRE AMBOS CONTROLADORES
%% ========================================================================
disp('╔════════════════════════════════════════════════════════════════╗');
disp('║         PARTE 3: COMPARACIÓN DE CONTROLADORES                 ║');
disp('╚════════════════════════════════════════════════════════════════╝');

% FIGURA 3: COMPARACIÓN DIRECTA
figure('Position', [200, 200, 1400, 900], 'Name', 'Comparación de Controladores SM', ...
    'Color', 'w');

% Subplot 1: Posición angular
subplot(3,3,1);
plot(t1, rad2deg(x1(:,1)), 'b-', 'LineWidth', 2.5, 'DisplayName', 'SM Básico');
hold on;
plot(t2, rad2deg(x2(:,1)), 'r-', 'LineWidth', 2.5, 'DisplayName', 'SM + Equivalente');
plot([0 tf], [rad2deg(xr) rad2deg(xr)], 'k--', 'LineWidth', 2, 'DisplayName', 'Referencia');
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Posición θ [°]', 'FontSize', 11);
title('Seguimiento de Posición Angular', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 10);
ylim([-25, 35]);
set(gca, 'FontSize', 10);

% Subplot 2: Velocidad angular
subplot(3,3,2);
plot(t1, rad2deg(x1(:,2)), 'b-', 'LineWidth', 2, 'DisplayName', 'SM Básico');
hold on;
plot(t2, rad2deg(x2(:,2)), 'r-', 'LineWidth', 2, 'DisplayName', 'SM + Equivalente');
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Velocidad ω [°/s]', 'FontSize', 11);
title('Velocidad Angular', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Subplot 3: Señal de control
subplot(3,3,3);
plot(t1, u1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'SM Básico');
hold on;
plot(t2, u2, 'r-', 'LineWidth', 1.5, 'DisplayName', 'SM + Equivalente');
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Señal de Control', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([-40, 40]);
set(gca, 'FontSize', 10);

% Subplot 4: Superficie de deslizamiento
subplot(3,3,4);
plot(t1, s1, 'b-', 'LineWidth', 2, 'DisplayName', 'SM Básico');
hold on;
plot(t2, s2, 'r-', 'LineWidth', 2, 'DisplayName', 'SM + Equivalente');
plot([0 tf], [0 0], 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Superficie s', 'FontSize', 11);
title('Superficie de Deslizamiento', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Subplot 5: Error de posición
subplot(3,3,5);
plot(t1, error1, 'b-', 'LineWidth', 2, 'DisplayName', 'SM Básico');
hold on;
plot(t2, error2, 'r-', 'LineWidth', 2, 'DisplayName', 'SM + Equivalente');
plot([0 tf], [0 0], 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Error e [°]', 'FontSize', 11);
title('Error de Posición', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Subplot 6: Plano de fase
subplot(3,3,6);
plot(rad2deg(x1(:,1)), rad2deg(x1(:,2)), 'b-', 'LineWidth', 2.5, ...
    'DisplayName', 'SM Básico');
hold on;
plot(rad2deg(x2(:,1)), rad2deg(x2(:,2)), 'r-', 'LineWidth', 2.5, ...
    'DisplayName', 'SM + Equivalente');
plot(rad2deg(x0(1)), rad2deg(x0(2)), 'go', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');
plot(rad2deg(xr), 0, 'kx', 'MarkerSize', 15, 'LineWidth', 3, ...
    'DisplayName', 'Objetivo');
grid on;
xlabel('Posición θ [°]', 'FontSize', 11);
ylabel('Velocidad ω [°/s]', 'FontSize', 11);
title('Plano de Fase', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);
set(gca, 'FontSize', 10);

% Subplot 7: Comparación de chattering
subplot(3,3,7);
plot(t1(2:end), du1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'SM Básico');
hold on;
plot(t2(2:end), du2, 'r-', 'LineWidth', 1.5, 'DisplayName', 'SM + Equivalente');
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('|du/dt| [V/s]', 'FontSize', 11);
title('Índice de Chattering', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([0, 5000]);
set(gca, 'FontSize', 10);

% Subplot 8: Zoom comparativo (transitorio)
subplot(3,3,8);
idx_zoom = find(t1 >= 0 & t1 <= 1);
plot(t1(idx_zoom), u1(idx_zoom), 'b-', 'LineWidth', 1.5, 'DisplayName', 'SM Básico');
hold on;
plot(t2(idx_zoom), u2(idx_zoom), 'r-', 'LineWidth', 1.5, 'DisplayName', 'SM + Equivalente');
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Zoom: Transitorio (0-1s)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([-40, 40]);
set(gca, 'FontSize', 10);

% Subplot 9: Zoom comparativo (estado estacionario)
subplot(3,3,9);
idx_zoom = find(t1 >= 4 & t1 <= 5);
plot(t1(idx_zoom), u1(idx_zoom), 'b-', 'LineWidth', 1.5, 'DisplayName', 'SM Básico');
hold on;
plot(t2(idx_zoom), u2(idx_zoom), 'r-', 'LineWidth', 1.5, 'DisplayName', 'SM + Equivalente');
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Voltaje u [V]', 'FontSize', 11);
title('Zoom: Estado Estacionario (4-5s)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([-20, 20]);
set(gca, 'FontSize', 10);

% Agregar título general
sgtitle('COMPARACIÓN: SM BÁSICO vs SM + CONTROL EQUIVALENTE', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% TABLA COMPARATIVA DETALLADA
disp(' ');
disp('╔════════════════════════════════════════════════════════════════╗');
disp('║              TABLA COMPARATIVA DE DESEMPEÑO                    ║');
disp('╚════════════════════════════════════════════════════════════════╝');
disp(' ');

fprintf('%-40s %15s %15s %15s\n', 'Métrica', 'SM Básico', 'SM+Equiv', 'Mejora[%]');
fprintf('%-40s %15s %15s %15s\n', '-------', '---------', '--------', '---------');

% Tiempo de establecimiento
mejora_ts = (metrics1.ts - metrics2.ts) / metrics1.ts * 100;
fprintf('%-40s %15.3f %15.3f %15.1f\n', 'Tiempo establecimiento [s]', ...
    metrics1.ts, metrics2.ts, mejora_ts);

% Sobrepaso
fprintf('%-40s %15.2f %15.2f %15s\n', 'Sobrepaso [%]', ...
    metrics1.overshoot, metrics2.overshoot, '---');

% Esfuerzo de control
mejora_effort = (metrics1.control_effort - metrics2.control_effort) / metrics1.control_effort * 100;
fprintf('%-40s %15.1f %15.1f %15.1f\n', 'Esfuerzo de control [V²·s]', ...
    metrics1.control_effort, metrics2.control_effort, mejora_effort);

% Índice de chattering
mejora_chat = (metrics1.chattering - metrics2.chattering) / metrics1.chattering * 100;
fprintf('%-40s %15.1f %15.1f %15.1f\n', 'Índice de chattering [V/s]', ...
    metrics1.chattering, metrics2.chattering, mejora_chat);

% Voltaje máximo
mejora_umax = (metrics1.u_max - metrics2.u_max) / metrics1.u_max * 100;
fprintf('%-40s %15.1f %15.1f %15.1f\n', 'Voltaje máximo [V]', ...
    metrics1.u_max, metrics2.u_max, mejora_umax);

% Error RMS
mejora_rms = (metrics1.error_rms - metrics2.error_rms) / metrics1.error_rms * 100;
fprintf('%-40s %15.3f %15.3f %15.1f\n', 'Error RMS final [°]', ...
    metrics1.error_rms, metrics2.error_rms, mejora_rms);

% Ganancia beta
reduccion_beta = (beta1 - beta2) / beta1 * 100;
fprintf('%-40s %15.1f %15.1f %15.1f\n', 'Ganancia β [V]', ...
    beta1, beta2, reduccion_beta);

disp(' ');
disp('Resumen de mejoras:');
disp('───────────────────────────────────────────────────────────────');
fprintf('✓ Reducción de chattering: %.1f%%\n', mejora_chat);
fprintf('✓ Reducción de esfuerzo de control: %.1f%%\n', mejora_effort);
fprintf('✓ Mejora en tiempo de establecimiento: %.1f%%\n', mejora_ts);
fprintf('✓ Reducción de voltaje máximo: %.1f%%\n', mejora_umax);
fprintf('✓ Reducción de ganancia β requerida: %.1f%%\n', reduccion_beta);
disp(' ');

%% GUARDADO DE RESULTADOS
disp('Guardando resultados en workspace...');
results.sm_basic.t = t1;
results.sm_basic.x = x1;
results.sm_basic.u = u1;
results.sm_basic.s = s1;
results.sm_basic.metrics = metrics1;

results.sm_equivalent.t = t2;
results.sm_equivalent.x = x2;
results.sm_equivalent.u = u2;
results.sm_equivalent.s = s2;
results.sm_equivalent.metrics = metrics2;

disp('✓ Análisis completo finalizado!');
disp('✓ 3 figuras generadas');
disp('✓ Variable "results" guardada en workspace');

%% ========================================================================
%% FUNCIONES AUXILIARES
%% ========================================================================

function [t, x, u, s] = simulate_sm_basic(x0, xr, tf, a, beta, phi, ...
    J, B, Km, R, M, l, g)
    % Simulación del control SM básico
    
    dt = 0.001;
    t = 0:dt:tf;
    x = zeros(length(t), 2);
    u = zeros(length(t), 1);
    s = zeros(length(t), 1);
    
    x(1,:) = x0';
    
    for i = 1:length(t)-1
        % Calcular superficie
        s(i) = a*(x(i,1)-xr) + x(i,2);
        
        % Control SM básico
        u(i) = -beta * sat(s(i)/phi);
        
        % Dinámica del sistema
        x_dot = [x(i,2);
                -B/J*x(i,2) - M*g*l/J*sin(x(i,1)) + Km/(J*R)*u(i)];
        
        % Integración Runge-Kutta 4
        k1 = x_dot;
        k2 = [x(i,2) + dt/2*k1(2);
              -B/J*(x(i,2)+dt/2*k1(2)) - M*g*l/J*sin(x(i,1)+dt/2*k1(1)) + Km/(J*R)*u(i)];
        k3 = [x(i,2) + dt/2*k2(2);
              -B/J*(x(i,2)+dt/2*k2(2)) - M*g*l/J*sin(x(i,1)+dt/2*k2(1)) + Km/(J*R)*u(i)];
        k4 = [x(i,2) + dt*k3(2);
              -B/J*(x(i,2)+dt*k3(2)) - M*g*l/J*sin(x(i,1)+dt*k3(1)) + Km/(J*R)*u(i)];
        
        x(i+1,:) = x(i,:) + dt/6 * (k1 + 2*k2 + 2*k3 + k4)';
    end
    
    s(end) = a*(x(end,1)-xr) + x(end,2);
    u(end) = -beta * sat(s(end)/phi);
end

function [t, x, u, s] = simulate_sm_equivalent(x0, xr, tf, a, beta, phi, ...
    J_nom, B_nom, Km_nom, R_nom, M_nom, l_nom, g, ...
    J_real, B_real, Km_real, R_real, M_real, l_real, g_real)
    % Simulación del control SM con control equivalente
    
    dt = 0.001;
    t = 0:dt:tf;
    x = zeros(length(t), 2);
    u = zeros(length(t), 1);
    s = zeros(length(t), 1);
    
    x(1,:) = x0';
    
    for i = 1:length(t)-1
        % Calcular superficie
        s(i) = a*(x(i,1)-xr) + x(i,2);
        
        % Control equivalente (usando parámetros nominales)
        ueq = (-a*x(i,2) + B_nom/J_nom*x(i,2) + M_nom*g*l_nom/J_nom*sin(x(i,1))) ...
              / (Km_nom/(J_nom*R_nom));
        
        % Control total
        u(i) = ueq - beta * sat(s(i)/phi);
        
        % Dinámica del sistema real
        x_dot = [x(i,2);
                -B_real/J_real*x(i,2) - M_real*g_real*l_real/J_real*sin(x(i,1)) ...
                + Km_real/(J_real*R_real)*u(i)];
        
        % Integración Runge-Kutta 4
        k1 = x_dot;
        k2 = [x(i,2) + dt/2*k1(2);
              -B_real/J_real*(x(i,2)+dt/2*k1(2)) - M_real*g_real*l_real/J_real*sin(x(i,1)+dt/2*k1(1)) ...
              + Km_real/(J_real*R_real)*u(i)];
        k3 = [x(i,2) + dt/2*k2(2);
              -B_real/J_real*(x(i,2)+dt/2*k2(2)) - M_real*g_real*l_real/J_real*sin(x(i,1)+dt/2*k2(1)) ...
              + Km_real/(J_real*R_real)*u(i)];
        k4 = [x(i,2) + dt*k3(2);
              -B_real/J_real*(x(i,2)+dt*k3(2)) - M_real*g_real*l_real/J_real*sin(x(i,1)+dt*k3(1)) ...
              + Km_real/(J_real*R_real)*u(i)];
        
        x(i+1,:) = x(i,:) + dt/6 * (k1 + 2*k2 + 2*k3 + k4)';
    end
    
    s(end) = a*(x(end,1)-xr) + x(end,2);
    ueq = (-a*x(end,2) + B_nom/J_nom*x(end,2) + M_nom*g*l_nom/J_nom*sin(x(end,1))) ...
          / (Km_nom/(J_nom*R_nom));
    u(end) = ueq - beta * sat(s(end)/phi);
end

function metrics = calculate_metrics(t, x, u, s, x0, xr, tf)
    % Calcular métricas de desempeño
    
    metrics = struct();
    
    % Tiempo de establecimiento (2%)
    threshold = 0.02 * abs(xr - x0(1));
    idx = find(abs(x(:,1) - xr) < threshold, 1, 'first');
    if ~isempty(idx)
        metrics.ts = t(idx);
    else
        metrics.ts = tf;
    end
    
    % Sobrepaso
    metrics.overshoot = max((x(:,1) - xr)/abs(xr - x0(1))) * 100;
    
    % Esfuerzo de control
    metrics.control_effort = trapz(t, u.^2);
    
    % Índice de chattering
    metrics.chattering = sum(abs(diff(u))) / tf;
    
    % Voltaje máximo
    metrics.u_max = max(abs(u));
    
    % Error RMS final (último 20% de la simulación)
    idx_final = round(0.8*length(t)):length(t);
    metrics.error_rms = rad2deg(sqrt(mean((x(idx_final,1) - xr).^2)));
end

function y = sat(x)
    % Función de saturación
    if abs(x) <= 1
        y = x;
    else
        y = sign(x);
    end
end
