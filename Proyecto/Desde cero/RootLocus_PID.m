%% Análisis de Root Locus para el Controlador PID
% Lugar de las raíces y análisis de estabilidad

clear; close all; clc;
s = tf('s');

%% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

% Coeficientes del modelo linealizado
a0 = (MgL*cos(th0))/J;
a1 = Bv/J;
b0 = Ku/J;

% Función de transferencia de la planta
G = tf(b0, [1 a1 a0]);

fprintf('Planta G(s) = %.4f / (s² + %.4fs + %.4f)\n', b0, a1, a0);
fprintf('Polos de la planta: \n');
disp(pole(G));

%% Parámetros PID por asignación de polos
zeta = 0.9; 
wn   = 2.2; 
p3   = 4*wn;

% Polos deseados
a2d = 2*zeta*wn + p3;
a1d = wn^2 + 2*zeta*wn*p3;
a0d = (wn^2)*p3;

% Ganancias PID
Kp    = (a2d - a1)/b0;
Kd    = (a1d - a0)/b0;
Ki    = a0d/b0;
tau_d = 0.05;  % Filtro derivativo

fprintf('\n--- Ganancias PID ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);
fprintf('tau_d = %.4f s\n', tau_d);

%% Controlador PID
% PID ideal:  C(s) = Kp + Ki/s + Kd*s
% PID con filtro derivativo:  C(s) = Kp + Ki/s + Kd*s/(1 + tau_d*s)

C_pid_ideal = Kp + Ki/s + Kd*s;
C_pid_real  = Kp + Ki/s + (Kd*s)/(1 + tau_d*s);

% Simplificar
C_pid_ideal = minreal(C_pid_ideal);
C_pid_real  = minreal(C_pid_real);

fprintf('\n--- Controlador PID (ideal) ---\n');
zpk(C_pid_ideal)

fprintf('\n--- Controlador PID (con filtro derivativo) ---\n');
zpk(C_pid_real)

%% Lazo abierto
L_ideal = C_pid_ideal * G;
L_real  = C_pid_real * G;

L_ideal = minreal(L_ideal);
L_real  = minreal(L_real);

%% Lazo cerrado
T_ideal = feedback(L_ideal, 1);
T_real  = feedback(L_real, 1);

T_ideal = minreal(T_ideal);
T_real  = minreal(T_real);

%% === FIGURA 1: Root Locus de la Planta ===
figure('Name', 'Root Locus - Planta', 'Position', [50, 400, 600, 500]);

rlocus(G);
title('ROOT LOCUS - Planta G(s)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Eje Real', 'FontSize', 11);
ylabel('Eje Imaginario', 'FontSize', 11);
grid on;
sgrid(zeta, wn);  % Líneas de amortiguamiento y frecuencia natural

% Marcar polos de la planta
hold on;
p_plant = pole(G);
plot(real(p_plant), imag(p_plant), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
legend('Lugar de raíces', 'Polos de G(s)', 'Location', 'best');

%% === FIGURA 2: Root Locus del Sistema con PID ===
figure('Name', 'Root Locus - Sistema con PID', 'Position', [100, 350, 1200, 500]);

% Root Locus del lazo abierto L(s) = C(s)*G(s)
subplot(1,2,1);
rlocus(L_real);
title('ROOT LOCUS - Lazo Abierto L(s) = C_{PID}(s)·G(s)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Eje Real', 'FontSize', 11);
ylabel('Eje Imaginario', 'FontSize', 11);
grid on;

% Marcar polos y ceros del lazo abierto
hold on;
p_L = pole(L_real);
z_L = zero(L_real);
plot(real(p_L), imag(p_L), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
plot(real(z_L), imag(z_L), 'bo', 'MarkerSize', 12, 'LineWidth', 2);
sgrid([0. 5 0.7 0.9], [1 2 3 5]);
legend('Lugar de raíces', 'Polos', 'Ceros', 'Location', 'best');

% Root Locus con ganancia variable (para análisis)
subplot(1,2,2);
% Separar el controlador en ganancia * forma normalizada
[num_L, den_L] = tfdata(L_real, 'v');
K_total = num_L(1);  % Ganancia total
L_norm = tf(num_L/K_total, den_L);

rlocus(L_norm);
title('ROOT LOCUS - Variando Ganancia K', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Eje Real', 'FontSize', 11);
ylabel('Eje Imaginario', 'FontSize', 11);
grid on;
sgrid([0.5 0.7 0.9], [1 2 3 5]);

% Marcar punto de operación actual
hold on;
p_closed = pole(T_real);
plot(real(p_closed), imag(p_closed), 'mp', 'MarkerSize', 15, 'LineWidth', 2, ... 
     'MarkerFaceColor', 'm');
legend('Lugar de raíces', 'Polos en lazo cerrado', 'Location', 'best');

sgtitle('ANÁLISIS ROOT LOCUS - CONTROLADOR PID', 'FontSize', 14, 'FontWeight', 'bold');

%% === FIGURA 3: Polos en Lazo Cerrado ===
figure('Name', 'Polos Lazo Cerrado', 'Position', [150, 300, 800, 600]);

% Polos deseados (diseño)
p_desired = roots([1 a2d a1d a0d]);

% Graficar plano complejo
plot(real(p_closed), imag(p_closed), 'bp', 'MarkerSize', 15, 'LineWidth', 2, ...
     'MarkerFaceColor', 'b', 'DisplayName', 'Polos reales (con filtro)');
hold on;
plot(real(p_desired), imag(p_desired), 'r*', 'MarkerSize', 15, 'LineWidth', 2, ...
     'DisplayName', 'Polos deseados (diseño)');

% Polos de la planta original
plot(real(p_plant), imag(p_plant), 'ko', 'MarkerSize', 12, 'LineWidth', 2, ... 
     'DisplayName', 'Polos planta original');

% Líneas de referencia
xline(0, 'k-', 'LineWidth', 1);
yline(0, 'k-', 'LineWidth', 1);

% Círculos de frecuencia natural constante
theta_grid = linspace(0, 2*pi, 100);
for wn_val = [1, 2, 3, 5, 8]
    plot(wn_val*cos(theta_grid), wn_val*sin(theta_grid), 'k:', 'LineWidth', 0.5);
    text(-wn_val*0.1, wn_val, sprintf('ω_n=%.0f', wn_val), 'FontSize', 8);
end

% Líneas de amortiguamiento constante
for zeta_val = [0.3, 0.5, 0.7, 0.9]
    theta_zeta = acos(zeta_val);
    r_max = 10;
    plot([0 -r_max*cos(theta_zeta)], [0 r_max*sin(theta_zeta)], 'k:', 'LineWidth', 0.5);
    plot([0 -r_max*cos(theta_zeta)], [0 -r_max*sin(theta_zeta)], 'k:', 'LineWidth', 0.5);
    text(-r_max*0.6*cos(theta_zeta), r_max*0.6*sin(theta_zeta), ...
         sprintf('ζ=%.1f', zeta_val), 'FontSize', 8);
end

grid on;
axis equal;
xlim([-15, 5]);
ylim([-10, 10]);
xlabel('Eje Real (σ)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Eje Imaginario (jω)', 'FontSize', 12, 'FontWeight', 'bold');
title('UBICACIÓN DE POLOS EN EL PLANO COMPLEJO', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

%% === FIGURA 4: Respuesta en Frecuencia y Temporal ===
figure('Name', 'Análisis Complementario', 'Position', [200, 250, 1200, 500]);

% Diagrama de Bode del lazo abierto
subplot(1,2,1);
margin(L_real);
title('DIAGRAMA DE BODE - Lazo Abierto', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% Respuesta al escalón
subplot(1,2,2);
t = 0:0.01:5;
[y, t] = step(T_real, t);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;
yline(1, 'r--', 'LineWidth', 1.5);
yline(1.02, 'g:', 'LineWidth', 1);
yline(0.98, 'g:', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Respuesta', 'FontSize', 11, 'FontWeight', 'bold');
title('RESPUESTA AL ESCALÓN - Lazo Cerrado', 'FontSize', 12, 'FontWeight', 'bold');

% Calcular características
info = stepinfo(T_real);
text(0.5, 0.5, sprintf('Tr = %.3f s\nTs = %.3f s\nMp = %.2f%%\nEss = %.4f', ...
     info.RiseTime, info. SettlingTime, info. Overshoot, 1-dcgain(T_real)), ...
     'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

sgtitle('ANÁLISIS DE ESTABILIDAD Y RESPUESTA - PID', 'FontSize', 14, 'FontWeight', 'bold');

%% Mostrar información en consola
fprintf('\n========================================\n');
fprintf('   ANÁLISIS DE POLOS - LAZO CERRADO\n');
fprintf('========================================\n');
fprintf('\nPolos del sistema en lazo cerrado:\n');
for i = 1:length(p_closed)
    if imag(p_closed(i)) >= 0
        fprintf('  p%d = %.4f + %. 4fj\n', i, real(p_closed(i)), imag(p_closed(i)));
    else
        fprintf('  p%d = %.4f - %. 4fj\n', i, real(p_closed(i)), abs(imag(p_closed(i))));
    end
end

fprintf('\nPolos deseados (diseño):\n');
for i = 1:length(p_desired)
    if imag(p_desired(i)) >= 0
        fprintf('  p%d = %.4f + %.4fj\n', i, real(p_desired(i)), imag(p_desired(i)));
    else
        fprintf('  p%d = %.4f - %. 4fj\n', i, real(p_desired(i)), abs(imag(p_desired(i))));
    end
end

fprintf('\n--- Márgenes de Estabilidad ---\n');
[Gm, Pm, Wcg, Wcp] = margin(L_real);
fprintf('Margen de Ganancia: %. 2f dB (@ %. 2f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('Margen de Fase: %.2f° (@ %.2f rad/s)\n', Pm, Wcp);

fprintf('\n--- Características de Respuesta ---\n');
fprintf('Tiempo de subida (Tr): %.4f s\n', info.RiseTime);
fprintf('Tiempo de establecimiento (Ts): %.4f s\n', info. SettlingTime);
fprintf('Sobrepaso (Mp): %.2f %%\n', info.Overshoot);
fprintf('========================================\n');