%% Simulación SMC con Tren de Pulsos Variable (Sin Simulink)
% Análisis de seguimiento de referencia sin perturbaciones

clear; close all; clc;

% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

% Parámetros SMC
lambda   = 3.7;     % Ganancia de la superficie deslizante [rad/s]
k_torque = 0.01;    % Ganancia de conmutación [N·m]
phi      = 0.5;     % Ancho de la capa límite [rad/s]

% Parámetros de simulación
Ts   = 0.01;
Tend = 25;
t    = 0:Ts:Tend;
N    = length(t);

%% Generar tren de pulsos variable
ref = th0 * ones(size(t));

pulsos = [
    2,   4,   10;
    5,   7,  -15;
    8,  10,   20;
   11,  12,  -10;
   13,  16,   25;
   17,  18,  -20;
   19,  22,   15;
   23,  25,  -5;
];

for i = 1:size(pulsos, 1)
    t_ini = pulsos(i, 1);
    t_fin = pulsos(i, 2);
    amp   = deg2rad(pulsos(i, 3));
    idx   = (t >= t_ini) & (t < t_fin);
    ref(idx) = th0 + amp;
end

%% Simulación con SMC
theta    = zeros(N, 1);
omega    = zeros(N, 1);
u_hist   = zeros(N, 1);
e_hist   = zeros(N, 1);
s_hist   = zeros(N, 1);  % Superficie deslizante

theta(1) = th0;
omega(1) = 0;

for k = 1:N-1
    % Error de posición (theta - ref para SMC clásico)
    e_theta = theta(k) - ref(k);
    e_hist(k) = -e_theta;  % Para graficar como (ref - theta)
    
    % Superficie deslizante:  s = omega + lambda * e_theta
    s = omega(k) + lambda * e_theta;
    s_hist(k) = s;
    
    % Control equivalente
    u_eq = (J*(-lambda*omega(k)) + Bv*omega(k) + MgL*sin(theta(k))) / Ku;
    
    % Término de conmutación (con tanh para suavizar)
    u_sw = -(k_torque/Ku) * tanh(s/phi);
    
    % Ley de control total
    u = u_eq + u_sw;
    u = max(-1, min(1, u));  % Saturación
    u_hist(k) = u;
    
    % Dinámica no lineal (Euler explícito)
    domega = (Ku*u - Bv*omega(k) - MgL*sin(theta(k))) / J;
    omega(k+1) = omega(k) + Ts * domega;
    theta(k+1) = theta(k) + Ts * omega(k+1);
end
e_hist(N) = ref(N) - theta(N);
u_hist(N) = u_hist(N-1);
s_hist(N) = s_hist(N-1);

%% Métricas
e_deg = rad2deg(e_hist);
IAE   = trapz(t, abs(e_deg));
ISE   = trapz(t, e_deg.^2);
ITAE  = trapz(t, t' .* abs(e_deg));
Emax  = max(abs(e_deg));

%% Gráficas
figure('Name', 'Controlador SMC - Tren de Pulsos Variable', ... 
       'NumberTitle', 'off', 'Position', [200, 60, 1200, 900]);

% 1) Posición
subplot(4,1,1);
plot(t, rad2deg(theta - th0), 'b-', 'LineWidth', 2, 'DisplayName', 'Salida θ(t)');
hold on;
plot(t, rad2deg(ref - th0), 'r--', 'LineWidth', 2, 'DisplayName', 'Referencia r(t)');
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Ángulo [grados]', 'FontSize', 11, 'FontWeight', 'bold');
title('POSICIÓN - Controlador SMC', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([min(rad2deg(ref-th0))-5, max(rad2deg(ref-th0))+5]);

% 2) Señal de control
subplot(4,1,2);
stairs(t, u_hist, 'k-', 'LineWidth', 1.5);
hold on;
yline(1, 'r--', 'LineWidth', 1.5);
yline(-1, 'r--', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('u(t) [duty cycle]', 'FontSize', 11, 'FontWeight', 'bold');
title('SEÑAL DE CONTROL - Controlador SMC', 'FontSize', 14, 'FontWeight', 'bold');
ylim([-1.3, 1.3]);

% 3) Error de seguimiento
subplot(4,1,3);
plot(t, e_deg, 'Color', [0.6, 0.2, 0.8], 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1.5);
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Error [grados]', 'FontSize', 11, 'FontWeight', 'bold');
title('ERROR DE SEGUIMIENTO - Controlador SMC', 'FontSize', 14, 'FontWeight', 'bold');

metStr = sprintf('IAE = %.2f°·s | ISE = %.2f°²·s | ITAE = %.2f°·s² | E_{max} = %.2f°', ... 
                 IAE, ISE, ITAE, Emax);
text(0.5, 0.95, metStr, 'Units', 'normalized', 'FontSize', 10, ...
     'FontWeight', 'bold', 'HorizontalAlignment', 'center', ... 
     'BackgroundColor', 'w', 'EdgeColor', 'k');

% 4) Superficie deslizante
subplot(4,1,4);
plot(t, s_hist, 'Color', [0.1, 0.5, 0.7], 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1.5);
yline(phi, 'g--', 'LineWidth', 1, 'DisplayName', '+φ (capa límite)');
yline(-phi, 'g--', 'LineWidth', 1, 'DisplayName', '-φ (capa límite)');
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('s(t) [rad/s]', 'FontSize', 11, 'FontWeight', 'bold');
title('SUPERFICIE DESLIZANTE - Controlador SMC', 'FontSize', 14, 'FontWeight', 'bold');
legend('s(t)', '+φ', '-φ', 'Location', 'best');

sgtitle('ANÁLISIS SMC - TREN DE PULSOS VARIABLE (Sin Perturbaciones)', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% Resultados en consola
fprintf('\n========================================\n');
fprintf('   CONTROLADOR SMC - RESULTADOS\n');
fprintf('========================================\n');
fprintf('Parámetros SMC:\n');
fprintf('  λ (lambda)  = %.2f rad/s\n', lambda);
fprintf('  k_torque    = %.4f N·m\n', k_torque);
fprintf('  φ (phi)     = %.2f rad/s\n', phi);
fprintf('\nMétricas de desempeño:\n');
fprintf('  IAE  = %.2f °·s\n', IAE);
fprintf('  ISE  = %.2f °²·s\n', ISE);
fprintf('  ITAE = %.2f °·s²\n', ITAE);
fprintf('  Error máximo = %. 2f °\n', Emax);
fprintf('========================================\n');