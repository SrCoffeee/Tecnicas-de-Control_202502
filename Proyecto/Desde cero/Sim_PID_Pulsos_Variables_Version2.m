%% Simulación PID con Tren de Pulsos Variable (Sin Simulink)
% Análisis de seguimiento de referencia sin perturbaciones

clear; close all; clc;

% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación (50°)

% Parámetros PID por asignación de polos
zeta = 0.9; wn = 2.2; p3 = 4*wn;
a0  = (MgL*cos(th0))/J;
a1  = Bv/J;
b0  = Ku/J;
a2d = 2*zeta*wn + p3;
a1d = wn^2 + 2*zeta*wn*p3;
a0d = (wn^2)*p3;

Kp    = (a2d - a1)/b0;
Kd    = (a1d - a0)/b0;
Ki    = a0d/b0;
tau_d = 0.05;
Tt    = 0.35;

% Parámetros de simulación
Ts   = 0.01;        % Período de muestreo
Tend = 25;          % Tiempo total de simulación
t    = 0:Ts:Tend;
N    = length(t);

%% Generar tren de pulsos variable
% Amplitudes y duraciones variables para analizar respuesta dinámica
ref = th0 * ones(size(t));  % Iniciar en punto de operación

% Definir pulsos:  [tiempo_inicio, tiempo_fin, amplitud_en_grados]
pulsos = [
    2,   4,   10;    % Pulso +10°
    5,   7,  -15;    % Pulso -15°
    8,  10,   20;    % Pulso +20°
   11,  12,  -10;    % Pulso corto -10°
   13,  16,   25;    % Pulso largo +25°
   17,  18,  -20;    % Pulso -20°
   19,  22,   15;    % Pulso +15°
   23,  25,  -5;     % Pulso pequeño -5°
];

for i = 1:size(pulsos, 1)
    t_ini = pulsos(i, 1);
    t_fin = pulsos(i, 2);
    amp   = deg2rad(pulsos(i, 3));
    idx   = (t >= t_ini) & (t < t_fin);
    ref(idx) = th0 + amp;
end

%% Simulación del sistema con PID
theta    = zeros(N, 1);
omega    = zeros(N, 1);
u_hist   = zeros(N, 1);
e_hist   = zeros(N, 1);

theta(1) = th0;
omega(1) = 0;
Iint     = 0;       % Integral del error
Df       = 0;       % Derivada filtrada
eprev    = 0;

for k = 1:N-1
    % Error
    e = ref(k) - theta(k);
    e_hist(k) = e;
    
    % Feedforward
    u_ff = (MgL/Ku) * sin(ref(k));
    
    % PID con filtro derivativo y anti-windup
    a  = tau_d / (tau_d + Ts);
    b  = Kd / (tau_d + Ts);
    Df = a * Df + b * (e - eprev);
    
    u_uns = u_ff + Kp*e + Df + Iint;
    u     = max(-1, min(1, u_uns));  % Saturación
    
    % Anti-windup (back-calculation)
    Iint = Iint + Ki*Ts*e + (Ts/Tt)*(u - u_uns);
    
    u_hist(k) = u;
    
    % Dinámica no lineal (Euler explícito)
    domega = (Ku*u - Bv*omega(k) - MgL*sin(theta(k))) / J;
    omega(k+1) = omega(k) + Ts * domega;
    theta(k+1) = theta(k) + Ts * omega(k+1);
    
    eprev = e;
end
e_hist(N) = ref(N) - theta(N);
u_hist(N) = u_hist(N-1);

%% Calcular métricas de desempeño
e_deg = rad2deg(e_hist);
IAE   = trapz(t, abs(e_deg));
ISE   = trapz(t, e_deg.^2);
ITAE  = trapz(t, t' .* abs(e_deg));
Emax  = max(abs(e_deg));

%% Gráficas
figure('Name', 'Controlador PID - Tren de Pulsos Variable', ... 
       'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% 1) Posición angular
subplot(3,1,1);
plot(t, rad2deg(theta - th0), 'b-', 'LineWidth', 2, 'DisplayName', 'Salida θ(t)');
hold on;
plot(t, rad2deg(ref - th0), 'r--', 'LineWidth', 2, 'DisplayName', 'Referencia r(t)');
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Ángulo [grados]', 'FontSize', 11, 'FontWeight', 'bold');
title('POSICIÓN - Controlador PID', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([min(rad2deg(ref-th0))-5, max(rad2deg(ref-th0))+5]);

% 2) Señal de control
subplot(3,1,2);
stairs(t, u_hist, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Control u(t)');
hold on;
yline(1, 'r--', 'LineWidth', 1.5);
yline(-1, 'r--', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('u(t) [duty cycle]', 'FontSize', 11, 'FontWeight', 'bold');
title('SEÑAL DE CONTROL - Controlador PID', 'FontSize', 14, 'FontWeight', 'bold');
ylim([-1.3, 1.3]);
legend('Location', 'best', 'FontSize', 10);

% 3) Error de seguimiento
subplot(3,1,3);
plot(t, e_deg, 'Color', [0.8, 0.2, 0.2], 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1.5);
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Error [grados]', 'FontSize', 11, 'FontWeight', 'bold');
title('ERROR DE SEGUIMIENTO - Controlador PID', 'FontSize', 14, 'FontWeight', 'bold');

% Mostrar métricas
metStr = sprintf('IAE = %.2f°·s | ISE = %.2f°²·s | ITAE = %.2f°·s² | E_{max} = %.2f°', ...
                 IAE, ISE, ITAE, Emax);
text(0.5, 0.95, metStr, 'Units', 'normalized', 'FontSize', 10, ... 
     'FontWeight', 'bold', 'HorizontalAlignment', 'center', ... 
     'BackgroundColor', 'w', 'EdgeColor', 'k');

sgtitle('ANÁLISIS PID - TREN DE PULSOS VARIABLE (Sin Perturbaciones)', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% Mostrar parámetros en consola
fprintf('\n========================================\n');
fprintf('   CONTROLADOR PID - RESULTADOS\n');
fprintf('========================================\n');
fprintf('Parámetros PID:\n');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f\n', Ki);
fprintf('  Kd = %. 4f\n', Kd);
fprintf('  tau_d = %.4f s\n', tau_d);
fprintf('  Tt = %.4f s (anti-windup)\n', Tt);
fprintf('\nMétricas de desempeño:\n');
fprintf('  IAE  = %.2f °·s\n', IAE);
fprintf('  ISE  = %.2f °²·s\n', ISE);
fprintf('  ITAE = %.2f °·s²\n', ITAE);
fprintf('  Error máximo = %.2f °\n', Emax);
fprintf('========================================\n');