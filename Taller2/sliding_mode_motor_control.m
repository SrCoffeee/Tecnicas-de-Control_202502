%% Control Robusto de Modo Deslizante: Motor D.C. de Imán Permanente
% Jorge Sofrony - Universidad Nacional de Colombia
% Implementación y análisis comparativo
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

%% SIMULACIÓN 1: CONTROL SM BÁSICO (PASO I)
disp('=== SIMULACIÓN 1: Control SM Básico ===');

% Sistema con control SM básico
[t1, x1, u1, s1] = simulate_sm_basic(x0, xr, tf, a, beta1, phi, ...
    J_real, B_real, Km_real, R_real, M, l, g);

%% SIMULACIÓN 2: CONTROL SM CON CONTROL EQUIVALENTE (PASO II)
disp('=== SIMULACIÓN 2: Control SM con Control Equivalente ===');

% Sistema con control SM + equivalente
[t2, x2, u2, s2] = simulate_sm_equivalent(x0, xr, tf, a, beta2, phi, ...
    J, B, Km, R, M_nom, l_nom, g, ...  % Parámetros nominales para ueq
    J_real, B_real, Km_real, R_real, M, l, g);  % Parámetros reales

%% SIMULACIÓN 3: ANÁLISIS DE ROBUSTEZ (Multiple escenarios)
disp('=== SIMULACIÓN 3: Análisis de Robustez ===');

% Generar múltiples escenarios con diferentes variaciones
n_scenarios = 5;
scenarios_results = cell(n_scenarios, 2);

for i = 1:n_scenarios
    var_factor = (i-1)/(n_scenarios-1) * 0.4 - 0.2;  % De -20% a +20%
    
    % Parámetros con variación
    J_var = J * (1 + var_factor);
    B_var = B * (1 + var_factor*0.5);
    Km_var = Km * (1 + var_factor*0.8);
    R_var = R * (1 + var_factor*0.3);
    M_var = M_nom * (1 + var_factor);
    l_var = l_nom * (1 + var_factor*0.5);
    
    % Simular ambos controladores
    [t_rob1, x_rob1, ~, ~] = simulate_sm_basic(x0, xr, 3, a, beta1, phi, ...
        J_var, B_var, Km_var, R_var, M_var, l_var, g);
    
    [t_rob2, x_rob2, ~, ~] = simulate_sm_equivalent(x0, xr, 3, a, beta2, phi, ...
        J, B, Km, R, M_nom, l_nom, g, ...
        J_var, B_var, Km_var, R_var, M_var, l_var, g);
    
    scenarios_results{i,1} = x_rob1;
    scenarios_results{i,2} = x_rob2;
end

%% VISUALIZACIÓN DE RESULTADOS
% Configuración de figuras
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.5);

% FIGURA 1: Comparación de trayectorias
figure('Position', [100, 100, 1200, 800], 'Name', 'Comparación de Controladores SM');

% Posición angular
subplot(3,2,1);
plot(t1, rad2deg(x1(:,1)), 'b-', 'DisplayName', 'SM Básico');
hold on;
plot(t2, rad2deg(x2(:,1)), 'r--', 'DisplayName', 'SM + Equivalente');
plot([0 tf], [rad2deg(xr) rad2deg(xr)], 'k:', 'LineWidth', 1, 'DisplayName', 'Referencia');
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [°]');
title('Seguimiento de Posición Angular');
legend('Location', 'best');
ylim([-25, 35]);

% Velocidad angular
subplot(3,2,2);
plot(t1, rad2deg(x1(:,2)), 'b-', 'DisplayName', 'SM Básico');
hold on;
plot(t2, rad2deg(x2(:,2)), 'r--', 'DisplayName', 'SM + Equivalente');
grid on;
xlabel('Tiempo [s]');
ylabel('Velocidad [°/s]');
title('Velocidad Angular');
legend('Location', 'best');

% Señal de control
subplot(3,2,3);
plot(t1, u1, 'b-', 'DisplayName', 'SM Básico');
hold on;
plot(t2, u2, 'r--', 'DisplayName', 'SM + Equivalente');
grid on;
xlabel('Tiempo [s]');
ylabel('Voltaje [V]');
title('Señal de Control');
legend('Location', 'best');
ylim([-40, 40]);

% Superficie de deslizamiento
subplot(3,2,4);
plot(t1, s1, 'b-', 'DisplayName', 'SM Básico');
hold on;
plot(t2, s2, 'r--', 'DisplayName', 'SM + Equivalente');
plot([0 tf], [0 0], 'k:', 'LineWidth', 1);
grid on;
xlabel('Tiempo [s]');
ylabel('s');
title('Superficie de Deslizamiento');
legend('Location', 'best');

% Plano de fase
subplot(3,2,5);
plot(rad2deg(x1(:,1)), rad2deg(x1(:,2)), 'b-', 'DisplayName', 'SM Básico');
hold on;
plot(rad2deg(x2(:,1)), rad2deg(x2(:,2)), 'r--', 'DisplayName', 'SM + Equivalente');
plot(rad2deg(x0(1)), rad2deg(x0(2)), 'go', 'MarkerSize', 8, 'DisplayName', 'Inicio');
plot(rad2deg(xr), 0, 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Objetivo');
grid on;
xlabel('Posición [°]');
ylabel('Velocidad [°/s]');
title('Plano de Fase');
legend('Location', 'best');

% Análisis de chattering
subplot(3,2,6);
% Calcular la derivada numérica de la señal de control
du1 = diff(u1)./diff(t1);
du2 = diff(u2)./diff(t2);
plot(t1(2:end), abs(du1), 'b-', 'DisplayName', 'SM Básico');
hold on;
plot(t2(2:end), abs(du2), 'r--', 'DisplayName', 'SM + Equivalente');
grid on;
xlabel('Tiempo [s]');
ylabel('|du/dt| [V/s]');
title('Análisis de Chattering (Tasa de cambio del control)');
legend('Location', 'best');
ylim([0, 5000]);

% FIGURA 2: Análisis de Robustez
figure('Position', [150, 150, 1000, 600], 'Name', 'Análisis de Robustez');

subplot(2,2,1);
hold on;
colors = lines(n_scenarios);
for i = 1:n_scenarios
    var_percent = (i-1)/(n_scenarios-1) * 40 - 20;
    x_data = scenarios_results{i,1};
    plot(t_rob1(1:length(x_data)), rad2deg(x_data(:,1)), ...
        'Color', colors(i,:), 'DisplayName', sprintf('Var: %.0f%%', var_percent));
end
plot([0 3], [rad2deg(xr) rad2deg(xr)], 'k:', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [°]');
title('SM Básico - Respuesta bajo incertidumbres');
legend('Location', 'best');

subplot(2,2,2);
hold on;
for i = 1:n_scenarios
    var_percent = (i-1)/(n_scenarios-1) * 40 - 20;
    x_data = scenarios_results{i,2};
    plot(t_rob2(1:length(x_data)), rad2deg(x_data(:,1)), ...
        'Color', colors(i,:), 'DisplayName', sprintf('Var: %.0f%%', var_percent));
end
plot([0 3], [rad2deg(xr) rad2deg(xr)], 'k:', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [°]');
title('SM + Equivalente - Respuesta bajo incertidumbres');
legend('Location', 'best');

% Error cuadrático medio
subplot(2,2,3:4);
error_sm_basic = zeros(n_scenarios, 1);
error_sm_equiv = zeros(n_scenarios, 1);
variations = linspace(-20, 20, n_scenarios);

for i = 1:n_scenarios
    x_basic = scenarios_results{i,1};
    x_equiv = scenarios_results{i,2};
    error_sm_basic(i) = sqrt(mean((x_basic(:,1) - xr).^2));
    error_sm_equiv(i) = sqrt(mean((x_equiv(:,1) - xr).^2));
end

plot(variations, rad2deg(error_sm_basic), 'bo-', 'DisplayName', 'SM Básico');
hold on;
plot(variations, rad2deg(error_sm_equiv), 'rs-', 'DisplayName', 'SM + Equivalente');
grid on;
xlabel('Variación Paramétrica [%]');
ylabel('RMSE [°]');
title('Error RMS vs Incertidumbre Paramétrica');
legend('Location', 'best');

%% ANÁLISIS CUANTITATIVO
disp('===========================================');
disp('ANÁLISIS CUANTITATIVO DE DESEMPEÑO');
disp('===========================================');

% Tiempo de establecimiento (2% de banda)
threshold = 0.02 * abs(xr - x0(1));
idx1 = find(abs(x1(:,1) - xr) < threshold, 1, 'first');
idx2 = find(abs(x2(:,1) - xr) < threshold, 1, 'first');

ts1 = t1(idx1);
ts2 = t2(idx2);

% Sobreimpulso
overshoot1 = max((x1(:,1) - xr)/abs(xr - x0(1))) * 100;
overshoot2 = max((x2(:,1) - xr)/abs(xr - x0(1))) * 100;

% Esfuerzo de control
control_effort1 = trapz(t1, u1.^2);
control_effort2 = trapz(t2, u2.^2);

% Índice de chattering
chattering1 = sum(abs(diff(u1))) / tf;
chattering2 = sum(abs(diff(u2))) / tf;

% Mostrar resultados
fprintf('\n%-30s %15s %15s\n', 'Métrica', 'SM Básico', 'SM+Equivalente');
fprintf('%-30s %15s %15s\n', '-------', '---------', '--------------');
fprintf('%-30s %15.3f %15.3f\n', 'Tiempo establecimiento [s]', ts1, ts2);
fprintf('%-30s %15.2f %15.2f\n', 'Sobreimpulso [%]', overshoot1, overshoot2);
fprintf('%-30s %15.1f %15.1f\n', 'Esfuerzo de control [V²s]', control_effort1, control_effort2);
fprintf('%-30s %15.1f %15.1f\n', 'Índice de chattering [V/s]', chattering1, chattering2);
fprintf('%-30s %15.1f %15.1f\n', 'Control máximo [V]', max(abs(u1)), max(abs(u2)));
fprintf('%-30s %15.3f %15.3f\n', 'Error RMS final [°]', ...
    rad2deg(sqrt(mean((x1(end-100:end,1)-xr).^2))), ...
    rad2deg(sqrt(mean((x2(end-100:end,1)-xr).^2))));

% Reducción porcentual del chattering
reduction = (chattering1 - chattering2) / chattering1 * 100;
fprintf('\n');
fprintf('Reducción de chattering con control equivalente: %.1f%%\n', reduction);

%% FUNCIONES DE SIMULACIÓN

function [t, x, u, s] = simulate_sm_basic(x0, xr, tf, a, beta, phi, J, B, Km, R, M, l, g)
    % Simulación del control SM básico
    
    % Función del sistema
    dynamics = @(t, x, u) [
        x(2);
        -B/J * x(2) - M*g*l/J * sin(x(1)) + Km/(J*R) * u
    ];
    
    % Control SM básico
    control = @(x) -beta * sat((a*(x(1)-xr) + x(2))/phi);
    
    % Simulación con ODE45
    dt = 0.001;
    t = 0:dt:tf;
    x = zeros(length(t), 2);
    u = zeros(length(t), 1);
    s = zeros(length(t), 1);
    
    x(1,:) = x0';
    
    for i = 1:length(t)-1
        % Calcular control
        u(i) = control(x(i,:));
        
        % Calcular superficie
        s(i) = a*(x(i,1)-xr) + x(i,2);
        
        % Integración Runge-Kutta
        k1 = dynamics(t(i), x(i,:)', u(i));
        k2 = dynamics(t(i)+dt/2, x(i,:)'+k1*dt/2, u(i));
        k3 = dynamics(t(i)+dt/2, x(i,:)'+k2*dt/2, u(i));
        k4 = dynamics(t(i)+dt, x(i,:)'+k3*dt, u(i));
        
        x(i+1,:) = x(i,:) + dt/6 * (k1 + 2*k2 + 2*k3 + k4)';
    end
    
    u(end) = control(x(end,:));
    s(end) = a*(x(end,1)-xr) + x(end,2);
end

function [t, x, u, s] = simulate_sm_equivalent(x0, xr, tf, a, beta, phi, ...
    J_nom, B_nom, Km_nom, R_nom, M_nom, l_nom, g, ...
    J_real, B_real, Km_real, R_real, M_real, l_real, g_real)
    % Simulación del control SM con control equivalente
    
    % Función del sistema real
    dynamics = @(t, x, u) [
        x(2);
        -B_real/J_real * x(2) - M_real*g_real*l_real/J_real * sin(x(1)) + Km_real/(J_real*R_real) * u
    ];
    
    % Control equivalente (usando parámetros nominales)
    ueq = @(x) (-a*x(2) + B_nom/J_nom*x(2) + M_nom*g*l_nom/J_nom*sin(x(1))) / (Km_nom/(J_nom*R_nom));
    
    % Control total: equivalente + switching
    control = @(x) ueq(x) - beta * sat((a*(x(1)-xr) + x(2))/phi);
    
    % Simulación
    dt = 0.001;
    t = 0:dt:tf;
    x = zeros(length(t), 2);
    u = zeros(length(t), 1);
    s = zeros(length(t), 1);
    
    x(1,:) = x0';
    
    for i = 1:length(t)-1
        % Calcular control
        u(i) = control(x(i,:));
        
        % Calcular superficie
        s(i) = a*(x(i,1)-xr) + x(i,2);
        
        % Integración Runge-Kutta
        k1 = dynamics(t(i), x(i,:)', u(i));
        k2 = dynamics(t(i)+dt/2, x(i,:)'+k1*dt/2, u(i));
        k3 = dynamics(t(i)+dt/2, x(i,:)'+k2*dt/2, u(i));
        k4 = dynamics(t(i)+dt, x(i,:)'+k3*dt, u(i));
        
        x(i+1,:) = x(i,:) + dt/6 * (k1 + 2*k2 + 2*k3 + k4)';
    end
    
    u(end) = control(x(end,:));
    s(end) = a*(x(end,1)-xr) + x(end,2);
end

function y = sat(x)
    % Función de saturación para reducir chattering
    if abs(x) <= 1
        y = x;
    else
        y = sign(x);
    end
end
