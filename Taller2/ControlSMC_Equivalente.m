%% Simulación Control Equivalente Motor DC - Modo Deslizante
% Control Robusto: Motor D.C. de imán permanente
% Paso II: Control equivalente nominal con u_sm = 0

clear all; close all; clc;

%% Parámetros nominales del motor DC
R = 8.0;              % Resistencia (Ohms)
J = 9.85e-3;          % Momento de inercia (kg-m^2)
B = 2.52e-3;          % Constante de fricción (Nm-s/rad)
Km = 1.57e-2;         % Constante de motor (Nm/Amp)

% Parámetros de la carga
M = 0.3;              % Masa nominal (kg) - 300g
L = 0.05;             % Distancia al eje (m) - 5cm
g = 9.81;             % Gravedad (m/s^2)

% Parámetro de diseño de la superficie deslizante
a = 5.0;              % Constante positiva (puede ajustarse)

% Referencia (set-point)
x_r = deg2rad(20);    % 20 grados en radianes

%% Condiciones iniciales
% IMPORTANTE: Para que u_eq funcione solo, debemos iniciar en la superficie
% s = a*x_e + x_2 = 0, entonces x_2 = -a*x_e
x1_0 = 0;                        % Posición inicial (rad)
x_e_0 = x1_0 - x_r;              % Error inicial
x2_0 = -a*x_e_0;                 % Velocidad inicial para estar en s=0
x0 = [x1_0; x2_0];               % [posición (rad); velocidad (rad/s)]

fprintf('Condiciones iniciales (para estar en superficie s=0):\n');
fprintf('  Posición inicial: %.2f°\n', rad2deg(x1_0));
fprintf('  Velocidad inicial: %.4f rad/s\n', x2_0);

%% Tiempo de simulación
t_final = 5;
tspan = [0 t_final];

%% Simulación usando ODE45
[t, x] = ode45(@(t,x) motor_dynamics(t, x, x_r, R, J, B, Km, M, L, g, a), tspan, x0);

x1 = x(:,1);          % Posición
x2 = x(:,2);          % Velocidad

%% Calcular señal de control y superficie deslizante
n = length(t);
u_control = zeros(n, 1);
s_surface = zeros(n, 1);

for i = 1:n
    u_control(i) = control_equivalente(x1(i), x2(i), x_r, R, J, B, Km, M, L, g, a);
    x_e = x1(i) - x_r;
    s_surface(i) = a*x_e + x2(i);
end

%% Gráficas
figure('Position', [100 100 1200 800]);

% Subplot 1: Posición vs tiempo
subplot(2,2,1);
plot(t, rad2deg(x1), 'b', 'LineWidth', 2);
hold on;
plot(t, rad2deg(x_r)*ones(size(t)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('Posición (grados)', 'FontSize', 11);
title('Posición Angular', 'FontSize', 12, 'FontWeight', 'bold');
legend('Posición', 'Referencia', 'Location', 'best');

% Subplot 2: Velocidad vs tiempo
subplot(2,2,2);
plot(t, x2, 'g', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('Velocidad (rad/s)', 'FontSize', 11);
title('Velocidad Angular', 'FontSize', 12, 'FontWeight', 'bold');

% Subplot 3: Señal de control
subplot(2,2,3);
plot(t, u_control, 'm', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('Voltaje (V)', 'FontSize', 11);
title('Señal de Control u_{eq} (u_{sm} = 0)', 'FontSize', 12, 'FontWeight', 'bold');

% Subplot 4: Superficie deslizante
subplot(2,2,4);
plot(t, s_surface, 'c', 'LineWidth', 2);
hold on;
yline(0, 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('s', 'FontSize', 11);
title('Superficie Deslizante s = ax_e + x_2', 'FontSize', 12, 'FontWeight', 'bold');

%% Imprimir información
fprintf('============================================================\n');
fprintf('SIMULACIÓN: Control Equivalente Nominal (u_sm = 0)\n');
fprintf('============================================================\n');

fprintf('\n--- FUNCIÓN DE CONTROL EQUIVALENTE ---\n');
fprintf('\nForma general:\n');
fprintf('  u_eq = -(a*x2 + h_hat(x)) / g_hat(x)\n');
fprintf('\nDonde:\n');
fprintf('  h_hat(x) = -(B/J)*x2 - (M*g*L*sin(x1))/J\n');
fprintf('  g_hat(x) = Km/(J*R)\n');
fprintf('\nSustituyendo valores nominales:\n');
fprintf('  h_hat(x) = -(%.2e/%.2e)*x2 - (%.1f*%.2f*%.2f*sin(x1))/%.2e\n', B, J, M, g, L, J);
fprintf('  h_hat(x) = -%.4f*x2 - %.4f*sin(x1)\n', B/J, (M*g*L)/J);
fprintf('  g_hat(x) = %.2e/(%.2e*%.1f) = %.6f\n', Km, J, R, Km/(J*R));
fprintf('\nPor lo tanto:\n');
fprintf('  u_eq = -[%.2f*x2 + (-%.4f*x2 - %.4f*sin(x1))] / %.6f\n', a, B/J, (M*g*L)/J, Km/(J*R));
fprintf('  u_eq = -[%.4f*x2 - %.4f*sin(x1)] / %.6f\n', a - B/J, (M*g*L)/J, Km/(J*R));
fprintf('  u_eq = %.2f*x2 + %.2f*sin(x1)\n', -(a - B/J)/(Km/(J*R)), (M*g*L/J)/(Km/(J*R)));
fprintf('\n');
fprintf('\nParámetros del sistema:\n');
fprintf('  R  = %.2f Ω\n', R);
fprintf('  J  = %.2f × 10^-3 kg-m^2\n', J*1e3);
fprintf('  B  = %.2f × 10^-3 Nm-s/rad\n', B*1e3);
fprintf('  Km = %.2f × 10^-2 Nm/Amp\n', Km*1e2);
fprintf('  M  = %.0f g\n', M*1e3);
fprintf('  L  = %.0f cm\n', L*1e2);
fprintf('\nParámetro de diseño:\n');
fprintf('  a = %.2f\n', a);
fprintf('\nReferencia: %.1f°\n', rad2deg(x_r));
fprintf('\nError final: %.4f°\n', rad2deg(x1(end) - x_r));
fprintf('Valor máximo de control: %.2f V\n', max(abs(u_control)));

% Calcular tiempo de establecimiento (criterio 2%)
error_porcentual = abs((x1 - x_r)/x_r * 100);
idx_settled = find(error_porcentual < 2, 1, 'first');
if ~isempty(idx_settled)
    fprintf('Tiempo de establecimiento (2%%): %.3f s\n', t(idx_settled));
else
    fprintf('Tiempo de establecimiento (2%%): No alcanzado\n');
end
fprintf('============================================================\n');

%% Función: Dinámica del motor DC
function dx = motor_dynamics(t, x, x_r, R, J, B, Km, M, L, g, a)
    x1 = x(1);  % Posición
    x2 = x(2);  % Velocidad
    
    % Calcular control equivalente
    u = control_equivalente(x1, x2, x_r, R, J, B, Km, M, L, g, a);
    
    % Ecuaciones del sistema
    dx1 = x2;
    dx2 = -(B/J)*x2 - (M*g*L*sin(x1))/J + (Km/(J*R))*u;
    
    dx = [dx1; dx2];
end

%% Función: Control equivalente
function u_eq = control_equivalente(x1, x2, x_r, R, J, B, Km, M, L, g, a)
    % Control equivalente nominal
    % u_eq = -(ax_2 + h_hat(x)) / g_hat(x)
    
    % h_hat(x) = -(B/J)*x2 - (M*g*L*sin(x1))/J
    % g_hat(x) = Km/(J*R)
    
    h_hat = -(B/J)*x2 - (M*g*L*sin(x1))/J;
    g_hat = Km/(J*R);
    
    u_eq = -(a*x2 + h_hat) / g_hat;
end
%% Motor D.C. con parámetros reales diferentes a nominales
% Demostración de delta(x) y diseño de beta

%% Parámetros REALES (sistema real con variación del 20%)
variacion = 0.20; % 20% de variación

R_real = R* (1 + variacion*0.5);      % +10%
J_real = J * (1 - variacion*0.8);      % -16%
B_real = B * (1 + variacion*0.6);      % +12%
Km_real = Km * (1 - variacion*0.4);    % -8%
M_real = M * (1 + variacion);          % +20%
L_real = L * (1 + variacion*0.5);      % +10%

fprintf('============================================================\n');
fprintf('PARÁMETROS DEL SISTEMA\n');
fprintf('============================================================\n');
fprintf('Parámetro\t\tNominal\t\tReal\t\tVariación\n');
fprintf('------------------------------------------------------------\n');
fprintf('R (Ω)\t\t\t%.3f\t\t%.3f\t\t%+.1f%%\n', R, R_real, (R_real/R-1)*100);
fprintf('J (kg-m²)\t\t%.2e\t%.2e\t%+.1f%%\n', J, J_real, (J_real/J-1)*100);
fprintf('B (Nm-s/rad)\t%.2e\t%.2e\t%+.1f%%\n', B, B_real, (B_real/B-1)*100);
fprintf('Km (Nm/Amp)\t\t%.2e\t%.2e\t%+.1f%%\n', Km, Km_real, (Km_real/Km-1)*100);
fprintf('M (kg)\t\t\t%.3f\t\t%.3f\t\t%+.1f%%\n', M, M_real, (M_real/M-1)*100);
fprintf('L (m)\t\t\t%.3f\t\t%.3f\t\t%+.1f%%\n', L, L_real, (L_real/L-1)*100);
fprintf('============================================================\n\n');

%% Parámetros de diseño
a = 5.0;              % Constante de la superficie deslizante
x_r = deg2rad(25);    % Referencia: 25 grados

%% Cálculo de beta
% Restricciones: |x1| <= pi, |x2| <= pi/2
x1_max = pi;
x2_max = pi/2;

% Calcular cota superior de |delta(x)|/g(x)
% Necesitamos evaluar delta en el peor caso

% Cotas para cada término de delta(x)
g_real_min = Km_real/(J_real*R_real) * 0.8;  % Cota inferior conservadora
g_real_max = Km_real/(J_real*R_real) * 1.2;  % Cota superior conservadora
g_hat_val = Km/(J*R);

% Término 1: a*(1 - g(x)/g_hat(x))*x2
term1_max = abs(a * (1 - g_real_min/g_hat_val) * x2_max);

% Término 2: h(x)
h_max = abs(-B_real/J_real * x2_max) + abs(M_real*g*L_real*sin(x1_max)/J_real);

% Término 3: g(x)/g_hat(x) * h_hat(x)
h_hat_max = abs(-B/J * x2_max) + abs(M*g*L*sin(x1_max)/J);
term3_max = abs(g_real_max/g_hat_val * h_hat_max);

delta_max = term1_max + h_max + term3_max;
beta = delta_max / g_real_min * 1.5;  % Factor 1.5 de seguridad

fprintf('============================================================\n');
fprintf('DISEÑO DEL CONTROLADOR\n');
fprintf('============================================================\n');
fprintf('Parámetro a = %.2f\n', a);
fprintf('Referencia = %.1f°\n', rad2deg(x_r));
fprintf('\nCálculo de beta:\n');
fprintf('  |delta(x)|_max ≈ %.4f\n', delta_max);
fprintf('  g(x)_min ≈ %.6f\n', g_real_min);
fprintf('  beta_min = %.4f\n', delta_max / g_real_min);
fprintf('  beta (con factor 1.5) = %.4f\n', beta);
fprintf('============================================================\n\n');

%% Condiciones iniciales
x1_0 = deg2rad(0);    % Posición inicial
x2_0 = 0;             % Velocidad inicial
x0 = [x1_0; x2_0];

%% Tiempo de simulación
t_final = 3;
tspan = [0 t_final];

%% Parámetros para pasar a la función
params_real = [R_real, J_real, B_real, Km_real, M_real, L_real];
params_nom = [R, J, B, Km, M, L];

%% Simulación
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t, x] = ode45(@(t,x) motor_dynamics_completo(t, x, x_r, params_real, params_nom, g, a, beta), ...
               tspan, x0, options);

x1 = x(:,1);
x2 = x(:,2);

%% Calcular señales de control y superficie
n = length(t);
u_eq = zeros(n, 1);
u_sm = zeros(n, 1);
u_total = zeros(n, 1);
s_surface = zeros(n, 1);
delta_x = zeros(n, 1);

for i = 1:n
    [u_eq(i), u_sm(i), u_total(i), s_surface(i), delta_x(i)] = ...
        calcular_control(x1(i), x2(i), x_r, params_real, params_nom, g, a, beta);
end

%% Gráficas
figure('Position', [50 50 1400 900]);

% Subplot 1: Posición
subplot(3,3,1);
plot(t, rad2deg(x1), 'b', 'LineWidth', 2);
hold on;
plot(t, rad2deg(x_r)*ones(size(t)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Posición (°)');
title('Posición Angular');
legend('Posición', 'Referencia', 'Location', 'best');

% Subplot 2: Velocidad
subplot(3,3,2);
plot(t, x2, 'g', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Velocidad (rad/s)');
title('Velocidad Angular');

% Subplot 3: Error
subplot(3,3,3);
plot(t, rad2deg(x1 - x_r), 'k', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Error (°)');
title('Error de Posición');

% Subplot 4: Superficie deslizante
subplot(3,3,4);
plot(t, s_surface, 'c', 'LineWidth', 2);
hold on;
yline(0, 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo (s)');
ylabel('s');
title('Superficie Deslizante');

% Subplot 5: Control total
subplot(3,3,5);
plot(t, u_total, 'm', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Control Total u = u_{eq} + u_{sm}');

% Subplot 6: Control equivalente
subplot(3,3,6);
plot(t, u_eq, 'Color', [0.8 0.4 0], 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Control Equivalente u_{eq}');

% Subplot 7: Control conmutado
subplot(3,3,7);
plot(t, u_sm, 'r', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Control Conmutado u_{sm}');

% Subplot 8: Delta(x)
subplot(3,3,8);
plot(t, delta_x, 'Color', [0.5 0 0.5], 'LineWidth', 2);
hold on;
yline(beta*g_real_min, 'r--', 'LineWidth', 1.5);
yline(-beta*g_real_min, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('\delta(x)');
title('\delta(x) vs límites \pm\beta g(x)');
legend('\delta(x)', '\pm\beta g(x)_{min}');

% Subplot 9: Plano de fase
subplot(3,3,9);
plot(rad2deg(x1), x2, 'b', 'LineWidth', 1.5);
hold on;
plot(rad2deg(x1(1)), x2(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(rad2deg(x_r), 0, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
% Dibujar superficie deslizante
x1_line = linspace(-30, 30, 100);
x2_line = -a * (deg2rad(x1_line) - x_r);
plot(x1_line, x2_line, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Posición (°)');
ylabel('Velocidad (rad/s)');
title('Plano de Fase');
legend('Trayectoria', 'Inicio', 'Referencia', 's=0');
axis equal;
xlim([-35 35]);

%% Estadísticas
fprintf('============================================================\n');
fprintf('RESULTADOS DE LA SIMULACIÓN\n');
fprintf('============================================================\n');
fprintf('Error final: %.4f° (%.2e rad)\n', rad2deg(x1(end) - x_r), x1(end) - x_r);
fprintf('Tiempo de establecimiento (2%%): ');
error_pct = abs((x1 - x_r)/x_r * 100);
idx_settled = find(error_pct < 2, 1, 'first');
if ~isempty(idx_settled)
    fprintf('%.3f s\n', t(idx_settled));
else
    fprintf('No alcanzado\n');
end
fprintf('\nSeñales de control:\n');
fprintf('  |u_total|_max = %.2f V\n', max(abs(u_total)));
fprintf('  |u_eq|_max = %.2f V\n', max(abs(u_eq)));
fprintf('  |u_sm|_max = %.2f V\n', max(abs(u_sm)));
fprintf('\n|delta(x)|_max simulado = %.4f\n', max(abs(delta_x)));
fprintf('beta*g(x)_min = %.4f\n', beta*g_real_min);
fprintf('Condición cumplida: %s\n', iif(max(abs(delta_x)) < beta*g_real_min, 'SÍ ✓', 'NO ✗'));
fprintf('============================================================\n');

%% Funciones auxiliares
function dx = motor_dynamics_completo(t, x, x_r, params_real, params_nom, g, a, beta)
    x1 = x(1);
    x2 = x(2);
    
    R_real = params_real(1);
    J_real = params_real(2);
    B_real = params_real(3);
    Km_real = params_real(4);
    M_real = params_real(5);
    L_real = params_real(6);
    
    % Calcular control
    [~, ~, u_total, ~, ~] = calcular_control(x1, x2, x_r, params_real, params_nom, g, a, beta);
    
    % Dinámica real del sistema
    dx1 = x2;
    dx2 = -(B_real/J_real)*x2 - (M_real*g*L_real*sin(x1))/J_real + (Km_real/(J_real*R_real))*u_total;
    
    dx = [dx1; dx2];
end

function [u_eq, u_sm, u_total, s, delta] = calcular_control(x1, x2, x_r, params_real, params_nom, g, a, beta)
    % Parámetros nominales
    R_hat = params_nom(1);
    J_hat = params_nom(2);
    B_hat = params_nom(3);
    Km_hat = params_nom(4);
    M_hat = params_nom(5);
    L_hat = params_nom(6);
    
    % Parámetros reales
    R_real = params_real(1);
    J_real = params_real(2);
    B_real = params_real(3);
    Km_real = params_real(4);
    M_real = params_real(5);
    L_real = params_real(6);
    
    % Error
    x_e = x1 - x_r;
    
    % Superficie deslizante
    s = a*x_e + x2;
    
    % Funciones nominales
    h_hat = -(B_hat/J_hat)*x2 - (M_hat*g*L_hat*sin(x1))/J_hat;
    g_hat = Km_hat/(J_hat*R_hat);
    
    % Control equivalente (usa parámetros nominales)
    u_eq = -(a*x2 + h_hat) / g_hat;
    
    % Control conmutado
    u_sm = -beta * sign(s);
    
    % Control total
    u_total = u_eq + u_sm;
    
    % Calcular delta(x) para análisis (usa parámetros reales)
    h_real = -(B_real/J_real)*x2 - (M_real*g*L_real*sin(x1))/J_real;
    g_real = Km_real/(J_real*R_real);
    
    delta = a*(1 - g_real/g_hat)*x2 + h_real - (g_real/g_hat)*h_hat;
end

function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
%% %% ========================================================================
%% SECCIÓN 3: COMPARACIÓN DE CONTROLADORES
%% Controlador 1: u = -beta*sign(s) [SIN control equivalente]
%% Controlador 2: u = u_eq - beta*sign(s) [CON control equivalente]
%% ========================================================================

fprintf('\n\n');
fprintf('################################################################\n');
fprintf('##   COMPARACIÓN DE CONTROLADORES POR MODO DESLIZANTE        ##\n');
fprintf('################################################################\n\n');

%% Parámetros de diseño para comparación
a_comp = 8.0;              % Valor de 'a' para comparación
x_r_comp = deg2rad(25);    % Referencia: 25 grados

%% DISEÑO DE BETA para Controlador 1 (sin u_eq)
fprintf('------------------------------------------------------------\n');
fprintf('CONTROLADOR 1: u = -beta1*sign(s)  [SIN u_eq]\n');
fprintf('------------------------------------------------------------\n');

% Para este controlador: dot(s) = ax2 + h(x) + g(x)*u
% Necesitamos: |ax2 + h(x)|/g(x) < beta1

% Peor caso de h(x) con parámetros reales
h_max_ctrl1 = abs(B_real/J_real * x2_max) + abs(M_real*g*L_real/J_real);
numerador_max_ctrl1 = a_comp*x2_max + h_max_ctrl1;

beta1 = (numerador_max_ctrl1 / g_real_min) * 1.3;  % Factor de seguridad 1.3

fprintf('Análisis de la condición |ax2 - f|/g(x) < beta1:\n');
fprintf('  |ax2|_max = %.2f * %.4f = %.4f\n', a_comp, x2_max, a_comp*x2_max);
fprintf('  |f(x)|_max = |-(B/J)*x2 - (Mgl/J)*sin(x1)|_max = %.4f\n', h_max_ctrl1);
fprintf('  Numerador_max = %.4f\n', numerador_max_ctrl1);
fprintf('  g(x)_min = %.6f\n', g_real_min);
fprintf('  beta1_min = %.4f\n', numerador_max_ctrl1 / g_real_min);
fprintf('  beta1 (con factor 1.3) = %.4f\n\n', beta1);

%% DISEÑO DE BETA para Controlador 2 (con u_eq)
fprintf('------------------------------------------------------------\n');
fprintf('CONTROLADOR 2: u = u_eq - beta2*sign(s)  [CON u_eq]\n');
fprintf('------------------------------------------------------------\n');

% Ya calculamos beta en la sección anterior, lo usamos
beta2 = beta;

fprintf('Análisis de la condición |delta(x)|/g(x) < beta2:\n');
fprintf('  Término 1: a*(1-g/g_hat)*x2_max = %.4f\n', term1_max);
fprintf('  Término 2: |h(x)|_max = %.4f\n', h_max);
fprintf('  Término 3: (g/g_hat)*|h_hat|_max = %.4f\n', term3_max);
fprintf('  |delta(x)|_max = %.4f\n', delta_max);
fprintf('  g(x)_min = %.6f\n', g_real_min);
fprintf('  beta2_min = %.4f\n', delta_max / g_real_min);
fprintf('  beta2 (con factor 1.5) = %.4f\n\n', beta2);

fprintf('------------------------------------------------------------\n');
fprintf('REDUCCIÓN DE BETA:\n');
fprintf('  beta1 = %.4f V\n', beta1);
fprintf('  beta2 = %.4f V\n', beta2);
fprintf('  Reducción: %.1f%%\n', (beta1-beta2)/beta1*100);
fprintf('  Beneficio: Menor chattering y menor esfuerzo de control\n');
fprintf('------------------------------------------------------------\n\n');

%% SIMULACIONES COMPARATIVAS

% Condiciones iniciales (desde reposo)
x0_comp = [deg2rad(0); 0];

% Tiempo de simulación
t_final_comp = 3;
tspan_comp = [0 t_final_comp];
options_comp = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

fprintf('Ejecutando simulaciones...\n');

%% Simulación Controlador 1
fprintf('  - Simulando Controlador 1 (sin u_eq)...\n');
[t_ctrl1, x_ctrl1] = ode45(@(t,x) dynamics_ctrl1(t, x, x_r_comp, params_real, g, a_comp, beta1), ...
                            tspan_comp, x0_comp, options_comp);

%% Simulación Controlador 2
fprintf('  - Simulando Controlador 2 (con u_eq)...\n');
[t_ctrl2, x_ctrl2] = ode45(@(t,x) dynamics_ctrl2(t, x, x_r_comp, params_real, params_nom, g, a_comp, beta2), ...
                            tspan_comp, x0_comp, options_comp);

fprintf('  - Simulaciones completadas.\n\n');

%% Calcular señales de control para análisis
n_ctrl1 = length(t_ctrl1);
n_ctrl2 = length(t_ctrl2);

% Controlador 1
u_ctrl1 = zeros(n_ctrl1, 1);
s_ctrl1 = zeros(n_ctrl1, 1);
for i = 1:n_ctrl1
    x_e_temp = x_ctrl1(i,1) - x_r_comp;
    s_ctrl1(i) = a_comp*x_e_temp + x_ctrl1(i,2);
    u_ctrl1(i) = -beta1 * sign(s_ctrl1(i));
end

% Controlador 2
u_ctrl2_eq = zeros(n_ctrl2, 1);
u_ctrl2_sm = zeros(n_ctrl2, 1);
u_ctrl2_total = zeros(n_ctrl2, 1);
s_ctrl2 = zeros(n_ctrl2, 1);
for i = 1:n_ctrl2
    x_e_temp = x_ctrl2(i,1) - x_r_comp;
    s_ctrl2(i) = a_comp*x_e_temp + x_ctrl2(i,2);
    
    % Control equivalente
    h_hat_temp = -(B/J)*x_ctrl2(i,2) - (M*g*L*sin(x_ctrl2(i,1)))/J;
    g_hat_temp = Km/(J*R);
    u_ctrl2_eq(i) = -(a_comp*x_ctrl2(i,2) + h_hat_temp) / g_hat_temp;
    
    % Control conmutado
    u_ctrl2_sm(i) = -beta2 * sign(s_ctrl2(i));
    
    % Control total
    u_ctrl2_total(i) = u_ctrl2_eq(i) + u_ctrl2_sm(i);
end

%% Estimar frecuencia de conmutación
freq_ctrl1 = estimate_switching_freq(u_ctrl1, t_ctrl1);
freq_ctrl2 = estimate_switching_freq(u_ctrl2_sm, t_ctrl2);

%% GRÁFICAS COMPARATIVAS
figure('Position', [100 50 1400 900]);
sgtitle('COMPARACIÓN DE CONTROLADORES POR MODO DESLIZANTE', 'FontSize', 14, 'FontWeight', 'bold');

% Subplot 1: Posición
subplot(3,3,1);
plot(t_ctrl1, rad2deg(x_ctrl1(:,1)), 'b', 'LineWidth', 2.5); hold on;
plot(t_ctrl2, rad2deg(x_ctrl2(:,1)), 'r', 'LineWidth', 2.5);
plot(t_ctrl1, rad2deg(x_r_comp)*ones(size(t_ctrl1)), 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Posición (°)');
title('Posición Angular');
legend('Ctrl 1 (sin u_{eq})', 'Ctrl 2 (con u_{eq})', 'Referencia', 'Location', 'southeast');
xlim([0 t_final_comp]);

% Subplot 2: Error de posición
subplot(3,3,2);
plot(t_ctrl1, rad2deg(x_ctrl1(:,1) - x_r_comp), 'b', 'LineWidth', 2); hold on;
plot(t_ctrl2, rad2deg(x_ctrl2(:,1) - x_r_comp), 'r', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo (s)');
ylabel('Error (°)');
title('Error de Posición');
legend('Ctrl 1', 'Ctrl 2', 'Location', 'best');
xlim([0 t_final_comp]);

% Subplot 3: Velocidad
subplot(3,3,3);
plot(t_ctrl1, x_ctrl1(:,2), 'b', 'LineWidth', 2); hold on;
plot(t_ctrl2, x_ctrl2(:,2), 'r', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
grid on;
xlabel('Tiempo (s)');
ylabel('Velocidad (rad/s)');
title('Velocidad Angular');
legend('Ctrl 1', 'Ctrl 2', 'Location', 'best');
xlim([0 t_final_comp]);

% Subplot 4: Superficie deslizante
subplot(3,3,4);
plot(t_ctrl1, s_ctrl1, 'b', 'LineWidth', 2); hold on;
plot(t_ctrl2, s_ctrl2, 'r', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('s');
title('Superficie Deslizante');
legend('Ctrl 1', 'Ctrl 2', 'Location', 'best');
xlim([0 t_final_comp]);

% Subplot 5: Control total (vista completa)
subplot(3,3,5);
plot(t_ctrl1, u_ctrl1, 'b', 'LineWidth', 1.5); hold on;
plot(t_ctrl2, u_ctrl2_total, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Señal de Control Total');
legend('Ctrl 1: u = -\beta sign(s)', 'Ctrl 2: u = u_{eq} - \beta sign(s)', 'Location', 'best');
xlim([0 t_final_comp]);

% Subplot 6: Zoom del control (primeros 0.5s)
subplot(3,3,6);
idx_zoom1 = find(t_ctrl1 <= 0.5);
idx_zoom2 = find(t_ctrl2 <= 0.5);
plot(t_ctrl1(idx_zoom1), u_ctrl1(idx_zoom1), 'b', 'LineWidth', 1.5); hold on;
plot(t_ctrl2(idx_zoom2), u_ctrl2_total(idx_zoom2), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Control (Zoom: 0-0.5s)');
legend('Ctrl 1', 'Ctrl 2', 'Location', 'best');
xlim([0 0.5]);

% Subplot 7: Componentes de Ctrl 2
subplot(3,3,7);
plot(t_ctrl2, u_ctrl2_eq, 'Color', [0.8 0.4 0], 'LineWidth', 2); hold on;
plot(t_ctrl2, u_ctrl2_sm, 'g', 'LineWidth', 2);
plot(t_ctrl2, u_ctrl2_total, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Componentes de Controlador 2');
legend('u_{eq}', 'u_{sm}', 'u_{total}', 'Location', 'best');
xlim([0 t_final_comp]);

% Subplot 8: Plano de fase
subplot(3,3,8);
plot(rad2deg(x_ctrl1(:,1)), x_ctrl1(:,2), 'b', 'LineWidth', 2); hold on;
plot(rad2deg(x_ctrl2(:,1)), x_ctrl2(:,2), 'r', 'LineWidth', 2);
plot(rad2deg(x0_comp(1)), x0_comp(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(rad2deg(x_r_comp), 0, 'k*', 'MarkerSize', 12, 'LineWidth', 2);
% Superficie deslizante s=0
x1_surf = linspace(-5, 30, 100);
x2_surf = -a_comp * (deg2rad(x1_surf) - x_r_comp);
plot(x1_surf, x2_surf, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Posición (°)');
ylabel('Velocidad (rad/s)');
title('Plano de Fase');
legend('Ctrl 1', 'Ctrl 2', 'Inicio', 'Objetivo', 's=0', 'Location', 'best');

% Subplot 9: Comparación de magnitudes
subplot(3,3,9);
categorias = categorical({'|u|_{max}', '|u|_{prom}'});
valores_comp = [max(abs(u_ctrl1)), max(abs(u_ctrl2_total)); 
                mean(abs(u_ctrl1)), mean(abs(u_ctrl2_total))];
b = bar(categorias, valores_comp);
b(1).FaceColor = 'b';
b(2).FaceColor = 'r';
ylabel('Voltaje (V)');
title('Magnitud del Control');
legend('Ctrl 1', 'Ctrl 2', 'Location', 'best');
grid on;

%% TABLA DE RESULTADOS COMPARATIVOS
fprintf('============================================================\n');
fprintf('TABLA DE RESULTADOS COMPARATIVOS (a = %.1f)\n', a_comp);
fprintf('============================================================\n');
fprintf('%-35s %10s %10s %10s\n', 'Métrica', 'Ctrl 1', 'Ctrl 2', 'Mejora');
fprintf('------------------------------------------------------------\n');

% Error final
err_final_ctrl1 = rad2deg(abs(x_ctrl1(end,1) - x_r_comp));
err_final_ctrl2 = rad2deg(abs(x_ctrl2(end,1) - x_r_comp));
fprintf('%-35s %10.4f %10.4f %9s\n', 'Error final (°)', err_final_ctrl1, err_final_ctrl2, '-');

% Tiempo de establecimiento (2%)
ts_ctrl1 = settling_time(t_ctrl1, x_ctrl1(:,1), x_r_comp, 0.02);
ts_ctrl2 = settling_time(t_ctrl2, x_ctrl2(:,1), x_r_comp, 0.02);
if ts_ctrl1 < inf && ts_ctrl2 < inf
    mejora_ts = sprintf('%.1f%%', (ts_ctrl1-ts_ctrl2)/ts_ctrl1*100);
else
    mejora_ts = '-';
end
fprintf('%-35s %10.3f %10.3f %10s\n', 'Tiempo establecimiento 2% (s)', ts_ctrl1, ts_ctrl2, mejora_ts);

% Control máximo
u_max_ctrl1 = max(abs(u_ctrl1));
u_max_ctrl2 = max(abs(u_ctrl2_total));
mejora_umax = sprintf('%.1f%%', (u_max_ctrl1-u_max_ctrl2)/u_max_ctrl1*100);
fprintf('%-35s %10.2f %10.2f %10s\n', '|u|_max (V)', u_max_ctrl1, u_max_ctrl2, mejora_umax);

% Control promedio
u_prom_ctrl1 = mean(abs(u_ctrl1));
u_prom_ctrl2 = mean(abs(u_ctrl2_total));
mejora_uprom = sprintf('%.1f%%', (u_prom_ctrl1-u_prom_ctrl2)/u_prom_ctrl1*100);
fprintf('%-35s %10.2f %10.2f %10s\n', '|u|_promedio (V)', u_prom_ctrl1, u_prom_ctrl2, mejora_uprom);

% Frecuencia de conmutación
mejora_freq = sprintf('%.1f%%', (freq_ctrl1-freq_ctrl2)/freq_ctrl1*100);
fprintf('%-35s %10.1f %10.1f %10s\n', 'Frecuencia conmutación (Hz)', freq_ctrl1, freq_ctrl2, mejora_freq);

% Parámetro beta
mejora_beta = sprintf('%.1f%%', (beta1-beta2)/beta1*100);
fprintf('%-35s %10.4f %10.4f %10s\n', 'Beta', beta1, beta2, mejora_beta);

fprintf('============================================================\n\n');

%% ANÁLISIS DEL EFECTO DEL PARÁMETRO 'a'
fprintf('============================================================\n');
fprintf('ANÁLISIS DEL PARÁMETRO "a" EN EL DESEMPEÑO\n');
fprintf('============================================================\n\n');

a_values_test = [3, 5, 8, 12];

figure('Position', [150 100 1200 400]);
sgtitle('EFECTO DEL PARÁMETRO "a" EN LA DINÁMICA DEL SISTEMA', 'FontSize', 13, 'FontWeight', 'bold');

colores = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250; 0.4940 0.1840 0.5560];

fprintf('%-10s %-12s %-20s %-20s\n', 'Valor a', 'Beta', 'T_establecimiento', 'Observación');
fprintf('--------------------------------------------------------------------\n');

for idx = 1:length(a_values_test)
    a_test = a_values_test(idx);
    
    % Diseñar beta para este 'a' (Controlador 1)
    numerador_test = a_test*x2_max + h_max_ctrl1;
    beta_test = (numerador_test / g_real_min) * 1.3;
    
    % Simular
    [t_test, x_test] = ode45(@(t,x) dynamics_ctrl1(t, x, x_r_comp, params_real, g, a_test, beta_test), ...
                             tspan_comp, x0_comp, options_comp);
    
    % Calcular tiempo de establecimiento
    ts_test = settling_time(t_test, x_test(:,1), x_r_comp, 0.02);
    
    % Observación
    if a_test <= 5
        obs = 'Lento, suave';
    elseif a_test <= 9
        obs = 'Balanceado';
    else
        obs = 'Rápido, agresivo';
    end
    
    fprintf('%-10.1f %-12.4f %-20.3f %-20s\n', a_test, beta_test, ts_test, obs);
    
    % Gráfica 1: Posición
    subplot(1,3,1);
    plot(t_test, rad2deg(x_test(:,1)), 'Color', colores(idx,:), 'LineWidth', 2, ...
         'DisplayName', sprintf('a=%.0f', a_test));
    hold on;
    
    % Gráfica 2: Error
    subplot(1,3,2);
    plot(t_test, rad2deg(x_test(:,1) - x_r_comp), 'Color', colores(idx,:), 'LineWidth', 2, ...
         'DisplayName', sprintf('a=%.0f', a_test));
    hold on;
    
    % Gráfica 3: Superficie
    subplot(1,3,3);
    x_e_test = x_test(:,1) - x_r_comp;
    s_test = a_test*x_e_test + x_test(:,2);
    plot(t_test, s_test, 'Color', colores(idx,:), 'LineWidth', 2, ...
         'DisplayName', sprintf('a=%.0f', a_test));
    hold on;
end

subplot(1,3,1);
plot(t_test, rad2deg(x_r_comp)*ones(size(t_test)), 'k--', 'LineWidth', 1.5);
grid on; xlabel('Tiempo (s)'); ylabel('Posición (°)');
title('Posición vs Tiempo');
legend('Location', 'southeast');
xlim([0 t_final_comp]);

subplot(1,3,2);
yline(0, 'k--', 'LineWidth', 1);
grid on; xlabel('Tiempo (s)'); ylabel('Error (°)');
title('Error vs Tiempo');
legend('Location', 'best');
xlim([0 t_final_comp]);

subplot(1,3,3);
yline(0, 'k--', 'LineWidth', 1.5);
grid on; xlabel('Tiempo (s)'); ylabel('s');
title('Superficie Deslizante vs Tiempo');
legend('Location', 'best');
xlim([0 t_final_comp]);

fprintf('--------------------------------------------------------------------\n');
fprintf('\nCONCLUSIONES:\n');
fprintf('• a PEQUEÑO (3-5):   Convergencia lenta, trayectoria suave\n');
fprintf('                     Recomendado cuando el confort es prioritario\n\n');
fprintf('• a MODERADO (5-8):  Buen balance velocidad/suavidad\n');
fprintf('                     Recomendado para la mayoría de aplicaciones\n\n');
fprintf('• a GRANDE (>10):    Convergencia muy rápida, posible sobrepaso\n');
fprintf('                     Recomendado solo si velocidad es crítica\n');
fprintf('============================================================\n\n');

%% COMENTARIOS FINALES
fprintf('################################################################\n');
fprintf('##   COMENTARIOS SOBRE EL DESEMPEÑO DE LOS CONTROLADORES     ##\n');
fprintf('################################################################\n\n');

fprintf('1. CONTROLADOR 1 (u = -beta*sign(s)) - SIN control equivalente:\n');
fprintf('   VENTAJAS:\n');
fprintf('   • Implementación simple\n');
fprintf('   • No requiere modelo del sistema\n');
fprintf('   • Robusto ante incertidumbres\n');
fprintf('   DESVENTAJAS:\n');
fprintf('   • Beta grande (mayor esfuerzo de control)\n');
fprintf('   • Alta frecuencia de conmutación (chattering)\n');
fprintf('   • Mayor desgaste del actuador\n');
fprintf('   • Menor eficiencia energética\n\n');

fprintf('2. CONTROLADOR 2 (u = u_eq - beta*sign(s)) - CON control equivalente:\n');
fprintf('   VENTAJAS:\n');
fprintf('   • Beta pequeño (%.1f%% de reducción)\n', (beta1-beta2)/beta1*100);
fprintf('   • Frecuencia de conmutación reducida (%.1f%% menos)\n', (freq_ctrl1-freq_ctrl2)/freq_ctrl1*100);
fprintf('   • Menor chattering → Mayor vida útil del actuador\n');
fprintf('   • Control más suave y eficiente\n');
fprintf('   • Mejor desempeño energético\n');
fprintf('   DESVENTAJAS:\n');
fprintf('   • Requiere modelo nominal del sistema\n');
fprintf('   • Implementación ligeramente más compleja\n\n');

fprintf('3. EFECTO DE LOS PARÁMETROS DE DISEÑO:\n');
fprintf('   Parámetro "a":\n');
fprintf('   • Determina la velocidad de convergencia en la superficie\n');
fprintf('   • Mayor "a" → Convergencia más rápida pero más agresiva\n');
fprintf('   • Menor "a" → Convergencia más lenta pero más suave\n');
fprintf('   • Selección típica: a ∈ [5, 10] para balance velocidad/suavidad\n\n');
fprintf('   Parámetro "beta":\n');
fprintf('   • Garantiza alcanzabilidad y robustez\n');
fprintf('   • Debe satisfacer: beta > |incertidumbre|/g(x)\n');
fprintf('   • Mayor "beta" → Mayor robustez pero más chattering\n');
fprintf('   • El control equivalente permite usar beta más pequeño\n\n');

fprintf('4. RECOMENDACIÓN FINAL:\n');
fprintf('   El CONTROLADOR 2 (con control equivalente) es SUPERIOR en todos\n');
fprintf('   los aspectos relevantes para aplicaciones prácticas:\n');
fprintf('   • Reduce chattering significativamente\n');
fprintf('   • Mantiene la robustez del modo deslizante\n');
fprintf('   • Protege el actuador del desgaste excesivo\n');
fprintf('   • Es la opción recomendada para implementación real\n');
fprintf('################################################################\n\n');

%% Funciones auxiliares para esta sección
function dx = dynamics_ctrl1(t, x, x_r, params, g, a, beta)
    % Controlador 1: u = -beta*sign(s)
    R = params(1); J = params(2); B = params(3);
    Km = params(4); M = params(5); L = params(6);
    
    x1 = x(1); x2 = x(2);
    x_e = x1 - x_r;
    s = a*x_e + x2;
    
    u = -beta * sign(s);
    
    dx1 = x2;
    dx2 = -(B/J)*x2 - (M*g*L*sin(x1))/J + (Km/(J*R))*u;
    dx = [dx1; dx2];
end

function dx = dynamics_ctrl2(t, x, x_r, params_real, params_nom, g, a, beta)
    % Controlador 2: u = u_eq - beta*sign(s)
    R = params_real(1); J = params_real(2); B = params_real(3);
    Km = params_real(4); M = params_real(5); L = params_real(6);
    
    R_h = params_nom(1); J_h = params_nom(2); B_h = params_nom(3);
    Km_h = params_nom(4); M_h = params_nom(5); L_h = params_nom(6);
    
    x1 = x(1); x2 = x(2);
    x_e = x1 - x_r;
    s = a*x_e + x2;
    
    % Control equivalente (usa parámetros nominales)
    h_hat = -(B_h/J_h)*x2 - (M_h*g*L_h*sin(x1))/J_h;
    g_hat = Km_h/(J_h*R_h);
    u_eq = -(a*x2 + h_hat) / g_hat;
    
    % Control conmutado
    u_sm = -beta * sign(s);
    
    u = u_eq + u_sm;
    
    dx1 = x2;
    dx2 = -(B/J)*x2 - (M*g*L*sin(x1))/J + (Km/(J*R))*u;
    dx = [dx1; dx2];
end

function ts = settling_time(t, x1, x_r, tolerance)
    error_pct = abs((x1 - x_r)/x_r * 100);
    idx = find(error_pct < tolerance*100, 1, 'first');
    if ~isempty(idx)
        ts = t(idx);
    else
        ts = inf;
    end
end

function freq = estimate_switching_freq(u, t)
    % Estimar frecuencia de conmutación contando cambios de signo
    sign_changes = sum(abs(diff(sign(u))) > 0);
    total_time = t(end) - t(1);
    freq = sign_changes / total_time;
end