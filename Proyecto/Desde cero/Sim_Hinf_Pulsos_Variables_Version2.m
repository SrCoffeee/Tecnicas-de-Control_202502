%% Simulación H-infinity con Tren de Pulsos Variable (Sin Simulink)
% Análisis de seguimiento de referencia sin perturbaciones

clear; close all; clc;
s = tf('s');

% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

a0  = (MgL*cos(th0))/J;
a1  = Bv/J;
b0  = Ku/J;

% Controlador H-infinity (lead-lag)
k  = 1.5;
wz = 2.0;
wp = 8.0;
Kh_s = k * (1 + s/wz) / (1 + s/wp);
Ts   = 0.01;
Khd  = c2d(Kh_s, Ts, 'tustin');

% Intentar usar hinfsyn si está disponible
try
    G = tf(b0, [1 a1 a0]);
    W1 = makeweight(1/2, 2.0, 0.1);
    W2 = tf(0.2);
    P  = augw(G, W1, W2, []);
    [Khinf_s, ~, gam] = hinfsyn(P, 1, 1);
    Khinf_d = c2d(Khinf_s, Ts, 'tustin');
    fprintf('Usando hinfsyn con gamma = %.4f\n', gam);
    ctrl = Khinf_d;
catch
    fprintf('Robust Control Toolbox no disponible:  usando lead-lag.\n');
    ctrl = Khd;
end

[bh, ah] = tfdata(ctrl, 'v');
if ah(1) ~= 1
    bh = bh / ah(1);
    ah = ah / ah(1);
end

% Parámetros de simulación
Tend = 25;
t    = 0:Ts: Tend;
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

%% Simulación con H-infinity
theta    = zeros(N, 1);
omega    = zeros(N, 1);
u_hist   = zeros(N, 1);
e_hist   = zeros(N, 1);

theta(1) = th0;
omega(1) = 0;

% Estados del filtro IIR (Direct Form II Transposed)
n_states = max(numel(bh)-1, numel(ah)-1);
z_states = zeros(n_states, 1);

for k = 1:N-1
    % Error
    e = ref(k) - theta(k);
    e_hist(k) = e;
    
    % Feedforward
    u_ff = (MgL/Ku) * sin(ref(k));
    
    % Controlador H-infinity (Direct Form II Transposed)
    x = e;
    
    if ~isempty(bh)
        y = bh(1) * x;
        if n_states > 0
            y = y + z_states(1);
        end
    else
        y = 0;
    end
    
    % Actualizar estados
    for i = 1:n_states
        if i < n_states
            z_next = z_states(i+1);
        else
            z_next = 0;
        end
        if i+1 <= numel(bh)
            z_next = z_next + bh(i+1)*x;
        end
        if i+1 <= numel(ah)
            z_next = z_next - ah(i+1)*y;
        end
        z_states(i) = z_next;
    end
    
    u = max(-1, min(1, u_ff + y));
    u_hist(k) = u;
    
    % Dinámica no lineal
    domega = (Ku*u - Bv*omega(k) - MgL*sin(theta(k))) / J;
    omega(k+1) = omega(k) + Ts * domega;
    theta(k+1) = theta(k) + Ts * omega(k+1);
end
e_hist(N) = ref(N) - theta(N);
u_hist(N) = u_hist(N-1);

%% Métricas
e_deg = rad2deg(e_hist);
IAE   = trapz(t, abs(e_deg));
ISE   = trapz(t, e_deg.^2);
ITAE  = trapz(t, t' .* abs(e_deg));
Emax  = max(abs(e_deg));

%% Gráficas
figure('Name', 'Controlador H-infinity - Tren de Pulsos Variable', ...
       'NumberTitle', 'off', 'Position', [150, 80, 1200, 800]);

subplot(3,1,1);
plot(t, rad2deg(theta - th0), 'b-', 'LineWidth', 2, 'DisplayName', 'Salida θ(t)');
hold on;
plot(t, rad2deg(ref - th0), 'r--', 'LineWidth', 2, 'DisplayName', 'Referencia r(t)');
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Ángulo [grados]', 'FontSize', 11, 'FontWeight', 'bold');
title('POSICIÓN - Controlador H-infinity', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
ylim([min(rad2deg(ref-th0))-5, max(rad2deg(ref-th0))+5]);

subplot(3,1,2);
stairs(t, u_hist, 'k-', 'LineWidth', 1.5);
hold on;
yline(1, 'r--', 'LineWidth', 1.5);
yline(-1, 'r--', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('u(t) [duty cycle]', 'FontSize', 11, 'FontWeight', 'bold');
title('SEÑAL DE CONTROL - Controlador H-infinity', 'FontSize', 14, 'FontWeight', 'bold');
ylim([-1.3, 1.3]);

subplot(3,1,3);
plot(t, e_deg, 'Color', [0.2, 0.6, 0.2], 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1.5);
grid on; grid minor;
xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Error [grados]', 'FontSize', 11, 'FontWeight', 'bold');
title('ERROR DE SEGUIMIENTO - Controlador H-infinity', 'FontSize', 14, 'FontWeight', 'bold');

metStr = sprintf('IAE = %.2f°·s | ISE = %.2f°²·s | ITAE = %. 2f°·s² | E_{max} = %.2f°', ... 
                 IAE, ISE, ITAE, Emax);
text(0.5, 0.95, metStr, 'Units', 'normalized', 'FontSize', 10, ...
     'FontWeight', 'bold', 'HorizontalAlignment', 'center', ... 
     'BackgroundColor', 'w', 'EdgeColor', 'k');

sgtitle('ANÁLISIS H-INFINITY - TREN DE PULSOS VARIABLE (Sin Perturbaciones)', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% Resultados en consola
fprintf('\n========================================\n');
fprintf('   CONTROLADOR H-INFINITY - RESULTADOS\n');
fprintf('========================================\n');
fprintf('Parámetros del controlador:\n');
fprintf('  k = %.2f, wz = %.2f, wp = %.2f\n', k, wz, wp);
fprintf('\nMétricas de desempeño:\n');
fprintf('  IAE  = %.2f °·s\n', IAE);
fprintf('  ISE  = %.2f °²·s\n', ISE);
fprintf('  ITAE = %.2f °·s²\n', ITAE);
fprintf('  Error máximo = %. 2f °\n', Emax);
fprintf('========================================\n');