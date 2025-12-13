%% ============================================================
%%    CONTROL ADAPTATIVO MRAC PARA BRAZO-MOTOR (theta0=50°)
%% ============================================================
clear; close all; clc;

%% ===== Parámetros nominales de la planta =====
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación (50°)
Ts  = 0.01;         % período de muestreo (s)

fprintf('\n========================================\n');
fprintf('  PARÁMETROS DE LA PLANTA\n');
fprintf('========================================\n');
fprintf('J   = %.4e kg·m²\n', J);
fprintf('Bv  = %.4e N·m·s/rad\n', Bv);
fprintf('MgL = %.4e N·m\n', MgL);
fprintf('Ku  = %.4e N·m/duty\n', Ku);
fprintf('th0 = %.2f° (%.4f rad)\n', rad2deg(th0), th0);
fprintf('Ts  = %.3f s\n', Ts);

%% ===== Cambio de planta a los 4 segundos =====
plantVar.t_change  = 4.0;   % instante del cambio (s)
plantVar.Ku_factor = 0.6;   % motor pierde 40% de torque
plantVar.B_factor  = 2.0;   % fricción se duplica

fprintf('\n========================================\n');
fprintf('  VARIACIÓN DE LA PLANTA\n');
fprintf('========================================\n');
fprintf('Tiempo de cambio: %.1f s\n', plantVar.t_change);
fprintf('Después del cambio:\n');
fprintf('  Ku_nuevo = %.2f × Ku_nominal\n', plantVar.Ku_factor);
fprintf('  Bv_nuevo = %.2f × Bv_nominal\n', plantVar.B_factor);

%% ===== Parámetros MRAC =====
mrac.wn_ref   = 2.5;        % frecuencia natural del modelo de referencia [rad/s]
mrac.zeta_ref = 0.85;       % amortiguamiento del modelo de referencia

% Ganancias de adaptación
mrac.gamma_theta = 100;     % ganancia para vector de parámetros θ
mrac.gamma_k     = 60;      % ganancia para ganancia feedforward k

% Inicialización de parámetros adaptativos
mrac.theta_hat = [0.3; 0.3; 0.3];  % vector inicial [θ1, θ2, θ3]
mrac.k_hat     = 0.8;               % ganancia feedforward inicial

% Límites de saturación
mrac.theta_max = 50;
mrac.theta_min = -50;
mrac.k_max     = 10;
mrac.k_min     = 0.01;

fprintf('\n========================================\n');
fprintf('  PARÁMETROS DEL CONTROLADOR MRAC\n');
fprintf('========================================\n');
fprintf('Modelo de referencia:\n');
fprintf('  ωn  = %.2f rad/s\n', mrac.wn_ref);
fprintf('  ζ   = %.2f\n', mrac.zeta_ref);
fprintf('\nGanancias de adaptación:\n');
fprintf('  γ_θ = %.1f\n', mrac.gamma_theta);
fprintf('  γ_k = %.1f\n', mrac.gamma_k);
fprintf('\nParámetros adaptativos iniciales:\n');
fprintf('  θ̂₁(0) = %.2f\n', mrac.theta_hat(1));
fprintf('  θ̂₂(0) = %.2f\n', mrac.theta_hat(2));
fprintf('  θ̂₃(0) = %.2f\n', mrac.theta_hat(3));
fprintf('  k̂(0)  = %.2f\n', mrac.k_hat);
fprintf('\nLímites de saturación:\n');
fprintf('  θ ∈ [%.1f, %.1f]\n', mrac.theta_min, mrac.theta_max);
fprintf('  k ∈ [%.2f, %.2f]\n', mrac.k_min, mrac.k_max);

%% ===== Señales de referencia =====
Tend = 12;              % duración de simulación (s)
t    = 0:Ts:Tend;       % vector de tiempo

% Tres perfiles de referencia diferentes
ref_step = deg2rad(50)*ones(size(t));    % escalón de 50°
ref_ramp = deg2rad(10)*t;                % rampa de 10°/s
Aimp     = deg2rad(8); 
Timp     = 1;
ref_puls = (mod(t,Timp)<Timp/2)*Aimp - (mod(t,Timp)>=Timp/2)*Aimp;  % tren de pulsos ±8°

profiles = {'ESCALÓN',  ref_step; ...
            'RAMPA',    ref_ramp; ...
            'PULSOS',   ref_puls};

fprintf('\n========================================\n');
fprintf('  PERFILES DE REFERENCIA\n');
fprintf('========================================\n');
fprintf('1. ESCALÓN: %.1f°\n', rad2deg(ref_step(1)));
fprintf('2. RAMPA:   %.1f°/s\n', 10);
fprintf('3. PULSOS:  ±%.1f° (período %.1f s)\n', rad2deg(Aimp), Timp);
fprintf('\n');

%% =====================================================================
%%          FIGURA 1: RESPUESTA DEL CONTROLADOR MRAC
%% =====================================================================
fprintf('Simulando controlador MRAC...\n');

figure('Name','Controlador MRAC - Respuestas','NumberTitle','off', ...
       'Position', [50 50 1600 900], 'Color', 'w');

for i = 1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0;  % referencia absoluta
    
    fprintf('  Simulando perfil: %s\n', name);
    [th, u, th_m] = simulate_mrac(J,Bv,MgL,Ku,Ts,r,mrac,plantVar,th0);
    
    e = rad2deg(r - th);         % error en grados
    
    % 1) Gráfica de POSICIÓN
    subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2.5, ...
         'DisplayName', 'Salida \theta(t)'); hold on;
    plot(t, rad2deg(th_m), 'g--', 'LineWidth', 2, ...
         'DisplayName', 'Modelo ref \theta_m(t)');
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2, ...
         'DisplayName', 'Referencia r(t)');
    xline(plantVar.t_change, 'k:', 'LineWidth', 1.5, ...
          'DisplayName', 'Cambio planta');
    grid on; grid minor;
    title([name ' - POSICIÓN'], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Ángulo [°]', 'FontSize', 11);
    xlabel('Tiempo [s]', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);
    set(gca, 'FontSize', 10);
    
    % 2) Gráfica de SEÑAL DE CONTROL
    subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 2); hold on;
    yline(1, 'r--', 'LineWidth', 1.5);
    yline(-1, 'r--', 'LineWidth', 1.5);
    yline(0, 'k:', 'LineWidth', 1);
    xline(plantVar.t_change, 'k:', 'LineWidth', 1.5);
    grid on; grid minor;
    title([name ' - CONTROL'], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('u(t) [duty]', 'FontSize', 11);
    xlabel('Tiempo [s]', 'FontSize', 11);
    ylim([-1.3 1.3]);
    set(gca, 'FontSize', 10);
    
    % 3) Gráfica de ERROR
    subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.85 0.33 0.10], 'LineWidth', 2.5); hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    xline(plantVar.t_change, 'k:', 'LineWidth', 1.5);
    grid on; grid minor;
    title([name ' - ERROR'], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Error [°]', 'FontSize', 11);
    xlabel('Tiempo [s]', 'FontSize', 11);
    set(gca, 'FontSize', 10);
end

sgtitle('CONTROLADOR MRAC - RESPUESTA TEMPORAL', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% =====================================================================
%%          FIGURA 2: EVOLUCIÓN DE PARÁMETROS ADAPTATIVOS
%% =====================================================================
fprintf('\nGenerando gráficas de evolución de parámetros...\n');

theta1_all = zeros(length(t), 3);
theta2_all = zeros(length(t), 3);
theta3_all = zeros(length(t), 3);
k_all      = zeros(length(t), 3);

for i = 1:size(profiles,1)
    r = profiles{i,2} + th0;
    [~, ~, ~, theta_hist, k_hist] = simulate_mrac(J,Bv,MgL,Ku,Ts,r,mrac,plantVar,th0);
    theta1_all(:,i) = theta_hist(:,1);
    theta2_all(:,i) = theta_hist(:,2);
    theta3_all(:,i) = theta_hist(:,3);
    k_all(:,i)      = k_hist;
end

figure('Name','MRAC - Evolución de Parámetros','NumberTitle','off', ...
       'Position', [100 100 1600 900], 'Color', 'w');

colores = [0 0.45 0.74; 0.85 0.33 0.10; 0.93 0.69 0.13];
nombres = {'ESCALÓN', 'RAMPA', 'PULSOS'};

% Parámetro θ₁
subplot(2,2,1);
for i = 1:3
    plot(t, theta1_all(:,i), 'LineWidth', 2.5, 'Color', colores(i,:), ...
         'DisplayName', nombres{i}); hold on;
end
xline(plantVar.t_change, 'k--', 'Cambio', 'LineWidth', 2);
grid on; grid minor;
title('\theta_1(t) - Adaptación', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('\theta_1', 'FontSize', 11);
xlabel('Tiempo [s]', 'FontSize', 11);
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Parámetro θ₂
subplot(2,2,2);
for i = 1:3
    plot(t, theta2_all(:,i), 'LineWidth', 2.5, 'Color', colores(i,:), ...
         'DisplayName', nombres{i}); hold on;
end
xline(plantVar.t_change, 'k--', 'Cambio', 'LineWidth', 2);
grid on; grid minor;
title('\theta_2(t) - Adaptación', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('\theta_2', 'FontSize', 11);
xlabel('Tiempo [s]', 'FontSize', 11);
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Parámetro θ₃
subplot(2,2,3);
for i = 1:3
    plot(t, theta3_all(:,i), 'LineWidth', 2.5, 'Color', colores(i,:), ...
         'DisplayName', nombres{i}); hold on;
end
xline(plantVar.t_change, 'k--', 'Cambio', 'LineWidth', 2);
grid on; grid minor;
title('\theta_3(t) - Adaptación', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('\theta_3', 'FontSize', 11);
xlabel('Tiempo [s]', 'FontSize', 11);
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Ganancia k
subplot(2,2,4);
for i = 1:3
    plot(t, k_all(:,i), 'LineWidth', 2.5, 'Color', colores(i,:), ...
         'DisplayName', nombres{i}); hold on;
end
xline(plantVar.t_change, 'k--', 'Cambio', 'LineWidth', 2);
grid on; grid minor;
title('k(t) - Ganancia Feedforward', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('k', 'FontSize', 11);
xlabel('Tiempo [s]', 'FontSize', 11);
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 10);

sgtitle('EVOLUCIÓN DE PARÁMETROS ADAPTATIVOS DEL MRAC', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% ===== Imprimir valores finales de parámetros =====
fprintf('\n========================================\n');
fprintf('  PARÁMETROS FINALES DEL MRAC\n');
fprintf('========================================\n');
for i = 1:3
    fprintf('\n%s:\n', nombres{i});
    fprintf('  θ̂₁(final) = %8.4f\n', theta1_all(end,i));
    fprintf('  θ̂₂(final) = %8.4f\n', theta2_all(end,i));
    fprintf('  θ̂₃(final) = %8.4f\n', theta3_all(end,i));
    fprintf('  k̂(final)  = %8.4f\n', k_all(end,i));
end

fprintf('\n========================================\n');
fprintf('  SIMULACIÓN COMPLETADA ✓\n');
fprintf('========================================\n\n');

%% =====================================================================
%%               FUNCIÓN DE SIMULACIÓN DEL MRAC
%% =====================================================================
function [theta, u_hist, theta_m_hist, theta_hist, k_hist] = ...
         simulate_mrac(J, Bv, MgL, Ku, Ts, ref, mrac, plantVar, th0)

N = numel(ref);

% Inicialización de variables de la planta
theta      = zeros(N,1); 
omega      = 0; 
theta(1)   = th0;
u_hist     = zeros(N,1);

% Inicialización de parámetros adaptativos
theta_hat  = mrac.theta_hat;  
k_hat      = mrac.k_hat;      
theta_hist = zeros(N,3);
k_hist     = zeros(N,1);

% Inicialización del modelo de referencia
xm1 = 0;  % posición del modelo
xm2 = 0;  % velocidad del modelo
theta_m_hist = zeros(N,1);

% Parámetros del modelo de referencia
wn   = mrac.wn_ref;
zeta = mrac.zeta_ref;

for k = 1:N-1
    % Tiempo actual
    t_k = (k-1)*Ts;
    
    % Determinar parámetros reales de la planta
    Ku_true = Ku;
    B_true  = Bv;
    if t_k >= plantVar.t_change
        Ku_true = plantVar.Ku_factor * Ku;
        B_true  = plantVar.B_factor  * Bv;
    end
    
    % ===== MODELO DE REFERENCIA =====
    % Dinámica: ẍm + 2ζωn·ẋm + ωn²·xm = ωn²·r
    xm1_dot = xm2;
    xm2_dot = -wn^2*xm1 - 2*zeta*wn*xm2 + wn^2*(ref(k)-th0);
    
    % Integración Euler
    xm1 = xm1 + Ts*xm1_dot;
    xm2 = xm2 + Ts*xm2_dot;
    
    theta_m_hist(k) = xm1;
    
    % ===== CÁLCULO DEL ERROR =====
    e1 = (theta(k)-th0) - xm1;  % error de posición
    e2 = omega - xm2;            % error de velocidad
    
    % ===== VECTOR DE REGRESIÓN =====
    % ω(t) = [θ-θ0, θ̇, r-θ0]'
    omega_vec = [(theta(k)-th0); omega; (ref(k)-th0)];
    
    % ===== LEY DE CONTROL MRAC =====
    % u = θ̂'·ω + k̂·r + u_ff
    u_mrac = theta_hat' * omega_vec + k_hat * (ref(k)-th0);
    
    % Compensación feedforward gravitacional
    u_ff = (MgL/Ku) * sin(ref(k));
    
    % Control total con saturación
    u = u_ff + u_mrac;
    u = max(-1, min(1, u));
    
    % ===== LEYES DE ADAPTACIÓN =====
    % Error compuesto para adaptación
    e_adapt = e1 + 0.3*e2;
    
    % Gradiente descendente
    theta_hat_dot = -mrac.gamma_theta * omega_vec * e_adapt;
    k_hat_dot     = -mrac.gamma_k * (ref(k)-th0) * e_adapt;
    
    % Integración con saturación
    theta_hat = theta_hat + Ts * theta_hat_dot;
    k_hat     = k_hat + Ts * k_hat_dot;
    
    % Proyección a límites válidos
    theta_hat = max(mrac.theta_min, min(mrac.theta_max, theta_hat));
    k_hat     = max(mrac.k_min, min(mrac.k_max, k_hat));
    
    % Guardar historiales
    theta_hist(k,:) = theta_hat';
    k_hist(k)       = k_hat;
    
    % ===== DINÁMICA NO LINEAL DE LA PLANTA =====
    % J·θ̈ + B·θ̇ + MgL·sin(θ) = Ku·u
    domega = (Ku_true*u - B_true*omega - MgL*sin(theta(k))) / J;
    omega  = omega + Ts*domega;
    theta(k+1) = theta(k) + Ts*omega;
    
    u_hist(k) = u;
end

% Completar últimos valores
u_hist(end)        = u_hist(end-1);
theta_hist(end,:)  = theta_hist(end-1,:);
k_hist(end)        = k_hist(end-1);
theta_m_hist(end)  = xm1;

end