%% Diseño y simulación PID / Hinf / SMC para el brazo-motor (theta0=50°)
clear; close all; clc;
s = tf('s');  % Variable de Laplace

% Parámetros planta (del proyecto)
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

a0  = (MgL*cos(th0))/J;
a1  = Bv/J;
b0  = Ku/J;

G   = tf(b0, [1 a1 a0]);          % modelo lineal
Ts  = 0.01;                       % muestreo (Arduino)
disp('G(s) = b0/(s^2+a1 s + a0) with:'), [a0 a1 b0]

%% --- PID por asignación de polos (continuo) ---
zeta = 0.9; wn = 2.2; p3 = 4*wn;
a2d = 2*zeta*wn + p3;
a1d = wn^2 + 2*zeta*wn*p3;
a0d = (wn^2)*p3;

Kd    = (a2d - a1)/b0;
Kp    = (a1d - a0)/b0;
Ki    = a0d/b0;
tau_d = 0.05; 
Tt    = 0.35;

Cpid_s = Kp + Ki/s + (Kd*s)/(1+tau_d*s);
Cid    = c2d(Cpid_s, Ts, 'tustin');
disp('PID gains:'), [Kp Ki Kd tau_d Tt]

%% --- Hinf loop-shaping simple (lead-lag continuo) ---
k  = 1.5; 
wz = 2.0; 
wp = 8.0;
Kh_s = k * (1 + s/wz) / (1 + s/wp);
Khd  = c2d(Kh_s, Ts, 'tustin');

% (Opcional) Hinf con Robust Control Toolbox:
try
    W1 = makeweight(1/2, 2.0, 0.1);  % baja sensibilidad en baja freq, cruce ~2rad/s
    W2 = tf(0.2);                    % peso de esfuerzo
    P  = augw(G, W1, W2, []);        % planta aumentada
    [Khinf_s,~,gam] = hinfsyn(P,1,1);
    Khinf_d = c2d(Khinf_s, Ts, 'tustin');
    disp(['Hinfsyn gamma = ' num2str(gam)]);
catch
    Khinf_d = Khd;  % fallback
    disp('hinfsyn no disponible: uso lead-lag discretizado.');
end

%% --- SMC (simulación no lineal discreta) ---
lambda    = 3.0;   % rad/s
k_torque  = 0.02;   % N·m
phi       = 0.5;   % rad/s

%% Perfiles de referencia
Tend = 12; 
t    = 0:Ts:Tend;
ref_step = deg2rad(50)*ones(size(t));
ref_ramp = deg2rad(10)*t;           % rampa 10deg/s
Aimp     = deg2rad(8); 
Timp     = 1;                       % tren impulsos ±8deg
ref_puls = (mod(t,Timp)<Timp/2)*Aimp - (mod(t,Timp)>=Timp/2)*Aimp;

profiles = {'STEP',   ref_step; ...
            'RAMP',   ref_ramp; ...
            'PULSET', ref_puls};

%% Simulación helper (modo: 'PID','HINF','SMC')
sat   = @(u) max(-1,min(1,u));
simNL = @(ref,mode) simulate_nl(J,Bv,MgL,Ku,Ts,ref,mode, ...
                                Kp,Ki,Kd,tau_d,Tt,Khd,Khinf_d, ...
                                lambda,k_torque,phi,th0);

%% === Figura 1: Controlador PID ===
figure('Name','Controlador PID','NumberTitle','off', ...
       'Position', [100 100 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0; % ref absoluta alrededor de th0
    [th,u] = simNL(r, 'PID');
    
    % Calcular error en grados
    e = rad2deg(r - th);
    
    % 1) Posición (Salida del sistema)
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2, ...
         'DisplayName', 'Salida \theta(t)'); 
    hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2, ...
         'DisplayName', 'Ref r(t)');
    grid on; grid minor;
    title(['POSICION PID - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Ángulo [grados]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    legend(axPos,'Location', 'best', 'FontSize', 10);
    set(axPos, 'FontSize', 10);
    
    % 2) Señal de control (Entrada al sistema)
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Control u(t)'); 
    hold on;
    yline(1,   'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(-1,  'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(0,   'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(0.02*Tend,  1.05,  'Sat +1', 'Color', 'r', 'FontSize', 9);
    text(0.02*Tend, -1.15, 'Sat -1', 'Color', 'r', 'FontSize', 9);
    grid on; grid minor;
    title(['ENTRADA PID - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('u(t) [duty cycle]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    ylim([-1.2 1.2]);
    legend(axU,'Location', 'best', 'FontSize', 10);
    set(axU, 'FontSize', 10);
    
    % 3) Error de seguimiento
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.8 0.2 0.2], 'LineWidth', 2);
    hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    grid on; grid minor;
    title(['ERROR PID - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Error [grados]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    set(axErr, 'FontSize', 10);
    
    % Métricas
    iae      = trapz(t, abs(e));  % Integral del error absoluto
    ise      = trapz(t, e.^2);    % Integral del error cuadrático
    max_err  = max(abs(e));
    metStr   = sprintf('IAE=%.1f  ISE=%.1f\nE_{max}=%.1f', iae, ise, max_err);
    
    % Texto de métricas en el subplot de error (sin annotation)
    text(axErr, 0.02, 0.95, metStr, ...
         'Units','normalized', ...
         'BackgroundColor','w', 'EdgeColor','k', ...
         'FontSize',8, 'FontWeight','bold', ...
         'VerticalAlignment','top', 'HorizontalAlignment','left');
end
sgtitle('CONTROLADOR PID', 'FontSize', 16, 'FontWeight', 'bold');

%% === Figura 2: Controlador H-infinity ===
figure('Name','Controlador H-infinity','NumberTitle','off', ...
       'Position', [150 50 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0;
    [th,u] = simNL(r, 'HINF');
    
    % Error en grados
    e = rad2deg(r - th);
    
    % 1) Posición
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2, ...
         'DisplayName', 'Salida \theta(t)'); 
    hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2, ...
         'DisplayName', 'Ref r(t)');
    grid on; grid minor;
    title(['POSICION H-INF - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Ángulo [grados]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    legend(axPos,'Location', 'best', 'FontSize', 10);
    set(axPos, 'FontSize', 10);
    
    % 2) Señal de control
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Control u(t)'); 
    hold on;
    yline(1,   'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(-1,  'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(0,   'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(0.02*Tend,  1.05,  'Sat +1', 'Color', 'r', 'FontSize', 9);
    text(0.02*Tend, -1.15, 'Sat -1', 'Color', 'r', 'FontSize', 9);
    grid on; grid minor;
    title(['ENTRADA H-INF - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('u(t) [duty cycle]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    ylim([-1.2 1.2]);
    legend(axU,'Location', 'best', 'FontSize', 10);
    set(axU, 'FontSize', 10);
    
    % 3) Error
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.2 0.6 0.2], 'LineWidth', 2);
    hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    grid on; grid minor;
    title(['ERROR H-INF - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Error [grados]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    set(axErr, 'FontSize', 10);
    
    % Métricas
    iae     = trapz(t, abs(e));
    ise     = trapz(t, e.^2);
    max_err = max(abs(e));
    metStr  = sprintf('IAE=%.1f  ISE=%.1f\nE_{max}=%.1f', iae, ise, max_err);
    
    text(axErr, 0.02, 0.95, metStr, ...
         'Units','normalized', ...
         'BackgroundColor','w', 'EdgeColor','k', ...
         'FontSize',8, 'FontWeight','bold', ...
         'VerticalAlignment','top', 'HorizontalAlignment','left');
end
sgtitle('CONTROLADOR H-INFINITY', 'FontSize', 16, 'FontWeight', 'bold');

%% === Figura 3: Controlador SMC ===
figure('Name','Controlador SMC (Sliding Mode)','NumberTitle','off', ...
       'Position', [200 0 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0;
    [th,u] = simNL(r, 'SMC');
    
    % Error en grados
    e = rad2deg(r - th);
    
    % 1) Posición
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2, ...
         'DisplayName', 'Salida \theta(t)'); 
    hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2, ...
         'DisplayName', 'Ref r(t)');
    grid on; grid minor;
    title(['POSICION SMC - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Ángulo [grados]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    legend(axPos,'Location', 'best', 'FontSize', 10);
    set(axPos, 'FontSize', 10);
    
    % 2) Señal de control
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Control u(t)'); 
    hold on;
    yline(1,   'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(-1,  'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(0,   'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(0.02*Tend,  1.05,  'Sat +1', 'Color', 'r', 'FontSize', 9);
    text(0.02*Tend, -1.15, 'Sat -1', 'Color', 'r', 'FontSize', 9);
    grid on; grid minor;
    title(['ENTRADA SMC - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('u(t) [duty cycle]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    ylim([-1.2 1.2]);
    legend(axU,'Location', 'best', 'FontSize', 10);
    set(axU, 'FontSize', 10);
    
    % 3) Error
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.6 0.2 0.8], 'LineWidth', 2);
    hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    grid on; grid minor;
    title(['ERROR SMC - ' name], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Error [grados]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo [s]', 'FontSize', 11, 'FontWeight', 'bold');
    set(axErr, 'FontSize', 10);
    
    % Métricas
    iae     = trapz(t, abs(e));
    ise     = trapz(t, e.^2);
    max_err = max(abs(e));
    metStr  = sprintf('IAE=%.1f  ISE=%.1f\nE_{max}=%.1f', iae, ise, max_err);
    
    text(axErr, 0.02, 0.95, metStr, ...
         'Units','normalized', ...
         'BackgroundColor','w', 'EdgeColor','k', ...
         'FontSize',8, 'FontWeight','bold', ...
         'VerticalAlignment','top', 'HorizontalAlignment','left');
end
sgtitle('CONTROLADOR SMC (SLIDING MODE)', 'FontSize', 16, 'FontWeight', 'bold');

%% Exporta coeficientes discretos a Arduino 
% (Hinf en forma u[k]=a1 u[k-1] + b0 e[k] + b1 e[k-1])
[bh,ah] = tfdata(Khd,'v'); 
disp('Coeficientes Khd:')
disp(['bh = ' num2str(bh)])
disp(['ah = ' num2str(ah)])

% Normalizar si es necesario
if ~isempty(ah) && ah(1) ~= 1
    bh = bh / ah(1);
    ah = ah / ah(1);
end

% Para Arduino (forma directa II transpuesta)
if numel(bh) >= 2 && numel(ah) >= 2
    kh_a1 = -ah(2); 
    kh_b0 = bh(1); 
    kh_b1 = bh(2);
else
    kh_a1 = 0;
    kh_b0 = bh(1);
    kh_b1 = 0;
end
disp('Para Arduino: kh_a1, kh_b0, kh_b1 ='); 
[kh_a1, kh_b0, kh_b1]

%% ====== Función de simulación no lineal discreta ======
function [theta,u_hist]=simulate_nl(J,Bv,MgL,Ku,Ts,ref,mode, ...
                                    Kp,Ki,Kd,tau_d,Tt,Khd,Khinf_d, ...
                                    lambda,k_torque,phi,th0)
N        = numel(ref);
theta    = zeros(N,1); 
omega    = 0; 
u_hist   = zeros(N,1);
theta(1) = th0; 
Iint     = 0; 
Df       = 0; 
eprev    = 0;

% Preparar controlador HINF (usa Khinf_d para modo HINF, Khd para verificación)
if strcmp(mode,'HINF')
    ctrl = Khinf_d;
else
    ctrl = Khd;
end
[bh,ah] = tfdata(ctrl,'v');

% Normalizar coeficientes
if ~isempty(ah) && ah(1) ~= 0
    bh = bh / ah(1);
    ah = ah / ah(1);
end

% Estados del filtro IIR (Direct Form II Transposed)
n_states = max(numel(bh)-1, numel(ah)-1);
z_states = zeros(n_states, 1);

for k=1:N-1
  e    = (ref(k)-theta(k));
  u_ff = (MgL/Ku)*sin(ref(k));
  
  if strcmp(mode,'PID')
     % PID con filtro derivativo
     a   = tau_d/(tau_d+Ts); 
     b   = Kd/(tau_d+Ts);
     Df  = a*Df + b*(e-eprev);
     u_uns = u_ff + (Kp*e + Df + Iint);
     u     = max(-1,min(1,u_uns));
     Iint  = Iint + Ki*Ts*e + (Ts/Tt)*(u - u_uns);
     
  elseif strcmp(mode,'HINF')
     % Implementación Direct Form II Transposed
     x = e;  % entrada
     
     % salida
     if ~isempty(bh)
         y = bh(1) * x;
         if n_states > 0
             y = y + z_states(1);
         end
     else
         y = 0;
     end
     
     % actualizar estados
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
     
     u = max(-1,min(1,u_ff + y));
     
  else % SMC
     % Error de posición con signo clásico (theta - ref)
     e_theta = theta(k) - ref(k);
     
     % Superficie deslizante
     s = omega + lambda * e_theta;
     
     % Control equivalente (regulación alrededor de la ref)
     u_eq = (J*(-lambda*omega) + Bv*omega + MgL*sin(theta(k)))/Ku;
     
     % Término de conmutación
     u_sw = -(k_torque/Ku) * tanh(s/phi);
     
     % Ley total
     u = u_eq + u_sw;
     u = max(-1,min(1,u));
end

  
  % Dinámica no lineal (Euler explícito)
  domega = (Ku*u - Bv*omega - MgL*sin(theta(k)))/J;
  omega  = omega + Ts*domega;
  theta(k+1) = theta(k) + Ts*omega;
  u_hist(k)  = u;
  eprev      = e;
end
u_hist(end)=u_hist(end-1);
end
