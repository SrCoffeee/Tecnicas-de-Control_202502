%% Diseño y simulación PID / Hinf / SMC / APID para el brazo-motor (theta0=50°)
clear; close all; clc;
s = tf('s');  % Variable de Laplace

%% ===== Parámetros de la planta nominal (del proyecto) =====
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación (50°)

% Modelo lineal alrededor de th0
a0  = (MgL*cos(th0))/J;
a1  = Bv/J;
b0  = Ku/J;
G   = tf(b0, [1 a1 a0]);          % modelo lineal
Ts  = 0.01;                       % muestreo (Arduino / simulación)
disp('G(s) = b0/(s^2+a1 s + a0) con: [a0 a1 b0] =');
disp([a0 a1 b0])

%% ===== Cambio de planta para demostrar el APID =====
% A los 4 s:
%   - el motor pierde 40 % de torque  => Ku_true = 0.6*Ku
%   - la fricción viscosa se duplica => B_true  = 2*Bv
plantVar.t_change  = 4.0;   % instante del cambio (s)
plantVar.Ku_factor = 0.6;   % factor para Ku (después del cambio)
plantVar.B_factor  = 2.0;   % factor para Bv (después del cambio)

%% ===== PID por asignación de polos (continuo) =====
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
disp('Ganancias PID [Kp Ki Kd tau_d Tt] =');
disp([Kp Ki Kd tau_d Tt])

%% ===== Hinf "loop shaping" simple (lead-lag continuo) =====
k  = 1.5; 
wz = 2.0; 
wp = 8.0;
Kh_s = k * (1 + s/wz) / (1 + s/wp);
Khd  = c2d(Kh_s, Ts, 'tustin');

% (Opcional) Hinf con Robust Control Toolbox:
try
    W1 = makeweight(1/2, 2.0, 0.1);  % peso sensibilidad
    W2 = tf(0.2);                    % peso esfuerzo
    P  = augw(G, W1, W2, []);        % planta aumentada
    [Khinf_s,~,gam] = hinfsyn(P,1,1);
    Khinf_d = c2d(Khinf_s, Ts, 'tustin');
    disp(['hinfsyn gamma = ' num2str(gam)]);
catch
    Khinf_d = Khd;                    % fallback
    disp('hinfsyn no disponible: uso lead-lag discretizado.');
end

%% ===== SMC (Sliding Mode) =====
lambda    = 3.0;     % rad/s, define superficie
k_torque  = 0.01;    % N·m, intensidad del switching
phi       = 0.5;     % capa límite (reduce chattering)

%% ===== Parámetros RLS para control adaptativo (APID) =====
rls.Ku_hat = 0.6*Ku;         % estimación inicial equivocada a propósito
rls.B_hat  = 1.5*Bv;
rls.P      = 10*eye(2);      % poca confianza inicial
rls.lam    = 0.995;          % factor de olvido

%% ===== Perfiles de referencia =====
Tend = 12; 
t    = 0:Ts:Tend;

ref_step = deg2rad(50)*ones(size(t));    % escalón 50°
ref_ramp = deg2rad(10)*t;                % rampa 10deg/s
Aimp     = deg2rad(8); 
Timp     = 1;                            % tren impulsos ±8deg
ref_puls = (mod(t,Timp)<Timp/2)*Aimp - (mod(t,Timp)>=Timp/2)*Aimp;

profiles = {'STEP',   ref_step; ...
            'RAMP',   ref_ramp; ...
            'PULSET', ref_puls};

%% ===== Helper de simulación no lineal =====
simNL = @(ref,mode) simulate_nl(J,Bv,MgL,Ku,Ts,ref,mode, ...
                                Kp,Ki,Kd,tau_d,Tt,Khd,Khinf_d, ...
                                lambda,k_torque,phi,th0,rls,plantVar);

%% =====================================================================
%                        FIGURA 1: CONTROLADOR PID
% ======================================================================
figure('Name','Controlador PID','NumberTitle','off', ...
       'Position', [50 50 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0;           % ref absoluta
    [th,u] = simNL(r, 'PID');
    e = rad2deg(r - th);
    
    % 1) Posición
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2, ...
         'DisplayName', 'Salida \theta(t)'); hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2, ...
         'DisplayName', 'Ref r(t)');
    grid on; grid minor;
    title(['POSICION PID - ' name]);
    ylabel('Ángulo [grados]');
    xlabel('Tiempo [s]');
    legend(axPos,'Location', 'best');
    
    % 2) Control
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8); hold on;
    yline(1,'r--'); yline(-1,'r--'); yline(0,'k:');
    grid on; grid minor;
    title(['ENTRADA PID - ' name]);
    ylabel('u(t) [duty]');
    xlabel('Tiempo [s]');
    ylim([-1.2 1.2]);
    
    % 3) Error
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.8 0.2 0.2], 'LineWidth', 2); hold on;
    yline(0, 'k--');
    grid on; grid minor;
    title(['ERROR PID - ' name]);
    ylabel('Error [grados]');
    xlabel('Tiempo [s]');
end
sgtitle('CONTROLADOR PID','FontSize',16,'FontWeight','bold');

%% =====================================================================
%                     FIGURA 2: CONTROLADOR H-INFINITY
% ======================================================================
figure('Name','Controlador H-infinity','NumberTitle','off', ...
       'Position', [100 20 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0;
    [th,u] = simNL(r, 'HINF');
    e = rad2deg(r - th);
    
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2); hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2);
    grid on; grid minor;
    title(['POSICION H-INF - ' name]);
    ylabel('Ángulo [grados]'); xlabel('Tiempo [s]');
    
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8); hold on;
    yline(1,'r--'); yline(-1,'r--'); yline(0,'k:');
    grid on; grid minor;
    title(['ENTRADA H-INF - ' name]);
    ylabel('u(t) [duty]'); xlabel('Tiempo [s]');
    ylim([-1.2 1.2]);
    
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.2 0.6 0.2], 'LineWidth', 2); hold on;
    yline(0,'k--');
    grid on; grid minor;
    title(['ERROR H-INF - ' name]);
    ylabel('Error [grados]'); xlabel('Tiempo [s]');
end
sgtitle('CONTROLADOR H-INFINITY','FontSize',16,'FontWeight','bold');

%% =====================================================================
%                    FIGURA 3: CONTROLADOR SMC (SLIDING MODE)
% ======================================================================
figure('Name','Controlador SMC (Sliding Mode)','NumberTitle','off', ...
       'Position', [150 0 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1}; 
    r    = profiles{i,2} + th0;
    [th,u] = simNL(r, 'SMC');
    e = rad2deg(r - th);
    
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2); hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2);
    grid on; grid minor;
    title(['POSICION SMC - ' name]);
    ylabel('Ángulo [grados]'); xlabel('Tiempo [s]');
    
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8); hold on;
    yline(1,'r--'); yline(-1,'r--'); yline(0,'k:');
    grid on; grid minor;
    title(['ENTRADA SMC - ' name]);
    ylabel('u(t) [duty]'); xlabel('Tiempo [s]');
    ylim([-1.2 1.2]);
    
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.6 0.2 0.8], 'LineWidth', 2); hold on;
    yline(0,'k--');
    grid on; grid minor;
    title(['ERROR SMC - ' name]);
    ylabel('Error [grados]'); xlabel('Tiempo [s]');
end
sgtitle('CONTROLADOR SMC (SLIDING MODE)','FontSize',16,'FontWeight','bold');

%% =====================================================================
%                FIGURA 4: CONTROLADOR ADAPTATIVO PID (APID)
% ======================================================================
figure('Name','Controlador Adaptativo PID','NumberTitle','off', ...
       'Position', [200 30 1400 900]);

for i=1:size(profiles,1)
    name = profiles{i,1};
    r    = profiles{i,2} + th0;
    [th,u] = simNL(r, 'APID');          % aquí solo usamos theta y u
    e = rad2deg(r - th);
    
    axPos = subplot(3,3,3*(i-1)+1); 
    plot(t, rad2deg(th-th0), 'b-', 'LineWidth', 2); hold on;
    plot(t, rad2deg(r-th0), 'r--', 'LineWidth', 2);
    grid on; grid minor;
    title(['POSICION APID - ' name]);
    ylabel('Ángulo [grados]'); xlabel('Tiempo [s]');
    
    axU = subplot(3,3,3*(i-1)+2); 
    stairs(t, u, 'k-', 'LineWidth', 1.8); hold on;
    yline(1,'r--'); yline(-1,'r--'); yline(0,'k:');
    grid on; grid minor;
    title(['ENTRADA APID - ' name]);
    ylabel('u(t) [duty]'); xlabel('Tiempo [s]');
    ylim([-1.2 1.2]);
    
    axErr = subplot(3,3,3*(i-1)+3); 
    plot(t, e, 'Color', [0.1 0.3 0.7], 'LineWidth', 2); hold on;
    yline(0,'k--');
    grid on; grid minor;
    title(['ERROR APID - ' name]);
    ylabel('Error [grados]'); xlabel('Tiempo [s]');
end
sgtitle('CONTROLADOR ADAPTATIVO PID (APID)','FontSize',16,'FontWeight','bold');

%% =====================================================================
%        Evolución de parámetros Ku_hat y B_hat para el APID (STEP)
% ======================================================================
% Simulamos solo el escalón con APID y recuperamos las estimaciones
[~,~,Ku_hist,B_hist] = simNL(ref_step + th0, 'APID');

figure('Name','Evolución de parámetros del APID','NumberTitle','off');
subplot(2,1,1);
plot(t, Ku_hist, 'LineWidth',2); hold on;
yline(Ku,'r--','LineWidth',1.5);
xline(plantVar.t_change,'k--','Cambio planta');
title('Estimación K_u^{hat}(t)');
xlabel('Tiempo [s]');
ylabel('K_u^{hat}');
legend('Estimado','Valor nominal','Cambio planta','Location','best');
grid on;

subplot(2,1,2);
plot(t, B_hist, 'LineWidth',2); hold on;
yline(Bv,'r--','LineWidth',1.5);
xline(plantVar.t_change,'k--','Cambio planta');
title('Estimación B^{hat}(t)');
xlabel('Tiempo [s]');
ylabel('B^{hat}');
legend('Estimado','Valor nominal','Cambio planta','Location','best');
grid on;

%% =====================================================================
%                 FUNCIÓN DE SIMULACIÓN NO LINEAL DISCRETA
% ======================================================================
function [theta,u_hist,Ku_hist,B_hist]=simulate_nl(J,Bv,MgL,Ku,Ts,ref,mode, ...
                                    Kp,Ki,Kd,tau_d,Tt,Khd,Khinf_d, ...
                                    lambda,k_torque,phi,th0,rls,plantVar)
N        = numel(ref);
theta    = zeros(N,1); 
omega    = 0; 
u_hist   = zeros(N,1);
theta(1) = th0; 
Iint     = 0; 
Df       = 0; 
eprev    = 0;

Ku_hist  = zeros(N,1);
B_hist   = zeros(N,1);

% Control Hinf: elegir controlador discreto
if strcmp(mode,'HINF')
    ctrl = Khinf_d;
else
    ctrl = Khd;
end
[bh,ah] = tfdata(ctrl,'v');

if ~isempty(ah) && ah(1) ~= 0
    bh = bh / ah(1);
    ah = ah / ah(1);
end

n_states = max(numel(bh)-1, numel(ah)-1);
z_states = zeros(n_states, 1);

% Estados para RLS (APID)
Ku_hat = rls.Ku_hat;
B_hat  = rls.B_hat;
P      = rls.P;
lam_rls = rls.lam;
theta_meas_prev = th0;
omega_meas_prev = 0;

for k=1:N-1
  % Tiempo actual
  t_k = (k-1)*Ts;
  
  % Plantas "verdaderas" con posible cambio
  Ku_true = Ku;
  B_true  = Bv;
  if t_k >= plantVar.t_change
      Ku_true = plantVar.Ku_factor * Ku;
      B_true  = plantVar.B_factor  * Bv;
  end
  
  e    = (ref(k)-theta(k));
  u_ff_nom = (MgL/Ku)*sin(ref(k)); %#ok<NASGU> % por si se quiere comparar
  
  if strcmp(mode,'PID')
     % --- PID con filtro derivativo y anti-windup ---
     a   = tau_d/(tau_d+Ts); 
     b   = Kd/(tau_d+Ts);
     Df  = a*Df + b*(e-eprev);
     u_uns = u_ff_nom + (Kp*e + Df + Iint);
     u     = max(-1,min(1,u_uns));
     Iint  = Iint + Ki*Ts*e + (Ts/Tt)*(u - u_uns);
     
  elseif strcmp(mode,'HINF')
     % --- Hinf discreto (IIR forma directa II transpuesta) ---
     x = e;  % entrada al compensador
     
     if ~isempty(bh)
         y = bh(1) * x;
         if n_states > 0
             y = y + z_states(1);
         end
     else
         y = 0;
     end
     
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
     
     u = max(-1,min(1,u_ff_nom + y));
     
  elseif strcmp(mode,'APID')
     % ---------- CONTROL ADAPTATIVO PID + RLS ----------
     % Medición de theta y derivadas
     theta_meas    = theta(k);
     omega_meas    = (theta_meas - theta_meas_prev)/Ts;
     omegadot_meas = (omega_meas - omega_meas_prev)/Ts;
     theta_meas_prev = theta_meas;
     omega_meas_prev = omega_meas;
     
     % RLS: y = J*ddot(theta) + MgL*sin(theta) = [u, -omega]*[Ku;B]
     y_rls   = J*omegadot_meas + MgL*sin(theta_meas);
     u_prev  = u_hist(max(k-1,1));   % entrada aplicada en paso previo
     phi_rls = [u_prev; -omega_meas];
     
     K_rls   = (P*phi_rls)/(lam_rls + phi_rls.'*P*phi_rls);
     err_rls = y_rls - [Ku_hat B_hat]*phi_rls;
     
     theta_par = [Ku_hat; B_hat] + K_rls*err_rls;
     Ku_hat    = max(1e-4, theta_par(1));  % evita valores no físicos
     B_hat     = max(0,    theta_par(2));
     
     P = (P - K_rls*phi_rls.'*P)/lam_rls;
     
     % Guardar estimaciones para análisis
     Ku_hist(k) = Ku_hat;
     B_hist(k)  = B_hat;
     
     % Feed-forward gravitacional usando Ku_hat
     Ku_eff = Ku_hat;
     if Ku_eff <= 1e-4
         Ku_eff = Ku;  % fallback
     end
     u_ff = (MgL/Ku_eff)*sin(ref(k));
     
     % PID clásico sobre el error con anti-windup
     a   = tau_d/(tau_d+Ts); 
     b   = Kd/(tau_d+Ts);
     Df  = a*Df + b*(e-eprev);
     u_uns = u_ff + (Kp*e + Df + Iint);
     u     = max(-1,min(1,u_uns));
     Iint  = Iint + Ki*Ts*e + (Ts/Tt)*(u - u_uns);
     
  else
     % ---------- SMC ----------
     % Superficie deslizante
     s = omega + lambda*(ref(k) - theta(k));
     
     % Derivada de referencia
     if k > 1
         ref_dot = (ref(k) - ref(k-1))/Ts;
     else
         ref_dot = 0;
     end
     
     % Control equivalente
     u_eq = (B_true*omega + MgL*sin(theta(k)) ...
             - J*lambda*(ref_dot - omega))/Ku_true;
     
     % Switching suavizado
     u_sw = -(k_torque/Ku_true) * tanh(s/phi);
     
     % Ley total
     u = u_eq + u_sw;
     u = max(-1,min(1,u));
  end
  
  % Dinámica no lineal (pendulito actuado) con planta "verdadera"
  domega = (Ku_true*u - B_true*omega - MgL*sin(theta(k)))/J;
  omega  = omega + Ts*domega;
  theta(k+1) = theta(k) + Ts*omega;
  u_hist(k)  = u;
  eprev      = e;
end

% Ajuste final de historiales
u_hist(end) = u_hist(end-1);
Ku_hist(end)= Ku_hist(end-1);
B_hist(end) = B_hist(end-1);
end
