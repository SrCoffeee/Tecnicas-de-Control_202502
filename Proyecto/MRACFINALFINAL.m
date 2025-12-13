%% ============================================================
%%    CONTROL ADAPTATIVO MRAC PARA BRAZO-MOTOR (theta0=50°)
%% ============================================================
clear; close all; clc;

%% ===== Parámetros nominales de la planta =====
J   = 1.08e-3;      
Bv  = 2.61e-3;      
MgL = 4.999e-3;     
Ku  = 1.13e-2;      
th0 = deg2rad(50);  
Ts  = 0.01;         

%% ===== Cambio de planta =====
plantVar.t_change  = 4.0;
plantVar.Ku_factor = 0.6;
plantVar.B_factor  = 2.0;

%% ===== Parámetros MRAC =====
mrac.wn_ref   = 2.5;
mrac.zeta_ref = 0.85;

mrac.gamma_theta = 100;
mrac.gamma_k     = 60;

mrac.theta_hat = [0.3; 0.3; 0.3];
mrac.k_hat     = 0.8;

mrac.theta_max = 50;
mrac.theta_min = -50;
mrac.k_max     = 10;
mrac.k_min     = 0.01;

%% ===== Señales de referencia =====
Tend = 12;
t    = 0:Ts:Tend;

ref_step = deg2rad(50)*ones(size(t));
ref_ramp = deg2rad(10)*t;
Aimp     = deg2rad(8);
Timp     = 1;
ref_puls = (mod(t,Timp)<Timp/2)*Aimp - ...
           (mod(t,Timp)>=Timp/2)*Aimp;

profiles = {'ESCALÓN', ref_step;
            'RAMPA',   ref_ramp;
            'PULSOS',  ref_puls};

%% ============================================================
%% FIGURA 1: RESPUESTA MRAC (POSICIÓN Y CONTROL)
%% ============================================================
figure('Name','MRAC - Respuesta Temporal','Color','w',...
       'Position',[50 50 1600 850]);

for i = 1:size(profiles,1)
    name = profiles{i,1};
    r    = profiles{i,2} + th0;
    
    [th, u, th_m] = simulate_mrac(J,Bv,MgL,Ku,Ts,r,mrac,plantVar,th0);
    
    % ---- POSICIÓN ----
    subplot(3,2,2*i-1)
    plot(t, rad2deg(th-th0),'b','LineWidth',2.5); hold on;
    plot(t, rad2deg(th_m),'g--','LineWidth',2);
    plot(t, rad2deg(r-th0),'r--','LineWidth',2);
    xline(plantVar.t_change,'k:','LineWidth',1.5);
    grid on; grid minor;
    title([name ' - Posición'],'FontWeight','bold');
    ylabel('Ángulo [°]');
    xlabel('Tiempo [s]');
    legend('Salida','Modelo ref','Referencia','Location','best');
    
    % ---- CONTROL ----
    subplot(3,2,2*i)
    stairs(t,u,'k','LineWidth',2); hold on;
    yline(1,'r--'); yline(-1,'r--');
    xline(plantVar.t_change,'k:','LineWidth',1.5);
    grid on; grid minor;
    title([name ' - Señal de control'],'FontWeight','bold');
    ylabel('u(t)');
    xlabel('Tiempo [s]');
    ylim([-1.3 1.3]);
end

sgtitle('CONTROLADOR MRAC – RESPUESTA TEMPORAL','FontSize',16,'FontWeight','bold');

%% ============================================================
%% FIGURA 2: EVOLUCIÓN DE PARÁMETROS ADAPTATIVOS
%% ============================================================
theta1_all = zeros(length(t),3);
theta2_all = zeros(length(t),3);
theta3_all = zeros(length(t),3);
k_all      = zeros(length(t),3);

for i = 1:3
    r = profiles{i,2} + th0;
    [~,~,~,theta_hist,k_hist] = ...
        simulate_mrac(J,Bv,MgL,Ku,Ts,r,mrac,plantVar,th0);
    theta1_all(:,i) = theta_hist(:,1);
    theta2_all(:,i) = theta_hist(:,2);
    theta3_all(:,i) = theta_hist(:,3);
    k_all(:,i)      = k_hist;
end

figure('Name','MRAC - Parámetros Adaptativos','Color','w',...
       'Position',[100 100 1600 850]);

labels = {'ESCALÓN','RAMPA','PULSOS'};
colors = lines(3);

subplot(2,2,1)
plot(t,theta1_all,'LineWidth',2.5);
xline(plantVar.t_change,'k--'); grid on;
title('\theta_1(t)'); legend(labels);

subplot(2,2,2)
plot(t,theta2_all,'LineWidth',2.5);
xline(plantVar.t_change,'k--'); grid on;
title('\theta_2(t)'); legend(labels);

subplot(2,2,3)
plot(t,theta3_all,'LineWidth',2.5);
xline(plantVar.t_change,'k--'); grid on;
title('\theta_3(t)'); legend(labels);

subplot(2,2,4)
plot(t,k_all,'LineWidth',2.5);
xline(plantVar.t_change,'k--'); grid on;
title('k(t)'); legend(labels);

sgtitle('EVOLUCIÓN DE PARÁMETROS ADAPTATIVOS – MRAC',...
        'FontSize',16,'FontWeight','bold');

%% ============================================================
%% FUNCIÓN DE SIMULACIÓN MRAC
%% ============================================================
function [theta,u_hist,theta_m_hist,theta_hist,k_hist] = ...
         simulate_mrac(J,Bv,MgL,Ku,Ts,ref,mrac,plantVar,th0)

N = numel(ref);
theta = zeros(N,1); theta(1)=th0;
omega = 0; u_hist=zeros(N,1);

theta_hat = mrac.theta_hat;
k_hat = mrac.k_hat;
theta_hist=zeros(N,3); k_hist=zeros(N,1);

xm1=0; xm2=0; theta_m_hist=zeros(N,1);

for k=1:N-1
    t = (k-1)*Ts;
    
    Ku_t = Ku; Bv_t = Bv;
    if t>=plantVar.t_change
        Ku_t = plantVar.Ku_factor*Ku;
        Bv_t = plantVar.B_factor*Bv;
    end
    
    % Modelo referencia
    xm1_dot = xm2;
    xm2_dot = -mrac.wn_ref^2*xm1 ...
              -2*mrac.zeta_ref*mrac.wn_ref*xm2 ...
              +mrac.wn_ref^2*(ref(k)-th0);
    xm1 = xm1 + Ts*xm1_dot;
    xm2 = xm2 + Ts*xm2_dot;
    theta_m_hist(k)=xm1;
    
    % Error compuesto
    e1 = (theta(k)-th0)-xm1;
    e2 = omega-xm2;
    e  = e1+0.3*e2;
    
    omega_vec = [(theta(k)-th0); omega; (ref(k)-th0)];
    
    u = theta_hat'*omega_vec + k_hat*(ref(k)-th0);
    u = u + (MgL/Ku)*sin(ref(k));
    u = max(-1,min(1,u));
    
    theta_hat = theta_hat - Ts*mrac.gamma_theta*omega_vec*e;
    k_hat     = k_hat - Ts*mrac.gamma_k*(ref(k)-th0)*e;
    
    theta_hat = max(mrac.theta_min,min(mrac.theta_max,theta_hat));
    k_hat     = max(mrac.k_min,min(mrac.k_max,k_hat));
    
    theta_hist(k,:)=theta_hat';
    k_hist(k)=k_hat;
    
    domega=(Ku_t*u-Bv_t*omega-MgL*sin(theta(k)))/J;
    omega=omega+Ts*domega;
    theta(k+1)=theta(k)+Ts*omega;
    
    u_hist(k)=u;
end
end
