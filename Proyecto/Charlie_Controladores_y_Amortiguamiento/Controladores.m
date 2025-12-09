% ================================================================
%  control_console.m — Consola MATLAB para Arduino (AS5600+MPU6050+L298N)
%  Firma: Nadia
%  Funciones:
%   - Selección de controlador: 'PID' | 'HINF' | 'SMC' | 'ADAPTIVE_PID'
%   - Selección de perfil: 'STEP' | 'RAMP' | 'PULSET'
%   - Gráficas con anotaciones (pico, Mp, Ts)
%   - Botón STOP (envía 'STOP\n')
%  Requisitos: R2020b+ (serialport)
% ================================================================
clear; clc; close all;

% ---------- CONFIG ----------
PORT = "COM3";       % <-- AJUSTA
BAUD = 115200;
Ts   = 0.01;         % debe coincidir con Arduino
Tsim = 15;           % [s] duración de prueba

controller = "PID";         % 'PID'|'HINF'|'SMC'|'ADAPTIVE_PID'
profile    = "STEP";        % 'STEP'|'RAMP'|'PULSET'

theta_ref_deg = 60;         % para STEP
ramp_deg_s    = 10;         % para RAMP (pendiente en deg/s)
pulse_amp_deg = 15;         % amplitud de impulsos (deg)
pulse_T       = 1.0;        % periodo del tren de impulsos (s)

% ---------- SERIAL ----------
fprintf("Abriendo %s @ %d...\n", PORT, BAUD);
sp = serialport(PORT, BAUD);
configureTerminator(sp, "LF");
flush(sp);

% ---------- UI STOP ----------
fig = figure('Name','Consola Control','NumberTitle','off','Color','w');
ax1 = subplot(2,1,1,'Parent',fig); hold(ax1,'on'); grid(ax1,'on');
ylabel(ax1,'\theta [deg]');
ax2 = subplot(2,1,2,'Parent',fig); hold(ax2,'on'); grid(ax2,'on');
ylabel(ax2,'u [duty]'); xlabel(ax2,'t [s]');
btn = uicontrol('Style','pushbutton','String','STOP','FontSize',12, ...
    'Position',[10 10 80 30], 'Callback', @(~,~) writeline(sp, "STOP"));

% ---------- ENVÍA CONFIG ----------
cfg = sprintf("CFG,TS=%.3f,T=%g,MODE=%s,PROF=%s,REF=%g,RAMPS=%g,PAMP=%g,PT=%g",
    Ts, Tsim, controller, profile, theta_ref_deg, ramp_deg_s, pulse_amp_deg, pulse_T);
writeline(sp, cfg);

% ---------- LOGS ----------
N = ceil(Tsim/Ts)+1000;
t = NaN(N,1); theta = NaN(N,1); ref = NaN(N,1); u = NaN(N,1); e = NaN(N,1);
Ku_hat = NaN(N,1); B_hat = NaN(N,1);

i = 0; t0 = [];
fprintf("Esperando telemetría...\n");
while true
    if sp.NumBytesAvailable>0
        line = strtrim(readline(sp));
        if strlength(line)==0, continue; end
        if startsWith(line,"DONE")
            break;
        end
        % CSV: t,deg,ref,deg,e,deg,omega,deg_s,u,KuHat,BHat,mode
        C = split(line, ',');
        if numel(C) < 11, continue; end
        i = i+1;
        t(i)     = str2double(C{1});
        theta(i) = str2double(C{2});
        ref(i)   = str2double(C{3});
        u(i)     = str2double(C{9});
        Ku_hat(i)= str2double(C{10});
        B_hat(i) = str2double(C{11});

        % Plot streaming
        if isempty(t0), t0=t(i); end
        tt = t(1:i)-t0;
        if isvalid(ax1)
            cla(ax1); plot(ax1, tt, theta(1:i), 'LineWidth',1.2);
            plot(ax1, tt, ref(1:i), '--', 'LineWidth',1.2);
            title(ax1, sprintf("Modo: %s | Perfil: %s", controller, profile));
        end
        if isvalid(ax2)
            cla(ax2); plot(ax2, tt, u(1:i), 'LineWidth',1.2);
            yyaxis(ax2,'right'); plot(ax2, tt, Ku_hat(1:i),':', tt, B_hat(1:i),'-.','LineWidth',1.0);
            ylabel(ax2,'\hat{K}_u , \hat{B}');
            yyaxis(ax2,'left'); ylabel(ax2,'u [duty]');
        end
        drawnow limitrate
    else
        pause(0.002);
    end
end

% ---------- MÉTRICAS DE ESCALÓN (si aplica) ----------
if profile=="STEP"
    idx = find(~isnan(theta),1):i;
    tt  = t(idx)-t(idx(1));
    y   = theta(idx);
    r   = ref(idx);
    rfin = r(end);
    ynorm = y / rfin;
    % pico
    [yp, ip] = max(y); Mp = (yp-rfin)/rfin*100;
    % Ts (±2%)
    idxTs = find(abs(y - rfin) <= 0.02*abs(rfin),1,'first');
    if ~isempty(idxTs), Ts_est = tt(idxTs); else, Ts_est = NaN; end

    figure('Color','w'); plot(tt, y, 'LineWidth',1.4); grid on; hold on;
    plot(tt, r, '--', 'LineWidth',1.2);
    if ~isnan(Ts_est), xline(Ts_est,':','T_s','LabelHorizontalAlignment','left'); end
    plot(tt(ip), yp,'ro','MarkerSize',6,'DisplayName','Pico');
    title(sprintf('Escalón — Mp=%.1f%%, T_s≈%.2fs',Mp, Ts_est)); xlabel('t [s]'); ylabel('\theta [deg]');
end

% ---------- FIN ----------
clear sp
fprintf("Listo.\n");
