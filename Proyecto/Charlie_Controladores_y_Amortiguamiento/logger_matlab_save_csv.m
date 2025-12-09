% amortiguamiento_desde_serial.m — Firma: Nadia
clear; clc; close all;

% ---- AJUSTES ----
PORT = "COM3";    % <-- tu COM
BAUD = 115200;
J    = 1.08e-3;   % kg·m^2 (ajústalo a tu planta)
omega_min = deg2rad(5); % umbral para evitar zona de ruido

% ---- Conexión y RUN ----
sp = serialport(PORT, BAUD); configureTerminator(sp,"LF"); flush(sp);
writeline(sp,"RUN"); fprintf(">> RUN enviado. Esperando COAST y BRAKE...\n");

COAST=[]; BRAKE=[]; block=""; gotHeader=false; TIMEOUT=45; tBlock=tic;
while true
    if sp.NumBytesAvailable>0
        s=strtrim(readline(sp));
        if s=="DONE", break; end
        if s=="COAST" || s=="BRAKE"
            block=s; gotHeader=false; tBlock=tic; fprintf(">> Bloque: %s\n",block);
            continue
        end
        if strcmpi(s,"t,theta_deg,omega_deg_s")
            gotHeader=true; continue
        end
        if gotHeader && ~isempty(block)
            C=split(s,','); if numel(C)==3
                row=[str2double(C{1}), str2double(C{2}), str2double(C{3})];
                if all(~isnan(row))
                    if block=="COAST", COAST(end+1,:)=row; else, BRAKE(end+1,:)=row; end %#ok<AGROW>
                end
            end
        end
    else
        pause(0.003);
    end
    if ~isempty(block) && toc(tBlock)>TIMEOUT
        fprintf("Timeout %s.\n",block); block=""; gotHeader=false;
    end
end
clear sp;

% ---- Guardar CSV ----
ts = datestr(now,'yyyymmdd_HHMMSS');
if ~isempty(COAST), fnC=sprintf('coast_encoder_%s.csv',ts); writematrix(COAST,fnC); fprintf("CSV: %s (%d filas)\n",fnC,size(COAST,1)); end
if ~isempty(BRAKE), fnB=sprintf('brake_encoder_%s.csv',ts); writematrix(BRAKE,fnB); fprintf("CSV: %s (%d filas)\n",fnB,size(BRAKE,1)); end

% ---- Fit exponencial ln|w| = m t + b  -> tau = -1/m ----
fit_tau = @(t,w) deal( ...
    -1/polyfit(t, log(w), 1)(1), ...   % tau
     exp(polyfit(t, log(w), 1)(2)) );  % A (no se usa luego)
% Nota: MATLAB no permite indexing así directo; hacemos en dos pasos:
function [tau,A] = tau_from(t,y)
    p = polyfit(t,log(y),1);
    tau = -1/p(1); A = exp(p(2));
end

% COAST -> b = J/tau_open
b = NaN; tau_open = NaN;
if ~isempty(COAST)
    tC = COAST(:,1); wC = deg2rad(COAST(:,3)); yC = abs(wC);
    idx = yC>omega_min; tC2=tC(idx); yC2=yC(idx);
    [tau_open, Aopen] = tau_from(tC2, yC2);
    b = J / tau_open;
    figure('Color','w');
    plot(tC,yC,'b','LineWidth',1.2); hold on;
    plot(tC2, Aopen*exp(-tC2/tau_open),'k--','LineWidth',1.2);
    grid on; title(sprintf('COAST: tau=%.3fs  =>  b=%.3e N·m·s/rad',tau_open,b));
    ylabel('|w| (rad/s)'); xlabel('t (s)');
end

% BRAKE -> Kt^2/R = J/tau_short - J/tau_open
Kt2_over_R = NaN; tau_short=NaN;
if ~isempty(BRAKE) && ~isnan(tau_open)
    tB = BRAKE(:,1); wB = deg2rad(BRAKE(:,3)); yB = abs(wB);
    idx = yB>omega_min; tB2=tB(idx); yB2=yB(idx);
    [tau_short, Ashort] = tau_from(tB2, yB2);
    Kt2_over_R = J/tau_short - J/tau_open;
    figure('Color','w');
    plot(tB,yB,'m','LineWidth',1.2); hold on;
    plot(tB2, Ashort*exp(-tB2/tau_short),'k--','LineWidth',1.2);
    grid on; title(sprintf('BRAKE: tau=%.3fs  =>  Kt^2/R=%.3e',tau_short,Kt2_over_R));
    ylabel('|w| (rad/s)'); xlabel('t (s)');
end

fprintf('\nResultados:\n');
if ~isnan(b), fprintf('  b = %.3e N·m·s/rad (tau_open=%.3fs)\n', b, tau_open); end
if ~isnan(Kt2_over_R), fprintf('  Kt^2/R = %.3e (tau_short=%.3fs)\n', Kt2_over_R, tau_short); end
