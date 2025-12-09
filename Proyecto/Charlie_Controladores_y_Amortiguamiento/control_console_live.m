%% Consola de pruebas — AS5600 + L298N + MPU6050
% Firma: Nadia
% - Selecciona controlador (PID/HINF/SMC/ADAPTIVE_PID)
% - Selecciona perfil (STEP/RAMP/PULSET)
% - Visualiza y exporta métricas (Mp, Ts) y STOP

%% Parámetros de conexión
PORT = "COM3"; BAUD = 115200;
Ts   = 0.01;   Tsim = 15;

controller = "PID";      % 'PID'|'HINF'|'SMC'|'ADAPTIVE_PID'
profile    = "STEP";     % 'STEP'|'RAMP'|'PULSET'

theta_ref_deg = 60; ramp_deg_s = 10; pulse_amp_deg = 15; pulse_T = 1.0;

%% Abrir puerto
sp = serialport(PORT, BAUD); configureTerminator(sp,"LF"); flush(sp);

%% Enviar configuración y preparar STOP
cfg = sprintf("CFG,TS=%.3f,T=%g,MODE=%s,PROF=%s,REF=%g,RAMPS=%g,PAMP=%g,PT=%g", ...
    Ts, Tsim, controller, profile, theta_ref_deg, ramp_deg_s, pulse_amp_deg, pulse_T);
writeline(sp, cfg);

%% Adquisición
N=ceil(Tsim/Ts)+2000; t=nan(N,1); y=t; r=y; u=y; Ku=y; Bb=y; i=0; t0=[];
f = figure('Color','w'); tl = tiledlayout(3,1,'TileSpacing','compact');
ax1 = nexttile; hold on; grid on; ylabel('\theta [deg]');
ax2 = nexttile; hold on; grid on; ylabel('u [duty]');
ax3 = nexttile; hold on; grid on; ylabel('\hat{K}_u / \hat{B}'); xlabel('t [s]');
uicontrol('Style','pushbutton','String','STOP','FontSize',12,'Position',[10 10 80 30],...
    'Callback', @(~,~) writeline(sp,"STOP"));

while true
    if sp.NumBytesAvailable>0
        s=strtrim(readline(sp));
        if startsWith(s,"DONE"), break; end
        C=split(s,','); if numel(C)<9, continue; end
        i=i+1; t(i)=str2double(C{1}); y(i)=str2double(C{2}); r(i)=str2double(C{3});
        u(i)=str2double(C{6}); Ku(i)=str2double(C{7}); Bb(i)=str2double(C{8});
        if isempty(t0), t0=t(i); end; tt=t(1:i)-t0;

        cla(ax1); plot(ax1,tt,y(1:i),'LineWidth',1.2); plot(ax1,tt,r(1:i),'--','LineWidth',1.2);
        title(ax1,sprintf("Modo: %s | Perfil: %s",controller,profile));
        cla(ax2); plot(ax2,tt,u(1:i),'LineWidth',1.2); ylim(ax2,[-1.05 1.05]);
        cla(ax3); yyaxis(ax3,'left'); plot(ax3,tt,Ku(1:i),':','LineWidth',1.2); ylabel(ax3,'\hat{K}_u');
        yyaxis(ax3,'right'); plot(ax3,tt,Bb(1:i),'-.','LineWidth',1.2); ylabel(ax3,'\hat{B}'); xlabel(ax3,'t [s]');
        drawnow limitrate
    else
        pause(0.002);
    end
end
clear sp

%% Métricas y exportación a PDF (solo STEP)
if profile=="STEP"
    idx=1:i; tt=t(idx)-t(idx(1)); yy=y(idx); rr=r(idx); rfin=rr(end);
    [yp,ip]=max(yy); Mp=(yp-rfin)/max(1e-6,abs(rfin))*100;
    iTs=find(abs(yy-rfin)<=0.02*abs(rfin),1,'first'); Ts_est=NaN; if ~isempty(iTs), Ts_est=tt(iTs); end

    fig2=figure('Color','w'); plot(tt,yy,'LineWidth',1.4); grid on; hold on; plot(tt,rr,'--','LineWidth',1.2);
    if ~isnan(Ts_est), xline(Ts_est,':','T_s','LabelHorizontalAlignment','left'); end
    plot(tt(ip),yp,'ro','MarkerFaceColor','r');
    title(sprintf('Escalón: M_p=%.1f%%, T_s=%.2fs',Mp,Ts_est)); xlabel('t [s]'); ylabel('\theta [deg]');

    % Exporta a PDF en la carpeta actual
    exportgraphics(fig2, 'Reporte_Escalon.pdf', 'ContentType','vector');
end
