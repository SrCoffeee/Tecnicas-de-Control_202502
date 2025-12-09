% id_from_logs.m — Firma: Nadia
% Usa logs de spin_down_id.ino para estimar b, Kt^2/R y, aparte,
% a partir de una respuesta al escalón, estima zeta, wn (si aportas datos).
clear; clc;

% ----- CARGA DE LOGS -----
% Si capturaste desde Serial, guarda dos CSV: coast.csv y brake.csv con columnas t,omega
coast = readmatrix('coast.csv');   % [t s, omega rad/s]
brake = readmatrix('brake.csv');

% Parámetros conocidos:
J   = 1.08e-3;     % kg·m^2
MgL = 4.999e-3;    % N·m

% Ajuste exponencial omega(t)=omega0*exp(-t/tau)
fexp = @(x,t) x(1)*exp(-t/x(2));
x0 = [coast(1,2), 2];              % guess
x_co = lsqcurvefit(fexp, x0, coast(:,1), coast(:,2));
x_br = lsqcurvefit(fexp, [brake(1,2), 0.5], brake(:,1), brake(:,2));
tau_open  = x_co(2);
tau_short = x_br(2);

b  = J / tau_open;
Kt2_over_R = J/tau_short - J/tau_open;

fprintf('b (viscoso) = %.3e N·m·s/rad\n', b);
fprintf('K_t^2/R     = %.3e N^2·m^2/Ohm\n', Kt2_over_R);

% ----- (Opcional) zeta, wn desde una respuesta al escalón -----
try
  stepd = readmatrix('step_log.csv');   % t,theta_deg,ref_deg,u,...
  t = stepd(:,1); y = deg2rad(stepd(:,2)); r = deg2rad(stepd(:,3));
  rfin = r(end); ytil = y/rfin;
  % log decrement
  [pks,locs] = findpeaks(ytil); if numel(pks)>=2
      d = log(pks(1)/pks(2));
      zeta = d/sqrt(4*pi^2 + d^2);
      T = t(locs(2))-t(locs(1)); wn = 2*pi/(T*sqrt(1-zeta^2));
      fprintf('zeta = %.3f, wn = %.3f rad/s\n', zeta, wn);
      % Si conoces theta0: J = MgL*cos(theta0)/wn^2  (linealización)
  end
catch
  fprintf('(Salto zeta/wn: no se encontró step_log.csv)\n');
end
