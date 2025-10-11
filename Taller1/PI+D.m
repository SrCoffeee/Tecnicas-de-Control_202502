%% CONTROL ROBUSTO Y ANÁLISIS DE DESEMPEÑO: VEHÍCULO AÉREO NO TRIPULADO
% Universidad Nacional de Colombia
% Solución Completa - Modelos Reales del UAV
% Implementación CORRECTA: PI+D Longitudinal + Lateral + Direccional

clear all; close all; clc;

%% ========================================================================
% PARTE 0: CARGA Y PREPARACIÓN DE MODELOS REALES
% ========================================================================

fprintf('========================================\n');
fprintf('CARGA DE MODELOS REALES DEL UAV\n');
fprintf('========================================\n\n');

% MODELO LONGITUDINAL (6 estados, 2 entradas, 7 salidas)
A_longmod = [
    -0.5961   0.8011  -0.8710  -9.7915   0.0001   0.0126;
    -0.7454  -7.5810  15.7162  -0.5272  -0.0009   0;
     1.0417  -7.4269 -15.8518   0       -0.0000  -0.0132;
     0        0        1.0000   0        0        0;
    -0.0538   0.9986   0      -17.0000   0        0;
   135.8430   7.3147   0        0       -0.0827  -5.9187
];

B_longmod = [
     0.4681      0;
    -2.7109      0;
  -134.0661      0;
     0           0;
     0           0;
     0      2506.1
];

C_longmod = [
     0.9986   0.0538   0   0   0   0;
    -0.0032   0.0587   0   0   0   0;
     0        0        1   0   0   0;
     0        0        0   1   0   0;
     0        0        0   0  -1   0;
    -0.5961   0.8011   0.0431   0   0.0001   0.0126;
    -0.7454  -7.5810  -1.2592   0  -0.0009   0
];

D_longmod = zeros(7,2);

% MODELO LATERAL (5 estados, 2 entradas, 6 salidas)
A_latmod = [
    -0.8750   0.8751 -16.8197   9.7914   0;
    -2.8312 -16.1385   3.3768   0        0;
     1.7063   0.5154  -2.7828   0        0;
     0        1.0000   0.0538   0        0;
     0        0        1.0014   0        0
];

B_latmod = [
     0        5.3170;
  -156.9094  -5.0216;
    11.5366 -82.2714;
     0        0;
     0        0
];

C_latmod = [
     0.0588   0   0   0   0;
     0        1   0   0   0;
     0        0   1   0   0;
     0        0   0   1   0;
     0        0   0   0   1;
     0        0   0   0   0
];

D_latmod = zeros(6,2);

% Limpiar valores despreciables (Instrucción 1)
threshold = 1e-3;
A_longmod(abs(A_longmod) < threshold) = 0;
B_longmod(abs(B_longmod) < threshold) = 0;
C_longmod(abs(C_longmod) < threshold) = 0;
A_latmod(abs(A_latmod) < threshold) = 0;
B_latmod(abs(B_latmod) < threshold) = 0;
C_latmod(abs(C_latmod) < threshold) = 0;

fprintf('✓ Matrices limpiadas\n\n');

sys_longmod = ss(A_longmod, B_longmod, C_longmod, D_longmod);
sys_latmod = ss(A_latmod, B_latmod, C_latmod, D_latmod);

fprintf('📊 MODELOS CARGADOS:\n');
fprintf('  Longitudinal: %d estados, %d entradas, %d salidas\n', ...
    size(A_longmod,1), size(B_longmod,2), size(C_longmod,1));
fprintf('  Lateral:      %d estados, %d entradas, %d salidas\n', ...
    size(A_latmod,1), size(B_latmod,2), size(C_latmod,1));

%% ========================================================================
% PARTE 1: ANÁLISIS DE POLOS Y CEROS (Instrucción 2)
% ========================================================================

fprintf('\n========================================\n');
fprintf('ANÁLISIS DE POLOS Y CEROS\n');
fprintf('========================================\n\n');

polos_long = eig(A_longmod);
fprintf('🎯 POLOS LONGITUDINALES:\n');
for i = 1:length(polos_long)
    if abs(imag(polos_long(i))) > 1e-6
        wn = abs(polos_long(i));
        zeta = -real(polos_long(i))/wn;
        fprintf('  p%d = %.4f ± %.4fj  (ωn=%.4f, ζ=%.4f)\n', i, ...
            real(polos_long(i)), abs(imag(polos_long(i))), wn, zeta);
    else
        fprintf('  p%d = %.4f\n', i, polos_long(i));
    end
end

polos_lat = eig(A_latmod);
fprintf('\n🎯 POLOS LATERALES:\n');
for i = 1:length(polos_lat)
    if abs(imag(polos_lat(i))) > 1e-6
        wn = abs(polos_lat(i));
        zeta = -real(polos_lat(i))/wn;
        fprintf('  p%d = %.4f ± %.4fj  (ωn=%.4f, ζ=%.4f)\n', i, ...
            real(polos_lat(i)), abs(imag(polos_lat(i))), wn, zeta);
    else
        fprintf('  p%d = %.4f\n', i, polos_lat(i));
    end
end

if all(real(polos_long) < 0)
    fprintf('\n✓ Sistema longitudinal ESTABLE\n');
else
    fprintf('\n✗ Sistema longitudinal INESTABLE\n');
end

if all(real(polos_lat) < 0)
    fprintf('✓ Sistema lateral ESTABLE\n');
else
    fprintf('✗ Sistema lateral INESTABLE\n');
end

%% ========================================================================
% PARTE 2: BARRIDO DE AMORTIGUAMIENTO (Instrucción 3)
% ========================================================================

fprintf('\n========================================\n');
fprintf('BARRIDO DE AMORTIGUAMIENTO\n');
fprintf('========================================\n\n');

all_poles = [polos_long; polos_lat];
zetas = [];
wns = [];

for i = 1:length(all_poles)
    if abs(imag(all_poles(i))) > 1e-6
        wn = abs(all_poles(i));
        zeta = -real(all_poles(i))/wn;
        zetas = [zetas; zeta];
        wns = [wns; wn];
    end
end

[min_zeta, idx_min] = min(zetas);

fprintf('⚠️  PEOR AMORTIGUAMIENTO: ζ = %.4f\n', min_zeta);
fprintf('    ωn = %.4f rad/s (%.2f Hz)\n\n', wns(idx_min), wns(idx_min)/(2*pi));

%% ========================================================================
% PARTE 3: ANÁLISIS DE ACOPLAMIENTO (Instrucciones 4 y 5)
% ========================================================================

fprintf('========================================\n');
fprintf('ANÁLISIS DE ACOPLAMIENTO\n');
fprintf('========================================\n\n');

% Instrucción 4: Heading
fprintf('🔗 ACOPLAMIENTO DEL HEADING (ψ):\n');
coupling_matrix = abs(A_latmod);
coupling_heading = coupling_matrix(5,:) + coupling_matrix(:,5)';
coupling_heading(5) = 0;

state_names_lat = {'v', 'p', 'r', 'φ', 'ψ'};
[max_coupling, max_idx] = max(coupling_heading);
fprintf('  Estado más acoplado: %s (%.4f)\n\n', state_names_lat{max_idx}, max_coupling);

% Instrucción 5: Vel x Rotación
fprintf('🔄 ACOPLAMIENTO u ↔ q:\n');
fprintf('  u → q: %.4f\n', A_longmod(3,1));
fprintf('  q → u: %.4f\n\n', A_longmod(1,3));

% Instrucción 6: Canal acelerador
fprintf('⚙️  CANAL ACELERADOR: B(6,2) = %.1f\n\n', B_longmod(6,2));

%% ========================================================================
% PARTE 4: VALORES SINGULARES
% ========================================================================

freq = logspace(-4, 3, 1000);

figure('Name', 'Valores Singulares', 'Position', [100 100 1200 400]);
subplot(1,2,1);
sigma(sys_longmod, freq);
title('Valores Singulares - Longitudinal');
grid on;

subplot(1,2,2);
sigma(sys_latmod, freq);
title('Valores Singulares - Lateral');
grid on;

%% ========================================================================
% PARTE 5: DISEÑO PI+D LONGITUDINAL (PITCH)
% ========================================================================

fprintf('========================================\n');
fprintf('DISEÑO PI+D LONGITUDINAL (PITCH)\n');
fprintf('========================================\n\n');

% Extraer G1 (δe→θ) y G2 (δe→q)
[num_G1_lon, den_G1_lon] = ss2tf(A_longmod, B_longmod(:,1), C_longmod(4,:), D_longmod(4,1));
G1_lon = tf(num_G1_lon, den_G1_lon);

[num_G2_lon, den_G2_lon] = ss2tf(A_longmod, B_longmod(:,1), C_longmod(3,:), D_longmod(3,1));
G2_lon = tf(num_G2_lon, den_G2_lon);

fprintf('✓ G1_lon: δe → θ\n');
fprintf('✓ G2_lon: δe → q\n\n');

% Barrido de D longitudinal
D_values_lon = linspace(-1, 1, 200);
min_zeta_lon = zeros(size(D_values_lon));

fprintf('Barrido de D_lon...\n');
for k = 1:length(D_values_lon)
    D = D_values_lon(k);
    try
        G_inner_temp = feedback(G1_lon, D*G2_lon, -1);
        polos_inner = pole(G_inner_temp);
        
        zetas_temp = [];
        for i = 1:length(polos_inner)
            if abs(imag(polos_inner(i))) > 1e-6
                wn = abs(polos_inner(i));
                zeta = -real(polos_inner(i))/wn;
                zetas_temp = [zetas_temp; zeta];
            end
        end
        
        if ~isempty(zetas_temp)
            min_zeta_lon(k) = min(zetas_temp);
        else
            min_zeta_lon(k) = 1;
        end
    catch
        min_zeta_lon(k) = -10;
    end
end

[max_zeta_lon, idx_max_lon] = max(min_zeta_lon);
D_opt_lon = D_values_lon(idx_max_lon);

fprintf('✓ D_lon óptimo = %.4f (ζ = %.4f)\n\n', D_opt_lon, max_zeta_lon);

figure('Name', 'Barrido D - Longitudinal');
plot(D_values_lon, min_zeta_lon, 'b-', 'LineWidth', 2);
hold on;
plot(D_opt_lon, max_zeta_lon, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('D_{lon}'); ylabel('ζ_{min}');
title('Barrido Ganancia Derivativa - LONGITUDINAL (Pitch)');
grid on;
legend('ζ_{min}(D)', sprintf('Óptimo: D=%.3f', D_opt_lon));
hold off;

% Sistema interno longitudinal
G_inner_lon = feedback(G1_lon, D_opt_lon*G2_lon, -1);

% Root Locus Kp
figure('Name', 'Root Locus Kp - Longitudinal');
rlocus(G_inner_lon);
title('Root Locus para Kp_{lon}');
grid on; sgrid([0.3 0.5 0.7], []);

Kp_lon = 0.75;
fprintf('Kp_lon = %.3f\n', Kp_lon);

% Root Locus Ki
s = tf('s');
figure('Name', 'Root Locus Ki - Longitudinal');
% CORRECTO
G_with_Kp = feedback(G_inner_lon * Kp_lon, 1);
rlocus(G_with_Kp / s);title('Root Locus para Ki_{lon}');
grid on; sgrid([0.3 0.5 0.7], []);

Ki_lon = 0.437;
fprintf('Ki_lon = %.3f\n\n', Ki_lon);

% Controlador PI+D longitudinal
C_PI_lon = Kp_lon + Ki_lon/s;
T_lon_pid = feedback(G_inner_lon * C_PI_lon, 1);

fprintf('✓ PI+D LONGITUDINAL:\n');
fprintf('  D  = %.4f\n', D_opt_lon);
fprintf('  Kp = %.4f\n', Kp_lon);
fprintf('  Ki = %.4f\n\n', Ki_lon);
%% %% ============================
%  H-infinity: LONGITUDINAL (δe→θ)
% =============================

% Selecciona canal: entrada δe (col 1), salida θ (fila 4)
[num_Gth, den_Gth] = ss2tf(A_longmod, B_longmod(:,1), C_longmod(4,:), D_longmod(4,1));
Gth = tf(num_Gth, den_Gth);
Gth = minreal(Gth);

% ===== Especificaciones en rad/s =====
wb  = 2*pi*8;   % BW deseada >= 8 Hz
wp  = 2*pi*6;   % perturbación hasta 6 Hz

% ===== Escala de actuador (≈ ±30°) =====
% Normalizamos el canal de control para que "1" ≈ 30°.
u_max_rad = pi/6;   % 30 grados
Su = u_max_rad;     % factor de escala
Gth_n = Gth * Su;   % planta normalizada en la entrada (control "unitario" = 30°)

% ===== Pesos de diseño (mixed-sensitivity) =====
s = tf('s');

% 1) Wp sobre S para seguimiento/robustez (BW >= 8 Hz, |S(0)| pequeño)
%    Forma tipo (s/M + wb)/(s + wb*A)
M  = 1.5;      % bound de pico de S (Ms ≲ M)
A  = 1e-3;     % error estático (S(0) ≲ A)
Wp = (s/M + wb) / (s + wb*A);

% 2) Wu sobre KS para limitar esfuerzo de control (≈ saturación a 1 [=30°])
%    Ganancia constante + ligero roll-off alto
Wu = (1/0.7) * (s/wp + 1) / (s/(6*wp) + 1);
%   penaliza KS en banda útil; 1/0.7 ≈ 1.43 da margen antes de saturar

% 3) Wt sobre T para rechazo de ruido de medición (ruido blanco ~ 1e-4)
%    T pequeño por encima de ~6–8 Hz.
%    Forma tipo (s/Mt + wt)/(s + wt*At) con At pequeño para gran atenuación HF
Mt = 1.3;      % bound de pico de T
At = 1e-3;     % T(w→∞) pequeño
wt = wb;       % transición ~ BW
Wt = (s/Mt + wt) / (s + wt*At);

% ===== Síntesis H∞ (mixed-sensitivity) =====
% min || [Wp*S ; Wu*K*S ; Wt*T] ||∞
[Khinf, CLhinf, gamma] = mixsyn(Gth_n, Wp, Wu, Wt);
Khinf = minreal(Khinf);

fprintf('\n=== H∞ (longitudinal) ===\n');
fprintf('gamma = %.3f\n', gamma);

% Controlador des-normalizado (volver a unidades originales de δe)
% Gth_n = Gth * Su  =>  K_phys = Khinf / Su
K_phys = Khinf / Su;
K_phys = minreal(K_phys);

% ===== Chequeos con sigma =====
S = feedback(1,  Gth_n*Khinf);    % Sensitivity
T = feedback(Gth_n*Khinf, 1);     % Complementary Sensitivity
KS = minreal(series(Khinf, S));   % KS

w = logspace(0, 3, 800); % 1 rad/s a 1000 rad/s

figure('Name','Hinf Checks (sigma) - Longitudinal','Position',[50 50 1200 800]);
subplot(3,1,1);
sigma(Wp*S, w); grid on; title('|Wp*S| (seguimiento/robustez)');
yline(0,':'); % 0 dB
subplot(3,1,2);
sigma(Wu*KS, w); grid on; title('|Wu*K*S| (esfuerzo de control)');
yline(0,':');
subplot(3,1,3);
sigma(Wt*T, w); grid on; title('|Wt*T| (rechazo de ruido)');
yline(0,':');

% Norma Hinf aproximada (máx. valor singular)
fprintf('||Wp*S||∞ ≈ %.2f dB\n', mag2db(max(squeeze(sigma(Wp*S, w)))));
fprintf('||Wu*KS||∞ ≈ %.2f dB\n', mag2db(max(squeeze(sigma(Wu*KS, w)))));
fprintf('||Wt*T||∞ ≈ %.2f dB\n', mag2db(max(squeeze(sigma(Wt*T, w)))));

% ===== Verificación de BW y ruido =====
figure('Name','S & T - Longitudinal'); 
subplot(2,1,1); sigma(S, w); grid on; title('S (sensibilidad)');
xline(wb,'--r','BW target');
subplot(2,1,2); sigma(T, w); grid on; title('T (compl. sensibilidad)');
xline(wp,'--k','noise/dist. cutoff');

% ===== Respuesta temporal y damping =====
T_cl = feedback(Gth*K_phys, 1);  % usar planta sin normalizar y K físico
figure('Name','Step θ (Longitudinal)'); 
step(T_cl, 3); grid on; title('Paso en θ (1 rad)');

% Polos y amortiguamiento aproximado (pares complejos)
p_cl = pole(T_cl);
cc  = p_cl(abs(imag(p_cl))>1e-6);
zeta = -real(cc)./abs(cc);
fprintf('ζ mínimos (pares complejos): ');
if ~isempty(zeta), fprintf('%.3f ', min(zeta)); else, fprintf('N/A '); end
fprintf('\n');

% Estimación de saturación de control ante escalón
[yc,tc,xc] = step(feedback(series(Gth, K_phys), 1), 2);
% señal de control u = K * e  (para referencia unitaria)
% construir lazo para registrar u:
sys_u = feedback(K_phys, Gth);  % desde r a u (cuando se arma de forma estándar)
figure('Name','Control Effort (u)'); 
step(sys_u, 2); grid on; title('Señal de control u (rad). Límite ≈ ±π/6');

% ===== Reporte rápido =====
fprintf('\n[REPORTE H∞ Longitudinal]\n');
fprintf('- BW objetivo: %.2f Hz | BW observada (cruce T≈-3dB): inspeccionar curva T\n', wb/2/pi);
fprintf('- Robustez (incertidumbre aditiva de entrada hasta 6 Hz): ver |Wp*S| y |Wu*KS|\n');
fprintf('- Ruido (longitudinal ~1e-4): ver caída de T por encima de ~6–8 Hz\n');
fprintf('- Esfuerzo de control: revisar step de u, chequear |u|max ≲ %.3f rad (30°)\n', u_max_rad);

%% ========================================================================
% PARTE 6: DISEÑO PI+D LATERAL (ROLL - φ)
% ========================================================================

fprintf('========================================\n');
fprintf('DISEÑO PI+D LATERAL (ROLL)\n');
fprintf('========================================\n\n');

% Extraer G1_roll (δa→φ) y G2_roll (δa→p)
[num_G1_roll, den_G1_roll] = ss2tf(A_latmod, B_latmod(:,1), C_latmod(4,:), D_latmod(4,1));
G1_roll = tf(num_G1_roll, den_G1_roll);

[num_G2_roll, den_G2_roll] = ss2tf(A_latmod, B_latmod(:,1), C_latmod(2,:), D_latmod(2,1));
G2_roll = tf(num_G2_roll, den_G2_roll);

fprintf('✓ G1_roll: δa → φ (roll angle)\n');
fprintf('✓ G2_roll: δa → p (roll rate)\n\n');

% Barrido de D roll
D_values_roll = linspace(-1, 1, 200);
min_zeta_roll = zeros(size(D_values_roll));

fprintf('Barrido de D_roll...\n');
for k = 1:length(D_values_roll)
    D = D_values_roll(k);
    try
        G_inner_temp = feedback(G1_roll, D*G2_roll, -1);
        polos_inner = pole(G_inner_temp);
        
        zetas_temp = [];
        for i = 1:length(polos_inner)
            if abs(imag(polos_inner(i))) > 1e-6
                wn = abs(polos_inner(i));
                zeta = -real(polos_inner(i))/wn;
                zetas_temp = [zetas_temp; zeta];
            end
        end
        
        if ~isempty(zetas_temp)
            min_zeta_roll(k) = min(zetas_temp);
        else
            min_zeta_roll(k) = 1;
        end
    catch
        min_zeta_roll(k) = -10;
    end
end

[max_zeta_roll, idx_max_roll] = max(min_zeta_roll);
D_opt_roll = D_values_roll(idx_max_roll);

fprintf('✓ D_roll óptimo = %.4f (ζ = %.4f)\n\n', D_opt_roll, max_zeta_roll);

figure('Name', 'Barrido D - Roll');
plot(D_values_roll, min_zeta_roll, 'b-', 'LineWidth', 2);
hold on;
plot(D_opt_roll, max_zeta_roll, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('D_{roll}'); ylabel('ζ_{min}');
title('Barrido Ganancia Derivativa - LATERAL (Roll)');
grid on;
legend('ζ_{min}(D)', sprintf('Óptimo: D=%.3f', D_opt_roll));
hold off;

% Sistema interno roll
G_inner_roll = feedback(G1_roll, D_opt_roll*G2_roll, -1);

% Root Locus Kp roll
figure('Name', 'Root Locus Kp - Roll');
rlocus(G_inner_roll);
title('Root Locus para Kp_{roll}');
grid on; sgrid([0.3 0.5 0.7], []);

Kp_roll = 0.7;
fprintf('Kp_roll = %.3f\n', Kp_roll);

% Root Locus Ki roll
figure('Name', 'Root Locus Ki - Roll');
% CORRECTO
G_with_Kp_roll = feedback(G_inner_roll * Kp_roll, 1);
rlocus(G_with_Kp_roll / s);
title('Root Locus para Ki_{roll}');
grid on; sgrid([0.3 0.5 0.7], []);


Ki_roll = 0.05;
fprintf('Ki_roll = %.3f\n\n', Ki_roll);

% Controlador PI+D roll
C_PI_roll = Kp_roll + Ki_roll/s;
T_roll = feedback(G_inner_roll * C_PI_roll, 1);

fprintf('✓ PI+D ROLL:\n');
fprintf('  D  = %.4f\n', D_opt_roll);
fprintf('  Kp = %.4f\n', Kp_roll);
fprintf('  Ki = %.4f\n\n', Ki_roll);

%% ========================================================================
% PARTE 7: DISEÑO PI+D DIRECCIONAL (YAW - ψ)
% ========================================================================

fprintf('========================================\n');
fprintf('DISEÑO PI+D DIRECCIONAL (YAW)\n');
fprintf('========================================\n\n');

% Extraer G1_yaw (δr→ψ) y G2_yaw (δr→r)
[num_G1_yaw, den_G1_yaw] = ss2tf(A_latmod, B_latmod(:,2), C_latmod(5,:), D_latmod(5,2));
G1_yaw = tf(num_G1_yaw, den_G1_yaw);

[num_G2_yaw, den_G2_yaw] = ss2tf(A_latmod, B_latmod(:,2), C_latmod(3,:), D_latmod(3,2));
G2_yaw = tf(num_G2_yaw, den_G2_yaw);

fprintf('✓ G1_yaw: δr → ψ (yaw angle)\n');
fprintf('✓ G2_yaw: δr → r (yaw rate)\n\n');

% Barrido de D yaw
D_values_yaw = linspace(-1, 1, 200);
min_zeta_yaw = zeros(size(D_values_yaw));

fprintf('Barrido de D_yaw...\n');
for k = 1:length(D_values_yaw)
    D = D_values_yaw(k);
    try
        G_inner_temp = feedback(G1_yaw, D*G2_yaw, -1);
        polos_inner = pole(G_inner_temp);
        
        zetas_temp = [];
        for i = 1:length(polos_inner)
            if abs(imag(polos_inner(i))) > 1e-6
                wn = abs(polos_inner(i));
                zeta = -real(polos_inner(i))/wn;
                zetas_temp = [zetas_temp; zeta];
            end
        end
        
        if ~isempty(zetas_temp)
            min_zeta_yaw(k) = min(zetas_temp);
        else
            min_zeta_yaw(k) = 1;
        end
    catch
        min_zeta_yaw(k) = -10;
    end
end

[max_zeta_yaw, idx_max_yaw] = max(min_zeta_yaw);
D_opt_yaw = D_values_yaw(idx_max_yaw);

fprintf('✓ D_yaw óptimo = %.4f (ζ = %.4f)\n\n', D_opt_yaw, max_zeta_yaw);

figure('Name', 'Barrido D - Yaw');
plot(D_values_yaw, min_zeta_yaw, 'b-', 'LineWidth', 2);
hold on;
plot(D_opt_yaw, max_zeta_yaw, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('D_{yaw}'); ylabel('ζ_{min}');
title('Barrido Ganancia Derivativa - DIRECCIONAL (Yaw)');
grid on;
legend('ζ_{min}(D)', sprintf('Óptimo: D=%.3f', D_opt_yaw));
hold off;

% Sistema interno yaw
G_inner_yaw = feedback(G1_yaw, D_opt_yaw*G2_yaw, -1);

% Root Locus Kp yaw
figure('Name', 'Root Locus Kp - Yaw');
rlocus(G_inner_yaw);
title('Root Locus para Kp_{yaw}');
grid on; sgrid([0.3 0.5 0.7], []);

Kp_yaw = 0.7;
fprintf('Kp_yaw = %.3f\n', Kp_yaw);

% Root Locus Ki yaw
figure('Name', 'Root Locus Ki - Yaw');
% CORRECTO
G_with_Kp_yaw = feedback(G_inner_yaw * Kp_yaw, 1);
rlocus(G_with_Kp_yaw / s);
title('Root Locus para Ki_{yaw}');
grid on; sgrid([0.3 0.5 0.7], []);

Ki_yaw = 0.1;
fprintf('Ki_yaw = %.3f\n\n', Ki_yaw);

% Controlador PI+D yaw
C_PI_yaw = Kp_yaw + Ki_yaw/s;
T_yaw = feedback(G_inner_yaw * C_PI_yaw, 1);

fprintf('✓ PI+D YAW:\n');
fprintf('  D  = %.4f\n', D_opt_yaw);
fprintf('  Kp = %.4f\n', Kp_yaw);
fprintf('  Ki = %.4f\n\n', Ki_yaw);

%% ========================================================================
% PARTE 8: DISEÑO H-INFINITY
% ========================================================================

fprintf('========================================\n');
fprintf('DISEÑO H∞\n');
fprintf('========================================\n\n');

M = 2; w0 = 10; epsilon = 0.01;
W1 = tf([1/M w0], [1 w0*epsilon]);
W2 = tf([1 10], [1 100]);
W3 = tf([1 20], [0.5 20]);

P_lon = augw(G1_lon, W1, W2, W3);
P_roll = augw(G1_roll, W1, W2, W3);
P_yaw = augw(G1_yaw, W1, W2, W3);

[K_hinf_lon, ~, gamma_lon] = hinfsyn(P_lon, 1, 1);
[K_hinf_roll, ~, gamma_roll] = hinfsyn(P_roll, 1, 1);
[K_hinf_yaw, ~, gamma_yaw] = hinfsyn(P_yaw, 1, 1);

T_lon_hinf = feedback(G1_lon * K_hinf_lon, 1);
T_roll_hinf = feedback(G1_roll * K_hinf_roll, 1);
T_yaw_hinf = feedback(G1_yaw * K_hinf_yaw, 1);

fprintf('✓ γ_lon  = %.3f\n', gamma_lon);
fprintf('✓ γ_roll = %.3f\n', gamma_roll);
fprintf('✓ γ_yaw  = %.3f\n\n', gamma_yaw);

%% ========================================================================
% PARTE 9: COMPARACIONES PI+D vs H∞
% ========================================================================

figure('Name', 'Comparación PI+D vs H∞', 'Position', [100 100 1400 400]);

subplot(1,3,1);
step(T_lon_pid, 'b-', T_lon_hinf, 'r--', 10);
title('LONGITUDINAL (θ)');
xlabel('Tiempo (s)'); ylabel('θ (rad)');
legend('PI+D', 'H∞', 'Location', 'best');
grid on;

subplot(1,3,2);
step(T_roll, 'b-', T_roll_hinf, 'r--', 10);
title('LATERAL (φ - Roll)');
xlabel('Tiempo (s)'); ylabel('φ (rad)');
legend('PI+D', 'H∞', 'Location', 'best');
grid on;

subplot(1,3,3);
step(T_yaw, 'b-', T_yaw_hinf, 'r--', 10);
title('DIRECCIONAL (ψ - Yaw)');
xlabel('Tiempo (s)'); ylabel('ψ (rad)');
legend('PI+D', 'H∞', 'Location', 'best');
grid on;

