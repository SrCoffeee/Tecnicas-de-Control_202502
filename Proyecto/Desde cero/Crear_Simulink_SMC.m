%% Script para crear modelo Simulink:  Controlador SMC con Perturbaciones
% Ejecutar este script para generar el modelo automáticamente

clear; clc;

% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

% Parámetros SMC
lambda    = 3.0;    % rad/s
k_torque  = 0.02;   % N·m
phi       = 0.5;    % rad/s

% Crear nuevo modelo
modelName = 'Control_SMC_con_Perturbaciones';
try
    close_system(modelName, 0);
catch
end
new_system(modelName);
open_system(modelName);

% Posiciones
pos_step = [30, 100, 60, 130];
pos_smc = [200, 60, 320, 180];
pos_sat = [390, 100, 430, 130];
pos_gain_ku = [480, 100, 520, 130];
pos_sum_pert = [580, 105, 600, 125];
pos_plant = [660, 60, 760, 180];
pos_scope1 = [840, 95, 880, 135];
pos_scope2 = [840, 205, 880, 245];
pos_pert = [480, 200, 540, 230];

%% Crear el código SMC como string
smc_code = sprintf([...
    'function u = fcn(ref, theta, omega)\n' ...
    '%%%% Controlador SMC para péndulo\n' ... 
    'J = %. 10e;\n' ... 
    'Bv = %.10e;\n' ... 
    'MgL = %.10e;\n' ...
    'Ku = %. 10e;\n' ...
    'lambda = %.10e;\n' ...
    'k_torque = %.10e;\n' ...
    'phi = %.10e;\n\n' ...
    '%%%% Error de posición\n' ... 
    'e_theta = theta - ref;\n\n' ...
    '%%%% Superficie deslizante:  s = omega + lambda*e_theta\n' ...
    's = omega + lambda * e_theta;\n\n' ... 
    '%%%% Control equivalente\n' ...
    'u_eq = (J*(-lambda*omega) + Bv*omega + MgL*sin(theta)) / Ku;\n\n' ... 
    '%%%% Término de conmutación con tanh (suavizado)\n' ...
    'u_sw = -(k_torque/Ku) * tanh(s/phi);\n\n' ... 
    '%%%% Ley de control total\n' ... 
    'u = u_eq + u_sw;\n' ...
    'end\n'], ... 
    J, Bv, MgL, Ku, lambda, k_torque, phi);

%% Guardar código SMC en archivo temporal
smcFileName = 'smc_controller_fcn.m';
fid = fopen(smcFileName, 'w');
fprintf(fid, '%s', smc_code);
fclose(fid);

%% Agregar bloques

% 1. Escalón de referencia
add_block('simulink/Sources/Step', [modelName '/Referencia']);
set_param([modelName '/Referencia'], 'Position', pos_step);
set_param([modelName '/Referencia'], 'Time', '0');
set_param([modelName '/Referencia'], 'After', num2str(th0));

% 2. Controlador SMC usando Interpreted MATLAB Function (más simple)
% Usaremos 3 bloques Fcn separados combinados, o un subsistema
% ALTERNATIVA: Usar un Subsystem con MATLAB Function interno

% Crear subsistema para el controlador SMC
add_block('built-in/Subsystem', [modelName '/SMC_Controller']);
set_param([modelName '/SMC_Controller'], 'Position', pos_smc);

% Limpiar subsistema
subsysSMC = [modelName '/SMC_Controller'];
lines = find_system(subsysSMC, 'FindAll', 'on', 'Type', 'line');
for i = 1:length(lines)
    try delete_line(lines(i)); catch, end
end
blocks = find_system(subsysSMC, 'SearchDepth', 1, 'Type', 'block');
for i = 1:length(blocks)
    if ~strcmp(blocks{i}, subsysSMC)
        try delete_block(blocks{i}); catch, end
    end
end

% Agregar puertos de entrada al subsistema SMC
add_block('simulink/Sources/In1', [subsysSMC '/ref']);
set_param([subsysSMC '/ref'], 'Position', [20, 30, 50, 50], 'Port', '1');

add_block('simulink/Sources/In1', [subsysSMC '/theta']);
set_param([subsysSMC '/theta'], 'Position', [20, 80, 50, 100], 'Port', '2');

add_block('simulink/Sources/In1', [subsysSMC '/omega']);
set_param([subsysSMC '/omega'], 'Position', [20, 130, 50, 150], 'Port', '3');

% Agregar Mux para combinar entradas
add_block('simulink/Signal Routing/Mux', [subsysSMC '/Mux']);
set_param([subsysSMC '/Mux'], 'Position', [100, 50, 105, 130], 'Inputs', '3');

% Agregar bloque Fcn para implementar SMC
add_block('simulink/User-Defined Functions/Fcn', [subsysSMC '/SMC_Fcn']);
set_param([subsysSMC '/SMC_Fcn'], 'Position', [150, 70, 350, 110]);

% Expresión del controlador SMC
smc_expr = sprintf(['((%. 10e*(-%. 10e*u(3)) + %.10e*u(3) + %.10e*sin(u(2)))/%.10e) ' ...
    '- (%.10e/%.10e)*((2/(1+exp(-2*(u(3)+%. 10e*(u(2)-u(1)))/%.10e)))-1)'], ...
    J, lambda, Bv, MgL, Ku, ...   % u_eq
    k_torque, Ku, lambda, phi); % u_sw con tanh aproximado
set_param([subsysSMC '/SMC_Fcn'], 'Expr', smc_expr);

% Salida del subsistema
add_block('simulink/Sinks/Out1', [subsysSMC '/u_out']);
set_param([subsysSMC '/u_out'], 'Position', [400, 85, 430, 105]);

% Conexiones dentro del subsistema SMC
add_line(subsysSMC, 'ref/1', 'Mux/1');
add_line(subsysSMC, 'theta/1', 'Mux/2');
add_line(subsysSMC, 'omega/1', 'Mux/3');
add_line(subsysSMC, 'Mux/1', 'SMC_Fcn/1');
add_line(subsysSMC, 'SMC_Fcn/1', 'u_out/1');

% 3. Saturación
add_block('simulink/Discontinuities/Saturation', [modelName '/Saturacion']);
set_param([modelName '/Saturacion'], 'Position', pos_sat);
set_param([modelName '/Saturacion'], 'UpperLimit', '1');
set_param([modelName '/Saturacion'], 'LowerLimit', '-1');

% 4. Ganancia Ku
add_block('simulink/Math Operations/Gain', [modelName '/Ku']);
set_param([modelName '/Ku'], 'Position', pos_gain_ku);
set_param([modelName '/Ku'], 'Gain', num2str(Ku));

% 5. Sumador para perturbación
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Perturbacion']);
set_param([modelName '/Sum_Perturbacion'], 'Position', pos_sum_pert);
set_param([modelName '/Sum_Perturbacion'], 'Inputs', '++');

% 6. Perturbación (pulsos)
add_block('simulink/Sources/Pulse Generator', [modelName '/Perturbacion']);
set_param([modelName '/Perturbacion'], 'Position', pos_pert);
set_param([modelName '/Perturbacion'], 'Amplitude', '0.002');
set_param([modelName '/Perturbacion'], 'Period', '3');
set_param([modelName '/Perturbacion'], 'PulseWidth', '20');

% 7. Planta no lineal
add_block('built-in/Subsystem', [modelName '/Planta_NoLineal']);
set_param([modelName '/Planta_NoLineal'], 'Position', pos_plant);

% Limpiar subsistema planta
subsysPath = [modelName '/Planta_NoLineal'];
lines = find_system(subsysPath, 'FindAll', 'on', 'Type', 'line');
for i = 1:length(lines)
    try delete_line(lines(i)); catch, end
end
blocks = find_system(subsysPath, 'SearchDepth', 1, 'Type', 'block');
for i = 1:length(blocks)
    if ~strcmp(blocks{i}, subsysPath)
        try delete_block(blocks{i}); catch, end
    end
end

% Contenido del subsistema planta
add_block('simulink/Sources/In1', [subsysPath '/Torque_in']);
set_param([subsysPath '/Torque_in'], 'Position', [20, 50, 50, 70]);

add_block('simulink/User-Defined Functions/Fcn', [subsysPath '/MgL_sin']);
set_param([subsysPath '/MgL_sin'], 'Position', [150, 150, 250, 180]);
set_param([subsysPath '/MgL_sin'], 'Expr', [num2str(MgL) '*sin(u(1))']);

add_block('simulink/Math Operations/Sum', [subsysPath '/Sum_Din']);
set_param([subsysPath '/Sum_Din'], 'Position', [320, 80, 340, 140]);
set_param([subsysPath '/Sum_Din'], 'Inputs', '++--');

add_block('simulink/Math Operations/Gain', [subsysPath '/Gain_J']);
set_param([subsysPath '/Gain_J'], 'Position', [380, 100, 420, 130]);
set_param([subsysPath '/Gain_J'], 'Gain', num2str(1/J));

add_block('simulink/Continuous/Integrator', [subsysPath '/Int_omega']);
set_param([subsysPath '/Int_omega'], 'Position', [460, 100, 490, 130]);
set_param([subsysPath '/Int_omega'], 'InitialCondition', '0');

add_block('simulink/Math Operations/Gain', [subsysPath '/Bv']);
set_param([subsysPath '/Bv'], 'Position', [380, 30, 420, 60]);
set_param([subsysPath '/Bv'], 'Gain', num2str(Bv));

add_block('simulink/Continuous/Integrator', [subsysPath '/Int_theta']);
set_param([subsysPath '/Int_theta'], 'Position', [540, 100, 570, 130]);
set_param([subsysPath '/Int_theta'], 'InitialCondition', num2str(th0));

add_block('simulink/Sinks/Out1', [subsysPath '/Theta_out']);
set_param([subsysPath '/Theta_out'], 'Position', [620, 85, 650, 105], 'Port', '1');

add_block('simulink/Sinks/Out1', [subsysPath '/Omega_out']);
set_param([subsysPath '/Omega_out'], 'Position', [620, 135, 650, 155], 'Port', '2');

% Conexiones subsistema planta
add_line(subsysPath, 'Torque_in/1', 'Sum_Din/1');
add_line(subsysPath, 'Sum_Din/1', 'Gain_J/1');
add_line(subsysPath, 'Gain_J/1', 'Int_omega/1');
add_line(subsysPath, 'Int_omega/1', 'Int_theta/1');
add_line(subsysPath, 'Int_theta/1', 'Theta_out/1');
add_line(subsysPath, 'Int_omega/1', 'Omega_out/1');
add_line(subsysPath, 'Int_theta/1', 'MgL_sin/1');
add_line(subsysPath, 'MgL_sin/1', 'Sum_Din/4');
add_line(subsysPath, 'Int_omega/1', 'Bv/1', 'autorouting', 'on');
add_line(subsysPath, 'Bv/1', 'Sum_Din/3', 'autorouting', 'on');

% 8. Scopes
add_block('simulink/Sinks/Scope', [modelName '/Scope_Salida']);
set_param([modelName '/Scope_Salida'], 'Position', pos_scope1);

add_block('simulink/Sinks/Scope', [modelName '/Scope_Control']);
set_param([modelName '/Scope_Control'], 'Position', pos_scope2);

% Scope de control (Goto/From)
add_block('simulink/Signal Routing/Goto', [modelName '/Tap_Control'], ... 
    'GotoTag', 'u_control', 'Position', [390, 160, 430, 180]);
add_block('simulink/Signal Routing/From', [modelName '/From_Control'], ... 
    'GotoTag', 'u_control', 'Position', [780, 210, 820, 230]);

%% Conexiones principales
add_line(modelName, 'Referencia/1', 'SMC_Controller/1');
add_line(modelName, 'SMC_Controller/1', 'Saturacion/1');
add_line(modelName, 'Saturacion/1', 'Ku/1');
add_line(modelName, 'Saturacion/1', 'Tap_Control/1', 'autorouting', 'on');
add_line(modelName, 'Ku/1', 'Sum_Perturbacion/1');
add_line(modelName, 'Perturbacion/1', 'Sum_Perturbacion/2');
add_line(modelName, 'Sum_Perturbacion/1', 'Planta_NoLineal/1');
add_line(modelName, 'Planta_NoLineal/1', 'Scope_Salida/1');
add_line(modelName, 'Planta_NoLineal/1', 'SMC_Controller/2', 'autorouting', 'on');
add_line(modelName, 'Planta_NoLineal/2', 'SMC_Controller/3', 'autorouting', 'on');
add_line(modelName, 'From_Control/1', 'Scope_Control/1');

%% Configuración
set_param(modelName, 'Solver', 'ode45');
set_param(modelName, 'StopTime', '12');
set_param(modelName, 'SaveState', 'on');
set_param(modelName, 'SaveOutput', 'on');

save_system(modelName);

% Limpiar archivo temporal
if exist(smcFileName, 'file')
    delete(smcFileName);
end

disp('===========================================');
disp(['Modelo "' modelName '" creado exitosamente!']);
disp('===========================================');
disp(' ');
disp('Parámetros SMC:');
fprintf('  lambda = %.2f rad/s\n', lambda);
fprintf('  k_torque = %.4f N·m\n', k_torque);
fprintf('  phi = %.2f rad/s\n', phi);
disp(' ');
disp('Para simular ejecuta: ');
disp(['  sim(''' modelName ''')']);