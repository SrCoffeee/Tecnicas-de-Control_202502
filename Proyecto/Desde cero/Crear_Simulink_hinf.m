%% Script para crear modelo Simulink:  Controlador H-infinity con Perturbaciones
% Ejecutar este script para generar el modelo automáticamente

clear; clc;
s = tf('s');

% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

a0  = (MgL*cos(th0))/J;
a1  = Bv/J;
b0  = Ku/J;

% Controlador Hinf (lead-lag simple)
k  = 1.5; 
wz = 2.0; 
wp = 8.0;
Kh_s = k * (1 + s/wz) / (1 + s/wp);
Ts   = 0.01;
Khd  = c2d(Kh_s, Ts, 'tustin');

% Intentar usar hinfsyn si está disponible
try
    G = tf(b0, [1 a1 a0]);
    W1 = makeweight(1/2, 2.0, 0.1);
    W2 = tf(0.2);
    P  = augw(G, W1, W2, []);
    [Khinf_s,~,gam] = hinfsyn(P,1,1);
    Khinf_d = c2d(Khinf_s, Ts, 'tustin');
    disp(['Usando hinfsyn con gamma = ' num2str(gam)]);
    ctrl_tf = Khinf_d;
catch
    disp('Robust Control Toolbox no disponible:  usando lead-lag discretizado.');
    ctrl_tf = Khd;
end

[num_h, den_h] = tfdata(ctrl_tf, 'v');

% Crear nuevo modelo
modelName = 'Control_Hinf_con_Perturbaciones';
try
    close_system(modelName, 0);
catch
end
new_system(modelName);
open_system(modelName);

% Posiciones
pos_step = [30, 100, 60, 130];
pos_sum1 = [120, 105, 140, 125];
pos_hinf = [200, 80, 280, 150];
pos_sum2 = [350, 105, 370, 125];
pos_sat = [420, 100, 460, 130];
pos_gain_ku = [520, 100, 560, 130];
pos_sum3 = [620, 105, 640, 125];
pos_plant = [700, 60, 800, 180];
pos_scope1 = [880, 95, 920, 135];
pos_scope2 = [880, 205, 920, 245];
pos_pert = [520, 200, 580, 230];

%% Agregar bloques

% 1. Escalón de referencia
add_block('simulink/Sources/Step', [modelName '/Referencia']);
set_param([modelName '/Referencia'], 'Position', pos_step);
set_param([modelName '/Referencia'], 'Time', '0');
set_param([modelName '/Referencia'], 'After', num2str(th0));

% 2. Sumador de error
add_block('simulink/Math Operations/Sum', [modelName '/Error']);
set_param([modelName '/Error'], 'Position', pos_sum1);
set_param([modelName '/Error'], 'Inputs', '+-');

% 3. Controlador H-infinity (Discrete Transfer Fcn)
add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Hinf_Controller']);
set_param([modelName '/Hinf_Controller'], 'Position', pos_hinf);
set_param([modelName '/Hinf_Controller'], 'Numerator', mat2str(num_h));
set_param([modelName '/Hinf_Controller'], 'Denominator', mat2str(den_h));
set_param([modelName '/Hinf_Controller'], 'SampleTime', num2str(Ts));

% 4. Sumador pre-saturación
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Control']);
set_param([modelName '/Sum_Control'], 'Position', pos_sum2);
set_param([modelName '/Sum_Control'], 'Inputs', '++');

% 5. Saturación
add_block('simulink/Discontinuities/Saturation', [modelName '/Saturacion']);
set_param([modelName '/Saturacion'], 'Position', pos_sat);
set_param([modelName '/Saturacion'], 'UpperLimit', '1');
set_param([modelName '/Saturacion'], 'LowerLimit', '-1');

% 6. Ganancia Ku
add_block('simulink/Math Operations/Gain', [modelName '/Ku']);
set_param([modelName '/Ku'], 'Position', pos_gain_ku);
set_param([modelName '/Ku'], 'Gain', num2str(Ku));

% 7. Sumador para perturbación
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Perturbacion']);
set_param([modelName '/Sum_Perturbacion'], 'Position', pos_sum3);
set_param([modelName '/Sum_Perturbacion'], 'Inputs', '++');

% 8. Perturbación (pulsos)
add_block('simulink/Sources/Pulse Generator', [modelName '/Perturbacion']);
set_param([modelName '/Perturbacion'], 'Position', pos_pert);
set_param([modelName '/Perturbacion'], 'Amplitude', '0.002');
set_param([modelName '/Perturbacion'], 'Period', '3');
set_param([modelName '/Perturbacion'], 'PulseWidth', '20');

% 9. Planta no lineal
add_block('built-in/Subsystem', [modelName '/Planta_NoLineal']);
set_param([modelName '/Planta_NoLineal'], 'Position', pos_plant);

% Limpiar el subsistema
subsysPath = [modelName '/Planta_NoLineal'];
lines = find_system(subsysPath, 'FindAll', 'on', 'Type', 'line');
for i = 1:length(lines)
    try
        delete_line(lines(i));
    catch
    end
end
blocks = find_system(subsysPath, 'SearchDepth', 1, 'Type', 'block');
for i = 1:length(blocks)
    if ~strcmp(blocks{i}, subsysPath)
        try
            delete_block(blocks{i});
        catch
        end
    end
end

% Contenido del subsistema
add_block('simulink/Sources/In1', [subsysPath '/Torque_in']);
set_param([subsysPath '/Torque_in'], 'Position', [20, 50, 50, 70]);

add_block('simulink/User-Defined Functions/Fcn', [subsysPath '/MgL_sin']);
set_param([subsysPath '/MgL_sin'], 'Position', [150, 150, 250, 180]);
set_param([subsysPath '/MgL_sin'], 'Expr', sprintf('%. 10e*sin(u(1))', MgL));

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
set_param([subsysPath '/Theta_out'], 'Position', [620, 105, 650, 125]);

% Conexiones subsistema
add_line(subsysPath, 'Torque_in/1', 'Sum_Din/1');
add_line(subsysPath, 'Sum_Din/1', 'Gain_J/1');
add_line(subsysPath, 'Gain_J/1', 'Int_omega/1');
add_line(subsysPath, 'Int_omega/1', 'Int_theta/1');
add_line(subsysPath, 'Int_theta/1', 'Theta_out/1');
add_line(subsysPath, 'Int_theta/1', 'MgL_sin/1');
add_line(subsysPath, 'MgL_sin/1', 'Sum_Din/4');
add_line(subsysPath, 'Int_omega/1', 'Bv/1', 'autorouting', 'on');
add_line(subsysPath, 'Bv/1', 'Sum_Din/3', 'autorouting', 'on');

% 10. Scopes
add_block('simulink/Sinks/Scope', [modelName '/Scope_Salida']);
set_param([modelName '/Scope_Salida'], 'Position', pos_scope1);

add_block('simulink/Sinks/Scope', [modelName '/Scope_Control']);
set_param([modelName '/Scope_Control'], 'Position', pos_scope2);

% 11. Feedforward - USANDO BLOQUE Fcn EN LUGAR DE MATLAB FUNCTION
add_block('simulink/User-Defined Functions/Fcn', [modelName '/Feedforward']);
set_param([modelName '/Feedforward'], 'Position', [200, 190, 280, 220]);
% Expresión: u_ff = (MgL/Ku)*sin(ref)
ff_expr = sprintf('(%. 10e/%.10e)*sin(u(1))', MgL, Ku);
set_param([modelName '/Feedforward'], 'Expr', ff_expr);

%% Conexiones principales
add_line(modelName, 'Referencia/1', 'Error/1');
add_line(modelName, 'Error/1', 'Hinf_Controller/1');
add_line(modelName, 'Hinf_Controller/1', 'Sum_Control/1');
add_line(modelName, 'Sum_Control/1', 'Saturacion/1');
add_line(modelName, 'Saturacion/1', 'Ku/1');
add_line(modelName, 'Ku/1', 'Sum_Perturbacion/1');
add_line(modelName, 'Perturbacion/1', 'Sum_Perturbacion/2');
add_line(modelName, 'Sum_Perturbacion/1', 'Planta_NoLineal/1');
add_line(modelName, 'Planta_NoLineal/1', 'Scope_Salida/1');
add_line(modelName, 'Planta_NoLineal/1', 'Error/2', 'autorouting', 'on');

add_line(modelName, 'Referencia/1', 'Feedforward/1', 'autorouting', 'on');
add_line(modelName, 'Feedforward/1', 'Sum_Control/2', 'autorouting', 'on');

% Scope de control
add_block('simulink/Signal Routing/Goto', [modelName '/Tap_Control'], ... 
    'GotoTag', 'u_control', 'Position', [420, 160, 460, 180]);
add_line(modelName, 'Saturacion/1', 'Tap_Control/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/From', [modelName '/From_Control'], ... 
    'GotoTag', 'u_control', 'Position', [820, 210, 860, 230]);
add_line(modelName, 'From_Control/1', 'Scope_Control/1');

%% Configuración
set_param(modelName, 'Solver', 'ode45');
set_param(modelName, 'StopTime', '12');
set_param(modelName, 'SaveState', 'on');
set_param(modelName, 'SaveOutput', 'on');

save_system(modelName);

disp('===========================================');
disp(['Modelo "' modelName '" creado exitosamente!']);
disp('===========================================');
disp(' ');
disp('Controlador H-infinity: ');
disp(['  Numerador:    ' mat2str(num_h)]);
disp(['  Denominador: ' mat2str(den_h)]);
disp(['  Ts = ' num2str(Ts) ' s']);
disp(' ');
disp('Feedforward:');
disp(['  u_ff = (MgL/Ku)*sin(ref) = ' sprintf('%.4f*sin(ref)', MgL/Ku)]);
disp(' ');
disp('Para simular: presiona "Run" en Simulink o ejecuta:');
disp(['  sim(''' modelName ''')']);