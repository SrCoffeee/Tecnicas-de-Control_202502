%% Script CORREGIDO para crear modelo Simulink:  Controlador PID con Perturbaciones
% Ejecutar este script para generar el modelo automáticamente

clear; clc;

% Parámetros de la planta
J   = 1.08e-3;      % kg m^2
Bv  = 2.61e-3;      % N m s/rad
MgL = 4.999e-3;     % N m
Ku  = 1.13e-2;      % N m / duty
th0 = deg2rad(50);  % punto de operación

% Parámetros PID (del código original)
zeta = 0.9; wn = 2.2; p3 = 4*wn;
a0  = (MgL*cos(th0))/J;
a1  = Bv/J;
b0  = Ku/J;
a2d = 2*zeta*wn + p3;
a1d = wn^2 + 2*zeta*wn*p3;
a0d = (wn^2)*p3;

Kp    = 5.088;
Kd    = 0.486;
Ki    =7.0707;
tau_d = 0.05; 
Tt    = 0.35;

% Crear nuevo modelo
modelName = 'Control_PID_con_Perturbaciones';
try
    close_system(modelName, 0);
catch
end
new_system(modelName);
open_system(modelName);

% Posiciones para organizar bloques
pos_step = [30, 100, 60, 130];
pos_sum1 = [120, 105, 140, 125];
pos_pid = [200, 80, 280, 150];
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
set_param([modelName '/Referencia'], 'SampleTime', '0');

% 2. Sumador de error (Ref - Salida)
add_block('simulink/Math Operations/Sum', [modelName '/Error']);
set_param([modelName '/Error'], 'Position', pos_sum1);
set_param([modelName '/Error'], 'Inputs', '+-');

% 3. Controlador PID
add_block('simulink/Continuous/PID Controller', [modelName '/PID']);
set_param([modelName '/PID'], 'Position', pos_pid);
set_param([modelName '/PID'], 'Controller', 'PID');
set_param([modelName '/PID'], 'P', num2str(Kp));
set_param([modelName '/PID'], 'I', num2str(Ki));
set_param([modelName '/PID'], 'D', num2str(Kd));
set_param([modelName '/PID'], 'N', num2str(1/tau_d));
set_param([modelName '/PID'], 'Kb', num2str(1/Tt));

% 4. Sumador pre-saturación (Control + Feedforward)
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Control']);
set_param([modelName '/Sum_Control'], 'Position', pos_sum2);
set_param([modelName '/Sum_Control'], 'Inputs', '++');

% 5. Saturación [-1, 1]
add_block('simulink/Discontinuities/Saturation', [modelName '/Saturacion']);
set_param([modelName '/Saturacion'], 'Position', pos_sat);
set_param([modelName '/Saturacion'], 'UpperLimit', '1');
set_param([modelName '/Saturacion'], 'LowerLimit', '-1');

% 6. Ganancia Ku
add_block('simulink/Math Operations/Gain', [modelName '/Ku']);
set_param([modelName '/Ku'], 'Position', pos_gain_ku);
set_param([modelName '/Ku'], 'Gain', num2str(Ku));

% 7. Sumador para perturbación (Torque + Perturbación)
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Perturbacion']);
set_param([modelName '/Sum_Perturbacion'], 'Position', pos_sum3);
set_param([modelName '/Sum_Perturbacion'], 'Inputs', '++');

% 8. Perturbación (pulsos)
add_block('simulink/Sources/Pulse Generator', [modelName '/Perturbacion']);
set_param([modelName '/Perturbacion'], 'Position', pos_pert);
set_param([modelName '/Perturbacion'], 'Amplitude', '0.002');
set_param([modelName '/Perturbacion'], 'Period', '3');
set_param([modelName '/Perturbacion'], 'PulseWidth', '20');

% 9. Planta no lineal (subsistema)
subsys_path = [modelName '/Planta_NoLineal'];
add_block('built-in/Subsystem', subsys_path);
set_param(subsys_path, 'Position', pos_plant);

% Limpiar subsistema de forma segura
lines = find_system(subsys_path, 'FindAll', 'on', 'Type', 'line');
for i = 1:length(lines)
    try
        delete_line(lines(i));
    catch
    end
end
blocks = find_system(subsys_path, 'SearchDepth', 1, 'Type', 'block');
for i = 1:length(blocks)
    if ~strcmp(blocks{i}, subsys_path)
        try
            delete_block(blocks{i});
        catch
        end
    end
end

% Crear contenido del subsistema de la planta
add_block('simulink/Sources/In1', [subsys_path '/Torque_in']);
set_param([subsys_path '/Torque_in'], 'Position', [20, 50, 50, 70]);

% Fcn para MgL*sin(theta) - CORREGIDO:  usar u(1) en lugar de u[1]
add_block('simulink/User-Defined Functions/Fcn', [subsys_path '/MgL_sin']);
set_param([subsys_path '/MgL_sin'], 'Position', [150, 150, 250, 180]);
set_param([subsys_path '/MgL_sin'], 'Expr', sprintf('%g*sin(u(1))', MgL));

% Sumador dinámica
add_block('simulink/Math Operations/Sum', [subsys_path '/Sum_Din']);
set_param([subsys_path '/Sum_Din'], 'Position', [320, 80, 340, 140]);
set_param([subsys_path '/Sum_Din'], 'Inputs', '++--');

% Ganancia 1/J
add_block('simulink/Math Operations/Gain', [subsys_path '/Gain_J']);
set_param([subsys_path '/Gain_J'], 'Position', [380, 100, 420, 130]);
set_param([subsys_path '/Gain_J'], 'Gain', num2str(1/J));

% Integrador para omega
add_block('simulink/Continuous/Integrator', [subsys_path '/Int_omega']);
set_param([subsys_path '/Int_omega'], 'Position', [460, 100, 490, 130]);
set_param([subsys_path '/Int_omega'], 'InitialCondition', '0');

% Ganancia Bv
add_block('simulink/Math Operations/Gain', [subsys_path '/Bv']);
set_param([subsys_path '/Bv'], 'Position', [380, 30, 420, 60]);
set_param([subsys_path '/Bv'], 'Gain', num2str(Bv));

% Integrador para theta
add_block('simulink/Continuous/Integrator', [subsys_path '/Int_theta']);
set_param([subsys_path '/Int_theta'], 'Position', [540, 100, 570, 130]);
set_param([subsys_path '/Int_theta'], 'InitialCondition', num2str(th0));

% Salida
add_block('simulink/Sinks/Out1', [subsys_path '/Theta_out']);
set_param([subsys_path '/Theta_out'], 'Position', [620, 105, 650, 125]);

% Conexiones dentro del subsistema
add_line(subsys_path, 'Torque_in/1', 'Sum_Din/1');
add_line(subsys_path, 'Sum_Din/1', 'Gain_J/1');
add_line(subsys_path, 'Gain_J/1', 'Int_omega/1');
add_line(subsys_path, 'Int_omega/1', 'Int_theta/1');
add_line(subsys_path, 'Int_theta/1', 'Theta_out/1');
add_line(subsys_path, 'Int_theta/1', 'MgL_sin/1');
add_line(subsys_path, 'MgL_sin/1', 'Sum_Din/4');
add_line(subsys_path, 'Int_omega/1', 'Bv/1', 'autorouting', 'on');
add_line(subsys_path, 'Bv/1', 'Sum_Din/3', 'autorouting', 'on');

% 10. Scope para salida
add_block('simulink/Sinks/Scope', [modelName '/Scope_Salida']);
set_param([modelName '/Scope_Salida'], 'Position', pos_scope1);

% 11. Scope para señal de control
add_block('simulink/Sinks/Scope', [modelName '/Scope_Control']);
set_param([modelName '/Scope_Control'], 'Position', pos_scope2);

% 12. Feedforward - USANDO BLOQUE Fcn EN LUGAR DE MATLAB FUNCTION
add_block('simulink/User-Defined Functions/Fcn', [modelName '/Feedforward']);
set_param([modelName '/Feedforward'], 'Position', [200, 190, 280, 220]);
% Expresión:  u_ff = (MgL/Ku)*sin(ref)
ff_value = MgL/Ku;
set_param([modelName '/Feedforward'], 'Expr', sprintf('%g*sin(u(1))', ff_value));

%% Conexiones principales
add_line(modelName, 'Referencia/1', 'Error/1');
add_line(modelName, 'Error/1', 'PID/1');
add_line(modelName, 'PID/1', 'Sum_Control/1');
add_line(modelName, 'Sum_Control/1', 'Saturacion/1');
add_line(modelName, 'Saturacion/1', 'Ku/1');
add_line(modelName, 'Ku/1', 'Sum_Perturbacion/1');
add_line(modelName, 'Perturbacion/1', 'Sum_Perturbacion/2');
add_line(modelName, 'Sum_Perturbacion/1', 'Planta_NoLineal/1');
add_line(modelName, 'Planta_NoLineal/1', 'Scope_Salida/1');
add_line(modelName, 'Planta_NoLineal/1', 'Error/2', 'autorouting', 'on');

% Feedforward
add_line(modelName, 'Referencia/1', 'Feedforward/1', 'autorouting', 'on');
add_line(modelName, 'Feedforward/1', 'Sum_Control/2', 'autorouting', 'on');

% Scope de control
add_block('simulink/Signal Routing/Goto', [modelName '/Tap_Control'], ...
    'GotoTag', 'u_control', 'Position', [420, 160, 460, 180]);
add_line(modelName, 'Saturacion/1', 'Tap_Control/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/From', [modelName '/From_Control'], ... 
    'GotoTag', 'u_control', 'Position', [820, 210, 860, 230]);
add_line(modelName, 'From_Control/1', 'Scope_Control/1');

%% Configuración de simulación
set_param(modelName, 'Solver', 'ode45');
set_param(modelName, 'StopTime', '12');
set_param(modelName, 'SaveState', 'on');
set_param(modelName, 'StateSaveName', 'xout');
set_param(modelName, 'SaveOutput', 'on');
set_param(modelName, 'OutputSaveName', 'yout');

% Guardar modelo
save_system(modelName);

disp('===========================================');
disp(['Modelo "' modelName '" creado exitosamente!']);
disp('===========================================');
disp(' ');
disp('Parámetros PID:');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %. 4f\n', Ki);
fprintf('  Kd = %.4f\n', Kd);
fprintf('  tau_d = %.4f\n', tau_d);
fprintf('  Tt (anti-windup) = %.4f\n', Tt);
disp(' ');
disp('Feedforward:');
fprintf('  u_ff = %.4f * sin(ref)\n', ff_value);
disp(' ');
disp('Para simular: presiona "Run" en Simulink o ejecuta:');
disp(['  sim(''' modelName ''')']);
disp(' ');
disp('La perturbación es un pulso de 2mNm cada 3 segundos.');