function build_brazo_step_perturb_sim()
% Construye el modelo Simulink:
%   Brazo_Step_Perturb_All.slx
% con una planta no lineal + perturbación de torque
% y tres controladores en paralelo: PID / Hinf / SMC.

mdl = 'Brazo_Step_Perturb_All';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
open_system(mdl);

% Parámetros generales de dibujo
x0 = 30;  dx = 250;  y0 = 80;  dy = 180;

%% Bloques de referencia y perturbación
add_block('simulink/Sources/Step', [mdl '/StepRef'], ...
    'Time','0','Before','0','After','deg2rad(50)', ...
    'Position',[x0 y0 x0+40 y0+40]);

add_block('simulink/Sources/Step', [mdl '/StepDist'], ...
    'Time','3','Before','0','After','0.001', ...
    'Position',[x0 y0+dy x0+40 y0+dy+40]);

%% Planta no lineal en un Subsystem: PlantNL
plant = [mdl '/PlantNL'];
add_block('simulink/Ports & Subsystems/Subsystem', plant, ...
    'Position',[x0+dx y0 x0+dx+220 y0+220]);

% Crear puertos de la planta: u (control) y d (perturbación)
open_system(plant);
add_block('simulink/Sources/In1', [plant '/u'], ...
    'Position',[40 60 70 80]);
add_block('simulink/Sources/In1', [plant '/d'], ...
    'Position',[40 120 70 140],'Port','2');
add_block('simulink/Sinks/Out1', [plant '/theta'], ...
    'Position',[370 90 400 110]);

% Integradores: theta_dd -> theta_dot -> theta
add_block('simulink/Continuous/Integrator', [plant '/Int1_theta_dot'], ...
    'Position',[210 60 240 90]);
add_block('simulink/Continuous/Integrator', [plant '/Int2_theta'], ...
    'Position',[290 60 320 90]);

% Bloques auxiliares
add_block('simulink/Math Operations/Gain', [plant '/KuGain'], ...
    'Gain','Ku','Position',[110 50 150 80]);
add_block('simulink/Math Operations/Gain', [plant '/BvGain'], ...
    'Gain','Bv','Position',[110 120 150 150]);
add_block('simulink/Math Operations/Gain', [plant '/MgLGain'], ...
    'Gain','MgL','Position',[110 180 150 210]);
add_block('simulink/Math Operations/Trigonometric Function', ...
    [plant '/SinTheta'], 'Operator','sin', ...
    'Position',[210 180 240 210]);
add_block('simulink/Math Operations/Gain', [plant '/InvJ'], ...
    'Gain','1/J','Position',[170 230 210 260]);
add_block('simulink/Math Operations/Sum', [plant '/SumTorque'], ...
    'Inputs','+++','Position',[170 110 200 170]);
add_block('simulink/Math Operations/Sum', [plant '/SumDin'], ...
    'Inputs','++-','Position',[240 120 270 180]);

% Conexiones dentro de la planta
% u -> Ku -> SumTorque(+)
add_line(plant,'u/1','KuGain/1');
add_line(plant,'KuGain/1','SumTorque/1');
% d -> SumTorque(+)
add_line(plant,'d/1','SumTorque/2');
% theta_dot -> BvGain -> SumDin(-)
add_line(plant,'Int1_theta_dot/1','BvGain/1');
add_line(plant,'BvGain/1','SumDin/3');
% theta -> Sin -> MgL -> SumDin(+)
add_line(plant,'Int2_theta/1','SinTheta/1');
add_line(plant,'SinTheta/1','MgLGain/1');
add_line(plant,'MgLGain/1','SumDin/1');
% SumTorque -> SumDin(+)
add_line(plant,'SumTorque/1','SumDin/2');
% SumDin -> InvJ -> Int1(theta_dot)
add_line(plant,'SumDin/1','InvJ/1');
add_line(plant,'InvJ/1','Int1_theta_dot/1');
% Int1 -> Int2 -> theta
add_line(plant,'Int1_theta_dot/1','Int2_theta/1');
add_line(plant,'Int2_theta/1','theta/1');

close_system(plant);

%% Nodo de error
add_block('simulink/Math Operations/Sum', [mdl '/SumError'], ...
    'Inputs','+-', ...
    'Position',[x0+dx+260 y0+40 x0+dx+290 y0+80]);

% Conectar StepRef -> SumError (+)
add_line(mdl,'StepRef/1','SumError/1');

%% Señal de perturbación al plant
add_line(mdl,'StepDist/1','PlantNL/2');

%% Tres ramas de control: PID, HINF, SMC

% 1) PID: bloque Discrete PID Controller
add_block('simulink/Discrete/Discrete PID Controller', ...
    [mdl '/PID_ctrl'], ...
    'P','Kp','I','Ki','D','Kd', ...
    'SampleTime','Ts', ...
    'Position',[x0+dx+320 y0 x0+dx+380 y0+60]);
%% Parámetros mínimos para el controlador H-inf (lead–lag discreto)
% Si Khd no existe en el workspace, lo calculamos aquí

    s  = tf('s');
    Ts = 0.01;      % mismo Ts que usas en el diseño

    % Lead–lag de lazo H-inf "simple" (loop shaping)
    k  = 1.5;
    wz = 2.0;
    wp = 8.0;
    Kh_s = k * (1 + s/wz) / (1 + s/wp);

    Khd = c2d(Kh_s, Ts, 'tustin');


% 2) HINF: Discrete Transfer Fcn (usa Khd del workspace)
[numH, denH] = tfdata(Khd,'v');
add_block('simulink/Discrete/Discrete Transfer Fcn', ...
    [mdl '/Hinf_ctrl'], ...
    'Numerator',mat2str(numH), ...
    'Denominator',mat2str(denH), ...
    'SampleTime','Ts', ...
    'Position',[x0+dx+320 y0+80 x0+dx+380 y0+140]);

% 3) SMC: MATLAB Function con la ley discreta
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/SMC_ctrl'], ...
    'Position',[x0+dx+320 y0+160 x0+dx+420 y0+220]);

% Configurar el número de puertos de entrada (4: e, omega, theta, ref)
set_param([mdl '/SMC_ctrl'], 'Inputs', '4');

% Configurar el código de la función (parámetro correcto: 'MATLABCode')
set_param([mdl '/SMC_ctrl'], 'MATLABCode', ...
    ['function u = SMC_ctrl(e, omega, theta, ref)', newline, ...
     '%#codegen', newline, ...
     'lambda = 3.0; k_torque = 0.01; phi = 0.5;', newline, ...
     's = omega + lambda*e;', newline, ...
     'persistent ref_prev;', newline, ...
     'if isempty(ref_prev)', newline, ...
     '    ref_prev = ref;', newline, ...
     'end', newline, ...
     'Ts = evalin(''base'', ''Ts'');', newline, ...
     'ref_dot = (ref - ref_prev)/Ts;', newline, ...
     'ref_prev = ref;', newline, ...
     'J = evalin(''base'', ''J'');', newline, ...
     'Bv = evalin(''base'', ''Bv'');', newline, ...
     'MgL = evalin(''base'', ''MgL'');', newline, ...
     'Ku  = evalin(''base'', ''Ku'');', newline, ...
     'u_eq = (Bv*omega + MgL*sin(theta) - J*lambda*(ref_dot - omega))/Ku;', newline, ...
     'u_sw = -(k_torque/Ku)*tanh(s/phi);', newline, ...
     'u = max(-1, min(1, u_eq + u_sw));', newline]);
%% Selección de controlador (Switches) y planta

% Multiplexores para mandar la misma planta a todos
add_block('simulink/Sinks/Scope',[mdl '/Scope_theta'],...
    'Position',[x0+dx+620 y0 x0+dx+650 y0+120]);
add_block('simulink/Sinks/Scope',[mdl '/Scope_u_PID'],...
    'Position',[x0+dx+620 y0+140 x0+dx+650 y0+260]);
add_block('simulink/Sinks/Scope',[mdl '/Scope_u_HINF'],...
    'Position',[x0+dx+620 y0+280 x0+dx+650 y0+400]);
add_block('simulink/Sinks/Scope',[mdl '/Scope_u_SMC'],...
    'Position',[x0+dx+620 y0+420 x0+dx+650 y0+540]);

% Conexiones principales:
% error e = ref - theta
add_line(mdl,'PlantNL/1','Scope_theta/1');
add_line(mdl,'PlantNL/1','SumError/2');

% Señales de error a PID / Hinf / SMC
add_line(mdl,'SumError/1','PID_ctrl/1');
add_line(mdl,'SumError/1','Hinf_ctrl/1');

% Para el SMC necesito e, omega, theta, ref:
% saco omega desde dentro de PlantNL usando un Outport adicional si quieres.
% (para simplificar aquí asumimos omega ~ d(theta)/dt usando Derivative)
add_block('simulink/Continuous/Derivative',[mdl '/DerivTheta'], ...
    'Position',[x0+dx+460 y0+10 x0+dx+490 y0+40]);
add_line(mdl,'PlantNL/1','DerivTheta/1');

add_block('simulink/Math Operations/Gain',[mdl '/GainOmega'], ...
    'Gain','1','Position',[x0+dx+520 y0+10 x0+dx+550 y0+40]);
add_line(mdl,'DerivTheta/1','GainOmega/1');

add_block('simulink/Signal Routing/Bus Creator',[mdl '/BusSMC'], ...
    'Inputs','4','Position',[x0+dx+460 y0+80 x0+dx+500 y0+140]);
add_line(mdl,'SumError/1','BusSMC/1');
add_line(mdl,'GainOmega/1','BusSMC/2');
add_line(mdl,'PlantNL/1','BusSMC/3');
add_line(mdl,'StepRef/1','BusSMC/4');

add_block('simulink/Signal Routing/Bus Selector',[mdl '/BusSelSMC'], ...
    'OutputSignals','1,2,3,4', ...
    'Position',[x0+dx+520 y0+80 x0+dx+560 y0+140]);
add_line(mdl,'BusSMC/1','BusSelSMC/1');

add_line(mdl,'BusSelSMC/1','SMC_ctrl/1'); % e
add_line(mdl,'BusSelSMC/2','SMC_ctrl/2'); % omega
add_line(mdl,'BusSelSMC/3','SMC_ctrl/3'); % theta
add_line(mdl,'BusSelSMC/4','SMC_ctrl/4'); % ref

% Salidas de control a la planta (usa PID por defecto, puedes crear switches)
add_block('simulink/Signal Routing/Manual Switch',[mdl '/Sw_u'], ...
    'Position',[x0+dx+440 y0+200 x0+dx+470 y0+230]);
add_line(mdl,'PID_ctrl/1','Sw_u/1');
add_line(mdl,'Hinf_ctrl/1','Sw_u/2');
add_line(mdl,'Sw_u/1','PlantNL/1');
add_line(mdl,'Sw_u/1','Scope_u_PID/1');
add_line(mdl,'Hinf_ctrl/1','Scope_u_HINF/1');
add_line(mdl,'SMC_ctrl/1','Scope_u_SMC/1');

% Guardar modelo
save_system(mdl,[mdl '.slx']);
disp(['Modelo Simulink creado: ' mdl '.slx']);
end
