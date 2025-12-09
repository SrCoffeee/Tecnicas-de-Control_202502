% build_simulink_model.m — Firma: Nadia
function build_simulink_model()
mdl = 'AS5600_L298N_Control';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

assignin('base','Ts',0.01);
assignin('base','J', 1.08e-3); assignin('base','B', 2.61e-3);
assignin('base','MgL',4.999e-3); assignin('base','Ku',1.13e-2);
assignin('base','tauC',0);
assignin('base','theta_ref_deg',60);
assignin('base','profile','STEP'); % 'STEP'|'RAMP'|'PULSET'
assignin('base','ramp_deg_s',10);
assignin('base','pulse_amp_deg',15);
assignin('base','pulse_T',1.0);
assignin('base','useFF',true);
assignin('base','controller','PID'); % 'PID'|'HINF'|'SMC'
assignin('base','Kp',2.141); assignin('base','Ki',2.290);
assignin('base','Kd',0.685); assignin('base','tau_d',0.05);
A = 0.85; Bm = 1; C = [0.15 -0.14]; D = 0;  % H∞ discreto simple
assignin('base','HinfA',A); assignin('base','HinfB',Bm);
assignin('base','HinfC',C); assignin('base','HinfD',D);

add_block('simulink/Sources/Clock',[mdl '/Clock'],'Position',[20 20 50 40]);
add_block('simulink/User-Defined Functions/MATLAB Function',[mdl '/RefGen'],...
    'Position',[90 20 260 100]);
set_param([mdl '/RefGen'],'Script',sprintf([ ...
    'function ref = f(t)\n' ...
    'switch profile\n' ...
    ' case ''STEP'', ref = deg2rad(theta_ref_deg);\n' ...
    ' case ''RAMP'', ref = deg2rad(ramp_deg_s)*t;\n' ...
    ' case ''PULSET''\n' ...
    '  A = deg2rad(pulse_amp_deg); phase = mod(t,pulse_T);\n' ...
    '  if phase < pulse_T/2, ref =  A; else, ref = -A; end\n' ...
    ' otherwise, ref = 0; end\n' ...
    'end' ]));
add_block('simulink/Continuous/Transfer Fcn',[mdl '/PlantLIN'],...
    'Numerator','[Ku/J]','Denominator','[1 (B/J) (MgL/J)]','Position',[540 250 770 290]);
add_block('simulink/User-Defined Functions/MATLAB Function',[mdl '/PlantNL'],...
    'Position',[540 110 770 210]);
set_param([mdl '/PlantNL'],'Script',sprintf([ ...
    'function [theta,omega] = f(u)\n' ...
    'persistent x\n' ...
    'if isempty(x), x = [0;0]; end\n' ...
    'theta = x(1); omega = x(2);\n' ...
    'torque = Ku*u - B*omega - MgL*sin(theta);\n' ...
    'alpha  = torque / J;\n' ...
    'x = x + Ts * [omega; alpha];\n' ...
    'theta = x(1); omega = x(2);\n' ...
    'end' ]));
add_block('simulink/Signal Routing/Manual Switch',[mdl '/PlantSel'],'Position',[790 170 820 210]);

add_block('simulink/Continuous/PID Controller',[mdl '/PID'],...
    'P','Kp','I','Ki','D','Kd','N','1/tau_d','Position',[300 120 420 180]);
add_block('simulink/Discrete/Discrete State-Space',[mdl '/HinfSS'],...
    'A','HinfA','B','HinfB','C','HinfC','D','HinfD','SampleTime','Ts', ...
    'Position',[300 190 420 230]);
add_block('simulink/User-Defined Functions/MATLAB Function',[mdl '/SMC'],...
    'Position',[300 250 420 310]);
set_param([mdl '/SMC'],'Script',sprintf([ ...
    'function u = f(e,omega,theta)\n' ...
    'lambda = 3.6; kDuty = 0.27; phi = 0.02;\n' ...
    's = omega + lambda*e;\n' ...
    'u_eq = (J*(-lambda*omega) + B*omega + MgL*sin(theta))/Ku;\n' ...
    'u = u_eq - kDuty*max(-1,min(1,s/phi));\n' ...
    'end' ]));

add_block('simulink/Math Operations/Sum',[mdl '/Sum_e'],'Inputs','+-','Position',[260 130 280 170]);
add_block('simulink/Math Operations/Sum',[mdl '/Sum_u'],'Inputs','++','Position',[450 160 480 200]);
add_block('simulink/User-Defined Functions/MATLAB Function',[mdl '/FF'],...
    'Position',[300 90 420 120]);
set_param([mdl '/FF'],'Script','function uff = f(ref)\nif useFF, uff = (MgL*sin(ref))/Ku; else, uff=0; end');

add_block('simulink/Discontinuities/Saturation',[mdl '/Sat'], ...
    'UpperLimit','1','LowerLimit','-1','Position',[500 160 530 200]);

add_block('simulink/Sinks/To Workspace',[mdl '/toTheta'],'VariableName','theta_log','SaveFormat','Structure With Time','Position',[860 110 930 130]);
add_block('simulink/Sinks/To Workspace',[mdl '/toRef'],'VariableName','ref_log','SaveFormat','Structure With Time','Position',[860 50 930 70]);
add_block('simulink/Sinks/To Workspace',[mdl '/toU'],'VariableName','u_log','SaveFormat','Structure With Time','Position',[860 200 930 220]);

add_block('simulink/Signal Routing/Manual Switch',[mdl '/CtrlSel'],'Position',[430 120 460 260]);

% Conexiones principales
add_line(mdl,'Clock/1','RefGen/1');
add_line(mdl,'RefGen/1','toRef/1');
add_line(mdl,'RefGen/1','Sum_e/1');

% Derivada (velocidad aprox) para SMC
add_block('simulink/Discrete/Discrete Derivative',[mdl '/dtheta'],...
    'SampleTime','Ts','Gain','1','ICPrevOutput','0','Position',[820 140 850 160]);

% Selector de planta y medición
add_line(mdl,'Sat/1','PlantLIN/1');
add_line(mdl,'Sat/1','PlantNL/1');
add_line(mdl,'PlantLIN/1','PlantSel/1');
add_line(mdl,'PlantNL/1','PlantSel/2');

add_line(mdl,'PlantSel/1','dtheta/1');
add_line(mdl,'PlantSel/1','toTheta/1');
add_line(mdl,'PlantSel/1','Sum_e/2','autorouting','on');

% Controladores
add_line(mdl,'Sum_e/1','PID/1');
add_line(mdl,'Sum_e/1','HinfSS/1');

% SMC necesita e, omega y theta
add_block('simulink/Signal Routing/Bus Creator',[mdl '/busSMC'],'Inputs','3','Position',[250 240 270 300]);
add_line(mdl,'Sum_e/1','busSMC/1');
add_line(mdl,'dtheta/1','busSMC/2');
add_line(mdl,'PlantSel/1','busSMC/3');
add_line(mdl,'busSMC/1','SMC/1');
add_line(mdl,'busSMC/2','SMC/2');
add_line(mdl,'busSMC/3','SMC/3');

% FF + controlador seleccionado + saturación
add_line(mdl,'FF/1','Sum_u/1');
add_line(mdl,'PID/1','CtrlSel/1');
add_line(mdl,'HinfSS/1','CtrlSel/2');
add_line(mdl,'SMC/1','CtrlSel/3');
add_line(mdl,'CtrlSel/1','Sum_u/2');
add_line(mdl,'Sum_u/1','Sat/1');
add_line(mdl,'Sat/1','toU/1');

save_system(mdl);
disp('Modelo AS5600_L298N_Control creado.');
end
