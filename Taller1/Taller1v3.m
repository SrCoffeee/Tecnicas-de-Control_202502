%% GENERACIÓN AUTOMÁTICA DE MODELO SIMULINK - H∞ LATERAL-DIRECCIONAL
% Universidad Nacional de Colombia
% Este script crea automáticamente el modelo Simulink para el control H∞

clear; clc;

fprintf('========================================\n');
fprintf('GENERANDO MODELO SIMULINK H∞\n');
fprintf('========================================\n\n');

%% ========================================================================
% PASO 1: CARGAR MODELO Y CONTROLADOR H∞
% ========================================================================

% Verificar si existe el archivo de resultados
if exist('H_inf_lat_results_MEJORADO.mat', 'file')
    fprintf('✓ Cargando resultados guardados...\n');
    load('H_inf_lat_results_MEJORADO.mat');
else
    fprintf('⚠ No se encontró archivo de resultados. Ejecutando diseño...\n');
    
    % Modelo lateral (5 estados, 2 entradas, 6 salidas)
    A_LD = [
        -0.8750   0.8751 -16.8197   9.7914   0;
        -2.8312 -16.1385   3.3768   0        0;
         1.7063   0.5154  -2.7828   0        0;
         0        1.0000   0.0538   0        0;
         0        0        1.0014   0        0
    ];
    
    B_LD = [
         0        5.3170;
      -156.9094  -5.0216;
        11.5366 -82.2714;
         0        0;
         0        0
    ];
    
    C_LD = [
         0        1   0   0   0;
         0        0   1   0   0;
         0        0   0   1   0;
         0        0   0   0   1
    ];
    
    D_LD = zeros(4,2);
    
    % Crear sistema
    G_LD = ss(A_LD, B_LD, C_LD, D_LD);
    
    % Funciones de peso (versión mejorada)
    W1_Lat_vel = 2.5;
    W1_Dir_vel = 2.5;
    
    M1_pos = 1.8;
    A1_pos = 0.00001;
    Wb1_pos = 2*pi*10;
    W1_pos = tf([1/M1_pos Wb1_pos], [1 A1_pos*Wb1_pos]);
    
    W1_LD = blkdiag(W1_Dir_vel, W1_Lat_vel, W1_pos, W1_pos);
    W2_LD = 0.8;
    
    M3_LD = 0.008;
    A3_LD = 1.1;
    Wb3_LD = 2*pi*250;
    W3_LD = tf([1/M3_LD Wb3_LD], [1 A3_LD*Wb3_LD]);
    
    % Síntesis H∞ con ambos métodos
    fprintf('Sintetizando controlador H∞...\n');
    
    P_LD = augw(G_LD, W1_LD, W2_LD, W3_LD);
    
    % Método 1: hinfsyn (preferido para Simulink)
    try
        [K_LD_hinfsyn, ~, gamma_hinfsyn] = hinfsyn(P_LD, 4, 2);
        K_LD = K_LD_hinfsyn;
        gamma_LD = gamma_hinfsyn;
        fprintf('  ✓ hinfsyn exitoso: γ = %.4f\n', gamma_LD);
    catch ME
        warning('hinfsyn falló: %s', ME.message);
        
        % Método 2: mixsyn (backup)
        try
            [K_LD_mixsyn, ~, gamma_mixsyn] = mixsyn(G_LD, W1_LD, W2_LD, W3_LD);
            K_LD = K_LD_mixsyn;
            gamma_LD = gamma_mixsyn;
            fprintf('  ✓ mixsyn exitoso: γ = %.4f\n', gamma_LD);
        catch ME2
            error('Ambos métodos fallaron. Verifica las funciones de peso.');
        end
    end
end

fprintf('✓ Modelo y controlador cargados\n');

% VERIFICAR Y CORREGIR DIMENSIONES DEL CONTROLADOR
fprintf('\nVerificando compatibilidad del controlador...\n');
[A_K, B_K, C_K, D_K] = ssdata(K_LD);

fprintf('  Dimensiones actuales:\n');
fprintf('    Entradas: %d (debe ser 4: error en p, r, φ, ψ)\n', size(B_K,2));
fprintf('    Salidas:  %d (debe ser 2: δa, δr)\n', size(C_K,1));

% Verificar si las dimensiones son correctas
if size(B_K,2) ~= 4
    error(['ERROR: El controlador tiene %d entradas, pero debe tener 4.\n' ...
           'Verifica que el controlador fue diseñado con hinfsyn(P_LD, 4, 2)'], size(B_K,2));
end

if size(C_K,1) ~= 2
    error(['ERROR: El controlador tiene %d salidas, pero debe tener 2.\n' ...
           'Verifica que el controlador fue diseñado con hinfsyn(P_LD, 4, 2)'], size(C_K,1));
end

fprintf('  ✓ Dimensiones correctas: 4 entradas, 2 salidas\n');
fprintf('  ✓ Orden del controlador: %d estados\n\n', size(A_K,1));

%% ========================================================================
% PASO 2: CREAR MODELO SIMULINK
% ========================================================================

model_name = 'UAV_Hinf_Lateral_Control';

% Verificar si el modelo ya existe
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Crear nuevo modelo
fprintf('Creando modelo Simulink: %s\n', model_name);
new_system(model_name);
open_system(model_name);

%% ========================================================================
% PASO 3: CONFIGURAR PARÁMETROS DEL MODELO
% ========================================================================

set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'StopTime', '30');
set_param(model_name, 'SolverType', 'Variable-step');

%% ========================================================================
% PASO 4: AGREGAR BLOQUES - REFERENCIAS
% ========================================================================

fprintf('Agregando bloques de referencia...\n');

% Referencia Roll (φ)
add_block('simulink/Sources/Step', [model_name '/Ref_Roll']);
set_param([model_name '/Ref_Roll'], 'Position', [50, 100, 80, 130]);
set_param([model_name '/Ref_Roll'], 'Time', '0');
set_param([model_name '/Ref_Roll'], 'After', '1');
set_param([model_name '/Ref_Roll'], 'SampleTime', '0');

% Referencia Yaw (ψ)
add_block('simulink/Sources/Step', [model_name '/Ref_Yaw']);
set_param([model_name '/Ref_Yaw'], 'Position', [50, 180, 80, 210]);
set_param([model_name '/Ref_Yaw'], 'Time', '0');
set_param([model_name '/Ref_Yaw'], 'After', '1');

% Referencia Roll Rate (p) - generalmente cero
add_block('simulink/Sources/Constant', [model_name '/Ref_p']);
set_param([model_name '/Ref_p'], 'Position', [50, 20, 80, 50]);
set_param([model_name '/Ref_p'], 'Value', '0');

% Referencia Yaw Rate (r) - generalmente cero
add_block('simulink/Sources/Constant', [model_name '/Ref_r']);
set_param([model_name '/Ref_r'], 'Position', [50, 260, 80, 290]);
set_param([model_name '/Ref_r'], 'Value', '0');

%% ========================================================================
% PASO 5: MUX DE REFERENCIAS
% ========================================================================

add_block('simulink/Signal Routing/Mux', [model_name '/Mux_Ref']);
set_param([model_name '/Mux_Ref'], 'Position', [130, 89, 135, 211]);
set_param([model_name '/Mux_Ref'], 'Inputs', '4');
set_param([model_name '/Mux_Ref'], 'DisplayOption', 'bar');

%% ========================================================================
% PASO 6: SUMADOR (Error = Referencia - Salida)
% ========================================================================

add_block('simulink/Math Operations/Sum', [model_name '/Error']);
set_param([model_name '/Error'], 'Position', [200, 120, 220, 180]);
set_param([model_name '/Error'], 'Inputs', '+-');
set_param([model_name '/Error'], 'IconShape', 'round');

%% ========================================================================
% PASO 7: CONTROLADOR H∞
% ========================================================================

fprintf('Agregando controlador H∞...\n');

add_block('simulink/Continuous/State-Space', [model_name '/Controller_Hinf']);
set_param([model_name '/Controller_Hinf'], 'Position', [280, 125, 380, 175]);

% Convertir controlador a state-space y verificar dimensiones
[A_K, B_K, C_K, D_K] = ssdata(K_LD);

fprintf('  Dimensiones del controlador:\n');
fprintf('    A_K: %dx%d\n', size(A_K,1), size(A_K,2));
fprintf('    B_K: %dx%d (entradas)\n', size(B_K,1), size(B_K,2));
fprintf('    C_K: %dx%d (salidas)\n', size(C_K,1), size(C_K,2));
fprintf('    D_K: %dx%d\n', size(D_K,1), size(D_K,2));

% Verificar compatibilidad
if size(B_K,2) ~= 4 || size(C_K,1) ~= 2
    warning('Dimensiones del controlador no estándar. Ajustando...');
    % El controlador debe tener 4 entradas (error) y 2 salidas (control)
    % Si no coincide, se usará el controlador completo
end

% Configurar el bloque con formato de cadena para matrices grandes
set_param([model_name '/Controller_Hinf'], 'A', mat2str(A_K, 10));
set_param([model_name '/Controller_Hinf'], 'B', mat2str(B_K, 10));
set_param([model_name '/Controller_Hinf'], 'C', mat2str(C_K, 10));
set_param([model_name '/Controller_Hinf'], 'D', mat2str(D_K, 10));

%% ========================================================================
% PASO 8: PLANTA (SISTEMA LATERAL-DIRECCIONAL)
% ========================================================================

fprintf('Agregando planta UAV...\n');

add_block('simulink/Continuous/State-Space', [model_name '/UAV_Plant']);
set_param([model_name '/UAV_Plant'], 'Position', [480, 125, 580, 175]);

set_param([model_name '/UAV_Plant'], 'A', mat2str(A_LD));
set_param([model_name '/UAV_Plant'], 'B', mat2str(B_LD));
set_param([model_name '/UAV_Plant'], 'C', mat2str(C_LD));
set_param([model_name '/UAV_Plant'], 'D', mat2str(D_LD));

%% ========================================================================
% PASO 9: DEMUX DE SALIDAS
% ========================================================================

add_block('simulink/Signal Routing/Demux', [model_name '/Demux_Output']);
set_param([model_name '/Demux_Output'], 'Position', [650, 117, 655, 183]);
set_param([model_name '/Demux_Output'], 'Outputs', '4');
set_param([model_name '/Demux_Output'], 'DisplayOption', 'bar');

%% ========================================================================
% PASO 10: SCOPES Y VISUALIZACIÓN
% ========================================================================

fprintf('Agregando scopes de visualización...\n');

% Scope Roll (φ)
add_block('simulink/Sinks/Scope', [model_name '/Scope_Roll']);
set_param([model_name '/Scope_Roll'], 'Position', [750, 95, 780, 125]);
set_param([model_name '/Scope_Roll'], 'NumInputPorts', '2');

% Scope Yaw (ψ)
add_block('simulink/Sinks/Scope', [model_name '/Scope_Yaw']);
set_param([model_name '/Scope_Yaw'], 'Position', [750, 165, 780, 195]);
set_param([model_name '/Scope_Yaw'], 'NumInputPorts', '2');

% Scope Control Signals
add_block('simulink/Sinks/Scope', [model_name '/Scope_Control']);
set_param([model_name '/Scope_Control'], 'Position', [450, 70, 480, 100]);
set_param([model_name '/Scope_Control'], 'NumInputPorts', '1');

%% ========================================================================
% PASO 11: TO WORKSPACE (para análisis posterior)
% ========================================================================

add_block('simulink/Sinks/To Workspace', [model_name '/To_WS_Roll']);
set_param([model_name '/To_WS_Roll'], 'Position', [750, 30, 810, 60]);
set_param([model_name '/To_WS_Roll'], 'VariableName', 'roll_output');
set_param([model_name '/To_WS_Roll'], 'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [model_name '/To_WS_Yaw']);
set_param([model_name '/To_WS_Yaw'], 'Position', [750, 220, 810, 250]);
set_param([model_name '/To_WS_Yaw'], 'VariableName', 'yaw_output');
set_param([model_name '/To_WS_Yaw'], 'SaveFormat', 'Timeseries');

%% ========================================================================
% PASO 12: PERTURBACIONES Y RUIDO (OPCIONAL)
% ========================================================================

% Ruido de medición
add_block('simulink/Sources/Band-Limited White Noise', [model_name '/Measurement_Noise']);
set_param([model_name '/Measurement_Noise'], 'Position', [480, 230, 510, 260]);
set_param([model_name '/Measurement_Noise'], 'Cov', '0.0001');
set_param([model_name '/Measurement_Noise'], 'SampleTime', '0');

% Sumador para agregar ruido
add_block('simulink/Math Operations/Sum', [model_name '/Add_Noise']);
set_param([model_name '/Add_Noise'], 'Position', [620, 135, 640, 165]);
set_param([model_name '/Add_Noise'], 'Inputs', '++');

%% ========================================================================
% PASO 13: CONECTAR BLOQUES
% ========================================================================

fprintf('Conectando bloques...\n');

% Referencias al Mux
add_line(model_name, 'Ref_p/1', 'Mux_Ref/1');
add_line(model_name, 'Ref_Roll/1', 'Mux_Ref/2');
add_line(model_name, 'Ref_r/1', 'Mux_Ref/3');
add_line(model_name, 'Ref_Yaw/1', 'Mux_Ref/4');

% Mux al sumador
add_line(model_name, 'Mux_Ref/1', 'Error/1');

% Error al controlador
add_line(model_name, 'Error/1', 'Controller_Hinf/1');

% Controlador a scope de control
add_line(model_name, 'Controller_Hinf/1', 'Scope_Control/1', 'autorouting', 'on');

% Controlador a planta
add_line(model_name, 'Controller_Hinf/1', 'UAV_Plant/1');

% Planta al sumador de ruido
add_line(model_name, 'UAV_Plant/1', 'Add_Noise/1');

% Ruido al sumador
add_line(model_name, 'Measurement_Noise/1', 'Add_Noise/2');

% Salida con ruido al Demux
add_line(model_name, 'Add_Noise/1', 'Demux_Output/1');

% Demux a retroalimentación (negativa al error)
add_line(model_name, 'Demux_Output/1', 'Error/2', 'autorouting', 'on');

% Salidas Roll (canal 3)
add_line(model_name, 'Demux_Output/3', 'Scope_Roll/2', 'autorouting', 'on');
add_line(model_name, 'Ref_Roll/1', 'Scope_Roll/1', 'autorouting', 'on');
add_line(model_name, 'Demux_Output/3', 'To_WS_Roll/1', 'autorouting', 'on');

% Salidas Yaw (canal 4)
add_line(model_name, 'Demux_Output/4', 'Scope_Yaw/2', 'autorouting', 'on');
add_line(model_name, 'Ref_Yaw/1', 'Scope_Yaw/1', 'autorouting', 'on');
add_line(model_name, 'Demux_Output/4', 'To_WS_Yaw/1', 'autorouting', 'on');

%% ========================================================================
% PASO 14: ANOTACIONES Y DOCUMENTACIÓN
% ========================================================================

fprintf('Agregando anotaciones...\n');

% Título
add_block('built-in/Note', [model_name '/Title']);
set_param([model_name '/Title'], 'Position', [50, 350, 300, 400]);
set_param([model_name '/Title'], 'Text', sprintf(['UAV H∞ CONTROL - LATERAL-DIRECCIONAL\n' ...
    'Universidad Nacional de Colombia\n' ...
    'γ = %.4f\n' ...
    'Orden Controlador: %d'], gamma_LD, order(K_LD)));

% Subsistema de Referencias
add_block('built-in/Note', [model_name '/Note_Ref']);
set_param([model_name '/Note_Ref'], 'Position', [50, 320, 150, 340]);
set_param([model_name '/Note_Ref'], 'Text', 'Referencias');

% Subsistema de Control
add_block('built-in/Note', [model_name '/Note_Control']);
set_param([model_name '/Note_Control'], 'Position', [280, 200, 380, 220]);
set_param([model_name '/Note_Control'], 'Text', 'Controlador H∞');

% Subsistema de Planta
add_block('built-in/Note', [model_name '/Note_Plant']);
set_param([model_name '/Note_Plant'], 'Position', [480, 200, 580, 220]);
set_param([model_name '/Note_Plant'], 'Text', 'Planta UAV');

%% ========================================================================
% PASO 15: GUARDAR Y CONFIGURAR VISUALIZACIÓN
% ========================================================================

fprintf('Guardando modelo...\n');

% Guardar modelo
save_system(model_name);

% Ajustar zoom
set_param(model_name, 'ZoomFactor', 'FitSystem');

fprintf('\n========================================\n');
fprintf('✓ MODELO SIMULINK CREADO EXITOSAMENTE\n');
fprintf('========================================\n\n');

fprintf('Información del modelo:\n');
fprintf('  Nombre: %s.slx\n', model_name);
fprintf('  Tiempo de simulación: 30 segundos\n');
fprintf('  Solver: ode45 (paso variable)\n');
fprintf('  Orden del controlador: %d\n', order(K_LD));
fprintf('  γ óptimo: %.4f\n\n', gamma_LD);

fprintf('Bloques incluidos:\n');
fprintf('  ✓ Referencias: Step (Roll, Yaw) + Constantes (p, r)\n');
fprintf('  ✓ Controlador H∞ (State-Space)\n');
fprintf('  ✓ Planta UAV lateral (State-Space)\n');
fprintf('  ✓ Ruido de medición (Band-Limited White Noise)\n');
fprintf('  ✓ Scopes: Roll, Yaw, Control\n');
fprintf('  ✓ To Workspace: roll_output, yaw_output\n\n');

fprintf('Para ejecutar la simulación:\n');
fprintf('  1. Haz clic en "Run" en la barra de herramientas\n');
fprintf('  2. Observa los scopes en tiempo real\n');
fprintf('  3. Los datos se guardan automáticamente en el workspace\n\n');

fprintf('Para modificar referencias:\n');
fprintf('  - Doble clic en "Ref_Roll" o "Ref_Yaw"\n');
fprintf('  - Cambiar "After" para amplitud del escalón\n');
fprintf('  - Cambiar "Time" para retrasar el escalón\n\n');

fprintf('Variables disponibles después de la simulación:\n');
fprintf('  - roll_output (Timeseries)\n');
fprintf('  - yaw_output (Timeseries)\n\n');

%% ========================================================================
% PASO 16: CREAR SCRIPT DE ANÁLISIS POST-SIMULACIÓN
% ========================================================================

% Crear script auxiliar para análisis
fid = fopen('analizar_simulacion_hinf.m', 'w');
fprintf(fid, '%% ANÁLISIS DE RESULTADOS DE SIMULACIÓN H∞\n');
fprintf(fid, '%% Ejecutar DESPUÉS de correr la simulación en Simulink\n\n');
fprintf(fid, 'if ~exist(''roll_output'', ''var'')\n');
fprintf(fid, '    error(''Primero ejecuta la simulación en Simulink'');\n');
fprintf(fid, 'end\n\n');
fprintf(fid, 'fprintf(''Analizando resultados...\\n\\n'');\n\n');
fprintf(fid, '%% Extraer datos\n');
fprintf(fid, 't_roll = roll_output.Time;\n');
fprintf(fid, 'y_roll = roll_output.Data;\n');
fprintf(fid, 't_yaw = yaw_output.Time;\n');
fprintf(fid, 'y_yaw = yaw_output.Data;\n\n');
fprintf(fid, '%% Graficar\n');
fprintf(fid, 'figure(''Name'', ''Resultados Simulink H∞'');\n');
fprintf(fid, 'subplot(2,1,1);\n');
fprintf(fid, 'plot(t_roll, y_roll, ''b'', ''LineWidth'', 2);\n');
fprintf(fid, 'hold on; plot(t_roll, ones(size(t_roll)), ''r--'');\n');
fprintf(fid, 'xlabel(''Time (s)''); ylabel(''Roll φ (rad)'');\n');
fprintf(fid, 'title(''Roll Angle Response''); grid on; legend(''Output'', ''Reference'');\n\n');
fprintf(fid, 'subplot(2,1,2);\n');
fprintf(fid, 'plot(t_yaw, y_yaw, ''b'', ''LineWidth'', 2);\n');
fprintf(fid, 'hold on; plot(t_yaw, ones(size(t_yaw)), ''r--'');\n');
fprintf(fid, 'xlabel(''Time (s)''); ylabel(''Yaw ψ (rad)'');\n');
fprintf(fid, 'title(''Yaw Angle Response''); grid on; legend(''Output'', ''Reference'');\n\n');
fprintf(fid, '%% Métricas\n');
fprintf(fid, 'info_roll = stepinfo(y_roll, t_roll, 1, ''SettlingTimeThreshold'', 0.02);\n');
fprintf(fid, 'info_yaw = stepinfo(y_yaw, t_yaw, 1, ''SettlingTimeThreshold'', 0.02);\n\n');
fprintf(fid, 'fprintf(''Roll: Ts=%%%.2fs, OS=%%%.1f%%%%\\n'', info_roll.SettlingTime, info_roll.Overshoot);\n');
fprintf(fid, 'fprintf(''Yaw:  Ts=%%%.2fs, OS=%%%.1f%%%%\\n'', info_yaw.SettlingTime, info_yaw.Overshoot);\n');
fclose(fid);

fprintf('✓ Script de análisis creado: analizar_simulacion_hinf.m\n\n');

fprintf('========================================\n');
fprintf('PROCESO COMPLETADO\n');
fprintf('========================================\n');