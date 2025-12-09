function AerobalancinGUI()
% INTERFAZ GRÁFICA PARA CONTROL DE AEROBALANCÍN
% =============================================
% Sistema de control con selección de controladores
% Visualización en tiempo real
% Análisis de respuestas (escalón, rampa, impulsos)
%
% Autor: Sistema de Control Avanzado
% Fecha: 2024

    % Variables globales
    fig = [];
    arduino = [];
    isConnected = false;
    isRunning = false;
    
    % Arrays de datos
    tiempoData = [];
    setpointData = [];
    anguloData = [];
    errorData = [];
    pwmData = [];
    gyroData = [];
    theta1Data = [];
    theta2Data = [];
    theta3Data = [];
    
    % Parámetros
    tiempoActual = 0;
    controladorActual = 0;
    referenciaActual = 0;
    
    % Crear figura principal
    crearInterfaz();
    
    function crearInterfaz()
        % Crear ventana principal
        fig = figure('Name', 'Control de Aerobalancín - Sistema Avanzado', ...
                     'NumberTitle', 'off', ...
                     'Position', [50 50 1400 900], ...
                     'MenuBar', 'none', ...
                     'ToolBar', 'none', ...
                     'CloseRequestFcn', @cerrarAplicacion);
        
        % Color de fondo
        set(fig, 'Color', [0.95 0.95 0.95]);
        
        % ===== PANEL DE CONEXIÓN =====
        panelConexion = uipanel('Parent', fig, ...
                               'Title', 'Conexión Serial', ...
                               'FontSize', 11, ...
                               'FontWeight', 'bold', ...
                               'Position', [0.01 0.88 0.25 0.11]);
        
        % Puerto COM
        uicontrol('Parent', panelConexion, ...
                 'Style', 'text', ...
                 'String', 'Puerto:', ...
                 'Position', [10 40 60 20], ...
                 'HorizontalAlignment', 'left');
        
        puertoCOM = uicontrol('Parent', panelConexion, ...
                             'Style', 'edit', ...
                             'String', 'COM3', ...
                             'Position', [80 40 80 25]);
        
        % Botón conectar
        btnConectar = uicontrol('Parent', panelConexion, ...
                               'Style', 'pushbutton', ...
                               'String', 'CONECTAR', ...
                               'Position', [170 38 120 28], ...
                               'FontWeight', 'bold', ...
                               'BackgroundColor', [0.2 0.8 0.2], ...
                               'ForegroundColor', 'white', ...
                               'Callback', @conectarArduino);
        
        % Indicador de estado
        estadoIndicador = uicontrol('Parent', panelConexion, ...
                                   'Style', 'text', ...
                                   'String', '● DESCONECTADO', ...
                                   'Position', [10 10 280 20], ...
                                   'ForegroundColor', [0.8 0 0], ...
                                   'FontWeight', 'bold', ...
                                   'FontSize', 10);
        
        % ===== PANEL DE CONTROLADORES =====
        panelControl = uipanel('Parent', fig, ...
                              'Title', 'Selección de Controlador', ...
                              'FontSize', 11, ...
                              'FontWeight', 'bold', ...
                              'Position', [0.27 0.88 0.30 0.11]);
        
        % Botones de controladores
        btnPID = uicontrol('Parent', panelControl, ...
                          'Style', 'pushbutton', ...
                          'String', 'PID', ...
                          'Position', [10 38 80 30], ...
                          'FontWeight', 'bold', ...
                          'BackgroundColor', [0.3 0.6 1], ...
                          'ForegroundColor', 'white', ...
                          'Callback', @(~,~)iniciarControl(1));
        
        btnHinf = uicontrol('Parent', panelControl, ...
                           'Style', 'pushbutton', ...
                           'String', 'H∞', ...
                           'Position', [100 38 80 30], ...
                           'FontWeight', 'bold', ...
                           'BackgroundColor', [0.3 0.6 1], ...
                           'ForegroundColor', 'white', ...
                           'Callback', @(~,~)iniciarControl(2));
        
        btnSMC = uicontrol('Parent', panelControl, ...
                          'Style', 'pushbutton', ...
                          'String', 'SMC', ...
                          'Position', [190 38 80 30], ...
                          'FontWeight', 'bold', ...
                          'BackgroundColor', [0.3 0.6 1], ...
                          'ForegroundColor', 'white', ...
                          'Callback', @(~,~)iniciarControl(3));
        
        btnAdapt = uicontrol('Parent', panelControl, ...
                            'Style', 'pushbutton', ...
                            'String', 'ADAPT', ...
                            'Position', [280 38 80 30], ...
                            'FontWeight', 'bold', ...
                            'BackgroundColor', [0.3 0.6 1], ...
                            'ForegroundColor', 'white', ...
                            'Callback', @(~,~)iniciarControl(4));
        
        % Botón STOP
        btnStop = uicontrol('Parent', panelControl, ...
                           'Style', 'pushbutton', ...
                           'String', '⏹ DETENER', ...
                           'Position', [370 38 80 30], ...
                           'FontWeight', 'bold', ...
                           'BackgroundColor', [0.9 0.2 0.2], ...
                           'ForegroundColor', 'white', ...
                           'Callback', @detenerControl);
        
        % Indicador de controlador activo
        lblControlador = uicontrol('Parent', panelControl, ...
                                  'Style', 'text', ...
                                  'String', 'Ninguno activo', ...
                                  'Position', [10 10 440 20], ...
                                  'FontSize', 9, ...
                                  'FontWeight', 'bold');
        
        % ===== PANEL DE REFERENCIA =====
        panelRef = uipanel('Parent', fig, ...
                          'Title', 'Tipo de Referencia', ...
                          'FontSize', 11, ...
                          'FontWeight', 'bold', ...
                          'Position', [0.58 0.88 0.25 0.11]);
        
        % Botones de referencia
        btnEscalon = uicontrol('Parent', panelRef, ...
                              'Style', 'pushbutton', ...
                              'String', '📊 Escalón', ...
                              'Position', [10 38 100 30], ...
                              'FontWeight', 'bold', ...
                              'Callback', @(~,~)cambiarReferencia(0));
        
        btnRampa = uicontrol('Parent', panelRef, ...
                            'Style', 'pushbutton', ...
                            'String', '📈 Rampa', ...
                            'Position', [120 38 100 30], ...
                            'FontWeight', 'bold', ...
                            'Callback', @(~,~)cambiarReferencia(1));
        
        btnImpulsos = uicontrol('Parent', panelRef, ...
                               'Style', 'pushbutton', ...
                               'String', '⚡ Impulsos', ...
                               'Position', [230 38 100 30], ...
                               'FontWeight', 'bold', ...
                               'Callback', @(~,~)cambiarReferencia(2));
        
        % Setpoint
        uicontrol('Parent', panelRef, ...
                 'Style', 'text', ...
                 'String', 'Setpoint:', ...
                 'Position', [10 10 60 20]);
        
        editSetpoint = uicontrol('Parent', panelRef, ...
                                'Style', 'edit', ...
                                'String', '45', ...
                                'Position', [80 10 60 23]);
        
        uicontrol('Parent', panelRef, ...
                 'Style', 'pushbutton', ...
                 'String', 'Aplicar', ...
                 'Position', [150 10 70 23], ...
                 'Callback', @aplicarSetpoint);
        
        % ===== PANEL DE ANÁLISIS =====
        panelAnalisis = uipanel('Parent', fig, ...
                               'Title', 'Herramientas', ...
                               'FontSize', 11, ...
                               'FontWeight', 'bold', ...
                               'Position', [0.84 0.88 0.15 0.11]);
        
        btnLimpiar = uicontrol('Parent', panelAnalisis, ...
                              'Style', 'pushbutton', ...
                              'String', '🗑 Limpiar', ...
                              'Position', [10 38 90 28], ...
                              'Callback', @limpiarGraficas);
        
        btnGuardar = uicontrol('Parent', panelAnalisis, ...
                              'Style', 'pushbutton', ...
                              'String', '💾 Guardar', ...
                              'Position', [110 38 90 28], ...
                              'Callback', @guardarDatos);
        
        btnIdent = uicontrol('Parent', panelAnalisis, ...
                            'Style', 'pushbutton', ...
                            'String', '🔍 Ident', ...
                            'Position', [10 8 90 25], ...
                            'Callback', @iniciarIdentificacion);
        
        btnExportar = uicontrol('Parent', panelAnalisis, ...
                               'Style', 'pushbutton', ...
                               'String', '📊 Export', ...
                               'Position', [110 8 90 25], ...
                               'Callback', @exportarGraficas);
        
        % ===== GRÁFICAS =====
        % Subplot 1: Respuesta Angular
        ax1 = subplot(3,2,1);
        hold(ax1, 'on');
        grid(ax1, 'on');
        title(ax1, 'Respuesta Angular', 'FontSize', 11, 'FontWeight', 'bold');
        xlabel(ax1, 'Tiempo [s]', 'FontSize', 10);
        ylabel(ax1, 'Ángulo [°]', 'FontSize', 10);
        lineSetpoint = plot(ax1, 0, 0, 'r--', 'LineWidth', 2, 'DisplayName', 'Setpoint');
        lineAngulo = plot(ax1, 0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ángulo');
        legend(ax1, 'Location', 'best');
        set(ax1, 'Position', [0.08 0.68 0.40 0.17]);
        
        % Subplot 2: Error
        ax2 = subplot(3,2,2);
        hold(ax2, 'on');
        grid(ax2, 'on');
        title(ax2, 'Error de Seguimiento', 'FontSize', 11, 'FontWeight', 'bold');
        xlabel(ax2, 'Tiempo [s]', 'FontSize', 10);
        ylabel(ax2, 'Error [°]', 'FontSize', 10);
        lineError = plot(ax2, 0, 0, 'r-', 'LineWidth', 1.5);
        yline(ax2, 0, 'k--', 'LineWidth', 1);
        set(ax2, 'Position', [0.56 0.68 0.40 0.17]);
        
        % Subplot 3: Esfuerzo de Control
        ax3 = subplot(3,2,3);
        hold(ax3, 'on');
        grid(ax3, 'on');
        title(ax3, 'Esfuerzo de Control (PWM)', 'FontSize', 11, 'FontWeight', 'bold');
        xlabel(ax3, 'Tiempo [s]', 'FontSize', 10);
        ylabel(ax3, 'PWM [%]', 'FontSize', 10);
        linePWM = plot(ax3, 0, 0, 'g-', 'LineWidth', 1.5);
        ylim(ax3, [0 40]);
        set(ax3, 'Position', [0.08 0.42 0.40 0.17]);
        
        % Subplot 4: Velocidad Angular (Gyro)
        ax4 = subplot(3,2,4);
        hold(ax4, 'on');
        grid(ax4, 'on');
        title(ax4, 'Velocidad Angular', 'FontSize', 11, 'FontWeight', 'bold');
        xlabel(ax4, 'Tiempo [s]', 'FontSize', 10);
        ylabel(ax4, 'ω [°/s]', 'FontSize', 10);
        lineGyro = plot(ax4, 0, 0, 'm-', 'LineWidth', 1.5);
        yline(ax4, 0, 'k--', 'LineWidth', 1);
        set(ax4, 'Position', [0.56 0.42 0.40 0.17]);
        
        % Subplot 5: Parámetros Adaptativos
        ax5 = subplot(3,2,5);
        hold(ax5, 'on');
        grid(ax5, 'on');
        title(ax5, 'Parámetros Adaptativos (θ₁, θ₂, θ₃)', 'FontSize', 11, 'FontWeight', 'bold');
        xlabel(ax5, 'Tiempo [s]', 'FontSize', 10);
        ylabel(ax5, 'Valor', 'FontSize', 10);
        lineTheta1 = plot(ax5, 0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'θ₁');
        lineTheta2 = plot(ax5, 0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'θ₂');
        lineTheta3 = plot(ax5, 0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'θ₃');
        legend(ax5, 'Location', 'best');
        set(ax5, 'Position', [0.08 0.16 0.40 0.17]);
        
        % Subplot 6: Métricas en Tiempo Real
        ax6 = subplot(3,2,6);
        axis(ax6, 'off');
        txtMetricas = uicontrol('Parent', fig, ...
                               'Style', 'text', ...
                               'String', 'Métricas de Desempeño', ...
                               'Position', [780 145 400 120], ...
                               'FontSize', 10, ...
                               'HorizontalAlignment', 'left', ...
                               'BackgroundColor', [1 1 1]);
        set(ax6, 'Position', [0.56 0.16 0.40 0.17]);
        
        % ===== PANEL DE INFORMACIÓN =====
        panelInfo = uipanel('Parent', fig, ...
                           'Title', 'Información del Sistema', ...
                           'FontSize', 10, ...
                           'FontWeight', 'bold', ...
                           'Position', [0.01 0.01 0.98 0.13]);
        
        txtInfo = uicontrol('Parent', panelInfo, ...
                           'Style', 'text', ...
                           'String', 'Sistema listo. Conecte Arduino para comenzar.', ...
                           'Position', [10 10 1360 95], ...
                           'FontSize', 9, ...
                           'HorizontalAlignment', 'left', ...
                           'BackgroundColor', [1 1 1]);
    end

    % ===== FUNCIONES DE CALLBACK =====
    
    function conectarArduino(~, ~)
        if ~isConnected
            try
                puerto = get(puertoCOM, 'String');
                arduino = serialport(puerto, 115200);
                configureTerminator(arduino, "LF");
                isConnected = true;
                
                set(estadoIndicador, 'String', '● CONECTADO', ...
                    'ForegroundColor', [0 0.8 0]);
                set(btnConectar, 'String', 'DESCONECTAR', ...
                    'BackgroundColor', [0.8 0.2 0.2]);
                
                actualizarInfo('Arduino conectado en ' + string(puerto));
                
                % Iniciar timer de lectura
                tmr = timer('ExecutionMode', 'fixedRate', ...
                           'Period', 0.05, ...
                           'TimerFcn', @leerDatosArduino);
                start(tmr);
                
            catch ME
                actualizarInfo('Error: ' + string(ME.message));
            end
        else
            % Desconectar
            if exist('tmr', 'var') && isvalid(tmr)
                stop(tmr);
                delete(tmr);
            end
            
            delete(arduino);
            isConnected = false;
            
            set(estadoIndicador, 'String', '● DESCONECTADO', ...
                'ForegroundColor', [0.8 0 0]);
            set(btnConectar, 'String', 'CONECTAR', ...
                'BackgroundColor', [0.2 0.8 0.2]);
            
            actualizarInfo('Arduino desconectado');
        end
    end

    function iniciarControl(tipo)
        if ~isConnected
            actualizarInfo('Error: Arduino no conectado');
            return;
        end
        
        % Limpiar datos anteriores
        limpiarGraficas();
        
        % Enviar comando
        writeline(arduino, sprintf('START:%d', tipo));
        isRunning = true;
        controladorActual = tipo;
        
        nombres = {'PID Clásico', 'H∞ Mixto', 'SMC', 'Adaptativo'};
        set(lblControlador, 'String', sprintf('Activo: %s', nombres{tipo}));
        actualizarInfo(sprintf('Control %s iniciado', nombres{tipo}));
    end

    function detenerControl(~, ~)
        if isConnected
            writeline(arduino, 'STOP');
            isRunning = false;
            controladorActual = 0;
            set(lblControlador, 'String', 'Ninguno activo');
            actualizarInfo('Control detenido');
        end
    end

    function cambiarReferencia(tipo)
        if ~isConnected
            return;
        end
        
        writeline(arduino, sprintf('REF:%d', tipo));
        referenciaActual = tipo;
        
        tipos = {'Escalón', 'Rampa', 'Tren de Impulsos'};
        actualizarInfo(sprintf('Referencia cambiada a: %s', tipos{tipo+1}));
    end

    function aplicarSetpoint(~, ~)
        if ~isConnected
            return;
        end
        
        sp = str2double(get(editSetpoint, 'String'));
        writeline(arduino, sprintf('SET:%.2f', sp));
        actualizarInfo(sprintf('Setpoint cambiado a %.2f°', sp));
    end

    function leerDatosArduino(~, ~)
        if ~isConnected || arduino.NumBytesAvailable == 0
            return;
        end
        
        try
            linea = readline(arduino);
            
            if startsWith(linea, 'DATA,')
                % Parsear datos
                datos = str2double(split(linea(6:end), ','));
                
                if length(datos) >= 9
                    tiempo = datos(1) / 1000.0;  % ms a segundos
                    sp = datos(2);
                    ang = datos(3);
                    err = datos(4);
                    pwm = datos(5);
                    gyro = datos(6);
                    th1 = datos(7);
                    th2 = datos(8);
                    th3 = datos(9);
                    
                    % Almacenar
                    tiempoData = [tiempoData; tiempo];
                    setpointData = [setpointData; sp];
                    anguloData = [anguloData; ang];
                    errorData = [errorData; err];
                    pwmData = [pwmData; pwm];
                    gyroData = [gyroData; gyro];
                    theta1Data = [theta1Data; th1];
                    theta2Data = [theta2Data; th2];
                    theta3Data = [theta3Data; th3];
                    
                    % Actualizar gráficas (cada 10 muestras para eficiencia)
                    if mod(length(tiempoData), 10) == 0
                        actualizarGraficas();
                    end
                end
            elseif startsWith(linea, 'MSG,')
                actualizarInfo(extractAfter(linea, 4));
            end
            
        catch ME
            % Silenciar errores de lectura
        end
    end

    function actualizarGraficas()
        if isempty(tiempoData)
            return;
        end
        
        % Gráfica 1: Respuesta angular
        set(lineSetpoint, 'XData', tiempoData, 'YData', setpointData);
        set(lineAngulo, 'XData', tiempoData, 'YData', anguloData);
        xlim(ax1, [max(0, tiempoData(end)-30) tiempoData(end)+1]);
        
        % Gráfica 2: Error
        set(lineError, 'XData', tiempoData, 'YData', errorData);
        xlim(ax2, [max(0, tiempoData(end)-30) tiempoData(end)+1]);
        
        % Gráfica 3: PWM
        set(linePWM, 'XData', tiempoData, 'YData', pwmData);
        xlim(ax3, [max(0, tiempoData(end)-30) tiempoData(end)+1]);
        
        % Gráfica 4: Velocidad
        set(lineGyro, 'XData', tiempoData, 'YData', gyroData);
        xlim(ax4, [max(0, tiempoData(end)-30) tiempoData(end)+1]);
        
        % Gráfica 5: Parámetros adaptativos
        if controladorActual == 4
            set(lineTheta1, 'XData', tiempoData, 'YData', theta1Data);
            set(lineTheta2, 'XData', tiempoData, 'YData', theta2Data);
            set(lineTheta3, 'XData', tiempoData, 'YData', theta3Data);
            xlim(ax5, [max(0, tiempoData(end)-30) tiempoData(end)+1]);
        end
        
        % Actualizar métricas
        actualizarMetricas();
        
        drawnow limitrate;
    end

    function actualizarMetricas()
        if length(errorData) < 10
            return;
        end
        
        % Calcular métricas
        errorAbs = abs(errorData);
        errorRMS = sqrt(mean(errorData.^2));
        errorMax = max(errorAbs);
        errorSS = mean(errorAbs(max(1,end-50):end));
        pwmMean = mean(pwmData);
        pwmStd = std(pwmData);
        
        % Tiempo de establecimiento (2%)
        tol = 0.02 * abs(setpointData(end));
        idx = find(errorAbs <= tol, 1, 'first');
        if ~isempty(idx)
            ts = tiempoData(idx);
        else
            ts = inf;
        end
        
        % Formatear texto
        texto = sprintf(['MÉTRICAS DE DESEMPEÑO\n\n' ...
                        '• Error RMS:        %.3f°\n' ...
                        '• Error Máximo:     %.3f°\n' ...
                        '• Error Estac.:     %.3f°\n' ...
                        '• Tiempo establ.:   %.2f s\n' ...
                        '• PWM promedio:     %.2f%%\n' ...
                        '• PWM desv. std:    %.2f%%'], ...
                        errorRMS, errorMax, errorSS, ts, pwmMean, pwmStd);
        
        set(txtMetricas, 'String', texto);
    end

    function limpiarGraficas(~, ~)
        tiempoData = [];
        setpointData = [];
        anguloData = [];
        errorData = [];
        pwmData = [];
        gyroData = [];
        theta1Data = [];
        theta2Data = [];
        theta3Data = [];
        
        set(lineSetpoint, 'XData', 0, 'YData', 0);
        set(lineAngulo, 'XData', 0, 'YData', 0);
        set(lineError, 'XData', 0, 'YData', 0);
        set(linePWM, 'XData', 0, 'YData', 0);
        set(lineGyro, 'XData', 0, 'YData', 0);
        set(lineTheta1, 'XData', 0, 'YData', 0);
        set(lineTheta2, 'XData', 0, 'YData', 0);
        set(lineTheta3, 'XData', 0, 'YData', 0);
        
        actualizarInfo('Gráficas limpiadas');
    end

    function guardarDatos(~, ~)
        if isempty(tiempoData)
            actualizarInfo('No hay datos para guardar');
            return;
        end
        
        [file, path] = uiputfile('datos_control.mat', 'Guardar datos');
        if file ~= 0
            data.tiempo = tiempoData;
            data.setpoint = setpointData;
            data.angulo = anguloData;
            data.error = errorData;
            data.pwm = pwmData;
            data.gyro = gyroData;
            data.theta1 = theta1Data;
            data.theta2 = theta2Data;
            data.theta3 = theta3Data;
            data.controlador = controladorActual;
            data.referencia = referenciaActual;
            
            save(fullfile(path, file), 'data');
            actualizarInfo(['Datos guardados: ' file]);
        end
    end

    function exportarGraficas(~, ~)
        figExport = figure('Position', [100 100 1200 900]);
        
        subplot(3,2,1);
        plot(tiempoData, setpointData, 'r--', 'LineWidth', 2);
        hold on;
        plot(tiempoData, anguloData, 'b-', 'LineWidth', 1.5);
        grid on;
        title('Respuesta Angular');
        xlabel('Tiempo [s]');
        ylabel('Ángulo [°]');
        legend('Setpoint', 'Ángulo');
        
        subplot(3,2,2);
        plot(tiempoData, errorData, 'r-', 'LineWidth', 1.5);
        grid on;
        title('Error de Seguimiento');
        xlabel('Tiempo [s]');
        ylabel('Error [°]');
        
        subplot(3,2,3);
        plot(tiempoData, pwmData, 'g-', 'LineWidth', 1.5);
        grid on;
        title('Esfuerzo de Control');
        xlabel('Tiempo [s]');
        ylabel('PWM [%]');
        
        subplot(3,2,4);
        plot(tiempoData, gyroData, 'm-', 'LineWidth', 1.5);
        grid on;
        title('Velocidad Angular');
        xlabel('Tiempo [s]');
        ylabel('ω [°/s]');
        
        subplot(3,2,5);
        plot(tiempoData, theta1Data, 'r-', 'LineWidth', 1.5);
        hold on;
        plot(tiempoData, theta2Data, 'b-', 'LineWidth', 1.5);
        plot(tiempoData, theta3Data, 'g-', 'LineWidth', 1.5);
        grid on;
        title('Parámetros Adaptativos');
        xlabel('Tiempo [s]');
        ylabel('Valor');
        legend('θ₁', 'θ₂', 'θ₃');
        
        [file, path] = uiputfile('graficas.png', 'Exportar gráficas');
        if file ~= 0
            saveas(figExport, fullfile(path, file));
            actualizarInfo(['Gráficas exportadas: ' file]);
        end
        
        close(figExport);
    end

    function iniciarIdentificacion(~, ~)
        if ~isConnected
            actualizarInfo('Error: Arduino no conectado');
            return;
        end
        
        writeline(arduino, 'IDENT');
        actualizarInfo('Modo identificación iniciado (10 segundos)');
    end

    function actualizarInfo(mensaje)
        tiempo_str = datestr(now, 'HH:MM:SS');
        texto_actual = get(txtInfo, 'String');
        
        % Agregar nueva línea
        lineas = strsplit(texto_actual, newline);
        if length(lineas) > 10
            lineas = lineas(end-9:end);
        end
        
        nuevo_texto = sprintf('%s\n[%s] %s', strjoin(lineas, newline), ...
                             tiempo_str, mensaje);
        set(txtInfo, 'String', nuevo_texto);
    end

    function cerrarAplicacion(~, ~)
        % Detener control
        if isConnected
            try
                writeline(arduino, 'STOP');
            catch
            end
        end
        
        % Cerrar timer
        tmrs = timerfindall;
        if ~isempty(tmrs)
            stop(tmrs);
            delete(tmrs);
        end
        
        % Cerrar puerto serial
        if isConnected
            delete(arduino);
        end
        
        delete(fig);
    end
end
