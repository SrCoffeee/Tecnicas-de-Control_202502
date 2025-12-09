function identificacionParametros()
% IDENTIFICACIÓN DE PARÁMETROS DEL AEROBALANCÍN
% ==============================================
% Calcula: ωn (frecuencia natural)
%          ζ (factor de amortiguamiento)
%          B (coeficiente de fricción)
%
% MÉTODO: Análisis de respuesta al escalón

    % Configuración
    fprintf('=== IDENTIFICACIÓN DE PARÁMETROS ===\n\n');
    
    % Conectar a Arduino
    puerto = input('Puerto COM (ej: COM3): ', 's');
    if isempty(puerto)
        puerto = 'COM3';
    end
    
    try
        arduino = serialport(puerto, 115200);
        configureTerminator(arduino, "LF");
        fprintf('✓ Conectado a %s\n\n', puerto);
    catch
        error('No se pudo conectar al Arduino');
    end
    
    % Limpiar buffer
    pause(0.5);
    flush(arduino);
    
    % Iniciar identificación
    fprintf('Iniciando identificación...\n');
    writeline(arduino, 'START');
    
    % Esperar mensaje
    timeout = 10;
    tic;
    while toc < timeout
        if arduino.NumBytesAvailable > 0
            linea = readline(arduino);
            fprintf('%s\n', linea);
            
            if contains(linea, 'Identificación completa')
                break;
            end
        end
        pause(0.1);
    end
    
    fprintf('\nObteniendo datos...\n');
    pause(1);
    
    % Solicitar datos
    writeline(arduino, 'DATA');
    pause(0.5);
    
    % Leer datos
    tiempo = [];
    angulo = [];
    leyendo = false;
    
    timeout = 5;
    tic;
    while toc < timeout
        if arduino.NumBytesAvailable > 0
            linea = readline(arduino);
            
            if contains(linea, 'DATA_START')
                leyendo = true;
                fprintf('Recibiendo datos...\n');
                continue;
            end
            
            if contains(linea, 'DATA_END')
                break;
            end
            
            if leyendo
                datos = str2double(split(linea, ','));
                if length(datos) == 2
                    tiempo = [tiempo; datos(1)/1000]; % ms a s
                    angulo = [angulo; datos(2)];
                end
            end
        end
        pause(0.01);
    end
    
    % Cerrar puerto
    delete(arduino);
    
    % Verificar datos
    if isempty(tiempo)
        error('No se recibieron datos');
    end
    
    fprintf('✓ Datos recibidos: %d muestras\n\n', length(tiempo));
    
    % ===== ANÁLISIS DE DATOS =====
    procesarDatos(tiempo, angulo);
end

function procesarDatos(t, theta)
    % Convertir a radianes
    theta_rad = theta * pi / 180;
    
    % ===== PASO 1: Identificar región de decaimiento =====
    % Detectar pico máximo
    [pico_max, idx_max] = max(theta);
    
    % Tomar solo la región de decaimiento libre
    t_decay = t(idx_max:end) - t(idx_max);
    theta_decay = theta(idx_max:end);
    
    fprintf('=== RESULTADOS DE IDENTIFICACIÓN ===\n\n');
    fprintf('Ángulo máximo alcanzado: %.2f°\n', pico_max);
    
    % ===== PASO 2: Método del decremento logarítmico =====
    % Encontrar picos sucesivos
    [picos, locs_picos] = findpeaks(theta_decay, 'MinPeakHeight', 0.5);
    
    if length(picos) >= 2
        % Decremento logarítmico
        delta = log(picos(1) / picos(2));
        
        % Factor de amortiguamiento
        zeta = delta / sqrt((2*pi)^2 + delta^2);
        
        % Período amortiguado
        T_d = t_decay(locs_picos(2)) - t_decay(locs_picos(1));
        omega_d = 2*pi / T_d;
        
        % Frecuencia natural
        omega_n = omega_d / sqrt(1 - zeta^2);
        
        fprintf('\n--- Método del Decremento Logarítmico ---\n');
        fprintf('Decremento logarítmico (δ): %.4f\n', delta);
        fprintf('Factor de amortiguamiento (ζ): %.4f\n', zeta);
        fprintf('Frecuencia natural (ωn): %.3f rad/s\n', omega_n);
        fprintf('Período amortiguado (Td): %.3f s\n', T_d);
        
    else
        fprintf('Advertencia: No se encontraron suficientes picos\n');
        zeta = NaN;
        omega_n = NaN;
    end
    
    % ===== PASO 3: Ajuste de curva exponencial =====
    % Modelo: θ(t) = A*exp(-ζ*ωn*t)*cos(ωd*t + φ)
    
    try
        % Envolvente superior
        env_sup = abs(hilbert(theta_decay));
        
        % Ajuste exponencial de la envolvente
        ft = fittype('a*exp(-b*x)', 'independent', 'x');
        f = fit(t_decay, env_sup, ft, 'StartPoint', [pico_max, 1.0]);
        
        coeff_decay = f.b;  % ζ*ωn
        
        if ~isnan(omega_n) && omega_n > 0
            zeta_fit = coeff_decay / omega_n;
            
            fprintf('\n--- Método de Ajuste de Curva ---\n');
            fprintf('Coeficiente de decaimiento: %.3f\n', coeff_decay);
            fprintf('Factor de amortiguamiento (ζ): %.4f\n', zeta_fit);
        end
    catch
        fprintf('No se pudo realizar ajuste de curva\n');
    end
    
    % ===== PASO 4: FFT para frecuencia natural =====
    % Quitar tendencia
    theta_detrend = detrend(theta_decay);
    
    % FFT
    Fs = 1 / mean(diff(t_decay));  % Frecuencia de muestreo
    L = length(theta_detrend);
    Y = fft(theta_detrend);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L/2))/L;
    
    % Encontrar pico dominante
    [~, idx_peak] = max(P1(2:end));  % Ignorar DC
    freq_peak = f(idx_peak + 1);
    omega_d_fft = 2*pi*freq_peak;
    
    fprintf('\n--- Análisis FFT ---\n');
    fprintf('Frecuencia dominante: %.3f Hz\n', freq_peak);
    fprintf('Frecuencia angular (ωd): %.3f rad/s\n', omega_d_fft);
    
    % Estimar ωn desde ωd
    if ~isnan(zeta) && zeta < 1
        omega_n_fft = omega_d_fft / sqrt(1 - zeta^2);
        fprintf('Frecuencia natural estimada (ωn): %.3f rad/s\n', omega_n_fft);
    end
    
    % ===== PASO 5: Calcular coeficiente de fricción B =====
    % Modelo: J*θ̈ + B*θ̇ + K*θ = 0
    % Donde: ωn² = K/J, 2ζωn = B/J
    
    % Parámetros conocidos del sistema
    fprintf('\n=== PARÁMETROS FÍSICOS ===\n');
    J_nominal = input('Inercia J [kg·m²] (Enter para 0.0625): ');
    if isempty(J_nominal)
        J_nominal = 0.0625;
    end
    
    if ~isnan(zeta) && ~isnan(omega_n)
        B_calc = 2 * zeta * omega_n * J_nominal;
        K_calc = omega_n^2 * J_nominal;
        
        fprintf('\n--- Resultados Finales ---\n');
        fprintf('Inercia (J): %.4f kg·m²\n', J_nominal);
        fprintf('Fricción viscosa (B): %.5f N·m·s\n', B_calc);
        fprintf('Rigidez equivalente (K): %.4f N·m/rad\n', K_calc);
        fprintf('Constante de tiempo (τ): %.3f s\n', 1/(zeta*omega_n));
    end
    
    % ===== GRÁFICAS =====
    figure('Position', [100 100 1200 800]);
    
    % Subplot 1: Respuesta completa
    subplot(2,2,1);
    plot(t, theta, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Tiempo [s]');
    ylabel('Ángulo [°]');
    title('Respuesta Completa al Escalón');
    hold on;
    plot(t(idx_max), theta(idx_max), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Respuesta', 'Pico máximo');
    
    % Subplot 2: Decaimiento libre con picos
    subplot(2,2,2);
    plot(t_decay, theta_decay, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Tiempo [s]');
    ylabel('Ángulo [°]');
    title('Decaimiento Libre');
    hold on;
    
    if exist('picos', 'var') && ~isempty(picos)
        plot(t_decay(locs_picos), picos, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    end
    
    if exist('f', 'var')
        t_fit = linspace(0, max(t_decay), 100);
        theta_fit = f(t_fit);
        plot(t_fit, theta_fit, 'r--', 'LineWidth', 1.5);
        plot(t_fit, -theta_fit, 'r--', 'LineWidth', 1.5);
        legend('Respuesta', 'Picos', 'Envolvente');
    end
    
    % Subplot 3: FFT
    subplot(2,2,3);
    plot(f, P1, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Frecuencia [Hz]');
    ylabel('|P1(f)|');
    title('Análisis Espectral (FFT)');
    xlim([0 5]);
    hold on;
    
    if exist('freq_peak', 'var')
        plot(freq_peak, P1(idx_peak+1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        text(freq_peak, P1(idx_peak+1), sprintf(' %.2f Hz', freq_peak), ...
             'FontSize', 10, 'FontWeight', 'bold');
    end
    
    % Subplot 4: Diagrama de polos
    subplot(2,2,4);
    if ~isnan(zeta) && ~isnan(omega_n) && zeta < 1
        % Polos: s = -ζωn ± jωd
        pole_real = -zeta * omega_n;
        pole_imag = omega_d;
        
        plot(pole_real, pole_imag, 'rx', 'MarkerSize', 15, 'LineWidth', 3);
        hold on;
        plot(pole_real, -pole_imag, 'rx', 'MarkerSize', 15, 'LineWidth', 3);
        
        % Círculo unitario en frecuencia
        theta_circle = linspace(0, 2*pi, 100);
        plot(omega_n*cos(theta_circle), omega_n*sin(theta_circle), 'b--');
        
        grid on;
        xlabel('Parte Real');
        ylabel('Parte Imaginaria');
        title('Ubicación de Polos en el Plano S');
        axis equal;
        xlim([-omega_n*1.5 omega_n*0.5]);
        ylim([-omega_n*1.2 omega_n*1.2]);
        
        % Añadir línea de amortiguamiento
        x_line = linspace(-omega_n*1.5, 0, 50);
        y_line_pos = abs(x_line) * tan(acos(zeta));
        y_line_neg = -y_line_pos;
        plot(x_line, y_line_pos, 'g--', 'LineWidth', 1);
        plot(x_line, y_line_neg, 'g--', 'LineWidth', 1);
        
        legend('Polos', '', 'Círculo ωn', sprintf('Línea ζ=%.2f', zeta), ...
               'Location', 'best');
    else
        text(0.5, 0.5, 'No disponible', ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 12);
        axis off;
    end
    
    % Ajustar layout
    sgtitle('Análisis de Identificación de Parámetros', ...
            'FontSize', 14, 'FontWeight', 'bold');
    
    % ===== GUARDAR RESULTADOS =====
    guardar = input('\n¿Guardar resultados? (s/n): ', 's');
    
    if strcmpi(guardar, 's')
        % Crear estructura de resultados
        resultados.tiempo = t;
        resultados.angulo = theta;
        resultados.zeta = zeta;
        resultados.omega_n = omega_n;
        resultados.B = B_calc;
        resultados.J = J_nominal;
        
        if exist('K_calc', 'var')
            resultados.K = K_calc;
        end
        
        % Guardar
        [file, path] = uiputfile('parametros_identificados.mat', ...
                                 'Guardar resultados');
        if file ~= 0
            save(fullfile(path, file), 'resultados');
            fprintf('✓ Resultados guardados en: %s\n', file);
        end
        
        % Exportar gráficas
        [file_fig, path_fig] = uiputfile('identificacion.png', ...
                                          'Guardar gráficas');
        if file_fig ~= 0
            saveas(gcf, fullfile(path_fig, file_fig));
            fprintf('✓ Gráficas guardadas en: %s\n', file_fig);
        end
        
        % Generar reporte en texto
        [file_txt, path_txt] = uiputfile('reporte_identificacion.txt', ...
                                          'Guardar reporte');
        if file_txt ~= 0
            generarReporte(fullfile(path_txt, file_txt), resultados);
        end
    end
    
    fprintf('\n✓ Identificación completa\n');
end

function generarReporte(archivo, res)
    fid = fopen(archivo, 'w');
    
    fprintf(fid, '========================================\n');
    fprintf(fid, 'REPORTE DE IDENTIFICACIÓN DE PARÁMETROS\n');
    fprintf(fid, '========================================\n\n');
    fprintf(fid, 'Fecha: %s\n\n', datestr(now));
    
    fprintf(fid, 'PARÁMETROS IDENTIFICADOS:\n');
    fprintf(fid, '-------------------------\n');
    fprintf(fid, 'Factor de amortiguamiento (ζ): %.4f\n', res.zeta);
    fprintf(fid, 'Frecuencia natural (ωn): %.3f rad/s\n', res.omega_n);
    fprintf(fid, 'Inercia (J): %.4f kg·m²\n', res.J);
    fprintf(fid, 'Fricción viscosa (B): %.5f N·m·s\n', res.B);
    
    if isfield(res, 'K')
        fprintf(fid, 'Rigidez equivalente (K): %.4f N·m/rad\n', res.K);
    end
    
    fprintf(fid, '\nFUNCIÓN DE TRANSFERENCIA:\n');
    fprintf(fid, '-------------------------\n');
    fprintf(fid, '         %.4f\n', res.omega_n^2);
    fprintf(fid, 'G(s) = ──────────────────────\n');
    fprintf(fid, '       s² + %.3fs + %.3f\n', ...
            2*res.zeta*res.omega_n, res.omega_n^2);
    
    fprintf(fid, '\nPARA USAR EN ARDUINO:\n');
    fprintf(fid, '--------------------\n');
    fprintf(fid, 'const float zeta = %.4f;\n', res.zeta);
    fprintf(fid, 'const float omega_n = %.3f;\n', res.omega_n);
    fprintf(fid, 'const float B = %.5f;\n', res.B);
    
    fclose(fid);
    fprintf('✓ Reporte guardado en: %s\n', archivo);
end
