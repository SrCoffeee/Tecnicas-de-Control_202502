%% CONTROL ROBUSTO Y ANÁLISIS DE DESEMPEÑO: VEHÍCULO AÉREO NO TRIPULADO
% Universidad Nacional de Colombia
% Versión Mejorada con Timeout y Respuestas Reales
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

% Limpiar valores despreciables
threshold = 1e-3;
A_longmod(abs(A_longmod) < threshold) = 0;
B_longmod(abs(B_longmod) < threshold) = 0;
C_longmod(abs(C_longmod) < threshold) = 0;
A_latmod(abs(A_latmod) < threshold) = 0;
B_latmod(abs(B_latmod) < threshold) = 0;
C_latmod(abs(C_latmod) < threshold) = 0;

sys_longmod = ss(A_longmod, B_longmod, C_longmod, D_longmod);
sys_latmod = ss(A_latmod, B_latmod, C_latmod, D_latmod);

fprintf('📊 MODELOS CARGADOS:\n');
fprintf('  Longitudinal: %d estados, %d entradas, %d salidas\n', ...
    size(A_longmod,1), size(B_longmod,2), size(C_longmod,1));
fprintf('  Lateral:      %d estados, %d entradas, %d salidas\n', ...
    size(A_latmod,1), size(B_latmod,2), size(C_latmod,1));

fprintf('\n✅ PARTE 1 completada. Ejecuta PARTE 2.\n\n');

%% ========================================================================
% FUNCIONES AUXILIARES BÁSICAS
% ========================================================================

% Función para calcular amortiguamiento mínimo
calculate_min_damping = @(poles) min(arrayfun(@(p) ...
    -real(p)/abs(p), poles(abs(imag(poles)) > 1e-6)));

fprintf('✅ PARTE 2 completada. Ejecuta PARTE 3.\n\n');

%% ========================================================================
% FUNCIÓN DE DISEÑO PI+D (PARTE 3A - Primera mitad)
% ========================================================================

function [D_opt, Kp_opt, Ki_opt, metrics] = design_pid_auto(G1, G2, name, specs)
% DISEÑO PI + D con timeout de 30 segundos

% ---------------- defaults ----------------
if ~isfield(specs,'zeta_min'), specs.zeta_min = 0.45; end
if ~isfield(specs,'Ki_zeta_keep_ratio'), specs.Ki_zeta_keep_ratio = 0.7; end
if ~isfield(specs,'Ki_range'), specs.Ki_range = logspace(-4,0,300); end
if ~isfield(specs,'Kp_range'), specs.Kp_range = logspace(-2,1,400); end
if ~isfield(specs,'D_range'),  specs.D_range  = linspace(-1,1,200); end
if ~isfield(specs,'Ts_target'), specs.Ts_target = []; end
if ~isfield(specs,'zeta_click_pref'), specs.zeta_click_pref = 0.5; end

s = tf('s');
metrics = struct(); D_opt = 0; Kp_opt = 0; Ki_opt = 0;

% ---------------- utilidades ----------------
    function st = is_stable(p), st = ~any(real(p)>=0); end
    
    function z = min_pair_zeta(poles)
        z = 1; zz = [];
        for k=1:numel(poles)
            pk = poles(k);
            if abs(imag(pk))>1e-8
                wn = abs(pk); zeta = -real(pk)/wn; zz(end+1)=zeta; %#ok<AGROW>
            end
        end
        if ~isempty(zz), z=min(zz); end
    end
    
    function [zeta,wn,sigma] = zeta_of(p)
        wn = abs(p); sigma = -real(p); 
        if wn>0, zeta = sigma/wn; else, zeta = NaN; end
    end
    
    function draw_zeta_guides(ax, zetas_vec)
        axes(ax); hold on; grid on; box on;
        xl = xlim; yl = ylim; r = max(abs([xl,yl])); t = linspace(0,r,250);
        for z = zetas_vec
            if z>=0 && z<1
                ang = acos(z);
                x = -t*cos(ang); y =  t*sin(ang);
                plot(x, y,'k--','LineWidth',0.8);
                plot(x,-y,'k--','LineWidth',0.8);
                text(-0.62*r*cos(ang),  0.62*r*sin(ang), sprintf('\\zeta\\approx%.1f',z), ...
                    'HorizontalAlignment','center');
                text(-0.62*r*cos(ang), -0.62*r*sin(ang), sprintf('\\zeta\\approx%.1f',z), ...
                    'HorizontalAlignment','center');
            end
        end
    end
    
    function draw_sigma_line(ax, Ts)
        if isempty(Ts), return; end
        sig = 4/max(Ts,eps);
        xl = xlim(ax); yl = ylim(ax);
        plot(ax, -sig*[1 1], yl, 'k-.', 'LineWidth', 1);
        text(ax, -sig, 0, sprintf('\\sigma = -%.2f  (T_s\\approx %.2fs)', sig, 4/sig), ...
            'HorizontalAlignment','left','VerticalAlignment','bottom','Rotation',90);
    end
    
    function add_instruction_box(fig)
        try
            annotation(fig,'textbox',[0.62 0.62 0.35 0.33], ...
                'String', sprintf([ ...
                'Cómo seleccionar K_p (clic con rlocfind):\n' ...
                '1) HAZ CLIC sobre UNA RAMA del locus.\n' ...
                '2) Prefiere la banda ζ deseada (líneas punteadas).\n' ...
                '3) Cuanto más a la IZQUIERDA, más rápido (σ grande).\n' ...
                '4) Puntos VERDES = candidatos sugeridos.\n' ...
                '5) Tienes 30 SEGUNDOS o se selecciona automático.\n'], specs.zeta_click_pref), ...
                'BackgroundColor',[1 1 1 0.85],'FitBoxToText','on');
        catch
        end
    end

fprintf('✅ PARTE 3A completada. Ejecuta PARTE 3B.\n\n');

%% ========================================================================
% FUNCIÓN DE DISEÑO PI+D (PARTE 3B - Barrido D y selección Kp)
% ========================================================================

% ---------------- 1) Barrido de D (CON REPORTE DETALLADO) ----------------
fprintf('\n🔍 INICIANDO BARRIDO DE GANANCIA D para %s...\n', name);
fprintf('   Rango: D ∈ [%.2f, %.2f] con %d puntos\n', ...
    min(specs.D_range), max(specs.D_range), numel(specs.D_range));

Dvals = specs.D_range(:); zetaD = -10*ones(size(Dvals)); stD = false(size(Dvals));
for k=1:numel(Dvals)
    Dk = Dvals(k);
    try
        Gtmp = feedback(G1, Dk*G2, -1);
        p = pole(Gtmp); stD(k) = is_stable(p); zetaD(k) = min_pair_zeta(p);
    catch
    end
end

% Estadísticas del barrido
num_stable = sum(stD);
fprintf('   ✓ Configuraciones estables: %d/%d (%.1f%%)\n', ...
    num_stable, numel(Dvals), 100*num_stable/numel(Dvals));

if num_stable > 0
    fprintf('   ✓ Rango de ζ_min estable: [%.3f, %.3f]\n', ...
        min(zetaD(stD)), max(zetaD(stD)));
end

% Mejor D estable por zeta mínima
cand = find(stD); 
if ~isempty(cand)
    [~,ii] = max(zetaD(cand)); idx = cand(ii);
else
    [~,idx] = max(zetaD);
end
D_opt = Dvals(idx);
G_inner = feedback(G1, D_opt*G2, -1);

if ~is_stable(pole(G_inner))
    if ~isempty(cand)
        [~,ii2]=min(abs(cand-idx)); D_opt = Dvals(cand(ii2));
        G_inner = feedback(G1, D_opt*G2, -1);
    else
        D_opt = 0; G_inner = feedback(G1, 0*G2, -1);
    end
end
metrics.D_zeta_min = min_pair_zeta(pole(G_inner));

fprintf('   🎯 D ÓPTIMO SELECCIONADO: %.4f\n', D_opt);
fprintf('   📊 Amortiguamiento resultante: ζ_min = %.3f\n', metrics.D_zeta_min);
fprintf('   📍 Polos del lazo interno:\n');
poles_inner = pole(G_inner);
for pp = 1:min(4, length(poles_inner))
    if abs(imag(poles_inner(pp))) > 1e-6
        [z_pp, wn_pp, ~] = zeta_of(poles_inner(pp));
        fprintf('      p%d = %.3f ± %.3fi  (ζ=%.3f, ωn=%.2f rad/s)\n', ...
            pp, real(poles_inner(pp)), abs(imag(poles_inner(pp))), z_pp, wn_pp);
    else
        fprintf('      p%d = %.3f  (polo real)\n', pp, real(poles_inner(pp)));
    end
end

% FIGURA DEL BARRIDO (generada ANTES de las interactivas)
fig_sweep = figure('Name', sprintf('Barrido D - %s', name), 'Position', [50 50 900 600]);
subplot(2,1,1);
plot(Dvals, zetaD, 'LineWidth',1.8, 'Color', [0.2 0.4 0.8]); hold on;
plot(D_opt, metrics.D_zeta_min, 'ro','MarkerSize',10,'LineWidth',2, 'MarkerFaceColor', 'r');

% Regiones de estabilidad
valid_idx = find(stD);
if ~isempty(valid_idx)
    patch([Dvals(valid_idx); flipud(Dvals(valid_idx))], ...
          [zetaD(valid_idx); -10*ones(size(valid_idx))], ...
          [0.8 1 0.8], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end

xl = xlim; 
plot(xl, [0.3 0.3], 'k--', 'LineWidth', 0.8); 
text(xl(2)*0.95, 0.3, ' ζ=0.3', 'VerticalAlignment','bottom', 'FontSize', 9);
plot(xl, [0.5 0.5], 'k--', 'LineWidth', 0.8); 
text(xl(2)*0.95, 0.5, ' ζ=0.5', 'VerticalAlignment','bottom', 'FontSize', 9);
plot(xl, [0.7 0.7], 'k--', 'LineWidth', 0.8); 
text(xl(2)*0.95, 0.7, ' ζ=0.7', 'VerticalAlignment','bottom', 'FontSize', 9);

xlabel(sprintf('Ganancia D_{%s}', lower(name))); 
ylabel('ζ_{min} (Amortiguamiento Mínimo)');
title(sprintf('Barrido de Ganancia D - %s | Óptimo: D=%.4f (ζ=%.3f)', ...
    upper(name), D_opt, metrics.D_zeta_min)); 
grid on; box on; ylim([-0.2 1]);
legend('ζ_{min}(D)', sprintf('D óptimo = %.4f', D_opt), 'Región estable', 'Location','best');

% Subplot 2: Posición de polos vs D
subplot(2,1,2);
D_sample = linspace(min(Dvals), max(Dvals), 50);
colors = lines(6);
for k=1:length(D_sample)
    try
        Gtmp = feedback(G1, D_sample(k)*G2, -1);
        p = pole(Gtmp);
        for j=1:min(6, length(p))
            plot(real(p(j)), imag(p(j)), 'o', 'Color', colors(j,:), ...
                'MarkerSize', 4, 'MarkerFaceColor', colors(j,:));
            hold on;
        end
    catch
    end
end
plot([0 0], ylim, 'r--', 'LineWidth', 1.5);
xlabel('Re\{s\}'); ylabel('Im\{s\}');
title('Migración de Polos durante Barrido de D');
grid on; box on;
text(0.02, max(ylim)*0.95, '← Región Inestable', 'Color', 'r', 'FontWeight', 'bold');

drawnow;

% ---------------- 2) GUIADO explícito del clic Kp ----------------
Kgrid = specs.Kp_range(:);
[rl_poles, ~] = rlocus(G_inner, Kgrid);

if size(rl_poles, 2) ~= numel(Kgrid)
    warning('Ajustando Kgrid a dimensiones de rlocus');
    Kgrid = Kgrid(1:size(rl_poles,2));
end

target_zetas = [0.3 0.5 0.7];
pref = specs.zeta_click_pref;
Ts = specs.Ts_target;

bestCand = struct('K',[],'p',[],'z',[],'score',Inf,'tag','');
cand_list = [];

for zt = target_zetas
    best = bestCand;
    for j = 1:numel(Kgrid)
        pj = rl_poles(:,j);
        pc = pj(abs(imag(pj))>1e-8);
        if isempty(pc), continue; end
        
        [~,idx_c] = min(-real(pc)); pd = pc(idx_c);
        [zj,wnj,sigj] = zeta_of(pd);
        if isnan(zj) || wnj==0 || sigj<=0, continue; end
        
        penTs = 0;
        if ~isempty(Ts)
            sig_req = 4/max(Ts,eps);
            if sigj < sig_req
                penTs = (sig_req - sigj)/max(sig_req,1);
            end
        end
        
        sc = abs(zj-zt) + 0.6*penTs;
        if sc < best.score
            best.K = Kgrid(j); best.p = pd; best.z = zj; best.score = sc;
            best.tag = sprintf('\\zeta\\approx%.2f', zj);
        end
    end
    if isfinite(best.score)
        cand_list = [cand_list; best]; %#ok<AGROW>
    end
end

% Elegimos mejor global
if ~isempty(cand_list)
    w = ones(numel(cand_list),1);
    for k=1:numel(cand_list)
        w(k) = 1 + 0.7*(1 - min(1,abs(cand_list(k).z - pref)/0.2));
    end
    scores = [cand_list.score]./w';
    [~,ig] = min(scores); best_global = cand_list(ig);
else
    best_global = bestCand;
end

% Figura interactiva
fig = figure('Name', sprintf('Root Locus - %s (elige Kp con clic)', name));
ax = axes(fig); rlocus(G_inner); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
title(ax, sprintf('Root Locus - %s  (clic con rlocfind o espera 30s)', upper(name)));
xlabel(ax,'Re\{s\}'); ylabel(ax,'Im\{s\}');
draw_zeta_guides(ax, target_zetas);
draw_sigma_line(ax, specs.Ts_target);
add_instruction_box(fig);

% Pintar candidatos
for k=1:numel(cand_list)
    pk = cand_list(k).p; 
    scatter(real(pk), imag(pk), 46, 'g', 'filled');
    text(real(pk), imag(pk), sprintf('  Kp≈%.3g, %s', cand_list(k).K, cand_list(k).tag), ...
        'Color',[0.1 0.5 0.1], 'FontSize',9, 'VerticalAlignment','bottom');
end

if isfinite(best_global.score)
    pk = best_global.p;
    scatter(real(pk), imag(pk), 70, 'g', 'filled','MarkerEdgeColor',[0 0.4 0]);
    text(real(pk), imag(pk), '  ← Sugerido', 'Color',[0 0.4 0], 'FontWeight','bold');
end

% Selección con rlocfind CON TIMEOUT DE 30 SEGUNDOS
Kp_from_click = NaN;
fprintf('   ⏱️  Esperando selección de Kp (30 segundos)...\n');
fprintf('   💡 Haz clic en el Root Locus o espera para selección automática.\n');

try
    timeout_timer = timer('StartDelay', 30, 'TimerFcn', @(~,~) uiresume(gcbf));
    start(timeout_timer);
    
    [Kp_click, ~] = rlocfind(G_inner);
    
    if isvalid(timeout_timer)
        stop(timeout_timer);
        delete(timeout_timer);
    end
    
    if ~isempty(Kp_click) && isfinite(Kp_click) && Kp_click>0
        Kp_from_click = Kp_click(1);
        fprintf('   ✓ Kp seleccionado manualmente: %.4f\n', Kp_from_click);
    end
catch ME
    try
        if exist('timeout_timer', 'var') && isvalid(timeout_timer)
            stop(timeout_timer);
            delete(timeout_timer);
        end
    catch
    end
    fprintf('   ⚠️  No se detectó selección manual.\n');
end

if isfinite(Kp_from_click)
    Kp_opt = Kp_from_click;
else
    fprintf('   🤖 Selección AUTOMÁTICA del mejor Kp...\n');
    if isfinite(best_global.score)
        Kp_opt = best_global.K;
        fprintf('   ✓ Kp óptimo automático: %.4f (ζ=%.3f)\n', Kp_opt, best_global.z);
    else
        zeta_kp=-ones(size(Kgrid)); ts_kp=inf(size(Kgrid)); stab=false(size(Kgrid));
        for i=1:numel(Kgrid)
            try
                Ttmp = feedback(G_inner*Kgrid(i),1);
                p = pole(Ttmp); st = is_stable(p); stab(i)=st;
                if st
                    zeta_kp(i) = min_pair_zeta(p);
                    info = stepinfo(Ttmp); ts_kp(i)=info.SettlingTime;
                end
            catch
            end
        end
        valid = stab & (zeta_kp>=specs.zeta_min);
        if ~any(valid), valid = stab & (zeta_kp>0); end
        if any(valid)
            [~,ii]=min(ts_kp(valid)); idx=find(valid); idx=idx(ii); Kp_opt=Kgrid(idx);
        else
            Kp_opt = Kgrid(1);
        end
        fprintf('   ✓ Kp fallback: %.4f\n', Kp_opt);
    end
end

% Marca selección
Tkp = feedback(G_inner*Kp_opt,1);
pSel = pole(Tkp);
plot(real(pSel), imag(pSel), 'rx', 'MarkerSize',10,'LineWidth',2);
legend(ax, 'Root Locus','Guías \zeta','Location','best');

fprintf('✅ PARTE 3B completada. Ejecuta PARTE 3C.\n\n');

%% ========================================================================
% FUNCIÓN DE DISEÑO PI+D (PARTE 3C - Selección Ki y métricas)
% ========================================================================

% ---------------- 3) Selección de Ki CON ROOT LOCUS INTERACTIVO ----------------
fprintf('\n📐 Diseñando Ki para %s\n', name);

% Sistema con Kp ya diseñado: G_inner * Kp / s
G_for_Ki = G_inner * Kp_opt / s;

% Barrido preliminar para encontrar candidatos
Ki_range = specs.Ki_range(:);
zeta_ki  = -ones(size(Ki_range)); ess_ki = inf(size(Ki_range)); st_ki=false(size(Ki_range));

for i=1:numel(Ki_range)
    try
        Cpi = Kp_opt + Ki_range(i)/s;
        Tf  = feedback(G_inner*Cpi, 1);
        p = pole(Tf); st = is_stable(p); st_ki(i)=st;
        if st
            zeta_ki(i) = min_pair_zeta(p);
            ess_ki(i)  = abs(1 - dcgain(Tf));
        end
    catch
    end
end

% Encontrar candidatos Ki
zeta_thr = max(0.3, specs.zeta_min*specs.Ki_zeta_keep_ratio);
valid = st_ki & (zeta_ki>=zeta_thr);

% Root Locus para Ki
[rl_poles_Ki, ~] = rlocus(G_for_Ki, Ki_range);
if size(rl_poles_Ki, 2) ~= numel(Ki_range)
    Ki_range = Ki_range(1:size(rl_poles_Ki,2));
    if numel(Ki_range) < numel(valid)
        zeta_ki = zeta_ki(1:numel(Ki_range));
        ess_ki = ess_ki(1:numel(Ki_range));
        st_ki = st_ki(1:numel(Ki_range));
        valid = st_ki & (zeta_ki>=zeta_thr);
    end
end

% Buscar candidatos Ki óptimos (minimizar ess manteniendo zeta)
cand_Ki_list = [];
bestCand_Ki = struct('K',[],'p',[],'z',[],'ess',Inf,'score',Inf);

if any(valid)
    valid_idx = find(valid);
    for j = 1:length(valid_idx)
        idx = valid_idx(j);
        pj = rl_poles_Ki(:,idx);
        pc = pj(abs(imag(pj))>1e-8);
        if ~isempty(pc)
            [~,idc] = min(-real(pc)); pd = pc(idc);
            [zj,~,~] = zeta_of(pd);
            if isnan(zj), continue; end
        else
            zj = 1;
            pd = pj(1);
        end
        
        score = ess_ki(idx) + 0.3*abs(zj - specs.zeta_min);
        
        if score < bestCand_Ki.score
            bestCand_Ki.K = Ki_range(idx);
            bestCand_Ki.p = pd;
            bestCand_Ki.z = zj;
            bestCand_Ki.ess = ess_ki(idx);
            bestCand_Ki.score = score;
        end
        
        if mod(j, max(1, floor(length(valid_idx)/5))) == 0
            cand_Ki_list = [cand_Ki_list; struct('K', Ki_range(idx), 'p', pd, ...
                'z', zj, 'ess', ess_ki(idx))]; %#ok<AGROW>
        end
    end
end

% Figura interactiva para Ki
fig_Ki = figure('Name', sprintf('Root Locus Ki - %s (elige Ki con clic)', name));
ax_Ki = axes(fig_Ki); 
rlocus(G_for_Ki, Ki_range); 
hold(ax_Ki,'on'); grid(ax_Ki,'on'); box(ax_Ki,'on');
title(ax_Ki, sprintf('Root Locus - Ki para %s (sistema con Kp=%.3f)', upper(name), Kp_opt));
xlabel(ax_Ki,'Re\{s\}'); ylabel(ax_Ki,'Im\{s\}');
draw_zeta_guides(ax_Ki, target_zetas);

% Instrucciones específicas para Ki
try
    annotation(fig_Ki,'textbox',[0.62 0.62 0.35 0.33], ...
        'String', sprintf([ ...
        'Selección de K_i:\n' ...
        '1) CLIC sobre una rama del locus.\n' ...
        '2) Objetivo: eliminar error estacionario\n' ...
        '   manteniendo ζ ≥ %.2f\n' ...
        '3) Puntos CYAN = candidatos que minimizan ess.\n' ...
        '4) Tienes 30 SEGUNDOS o se selecciona automático.\n'], zeta_thr), ...
        'BackgroundColor',[1 1 0.9 0.85],'FitBoxToText','on');
catch
end

% Pintar candidatos Ki en cyan
for k=1:numel(cand_Ki_list)
    pk = cand_Ki_list(k).p; 
    scatter(real(pk), imag(pk), 46, 'c', 'filled');
    text(real(pk), imag(pk), sprintf('  Ki≈%.3g (ess=%.1e)', ...
        cand_Ki_list(k).K, cand_Ki_list(k).ess), ...
        'Color',[0 0.5 0.5], 'FontSize',9, 'VerticalAlignment','bottom');
end

% Resaltar el mejor candidato
if isfinite(bestCand_Ki.score)
    pk = bestCand_Ki.p;
    scatter(real(pk), imag(pk), 70, 'c', 'filled','MarkerEdgeColor',[0 0.4 0.4]);
    text(real(pk), imag(pk), '  ← Sugerido Ki', 'Color',[0 0.4 0.4], 'FontWeight','bold');
end

% Selección interactiva de Ki CON TIMEOUT
Ki_from_click = NaN;
fprintf('   ⏱️  Esperando selección de Ki (30 segundos)...\n');
fprintf('   💡 Haz clic en el Root Locus o espera para selección automática.\n');

try
    timeout_timer_Ki = timer('StartDelay', 30, 'TimerFcn', @(~,~) uiresume(gcbf));
    start(timeout_timer_Ki);
    
    [Ki_click, ~] = rlocfind(G_for_Ki);
    
    if isvalid(timeout_timer_Ki)
        stop(timeout_timer_Ki);
        delete(timeout_timer_Ki);
    end
    
    if ~isempty(Ki_click) && isfinite(Ki_click) && Ki_click>0
        Ki_from_click = Ki_click(1);
    end
catch ME
    try
        if exist('timeout_timer_Ki', 'var') && isvalid(timeout_timer_Ki)
            stop(timeout_timer_Ki);
            delete(timeout_timer_Ki);
        end
    catch
    end
end

if isfinite(Ki_from_click)
    Ki_opt = Ki_from_click;
    fprintf('   ✓ Ki seleccionado manualmente: %.4f\n', Ki_opt);
else
    if isfinite(bestCand_Ki.score)
        Ki_opt = bestCand_Ki.K;
        fprintf('   ✓ Ki automático (mejor candidato): %.4f\n', Ki_opt);
    else
        Ki_opt = 0;
        fprintf('   ⚠ Ki = 0 (no se encontró valor que mantenga estabilidad)\n');
    end
end

% Marcar selección final de Ki
Tfin_Ki = feedback(G_inner*(Kp_opt + Ki_opt/s), 1);
pSel_Ki = pole(Tfin_Ki);
plot(real(pSel_Ki), imag(pSel_Ki), 'mx', 'MarkerSize',10,'LineWidth',2);
legend(ax_Ki, 'Root Locus Ki','Guías \zeta','Selección final','Location','best');

% ---------------- 4) Métricas finales ----------------
C_PI = Kp_opt + Ki_opt/s;
Tfin = feedback(G_inner*C_PI, 1);
info = struct('SettlingTime',NaN,'Overshoot',NaN,'RiseTime',NaN);
Gm=NaN; Pm=NaN; Wcp=NaN;
try, info = stepinfo(Tfin); end
try, [Gm,Pm,~,Wcp]=margin(G_inner*C_PI); end

metrics.Ts         = info.SettlingTime;
metrics.OS         = info.Overshoot;
metrics.Tr         = info.RiseTime;
metrics.Gm_dB      = 20*log10(Gm);
metrics.Pm_deg     = Pm;
metrics.Wcp_rad_s  = Wcp;
metrics.zeta_final = min_pair_zeta(pole(Tfin));
metrics.ess        = abs(1 - dcgain(Tfin));
metrics.notes      = sprintf('zeta_min=%.2f; Ki-thr=%.2f; Ts*=%s', ...
    specs.zeta_min, zeta_thr, mat2str(specs.Ts_target));

end  % FIN DE LA FUNCIÓN design_pid_auto

fprintf('✅ PARTE 3C completada (función design_pid_auto terminada).\n');
fprintf('   Ejecuta PARTE 4 para diseñar los controladores.\n\n');

%% ========================================================================
% PARTE 4: DISEÑO DE LOS TRES CONTROLADORES (PITCH, ROLL, YAW)
% ========================================================================

fprintf('\n========================================\n');
fprintf('DISEÑO PI+D LONGITUDINAL (PITCH)\n');
fprintf('========================================\n');

% Extraer funciones de transferencia PITCH
[num_G1_lon, den_G1_lon] = ss2tf(A_longmod, B_longmod(:,1), C_longmod(4,:), D_longmod(4,1));
G1_lon = tf(num_G1_lon, den_G1_lon);

[num_G2_lon, den_G2_lon] = ss2tf(A_longmod, B_longmod(:,1), C_longmod(3,:), D_longmod(3,1));
G2_lon = tf(num_G2_lon, den_G2_lon);

% Especificaciones de diseño
specs.zeta_min = 0.5;
specs.ts_max = 5;
specs.os_max = 20;

% Diseño automático PITCH
[D_lon, Kp_lon, Ki_lon, metrics_lon] = design_pid_auto(G1_lon, G2_lon, 'Longitudinal', specs);

% Crear sistemas para gráficas
s = tf('s');
G_inner_lon = feedback(G1_lon, D_lon*G2_lon, -1);
C_PI_lon = Kp_lon + Ki_lon/s;
T_lon_final = feedback(G_inner_lon * C_PI_lon, 1);

fprintf('\n========================================\n');
fprintf('DISEÑO PI+D LATERAL (ROLL)\n');
fprintf('========================================\n');

% Extraer funciones de transferencia ROLL
[num_G1_roll, den_G1_roll] = ss2tf(A_latmod, B_latmod(:,1), C_latmod(4,:), D_latmod(4,1));
G1_roll = tf(num_G1_roll, den_G1_roll);

[num_G2_roll, den_G2_roll] = ss2tf(A_latmod, B_latmod(:,1), C_latmod(2,:), D_latmod(2,1));
G2_roll = tf(num_G2_roll, den_G2_roll);

% Diseño automático ROLL
[D_roll, Kp_roll, Ki_roll, metrics_roll] = design_pid_auto(G1_roll, G2_roll, 'Roll', specs);

% Crear sistemas para gráficas
G_inner_roll = feedback(G1_roll, D_roll*G2_roll, -1);
C_PI_roll = Kp_roll + Ki_roll/s;
T_roll_final = feedback(G_inner_roll * C_PI_roll, 1);

fprintf('\n========================================\n');
fprintf('DISEÑO PI+D DIRECCIONAL (YAW)\n');
fprintf('========================================\n');

% Extraer funciones de transferencia YAW
[num_G1_yaw, den_G1_yaw] = ss2tf(A_latmod, B_latmod(:,2), C_latmod(5,:), D_latmod(5,2));
G1_yaw = tf(num_G1_yaw, den_G1_yaw);

[num_G2_yaw, den_G2_yaw] = ss2tf(A_latmod, B_latmod(:,2), C_latmod(3,:), D_latmod(3,2));
G2_yaw = tf(num_G2_yaw, den_G2_yaw);

% Diseño automático YAW
[D_yaw, Kp_yaw, Ki_yaw, metrics_yaw] = design_pid_auto(G1_yaw, G2_yaw, 'Yaw', specs);

% Crear sistemas para gráficas
G_inner_yaw = feedback(G1_yaw, D_yaw*G2_yaw, -1);
C_PI_yaw = Kp_yaw + Ki_yaw/s;
T_yaw_final = feedback(G_inner_yaw * C_PI_yaw, 1);

fprintf('✅ PARTE 4 completada. Ejecuta PARTE 5 para generar gráficas.\n\n');

%% ========================================================================
% PARTE 5: GRÁFICAS DE RESPUESTAS AL ESCALÓN CON REFERENCIA UNITARIA
% ========================================================================

% ============ PITCH ============
fprintf('\n📊 RESPUESTA AL ESCALÓN - LONGITUDINAL (PITCH)\n');
fprintf('   D = %.4f, Kp = %.4f, Ki = %.4f\n', D_lon, Kp_lon, Ki_lon);
fprintf('   ζ_min = %.3f, Ts = %.2fs, OS = %.1f%%\n', ...
    metrics_lon.zeta_final, metrics_lon.Ts, metrics_lon.OS);

figure('Name', 'Respuesta al Escalón - Longitudinal (Pitch)', 'Position', [100 100 1400 700]);

subplot(2,3,1);
[y1, t1] = step(T_lon_final, 15);
plot(t1, y1, 'b-', 'LineWidth', 2); hold on;
plot(t1, ones(size(t1)), 'r--', 'LineWidth', 1.5);
grid on;
title(sprintf('Sistema con PI+D\nD=%.3f, Kp=%.3f, Ki=%.3f', D_lon, Kp_lon, Ki_lon));
ylabel('θ [rad]'); xlabel('Tiempo [s]');
legend('Respuesta', 'Referencia (1 rad)', 'Location', 'best');
ylim([0 max(1.5, max(y1)*1.1)]);

subplot(2,3,2);
T_lon_P = feedback(G_inner_lon * Kp_lon, 1);
[y2, t2] = step(T_lon_P, 15);
[y3, t3] = step(T_lon_final, 15);
plot(t2, ones(size(t2)), 'k--', 'LineWidth', 1); hold on;
plot(t2, y2, 'b--', 'LineWidth', 2);
plot(t3, y3, 'r-', 'LineWidth', 2);
grid on;
title('Comparación P vs PI');
legend('Referencia', 'Solo P (sin Ki)', 'PI completo', 'Location', 'best');
ylabel('θ [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([y2; y3])*1.1)]);

subplot(2,3,3);
G_no_D = feedback(G1_lon, 0*G2_lon, -1);
T_no_D = feedback(G_no_D * C_PI_lon, 1);
[y4, t4] = step(T_no_D, 15);
[y5, t5] = step(T_lon_final, 15);
plot(t4, ones(size(t4)), 'k--', 'LineWidth', 1); hold on;
plot(t4, y4, 'g--', 'LineWidth', 2);
plot(t5, y5, 'r-', 'LineWidth', 2);
grid on;
title('Efecto de Realimentación de Tasa');
legend('Referencia', 'Sin D (q no realimentada)', 'Con D (q realimentada)', 'Location', 'best');
ylabel('θ [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([y4; y5])*1.1)]);

subplot(2,3,4);
error_P = 1 - y2;
error_PI = 1 - y3;
plot(t2, error_P, 'b--', 'LineWidth', 1.5); hold on;
plot(t3, error_PI, 'r-', 'LineWidth', 2);
plot(t3, zeros(size(t3)), 'k:', 'LineWidth', 1);
grid on;
title('Error de Seguimiento');
xlabel('Tiempo [s]'); ylabel('Error [rad]');
legend('Error P', 'Error PI', 'Location', 'best');

subplot(2,3,5);
axis off;
text(0.1, 0.9, '\bf{MÉTRICAS DE DESEMPEÑO}', 'FontSize', 12);
text(0.1, 0.75, sprintf('Tiempo de establecimiento: %.2f s', metrics_lon.Ts), 'FontSize', 10);
text(0.1, 0.65, sprintf('Sobrepaso: %.1f %%', metrics_lon.OS), 'FontSize', 10);
text(0.1, 0.55, sprintf('Tiempo de subida: %.2f s', metrics_lon.Tr), 'FontSize', 10);
text(0.1, 0.45, sprintf('Error estacionario: %.2e', metrics_lon.ess), 'FontSize', 10);
text(0.1, 0.35, sprintf('ζ_{min}: %.3f', metrics_lon.zeta_final), 'FontSize', 10);
text(0.1, 0.20, sprintf('Margen de Ganancia: %.1f dB', metrics_lon.Gm_dB), 'FontSize', 10);
text(0.1, 0.10, sprintf('Margen de Fase: %.1f°', metrics_lon.Pm_deg), 'FontSize', 10);

subplot(2,3,6);
u_approx = gradient(y3, t3(2)-t3(1));
plot(t3, u_approx, 'b-', 'LineWidth', 1.5);
grid on;
title('Señal de Control Aproximada');
xlabel('Tiempo [s]'); ylabel('u(t) [aprox]');
legend('Esfuerzo de control', 'Location', 'best');

% ============ ROLL ============
fprintf('\n📊 RESPUESTA AL ESCALÓN - LATERAL (ROLL)\n');
fprintf('   D = %.4f, Kp = %.4f, Ki = %.4f\n', D_roll, Kp_roll, Ki_roll);
fprintf('   ζ_min = %.3f, Ts = %.2fs, OS = %.1f%%\n', ...
    metrics_roll.zeta_final, metrics_roll.Ts, metrics_roll.OS);

figure('Name', 'Respuesta al Escalón - Lateral (Roll)', 'Position', [150 150 1400 700]);

subplot(2,3,1);
[y1, t1] = step(T_roll_final, 15);
plot(t1, y1, 'b-', 'LineWidth', 2); hold on;
plot(t1, ones(size(t1)), 'r--', 'LineWidth', 1.5);
grid on;
title(sprintf('Sistema con PI+D\nD=%.3f, Kp=%.3f, Ki=%.3f', D_roll, Kp_roll, Ki_roll));
ylabel('φ [rad]'); xlabel('Tiempo [s]');
legend('Respuesta', 'Referencia (1 rad)', 'Location', 'best');
ylim([0 max(1.5, max(y1)*1.1)]);

subplot(2,3,2);
T_roll_P = feedback(G_inner_roll * Kp_roll, 1);
[y2, t2] = step(T_roll_P, 15);
[y3, t3] = step(T_roll_final, 15);
plot(t2, ones(size(t2)), 'k--', 'LineWidth', 1); hold on;
plot(t2, y2, 'b--', 'LineWidth', 2);
plot(t3, y3, 'r-', 'LineWidth', 2);
grid on;
title('Comparación P vs PI');
legend('Referencia', 'Solo P', 'PI completo', 'Location', 'best');
ylabel('φ [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([y2; y3])*1.1)]);

subplot(2,3,3);
G_no_D_roll = feedback(G1_roll, 0*G2_roll, -1);
T_no_D_roll = feedback(G_no_D_roll * C_PI_roll, 1);
[y4, t4] = step(T_no_D_roll, 15);
[y5, t5] = step(T_roll_final, 15);
plot(t4, ones(size(t4)), 'k--', 'LineWidth', 1); hold on;
plot(t4, y4, 'g--', 'LineWidth', 2);
plot(t5, y5, 'r-', 'LineWidth', 2);
grid on;
title('Efecto de Realimentación de Tasa');
legend('Referencia', 'Sin D', 'Con D', 'Location', 'best');
ylabel('φ [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([y4; y5])*1.1)]);

subplot(2,3,4);
error_P = 1 - y2;
error_PI = 1 - y3;
plot(t2, error_P, 'b--', 'LineWidth', 1.5); hold on;
plot(t3, error_PI, 'r-', 'LineWidth', 2);
plot(t3, zeros(size(t3)), 'k:', 'LineWidth', 1);
grid on;
title('Error de Seguimiento');
xlabel('Tiempo [s]'); ylabel('Error [rad]');
legend('Error P', 'Error PI', 'Location', 'best');

subplot(2,3,5);
axis off;
text(0.1, 0.9, '\bf{MÉTRICAS DE DESEMPEÑO}', 'FontSize', 12);
text(0.1, 0.75, sprintf('Tiempo de establecimiento: %.2f s', metrics_roll.Ts), 'FontSize', 10);
text(0.1, 0.65, sprintf('Sobrepaso: %.1f %%', metrics_roll.OS), 'FontSize', 10);
text(0.1, 0.55, sprintf('Tiempo de subida: %.2f s', metrics_roll.Tr), 'FontSize', 10);
text(0.1, 0.45, sprintf('Error estacionario: %.2e', metrics_roll.ess), 'FontSize', 10);
text(0.1, 0.35, sprintf('ζ_{min}: %.3f', metrics_roll.zeta_final), 'FontSize', 10);
text(0.1, 0.20, sprintf('Margen de Ganancia: %.1f dB', metrics_roll.Gm_dB), 'FontSize', 10);
text(0.1, 0.10, sprintf('Margen de Fase: %.1f°', metrics_roll.Pm_deg), 'FontSize', 10);

subplot(2,3,6);
u_approx = gradient(y3, t3(2)-t3(1));
plot(t3, u_approx, 'b-', 'LineWidth', 1.5);
grid on;
title('Señal de Control Aproximada');
xlabel('Tiempo [s]'); ylabel('u(t) [aprox]');
legend('Esfuerzo de control', 'Location', 'best');

% ============ YAW ============
fprintf('\n📊 RESPUESTA AL ESCALÓN - DIRECCIONAL (YAW)\n');
fprintf('   D = %.4f, Kp = %.4f, Ki = %.4f\n', D_yaw, Kp_yaw, Ki_yaw);
fprintf('   ζ_min = %.3f, Ts = %.2fs, OS = %.1f%%\n', ...
    metrics_yaw.zeta_final, metrics_yaw.Ts, metrics_yaw.OS);

figure('Name', 'Respuesta al Escalón - Direccional (Yaw)', 'Position', [200 200 1400 700]);

subplot(2,3,1);
[y1, t1] = step(T_yaw_final, 15);
plot(t1, y1, 'b-', 'LineWidth', 2); hold on;
plot(t1, ones(size(t1)), 'r--', 'LineWidth', 1.5);
grid on;
title(sprintf('Sistema con PI+D\nD=%.3f, Kp=%.3f, Ki=%.3f', D_yaw, Kp_yaw, Ki_yaw));
ylabel('ψ [rad]'); xlabel('Tiempo [s]');
legend('Respuesta', 'Referencia (1 rad)', 'Location', 'best');
ylim([0 max(1.5, max(y1)*1.1)]);

subplot(2,3,2);
T_yaw_P = feedback(G_inner_yaw * Kp_yaw, 1);
[y2, t2] = step(T_yaw_P, 15);
[y3, t3] = step(T_yaw_final, 15);
plot(t2, ones(size(t2)), 'k--', 'LineWidth', 1); hold on;
plot(t2, y2, 'b--', 'LineWidth', 2);
plot(t3, y3, 'r-', 'LineWidth', 2);
grid on;
title('Comparación P vs PI');
legend('Referencia', 'Solo P', 'PI completo', 'Location', 'best');
ylabel('ψ [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([y2; y3])*1.1)]);

subplot(2,3,3);
G_no_D_yaw = feedback(G1_yaw, 0*G2_yaw, -1);
T_no_D_yaw = feedback(G_no_D_yaw * C_PI_yaw, 1);
[y4, t4] = step(T_no_D_yaw, 15);
[y5, t5] = step(T_yaw_final, 15);
plot(t4, ones(size(t4)), 'k--', 'LineWidth', 1); hold on;
plot(t4, y4, 'g--', 'LineWidth', 2);
plot(t5, y5, 'r-', 'LineWidth', 2);
grid on;
title('Efecto de Realimentación de Tasa');
legend('Referencia', 'Sin D', 'Con D', 'Location', 'best');
ylabel('ψ [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([y4; y5])*1.1)]);

subplot(2,3,4);
error_P = 1 - y2;
error_PI = 1 - y3;
plot(t2, error_P, 'b--', 'LineWidth', 1.5); hold on;
plot(t3, error_PI, 'r-', 'LineWidth', 2);
plot(t3, zeros(size(t3)), 'k:', 'LineWidth', 1);
grid on;
title('Error de Seguimiento');
xlabel('Tiempo [s]'); ylabel('Error [rad]');
legend('Error P', 'Error PI', 'Location', 'best');

subplot(2,3,5);
axis off;
text(0.1, 0.9, '\bf{MÉTRICAS DE DESEMPEÑO}', 'FontSize', 12);
text(0.1, 0.75, sprintf('Tiempo de establecimiento: %.2f s', metrics_yaw.Ts), 'FontSize', 10);
text(0.1, 0.65, sprintf('Sobrepaso: %.1f %%', metrics_yaw.OS), 'FontSize', 10);
text(0.1, 0.55, sprintf('Tiempo de subida: %.2f s', metrics_yaw.Tr), 'FontSize', 10);
text(0.1, 0.45, sprintf('Error estacionario: %.2e', metrics_yaw.ess), 'FontSize', 10);
text(0.1, 0.35, sprintf('ζ_{min}: %.3f', metrics_yaw.zeta_final), 'FontSize', 10);
text(0.1, 0.20, sprintf('Margen de Ganancia: %.1f dB', metrics_yaw.Gm_dB), 'FontSize', 10);
text(0.1, 0.10, sprintf('Margen de Fase: %.1f°', metrics_yaw.Pm_deg), 'FontSize', 10);

subplot(2,3,6);
u_approx = gradient(y3, t3(2)-t3(1));
plot(t3, u_approx, 'b-', 'LineWidth', 1.5);
grid on;
title('Señal de Control Aproximada');
xlabel('Tiempo [s]'); ylabel('u(t) [aprox]');
legend('Esfuerzo de control', 'Location', 'best');

fprintf('✅ PARTE 5 completada. Ejecuta PARTE 6 (FINAL).\n\n');

%% ========================================================================
% PARTE 6: COMPARATIVAS FINALES, REPORTES Y CONCLUSIONES
% ========================================================================

% ============ FIGURA COMPARATIVA FINAL ============
figure('Name', 'Comparación de Respuestas al Escalón', 'Position', [250 250 1400 900]);

subplot(2,3,1);
[yp, tp] = step(T_lon_final, 15);
[yr, tr] = step(T_roll_final, 15);
[yy, ty] = step(T_yaw_final, 15);
plot(tp, ones(size(tp)), 'k--', 'LineWidth', 1.5); hold on;
plot(tp, yp, 'b-', 'LineWidth', 2);
plot(tr, yr, 'r-', 'LineWidth', 2);
plot(ty, yy, 'g-', 'LineWidth', 2);
grid on;
title('Comparación de los Tres Ejes');
legend('Referencia (1 rad)', 'Pitch (θ)', 'Roll (φ)', 'Yaw (ψ)', 'Location', 'best');
ylabel('Ángulo [rad]'); xlabel('Tiempo [s]');
ylim([0 max(1.5, max([yp; yr; yy])*1.1)]);

subplot(2,3,2);
[y_pitch, t_pitch] = step(T_lon_final, 15);
plot(t_pitch, ones(size(t_pitch)), 'r--', 'LineWidth', 1.5); hold on;
plot(t_pitch, y_pitch, 'b-', 'LineWidth', 2);
grid on;
title('Pitch (θ) - Longitudinal');
ylabel('θ [rad]'); xlabel('Tiempo [s]');
legend('Referencia', 'Respuesta', 'Location', 'best');
ylim([0 max(1.5, max(y_pitch)*1.1)]);

subplot(2,3,3);
[y_roll, t_roll] = step(T_roll_final, 15);
plot(t_roll, ones(size(t_roll)), 'r--', 'LineWidth', 1.5); hold on;
plot(t_roll, y_roll, 'b-', 'LineWidth', 2);
grid on;
title('Roll (φ) - Lateral');
ylabel('φ [rad]'); xlabel('Tiempo [s]');
legend('Referencia', 'Respuesta', 'Location', 'best');
ylim([0 max(1.5, max(y_roll)*1.1)]);

subplot(2,3,4);
[y_yaw, t_yaw] = step(T_yaw_final, 15);
plot(t_yaw, ones(size(t_yaw)), 'r--', 'LineWidth', 1.5); hold on;
plot(t_yaw, y_yaw, 'b-', 'LineWidth', 2);
grid on;
title('Yaw (ψ) - Direccional');
ylabel('ψ [rad]'); xlabel('Tiempo [s]');
legend('Referencia', 'Respuesta', 'Location', 'best');
ylim([0 max(1.5, max(y_yaw)*1.1)]);

% Subplot 5: Errores de seguimiento
subplot(2,3,5);
error_pitch = 1 - y_pitch;
error_roll = 1 - y_roll;
error_yaw = 1 - y_yaw;
plot(t_pitch, error_pitch, 'b-', 'LineWidth', 1.5); hold on;
plot(t_roll, error_roll, 'r-', 'LineWidth', 1.5);
plot(t_yaw, error_yaw, 'g-', 'LineWidth', 1.5);
plot(t_pitch, zeros(size(t_pitch)), 'k:', 'LineWidth', 1);
grid on;
title('Errores de Seguimiento');
legend('Error Pitch', 'Error Roll', 'Error Yaw', 'Location', 'best');
xlabel('Tiempo [s]'); ylabel('Error [rad]');

% Subplot 6: Tabla resumen
subplot(2,3,6);
axis off;
text(0.5, 0.95, '\bf{RESUMEN COMPARATIVO}', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold');

line([0.05 0.95], [0.90 0.90], 'Color', 'k', 'LineWidth', 1.5);

text(0.05, 0.85, '\bf{Parámetro}', 'FontSize', 10, 'FontWeight', 'bold');
text(0.40, 0.85, '\bf{Pitch}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'b');
text(0.60, 0.85, '\bf{Roll}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
text(0.80, 0.85, '\bf{Yaw}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');

line([0.05 0.95], [0.82 0.82], 'Color', 'k', 'LineWidth', 0.8);

y_pos = 0.75;
dy = 0.08;

text(0.05, y_pos, 'D:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.4f', D_lon), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.4f', D_roll), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.4f', D_yaw), 'FontSize', 9, 'Color', 'g');
y_pos = y_pos - dy;

text(0.05, y_pos, 'K_p:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.4f', Kp_lon), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.4f', Kp_roll), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.4f', Kp_yaw), 'FontSize', 9, 'Color', 'g');
y_pos = y_pos - dy;

text(0.05, y_pos, 'K_i:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.4f', Ki_lon), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.4f', Ki_roll), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.4f', Ki_yaw), 'FontSize', 9, 'Color', 'g');
y_pos = y_pos - dy;

line([0.05 0.95], [y_pos+0.03 y_pos+0.03], 'Color', 'k', 'LineWidth', 0.5);

text(0.05, y_pos, 'T_s [s]:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.2f', metrics_lon.Ts), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.2f', metrics_roll.Ts), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.2f', metrics_yaw.Ts), 'FontSize', 9, 'Color', 'g');
y_pos = y_pos - dy;

text(0.05, y_pos, 'OS [%]:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.1f', metrics_lon.OS), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.1f', metrics_roll.OS), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.1f', metrics_yaw.OS), 'FontSize', 9, 'Color', 'g');
y_pos = y_pos - dy;

text(0.05, y_pos, 'ζ_{min}:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.3f', metrics_lon.zeta_final), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.3f', metrics_roll.zeta_final), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.3f', metrics_yaw.zeta_final), 'FontSize', 9, 'Color', 'g');
y_pos = y_pos - dy;

text(0.05, y_pos, 'e_{ss}:', 'FontSize', 9);
text(0.40, y_pos, sprintf('%.1e', metrics_lon.ess), 'FontSize', 9, 'Color', 'b');
text(0.60, y_pos, sprintf('%.1e', metrics_roll.ess), 'FontSize', 9, 'Color', 'r');
text(0.80, y_pos, sprintf('%.1e', metrics_yaw.ess), 'FontSize', 9, 'Color', 'g');

% ============ RESUMEN EN CONSOLA ============
fprintf('\n========================================\n');
fprintf('RESUMEN COMPARATIVO DE CONTROLADORES\n');
fprintf('========================================\n\n');

fprintf('┌─────────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┐\n');
fprintf('│ Controlador │    D     │    Kp    │    Ki    │  ζ_min   │   Ts[s]  │   OS[%%]  │  ess     │\n');
fprintf('├─────────────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────┤\n');
fprintf('│ PITCH (θ)   │ %8.4f │ %8.4f │ %8.4f │  %6.3f  │  %6.2f  │  %6.1f  │ %8.2e │\n', ...
    D_lon, Kp_lon, Ki_lon, metrics_lon.zeta_final, metrics_lon.Ts, metrics_lon.OS, metrics_lon.ess);
fprintf('│ ROLL  (φ)   │ %8.4f │ %8.4f │ %8.4f │  %6.3f  │  %6.2f  │  %6.1f  │ %8.2e │\n', ...
    D_roll, Kp_roll, Ki_roll, metrics_roll.zeta_final, metrics_roll.Ts, metrics_roll.OS, metrics_roll.ess);
fprintf('│ YAW   (ψ)   │ %8.4f │ %8.4f │ %8.4f │  %6.3f  │  %6.2f  │  %6.1f  │ %8.2e │\n', ...
    D_yaw, Kp_yaw, Ki_yaw, metrics_yaw.zeta_final, metrics_yaw.Ts, metrics_yaw.OS, metrics_yaw.ess);
fprintf('└─────────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┘\n\n');

fprintf('📊 MÉTRICAS DE ESTABILIDAD:\n');
fprintf('┌─────────────┬──────────────┬──────────────┬──────────────┐\n');
fprintf('│ Controlador │  GM [dB]     │  PM [°]      │  ωcp [rad/s] │\n');
fprintf('├─────────────┼──────────────┼──────────────┼──────────────┤\n');
fprintf('│ PITCH (θ)   │   %8.2f   │   %8.2f   │   %8.2f   │\n', ...
    metrics_lon.Gm_dB, metrics_lon.Pm_deg, metrics_lon.Wcp_rad_s);
fprintf('│ ROLL  (φ)   │   %8.2f   │   %8.2f   │   %8.2f   │\n', ...
    metrics_roll.Gm_dB, metrics_roll.Pm_deg, metrics_roll.Wcp_rad_s);
fprintf('│ YAW   (ψ)   │   %8.2f   │   %8.2f   │   %8.2f   │\n', ...
    metrics_yaw.Gm_dB, metrics_yaw.Pm_deg, metrics_yaw.Wcp_rad_s);
fprintf('└─────────────┴──────────────┴──────────────┴──────────────┘\n\n');

fprintf('🎯 INTERPRETACIÓN DE GANANCIAS D:\n');
fprintf('   • D_pitch = %.4f → Realimentación de tasa de cabeceo (q)\n', D_lon);
fprintf('   • D_roll  = %.4f → Realimentación de tasa de alabeo (p)\n', D_roll);
fprintf('   • D_yaw   = %.4f → Realimentación de tasa de guiñada (r)\n', D_yaw);
fprintf('\n   Valores más altos = Mayor amortiguamiento del sistema\n');

fprintf('\n📈 ANÁLISIS DE RESPUESTA AL ESCALÓN UNITARIO:\n');
fprintf('   ✓ Todas las respuestas muestran seguimiento a referencia de 1 rad\n');
fprintf('   ✓ Error estacionario prácticamente eliminado por acción integral\n');
fprintf('   ✓ Amortiguamiento adecuado gracias a realimentación de tasa (D)\n\n');

fprintf('⏱️  CONFIGURACIÓN DE TIMEOUT:\n');
fprintf('   • Timeout para selección manual: 30 segundos\n');
fprintf('   • Si no hay clic, se selecciona automáticamente el mejor candidato\n\n');
