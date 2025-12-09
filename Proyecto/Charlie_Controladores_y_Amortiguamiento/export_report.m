% Usa figuras abiertas (figuras 1 y 2) y las exporta a un PDF multipágina
clear; clc;

% Si tienes las figuras en variables, usa exportgraphics directamente.
% Aquí asumimos que ya ejecutaste control_console.m o control_console_live.m
for f = 1:numel(findobj('Type','figure'))
    fig = figure(f);
    exportgraphics(fig, 'Reporte_Control.pdf', 'Append', true, 'ContentType','vector');
end
disp('Reporte_Control.pdf creado.');
