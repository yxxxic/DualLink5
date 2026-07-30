clc;
clear;
close all;
data = extractExperimentAngles('record_20260529_142637.csv');

angle1 = data.angle1;
angle2 = data.angle2;
ft_pos1 = data.ft_pos1;
ft_pos2 = data.ft_pos2;
t_sec = data.t_sec;

figure('Color', 'w');
hold on;
grid on;
box on;

plot(t_sec, angle1, ...
    'LineWidth', 1.2, ...
    'DisplayName', 'mt6835 angle1 (aligned to ft pos id=1)');

plot(t_sec, angle2, ...
    'LineWidth', 1.2, ...
    'DisplayName', 'mt6835 angle2 (aligned to ft pos id=2)');

plot(t_sec(data.idx_id1), ft_pos1(data.idx_id1), ...
    '-', 'LineWidth', 1.2, ...
    'DisplayName', 'ft pos, id = 1');

plot(t_sec(data.idx_id2), ft_pos2(data.idx_id2), ...
    '-', 'LineWidth', 2, ...
    'DisplayName', 'ft pos, id = 2');

xlabel('Time / s');
ylabel('Angle or Position / deg');
legend('Location', 'best');
hold off;