clc;
clear;
close all;

%% CSV 文件名
% file = 'record_20260511_154920.csv';
% file = 'record_20260511_154959.csv';
% file = 'record_20260511_161646.csv';

% file = 'record_20260513_145333.csv';  % 有效
% file = 'record_20260513_145550.csv';  % 有效
% file = 'record_20260513_145608.csv';  % 有效
%% 读取 CSV，保留原始列名
opts = detectImportOptions(file, 'VariableNamingRule', 'preserve');
T = readtable(file, opts);

%% 时间轴处理
time_raw = T.("timestamp");
if isdatetime(time_raw)
    t = time_raw;
else
    t = datetime(string(time_raw), ...
        'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSS');
end
t_sec = seconds(t - t(1));   % 相对秒数

%% 读取数据
angle1 = T.("mt6835_angle_angle1_deg");
angle2 = T.("mt6835_angle_angle2_deg");

ft_id  = T.("ft_sts_status_id[0]");
ft_pos = T.("ft_sts_status_pos[0]");

% 防止被读成字符串
if ~isnumeric(ft_id)
    ft_id = str2double(string(ft_id));
end
if ~isnumeric(ft_pos)
    ft_pos = str2double(string(ft_pos));
end

%% 1. 角度解缠绕（避免正常绕回被当作尖峰）
angle1_u = rad2deg(unwrap(deg2rad(angle1)));
angle2_u = rad2deg(unwrap(deg2rad(angle2)));

%% 2. 用 Hampel 滤波去除 angle 的剧烈跳动（毛刺）
k = 25;        % 窗口半宽（点数）
nsigma = 3;    % 异常阈值（中位数偏离倍数）

[angle1_f, idx1] = hampel(angle1_u, k, nsigma);
angle1_f(idx1) = NaN;   % 尖峰置 NaN

[angle2_f, idx2] = hampel(angle2_u, k, nsigma);
angle2_f(idx2) = NaN;

% 注意：ft_pos 不去尖峰，直接用原始值 ft_pos

%% 3. 按 ID 区分 pos
idx_id1 = ft_id == 1;
idx_id2 = ft_id == 2;

%% 4. 对齐：将 angle1 对齐到 ft_pos (id=1) 的初始值
N = 20;   % 使用前 N 对同时有效点计算偏移量（中位数）

% 4.1 angle1 对齐 id=1
valid1 = find(idx_id1 & ~isnan(ft_pos) & ~isnan(angle1_f));
valid1 = valid1(1:min(N, numel(valid1)));
if ~isempty(valid1)
    offset1 = median(ft_pos(valid1) - angle1_f(valid1), 'omitnan');
    fprintf('angle1 对齐偏移量 = %.3f deg（基于前 %d 对有效点）\n', offset1, numel(valid1));
else
    error('无法找到 id=1 的同时有效对齐点，请检查数据或调整滤波参数');
end
angle1_aligned = angle1_f + offset1;

% 4.2 angle2 对齐 id=2
valid2 = find(idx_id2 & ~isnan(ft_pos) & ~isnan(angle2_f));
valid2 = valid2(1:min(N, numel(valid2)));
if ~isempty(valid2)
    offset2 = median(ft_pos(valid2) - angle2_f(valid2), 'omitnan');
    fprintf('angle2 对齐偏移量 = %.3f deg（基于前 %d 对有效点）\n', offset2, numel(valid2));
else
    error('无法找到 id=2 的同时有效对齐点，请检查数据或调整滤波参数');
end
angle2_aligned = angle2_f + offset2;

%% 5. 绘图
figure('Color', 'w');
hold on;
grid on;
box on;

plot(t_sec, angle1_aligned, ...
    'LineWidth', 1.2, ...
    'DisplayName', 'mt6835 angle1 (aligned to ft pos id=1)');

plot(t_sec, angle2_aligned, ...
    'LineWidth', 1.2, ...
    'DisplayName', 'mt6835 angle2 (aligned to ft pos id=2)');

plot(t_sec(idx_id1), ft_pos(idx_id1), ...
    '.-', 'LineWidth', 1.1, 'MarkerSize', 8, ...
    'DisplayName', 'ft pos, id = 1 (raw)');

plot(t_sec(idx_id2), ft_pos(idx_id2), ...
    '.-', 'LineWidth', 1.1, 'MarkerSize', 8, ...
    'DisplayName', 'ft pos, id = 2 (raw)');

xlabel('Time / s');
ylabel('Angle or Position / deg');
legend('Location', 'best');
hold off;