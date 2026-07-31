clc;
clear;
close all;
%% CSV 文件名
% file = 'record_20260514_144252.csv';  % 有效
% file = 'record_20260514_144333.csv';  % 有效
% file = 'record_20260514_144441.csv';  % 抖动明显
% file = 'record_20260514_144522.csv';
% file = 'record_20260514_144802.csv';  % 抖动明显
% file = 'record_20260514_145123.csv';  % 抖动明显
% file = 'record_20260514_152927.csv';  % 有效
% file = 'record_20260514_153903.csv';  % 有效
% file = 'record_20260514_154100.csv';  % 有效

% file = 'record_20260520_164213.csv';  % 有效
% file = 'record_20260520_164514.csv';  % 有效

file = 'record_20260527_143842.csv';  % 有效
% file = 'record_20260527_144032.csv';

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
ft_pos_raw = ft_pos;

%% 1. 角度解缠绕（避免正常绕回被当作尖峰）
angle1_u = rad2deg(unwrap(deg2rad(angle1)));
angle2_u = rad2deg(unwrap(deg2rad(angle2)));

%% 2. 用 Hampel 滤波去除 angle 的剧烈跳动（毛刺）
k = 25;        % 窗口半宽（点数）
nsigma = 3;    % 异常阈值（中位数偏离倍数）

[angle1_f, idx1] = hampel(angle1_u, k, nsigma);
angle1_f(idx1) = NaN;   % 尖峰置 NaN

angle1_valid_for_reverse = find(~isnan(angle1_f), 1, 'first');
if isempty(angle1_valid_for_reverse)
    error('无法找到 angle1 的有效初始值，无法进行反向变化转换');
end
angle1_initial = angle1_f(angle1_valid_for_reverse);
angle1_f = 2 * angle1_initial - angle1_f;

[angle2_f, idx2] = hampel(angle2_u, k, nsigma);
angle2_f(idx2) = NaN;

% 注意：ft_pos 不去尖峰，直接用原始值 ft_pos

%% 3. 按 ID 区分 pos
idx_id1 = ft_id == 1;
idx_id2 = ft_id == 2;

ft_pos_calc = ft_pos_raw;
ft_pos_calc(idx_id2) = 180 - ft_pos_raw(idx_id2);

%% 4. 对齐：将 angle1 对齐到 ft_pos (id=1) 的初始值
N = 20;   % 使用前 N 对同时有效点计算偏移量（中位数）

% 4.1 angle1 对齐 id=1
valid1 = find(idx_id1 & ~isnan(ft_pos_calc) & ~isnan(angle1_f));
valid1 = valid1(1:min(N, numel(valid1)));
if ~isempty(valid1)
    offset1 = median(ft_pos_calc(valid1) - angle1_f(valid1), 'omitnan');
    fprintf('angle1 对齐偏移量 = %.3f deg（基于前 %d 对有效点）\n', offset1, numel(valid1));
else
    error('无法找到 id=1 的同时有效对齐点，请检查数据或调整滤波参数');
end
angle1_aligned = angle1_f + offset1;

% 4.2 angle2 对齐 id=2
valid2 = find(idx_id2 & ~isnan(ft_pos_calc) & ~isnan(angle2_f));
valid2 = valid2(1:min(N, numel(valid2)));
if ~isempty(valid2)
    offset2 = median(ft_pos_calc(valid2) - angle2_f(valid2), 'omitnan');
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

plot(t_sec(idx_id1), ft_pos_calc(idx_id1), ...
    '-', 'LineWidth', 1.1, 'MarkerSize', 8, ...
    'DisplayName', 'ft pos, id = 1');

plot(t_sec(idx_id2), ft_pos_calc(idx_id2), ...
    '-', 'LineWidth', 1.1, 'MarkerSize', 8, ...
    'DisplayName', 'ft pos, id = 2 (180 - raw)');

xlabel('Time / s');
ylabel('Angle or Position / deg');
legend('Location', 'best');
hold off;
