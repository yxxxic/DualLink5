experimentDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(experimentDir);
run(fullfile(projectRoot, 'startup.m'));
% 读取CSV文件
% data = readtable('record_20260417_153713.csv');
% data = readtable('record_20260417_153738.csv');
% data = readtable('record_20260417_153904.csv');
dataFile = fullfile(experimentDir, 'record_20260417_153928.csv');
data = readtable(dataFile);

% 将时间戳转换为 datetime 类型
time = datetime(data.timestamp,'InputFormat','yyyy-MM-dd HH:mm:ss.SSSSSS');

% 去掉 mt6835_angle_angle1_deg 中的0和接近360的干扰数据
angle1_raw = data.mt6835_angle_angle1_deg;
valid_idx = angle1_raw > 0 & angle1_raw < 359; % 去掉0和接近360的数据

angle1 = angle1_raw(valid_idx) - 30;
angle2 = data.mt6835_angle_angle2_deg(valid_idx) - 190;

% 分离 MT position 数据0x141和0x142
idx_141 = strcmp(data.mt_position_header_frame_id, 'mt_actuator_id_0x141') & valid_idx;
idx_142 = strcmp(data.mt_position_header_frame_id, 'mt_actuator_id_0x142') & valid_idx;
position_141 = data.mt_position_position(idx_141);
position_142 = data.mt_position_position(idx_142);
position_141 = position_141 - 360;
position_142 = position_142 - 760;
time_141 = time(idx_141);
time_142 = time(idx_142);
time_valid = time(valid_idx);

% 去掉所有数组中的负数和大于100的数据
angle1_range = angle1 >= 0 & angle1 <= 100;
angle2_range = angle2 >= 0 & angle2 <= 100;
position_141_range = position_141 >= -10 & position_141 <= 100;
position_142_range = position_142 >= -10 & position_142 <= 100;

% 绘图
figureHandle = figure;
hold on;

plot(time_valid(angle1_range), angle1(angle1_range), 'Color', [0.6,0,0], 'DisplayName', 'Angle 1'); % 深红
plot(time_valid(angle2_range), angle2(angle2_range), 'r', 'DisplayName', 'Motor1'); % 普通红
plot(time_141(position_141_range), position_141(position_141_range), 'Color', [0,0,0.6], 'DisplayName', 'Angle 2'); % 深蓝
plot(time_142(position_142_range), position_142(position_142_range), 'b', 'DisplayName', 'Motor2'); % 普通蓝
xlabel('Time');
ylabel('Value');
title('4');
legend;
grid on;
hold off;

duallink5.viz.exportFigure(figureHandle, ...
    fullfile(experimentDir, '4.png'), struct('resolution', 300));
