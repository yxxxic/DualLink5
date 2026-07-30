function data = extractExperimentAngles(file, varargin)
% extractExperimentAngles
% 从实验 CSV 中提取并转换为后续计算使用的数据。
%
% 输入：
%   file : CSV 文件路径
%
% 可选 Name-Value 参数：
%   'HampelWindow' : Hampel 滤波窗口半宽，默认 25
%   'HampelNSigma' : Hampel 异常阈值，默认 3
%   'AlignN'       : 用前 N 个有效点计算对齐偏移，默认 20
%   'DoAlign'      : 是否将 angle 对齐到 ft_pos，默认 true
%
% 输出 data 字段：
%   data.t_sec
%   data.angle1
%   data.angle2
%   data.ft_pos1
%   data.ft_pos2
%   data.angle1_raw
%   data.angle2_raw
%   data.ft_pos_raw
%   data.ft_pos_calc
%   data.offset1
%   data.offset2


% 使用：data = extractExperimentAngles('record_20260520_164213.csv');
% 可选参数：
% data = extractExperimentAngles( ...
%     'record_20260520_164213.csv', ...
%     'HampelWindow', 25, ...
%     'HampelNSigma', 3, ...
%     'AlignN', 20, ...
%     'DoAlign', true);

    parser = inputParser;
    parser.addRequired('file', @(x) ischar(x) || isstring(x));
    parser.addParameter('HampelWindow', 25, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parser.addParameter('HampelNSigma', 3, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parser.addParameter('AlignN', 20, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parser.addParameter('DoAlign', true, @(x) islogical(x) && isscalar(x));
    parser.parse(file, varargin{:});

    k = parser.Results.HampelWindow;
    nsigma = parser.Results.HampelNSigma;
    N = parser.Results.AlignN;
    do_align = parser.Results.DoAlign;

    opts = detectImportOptions(file, 'VariableNamingRule', 'preserve');
    T = readtable(file, opts);

    time_raw = T.("timestamp");
    if isdatetime(time_raw)
        t = time_raw;
    else
        t = datetime(string(time_raw), ...
            'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSS');
    end
    t_sec = seconds(t - t(1));

    angle1_raw = T.("mt6835_angle_angle1_deg");
    angle2_raw = T.("mt6835_angle_angle2_deg");

    ft_id  = T.("ft_sts_status_id[0]");
    ft_pos = T.("ft_sts_status_pos[0]");

    if ~isnumeric(ft_id)
        ft_id = str2double(string(ft_id));
    end
    if ~isnumeric(ft_pos)
        ft_pos = str2double(string(ft_pos));
    end
    ft_pos_raw = ft_pos;

    angle1_u = rad2deg(unwrap(deg2rad(angle1_raw)));
    angle2_u = rad2deg(unwrap(deg2rad(angle2_raw)));

    if k > 0
        [angle1_f, idx1] = hampel(angle1_u, k, nsigma);
        angle1_f(idx1) = NaN;

        [angle2_f, idx2] = hampel(angle2_u, k, nsigma);
        angle2_f(idx2) = NaN;
    else
        angle1_f = angle1_u;
        angle2_f = angle2_u;
    end

    angle1_valid_for_reverse = find(~isnan(angle1_f), 1, 'first');
    if isempty(angle1_valid_for_reverse)
        error('无法找到 angle1 的有效初始值，无法进行反向变化转换');
    end

    angle1_initial = angle1_f(angle1_valid_for_reverse);
    angle1_f = 2 * angle1_initial - angle1_f;

    idx_id1 = ft_id == 1;
    idx_id2 = ft_id == 2;

    ft_pos_calc = ft_pos_raw;

    ft_pos_calc(idx_id2) = 180 - ft_pos_raw(idx_id2);

    offset1 = 0;
    offset2 = 0;

    if do_align
        valid1 = find(idx_id1 & ~isnan(ft_pos_calc) & ~isnan(angle1_f));
        valid1 = valid1(1:min(N, numel(valid1)));
        if ~isempty(valid1)
            offset1 = median(ft_pos_calc(valid1) - angle1_f(valid1), 'omitnan');
        else
            error('无法找到 id=1 的同时有效对齐点，请检查数据或调整滤波参数');
        end

        valid2 = find(idx_id2 & ~isnan(ft_pos_calc) & ~isnan(angle2_f));
        valid2 = valid2(1:min(N, numel(valid2)));
        if ~isempty(valid2)
            offset2 = median(ft_pos_calc(valid2) - angle2_f(valid2), 'omitnan');
        else
            error('无法找到 id=2 的同时有效对齐点，请检查数据或调整滤波参数');
        end
    end

    angle1 = angle1_f + offset1;
    angle2 = angle2_f + offset2;

    data = struct();
    data.t_sec = t_sec;
    data.angle1 = angle1;
    data.angle2 = angle2;
    data.ft_pos1 = ft_pos_calc;
    data.ft_pos1(~idx_id1) = NaN;
    data.ft_pos2 = ft_pos_calc;
    data.ft_pos2(~idx_id2) = NaN;

    data.angle1_raw = angle1_raw;
    data.angle2_raw = angle2_raw;
    data.angle1_filtered = angle1_f;
    data.angle2_filtered = angle2_f;
    data.angle1_initial = angle1_initial;
    data.ft_id = ft_id;
    data.ft_pos_raw = ft_pos_raw;
    data.ft_pos_calc = ft_pos_calc;
    data.idx_id1 = idx_id1;
    data.idx_id2 = idx_id2;
    data.offset1 = offset1;
    data.offset2 = offset2;
end
