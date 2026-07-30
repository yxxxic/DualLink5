function plotWorkspace(theta_range, phi_range, params, G_func)
% plotWorkspace
% 计算并绘制机构工作空间（末端点 G），用颜色区分有无干涉。
%
% 输入：
%   theta_range  - theta 角度向量 (deg)
%   phi_range    - phi 角度向量 (deg)
%   params       - 机构参数
%   G_func       - 函数句柄，用于从所有点中提取 G 点
%                  格式： G = G_func(theta, phi, params)
%                  也可以直接传入一个结构体，但为了灵活，使用函数句柄。
%                  例如：G_func = @(t,p,par) calcPalphaPbetaFromED(t,p,par).G;
%
% 绘图：
%   绿色点：无干涉位姿
%   红色点：发生干涉位姿

    X_free = []; Y_free = [];
    X_inter = []; Y_inter = [];

    nTh = length(theta_range);
    nPh = length(phi_range);
    total = nTh * nPh;
    fprintf('开始计算工作空间，共 %d 个位姿...\n', total);

    count = 0;
    tStart = tic;

    for i = 1:nTh
        theta = theta_range(i);
        for j = 1:nPh
            phi = phi_range(j);
            count = count + 1;

            % 1) 计算基础五杆点 A~E
            [x_A, y_A, x_B, y_B, x_C, y_C, x_D, y_D, x_E, y_E] = ...
                modelingfx(theta, phi, params);
            A = [x_A, y_A]; B = [x_B, y_B]; C = [x_C, y_C];
            D = [x_D, y_D]; E = [x_E, y_E];

            % 2) 计算 Palpha/Pbeta 点（同时可获取 G 点）
            pp = calcPalphaPbetaFromED(A, C, D, E, params);

            % 3) 提取末端点 G（通过函数句柄，可以是 pp.G 或自定义）
            G = G_func(theta, phi, params);  % 这里也可以直接用 pp.G
            % 如果 G_func 就是直接返回 pp.G，可以写为：
            % G = pp.G;

            % 4) 干涉检查（使用预计算的点，不再重复建模）
            flag = checkInterferenceFromPoints(A, B, C, D, E, ...
                pp.Palpha1, pp.Palpha2, pp.Pbeta1, pp.Pbeta2, params);

            % 5) 分类保存
            if flag
                X_inter(end+1) = G(1);
                Y_inter(end+1) = G(2);
            else
                X_free(end+1) = G(1);
                Y_free(end+1) = G(2);
            end

            % 进度显示
            if mod(count, 500) == 0 || count == total
                elapsed = toc(tStart);
                fprintf('  进度 %d/%d，已用时 %.1f 秒\n', count, total, elapsed);
            end
        end
    end

    elapsed = toc(tStart);
    fprintf('计算完成，总用时 %.1f 秒\n', elapsed);

    % -- 绘图 --
    figure; hold on; axis equal; grid on; box on;
    if ~isempty(X_free)
        plot(X_free, Y_free, 'g.', 'MarkerSize', 4, 'DisplayName', '无干涉');
    end
    if ~isempty(X_inter)
        plot(X_inter, Y_inter, 'r.', 'MarkerSize', 4, 'DisplayName', '有干涉');
    end
    if ~isempty(X_free) && numel(X_free) >= 3
        try
            k = convhull(X_free, Y_free);
            plot(X_free(k), Y_free(k), 'c-', 'LineWidth', 1.5, 'DisplayName', '无干涉凸包');
        end
    end
    xlabel('X'); ylabel('Y');
    title('工作空间（绿=无干涉，红=干涉）');
    legend;
end