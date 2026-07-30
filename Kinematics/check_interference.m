function valid = check_interference(theta, phi, params)
% 返回 true 表示无干涉且可装配，false 表示应剔除

    % ----- 1. 装配合理性 -----
    if ~isValidPose(theta, phi, params)
        valid = false;
        return;
    end

    % ----- 2. 计算所有坐标 -----
    [A, B, C, D, E, G] = get_points(theta, phi, params);

    % ----- 3. 检查五根连杆两两之间是否真正干涉 -----
    links = { ...
        A, B, 'AB'; ...
        B, C, 'BC'; ...
        C, D, 'CD'; ...
        D, E, 'DE'; ...
        E, A, 'EA'  ...
        };

    tol = 1e-9;

    for i = 1:size(links, 1)-1
        for j = i+1:size(links, 1)

            p1 = links{i,1};
            p2 = links{i,2};
            q1 = links{j,1};
            q2 = links{j,2};

            if segments_interfere(p1, p2, q1, q2, tol)
                valid = false;
                return;
            end
        end
    end

    % ----- 4. 额外干涉检查 -----
    P = calcPalphaPbetaFromED(A, C, D, E, params);

    % ----- Pbeta1Pbeta2 与 DE 的厚度干涉检查 -----
    link_thickness = 7.5;  % 两根杆厚度均为 9
    clearance_limit = link_thickness;  % 9/2 + 9/2
    
    dist_Pbeta_DE = segment_distance_2d(P.Pbeta1, P.Pbeta2, D, E, tol);
    
    if dist_Pbeta_DE <= clearance_limit + tol
        valid = false;
        return;
    end

    % 仅当 Pbeta1-Pbeta2 的斜率小于 0 时，
    % 才要求 BC 的斜率大于 Pbeta1-Pbeta2 的斜率
    % if slope_less_than_zero(P.Pbeta1, P.Pbeta2, tol)
    %     if ~slope_greater_than(D, C, P.Pbeta1, P.Pbeta2, tol)
    %         valid = false;
    %         return;
    %     end
    % end
    % if segments_interfere(P.Pbeta2, P.Pbeta1, P.Pbeta2, C, tol)
    %     valid = false;
    %     return;
    % end
    % if segments_intersect_tol(P.Palpha2, P.Palpha3, B, C, tol)
    %     valid = false;
    %     return;
    % end
    % if segments_intersect_tol(P.Palpha2, P.Palpha3, A, B, tol)
    %     valid = false;
    %     return;
    % end
    % if segments_intersect_tol(P.Palpha1, P.Palpha2, A, B, tol)
    %     valid = false;
    %     return;
    % end
    % if segments_intersect_tol(P.Palpha1, P.Palpha2, P.Pbeta1, P.Pbeta2, tol)
    %     valid = false;
    %     return;
    % end

    valid = true;
end

function flag = segments_interfere(p1, p2, q1, q2, tol)
% 判断两线段是否真正干涉
%
% 规则：
% 1. 不相交：不干涉
% 2. 没有共享端点但相交：干涉
% 3. 只有一个共享端点，且除此之外无交叉/重叠：不干涉
% 4. 共线并且有长度重叠：干涉
% 5. 完全重合或部分重合：干涉

    if nargin < 5
        tol = 1e-9;
    end

    p1 = p1(:); 
    p2 = p2(:);
    q1 = q1(:); 
    q2 = q2(:);

    r = p2 - p1;
    s = q2 - q1;

    % 退化线段，直接认为非法
    if norm(r) < tol || norm(s) < tol
        flag = true;
        return;
    end

    % 如果两线段根本不相交，则不干涉
    if ~segments_intersect_tol(p1, p2, q1, q2, tol)
        flag = false;
        return;
    end

    % 判断共享端点数量
    shared_cnt = count_shared_endpoints(p1, p2, q1, q2, tol);

    % 没有共享端点但相交，必然干涉
    if shared_cnt == 0
        flag = true;
        return;
    end

    % 两个端点都共享，说明两线段相同或反向相同，必然干涉
    if shared_cnt >= 2
        flag = true;
        return;
    end

    % 只有一个共享端点
    % 如果两线段不平行，则它们只是在铰点处相交，不算干涉
    if abs(cross2d(r, s)) > tol * max(1, norm(r) * norm(s))
        flag = false;
        return;
    end

    % 如果平行并且相交，由于有共享点，基本就是共线情况
    % 需要判断是否在共享点之外还有重叠长度
    if abs(cross2d(q1 - p1, r)) > tol * max(1, norm(r))
        flag = false;
        return;
    end

    % 将 q1、q2 投影到 p1-p2 的参数坐标上
    % p1 对应 t=0，p2 对应 t=1
    tq1 = dot(q1 - p1, r) / dot(r, r);
    tq2 = dot(q2 - p1, r) / dot(r, r);

    qmin = min(tq1, tq2);
    qmax = max(tq1, tq2);

    % p 线段区间为 [0, 1]
    overlap_min = max(0, qmin);
    overlap_max = min(1, qmax);

    % 如果重叠区间有正长度，则说明除了共享端点外还有重叠，算干涉
    tol_t = tol / max(norm(r), 1);
    flag = (overlap_max - overlap_min) > tol_t;
end

function tf = segments_intersect_tol(p1, p2, q1, q2, tol)
% 判断两线段是否相交，包括端点接触，带容差

    if nargin < 5
        tol = 1e-9;
    end

    p1 = p1(:); 
    p2 = p2(:);
    q1 = q1(:); 
    q2 = q2(:);

    r = p2 - p1;
    s = q2 - q1;

    if norm(r) < tol || norm(s) < tol
        tf = false;
        return;
    end

    rxs = cross2d(r, s);
    qmp = q1 - p1;

    tol_cross = tol * max(1, norm(r) * norm(s));

    % 平行或共线
    if abs(rxs) < tol_cross

        % 平行但不共线
        if abs(cross2d(qmp, r)) > tol * max(1, norm(r))
            tf = false;
            return;
        end

        % 共线，判断投影区间是否重叠
        t0 = dot(q1 - p1, r) / dot(r, r);
        t1 = dot(q2 - p1, r) / dot(r, r);

        tmin = min(t0, t1);
        tmax = max(t0, t1);

        tf = (tmax >= -tol) && (tmin <= 1 + tol);
        return;
    end

    % 非平行，计算交点参数
    t = cross2d(qmp, s) / rxs;
    u = cross2d(qmp, r) / rxs;

    tf = (t >= -tol) && (t <= 1 + tol) && ...
         (u >= -tol) && (u <= 1 + tol);
end

function cnt = count_shared_endpoints(p1, p2, q1, q2, tol)
% 统计两线段共享端点数量

    cnt = 0;

    if norm(p1 - q1) <= tol
        cnt = cnt + 1;
    end

    if norm(p1 - q2) <= tol
        cnt = cnt + 1;
    end

    if norm(p2 - q1) <= tol
        cnt = cnt + 1;
    end

    if norm(p2 - q2) <= tol
        cnt = cnt + 1;
    end
end

function c = cross2d(a, b)
% 二维叉积的标量值
    c = a(1)*b(2) - a(2)*b(1);
end

% ---- 辅助函数：将 modelingfx 输出整理为点格式 ----
function [A, B, C, D, E, G] = get_points(theta, phi, params)
    [x_A, y_A, x_B, y_B, x_C, y_C, x_D, y_D, x_E, y_E, x_G, y_G] = ...
        modelingfx(theta, phi, params);
    A = [x_A, y_A];
    B = [x_B, y_B];
    C = [x_C, y_C];
    D = [x_D, y_D];
    E = [x_E, y_E];
    G = [x_G, y_G];
end

function flag = slope_greater_than(p1, p2, q1, q2, tol)
% 判断线段 p1-p2 的斜率是否大于 q1-q2 的斜率
%
% 使用 atan(k) 比较，避免竖直线 dy/dx 除零问题。
% 竖直线视为斜率 +Inf，对应角度 pi/2。

    if nargin < 5
        tol = 1e-9;
    end

    p1 = p1(:);
    p2 = p2(:);
    q1 = q1(:);
    q2 = q2(:);

    v1 = p2 - p1;
    v2 = q2 - q1;

    % 退化线段直接认为不满足
    if norm(v1) < tol || norm(v2) < tol
        flag = false;
        return;
    end

    a1 = slope_angle(v1, tol);
    a2 = slope_angle(v2, tol);

    % 严格要求 DC 斜率大于 Pbeta1-Pbeta2 斜率
    flag = a1 > a2 + tol;
end

function a = slope_angle(v, tol)
% 将线段方向转换为斜率对应的角度 atan(k)
% 返回范围近似为 [-pi/2, pi/2]

    dx = v(1);
    dy = v(2);

    if abs(dx) < tol
        % 竖直线，视为斜率 +Inf
        a = pi / 2;
    else
        a = atan(dy / dx);
    end
end

function flag = slope_less_than_zero(p1, p2, tol)
% 判断线段 p1-p2 的斜率是否小于 0

    if nargin < 3
        tol = 1e-9;
    end

    p1 = p1(:);
    p2 = p2(:);

    v = p2 - p1;

    % 退化线段，认为不满足
    if norm(v) < tol
        flag = false;
        return;
    end

    dx = v(1);
    dy = v(2);

    % 竖直线斜率无穷，不认为小于 0
    if abs(dx) < tol
        flag = false;
        return;
    end

    k = dy / dx;

    flag = k < -tol;
end

function d = segment_distance_2d(p1, p2, q1, q2, tol)
% 计算二维线段 p1-p2 与 q1-q2 的最短距离

    if nargin < 5
        tol = 1e-9;
    end

    p1 = p1(:);
    p2 = p2(:);
    q1 = q1(:);
    q2 = q2(:);

    % 如果中心线已经相交，距离为 0
    if segments_intersect_tol(p1, p2, q1, q2, tol)
        d = 0;
        return;
    end

    d = min([
        point_to_segment_distance(p1, q1, q2)
        point_to_segment_distance(p2, q1, q2)
        point_to_segment_distance(q1, p1, p2)
        point_to_segment_distance(q2, p1, p2)
    ]);
end


function d = point_to_segment_distance(p, a, b)
% 计算点 p 到线段 a-b 的最短距离

    p = p(:);
    a = a(:);
    b = b(:);

    ab = b - a;
    ab2 = dot(ab, ab);

    if ab2 < 1e-12
        d = norm(p - a);
        return;
    end

    t = dot(p - a, ab) / ab2;
    t = max(0, min(1, t));

    projection = a + t * ab;
    d = norm(p - projection);
end