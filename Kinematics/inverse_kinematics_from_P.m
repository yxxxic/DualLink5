function sol = inverse_kinematics_from_P(P, params)
% 已知 ED 中点 P，求五连杆机构的逆运动学解
%
% 机构顺序：
% A - B - C - D - E - A
%
% AB 为固定杆
% P 为 ED 中点
%
% 输出 sol 为结构体数组，每个元素包含：
% sol(k).theta
% sol(k).phi
% sol(k).A
% sol(k).B
% sol(k).C
% sol(k).D
% sol(k).E

    % -------- 1. 杆长 --------
    L_AB = params.l_base;
    L_EA = params.l_drivel;
    L_BC = params.l_drives;
    L_CD = params.l_driven;
    L_DE = params.l_trans;

    % -------- 2. 固定点 --------
    A = [0, 0];
    B = [L_AB, 0];

    P = P(:).';

    % -------- 3. 求 E 点 --------
    r_AE = L_EA;
    r_PE = L_DE / 2;

    E_list = circle_intersections(A, r_AE, P, r_PE);

    sol = struct([]);
    idx = 0;

    % -------- 4. 遍历所有 E 解 --------
    for i = 1:size(E_list, 1)

        E = E_list(i, :);

        % 由 P 为 ED 中点，求 D
        D = 2 * P - E;

        % -------- 5. 求 C 点 --------
        C_list = circle_intersections(B, L_BC, D, L_CD);

        % -------- 6. 遍历所有 C 解 --------
        for j = 1:size(C_list, 1)

            C = C_list(j, :);

            % -------- 7. 计算 theta --------
            v_AE = E - A;
            theta = atan2(v_AE(2), v_AE(1));

            % -------- 8. 计算 phi --------
            v_BA = A - B;
            v_BC = C - B;

            cos_phi = dot(v_BA, v_BC) / (norm(v_BA) * norm(v_BC));

            % 防止数值误差导致 acos 输入略微超过 [-1, 1]
            cos_phi = max(-1, min(1, cos_phi));

            phi = acos(cos_phi);

            theta_deg = rad2deg(theta);
            phi_deg   = rad2deg(phi);

            % 如果希望 theta 在 [0, 360)
            if theta_deg < 0
                theta_deg = theta_deg + 360;
            end

            % -------- 9. 保存解 --------
            idx = idx + 1;

            sol(idx).theta = theta_deg;
            sol(idx).phi   = phi_deg;

            sol(idx).A = A;
            sol(idx).B = B;
            sol(idx).C = C;
            sol(idx).D = D;
            sol(idx).E = E;
            sol(idx).P = P;
        end
    end
end

function pts = circle_intersections(O1, r1, O2, r2)
% 求两个圆的交点
%
% 圆1：圆心 O1，半径 r1
% 圆2：圆心 O2，半径 r2
%
% 返回：
% pts 为 n x 2 矩阵
% n = 0, 1, 2

    tol = 1e-9;

    O1 = O1(:).';
    O2 = O2(:).';

    d_vec = O2 - O1;
    d = norm(d_vec);

    pts = [];

    % 圆心重合
    if d < tol
        return;
    end

    % 两圆相离
    if d > r1 + r2 + tol
        return;
    end

    % 一个圆在另一个圆内部，无交点
    if d < abs(r1 - r2) - tol
        return;
    end

    % 单位方向
    ex = d_vec / d;

    % 从 O1 到两圆公共弦中点的距离
    a = (r1^2 - r2^2 + d^2) / (2 * d);

    % 垂直距离平方
    h2 = r1^2 - a^2;

    if h2 < -tol
        return;
    end

    % 数值误差修正
    h2 = max(0, h2);

    h = sqrt(h2);

    % 公共弦中点
    M = O1 + a * ex;

    % 垂直方向
    ey = [-ex(2), ex(1)];

    if h < tol
        % 相切，一个交点
        pts = M;
    else
        % 两个交点
        P1 = M + h * ey;
        P2 = M - h * ey;

        pts = [P1; P2];
    end
end