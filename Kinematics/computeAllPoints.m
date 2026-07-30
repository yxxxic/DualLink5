function P = computeAllPoints(theta, phi, params)
% computeAllPoints
%
% 输出机构所有关键点，包括：
%   A, B, C, D, E
%   Palpha1, Palpha2, Palpha3, Palpha4
%   Pbeta1,  Pbeta2,  Pbeta3,  Pbeta4
%
% 输入：
%   theta, phi  - 驱动角度，单位 deg
%   params      - 机构参数
%
% 输出：
%   P - 结构体，包含所有点坐标

    tol = 1e-9;

    % ---- 基础点 ----
    [x_A, y_A, x_B, y_B, x_C, y_C, x_D, y_D, x_E, y_E] = ...
        modelingfx(theta, phi, params);

    P.A = [x_A, y_A];
    P.B = [x_B, y_B];
    P.C = [x_C, y_C];
    P.D = [x_D, y_D];
    P.E = [x_E, y_E];

    % ---- 方向向量 ----
    CD = P.D - P.C;
    AE = P.E - P.A;

    L_CD = norm(CD);
    L_AE = norm(AE);

    if L_CD < tol
        error('C 和 D 重合。');
    end
    if L_AE < tol
        error('A 和 E 重合。');
    end

    u_CD = CD / L_CD;   % C -> D
    u_AE = AE / L_AE;   % A -> E

    % ---- 读取长度参数 ----
    l_D_Pbeta2 = getParam(params, {'l_D_Pbeta2','l_D_Pb2'}, [], true);
    l_Pbeta2_Pbeta3 = getParam(params, {'l_Pbeta2_Pbeta3'}, 0, false);

    l_E_Palpha1 = getParam(params, {'l_E_Palpha1','l_E_Pa1'}, [], true);
    l_Palpha1_Palpha4 = getParam(params, {'l_Palpha1_Palpha4'}, 0, false);

    sign_beta_offset = getParam(params, {'sign_beta_offset'}, 1, false);
    sign_beta_side   = getParam(params, {'sign_beta_side'},   1, false);

    sign_alpha_offset = getParam(params, {'sign_alpha_offset'}, 1, false);
    sign_alpha_side   = getParam(params, {'sign_alpha_side'},   1, false);

    % ---- Beta 平行四边形 ----
    r_beta = sign_beta_offset * l_D_Pbeta2 * u_CD;

    P.Pbeta1 = P.E + r_beta;
    P.Pbeta2 = P.D + r_beta;

    v_beta = sign_beta_side * l_Pbeta2_Pbeta3 * u_CD;

    P.Pbeta3 = P.Pbeta2 + v_beta;
    P.Pbeta4 = P.Pbeta1 + v_beta;

    % ---- Alpha 平行四边形 ----
    r_alpha = sign_alpha_offset * l_E_Palpha1 * u_AE;

    P.Palpha1 = P.E + r_alpha;
    P.Palpha2 = P.D + r_alpha;

    v_alpha = sign_alpha_side * l_Palpha1_Palpha4 * u_AE;

    P.Palpha4 = P.Palpha1 + v_alpha;
    P.Palpha3 = P.Palpha2 + v_alpha;
end