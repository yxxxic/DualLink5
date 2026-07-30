function P = calcPalphaPbetaFromED(A, C, D, E, params)
% calcPalphaPbetaFromED
%
% 根据已知 A, C, D, E 坐标，计算 Palpha 和 Pbeta 各点坐标
%
% 几何约束：
%   Pbeta1Pbeta2 // ED 且 |Pbeta1Pbeta2| = |ED|
%   Pbeta2Pbeta3 与 CD 共线
%
%   Palpha1Palpha2 // ED 且 |Palpha1Palpha2| = |ED|
%   Palpha1Palpha4 与 AE 共线
%
% 输入：
%   A, C, D, E : 点坐标 [x, y]
%   params     : 参数结构体
%
% 输出：
%   P          : 结构体，包含 Palpha、Pbeta 坐标

    A = A(:).';
    C = C(:).';
    D = D(:).';
    E = E(:).';

    tol = 1e-9;

    %% ===============================
    %  基本方向向量
    %% ===============================

    ED = D - E;
    L_ED = norm(ED);

    if L_ED < tol
        error('E 和 D 重合，无法定义 ED 方向。');
    end

    CD = D - C;
    L_CD = norm(CD);

    if L_CD < tol
        error('C 和 D 重合，无法定义 CD 方向。');
    end

    AE = E - A;
    L_AE = norm(AE);

    if L_AE < tol
        error('A 和 E 重合，无法定义 AE 方向。');
    end

    u_CD = CD / L_CD;   % C -> D 方向
    u_AE = AE / L_AE;   % A -> E 方向

    %% ===============================
    %  默认符号参数
    %% ===============================

    if ~isfield(params, 'sign_beta_offset')
        params.sign_beta_offset = 1;
    end

    if ~isfield(params, 'sign_beta_side')
        params.sign_beta_side = 1;
    end

    if ~isfield(params, 'sign_alpha_offset')
        params.sign_alpha_offset = 1;
    end

    if ~isfield(params, 'sign_alpha_side')
        params.sign_alpha_side = 1;
    end

    %% ===============================
    %  beta 组
    %
    %  Pbeta1Pbeta2 // ED
    %  |Pbeta1Pbeta2| = |ED|
    %
    %  Pbeta2Pbeta3 与 CD 共线
    %% ===============================

    r_beta = params.sign_beta_offset * params.l_D_Pbeta2 * u_CD;

    Pbeta1 = E + r_beta;
    Pbeta2 = D + r_beta;

    % beta 侧边，沿 CD 方向
    v_beta = params.sign_beta_side * params.l_Pbeta2_Pbeta3 * u_CD;

    Pbeta3 = Pbeta2 + v_beta;
    Pbeta4 = Pbeta1 + v_beta;

    %% ===============================
    %  alpha 组
    %
    %  Palpha1Palpha2 // ED
    %  |Palpha1Palpha2| = |ED|
    %
    %  Palpha1Palpha4 与 AE 共线
    %% ===============================

    r_alpha = params.sign_alpha_offset * params.l_E_Palpha1 * u_AE;

    Palpha1 = E + r_alpha;
    Palpha2 = D + r_alpha;

    % alpha 侧边，沿 AE 方向
    v_alpha = params.sign_alpha_side * params.l_Palpha1_Palpha4 * u_AE;

    Palpha4 = Palpha1 + v_alpha;
    Palpha3 = Palpha2 + v_alpha;

    %% ===============================
    %  几何校验
    %% ===============================

    % beta1 beta2 是否等于 ED
    if abs(norm(Pbeta2 - Pbeta1) - L_ED) > 1e-7
        warning('Pbeta1Pbeta2 长度不等于 ED。');
    end

    % alpha1 alpha2 是否等于 ED
    if abs(norm(Palpha2 - Palpha1) - L_ED) > 1e-7
        warning('Palpha1Palpha2 长度不等于 ED。');
    end

    % Pbeta2Pbeta3 是否与 CD 平行
    if abs(cross2d(Pbeta3 - Pbeta2, CD)) > 1e-7
        warning('Pbeta2Pbeta3 与 CD 不平行。');
    end

    % Palpha1Palpha4 是否与 AE 平行
    if abs(cross2d(Palpha4 - Palpha1, AE)) > 1e-7
        warning('Palpha1Palpha4 与 AE 不平行。');
    end

    %% ===============================
    %  输出
    %% ===============================

    P.Pbeta1 = Pbeta1;
    P.Pbeta2 = Pbeta2;
    P.Pbeta3 = Pbeta3;
    P.Pbeta4 = Pbeta4;

    P.Palpha1 = Palpha1;
    P.Palpha2 = Palpha2;
    P.Palpha3 = Palpha3;
    P.Palpha4 = Palpha4;
end


function c = cross2d(a, b)
% 二维叉积
    c = a(1) * b(2) - a(2) * b(1);
end