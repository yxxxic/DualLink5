function [x_A, y_A, x_B, y_B, x_C, y_C, x_D, y_D, x_E, y_E, x_G, y_G] = modelingfx(theta, phi, params)
% kinematic modeling
% 2-DOF TM (two-degree-of-freedom translational mechanism)
% 输入:
%   theta - 驱动长连杆角度 (deg)
%   phi   - 驱动短连杆角度 (deg)
% 输出:
%   x_G, y_G - 末端点 G 坐标

    % 读取参数
    AB = params.l_base;
    AE = params.l_drivel;
    BC = params.l_drives;
    CD = params.l_driven;
    DE = params.l_trans;

    % 防止 acosd 输入超出 [-1,1]
    clip = @(x) max(-1, min(1, x));

    % ===== 几何计算 =====
    AC = sqrt(AB^2 + BC^2 - 2*AB*BC*cosd(phi));

    angle_ACB = acosd(clip((AC^2 + BC^2 - AB^2) / (2*AC*BC)));
    angle_CAB = acosd(clip((AB^2 + AC^2 - BC^2) / (2*AB*AC)));

    angle_CAE = theta - angle_CAB;

    CE = sqrt(AC^2 + AE^2 - 2*AC*AE*cosd(angle_CAE));

    angle_ACE = acosd(clip((AC^2 + CE^2 - AE^2) / (2*AC*CE)));
    angle_DCE = acosd(clip((CD^2 + CE^2 - DE^2) / (2*CD*CE)));

    angle_BCD = 360 - angle_ACB - angle_ACE - angle_DCE;

    BD = sqrt(BC^2 + CD^2 - 2*BC*CD*cosd(angle_BCD));

    angle_CBD = acosd(clip((BC^2 + BD^2 - CD^2) / (2*BC*BD)));

    angle_DBH = 180 - phi - angle_CBD;

    % ===== 坐标计算 =====
    x_D = AB + BD * cosd(angle_DBH);
    y_D = BD * sind(angle_DBH);

    x_E = AE * cosd(theta);
    y_E = AE * sind(theta);

    x_B = AB;
    y_B = 0;

    x_A = 0;
    y_A = 0;

    x_C = AB - BC * cosd(phi);
    y_C = BC * sind(phi);

    % ===== 对称点 G =====
    x_G = x_D + x_E - x_B;
    y_G = y_D + y_E - y_B;

end
