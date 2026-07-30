function flag = isValidPose(theta, phi, params)
% 判断给定 theta 和 phi 下机构是否满足几何闭合条件且无干涉

    flag = true;

    AB = params.l_base;
    AE = params.l_drivel;
    BC = params.l_drives;
    CD = params.l_driven;
    DE = params.l_trans;

    tol = 1e-9;
    clip = @(x) max(-1, min(1, x));

    % ---- 几何闭合判断（原有内容） ----
    AC2 = AB^2 + BC^2 - 2 * AB * BC * cosd(phi);
    if AC2 <= tol, flag = false; return; end
    AC = sqrt(AC2);
    if AC > AB + BC + tol || AC < abs(AB - BC) - tol
        flag = false; return;
    end

    temp1 = (AC^2 + BC^2 - AB^2) / (2 * AC * BC);
    temp2 = (AB^2 + AC^2 - BC^2) / (2 * AB * AC);
    if abs(temp1) > 1 + tol || abs(temp2) > 1 + tol
        flag = false; return;
    end
    angle_ACB = acosd(clip(temp1));
    angle_CAB = acosd(clip(temp2));

    angle_CAE = theta - angle_CAB;
    CE2 = AC^2 + AE^2 - 2 * AC * AE * cosd(angle_CAE);
    if CE2 <= tol, flag = false; return; end
    CE = sqrt(CE2);
    if CE > CD + DE + tol || CE < abs(CD - DE) - tol
        flag = false; return;
    end

    temp3 = (AC^2 + CE^2 - AE^2) / (2 * AC * CE);
    temp4 = (CD^2 + CE^2 - DE^2) / (2 * CD * CE);
    if abs(temp3) > 1 + tol || abs(temp4) > 1 + tol
        flag = false; return;
    end
    angle_ACE = acosd(clip(temp3));
    angle_DCE = acosd(clip(temp4));

    angle_BCD = 360 - angle_ACB - angle_ACE - angle_DCE;
    if angle_BCD <= 3 || angle_BCD >= 357
        flag = false; return;
    end

    BD2 = BC^2 + CD^2 - 2 * BC * CD * cosd(angle_BCD);
    if BD2 <= tol, flag = false; return; end
    BD = sqrt(BD2);

    temp5 = (BC^2 + BD^2 - CD^2) / (2 * BC * BD);
    if abs(temp5) > 1 + tol
        flag = false; return;
    end
end