% kinematic modeling
% 2-DOF TM (two-degree-of-freedom translational mechanism)
% 二自由度平动结构运动学建模
% 输入参数 驱动长连杆夹角theta; 驱动短连杆夹角phi
% 输出参数 计算机构末端点坐标
addpath('../Definition'); 
run('../Definition/set_parameter.m');

phi = 52.93;
theta = 85;

AB = l_base;
AE = l_drivel;
BC = l_drives;
CD = l_driven;
DE = l_trans;

AC = sqrt(AB^2 + BC^2 - 2*AB*BC*cosd(phi));
% angle_ACB = asind(AB/AC * sind(phi));
angle_ACB = acosd((AC^2 + BC^2 - AB^2) / (2*AC*BC));
% angle_CAB = asind(BC/AC * sind(phi));
angle_CAB = acosd((AB^2 + AC^2 - BC^2) / (2*AB*AC));
angle_CAE = theta - angle_CAB;
CE = sqrt(AC^2 + AE^2 - 2*AC*AE*cosd(angle_CAE));
% angle_ACE = asind(AE/CE * sind(angle_CAE));
angle_ACE = acosd((AC^2 + CE^2 - AE^2) / (2*AC*CE));
angle_DCE = acosd((CD^2 + CE^2 - DE^2) / (2*CD*CE));
angle_BCD = 360 - angle_ACB - angle_ACE - angle_DCE;
BD = sqrt(BC^2 + CD^2 - 2*BC*CD*cosd(angle_BCD));
% angle_CBD = asind(CD/BD * sind(angle_BCD));
angle_CBD = acosd((BC^2 + BD^2 - CD^2) / (2*BC*BD));
angle_DBH = 180 - phi - angle_CBD;
x_D = AB + BD * cosd(angle_DBH);
y_D = BD * sind(angle_DBH);
x_E = AE * cosd(theta);
y_E = AE * sind(theta);
x_B = AB;
y_B = 0;
x_P = (x_D + x_E) / 2;
y_P = (y_D + y_E) / 2;
x_G = 2 * x_P - x_B;
y_G = 2 * y_P - y_B;