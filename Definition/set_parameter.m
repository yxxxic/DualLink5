% set_parameter
% 2-DOF TM (two-degree-of-freedom translational mechanism)
% 设置二自由度平动结构参数
% 包含五连杆长度参数

params.l_base = 80;
params.l_drivel = params.l_base;
params.l_drives = 62;
params.l_driven = 69;
params.l_trans = params.l_base;
params.l_parallel_b = params.l_base*0.75;
params.l_parallel_a = params.l_base*0.75;

params.l_D_Pbeta2 = params.l_parallel_b / 2;
params.l_Pbeta2_Pbeta3 = params.l_parallel_b;
params.l_E_Palpha1 = params.l_parallel_a / 2;
params.l_Palpha1_Palpha4 = params.l_parallel_a;
% beta 偏移方向
% u_CD = C -> D
% +1 表示从 D 继续沿 C->D 方向
% -1 表示从 D 朝 C 的方向
params.sign_beta_offset = -1;
% Pbeta2 -> Pbeta3 的方向
params.sign_beta_side = 1;
% alpha 偏移方向
% u_AE = A -> E
% +1 表示从 E 继续沿 A->E 方向
% -1 表示从 E 朝 A 的方向
params.sign_alpha_offset = -1;
% Palpha1 -> Palpha4 的方向
params.sign_alpha_side = 1;