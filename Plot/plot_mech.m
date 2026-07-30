clc;
clear;
close all;
addpath('../Definition'); 
addpath('../Kinematics');
run('../Definition/set_parameter.m');

theta = 95.4;
phi = 33.1;

[x_A, y_A, x_B, y_B, x_C, y_C, x_D, y_D, x_E, y_E, x_G, y_G] = modelingfx(theta, phi, params);

A = [x_A, y_A];
C = [x_C, y_C];
B = [x_B, y_B];
D = [x_D, y_D];
E = [x_E, y_E];
G = [x_G, y_G];
P = calcPalphaPbetaFromED(A, C, D, E, params);

Pbeta1  = P.Pbeta1;
Pbeta2  = P.Pbeta2;
Pbeta3  = P.Pbeta3;
Pbeta4  = P.Pbeta4;

Palpha1 = P.Palpha1;
Palpha2 = P.Palpha2;
Palpha3 = P.Palpha3;
Palpha4 = P.Palpha4;


figure;
hold on;
axis equal;
grid on;

% 原机构点
plot(A(1), A(2), 'ko', 'MarkerFaceColor', 'k');
text(A(1), A(2), ' A');

plot(B(1), B(2), 'ko', 'MarkerFaceColor', 'k');
text(B(1), B(2), ' B');

plot(C(1), C(2), 'ko', 'MarkerFaceColor', 'k');
text(C(1), C(2), ' C');

plot(D(1), D(2), 'ko', 'MarkerFaceColor', 'k');
text(D(1), D(2), ' D');

plot(E(1), E(2), 'ko', 'MarkerFaceColor', 'k');
text(E(1), E(2), ' E');

plot(G(1), G(2), 'ko', 'MarkerFaceColor', 'k');
text(G(1), G(2), ' G');

% ED
plot([E(1), D(1)], [E(2), D(2)], 'k-', 'LineWidth', 2);
plot([B(1), G(1)], [B(2), G(2)], 'c--', 'LineWidth', 2);
% CD, AE
plot([C(1), D(1)], [C(2), D(2)], 'g--', 'LineWidth', 1.5);
plot([A(1), E(1)], [A(2), E(2)], 'm--', 'LineWidth', 1.5);
plot([A(1), B(1)], [A(2), B(2)], 'm--', 'LineWidth', 1.5);
plot([B(1), C(1)], [B(2), C(2)], 'm--', 'LineWidth', 1.5);

% beta
plot([Pbeta1(1), Pbeta2(1), Pbeta3(1), Pbeta4(1), Pbeta1(1)], ...
     [Pbeta1(2), Pbeta2(2), Pbeta3(2), Pbeta4(2), Pbeta1(2)], ...
     'r-', 'LineWidth', 2);

text(Pbeta1(1), Pbeta1(2), ' P\beta1');
text(Pbeta2(1), Pbeta2(2), ' P\beta2');
text(Pbeta3(1), Pbeta3(2), ' P\beta3');
text(Pbeta4(1), Pbeta4(2), ' P\beta4');

% alpha
plot([Palpha1(1), Palpha2(1), Palpha3(1), Palpha4(1), Palpha1(1)], ...
     [Palpha1(2), Palpha2(2), Palpha3(2), Palpha4(2), Palpha1(2)], ...
     'b-', 'LineWidth', 2);

text(Palpha1(1), Palpha1(2), ' P\alpha1');
text(Palpha2(1), Palpha2(2), ' P\alpha2');
text(Palpha3(1), Palpha3(2), ' P\alpha3');
text(Palpha4(1), Palpha4(2), ' P\alpha4');

legend('A/C/D/E', 'ED', 'CD', 'AE', 'beta parallelogram', 'alpha parallelogram');