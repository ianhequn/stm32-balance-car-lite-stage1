%% run_a43_verification.m
% 一键运行 A43「数学建模」验证工程。
% 目标：证明两轮平衡小车无控制会发散，加入满足 k1>g、k2>0 的 PD 后具备恢复能力。

clear; clc;

a43_params;
build_a43_simulink_model;

model = 'a43_balance_model_verification';
simOut = sim(model);

open_angle = simOut.get('open_angle');
closed_angle = simOut.get('closed_angle');
disturbance = simOut.get('disturbance');

open_poles = roots(open_den);
closed_poles = roots(closed_den);

fprintf('\nA43 model parameters:\n');
fprintf('g = %.3f m/s^2\n', g);
fprintf('L = %.3f m\n', L);
fprintf('k1 = %.3f, k2 = %.3f\n', k1, k2);
fprintf('\nOpen-loop poles:\n');
disp(open_poles);
fprintf('Closed-loop poles:\n');
disp(closed_poles);

figure('Name', 'A43 Balance Car Modeling Verification', 'Color', 'w');

subplot(3, 1, 1);
plot(disturbance.time, disturbance.signals.values, 'k', 'LineWidth', 1.2);
grid on;
ylabel('disturb');
title('外力扰动：用 1s 脉冲模拟轻推小车');

subplot(3, 1, 2);
plot(open_angle.time, open_angle.signals.values, 'r', 'LineWidth', 1.2);
grid on;
ylabel('angle');
title('无控制开环：H(s)=1/(s^2-g/L)，有右半平面极点，角度发散');

subplot(3, 1, 3);
plot(closed_angle.time, closed_angle.signals.values, 'b', 'LineWidth', 1.2);
grid on;
ylabel('angle');
xlabel('time / s');
title('加入 PD：分母 s^2+(k2/L)s+(k1-g)/L，扰动后角度恢复');

outPng = fullfile(pwd, 'a43_balance_model_verification_result.png');
exportgraphics(gcf, outPng, 'Resolution', 160);

figure('Name', 'A43 Pole Map', 'Color', 'w');
plot(real(open_poles), imag(open_poles), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
plot(real(closed_poles), imag(closed_poles), 'bo', 'MarkerSize', 9, 'LineWidth', 2);
xline(0, '--k', 'LineWidth', 1);
yline(0, ':k', 'LineWidth', 1);
grid on;
xlabel('Real');
ylabel('Imag');
legend('Open-loop poles', 'Closed-loop poles', 'Location', 'best');
title('极点位置：开环有右半平面极点，PD 后极点进入左半平面');

outPolePng = fullfile(pwd, 'a43_pole_map.png');
exportgraphics(gcf, outPolePng, 'Resolution', 160);

fprintf('\nSimulation finished:\n');
fprintf('%s\n', outPng);
fprintf('%s\n', outPolePng);
