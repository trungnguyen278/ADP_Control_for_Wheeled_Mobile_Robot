function plot_sm1_full(results, s, p)
% PLOT_SM1_FULL  Ve hinh so sanh 5 controller tren full model
%
% 6 figures:
%   1. XY trajectory (khong nhieu)
%   2. XY trajectory (co nhieu)
%   3. Tracking error (khong nhieu)
%   4. Tracking error (co nhieu)
%   5. Mo-men + nhieu (BS, co nhieu)
%   6. Weight evolution (ADP-AC, CL, ADP-FT)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

colors.bs     = [1 0 0];           % red
colors.adp    = [0 0 1];           % blue
colors.smc    = [0 0.6 0];         % green
colors.cl     = [0.8 0 0.8];       % magenta
colors.adp_ft = [0.9 0.5 0];       % orange

methods = {'bs', 'adp', 'smc', 'cl', 'adp_ft'};
labels  = {'BS', 'ADP-AC', 'SMC', 'CL', 'ADP-FT'};
n_methods = length(methods);
fig_num = 0;

%% 1. Quy dao XY — khong nhieu
fig_num = fig_num + 1;
figure('Name', 'XY - No Disturbance', 'Position', [50 500 560 450]);

d = results.bs_nodist;
plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
for i = 1:n_methods
    d = results.([methods{i} '_nodist']);
    plot(d.q(1,:), d.q(2,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1.2);
end
xlabel('x [m]'); ylabel('y [m]');
title('Quy dao XY (khong nhieu)');
legend(['Ref', labels], 'Location', 'best');
axis equal; grid on;

%% 2. Quy dao XY — co nhieu
fig_num = fig_num + 1;
figure('Name', 'XY - With Disturbance', 'Position', [620 500 560 450]);

d = results.bs_dist;
plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
for i = 1:n_methods
    d = results.([methods{i} '_dist']);
    plot(d.q(1,:), d.q(2,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1.2);
end
xlabel('x [m]'); ylabel('y [m]');
title('Quy dao XY (co nhieu d(t))');
legend(['Ref', labels], 'Location', 'best');
axis equal; grid on;

%% 3. Sai so tracking — khong nhieu
fig_num = fig_num + 1;
figure('Name', 'Error - No Disturbance', 'Position', [50 50 700 520]);

zlabels = {'$z_x$ [m]', '$z_y$ [m]', '$z_\theta$ [rad]'};
for j = 1:3
    subplot(3,1,j);
    for i = 1:n_methods
        d = results.([methods{i} '_nodist']);
        plot(d.t, d.z(j,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1);
        hold on;
    end
    ylabel(zlabels{j}, 'Interpreter', 'latex');
    if j == 1
        title('Sai so tracking (khong nhieu)');
        legend(labels, 'Location', 'ne');
    end
    if j == 3, xlabel('t [s]'); end
    grid on;
end

%% 4. Sai so tracking — co nhieu
fig_num = fig_num + 1;
figure('Name', 'Error - With Disturbance', 'Position', [760 50 700 520]);

for j = 1:3
    subplot(3,1,j);
    for i = 1:n_methods
        d = results.([methods{i} '_dist']);
        plot(d.t, d.z(j,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1);
        hold on;
    end
    ylabel(zlabels{j}, 'Interpreter', 'latex');
    if j == 1
        title('Sai so tracking (co nhieu d(t))');
        legend(labels, 'Location', 'ne');
    end
    if j == 3, xlabel('t [s]'); end
    grid on;
end

%% 5. Mo-men va nhieu (ADP-FT, co nhieu — de thay inner loop)
fig_num = fig_num + 1;
figure('Name', 'Torque + Disturbance', 'Position', [100 300 650 420]);
d = results.adp_ft_dist;

subplot(2,1,1);
plot(d.t, d.tau(1,:), 'b-', d.t, d.tau(2,:), 'r-', 'LineWidth', 0.8);
ylabel('\tau [N\cdotm]');
title('Mo-men dieu khien (ADP-FT, co nhieu)');
legend('\tau_R', '\tau_L', 'Location', 'ne');
grid on;

subplot(2,1,2);
plot(d.t, d.dist(1,:), 'm-', 'LineWidth', 0.8);
ylabel('d(t) [N\cdotm]');
xlabel('t [s]');
title(sprintf('Nhieu: %.1f\\tau_{max}\\cdotsin(%.0ft)', s.dist_amp, s.dist_freq));
grid on;

%% 6. Weight evolution (ADP-AC, CL, ADP-FT — khong nhieu)
fig_num = fig_num + 1;
figure('Name', 'Weight Evolution', 'Position', [400 200 700 550]);

w_labels = {'W_1','W_2','W_3','W_4','W_5','W_6'};
w_colors = lines(6);

% ADP Actor-Critic: Wc
subplot(3,1,1);
d = results.adp_nodist;
for i = 1:s.l
    plot(d.t, d.Wc(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
    hold on;
end
ylabel('W_c');
title('ADP Actor-Critic (Critic weights)');
legend(w_labels, 'Location', 'ne', 'NumColumns', 3);
grid on;

% Critic-only CL: W
subplot(3,1,2);
d = results.cl_nodist;
for i = 1:s.l
    plot(d.t, d.W(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
    hold on;
end
ylabel('W');
title('Critic-only CL (SM2)');
grid on;

% ADP Fixed-time: W
subplot(3,1,3);
d = results.adp_ft_nodist;
for i = 1:s.l
    plot(d.t, d.W(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
    hold on;
end
ylabel('W');
xlabel('t [s]');
title('ADP Fixed-time (Wang et al.)');
grid on;

fprintf('Da ve %d figures.\n', fig_num);

end
