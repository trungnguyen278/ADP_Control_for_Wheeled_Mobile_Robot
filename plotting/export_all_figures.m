%% EXPORT_ALL_FIGURES  Xuat hinh SM2 + Thesis sang PDF
%
% Chay SAU sim_sm2.m va sim_thesis.m
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

clear; close all; clc;
addpath('../simulations', '../models');

set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.2);

%% ================================================================
%  SM2 FIGURES
%  ================================================================
fprintf('=== EXPORTING SM2 FIGURES ===\n');
load('../results/sm2_results.mat', 'results', 's');
fig_dir = '../docs/reports/sm2/figures';

colors.bs      = [1 0 0];
colors.adp_pe  = [0 0 1];
colors.adp_nope = [0.6 0.6 0.6];
colors.cl      = [0 0.7 0];
methods = {'bs', 'adp_pe', 'adp_nope', 'cl'};
labels  = {'BS', 'AC+PE', 'AC(no PE)', 'CL(no PE)'};
zlabels = {'$z_x$ [m]', '$z_y$ [m]', '$z_\theta$ [rad]'};

% SM2-1: XY circle
f = figure('Position', [100 100 520 420], 'Visible', 'off');
d = results.circle_bs;
plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
for i = 1:4
    d = results.(['circle_' methods{i}]);
    plot(d.q(1,:), d.q(2,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1.2);
end
xlabel('$x$ [m]', 'Interpreter', 'latex'); ylabel('$y$ [m]', 'Interpreter', 'latex');
legend(['Ref', labels], 'Location', 'best'); axis equal; grid on;
exportgraphics(f, fullfile(fig_dir, 'sm2_xy_circle.pdf'), 'ContentType', 'vector');
fprintf('  sm2_xy_circle.pdf\n');

% SM2-2: XY line
f = figure('Position', [100 100 520 420], 'Visible', 'off');
d = results.line_bs;
plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
for i = 1:4
    d = results.(['line_' methods{i}]);
    plot(d.q(1,:), d.q(2,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1.2);
end
xlabel('$x$ [m]', 'Interpreter', 'latex'); ylabel('$y$ [m]', 'Interpreter', 'latex');
legend(['Ref', labels], 'Location', 'best'); axis equal; grid on;
exportgraphics(f, fullfile(fig_dir, 'sm2_xy_line.pdf'), 'ContentType', 'vector');
fprintf('  sm2_xy_line.pdf\n');

% SM2-3: Error circle
f = figure('Position', [100 100 560 480], 'Visible', 'off');
for j = 1:3
    subplot(3,1,j);
    for i = 1:4
        d = results.(['circle_' methods{i}]);
        plot(d.t, d.z(j,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1); hold on;
    end
    ylabel(zlabels{j}, 'Interpreter', 'latex');
    if j == 1, legend(labels, 'Location', 'ne'); end
    if j == 3, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
    grid on;
end
exportgraphics(f, fullfile(fig_dir, 'sm2_error_circle.pdf'), 'ContentType', 'vector');
fprintf('  sm2_error_circle.pdf\n');

% SM2-4: Weights comparison (AC+PE vs AC-PE vs CL)
f = figure('Position', [100 100 640 520], 'Visible', 'off');
w_colors = lines(6);

subplot(3,1,1);
d = results.circle_adp_pe;
for i = 1:6, plot(d.t, d.Wc(i,:), '-', 'Color', w_colors(i,:)); hold on; end
ylabel('$W_c$', 'Interpreter', 'latex'); title('AC + PE (Critic)');
xline(s.pe_off_time, 'k--'); grid on;

subplot(3,1,2);
d = results.circle_adp_nope;
for i = 1:6, plot(d.t, d.Wc(i,:), '-', 'Color', w_colors(i,:)); hold on; end
ylabel('$W_c$', 'Interpreter', 'latex'); title('AC (no PE)'); grid on;

subplot(3,1,3);
d = results.circle_cl;
for i = 1:6, plot(d.t, d.W(i,:), '-', 'Color', w_colors(i,:)); hold on; end
ylabel('$W$', 'Interpreter', 'latex'); title('CL (no PE)');
xlabel('$t$ [s]', 'Interpreter', 'latex'); grid on;

exportgraphics(f, fullfile(fig_dir, 'sm2_weights.pdf'), 'ContentType', 'vector');
fprintf('  sm2_weights.pdf\n');

% SM2-5: Bellman error
f = figure('Position', [100 100 700 280], 'Visible', 'off');
window = 2000;

subplot(1,3,1);
d = results.circle_adp_pe;
plot(d.t, movmean(d.delta, window), 'b-', 'LineWidth', 1.5);
xlabel('$t$ [s]', 'Interpreter', 'latex'); ylabel('$\delta$', 'Interpreter', 'latex');
title('AC + PE'); grid on; xline(s.pe_off_time, 'k--');

subplot(1,3,2);
d = results.circle_adp_nope;
plot(d.t, movmean(d.delta, window), 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
xlabel('$t$ [s]', 'Interpreter', 'latex'); title('AC (no PE)'); grid on;

subplot(1,3,3);
d = results.circle_cl;
plot(d.t, movmean(d.delta, window), 'Color', [0 0.5 0], 'LineWidth', 1.5);
xlabel('$t$ [s]', 'Interpreter', 'latex'); title('CL (no PE)'); grid on;

exportgraphics(f, fullfile(fig_dir, 'sm2_bellman.pdf'), 'ContentType', 'vector');
fprintf('  sm2_bellman.pdf\n');

% SM2-6: Running cost
f = figure('Position', [100 100 520 320], 'Visible', 'off');
for i = 1:4
    d = results.(['circle_' methods{i}]);
    semilogy(d.t, d.cost_running + 1e-10, '-', 'Color', colors.(methods{i}), 'LineWidth', 1.2);
    hold on;
end
xlabel('$t$ [s]', 'Interpreter', 'latex'); ylabel('Running cost (log)');
legend(labels, 'Location', 'ne'); grid on;
exportgraphics(f, fullfile(fig_dir, 'sm2_cost.pdf'), 'ContentType', 'vector');
fprintf('  sm2_cost.pdf\n');

%% ================================================================
%  THESIS FIGURES
%  ================================================================
fprintf('\n=== EXPORTING THESIS FIGURES ===\n');
clear results;
load('../results/thesis_results.mat', 'results_A', 'results_B', 'results_C', 's', 'p');
fig_dir = '../docs/thesis/figures';

colors5.bs     = [1 0 0];
colors5.adp    = [0 0 1];
colors5.smc    = [0 0.6 0];
colors5.cl     = [0.8 0 0.8];
colors5.adp_ft = [0.9 0.5 0];
m5 = {'bs', 'adp', 'smc', 'cl', 'adp_ft'};
l5 = {'BS', 'ADP-AC', 'SMC', 'CL', 'ADP-FT'};
trajs = {'circle', 'line', 'figure8'};
traj_titles = {'Circle', 'Line', 'Figure-8'};

% TH-1,2,3: XY trajectory (3 quy dao)
for t_idx = 1:3
    f = figure('Position', [100 100 500 420], 'Visible', 'off');
    d = results_A.([trajs{t_idx} '_bs']);
    plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
    for i = 1:5
        d = results_A.([trajs{t_idx} '_' m5{i}]);
        plot(d.q(1,:), d.q(2,:), '-', 'Color', colors5.(m5{i}), 'LineWidth', 1.2);
    end
    xlabel('$x$ [m]', 'Interpreter', 'latex'); ylabel('$y$ [m]', 'Interpreter', 'latex');
    legend(['Ref', l5], 'Location', 'best'); axis equal; grid on;
    exportgraphics(f, fullfile(fig_dir, sprintf('xy_%s.pdf', trajs{t_idx})), 'ContentType', 'vector');
    fprintf('  xy_%s.pdf\n', trajs{t_idx});
end

% TH-4,5,6: Error (3 quy dao)
for t_idx = 1:3
    f = figure('Position', [100 100 600 480], 'Visible', 'off');
    for j = 1:3
        subplot(3,1,j);
        for i = 1:5
            d = results_A.([trajs{t_idx} '_' m5{i}]);
            plot(d.t, d.z(j,:), '-', 'Color', colors5.(m5{i}), 'LineWidth', 1); hold on;
        end
        ylabel(zlabels{j}, 'Interpreter', 'latex');
        if j == 1, legend(l5, 'Location', 'ne'); end
        if j == 3, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
        grid on;
    end
    exportgraphics(f, fullfile(fig_dir, sprintf('error_%s.pdf', trajs{t_idx})), 'ContentType', 'vector');
    fprintf('  error_%s.pdf\n', trajs{t_idx});
end

% TH-7: Bar chart Jc
f = figure('Position', [100 100 650 380], 'Visible', 'off');
Jc_mat = zeros(3, 5);
for t_idx = 1:3
    for mi = 1:5
        Jc_mat(t_idx, mi) = results_A.([trajs{t_idx} '_' m5{mi}]).Jc;
    end
end
Jc_mat_plot = Jc_mat; Jc_mat_plot(3,3) = 500; % cap SMC figure8 for readability
b = bar(Jc_mat_plot);
bar_colors = [colors5.bs; colors5.adp; colors5.smc; colors5.cl; colors5.adp_ft];
for i = 1:5, b(i).FaceColor = bar_colors(i,:); end
set(gca, 'XTickLabel', traj_titles);
ylabel('$J_c$', 'Interpreter', 'latex');
legend(l5, 'Location', 'northwest'); grid on;
exportgraphics(f, fullfile(fig_dir, 'bar_jc_traj.pdf'), 'ContentType', 'vector');
fprintf('  bar_jc_traj.pdf\n');

% TH-8: Weight evolution ADP-FT
f = figure('Position', [100 100 640 480], 'Visible', 'off');
for t_idx = 1:3
    subplot(3,1,t_idx);
    d = results_A.([trajs{t_idx} '_adp_ft']);
    for i = 1:6, plot(d.t, d.W(i,:), '-', 'Color', w_colors(i,:)); hold on; end
    ylabel('$W$', 'Interpreter', 'latex');
    title(traj_titles{t_idx});
    if t_idx == 3, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
    grid on;
end
exportgraphics(f, fullfile(fig_dir, 'weights_adp_ft.pdf'), 'ContentType', 'vector');
fprintf('  weights_adp_ft.pdf\n');

% TH-9: Robustness mass - ||z|| vs time
colors3.bs = [1 0 0]; colors3.cl = [0.8 0 0.8]; colors3.adp_ft = [0.9 0.5 0];
rm = {'bs', 'cl', 'adp_ft'}; rl = {'BS', 'CL', 'ADP-FT'};
masses = [10, 12, 14];

f = figure('Position', [100 100 640 480], 'Visible', 'off');
for mi = 1:3
    subplot(3,1,mi);
    for m_idx = 1:3
        tag = sprintf('m%d_%s', masses(mi), rm{m_idx});
        d = results_B.(tag);
        z_norm = sqrt(sum(d.z.^2, 1));
        plot(d.t, z_norm, '-', 'Color', colors3.(rm{m_idx}), 'LineWidth', 1.2); hold on;
    end
    ylabel('$\|z\|$', 'Interpreter', 'latex');
    title(sprintf('$m = %d$ kg', masses(mi)), 'Interpreter', 'latex');
    if mi == 1, legend(rl, 'Location', 'ne'); end
    if mi == 3, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
    grid on; ylim([0 1]);
end
exportgraphics(f, fullfile(fig_dir, 'robust_mass.pdf'), 'ContentType', 'vector');
fprintf('  robust_mass.pdf\n');

% TH-10: Bar chart Jc vs mass
f = figure('Position', [100 100 480 340], 'Visible', 'off');
Jc_mass = zeros(3,3);
for mi = 1:3
    for m_idx = 1:3
        tag = sprintf('m%d_%s', masses(mi), rm{m_idx});
        Jc_mass(mi, m_idx) = results_B.(tag).Jc;
    end
end
Jc_mass_plot = min(Jc_mass, 500); % cap for readability
b = bar(Jc_mass_plot);
rob_colors = [colors3.bs; colors3.cl; colors3.adp_ft];
for i = 1:3, b(i).FaceColor = rob_colors(i,:); end
set(gca, 'XTickLabel', {'10 kg', '12 kg', '14 kg'});
ylabel('$J_c$', 'Interpreter', 'latex');
legend(rl, 'Location', 'northwest'); grid on;
exportgraphics(f, fullfile(fig_dir, 'bar_jc_mass.pdf'), 'ContentType', 'vector');
fprintf('  bar_jc_mass.pdf\n');

% TH-11: Robustness disturbance - ||z|| vs time
dist_amps = [0, 0.2, 0.4, 0.6];
f = figure('Position', [100 100 640 550], 'Visible', 'off');
for di = 1:4
    subplot(4,1,di);
    for m_idx = 1:3
        tag = sprintf('d%d_%s', round(dist_amps(di)*100), rm{m_idx});
        d = results_C.(tag);
        z_norm = sqrt(sum(d.z.^2, 1));
        plot(d.t, z_norm, '-', 'Color', colors3.(rm{m_idx}), 'LineWidth', 1.2); hold on;
    end
    ylabel('$\|z\|$', 'Interpreter', 'latex');
    title(sprintf('$d = %.0f\\%%\\,\\tau_{\\max}$', dist_amps(di)*100), 'Interpreter', 'latex');
    if di == 1, legend(rl, 'Location', 'ne'); end
    if di == 4, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
    grid on; ylim([0 0.6]);
end
exportgraphics(f, fullfile(fig_dir, 'robust_dist.pdf'), 'ContentType', 'vector');
fprintf('  robust_dist.pdf\n');

% TH-12: Bar chart Jc vs disturbance
f = figure('Position', [100 100 480 340], 'Visible', 'off');
Jc_dist = zeros(4,3);
for di = 1:4
    for m_idx = 1:3
        tag = sprintf('d%d_%s', round(dist_amps(di)*100), rm{m_idx});
        Jc_dist(di, m_idx) = results_C.(tag).Jc;
    end
end
b = bar(Jc_dist);
for i = 1:3, b(i).FaceColor = rob_colors(i,:); end
set(gca, 'XTickLabel', {'0%', '20%', '40%', '60%'});
xlabel('Disturbance amplitude ($\%\,\tau_{\max}$)', 'Interpreter', 'latex');
ylabel('$J_c$', 'Interpreter', 'latex');
legend(rl, 'Location', 'northwest'); grid on;
exportgraphics(f, fullfile(fig_dir, 'bar_jc_dist.pdf'), 'ContentType', 'vector');
fprintf('  bar_jc_dist.pdf\n');

%% Also export SM1 full figures
fprintf('\n=== EXPORTING SM1 FULL FIGURES ===\n');
clear results_A results_B results_C;
load('../results/sm1_full_results.mat', 'results', 's', 'p');
fig_dir_th = '../docs/thesis/figures';

% Torque + disturbance (ADP-FT, dist)
f = figure('Position', [100 100 560 380], 'Visible', 'off');
d = results.adp_ft_dist;
subplot(2,1,1);
plot(d.t, d.tau(1,:), 'b-', d.t, d.tau(2,:), 'r-', 'LineWidth', 0.8);
ylabel('$\tau$ [N$\cdot$m]', 'Interpreter', 'latex');
legend('$\tau_R$', '$\tau_L$', 'Interpreter', 'latex', 'Location', 'ne'); grid on;
subplot(2,1,2);
plot(d.t, d.dist(1,:), 'm-', 'LineWidth', 0.8);
ylabel('$d(t)$ [N$\cdot$m]', 'Interpreter', 'latex');
xlabel('$t$ [s]', 'Interpreter', 'latex'); grid on;
exportgraphics(f, fullfile(fig_dir_th, 'torque_dist.pdf'), 'ContentType', 'vector');
fprintf('  torque_dist.pdf\n');

fprintf('\n=== ALL FIGURES EXPORTED ===\n');
