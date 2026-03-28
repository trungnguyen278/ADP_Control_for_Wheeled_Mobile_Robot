%% EXPORT_SM1_FIGURES  Xuat hinh SM1 sang PDF cho bao cao LaTeX
%
% Chay SAU sim_sm1.m (can file results/sm1_results.mat)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

clear; close all; clc;
addpath('../simulations', '../models');

load('../results/sm1_results.mat', 'results', 's');
fig_dir = '../docs/sm1_report/figures';

set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.2);

%% 1. Quy dao XY — Circle
f = figure('Position', [100 100 520 420], 'Visible', 'off');
d_bc   = results.circle_bc;
d_adp  = results.circle_adp;
d_nope = results.circle_nope;

plot(d_bc.qr(1,:), d_bc.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
plot(d_bc.q(1,:), d_bc.q(2,:), 'r-', 'LineWidth', 1.2);
plot(d_adp.q(1,:), d_adp.q(2,:), 'b-', 'LineWidth', 1.2);
plot(d_nope.q(1,:), d_nope.q(2,:), 'm-.', 'LineWidth', 1.0);
xlabel('x [m]'); ylabel('y [m]');
legend('Reference', 'BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'best');
axis equal; grid on;
exportgraphics(f, fullfile(fig_dir, 'xy_circle.pdf'), 'ContentType', 'vector');
fprintf('Saved xy_circle.pdf\n');

%% 2. Quy dao XY — Line
f = figure('Position', [100 100 520 420], 'Visible', 'off');
d_bc   = results.line_bc;
d_adp  = results.line_adp;
d_nope = results.line_nope;

plot(d_bc.qr(1,:), d_bc.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
plot(d_bc.q(1,:), d_bc.q(2,:), 'r-', 'LineWidth', 1.2);
plot(d_adp.q(1,:), d_adp.q(2,:), 'b-', 'LineWidth', 1.2);
plot(d_nope.q(1,:), d_nope.q(2,:), 'm-.', 'LineWidth', 1.0);
xlabel('x [m]'); ylabel('y [m]');
legend('Reference', 'BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'best');
axis equal; grid on;
exportgraphics(f, fullfile(fig_dir, 'xy_line.pdf'), 'ContentType', 'vector');
fprintf('Saved xy_line.pdf\n');

%% 3. Sai so tracking — Circle
f = figure('Position', [100 100 560 500], 'Visible', 'off');
d_bc  = results.circle_bc;
d_adp = results.circle_adp;
d_nope = results.circle_nope;

labels_z = {'$z_x$ [m]', '$z_y$ [m]', '$z_\theta$ [rad]'};
for i = 1:3
    subplot(3,1,i);
    plot(d_bc.t, d_bc.z(i,:), 'r-'); hold on;
    plot(d_adp.t, d_adp.z(i,:), 'b-');
    plot(d_nope.t, d_nope.z(i,:), 'm-.');
    ylabel(labels_z{i}, 'Interpreter', 'latex');
    if i == 3, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
    if i == 1, legend('BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'ne'); end
    grid on;
end
exportgraphics(f, fullfile(fig_dir, 'error_circle.pdf'), 'ContentType', 'vector');
fprintf('Saved error_circle.pdf\n');

%% 4. Sai so tracking — Line
f = figure('Position', [100 100 560 500], 'Visible', 'off');
d_bc  = results.line_bc;
d_adp = results.line_adp;
d_nope = results.line_nope;

for i = 1:3
    subplot(3,1,i);
    plot(d_bc.t, d_bc.z(i,:), 'r-'); hold on;
    plot(d_adp.t, d_adp.z(i,:), 'b-');
    plot(d_nope.t, d_nope.z(i,:), 'm-.');
    ylabel(labels_z{i}, 'Interpreter', 'latex');
    if i == 3, xlabel('$t$ [s]', 'Interpreter', 'latex'); end
    if i == 1, legend('BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'ne'); end
    grid on;
end
exportgraphics(f, fullfile(fig_dir, 'error_line.pdf'), 'ContentType', 'vector');
fprintf('Saved error_line.pdf\n');

%% 5. NN Weights — PE vs No-PE (circle)
f = figure('Position', [100 100 640 480], 'Visible', 'off');
d_pe   = results.circle_adp;
d_nope = results.circle_nope;

subplot(2,2,1);
plot(d_pe.t, d_pe.Wc); ylabel('$W_c$', 'Interpreter', 'latex');
title('Critic (PE)'); grid on;
xline(s.pe_off_time, 'k--');

subplot(2,2,2);
plot(d_nope.t, d_nope.Wc); ylabel('$W_c$', 'Interpreter', 'latex');
title('Critic (no PE)'); grid on;

subplot(2,2,3);
plot(d_pe.t, d_pe.Wa); ylabel('$W_a$', 'Interpreter', 'latex');
xlabel('$t$ [s]', 'Interpreter', 'latex');
title('Actor (PE)'); grid on;
xline(s.pe_off_time, 'k--');

subplot(2,2,4);
plot(d_nope.t, d_nope.Wa); ylabel('$W_a$', 'Interpreter', 'latex');
xlabel('$t$ [s]', 'Interpreter', 'latex');
title('Actor (no PE)'); grid on;

exportgraphics(f, fullfile(fig_dir, 'weights_comparison.pdf'), 'ContentType', 'vector');
fprintf('Saved weights_comparison.pdf\n');

%% 6. Bellman error — PE vs No-PE
f = figure('Position', [100 100 560 280], 'Visible', 'off');
window = 1000;

subplot(1,2,1);
delta_sm = movmean(d_pe.delta, window);
plot(d_pe.t, delta_sm, 'b-', 'LineWidth', 1.5);
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$\delta$ (smoothed)', 'Interpreter', 'latex');
title('Bellman error (PE)'); grid on;
xline(s.pe_off_time, 'k--');

subplot(1,2,2);
delta_np = movmean(d_nope.delta, window);
plot(d_nope.t, delta_np, 'r-', 'LineWidth', 1.5);
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$\delta$ (smoothed)', 'Interpreter', 'latex');
title('Bellman error (no PE)'); grid on;

exportgraphics(f, fullfile(fig_dir, 'bellman_error.pdf'), 'ContentType', 'vector');
fprintf('Saved bellman_error.pdf\n');

%% 7. Running cost — Circle
f = figure('Position', [100 100 520 320], 'Visible', 'off');
d_bc  = results.circle_bc;
d_adp = results.circle_adp;

semilogy(d_bc.t, d_bc.cost_running + 1e-10, 'r-'); hold on;
semilogy(d_adp.t, d_adp.cost_running + 1e-10, 'b-');
semilogy(d_nope.t, d_nope.cost_running + 1e-10, 'm-.');
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('Running cost (log)', 'Interpreter', 'latex');
legend('BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'ne');
grid on;

exportgraphics(f, fullfile(fig_dir, 'running_cost.pdf'), 'ContentType', 'vector');
fprintf('Saved running_cost.pdf\n');

fprintf('\nDa xuat tat ca figures vao %s\n', fig_dir);
