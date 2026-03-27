function plot_sm1_results(results, s)
% PLOT_SM1_RESULTS  Ve tat ca hinh cho SM1: ADP vs BC vs No-PE
%
% plot_sm1_results(results, s)
%
% Input:
%   results -- struct chua: circle_bc, circle_adp, circle_nope,
%                           line_bc, line_adp, line_nope
%   s       -- sm1_params struct
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

fig_num = 0;

for traj = {'circle', 'line'}
    tname = traj{1};
    d_bc   = results.([tname '_bc']);
    d_adp  = results.([tname '_adp']);
    d_nope = results.([tname '_nope']);

    %% --- Figure: Quy dao XY ---
    fig_num = fig_num + 1;
    figure('Name', sprintf('SM1 - XY %s', tname), ...
           'Position', [50 + 400*(fig_num-1) 500 550 450]);

    plot(d_bc.qr(1,:), d_bc.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
    plot(d_bc.q(1,:), d_bc.q(2,:), 'r-', 'LineWidth', 1.2);
    plot(d_adp.q(1,:), d_adp.q(2,:), 'b-', 'LineWidth', 1.2);
    plot(d_nope.q(1,:), d_nope.q(2,:), 'm--', 'LineWidth', 1.0);

    plot(d_bc.q(1,1), d_bc.q(2,1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(d_adp.q(1,1), d_adp.q(2,1), 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Quy dao XY - %s', upper(tname)));
    legend('Reference', 'BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'best');
    axis equal; grid on;

    %% --- Figure: Sai so tracking ---
    fig_num = fig_num + 1;
    figure('Name', sprintf('SM1 - Error %s', tname), ...
           'Position', [50 + 400*(fig_num-1) 50 650 550]);

    labels_z = {'z_x [m]', 'z_y [m]', 'z_\theta [rad]'};
    for i = 1:3
        subplot(3,1,i);
        plot(d_bc.t, d_bc.z(i,:), 'r-', 'LineWidth', 1.2); hold on;
        plot(d_adp.t, d_adp.z(i,:), 'b-', 'LineWidth', 1.2);
        plot(d_nope.t, d_nope.z(i,:), 'm--', 'LineWidth', 1.0);
        ylabel(labels_z{i});
        if i == 1
            title(sprintf('Sai so tracking - %s', upper(tname)));
        end
        if i == 3
            xlabel('t [s]');
        end
        legend('BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'northeast');
        grid on;
    end
end

%% --- Figure: NN Weights convergence: PE vs No-PE ---
if isfield(results, 'circle_adp') && isfield(results, 'circle_nope')
    d_pe   = results.circle_adp;
    d_nope = results.circle_nope;

    fig_num = fig_num + 1;
    figure('Name', 'SM1 - Weights: PE vs No-PE', 'Position', [50 300 800 550]);

    % Critic weights — co PE
    subplot(2,2,1);
    plot(d_pe.t, d_pe.Wc, 'LineWidth', 1.2);
    ylabel('W_c'); title('Critic (co PE)');
    grid on; xline(s.pe_off_time, 'k--', 'PE off', 'LineWidth', 1);

    % Critic weights — khong PE
    subplot(2,2,2);
    plot(d_nope.t, d_nope.Wc, 'LineWidth', 1.2);
    ylabel('W_c'); title('Critic (KHONG PE)');
    grid on;

    % Actor weights — co PE
    subplot(2,2,3);
    plot(d_pe.t, d_pe.Wa, 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('W_a'); title('Actor (co PE)');
    grid on; xline(s.pe_off_time, 'k--', 'PE off', 'LineWidth', 1);

    % Actor weights — khong PE
    subplot(2,2,4);
    plot(d_nope.t, d_nope.Wa, 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('W_a'); title('Actor (KHONG PE)');
    grid on;
end

%% --- Figure: Bellman error PE vs No-PE ---
if isfield(results, 'circle_adp') && isfield(results, 'circle_nope')
    fig_num = fig_num + 1;
    figure('Name', 'SM1 - Bellman Error', 'Position', [400 300 700 400]);

    window = min(1000, length(d_pe.delta));

    subplot(1,2,1);
    delta_sm = movmean(d_pe.delta, window);
    plot(d_pe.t, d_pe.delta, 'Color', [0.7 0.7 1], 'LineWidth', 0.3); hold on;
    plot(d_pe.t, delta_sm, 'b-', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('\delta');
    title('Bellman error (co PE)');
    grid on; xline(s.pe_off_time, 'k--', 'PE off', 'LineWidth', 1);

    subplot(1,2,2);
    delta_sm_np = movmean(d_nope.delta, window);
    plot(d_nope.t, d_nope.delta, 'Color', [1 0.7 0.7], 'LineWidth', 0.3); hold on;
    plot(d_nope.t, delta_sm_np, 'r-', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('\delta');
    title('Bellman error (KHONG PE)');
    grid on;
end

%% --- Figure: Running cost ---
fig_num = fig_num + 1;
figure('Name', 'SM1 - Running Cost', 'Position', [100 100 700 400]);

d_bc   = results.circle_bc;
d_adp  = results.circle_adp;
d_nope = results.circle_nope;

semilogy(d_bc.t, d_bc.cost_running + 1e-10, 'r-', 'LineWidth', 1.2); hold on;
semilogy(d_adp.t, d_adp.cost_running + 1e-10, 'b-', 'LineWidth', 1.2);
semilogy(d_nope.t, d_nope.cost_running + 1e-10, 'm--', 'LineWidth', 1.0);
xlabel('t [s]'); ylabel('Running cost (log)');
title('Running cost z^TQz + u_o^TRu_o — Circle');
legend('BC', 'ADP (PE)', 'ADP (no PE)', 'Location', 'northeast');
grid on;

fprintf('Da ve %d figures.\n', fig_num);

end
