function plot_sm2(results, s)
% PLOT_SM2  Ve hinh cho SM2: AC+PE vs AC-PE vs CL vs BS
%
% 7 figures:
%   1-2. XY trajectory (circle, line)
%   3-4. Tracking error (circle, line)
%   5.   Weight convergence: AC+PE vs AC-PE vs CL (circle)
%   6.   Bellman error: AC+PE vs AC-PE vs CL (circle)
%   7.   Running cost comparison (circle)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

colors.bs      = [1 0 0];         % red
colors.adp_pe  = [0 0 1];         % blue
colors.adp_nope = [0.6 0.6 0.6];  % gray
colors.cl      = [0 0.7 0];       % green

methods = {'bs', 'adp_pe', 'adp_nope', 'cl'};
labels  = {'BS', 'AC+PE', 'AC(no PE)', 'CL(no PE)'};
n_m = length(methods);
fig_num = 0;

for traj = {'circle', 'line'}
    tname = traj{1};

    %% XY trajectory
    fig_num = fig_num + 1;
    figure('Name', sprintf('SM2 - XY %s', tname), ...
           'Position', [50+400*(fig_num-1) 500 550 450]);

    d = results.([tname '_bs']);
    plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
    for i = 1:n_m
        d = results.([tname '_' methods{i}]);
        plot(d.q(1,:), d.q(2,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1.2);
    end
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Quy dao XY - %s', upper(tname)));
    legend(['Ref', labels], 'Location', 'best');
    axis equal; grid on;

    %% Tracking error
    fig_num = fig_num + 1;
    figure('Name', sprintf('SM2 - Error %s', tname), ...
           'Position', [50+400*(fig_num-1) 50 650 520]);

    zlabels = {'$z_x$ [m]', '$z_y$ [m]', '$z_\theta$ [rad]'};
    for j = 1:3
        subplot(3,1,j);
        for i = 1:n_m
            d = results.([tname '_' methods{i}]);
            plot(d.t, d.z(j,:), '-', 'Color', colors.(methods{i}), 'LineWidth', 1);
            hold on;
        end
        ylabel(zlabels{j}, 'Interpreter', 'latex');
        if j == 1
            title(sprintf('Sai so tracking - %s', upper(tname)));
            legend(labels, 'Location', 'ne');
        end
        if j == 3, xlabel('t [s]'); end
        grid on;
    end
end

%% Weight convergence (circle only)
fig_num = fig_num + 1;
figure('Name', 'SM2 - Weights', 'Position', [50 300 800 600]);
w_colors = lines(6);

% AC + PE: Critic weights
subplot(3,1,1);
d = results.circle_adp_pe;
for i = 1:s.l
    plot(d.t, d.Wc(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
    hold on;
end
ylabel('W_c'); title('AC + PE (Critic weights)');
xline(s.pe_off_time, 'k--', 'PE off', 'LineWidth', 1);
grid on;

% AC - PE: Critic weights
subplot(3,1,2);
d = results.circle_adp_nope;
for i = 1:s.l
    plot(d.t, d.Wc(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
    hold on;
end
ylabel('W_c'); title('AC (no PE) -- weights KHONG hoi tu');
grid on;

% CL: W
subplot(3,1,3);
d = results.circle_cl;
for i = 1:s.l
    plot(d.t, d.W(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
    hold on;
end
ylabel('W'); xlabel('t [s]');
title('CL (no PE) -- hoi tu KHONG can PE');
grid on;

%% Bellman error (circle)
fig_num = fig_num + 1;
figure('Name', 'SM2 - Bellman Error', 'Position', [400 300 800 450]);
window = min(2000, length(results.circle_adp_pe.delta));

subplot(1,3,1);
d = results.circle_adp_pe;
delta_sm = movmean(d.delta, window);
plot(d.t, d.delta, 'Color', [0.7 0.7 1], 'LineWidth', 0.3); hold on;
plot(d.t, delta_sm, 'b-', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('\delta');
title('AC + PE'); grid on;
xline(s.pe_off_time, 'k--', 'PE off');

subplot(1,3,2);
d = results.circle_adp_nope;
delta_sm = movmean(d.delta, window);
plot(d.t, d.delta, 'Color', [0.8 0.8 0.8], 'LineWidth', 0.3); hold on;
plot(d.t, delta_sm, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('\delta');
title('AC (no PE)'); grid on;

subplot(1,3,3);
d = results.circle_cl;
delta_sm = movmean(d.delta, window);
plot(d.t, d.delta, 'Color', [0.7 1 0.7], 'LineWidth', 0.3); hold on;
plot(d.t, delta_sm, 'Color', [0 0.5 0], 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('\delta');
title('CL (no PE)'); grid on;

%% Running cost (circle)
fig_num = fig_num + 1;
figure('Name', 'SM2 - Running Cost', 'Position', [100 100 700 400]);

for i = 1:n_m
    d = results.(['circle_' methods{i}]);
    semilogy(d.t, d.cost_running + 1e-10, '-', ...
        'Color', colors.(methods{i}), 'LineWidth', 1.2);
    hold on;
end
xlabel('t [s]'); ylabel('Running cost (log)');
title('z^TQz + u_o^TRu_o -- Circle');
legend(labels, 'Location', 'northeast');
grid on;

fprintf('SM2: da ve %d figures.\n', fig_num);

end
