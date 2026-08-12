function plot_thesis(results_A, results_B, results_C, s, p, ...
                     methods, method_names, rob_methods, rob_names, ...
                     trajs, masses, dist_amps)
% PLOT_THESIS  Ve hinh tong hop cho luan van
%
% Phan A: 3 quy dao x 5 phuong phap
%   Fig 1-3: XY trajectory (circle, line, figure8)
%   Fig 4-6: Tracking error (circle, line, figure8)
%   Fig 7:   Bar chart Jc comparison
%   Fig 8:   Weight evolution ADP-FT (3 quy dao)
%
% Phan B: Robustness khoi luong
%   Fig 9:   Tracking error ||z|| vs mass (BS, CL, ADP-FT)
%   Fig 10:  Bar chart Jc vs mass
%
% Phan C: Robustness nhieu
%   Fig 11:  Tracking error ||z|| vs dist amp (BS, CL, ADP-FT)
%   Fig 12:  Bar chart Jc vs dist amp
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

colors_5.bs     = [1 0 0];
colors_5.adp    = [0 0 1];
colors_5.smc    = [0 0.6 0];
colors_5.cl     = [0.8 0 0.8];
colors_5.adp_ft = [0.9 0.5 0];

colors_3.bs     = [1 0 0];
colors_3.cl     = [0.8 0 0.8];
colors_3.adp_ft = [0.9 0.5 0];

n_methods = length(methods);
fig_num = 0;
traj_titles = {'Circle', 'Line', 'Figure-8'};

%% ================================================================
%  PHAN A: XY trajectory (3 quy dao)
%  ================================================================
for t_idx = 1:length(trajs)
    fig_num = fig_num + 1;
    figure('Name', sprintf('A - XY %s', trajs{t_idx}), ...
           'Position', [50+350*(t_idx-1) 500 500 420]);

    d = results_A.([trajs{t_idx} '_bs']);
    plot(d.qr(1,:), d.qr(2,:), 'k--', 'LineWidth', 1.5); hold on;
    for i = 1:n_methods
        d = results_A.([trajs{t_idx} '_' methods{i}]);
        plot(d.q(1,:), d.q(2,:), '-', 'Color', colors_5.(methods{i}), 'LineWidth', 1.2);
    end
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('%s (co nhieu)', traj_titles{t_idx}));
    legend(['Ref', method_names], 'Location', 'best');
    axis equal; grid on;
end

%% Tracking error (3 quy dao)
zlabels = {'$z_x$ [m]', '$z_y$ [m]', '$z_\theta$ [rad]'};
for t_idx = 1:length(trajs)
    fig_num = fig_num + 1;
    figure('Name', sprintf('A - Error %s', trajs{t_idx}), ...
           'Position', [50+350*(t_idx-1) 50 600 480]);

    for j = 1:3
        subplot(3,1,j);
        for i = 1:n_methods
            d = results_A.([trajs{t_idx} '_' methods{i}]);
            plot(d.t, d.z(j,:), '-', 'Color', colors_5.(methods{i}), 'LineWidth', 1);
            hold on;
        end
        ylabel(zlabels{j}, 'Interpreter', 'latex');
        if j == 1
            title(sprintf('Sai so tracking - %s (co nhieu)', traj_titles{t_idx}));
            legend(method_names, 'Location', 'ne');
        end
        if j == 3, xlabel('t [s]'); end
        grid on;
    end
end

%% Bar chart Jc (Phan A)
fig_num = fig_num + 1;
figure('Name', 'A - Jc Comparison', 'Position', [100 300 700 400]);

Jc_mat = zeros(length(trajs), n_methods);
for t_idx = 1:length(trajs)
    for m_idx = 1:n_methods
        d = results_A.([trajs{t_idx} '_' methods{m_idx}]);
        Jc_mat(t_idx, m_idx) = d.Jc;
    end
end

b = bar(Jc_mat);
bar_colors = [colors_5.bs; colors_5.adp; colors_5.smc; colors_5.cl; colors_5.adp_ft];
for i = 1:n_methods
    b(i).FaceColor = bar_colors(i,:);
end
set(gca, 'XTickLabel', traj_titles);
ylabel('J_c (tich luy cost)');
title('So sanh chi phi tich luy (co nhieu)');
legend(method_names, 'Location', 'northwest');
grid on;

%% Weight evolution ADP-FT (3 quy dao)
fig_num = fig_num + 1;
figure('Name', 'A - ADP-FT Weights', 'Position', [400 200 700 550]);
w_colors = lines(6);

for t_idx = 1:length(trajs)
    subplot(length(trajs), 1, t_idx);
    d = results_A.([trajs{t_idx} '_adp_ft']);
    for i = 1:s.l
        plot(d.t, d.W(i,:), '-', 'Color', w_colors(i,:), 'LineWidth', 0.9);
        hold on;
    end
    ylabel('W');
    title(sprintf('ADP-FT weights - %s', traj_titles{t_idx}));
    if t_idx == 1
        legend('W_1','W_2','W_3','W_4','W_5','W_6', ...
            'Location', 'ne', 'NumColumns', 3);
    end
    if t_idx == length(trajs), xlabel('t [s]'); end
    grid on;
end

%% ================================================================
%  PHAN B: Robustness khoi luong
%  ================================================================

% ||z|| vs time cho moi mass (3 subplot, moi subplot 1 mass, 3 methods)
fig_num = fig_num + 1;
figure('Name', 'B - Mass Robustness', 'Position', [50 100 750 550]);

for mi = 1:length(masses)
    subplot(length(masses), 1, mi);
    for m_idx = 1:length(rob_methods)
        tag = sprintf('m%d_%s', masses(mi), rob_methods{m_idx});
        d = results_B.(tag);
        z_norm = sqrt(sum(d.z.^2, 1));
        plot(d.t, z_norm, '-', 'Color', colors_3.(rob_methods{m_idx}), 'LineWidth', 1.2);
        hold on;
    end
    ylabel('$\|z\|$', 'Interpreter', 'latex');
    title(sprintf('m = %d kg', masses(mi)));
    if mi == 1
        legend(rob_names, 'Location', 'ne');
    end
    if mi == length(masses), xlabel('t [s]'); end
    grid on;
    ylim([0, 0.8]);
end

% Bar chart Jc vs mass
fig_num = fig_num + 1;
figure('Name', 'B - Jc vs Mass', 'Position', [500 300 500 380]);

Jc_mass = zeros(length(masses), length(rob_methods));
for mi = 1:length(masses)
    for m_idx = 1:length(rob_methods)
        tag = sprintf('m%d_%s', masses(mi), rob_methods{m_idx});
        Jc_mass(mi, m_idx) = results_B.(tag).Jc;
    end
end

b = bar(Jc_mass);
rob_bar_colors = [colors_3.bs; colors_3.cl; colors_3.adp_ft];
for i = 1:length(rob_methods)
    b(i).FaceColor = rob_bar_colors(i,:);
end
mass_labels = arrayfun(@(m) sprintf('%d kg', m), masses, 'UniformOutput', false);
set(gca, 'XTickLabel', mass_labels);
ylabel('J_c');
title('Robustness: chi phi vs khoi luong');
legend(rob_names, 'Location', 'northwest');
grid on;

%% ================================================================
%  PHAN C: Robustness nhieu
%  ================================================================

% ||z|| vs time cho moi dist_amp
fig_num = fig_num + 1;
figure('Name', 'C - Dist Robustness', 'Position', [50 50 750 600]);

for di = 1:length(dist_amps)
    subplot(length(dist_amps), 1, di);
    for m_idx = 1:length(rob_methods)
        tag = sprintf('d%d_%s', round(dist_amps(di)*100), rob_methods{m_idx});
        d = results_C.(tag);
        z_norm = sqrt(sum(d.z.^2, 1));
        plot(d.t, z_norm, '-', 'Color', colors_3.(rob_methods{m_idx}), 'LineWidth', 1.2);
        hold on;
    end
    ylabel('$\|z\|$', 'Interpreter', 'latex');
    title(sprintf('d = %.0f%% \\tau_{max}', dist_amps(di)*100));
    if di == 1
        legend(rob_names, 'Location', 'ne');
    end
    if di == length(dist_amps), xlabel('t [s]'); end
    grid on;
    ylim([0, 0.8]);
end

% Bar chart Jc vs dist amp
fig_num = fig_num + 1;
figure('Name', 'C - Jc vs Dist', 'Position', [500 100 500 380]);

Jc_dist = zeros(length(dist_amps), length(rob_methods));
for di = 1:length(dist_amps)
    for m_idx = 1:length(rob_methods)
        tag = sprintf('d%d_%s', round(dist_amps(di)*100), rob_methods{m_idx});
        Jc_dist(di, m_idx) = results_C.(tag).Jc;
    end
end

b = bar(Jc_dist);
for i = 1:length(rob_methods)
    b(i).FaceColor = rob_bar_colors(i,:);
end
amp_labels = arrayfun(@(a) sprintf('%.0f%%', a*100), dist_amps, 'UniformOutput', false);
set(gca, 'XTickLabel', amp_labels);
xlabel('Bien do nhieu (% \tau_{max})');
ylabel('J_c');
title('Robustness: chi phi vs nhieu');
legend(rob_names, 'Location', 'northwest');
grid on;

fprintf('Thesis: da ve %d figures.\n', fig_num);

end
