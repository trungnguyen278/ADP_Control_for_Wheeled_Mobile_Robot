%% SIM_SM1  Seminar 1: Actor-Critic ADP vs Backstepping — Kinematic WMR Tracking
%
% Mo phong va so sanh 2 bo dieu khien tren kinematic model:
%   1. Backstepping Controller (BC) — Kanayama/Wang eq.31
%   2. Actor-Critic ADP           — Vamvoudakis & Lewis (2010)
%
% Quy dao test: circle va line
% Ket qua: hinh ve + bang cost Jc, Je
%
% Tham khao:
%   - Vamvoudakis & Lewis (2010): Online Actor-Critic ADP
%   - Wang et al. (2025): Error dynamics, basis functions, BC
%   - Kanayama (1990): Tracking error body frame
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

clear; close all; clc;
addpath('../models', '../controllers', '../plotting');

%% 1. Load tham so
s = sm1_params();

fprintf('\n========== SM1: Actor-Critic ADP vs Backstepping ==========\n');
fprintf('Q = diag(%.0f,%.0f,%.0f), R = diag(%.0f,%.0f)\n', ...
    s.Q(1,1), s.Q(2,2), s.Q(3,3), s.R(1,1), s.R(2,2));
fprintf('dt = %.4f s, T_sim = %.0f s\n', s.dt, s.T_sim);
fprintf('Initial offset: [%.2f, %.2f, %.2f]\n\n', s.q0_offset);

%% 2. Chay tung kich ban
results = struct();

for traj = {'circle', 'line'}
    s.traj_type = traj{1};
    fprintf('=== Quy dao: %s ===\n', upper(s.traj_type));

    % --- Backstepping ---
    fprintf('  Chay Backstepping...\n');
    data_bc = run_simulation('bc', s);
    results.([traj{1} '_bc']) = data_bc;
    fprintf('    Jc = %.4f, Je = %.4f\n', data_bc.Jc, data_bc.Je);

    % --- Actor-Critic ADP (co PE) ---
    fprintf('  Chay Actor-Critic ADP (co PE)...\n');
    data_adp = run_simulation('adp', s);
    results.([traj{1} '_adp']) = data_adp;
    fprintf('    Jc = %.4f, Je = %.4f\n', data_adp.Jc, data_adp.Je);

    % --- Actor-Critic ADP (KHONG PE) — chung minh PE can thiet ---
    fprintf('  Chay Actor-Critic ADP (KHONG PE)...\n');
    s_nope = s;
    s_nope.pe_off_time = 0;     % tat PE tu dau
    data_nope = run_simulation('adp', s_nope);
    results.([traj{1} '_nope']) = data_nope;
    fprintf('    Jc = %.4f, Je = %.4f\n', data_nope.Jc, data_nope.Je);

    fprintf('\n');
end

%% 3. Bang so sanh (giong Table I Wang)
fprintf('============ BANG SO SANH ============\n');
fprintf('%-12s | %-10s | %-10s | %-10s | %-10s\n', ...
    'Trajectory', 'Method', 'Jc', 'Je', 'z_rms');
fprintf('%s\n', repmat('-', 1, 58));
for traj = {'circle', 'line'}
    for method = {'bc', 'adp', 'nope'}
        d = results.([traj{1} '_' method{1}]);
        % RMS cua ||z|| trong 10s cuoi (steady-state)
        N_ss = round(10 / s.dt);
        z_ss = d.z(:, end-N_ss:end);
        z_rms = sqrt(mean(sum(z_ss.^2, 1)));
        if strcmp(method{1}, 'nope')
            label = 'ADP(no PE)';
        else
            label = upper(method{1});
        end
        fprintf('%-12s | %-10s | %10.4f | %10.4f | %10.6f\n', ...
            traj{1}, label, d.Jc, d.Je, z_rms);
    end
end
fprintf('\n');

%% 4. Ve hinh
plot_sm1_results(results, s);

%% 5. Luu ket qua
save('../results/sm1_results.mat', 'results', 's');
fprintf('Da luu ket qua vao results/sm1_results.mat\n');

fprintf('\n========== SM1 HOAN THANH ==========\n');

%% ====================================================================
%  HAM CON (local functions)
%  ====================================================================

function data = run_simulation(method, s)
% RUN_SIMULATION  Chay 1 kich ban mo phong kinematic tracking
%
% Input:
%   method -- 'bc' hoac 'adp'
%   s      -- sm1_params struct
%
% Output:
%   data -- struct chua tat ca ket qua

    N = round(s.T_sim / s.dt);
    dt = s.dt;

    %% Pre-allocate
    data.t    = zeros(1, N+1);
    data.q    = zeros(3, N+1);      % robot state [x;y;theta]
    data.qr   = zeros(3, N+1);      % reference
    data.z    = zeros(3, N+1);      % tracking error
    data.u    = zeros(2, N+1);      % control [v; omega]
    data.cost_running = zeros(1, N+1);  % z'Qz + uo'Ruo tai moi buoc
    data.method = method;

    is_adp = strcmp(method, 'adp');
    if is_adp
        data.Wc    = zeros(s.l, N+1);
        data.Wa    = zeros(s.l, N+1);
        data.delta = zeros(1, N+1);     % Bellman error
    end

    %% Dieu kien ban dau
    [qr0, ~, ~] = ref_trajectory(0, s);
    q = qr0 + s.q0_offset;
    q(3) = atan2(sin(q(3)), cos(q(3)));  % wrap theta

    % Khoi tao ADP weights
    if is_adp
        adp.Wc = s.Wc0;
        adp.Wa = s.Wa0;
    end

    %% Vong lap chinh (Euler integration)
    for k = 1:(N+1)
        t = (k-1) * dt;

        % Reference tai thoi diem t
        [qr, vr, omegar] = ref_trajectory(t, s);

        % Controller
        if is_adp
            [u, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s);
            data.Wc(:, k) = adp.Wc;
            data.Wa(:, k) = adp.Wa;
            data.delta(k)  = info.delta;
            z = info.z;
            uo = info.uo;
        else
            [u, z] = backstepping_controller(q, qr, vr, omegar, s);
            uf = [vr * cos(z(3)); omegar];
            uo = u - uf;
        end

        % Luu du lieu
        data.t(k)    = t;
        data.q(:,k)  = q;
        data.qr(:,k) = qr;
        data.z(:,k)  = z;
        data.u(:,k)  = u;
        data.cost_running(k) = z' * s.Q * z + uo' * s.R * uo;

        % Euler step (khong tich phan buoc cuoi)
        if k <= N
            dq = wmr_kinematics(t, q, u(1), u(2));
            q = q + dt * dq;
            q(3) = atan2(sin(q(3)), cos(q(3)));  % wrap theta
        end
    end

    %% Tinh cost tich luy
    data.Jc = trapz(data.t, data.cost_running);           % total cost
    data.Je = trapz(data.t, sum(data.z.^2, 1));           % tracking error cost
end
