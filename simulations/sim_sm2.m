%% SIM_SM2  Seminar 2: Critic-only + Concurrent Learning vs Actor-Critic
%
% So sanh tren kinematic model (khong inner loop):
%   1. Backstepping (baseline)
%   2. Actor-Critic ADP co PE  (SM1 -- can PE de hoi tu)
%   3. Actor-Critic ADP KHONG PE (chung minh PE can thiet)
%   4. Critic-only + Concurrent Learning (SM2 -- KHONG can PE)
%
% Ket luan mong doi:
%   - AC + PE:   hoi tu tot (benchmark)
%   - AC - PE:   weights khong hoi tu, performance kem
%   - CL:        hoi tu tuong duong AC+PE ma KHONG can PE
%
% Quy dao: circle va line
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

clear; close all; clc;
addpath('../models', '../controllers', '../plotting');

%% 1. Load tham so
s = ctrl_params();
s.T_sim = 120;

fprintf('\n========== SM2: CRITIC-ONLY + CONCURRENT LEARNING ==========\n');
fprintf('Q = diag(%.0f,%.0f,%.0f), R = diag(%.0f,%.0f)\n', ...
    s.Q(1,1), s.Q(2,2), s.Q(3,3), s.R(1,1), s.R(2,2));
fprintf('AC: alpha_c=%.2f, alpha_a1=%.2f, PE off at %.0fs\n', ...
    s.alpha_c, s.alpha_a1, s.pe_off_time);
fprintf('CL: alpha=%.2f, alpha_hist=%.2f, stack=%d, NO PE\n\n', ...
    s.cl_alpha, s.cl_alpha_hist, s.cl_stack_max);

%% 2. Chay mo phong
methods = {'bs', 'adp_pe', 'adp_nope', 'cl'};
method_names = {'Backstepping', 'AC + PE', 'AC (no PE)', 'CL (no PE)'};
results = struct();

for traj = {'circle', 'line'}
    s.traj_type = traj{1};
    fprintf('=== Quy dao: %s ===\n', upper(s.traj_type));

    for m_idx = 1:length(methods)
        method = methods{m_idx};
        tag = sprintf('%s_%s', traj{1}, method);
        fprintf('  [%d/4] %s...', m_idx, method_names{m_idx});

        data = run_sm2_sim(method, s);
        results.(tag) = data;

        fprintf(' Jc=%.2f, Je=%.4f\n', data.Jc, data.Je);
    end
    fprintf('\n');
end

%% 3. Bang so sanh
fprintf('============ BANG SO SANH SM2 ============\n');
fprintf('%-8s | %-12s | %10s | %10s | %12s\n', ...
    'Traj', 'Method', 'Jc', 'Je', 'z_rms(10s)');
fprintf('%s\n', repmat('-', 1, 62));

for traj = {'circle', 'line'}
    for m_idx = 1:length(methods)
        tag = sprintf('%s_%s', traj{1}, methods{m_idx});
        d = results.(tag);
        N_ss = round(10 / s.dt);
        z_ss = d.z(:, end-N_ss:end);
        z_rms = sqrt(mean(sum(z_ss.^2, 1)));
        fprintf('%-8s | %-12s | %10.2f | %10.4f | %12.6f\n', ...
            traj{1}, method_names{m_idx}, d.Jc, d.Je, z_rms);
    end
end
fprintf('\n');

%% 4. Ve hinh
plot_sm2(results, s);

%% 5. Luu ket qua
save('../results/sm2_results.mat', 'results', 's');
fprintf('Da luu vao results/sm2_results.mat\n');
fprintf('\n========== SM2 HOAN THANH ==========\n');

%% ====================================================================
%  HAM PHU TRO
%  ====================================================================

function data = run_sm2_sim(method, s)
% RUN_SM2_SIM  Mo phong kinematic-only voi 1 method

    N  = round(s.T_sim / s.dt);
    dt = s.dt;

    data.t    = zeros(1, N+1);
    data.q    = zeros(3, N+1);
    data.qr   = zeros(3, N+1);
    data.z    = zeros(3, N+1);
    data.u    = zeros(2, N+1);
    data.cost_running = zeros(1, N+1);
    data.method = method;

    switch method
        case {'adp_pe', 'adp_nope'}
            data.Wc    = zeros(s.l, N+1);
            data.Wa    = zeros(s.l, N+1);
            data.delta = zeros(1, N+1);
        case 'cl'
            data.W     = zeros(s.l, N+1);
            data.delta = zeros(1, N+1);
            data.stack_count = zeros(1, N+1);
    end

    % Dieu kien ban dau
    [qr0, ~, ~] = ref_trajectory(0, s);
    q = qr0 + s.q0_offset;
    q(3) = atan2(sin(q(3)), cos(q(3)));

    % Khoi tao state
    switch method
        case 'adp_pe'
            adp.Wc = s.Wc0;
            adp.Wa = s.Wa0;
            s_run = s;
        case 'adp_nope'
            adp.Wc = s.Wc0;
            adp.Wa = s.Wa0;
            s_run = s;
            s_run.pe_off_time = 0;
        case 'cl'
            cl.W = s.cl_W0;
            cl.sigma_stack = zeros(s.l, s.cl_stack_max);
            cl.cost_stack = zeros(1, s.cl_stack_max);
            cl.stack_count = 0;
            cl.last_record_time = -inf;
    end

    for k = 1:(N+1)
        t = (k-1) * dt;
        [qr, vr, omegar] = ref_trajectory(t, s);

        switch method
            case 'bs'
                [u, z] = backstepping_controller(q, qr, vr, omegar, s);
                uf = [vr * cos(z(3)); omegar];
                uo = u - uf;

            case {'adp_pe', 'adp_nope'}
                [u, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s_run);
                z = info.z;
                uo = info.uo;
                data.Wc(:,k) = adp.Wc;
                data.Wa(:,k) = adp.Wa;
                data.delta(k) = info.delta;

            case 'cl'
                [u, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s);
                z = info.z;
                uo = info.uo;
                data.W(:,k) = cl.W;
                data.delta(k) = info.delta;
                data.stack_count(k) = info.stack_count;
        end

        data.t(k)    = t;
        data.q(:,k)  = q;
        data.qr(:,k) = qr;
        data.z(:,k)  = z;
        data.u(:,k)  = u;
        data.cost_running(k) = z' * s.Q * z + uo' * s.R * uo;

        if k <= N
            dq = wmr_kinematics(t, q, u(1), u(2));
            q = q + dt * dq;
            q(3) = atan2(sin(q(3)), cos(q(3)));
        end
    end

    data.Jc = trapz(data.t, data.cost_running);
    data.Je = trapz(data.t, sum(data.z.^2, 1));
end
