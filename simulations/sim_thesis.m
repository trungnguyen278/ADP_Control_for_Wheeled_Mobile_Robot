%% SIM_THESIS  Mo phong tong hop cho luan van
%
% 3 phan:
%   A. So sanh 5 phuong phap (+ bien the NTSMC) tren 3 quy dao (circle, line,
%      figure8), full model 5 state, co nhieu. Bang A2/A3: tach Jc = 10*Je + Ju,
%      thoi gian qua do, mo-men (Muc 5.2, 5.5 luan van)
%   B. Robustness: thay doi khoi luong m = {10, 12, 14} kg
%      So sanh ADP-FT vs BS vs CL (3 pp x 3 mass = 9 runs)
%   C. Robustness: thay doi bien do nhieu amp = {0, 0.2, 0.4, 0.6}
%      So sanh ADP-FT vs BS vs CL (3 pp x 4 amp = 12 runs)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

clear; close all; clc;
addpath('../models', '../controllers', '../plotting');

%% Tham so chung
p = wmr_params();
s = ctrl_params();
s.T_sim = 60;

fprintf('\n========== MO PHONG TONG HOP LUAN VAN ==========\n');

%% ================================================================
%  PHAN A: 3 quy dao x 5 phuong phap (co nhieu)
%  ================================================================
fprintf('\n--- PHAN A: 3 QUY DAO x 5 PHUONG PHAP ---\n');

methods = {'bs', 'adp', 'smc', 'ntsmc', 'cl', 'adp_ft'};
method_names = {'BS', 'ADP-AC', 'SMC', 'NTSMC', 'CL', 'ADP-FT'};
trajs = {'circle', 'line', 'figure8'};
results_A = struct();

for t_idx = 1:length(trajs)
    s.traj_type = trajs{t_idx};
    fprintf('  Quy dao: %s\n', upper(trajs{t_idx}));

    for m_idx = 1:length(methods)
        tag = sprintf('%s_%s', trajs{t_idx}, methods{m_idx});
        fprintf('    %-8s...', method_names{m_idx});

        data = run_full_sim(methods{m_idx}, true, s, p);
        results_A.(tag) = data;

        fprintf(' Jc=%.3f, Je=%.3f\n', data.Jc, data.Je);
    end
end

% Bang ket qua A
fprintf('\n  BANG A: So sanh 5 phuong phap (co nhieu)\n');
fprintf('  %-8s | %-8s | %8s | %8s | %10s\n', ...
    'Traj', 'Method', 'Jc', 'Je', 'z_rms(5s)');
fprintf('  %s\n', repmat('-', 1, 55));
for t_idx = 1:length(trajs)
    for m_idx = 1:length(methods)
        tag = sprintf('%s_%s', trajs{t_idx}, methods{m_idx});
        d = results_A.(tag);
        N_ss = round(5 / s.dt);
        z_ss = d.z(:, end-N_ss:end);
        z_rms = sqrt(mean(sum(z_ss.^2, 1)));
        fprintf('  %-8s | %-8s | %8.3f | %8.3f | %10.6f\n', ...
            trajs{t_idx}, method_names{m_idx}, d.Jc, d.Je, z_rms);
    end
end

% Bang A2: tach Jc = lambda_Q*Je + Ju (Q = 10*I nen z'Qz = 10*||z||^2) va thoi
% gian qua do: thoi diem cuoi cung ||z|| con vuot nguong (NaN = chua ve).
fprintf('\n  BANG A2: Tach chi phi va thoi gian qua do\n');
fprintf('  %-8s | %-8s | %8s | %7s | %8s %8s %8s\n', ...
    'Traj', 'Method', 'Ju', 'Ju/Jc', 't(0.05)', 't(0.02)', 't(0.01)');
fprintf('  %s\n', repmat('-', 1, 70));
for t_idx = 1:length(trajs)
    for m_idx = 1:length(methods)
        d = results_A.(sprintf('%s_%s', trajs{t_idx}, methods{m_idx}));
        Ju = d.Jc - s.Q(1,1) * d.Je;
        nz = sqrt(sum(d.z.^2, 1));
        fprintf('  %-8s | %-8s | %8.4f | %6.2f%% | %8.2f %8.2f %8.2f\n', ...
            trajs{t_idx}, method_names{m_idx}, Ju, 100*Ju/d.Jc, ...
            settle_last(d.t, nz, 0.05), settle_last(d.t, nz, 0.02), settle_last(d.t, nz, 0.01));
    end
end

% Bang A3: mo-men dau ra vong trong tren duong tron (Muc 5.5 luan van).
% tau da bi kep o +-tau_max trong dynamic_backstepping => bao hoa = cham bien.
fprintf('\n  BANG A3: Mo-men tren duong tron (tau_max = %.1f N.m)\n', p.tau_max);
fprintf('  %-8s | %9s | %9s | %9s | %9s | %9s | %7s\n', 'Method', ...
    'max 0-1s', 'max t>=1', 'tb t>=1', 'bh 0-1s', 'bh tong', 't95v');
fprintf('  %s\n', repmat('-', 1, 78));
for m_idx = 1:length(methods)
    d = results_A.(sprintf('circle_%s', methods{m_idx}));
    i1 = d.t >= 1;
    sat = any(abs(d.tau) >= p.tau_max - 1e-9, 1);
    k95 = find(d.eta(1,:) >= 0.95 * s.circle_vr, 1);
    fprintf('  %-8s | %9.3f | %9.3f | %9.4f | %8.1f%% | %8.2f%% | %7.3f\n', ...
        method_names{m_idx}, max(max(abs(d.tau(:, ~i1)))), max(max(abs(d.tau(:, i1)))), ...
        mean(mean(abs(d.tau(:, i1)))), 100*mean(sat(~i1)), 100*mean(sat), d.t(k95));
end

%% ================================================================
%  PHAN B: Robustness -- thay doi khoi luong
%  ================================================================
fprintf('\n--- PHAN B: ROBUSTNESS KHOI LUONG ---\n');

rob_methods = {'bs', 'cl', 'adp_ft'};
rob_names   = {'BS', 'CL', 'ADP-FT'};
masses = [10, 12, 14];
results_B = struct();
s.traj_type = 'circle';

for mi = 1:length(masses)
    p_plant = p;
    p_plant.m = masses(mi);
    p_plant.M = diag([p_plant.m, p.I]);
    p_plant.M_inv = diag([1/p_plant.m, 1/p.I]);
    fprintf('  m_plant = %d kg (controller dung m_nom = %d kg)\n', masses(mi), p.m);

    for m_idx = 1:length(rob_methods)
        tag = sprintf('m%d_%s', masses(mi), rob_methods{m_idx});
        fprintf('    %-8s...', rob_names{m_idx});

        data = run_full_sim_uncertain(rob_methods{m_idx}, true, s, p, p_plant);
        results_B.(tag) = data;

        fprintf(' Jc=%.3f\n', data.Jc);
    end
end

% Bang ket qua B
fprintf('\n  BANG B: Robustness khoi luong (circle, co nhieu)\n');
fprintf('  %-6s | %-8s | %8s | %8s | %10s\n', ...
    'Mass', 'Method', 'Jc', 'Je', 'z_rms(5s)');
fprintf('  %s\n', repmat('-', 1, 52));
for mi = 1:length(masses)
    for m_idx = 1:length(rob_methods)
        tag = sprintf('m%d_%s', masses(mi), rob_methods{m_idx});
        d = results_B.(tag);
        N_ss = round(5 / s.dt);
        z_ss = d.z(:, end-N_ss:end);
        z_rms = sqrt(mean(sum(z_ss.^2, 1)));
        fprintf('  %4dkg | %-8s | %8.3f | %8.3f | %10.6f\n', ...
            masses(mi), rob_names{m_idx}, d.Jc, d.Je, z_rms);
    end
end

%% ================================================================
%  PHAN C: Robustness -- thay doi bien do nhieu
%  ================================================================
fprintf('\n--- PHAN C: ROBUSTNESS NHIEU ---\n');

dist_amps = [0, 0.2, 0.4, 0.6];
results_C = struct();
s.traj_type = 'circle';

for di = 1:length(dist_amps)
    s_var = s;
    s_var.dist_amp = dist_amps(di);
    fprintf('  dist_amp = %.1f\n', dist_amps(di));

    for m_idx = 1:length(rob_methods)
        tag = sprintf('d%d_%s', round(dist_amps(di)*100), rob_methods{m_idx});
        fprintf('    %-8s...', rob_names{m_idx});

        use_dist = dist_amps(di) > 0;
        data = run_full_sim(rob_methods{m_idx}, use_dist, s_var, p);
        results_C.(tag) = data;

        fprintf(' Jc=%.3f\n', data.Jc);
    end
end

% Bang ket qua C
fprintf('\n  BANG C: Robustness nhieu (circle)\n');
fprintf('  %-8s | %-8s | %8s | %8s | %10s\n', ...
    'Amp', 'Method', 'Jc', 'Je', 'z_rms(5s)');
fprintf('  %s\n', repmat('-', 1, 52));
for di = 1:length(dist_amps)
    for m_idx = 1:length(rob_methods)
        tag = sprintf('d%d_%s', round(dist_amps(di)*100), rob_methods{m_idx});
        d = results_C.(tag);
        N_ss = round(5 / s.dt);
        z_ss = d.z(:, end-N_ss:end);
        z_rms = sqrt(mean(sum(z_ss.^2, 1)));
        fprintf('  %5.0f%% | %-8s | %8.3f | %8.3f | %10.6f\n', ...
            dist_amps(di)*100, rob_names{m_idx}, d.Jc, d.Je, z_rms);
    end
end

%% Ve hinh
plot_thesis(results_A, results_B, results_C, s, p, methods, method_names, ...
            rob_methods, rob_names, trajs, masses, dist_amps);

%% Luu
save('../results/thesis_results.mat', ...
    'results_A', 'results_B', 'results_C', 's', 'p');
fprintf('\nDa luu vao results/thesis_results.mat\n');
fprintf('\n========== HOAN THANH ==========\n');

%% ====================================================================
%  HAM PHU TRO (giong sim_sm1_full.m)
%  ====================================================================

function ts = settle_last(t, nz, tol)
% Thoi diem cuoi cung ||z|| con >= tol (sau do giu duoi tol). 0 neu chua bao
% gio vuot, NaN neu den cuoi mo phong van chua ve duoi tol.
    k = find(nz >= tol, 1, 'last');
    if isempty(k)
        ts = 0;
    elseif k == numel(nz)
        ts = NaN;
    else
        ts = t(k+1);
    end
end

function out = tern(cond, a, b)
    if cond, out = a; else, out = b; end
end

function data = run_full_sim(method, use_dist, s, p)

    N  = round(s.T_sim / s.dt);
    dt = s.dt;

    data.t     = zeros(1, N+1);
    data.q     = zeros(3, N+1);
    data.eta   = zeros(2, N+1);
    data.qr    = zeros(3, N+1);
    data.z     = zeros(3, N+1);
    data.eta_d = zeros(2, N+1);
    data.tau   = zeros(2, N+1);
    data.dist  = zeros(2, N+1);
    data.cost_running = zeros(1, N+1);
    data.method = method;
    data.use_dist = use_dist;

    switch method
        case 'adp'
            data.Wc = zeros(s.l, N+1);
            data.Wa = zeros(s.l, N+1);
        case {'cl', 'adp_ft'}
            data.W = zeros(s.l, N+1);
    end

    [qr0, ~, ~] = ref_trajectory(0, s);
    q   = qr0 + s.q0_offset;
    q(3) = atan2(sin(q(3)), cos(q(3)));
    eta = [0; 0];
    eta_d_prev = [0; 0];

    switch method
        case 'adp'
            adp.Wc = s.Wc0;
            adp.Wa = s.Wa0;
        case 'cl'
            cl.W = s.cl_W0;
            cl.sigma_stack = zeros(s.l, s.cl_stack_max);
            cl.cost_stack = zeros(1, s.cl_stack_max);
            cl.stack_count = 0;
            cl.last_record_time = -inf;
        case 'adp_ft'
            W_ft = s.ft_W0;
    end

    for k = 1:(N+1)
        t = (k-1) * dt;
        [qr, vr, omegar] = ref_trajectory(t, s);

        switch method
            case 'bs'
                [u_kin, z] = backstepping_controller(q, qr, vr, omegar, s);
                eta_d = u_kin;

            case 'adp'
                [u_kin, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s);
                eta_d = u_kin;
                z = info.z;
                data.Wc(:,k) = adp.Wc;
                data.Wa(:,k) = adp.Wa;

            case 'smc'
                [eta_d, z] = smc_kinematic(q, qr, vr, omegar, s);

            case 'ntsmc'
                [eta_d, z] = ntsmc_kinematic(q, qr, vr, omegar, s);

            case 'cl'
                [u_kin, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s);
                eta_d = u_kin;
                z = info.z;
                data.W(:,k) = cl.W;

            case 'adp_ft'
                [u_kin, W_ft, info] = adp_fixed_time(q, qr, vr, omegar, W_ft, s);
                eta_d = u_kin;
                z = info.z;
                data.W(:,k) = W_ft;
        end

        uo = eta_d - [vr*cos(z(3)); omegar];

        if k == 1
            eta_d_prev_k = eta_d;
        else
            eta_d_prev_k = eta_d_prev;
        end

        [tau, ~] = dynamic_backstepping(eta, eta_d, eta_d_prev_k, dt, p, s, z);
        eta_d_prev = eta_d;

        if use_dist
            d_t = s.dist_amp * p.tau_max * sin(s.dist_freq * t) * [1; 1];
        else
            d_t = [0; 0];
        end

        tau_plant = tau + d_t;

        data.t(k)     = t;
        data.q(:,k)   = q;
        data.eta(:,k) = eta;
        data.qr(:,k)  = qr;
        data.z(:,k)   = z;
        data.eta_d(:,k) = eta_d;
        data.tau(:,k)   = tau;
        data.dist(:,k)  = d_t;
        data.cost_running(k) = z'*s.Q*z + uo'*s.R*uo;

        if k <= N
            state = [q; eta];
            dstate = wmr_full_model(t, state, tau_plant, p);
            state = state + dt * dstate;
            q   = state(1:3);
            q(3) = atan2(sin(q(3)), cos(q(3)));
            eta = state(4:5);
        end
    end

    data.Jc = trapz(data.t, data.cost_running);
    data.Je = trapz(data.t, sum(data.z.^2, 1));
end

function data = run_full_sim_uncertain(method, use_dist, s, p_ctrl, p_plant)
% RUN_FULL_SIM_UNCERTAIN  Controller dung p_ctrl (nominal), plant dung p_plant (thuc)
% Test robustness khi co model uncertainty (vd: khoi luong khac nominal)

    N  = round(s.T_sim / s.dt);
    dt = s.dt;

    data.t     = zeros(1, N+1);
    data.q     = zeros(3, N+1);
    data.eta   = zeros(2, N+1);
    data.qr    = zeros(3, N+1);
    data.z     = zeros(3, N+1);
    data.eta_d = zeros(2, N+1);
    data.tau   = zeros(2, N+1);
    data.dist  = zeros(2, N+1);
    data.cost_running = zeros(1, N+1);
    data.method = method;
    data.use_dist = use_dist;

    switch method
        case 'adp'
            data.Wc = zeros(s.l, N+1);
            data.Wa = zeros(s.l, N+1);
        case {'cl', 'adp_ft'}
            data.W = zeros(s.l, N+1);
    end

    [qr0, ~, ~] = ref_trajectory(0, s);
    q   = qr0 + s.q0_offset;
    q(3) = atan2(sin(q(3)), cos(q(3)));
    eta = [0; 0];
    eta_d_prev = [0; 0];

    switch method
        case 'adp'
            adp.Wc = s.Wc0;
            adp.Wa = s.Wa0;
        case 'cl'
            cl.W = s.cl_W0;
            cl.sigma_stack = zeros(s.l, s.cl_stack_max);
            cl.cost_stack = zeros(1, s.cl_stack_max);
            cl.stack_count = 0;
            cl.last_record_time = -inf;
        case 'adp_ft'
            W_ft = s.ft_W0;
    end

    for k = 1:(N+1)
        t = (k-1) * dt;
        [qr, vr, omegar] = ref_trajectory(t, s);

        switch method
            case 'bs'
                [u_kin, z] = backstepping_controller(q, qr, vr, omegar, s);
                eta_d = u_kin;
            case 'adp'
                [u_kin, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s);
                eta_d = u_kin; z = info.z;
                data.Wc(:,k) = adp.Wc; data.Wa(:,k) = adp.Wa;
            case 'smc'
                [eta_d, z] = smc_kinematic(q, qr, vr, omegar, s);

            case 'ntsmc'
                [eta_d, z] = ntsmc_kinematic(q, qr, vr, omegar, s);
            case 'cl'
                [u_kin, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s);
                eta_d = u_kin; z = info.z; data.W(:,k) = cl.W;
            case 'adp_ft'
                [u_kin, W_ft, info] = adp_fixed_time(q, qr, vr, omegar, W_ft, s);
                eta_d = u_kin; z = info.z; data.W(:,k) = W_ft;
        end

        uo = eta_d - [vr*cos(z(3)); omegar];

        if k == 1, eta_d_prev_k = eta_d;
        else,      eta_d_prev_k = eta_d_prev; end

        % Inner loop dung p_ctrl (nominal) — KHONG biet mass thuc
        [tau, ~] = dynamic_backstepping(eta, eta_d, eta_d_prev_k, dt, p_ctrl, s, z);
        eta_d_prev = eta_d;

        if use_dist
            d_t = s.dist_amp * p_plant.tau_max * sin(s.dist_freq * t) * [1; 1];
        else
            d_t = [0; 0];
        end
        tau_plant = tau + d_t;

        data.t(k) = t; data.q(:,k) = q; data.eta(:,k) = eta;
        data.qr(:,k) = qr; data.z(:,k) = z;
        data.eta_d(:,k) = eta_d; data.tau(:,k) = tau;
        data.dist(:,k) = d_t;
        data.cost_running(k) = z'*s.Q*z + uo'*s.R*uo;

        if k <= N
            state = [q; eta];
            % Plant dung p_plant (mass thuc)
            dstate = wmr_full_model(t, state, tau_plant, p_plant);
            state = state + dt * dstate;
            q = state(1:3); q(3) = atan2(sin(q(3)), cos(q(3)));
            eta = state(4:5);
        end
    end

    data.Jc = trapz(data.t, data.cost_running);
    data.Je = trapz(data.t, sum(data.z.^2, 1));
end
