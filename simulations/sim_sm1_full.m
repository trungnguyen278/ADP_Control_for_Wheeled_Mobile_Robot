%% SIM_SM1_FULL  So sanh 5 controller tren mo hinh 5 state (co/khong nhieu)
%
% Kien truc 2 vong:
%   Vong ngoai (kinematic): z -> (v_d, w_d)  [BS / ADP-AC / SMC / CL / ADP-FT]
%   Vong trong (dynamic):   e_eta -> tau      [Dynamic BS chung, Fierro & Lewis]
%   Plant: tau + d(t) -> wmr_full_model (5 state)
%
% 10 kich ban: {BS, ADP-AC, SMC, CL, ADP-FT} x {no dist, dist}
% Quy dao: circle
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

clear; close all; clc;
addpath('../models', '../controllers', '../plotting');

%% 1. Load tham so
p = wmr_params();
s = ctrl_params();
s.traj_type = 'circle';
s.T_sim = 60;

fprintf('\n========== SO SANH FULL MODEL: 5 PHUONG PHAP ==========\n');
fprintf('Robot: m=%.1f, I=%.2f, r=%.3f, L=%.3f\n', p.m, p.I, p.r, p.L);
fprintf('Nhieu: d = %.1f*tau_max*sin(%.0f*t)\n', s.dist_amp, s.dist_freq);
fprintf('Vong trong: Kd_v=%.0f, Kd_w=%.0f\n\n', s.Kd_v, s.Kd_w);

%% 2. Chay 10 kich ban
methods = {'bs', 'adp', 'smc', 'ntsmc', 'cl', 'adp_ft'};
method_names = {'Backstepping', 'ADP Actor-Critic', 'SMC', 'NTSMC', 'Critic-only CL', 'ADP Fixed-time'};
results = struct();

for m_idx = 1:length(methods)
    method = methods{m_idx};
    for dist = [false, true]
        tag = sprintf('%s_%s', method, tern(dist, 'dist', 'nodist'));
        fprintf('  [%-6s] %s (%s)...', upper(method), method_names{m_idx}, ...
            tern(dist, 'co nhieu', 'khong nhieu'));

        data = run_full_sim(method, dist, s, p);
        results.(tag) = data;

        fprintf(' Jc=%.2f, Je=%.4f\n', data.Jc, data.Je);
    end
end

%% 3. Bang so sanh
fprintf('\n============ BANG SO SANH (Circle, T=%ds) ============\n', s.T_sim);
fprintf('%-8s | %-10s | %10s | %10s | %12s\n', ...
    'Method', 'Disturb', 'Jc', 'Je', 'z_rms(5s)');
fprintf('%s\n', repmat('-', 1, 62));

for m_idx = 1:length(methods)
    method = methods{m_idx};
    for dist = [false, true]
        tag = sprintf('%s_%s', method, tern(dist, 'dist', 'nodist'));
        d = results.(tag);
        N_ss = round(5 / s.dt);
        z_ss = d.z(:, end-N_ss:end);
        z_rms = sqrt(mean(sum(z_ss.^2, 1)));
        fprintf('%-8s | %-10s | %10.2f | %10.4f | %12.6f\n', ...
            upper(method), tern(dist, 'Yes', 'No'), d.Jc, d.Je, z_rms);
    end
end
fprintf('\n');

%% 4. Ve hinh
plot_sm1_full(results, s, p);

%% 5. Luu
save('../results/sm1_full_results.mat', 'results', 's', 'p');
fprintf('Da luu vao results/sm1_full_results.mat\n');
fprintf('\n========== HOAN THANH ==========\n');

%% ====================================================================
%  HAM PHU TRO
%  ====================================================================

function out = tern(cond, a, b)
    if cond, out = a; else, out = b; end
end

function data = run_full_sim(method, use_dist, s, p)
% RUN_FULL_SIM  Mo phong 5 state voi 1 controller + co/khong nhieu

    N  = round(s.T_sim / s.dt);
    dt = s.dt;

    % Pre-allocate
    data.t     = zeros(1, N+1);
    data.q     = zeros(3, N+1);    % [x;y;theta]
    data.eta   = zeros(2, N+1);    % [v;omega]
    data.qr    = zeros(3, N+1);
    data.z     = zeros(3, N+1);
    data.eta_d = zeros(2, N+1);    % van toc mong muon
    data.tau   = zeros(2, N+1);    % mo-men
    data.dist  = zeros(2, N+1);    % nhieu
    data.cost_running = zeros(1, N+1);
    data.method = method;
    data.use_dist = use_dist;

    % Weight storage tuy theo method
    switch method
        case 'adp'
            data.Wc = zeros(s.l, N+1);
            data.Wa = zeros(s.l, N+1);
        case {'cl', 'adp_ft'}
            data.W = zeros(s.l, N+1);
    end

    % Dieu kien ban dau
    [qr0, ~, ~] = ref_trajectory(0, s);
    q   = qr0 + s.q0_offset;
    q(3) = atan2(sin(q(3)), cos(q(3)));
    eta = [0; 0];       % robot dung yen
    eta_d_prev = [0; 0];

    % Khoi tao state rieng cua tung method
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

    % Vong lap chinh
    for k = 1:(N+1)
        t = (k-1) * dt;

        % Reference
        [qr, vr, omegar] = ref_trajectory(t, s);

        % === VONG NGOAI: chon controller ===
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

        % uo = effective feedback (nhat quan cho moi method)
        uo = eta_d - [vr*cos(z(3)); omegar];

        % === VONG TRONG: dynamic backstepping ===
        if k == 1
            eta_d_prev_k = eta_d;
        else
            eta_d_prev_k = eta_d_prev;
        end

        [tau, ~] = dynamic_backstepping(eta, eta_d, eta_d_prev_k, dt, p, s);
        eta_d_prev = eta_d;

        % === NHIEU ===
        if use_dist
            d_t = s.dist_amp * p.tau_max * sin(s.dist_freq * t) * [1; 1];
        else
            d_t = [0; 0];
        end

        tau_plant = tau + d_t;

        % Luu du lieu
        data.t(k)     = t;
        data.q(:,k)   = q;
        data.eta(:,k) = eta;
        data.qr(:,k)  = qr;
        data.z(:,k)   = z;
        data.eta_d(:,k) = eta_d;
        data.tau(:,k)   = tau;
        data.dist(:,k)  = d_t;
        data.cost_running(k) = z'*s.Q*z + uo'*s.R*uo;

        % === EULER STEP (5 state) ===
        if k <= N
            state = [q; eta];
            dstate = wmr_full_model(t, state, tau_plant, p);
            state = state + dt * dstate;
            q   = state(1:3);
            q(3) = atan2(sin(q(3)), cos(q(3)));
            eta = state(4:5);
        end
    end

    % Cost tich luy
    data.Jc = trapz(data.t, data.cost_running);
    data.Je = trapz(data.t, sum(data.z.^2, 1));
end
