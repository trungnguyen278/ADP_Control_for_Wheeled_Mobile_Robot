function data = run_extra_sim(method, s, p, dist_mode, p_plant)
% RUN_EXTRA_SIM  Mo phong full model 5 trang thai (vong ngoai + dynamic BS vong trong)
%
% data = run_extra_sim(method, s, p, dist_mode, p_plant)
%
% Dung chung cho sim_thesis_extra.m va sweep_gains.m (truoc 2026-10-26 la
% ham cuc bo trong sim_thesis_extra.m). Giong run_full_sim trong sim_thesis.m
% nhung cho phep chon kieu nhieu.
%
% Input:
%   method    -- 'bs' | 'smc' | 'ntsmc' | 'cl' | 'adp_ft' | 'adp'
%   s         -- ctrl_params struct (traj_type, T_sim, dist_amp, ...)
%   p         -- tham so controller dung (nominal), tu wmr_params()
%   dist_mode -- 'none' | 'common' | 'diff' | 'indep'
%   p_plant   -- tham so plant thuc (tuy chon; mac dinh = p, khong bat dinh)
%
% Output:
%   data -- struct: t, z, tau, W, Jc, Je, zrms (5 s cuoi), satv, method, dist_mode
%           satv = ti le thoi gian lenh van toc cham gioi han (|v_d| = v_max
%           hoac |w_d| = w_max) -- dung o Phan J de kiem tra lenh bi cat
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    08/2026 (tach file 10/2026)

if nargin < 5, p_plant = p; end

N = round(s.T_sim / s.dt);  dt = s.dt;
data.t   = zeros(1, N+1);
data.z   = zeros(3, N+1);
data.tau = zeros(2, N+1);
data.W   = zeros(s.l, N+1);
cost     = zeros(1, N+1);
satv     = false(1, N+1);

[qr0, ~, ~] = ref_trajectory(0, s);
q = qr0 + s.q0_offset;
q(3) = atan2(sin(q(3)), cos(q(3)));
eta = [0; 0];  eta_d_prev = [0; 0];

switch method
    case 'cl'
        cl.W = s.cl_W0;
        cl.sigma_stack = zeros(s.l, s.cl_stack_max);
        cl.cost_stack  = zeros(1, s.cl_stack_max);
        cl.stack_count = 0;
        cl.last_record_time = -inf;
    case 'adp_ft'
        W_ft = s.ft_W0;
    case 'adp'
        adp.Wc = s.Wc0;  adp.Wa = s.Wa0;
end

for k = 1:(N+1)
    t = (k-1) * dt;
    [qr, vr, omegar] = ref_trajectory(t, s);

    switch method
        case 'bs'
            [eta_d, z] = backstepping_controller(q, qr, vr, omegar, s);
        case 'smc'
            [eta_d, z] = smc_kinematic(q, qr, vr, omegar, s);
        case 'ntsmc'
            [eta_d, z] = ntsmc_kinematic(q, qr, vr, omegar, s);
        case 'cl'
            [eta_d, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s);
            z = info.z;  data.W(:,k) = cl.W;
        case 'adp_ft'
            [eta_d, W_ft, info] = adp_fixed_time(q, qr, vr, omegar, W_ft, s);
            z = info.z;  data.W(:,k) = W_ft;
        case 'adp'
            [eta_d, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s);
            z = info.z;
    end

    uo = eta_d - [vr*cos(z(3)); omegar];

    if k == 1, edp = eta_d; else, edp = eta_d_prev; end
    [tau, ~] = dynamic_backstepping(eta, eta_d, edp, dt, p, s);
    eta_d_prev = eta_d;

    a = s.dist_amp * p_plant.tau_max;
    w = s.dist_freq;
    switch dist_mode
        case 'none',   d_t = [0; 0];
        case 'common', d_t = a * sin(w*t) * [1;  1];
        case 'diff',   d_t = a * sin(w*t) * [1; -1];
        case 'indep',  d_t = a * [sin(w*t); sin(1.3*w*t + pi/3)];
    end

    data.t(k) = t;  data.z(:,k) = z;  data.tau(:,k) = tau;
    cost(k) = z'*s.Q*z + uo'*s.R*uo;
    satv(k) = abs(eta_d(1)) >= s.v_max - 1e-9 || abs(eta_d(2)) >= s.w_max - 1e-9;

    if k <= N
        st = [q; eta];
        st = st + dt * wmr_full_model(t, st, tau + d_t, p_plant);
        q = st(1:3);  q(3) = atan2(sin(q(3)), cos(q(3)));
        eta = st(4:5);
    end
end

data.Jc = trapz(data.t, cost);
data.Je = trapz(data.t, sum(data.z.^2, 1));
Nss = round(5 / s.dt);
zss = data.z(:, end-Nss:end);
data.zrms = sqrt(mean(sum(zss.^2, 1)));
data.satv = mean(satv);
data.method = method;
data.dist_mode = dist_mode;
end
