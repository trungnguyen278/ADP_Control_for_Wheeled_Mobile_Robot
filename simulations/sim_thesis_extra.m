%% SIM_THESIS_EXTRA  Ba thi nghiem bo sung cho luan van
%
% KHONG thay the sim_thesis.m. Script nay tao du lieu MOI, giu nguyen
% toan bo so lieu Phan A/B/C cu de con doi chieu.
%
% Phan D. Ablation ADP-UUB (Wang et al. eq.32-33)
%    ADP-UUB = ADP-FT nhung BO so hang fixed-time:
%      - kappa2 = 0            (bo sigma-mod bac 3 trong luat cap nhat)
%      - beta   = 0            (bo so hang z^3)
%      - p/q    = 1            (z^{p/q} thoai bien thanh z tuyen tinh)
%    Day la baseline cua CHINH bai bao goc, dung de tra loi cau hoi
%    "cac so hang fixed-time dong gop duoc gi?"
%
% Phan E. Quet dieu kien dau z0
%    Do t_settle (thoi diem ||z|| < tol va o lai) voi nhieu z0 khac nhau.
%    Fixed-time => t_settle phai gan nhu KHONG doi theo z0.
%
% Phan F. Nhieu hai kenh doc lap
%    Kich ban cu d = a*[1;1] bi ma tran B triet tieu hoan toan o kenh omega
%    (B*[1;1] = [r;0]). Phan nay chay 3 kich ban de tach bach:
%      common = a*[sin;  sin]          (cu, chi kenh v)
%      diff   = a*[sin; -sin]          (chi kenh omega)
%      indep  = a*[sin(w t); sin(1.3 w t + pi/3)]  (ca hai kenh)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    08/2026

clear; close all; clc;
addpath('../models', '../controllers', '../plotting');

p = wmr_params();
s = ctrl_params();
s.T_sim = 60;

fprintf('\n========== THI NGHIEM BO SUNG CHO LUAN VAN ==========\n');

%% ================================================================
%  PHAN D: Ablation ADP-UUB vs ADP-FT
%  ================================================================
fprintf('\n--- PHAN D: ABLATION ADP-UUB (bo cac so hang fixed-time) ---\n');

trajs = {'circle', 'line', 'figure8'};
results_D = struct();

for t_idx = 1:length(trajs)
    s_ft = s;  s_ft.traj_type = trajs{t_idx};

    % ADP-UUB: tat kappa2, beta, va cho p/q = 1
    s_uub = s_ft;
    s_uub.ft_kappa2 = 0;
    s_uub.ft_beta   = [0; 0; 0];
    s_uub.ft_p      = s_ft.ft_q;   % => p/q = 1, |z|^1*sign(z) = z

    d_ft  = run_extra_sim('adp_ft', s_ft,  p, 'common');
    d_uub = run_extra_sim('adp_ft', s_uub, p, 'common');

    results_D.([trajs{t_idx} '_ft'])  = d_ft;
    results_D.([trajs{t_idx} '_uub']) = d_uub;

    fprintf('  %-8s ADP-FT : Jc=%8.1f  Je=%7.3f  z_rms=%.3e\n', ...
        trajs{t_idx}, d_ft.Jc,  d_ft.Je,  d_ft.zrms);
    fprintf('  %-8s ADP-UUB: Jc=%8.1f  Je=%7.3f  z_rms=%.3e\n', ...
        trajs{t_idx}, d_uub.Jc, d_uub.Je, d_uub.zrms);
end

%% ================================================================
%  PHAN E: Quet dieu kien dau (kiem chung tinh fixed-time)
%  ================================================================
fprintf('\n--- PHAN E: QUET DIEU KIEN DAU z0 ---\n');

z0_list = { [0.2; -0.1; 0.1], [0.5; -0.25; 0.25], [1.0; -0.5; 0.5], ...
            [2.0; -1.0; 0.8],  [3.0; -1.5; 1.0] };
e_methods = {'adp_ft', 'bs', 'cl'};
e_names   = {'ADP-FT', 'BS', 'CL'};

% Nhieu nguong tol: tach bach "hoi tu cham" voi "sai so du lon".
% ADP-FT co z_rms xac lap ~0.046 nen tol=0.05 la sat nguong -- neu chi dung
% mot tol thi khong phan biet duoc hai nguyen nhan.
tol_list = [0.05, 0.10, 0.15, 0.25];

results_E = struct();
T_settle = nan(numel(e_methods), numel(z0_list), numel(tol_list));
z0_norm  = nan(1, numel(z0_list));

s_e = s;  s_e.traj_type = 'circle';
for zi = 1:numel(z0_list)
    s_e.q0_offset = z0_list{zi};
    for mi = 1:numel(e_methods)
        d = run_extra_sim(e_methods{mi}, s_e, p, 'common');
        for ti = 1:numel(tol_list)
            T_settle(mi, zi, ti) = settle_time(d.t, d.z, tol_list(ti));
        end
        z0_norm(zi) = norm(d.z(:,1));
        results_E.(sprintf('%s_z%d', e_methods{mi}, zi)) = d;
    end
end

for ti = 1:numel(tol_list)
    fprintf('\n  BANG E%d: t_settle [s] voi tol = %.2f (NaN = khong dat trong %ds)\n', ...
        ti, tol_list(ti), s.T_sim);
    fprintf('  %-8s |', 'Method');
    for zi = 1:numel(z0_list), fprintf(' %9.3f |', z0_norm(zi)); end
    fprintf('   <- ||z0||\n');
    fprintf('  %s\n', repmat('-', 1, 12 + 12*numel(z0_list)));
    for mi = 1:numel(e_methods)
        fprintf('  %-8s |', e_names{mi});
        for zi = 1:numel(z0_list)
            if isnan(T_settle(mi,zi,ti)), fprintf('  KHONG DAT|');
            else,                         fprintf(' %9.2f |', T_settle(mi,zi,ti)); end
        end
        fprintf('\n');
    end
    % Chi so fixed-time: t_settle cang it phu thuoc z0 thi ti so cang gan 1
    fprintf('    Do tan xa (max/min tren cac z0 dat duoc):\n');
    for mi = 1:numel(e_methods)
        v = T_settle(mi, :, ti);  v = v(~isnan(v));
        if numel(v) >= 2
            fprintf('      %-8s: min=%.2fs max=%.2fs ti so=%.2f (khong dat: %d/%d)\n', ...
                e_names{mi}, min(v), max(v), max(v)/min(v), ...
                sum(isnan(T_settle(mi,:,ti))), numel(z0_list));
        else
            fprintf('      %-8s: khong du du lieu (%d/%d z0 khong dat)\n', ...
                e_names{mi}, sum(isnan(T_settle(mi,:,ti))), numel(z0_list));
        end
    end
end

%% ================================================================
%  PHAN F: Nhieu hai kenh doc lap
%  ================================================================
fprintf('\n--- PHAN F: NHIEU HAI KENH DOC LAP ---\n');

fprintf('  Kiem tra ma tran B:\n');
fprintf('    B*[1; 1] = [%.5f; %.5f]  <- dong pha: mo-men quay = 0\n', p.B*[1;1]);
fprintf('    B*[1;-1] = [%.5f; %.5f]  <- vi sai\n', p.B*[1;-1]);

f_methods = {'bs', 'cl', 'adp_ft'};
f_names   = {'BS', 'CL', 'ADP-FT'};
dist_amps = [0, 0.2, 0.4, 0.6];
dist_modes = {'common', 'diff', 'indep'};
results_F = struct();

s_f = s;  s_f.traj_type = 'circle';
for dm = 1:numel(dist_modes)
    fprintf('\n  Kieu nhieu: %s\n', dist_modes{dm});
    fprintf('  %-6s | %-8s | %8s | %8s | %10s\n', 'Amp', 'Method', 'Jc', 'Je', 'z_rms');
    fprintf('  %s\n', repmat('-', 1, 52));
    for di = 1:numel(dist_amps)
        s_f.dist_amp = dist_amps(di);
        for mi = 1:numel(f_methods)
            mode_k = dist_modes{dm};
            if dist_amps(di) == 0, mode_k = 'none'; end
            d = run_extra_sim(f_methods{mi}, s_f, p, mode_k);
            results_F.(sprintf('%s_d%d_%s', dist_modes{dm}, ...
                round(dist_amps(di)*100), f_methods{mi})) = d;
            fprintf('  %5.0f%% | %-8s | %8.1f | %8.3f | %10.3e\n', ...
                dist_amps(di)*100, f_names{mi}, d.Jc, d.Je, d.zrms);
        end
    end
end

%% ================================================================
%  PHAN G: Ablation LUAT CAP NHAT TRONG SO
%  ================================================================
% Wang eq.(17) nguyen ban (khong dung sai so Bellman) vs luat cua luan van
% (gradient descent tren sai so Bellman). Tach rieng anh huong cua thay doi
% nay -- truoc day no bi tron lan voi viec tune gain nen khong dinh luong duoc.
fprintf('\n--- PHAN G: ABLATION LUAT CAP NHAT TRONG SO ---\n');

results_G = struct();
fprintf('  %-8s | %-16s | %8s | %8s | %10s | %9s\n', ...
    'Traj', 'Luat cap nhat', 'Jc', 'Je', 'z_rms', '||W(T)||');
fprintf('  %s\n', repmat('-', 1, 72));
for t_idx = 1:length(trajs)
    for law = {'wang', 'bellman'}
        s_g = s;
        s_g.traj_type = trajs{t_idx};
        s_g.ft_update_law = law{1};
        d = run_extra_sim('adp_ft', s_g, p, 'common');
        results_G.([trajs{t_idx} '_' law{1}]) = d;
        fprintf('  %-8s | %-16s | %8.1f | %8.3f | %10.3e | %9.3f\n', ...
            trajs{t_idx}, law{1}, d.Jc, d.Je, d.zrms, norm(d.W(:,end)));
    end
end

%% ================================================================
%  PHAN H: SMC voi tham so thong dung vs SMC da tune
%  ================================================================
% Luan van dung lambda=3, eta=1.0 (gia tri thong dung cho mo hinh kinematic),
% trong khi ADP-FT duoc tune ky cho dual-loop. So sanh nhu vay khong cong bang.
% Phan nay chay them SMC voi gain da quet tren full model (lambda=0.7, eta=0.05)
% de co doi chieu dung muc.
fprintf('\n--- PHAN H: SMC THONG DUNG vs SMC DA TUNE ---\n');

results_H = struct();
smc_cfgs = { struct('name','SMC thong dung', 'tag','base',  'lam',3.0, 'eta',1.00), ...
             struct('name','SMC da tune',    'tag','tuned', 'lam',0.7, 'eta',0.05) };

fprintf('\n  H1. Ba quy dao (nhieu 20%%)\n');
fprintf('  %-16s | %10s | %10s | %10s\n', 'Cau hinh', 'circle', 'line', 'figure8');
fprintf('  %s\n', repmat('-', 1, 54));
for ci = 1:numel(smc_cfgs)
    fprintf('  %-16s |', smc_cfgs{ci}.name);
    for t_idx = 1:length(trajs)
        s_h = s;
        s_h.traj_type = trajs{t_idx};
        s_h.smc_lambda1 = smc_cfgs{ci}.lam;  s_h.smc_lambda2 = smc_cfgs{ci}.lam;
        s_h.smc_eta1    = smc_cfgs{ci}.eta;  s_h.smc_eta2    = smc_cfgs{ci}.eta;
        d = run_extra_sim('smc', s_h, p, 'common');
        results_H.(sprintf('%s_%s', smc_cfgs{ci}.tag, trajs{t_idx})) = d;
        fprintf(' %10.1f |', d.Jc);
    end
    fprintf('\n');
end

fprintf('\n  H2. Bon muc nhieu (circle)\n');
fprintf('  %-16s | %8s | %8s | %8s | %8s\n', 'Cau hinh', '0%', '20%', '40%', '60%');
fprintf('  %s\n', repmat('-', 1, 58));
for ci = 1:numel(smc_cfgs)
    fprintf('  %-16s |', smc_cfgs{ci}.name);
    for di = 1:numel(dist_amps)
        s_h = s;  s_h.traj_type = 'circle';  s_h.dist_amp = dist_amps(di);
        s_h.smc_lambda1 = smc_cfgs{ci}.lam;  s_h.smc_lambda2 = smc_cfgs{ci}.lam;
        s_h.smc_eta1    = smc_cfgs{ci}.eta;  s_h.smc_eta2    = smc_cfgs{ci}.eta;
        mode_k = 'common';  if dist_amps(di) == 0, mode_k = 'none'; end
        d = run_extra_sim('smc', s_h, p, mode_k);
        results_H.(sprintf('%s_d%d', smc_cfgs{ci}.tag, round(dist_amps(di)*100))) = d;
        fprintf(' %8.1f |', d.Jc);
    end
    fprintf('\n');
end

fprintf('\n  H3. z_rms 5s cuoi (circle, nhieu 20%%)\n');
for ci = 1:numel(smc_cfgs)
    d = results_H.(sprintf('%s_d20', smc_cfgs{ci}.tag));
    fprintf('    %-16s : %.3e\n', smc_cfgs{ci}.name, d.zrms);
end

%% Luu
save('../results/thesis_extra.mat', ...
     'results_D', 'results_E', 'results_F', 'results_G', 'results_H', ...
     'T_settle', 'z0_norm', 'tol_list', ...
     'z0_list', 'dist_amps', 'dist_modes', 's', 'p');
fprintf('\nDa luu vao results/thesis_extra.mat\n');
fprintf('\n========== HOAN THANH ==========\n');

%% ====================================================================
%  HAM PHU TRO
%  ====================================================================

function ts = settle_time(t, z, tol)
    n = sqrt(sum(z.^2, 1));
    ts = NaN;
    for k = 1:numel(n)
        if all(n(k:end) < tol), ts = t(k); return; end
    end
end

function data = run_extra_sim(method, s, p, dist_mode)
% Vong lap mo phong full model, giong run_full_sim trong sim_thesis.m
% nhung cho phep chon kieu nhieu qua dist_mode.

    N = round(s.T_sim / s.dt);  dt = s.dt;
    data.t   = zeros(1, N+1);
    data.z   = zeros(3, N+1);
    data.tau = zeros(2, N+1);
    data.W   = zeros(s.l, N+1);
    cost     = zeros(1, N+1);

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

        a = s.dist_amp * p.tau_max;
        w = s.dist_freq;
        switch dist_mode
            case 'none',   d_t = [0; 0];
            case 'common', d_t = a * sin(w*t) * [1;  1];
            case 'diff',   d_t = a * sin(w*t) * [1; -1];
            case 'indep',  d_t = a * [sin(w*t); sin(1.3*w*t + pi/3)];
        end

        data.t(k) = t;  data.z(:,k) = z;  data.tau(:,k) = tau;
        cost(k) = z'*s.Q*z + uo'*s.R*uo;

        if k <= N
            st = [q; eta];
            st = st + dt * wmr_full_model(t, st, tau + d_t, p);
            q = st(1:3);  q(3) = atan2(sin(q(3)), cos(q(3)));
            eta = st(4:5);
        end
    end

    data.Jc = trapz(data.t, cost);
    data.Je = trapz(data.t, sum(data.z.^2, 1));
    Nss = round(5 / s.dt);
    zss = data.z(:, end-Nss:end);
    data.zrms = sqrt(mean(sum(zss.^2, 1)));
    data.method = method;
    data.dist_mode = dist_mode;
end
