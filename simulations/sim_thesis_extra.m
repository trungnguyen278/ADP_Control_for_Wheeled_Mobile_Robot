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
%    Fixed-time (Polyakov) chi doi t_settle(z0) <= T_max voi MOI z0 -- KHONG
%    doi t_settle bang nhau. He fixed-time ly tuong van co t_settle tang theo
%    ||z0|| roi bao hoa, nen ti so max/min KHONG phai phep thu fixed-time.
%    Co bao hoa v_max thi t_settle buoc phai tang khi z0 du lon.
%
% Phan F. Nhieu hai kenh doc lap
%    Kich ban cu d = a*[1;1] bi ma tran B triet tieu hoan toan o kenh omega
%    (B*[1;1] = [2/r;0]). Phan nay chay 3 kich ban de tach bach:
%      common = a*[sin;  sin]          (cu, chi kenh v)
%      diff   = a*[sin; -sin]          (chi kenh omega)
%      indep  = a*[sin(w t); sin(1.3 w t + pi/3)]  (ca hai kenh)
%
% Phan G. Ablation luat cap nhat trong so (Wang eq.17 vs gradient Bellman)
% Phan H. SMC tham so thong dung vs SMC da tune
% Phan I. Do ben cua SMC da tune (khoi luong, z0, nhieu doc lap)
% Phan J. Ablation co che (toan tu chieu, ghep nonholonomic) + dong bang critic
% Phan K. ADP-FT co thich nghi khi doi Q, R khong
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

    fprintf('  %-8s ADP-FT : Jc=%8.3f  Je=%7.3f  z_rms=%.3e\n', ...
        trajs{t_idx}, d_ft.Jc,  d_ft.Je,  d_ft.zrms);
    fprintf('  %-8s ADP-UUB: Jc=%8.3f  Je=%7.3f  z_rms=%.3e\n', ...
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
% (Voi mo hinh B sai truoc 2026-10-26, ADP-FT co z_rms xac lap ~0.046 nen
% tol=0.05 sat nguong -- mot tol khong phan biet duoc hai nguyen nhan.)
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
    % Chi mo ta do tan xa -- KHONG phai phep thu fixed-time (xem dau file):
    % he fixed-time ly tuong tren cung dai ||z0|| cho ti so 2.35-3.59.
    fprintf('    Do tan xa (max/min tren cac z0 dat duoc, chi de mo ta):\n');
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
            fprintf('  %5.0f%% | %-8s | %8.3f | %8.3f | %10.3e\n', ...
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
        fprintf('  %-8s | %-16s | %8.3f | %8.3f | %10.3e | %9.3f\n', ...
            trajs{t_idx}, law{1}, d.Jc, d.Je, d.zrms, norm(d.W(:,end)));
    end
end

%% ================================================================
%  PHAN H: SMC voi tham so thong dung vs SMC da tune
%  ================================================================
% Doi chieu SMC voi gia tri thong dung (lambda=3, eta=1.0) va SMC da chinh deu
% (gia tri mac dinh trong ctrl_params.m, chon bang sweep_gains.m cung cong suc
% voi cac phuong phap khac).
fprintf('\n--- PHAN H: SMC THONG DUNG vs SMC DA TUNE ---\n');

results_H = struct();
smc_cfgs = { struct('name','SMC thong dung', 'tag','base',  'lam',3.0,           'eta',1.00), ...
             struct('name','SMC da tune',    'tag','tuned', 'lam',s.smc_lambda1, 'eta',s.smc_eta1) };

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
        fprintf(' %10.3f |', d.Jc);
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
        fprintf(' %8.3f |', d.Jc);
    end
    fprintf('\n');
end

fprintf('\n  H3. z_rms 5s cuoi (circle, nhieu 20%%)\n');
for ci = 1:numel(smc_cfgs)
    d = results_H.(sprintf('%s_d20', smc_cfgs{ci}.tag));
    fprintf('    %-16s : %.3e\n', smc_cfgs{ci}.name, d.zrms);
end

%% ================================================================
%  PHAN I: DO BEN CUA SMC DA TUNE
%  ================================================================
% Phan H chi so sanh Jc tai tham so danh nghia va mot dieu kien dau.
% Gain SMC da chinh (ctrl_params.m) duoc chon TAI DUNG diem van hanh do, nen can
% kiem tra xem uu the co ben khong hay chi ton tai quanh diem chon.
% Chay SMC da tune qua dung bo thi nghiem da ap cho ADP-FT.
fprintf('\n--- PHAN I: DO BEN CUA SMC DA TUNE ---\n');

results_I = struct();
smct = @(ss) ss;   % SMC da chinh = gia tri mac dinh trong ctrl_params.m

% --- I1. Bat dinh khoi luong ---
fprintf('\n  I1. Bat dinh khoi luong (circle, nhieu 20%%), Jc\n');
masses_I = [10, 12, 14, 16];
fprintf('  %-16s |', 'Phuong phap');
for mi = 1:numel(masses_I), fprintf(' %8dkg |', masses_I(mi)); end
fprintf('\n  %s\n', repmat('-', 1, 19 + 12*numel(masses_I)));

pm_methods = {'smc_tuned', 'adp_ft', 'bs'};
pm_names   = {'SMC da tune', 'ADP-FT', 'BS'};
for mi = 1:numel(pm_methods)
    fprintf('  %-16s |', pm_names{mi});
    for k = 1:numel(masses_I)
        s_i = s;  s_i.traj_type = 'circle';
        meth = pm_methods{mi};
        if strcmp(meth, 'smc_tuned'), s_i = smct(s_i);  meth = 'smc';  end
        pp = p;  pp.m = masses_I(k);
        pp.M = diag([pp.m, p.I]);  pp.M_inv = diag([1/pp.m, 1/p.I]);
        d = run_extra_sim(meth, s_i, p, 'common', pp);
        results_I.(sprintf('mass%d_%s', masses_I(k), pm_methods{mi})) = d;
        fprintf(' %10.3f |', d.Jc);
    end
    fprintf('\n');
end

% --- I2. Quet dieu kien dau ---
fprintf('\n  I2. Quet dieu kien dau (circle, nhieu 20%%), z_rms 5s cuoi\n');
fprintf('  %-16s |', 'Phuong phap');
for zi = 1:numel(z0_list), fprintf(' %9.3f |', z0_norm(zi)); end
fprintf('  <- ||z0||\n  %s\n', repmat('-', 1, 19 + 12*numel(z0_list)));
for mi = 1:numel(pm_methods)
    fprintf('  %-16s |', pm_names{mi});
    for zi = 1:numel(z0_list)
        s_i = s;  s_i.traj_type = 'circle';  s_i.q0_offset = z0_list{zi};
        meth = pm_methods{mi};
        if strcmp(meth, 'smc_tuned'), s_i = smct(s_i);  meth = 'smc';  end
        d = run_extra_sim(meth, s_i, p, 'common');
        results_I.(sprintf('z%d_%s', zi, pm_methods{mi})) = d;
        fprintf(' %9.2e |', d.zrms);
    end
    fprintf('\n');
end

% --- I3. Nhieu hai kenh doc lap ---
fprintf('\n  I3. Nhieu doc lap hai kenh (circle), Jc\n');
fprintf('  %-16s |', 'Phuong phap');
for di = 1:numel(dist_amps), fprintf(' %8.0f%% |', dist_amps(di)*100); end
fprintf('\n  %s\n', repmat('-', 1, 19 + 11*numel(dist_amps)));
for mi = 1:numel(pm_methods)
    fprintf('  %-16s |', pm_names{mi});
    for di = 1:numel(dist_amps)
        s_i = s;  s_i.traj_type = 'circle';  s_i.dist_amp = dist_amps(di);
        meth = pm_methods{mi};
        if strcmp(meth, 'smc_tuned'), s_i = smct(s_i);  meth = 'smc';  end
        mode_k = 'indep';  if dist_amps(di) == 0, mode_k = 'none'; end
        d = run_extra_sim(meth, s_i, p, mode_k);
        results_I.(sprintf('ind%d_%s', round(dist_amps(di)*100), pm_methods{mi})) = d;
        fprintf(' %9.3f |', d.Jc);
    end
    fprintf('\n');
end

%% ================================================================
%  PHAN J: ABLATION CO CHE + DONG BANG CRITIC
%  ================================================================
% Truoc 2026-10-26 cac so lieu nay (Muc 4.7.3, 5.7-5.8 luan van) duoc chay rieng
% le, khong nam trong script nao. Gom lai day de tai tao duoc.
%   Wang goc    -- tham so + luat cap nhat goc cua Wang et al. (muc IV-A bai bao),
%                  W(0) = 0, khong clamp uo, khong chieu, khong ghep
%   Chua co che -- gain da chinh cua luan van, chua co chieu va ghep
%   + chieu     -- chi bat toan tu chieu
%   + ghep      -- chi bat ghep nonholonomic
%   Day du      -- ca hai (mac dinh trong ctrl_params)
%   Dong bang   -- day du nhung Gamma = 0: W giu nguyen W0, critic KHONG hoc.
%                  Tra loi cau hoi "critic hoc duoc gi?"
fprintf('\n--- PHAN J: ABLATION CO CHE + DONG BANG CRITIC ---\n');

results_J = struct();
j_cfgs = { struct('name','Wang goc',    'tag','wang',   'proj',false, 'czy',0,         'gam',1, 'wang',true), ...
           struct('name','Chua co che', 'tag','goc',    'proj',false, 'czy',0,         'gam',1, 'wang',false), ...
           struct('name','+ chieu',     'tag','chieu',  'proj',true,  'czy',0,         'gam',1, 'wang',false), ...
           struct('name','+ ghep',      'tag','ghep',   'proj',false, 'czy',s.ft_c_zy, 'gam',1, 'wang',false), ...
           struct('name','Day du',      'tag','full',   'proj',true,  'czy',s.ft_c_zy, 'gam',1, 'wang',false), ...
           struct('name','Dong bang',   'tag','frozen', 'proj',true,  'czy',s.ft_c_zy, 'gam',0, 'wang',false) };
masses_J = [14, 16];

J_traj = nan(numel(j_cfgs), numel(trajs));
J_zrms = nan(numel(j_cfgs), numel(z0_list));
J_mass = nan(numel(j_cfgs), numel(masses_J));

for ci = 1:numel(j_cfgs)
    c = j_cfgs{ci};
    s_j = s;
    s_j.ft_proj  = c.proj;
    s_j.ft_c_zy  = c.czy;
    s_j.ft_Gamma = c.gam * s.ft_Gamma;
    if c.wang
        s_j.ft_lambda = [1; 1; 0.4];  s_j.ft_mu = [1; 1; 0.4];  s_j.ft_alpha = [1; 1; 0.4];
        s_j.ft_beta   = 1e5 * [1; 1; 1];  s_j.ft_rho = 10;
        s_j.ft_kappa1 = 0.1;  s_j.ft_kappa2 = 0.1;
        s_j.ft_Gamma  = 2 * eye(s.l);  s_j.ft_W0 = zeros(s.l, 1);
        s_j.ft_update_law = 'wang';  s_j.ft_uo_max = inf;
    end

    for t_idx = 1:numel(trajs)
        s_jt = s_j;  s_jt.traj_type = trajs{t_idx};
        d = run_extra_sim('adp_ft', s_jt, p, 'common');
        results_J.(sprintf('%s_%s', c.tag, trajs{t_idx})) = d;
        J_traj(ci, t_idx) = d.Jc;
    end
    for zi = 1:numel(z0_list)
        s_jz = s_j;  s_jz.traj_type = 'circle';  s_jz.q0_offset = z0_list{zi};
        d = run_extra_sim('adp_ft', s_jz, p, 'common');
        results_J.(sprintf('%s_z%d', c.tag, zi)) = d;
        J_zrms(ci, zi) = d.zrms;
    end
    for k = 1:numel(masses_J)
        s_jm = s_j;  s_jm.traj_type = 'circle';
        pp = p;  pp.m = masses_J(k);
        pp.M = diag([pp.m, p.I]);  pp.M_inv = diag([1/pp.m, 1/p.I]);
        d = run_extra_sim('adp_ft', s_jm, p, 'common', pp);
        results_J.(sprintf('%s_m%d', c.tag, masses_J(k))) = d;
        J_mass(ci, k) = d.Jc;
    end
end

fprintf('\n  J1. Jc (nhieu 20%%)\n');
fprintf('  %-12s | %8s | %8s | %8s | %8s | %8s\n', ...
    'Cau hinh', 'circle', 'line', 'figure8', '14kg', '16kg');
fprintf('  %s\n', repmat('-', 1, 68));
for ci = 1:numel(j_cfgs)
    fprintf('  %-12s | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f\n', ...
        j_cfgs{ci}.name, J_traj(ci,:), J_mass(ci,:));
end

fprintf('\n  J2. z_rms 5s cuoi theo dieu kien dau (circle, nhieu 20%%)\n');
fprintf('  %-12s |', 'Cau hinh');
for zi = 1:numel(z0_list), fprintf(' %9.3f |', z0_norm(zi)); end
fprintf('  <- ||z0||\n  %s\n', repmat('-', 1, 15 + 12*numel(z0_list)));
for ci = 1:numel(j_cfgs)
    fprintf('  %-12s |', j_cfgs{ci}.name);
    fprintf(' %9.2e |', J_zrms(ci,:));
    fprintf('\n');
end

%% ================================================================
%  PHAN K: ADP-FT CO THICH NGHI KHI DOI Q, R KHONG?
%  ================================================================
% Muc 6.3 luan van coi day la lap luan cuoi cung con lai cho ADP: SMC phai quet
% lai tham so, con ADP-FT nhan Q, R lam dau vao. Neu dung, tang Q/R phai giam
% Je (= tich phan ||z||^2) va tang Ju (= tich phan ||uo||^2), va nguoc lai.
fprintf('\n--- PHAN K: DOI Q, R ---\n');

results_K = struct();
qr_cfgs = [1 1; 0.1 1; 10 1; 1 0.1; 1 10];   % [he so nhan Q, he so nhan R]
fprintf('  %6s %6s | %8s | %8s | %9s\n', 'Q x', 'R x', 'Je', 'Ju', '||W(T)||');
fprintf('  %s\n', repmat('-', 1, 48));
for ki = 1:size(qr_cfgs, 1)
    s_k = s;  s_k.traj_type = 'circle';
    s_k.Q = qr_cfgs(ki,1) * s.Q;  s_k.R = qr_cfgs(ki,2) * s.R;  s_k.R_inv = inv(s_k.R);
    d = run_extra_sim('adp_ft', s_k, p, 'common');
    Ju = (d.Jc - s_k.Q(1,1) * d.Je) / s_k.R(1,1);   % Q, R deu la boi so cua I
    results_K.(sprintf('cfg%d', ki)) = d;
    fprintf('  %6g %6g | %8.4f | %8.4f | %9.4f\n', ...
        qr_cfgs(ki,1), qr_cfgs(ki,2), d.Je, Ju, norm(d.W(:,end)));
end

%% Luu
save('../results/thesis_extra.mat', ...
     'results_D', 'results_E', 'results_F', 'results_G', 'results_H', 'results_I', ...
     'results_J', 'J_traj', 'J_zrms', 'J_mass', 'masses_J', 'results_K', 'qr_cfgs', ...
     'T_settle', 'z0_norm', 'tol_list', 'masses_I', ...
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
