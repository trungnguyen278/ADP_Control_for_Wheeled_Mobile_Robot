%% SWEEP_GAINS  Chinh gain deu nhau cho moi phuong phap (mo hinh da sua B)
%
% Muc dich: so sanh cong bang. Moi phuong phap duoc chinh voi CUNG cong suc:
%   - mot he so nhan k ap len toan bo gain phan hoi danh nghia,
%   - cung luoi k = 2.^(-4:2) (7 gia tri, tu 1/16 den 4),
%   - cung tieu chi: tong Jc tren 3 quy dao (nhieu 20%, z0 chuan, T = 60 s).
%
% Gain danh nghia (k = 1) va tham so duoc nhan:
%   BS     : k1 = 1.5, k2 = 3, k3 = 2
%   SMC    : lambda = 3, eta = 1 (gia tri thong dung)
%   ADP-FT : lambda, mu, alpha, beta (robust), c_zy (ghep nonholonomic), W0
%   CL     : W0 = [3 3 3 0 0 0] -- khong co gain phan hoi co dinh, W0 dat
%            chinh sach ban dau
%   ADP-AC : Wc0 = Wa0 = [3 3 3 0 0 0]
% NTSMC khong chinh rieng: dung chung gain voi SMC da chinh (doi chung cung gain).
%
% Gia tri danh nghia ghi cung trong scale_gains() de lan chay lai khong phu
% thuoc gia tri da chon (ghi trong ctrl_params.m).
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    10/2026

clear; close all; clc;
addpath('../models', '../controllers');

p = wmr_params();
s0 = ctrl_params();
s0.T_sim = 60;

k_list  = 2.^(-4:2);
trajs   = {'circle', 'line', 'figure8'};
methods = {'bs', 'smc', 'adp_ft', 'cl', 'adp'};
names   = {'BS', 'SMC', 'ADP-FT', 'CL', 'ADP-AC'};

fprintf('\n========== CHINH GAIN DEU NHAU (k = 1/16 .. 4) ==========\n');

J = nan(numel(methods), numel(k_list), numel(trajs));
for mi = 1:numel(methods)
    for ki = 1:numel(k_list)
        s = scale_gains(s0, methods{mi}, k_list(ki));
        for ti = 1:numel(trajs)
            s.traj_type = trajs{ti};
            d = run_extra_sim(methods{mi}, s, p, 'common');
            J(mi, ki, ti) = d.Jc;
        end
        fprintf('  %-7s k = %-6.4g | circle %8.3f | line %8.3f | figure8 %8.3f | tong %8.3f\n', ...
            names{mi}, k_list(ki), J(mi, ki, 1), J(mi, ki, 2), J(mi, ki, 3), sum(J(mi, ki, :)));
    end
end

fprintf('\n  KET QUA: k tot nhat theo tong Jc 3 quy dao\n');
fprintf('  %-7s | %6s | %10s | %10s\n', 'PP', 'k*', 'tong(k*)', 'tong(k=1)');
fprintf('  %s\n', repmat('-', 1, 44));
Jsum = sum(J, 3);
for mi = 1:numel(methods)
    [Jbest, kb] = min(Jsum(mi, :));
    fprintf('  %-7s | %6.4g | %10.3f | %10.3f\n', ...
        names{mi}, k_list(kb), Jbest, Jsum(mi, k_list == 1));
end

save('../results/sweep_gains.mat', 'J', 'k_list', 'trajs', 'methods', 's0', 'p');
fprintf('\nDa luu vao results/sweep_gains.mat\n');

%% ====================================================================
function s = scale_gains(s, method, k)
% SCALE_GAINS  Nhan toan bo gain phan hoi danh nghia cua mot phuong phap voi k
    switch method
        case 'bs'
            s.k1 = 1.5 * k;  s.k2 = 3 * k;  s.k3 = 2 * k;
        case 'smc'
            s.smc_lambda1 = 3 * k;  s.smc_lambda2 = 3 * k;
            s.smc_eta1    = 1 * k;  s.smc_eta2    = 1 * k;
        case 'adp_ft'
            s.ft_lambda = [0.08; 0.08; 0.05] * k;
            s.ft_mu     = [0.08; 0.08; 0.05] * k;
            s.ft_alpha  = [0.08; 0.08; 0.05] * k;
            s.ft_beta   = [0.15; 0.15; 0.08] * k;
            s.ft_c_zy   = 2.0 * k;
            s.ft_W0     = [3; 3; 3; 0; 0; 0] * k;
        case 'cl'
            s.cl_W0 = [3; 3; 3; 0; 0; 0] * k;
        case 'adp'
            s.Wc0 = [3; 3; 3; 0; 0; 0] * k;
            s.Wa0 = [3; 3; 3; 0; 0; 0] * k;
    end
end
