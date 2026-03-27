%% TEST_OPENLOOP  Kiem tra mo hinh WMR bang 4 bai test vong ho
%
% Muc dich: Xac nhan mo hinh dynamic dung TRUOC KHI gan controller
% Moi test kiem tra 1 tinh chat vat ly cu the.
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    02/2026

clear; close all; clc;
addpath('../models');   % dieu chinh neu can

p = wmr_params();
fprintf('\n========== WMR DYNAMIC MODEL - OPEN LOOP TESTS ==========\n');
fprintf('Robot: m=%.1f kg, I=%.2f kg*m^2, r=%.3f m, L=%.3f m\n', ...
    p.m, p.I, p.r, p.L);
fprintf('Ma sat: fv=%.2f, fw=%.2f, fc=%.2f, fcw=%.2f\n', ...
    p.fv, p.fw, p.fc, p.fcw);
fprintf('tau_max = %.1f N*m\n\n', p.tau_max);

state0_rest = [0; 0; 0; 0; 0];  % goc toa do, dung yen
tol = 0.01;                      % nguong sai so chap nhan

%% ================================================================
%  TEST 1: tau = 0, robot dang chay -> phai dung lai do ma sat
%  ================================================================
fprintf('--- Test 1: Coasting (tau = 0) ---\n');

state0_coast = [0; 0; 0; 0.5; 0.3];  % v0=0.5, w0=0.3
tau_zero = [0; 0];
T1 = 60;

odefun1 = @(t, s) wmr_full_model(t, s, tau_zero, p);
[t1, s1] = ode45(odefun1, [0 T1], state0_coast);

v_final1 = s1(end, 4);
w_final1 = s1(end, 5);
fprintf('  v(%.0fs) = %.6f m/s   (ky vong ~ 0)\n', T1, v_final1);
fprintf('  w(%.0fs) = %.6f rad/s (ky vong ~ 0)\n', T1, w_final1);
assert(abs(v_final1) < tol, 'FAIL: v khong giam ve 0');
assert(abs(w_final1) < tol, 'FAIL: w khong giam ve 0');
fprintf('  => PASS\n\n');

% Ve hinh
figure('Name', 'Test 1: Coasting (tau=0)', 'Position', [50 500 700 400]);
subplot(2,1,1);
plot(t1, s1(:,4), 'b-', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('v [m/s]');
title('Test 1: Van toc dai — tau=0, giam dan ve 0');
grid on;
subplot(2,1,2);
plot(t1, s1(:,5), 'r-', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('\omega [rad/s]');
title('Test 1: Van toc goc — tau=0, giam dan ve 0');
grid on;

%% ================================================================
%  TEST 2: tau = const -> kiem tra van toc xac lap v_ss
%  ================================================================
fprintf('--- Test 2: Constant torque (di thang) ---\n');

tau_const = [5; 5];
T2 = 120;

odefun2 = @(t, s) wmr_full_model(t, s, tau_const, p);
[t2, s2] = ode45(odefun2, [0 T2], state0_rest);

% Ly thuyet: tai xac lap, v_dot = 0
%   B(1,:)*tau = fv*v_ss + fc*sign(v_ss)
%   => v_ss = (F_push - fc) / fv   (khi F_push > fc)
F_push_v = p.B(1,:) * tau_const;
v_ss_theory = (F_push_v - p.fc) / p.fv;

% Omega phai ~ 0 vi tauR = tauL
F_push_w = p.B(2,:) * tau_const;  % = 0

v_ss_sim = s2(end, 4);
w_ss_sim = s2(end, 5);
err_v2 = abs(v_ss_sim - v_ss_theory);

fprintf('  F_push = %.4f N\n', F_push_v);
fprintf('  v_ss (ly thuyet) = %.4f m/s\n', v_ss_theory);
fprintf('  v_ss (sim)       = %.4f m/s\n', v_ss_sim);
fprintf('  Sai so           = %.6f m/s\n', err_v2);
fprintf('  omega_ss (sim)   = %.6f rad/s (ky vong ~ 0)\n', w_ss_sim);
assert(err_v2 < tol, 'FAIL: v_ss sai so qua lon');
assert(abs(w_ss_sim) < tol, 'FAIL: omega khac 0 khi tauR=tauL');
fprintf('  => PASS\n\n');

% Ve hinh
figure('Name', 'Test 2: Constant torque', 'Position', [50 50 700 400]);
subplot(2,1,1);
plot(t2, s2(:,4), 'b-', 'LineWidth', 1.5); hold on;
yline(v_ss_theory, 'r--', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('v [m/s]');
title(sprintf('Test 2: v(t) \\rightarrow v_{ss} = %.3f m/s', v_ss_theory));
legend('Simulation', 'Ly thuyet', 'Location', 'southeast');
grid on;
subplot(2,1,2);
plot(s2(:,1), s2(:,2), 'b-', 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]');
title('Test 2: Quy dao xy (di thang)');
axis equal; grid on;

%% ================================================================
%  TEST 3: tauR != tauL -> quy dao cong, kiem tra ban kinh quay
%  ================================================================
fprintf('--- Test 3: Differential torque (quay) ---\n');

tau_turn = [5; 3];
T3 = 120;

odefun3 = @(t, s) wmr_full_model(t, s, tau_turn, p);
[t3, s3] = ode45(odefun3, [0 T3], state0_rest);

% Ly thuyet xac lap
F_push_v3 = p.B(1,:) * tau_turn;
F_push_w3 = p.B(2,:) * tau_turn;

% v_ss va w_ss (giai phuong trinh xac lap)
v_ss_3 = (F_push_v3 - p.fc) / p.fv;
w_ss_3 = (F_push_w3 - p.fcw) / p.fw;
R_theory = abs(v_ss_3 / w_ss_3);

v_end3 = s3(end, 4);
w_end3 = s3(end, 5);
R_sim = abs(v_end3 / w_end3);

fprintf('  tau = [%.0f; %.0f] N*m\n', tau_turn(1), tau_turn(2));
fprintf('  v_ss: ly thuyet=%.4f, sim=%.4f\n', v_ss_3, v_end3);
fprintf('  w_ss: ly thuyet=%.4f, sim=%.4f\n', w_ss_3, w_end3);
fprintf('  R:    ly thuyet=%.3f m, sim=%.3f m\n', R_theory, R_sim);
fprintf('  Sai so R = %.4f m\n', abs(R_sim - R_theory));
assert(abs(R_sim - R_theory) < 0.1, 'FAIL: Ban kinh quay sai');
fprintf('  => PASS\n\n');

% Ve hinh
figure('Name', 'Test 3: Turning', 'Position', [800 500 700 400]);
subplot(1,2,1);
plot(s3(:,1), s3(:,2), 'b-', 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]');
title('Test 3: Quy dao xy (quay)');
axis equal; grid on;
subplot(1,2,2);
plot(t3, s3(:,4), 'b-', t3, s3(:,5), 'r-', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('Van toc');
legend('v [m/s]', '\omega [rad/s]', 'Location', 'southeast');
title('Test 3: Van toc xac lap');
grid on;

%% ================================================================
%  TEST 4: tau = 100 (vuot tau_max) -> saturation clip ve tau_max
%  ================================================================
fprintf('--- Test 4: Saturation ---\n');

tau_overload = [100; 100];
T4 = 120;

odefun4 = @(t, s) wmr_full_model(t, s, tau_overload, p);
[t4, s4] = ode45(odefun4, [0 T4], state0_rest);

% Phai giong Test 2 vi dynamics clip ve [5; 5]
v_ss_4 = s4(end, 4);
v_ss_clipped = (p.B(1,:) * [p.tau_max; p.tau_max] - p.fc) / p.fv;

err_v4 = abs(v_ss_4 - v_ss_clipped);
fprintf('  tau yeu cau = [100; 100], tau thuc = [%.0f; %.0f]\n', ...
    p.tau_max, p.tau_max);
fprintf('  v_ss (sim)     = %.4f m/s\n', v_ss_4);
fprintf('  v_ss (clipped) = %.4f m/s\n', v_ss_clipped);
fprintf('  Sai so = %.6f m/s\n', err_v4);
assert(err_v4 < tol, 'FAIL: Saturation khong hoat dong');
fprintf('  => PASS\n\n');

%% ================================================================
%  TEST 5 (BONUS): Nang luong bao toan
%  Kiem tra: KE(0) = cong ma sat (vi tau=0)
%  ================================================================
fprintf('--- Test 5 (bonus): Kiem tra nang luong ---\n');

% Tu Test 1: KE_0 = 0.5*m*v0^2 + 0.5*I*w0^2
v0 = 0.5; w0 = 0.3;
KE_0 = 0.5*p.m*v0^2 + 0.5*p.I*w0^2;

% Nang luong dong hoc cuoi (phai ~ 0)
KE_end = 0.5*p.m*s1(end,4)^2 + 0.5*p.I*s1(end,5)^2;

% Cong ma sat (tich phan so)
W_friction = 0;
for k = 1:length(t1)-1
    dt_k = t1(k+1) - t1(k);
    vk = s1(k, 4);
    wk = s1(k, 5);
    Fv_k = p.fv*vk + p.fc*tanh(vk/p.eps_sign);
    Fw_k = p.fw*wk + p.fcw*tanh(wk/p.eps_sign);
    % Cong = luc * van toc * dt (cong suat tuc thoi)
    W_friction = W_friction + (Fv_k*vk + Fw_k*wk)*dt_k;
end

fprintf('  KE ban dau     = %.4f J\n', KE_0);
fprintf('  KE cuoi        = %.6f J (~ 0)\n', KE_end);
fprintf('  Cong ma sat    = %.4f J\n', W_friction);
fprintf('  Sai lech       = %.4f J (%.1f%%)\n', ...
    abs(KE_0 - W_friction), abs(KE_0 - W_friction)/KE_0*100);
% Cho phep sai so 10% do tich phan so
if abs(KE_0 - W_friction)/KE_0 < 0.10
    fprintf('  => PASS (sai so < 10%%)\n\n');
else
    fprintf('  => WARNING: sai lech lon, kiem tra lai\n\n');
end

%% ================================================================
%  TONG KET
%  ================================================================
fprintf('========== TAT CA TESTS PASS ==========\n');
fprintf('Mo hinh WMR dynamic san sang!\n\n');

% Luu ket qua
save('../results/test_openloop_results.mat', ...
    't1','s1', 't2','s2', 't3','s3', 't4','s4', 'p');
fprintf('Da luu ket qua vao results/test_openloop_results.mat\n');
