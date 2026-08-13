function s = ctrl_params()
% CTRL_PARAMS  Tham so controller + mo phong dung chung cho CA 6 phuong phap
%
% Cach dung: s = ctrl_params();
%
% Bao gom: cost function, ADP learning, PE signal, backstepping gains,
%          SMC, NTSMC, ADP fixed-time, Concurrent Learning,
%          quy dao tham chieu, nhieu, tham so mo phong.
%
% LUU Y: file nay truoc day ten sm1_params.m (chi phuc vu Seminar 1).
% Da doi ten 2026-08-12 vi no giu tham so cho toan bo cac phuong phap,
% khong con gioi han o SM1.
%
% Tham khao:
%   - Vamvoudakis & Lewis (2010): Actor-Critic ADP
%   - Wang et al. (2025): basis function, Q, R, ADP fixed-time
%   - Kanayama (1990): error dynamics, backstepping
%   - Feng et al. (2002): Non-singular Terminal SMC
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026 (doi ten 08/2026)

%% === COST FUNCTION ===
% J = integral( z'*Q*z + uo'*R*uo ) dt

s.Q = diag([10, 10, 10]);      % 3x3, phat sai so tracking
s.R = diag([2, 2]);             % 2x2, phat nang luong dieu khien
s.R_inv = inv(s.R);             % tinh truoc de tiet kiem

%% === BASIS FUNCTION ===
% phi(z) = [zx^2; zy^2; zth^2; zx*zy; zy*zth; zx*zth]
% l = 6 (so chieu basis)

s.l = 6;

%% === ACTOR-CRITIC LEARNING RATES ===

% Critic update: Wc_dot = -alpha_c * sigma_bar * delta - kappa_c * Wc
s.alpha_c = 0.5;
s.kappa_c = 0.01;               % sigma-modification (weight decay) cho Critic

% Actor update (V&L 2010 eq.26):
%   Wa_dot = -alpha_a1 * sigma_a_bar * e_a - alpha_a2 * (Wa - Wc)
s.alpha_a1 = 0.5;               % Actor gradient learning rate
s.alpha_a2 = 1.0;               % keo Wa ve Wc

% Gioi han feedback uo (chong phat tan khi weights chua hoi tu)
s.uo_max = 1.0;                 % [m/s] va [rad/s]

%% === TRONG SO BAN DAU ===

% LUU Y: W0 != 0 la "warm start" — tuong duong P-controller nho.
% Neu W0 = 0 → khong co feedback ban dau → he mat on dinh (da kiem chung).
% Day la HAN CHE cua Actor-Critic: can chinh sach ban dau on dinh.
% V ~ 1*(zx^2 + zy^2 + zth^2) → uo ~ [0.5*zx; 0.5*zth] gan z=0
% Yeu, chi du on dinh — buoc ADP phai hoc de cai thien
s.Wc0 = [3; 3; 3; 0; 0; 0];
s.Wa0 = [3; 3; 3; 0; 0; 0];

%% === PE SIGNAL (Persistence of Excitation) ===
% n(t) = sum( A_i * sin(w_i * t) )
% Can >= l+1 = 7 tan so de thoa dieu kien PE cho basis 6 chieu

s.pe_A = [0.08, 0.06, 0.07, 0.05, 0.06, 0.04, 0.05, 0.03]; % bien do (~20% vr)
s.pe_w = [1.0, 1.5, 2.3, 3.1, 4.7, 5.3, 7.1, 11.0];       % tan so [rad/s]
s.pe_off_time = 30;             % tat PE sau 30s (weights da hoi tu)

%% === BACKSTEPPING GAINS (de so sanh) ===
% v   = vr*cos(zth) + k1*zx
% w   = wr + k2*vr*zy + k3*sin(zth)

s.k1 = 1.5;                     % giam cho phu hop dynamic (v accel cham)
s.k2 = 3;
s.k3 = 2;

%% === QUY DAO THAM CHIEU ===

s.traj_type = 'circle';        % 'line' hoac 'circle'

% Circle: tam (0,0), ban kinh R, di nguoc chieu kim dong ho
s.circle_R  = 2;                % [m]
s.circle_vr = 0.3;              % [m/s]

% Line: di thang theo huong angle
s.line_vr    = 0.3;             % [m/s]
s.line_angle = 0;               % [rad] huong di (0 = truc x)

%% === QUY DAO HINH SO 8 ===
% x(t) = a*sin(w*t), y(t) = b*sin(2*w*t)
s.fig8_a     = 2;               % [m] bien do x
s.fig8_b     = 1;               % [m] bien do y
s.fig8_omega = 0.1;             % [rad/s] => T ~ 63s (cham hon de ADP hoi tu)

%% === DIEU KIEN BAN DAU ===
% Robot bat dau lech so voi diem tham chieu tai t=0

s.q0_offset = [0.2; -0.1; 0.1]; % [dx; dy; dtheta] nho hon cho full model

%% === MO PHONG ===

s.dt    = 0.001;                % [s] buoc Euler (1 kHz)
s.T_sim = 120;                  % [s] tong thoi gian (du de ADP hoi tu)

%% === GIOI HAN DIEU KHIEN ===
% Clamp output de tranh mat on dinh

s.v_max = 0.5;                  % [m/s]
s.w_max = 2.0;                  % [rad/s]

%% === DYNAMIC BACKSTEPPING (VONG TRONG, Fierro & Lewis 1997) ===
% tau = B_inv * [ M*(eta_d_dot + Kd*e) + F(eta) ]
% Kd lon => vong trong nhanh (separation of timescales)

s.Kd_v = 20;                    % gain van toc dai
s.Kd_w = 20;                    % gain van toc goc

%% === SMC KINEMATIC (VONG NGOAI) ===
% uo1 = zy*(wr+uo2) + lambda1*zx + eta1*tanh(zx/delta)
% uo2 = lambda2*zth + eta2*tanh(zth/delta)

s.smc_lambda1 = 3;              % equivalent gain zx
s.smc_lambda2 = 3;              % equivalent gain zth
s.smc_eta1    = 1.0;            % reaching gain zx
s.smc_eta2    = 1.0;            % reaching gain zth
s.smc_delta   = 0.05;           % boundary layer (tanh thay sign)
s.smc_c_zy    = 1.0;            % coupling zy vao mat truot sigma_2 = zth + c_zy*zy

%% === NTSMC KINEMATIC (Non-singular Fast Terminal SMC, VONG NGOAI) ===
% sigma_2 = zth + c_zy*zy + (1/beta)*sig_ns(zy, alpha_s)
% uo      = lambda*sigma + eta*sig_ns(sigma, alpha_r)
%
% Khac SMC thuong o 2 diem:
%   1. Mat truot co so hang |zy|^alpha_s  => zy hoi tu HUU HAN thoi gian
%   2. Luat tien toi dung |sigma|^alpha_r => den mat truot HUU HAN thoi gian
% (SMC thuong chi hoi tu MU vi toan bo la tuyen tinh + tanh)

% Gain duoi day CHON BANG SWEEP tren full model (circle, T=60s, co nhieu),
% khong phai copy tu bai bao. Chi tiet sweep: xem PROGRESS.md muc 2026-08-12.
% Gain ban dau (lambda=3, eta=1.0 copy tu SMC) cho Jc=1458 -- qua manh cho
% dual-loop, dung bai hoc trong memory/feedback_adp_ft_tuning.md.

s.nt_lambda1 = 0.7;             % equivalent gain sigma_1 (sweep: 0.7 toi uu)
s.nt_lambda2 = 0.7;             % equivalent gain sigma_2
s.nt_eta1    = 0.05;            % reaching gain sigma_1 (sweep: 0.05)
s.nt_eta2    = 0.05;            % reaching gain sigma_2
s.nt_c_zy    = 1.0;             % coupling zy vao mat truot (giong smc_c_zy)
s.nt_beta    = 10.0;            % he so so hang lu thua phan so tren zy
                                % beta lon => so hang terminal yeu di
s.nt_alpha_s = 0.6;             % lu thua mat truot, 0<alpha_s<1 (huu han thoi gian)
s.nt_alpha_r = 0.7;             % lu thua luat tien toi, 0<alpha_r<1
s.nt_eps     = 0.02;            % nguong noi tuyen tinh chong ky di
                                % nho => bam sat |x|^a; lon => muot hon, chan tot hon

% ===== KET QUA SWEEP (trung thuc, can biet khi viet luan van) =====
% 1. Optimum thuc su nam o beta -> vo cung, tuc TAT HAN so hang terminal.
%    beta=10 cho Jc=80.12; beta=1e6 cho Jc=79.97. Chenh 0.2%.
% 2. alpha_s KHONG anh huong gi (4 gia tri cho ket qua giong het chu so).
% 3. alpha_r anh huong ~0.08%.
% => Cai thien tu Jc=1458 xuong Jc=80 den TU VIEC GIAM GAIN, khong phai tu
%    cau truc terminal. Tren bai toan nay NTSMC khong hon SMC tune tot.
%    KHONG duoc viet trong luan van rang NTSMC tot hon SMC nho hoi tu huu han.

%% === NHIEU TAN SO CAO ===
% d(t) = d_amp * tau_max * sin(w_d * t), cong vao tau truoc plant
% Mo phong nhieu ngoai (rung, mat duong, ...)

s.dist_amp   = 0.2;             % 20% tau_max
s.dist_freq  = 30;              % [rad/s] ~ 4.8 Hz

%% === ADP FIXED-TIME (WANG ET AL. 2025, eq.16-18) ===
% Controller chinh cua luan van
% u = uf + uo_adp - g_pinv * robust
% robust = lambda.*tanh(z/rho) + mu.*z + alpha.*sig(z,p/q) + beta.*z^3
% W_dot = 0.5*Gamma*(nabla_phi*g*R_inv*g'*z - kappa1*W - kappa2*(W'W)*W)

s.ft_p = 17;                    % fixed-time exponent (tu so)
s.ft_q = 19;                    % fixed-time exponent (mau so), p/q < 1
s.ft_Gamma = 1 * eye(s.l);     % 6x6 learning rate (giam tu 2 cho on dinh dual-loop)
s.ft_kappa1 = 0.04;            % sigma-modification (tang tu 0.02 chong weight drift)
s.ft_kappa2 = 0.01;            % sigma-modification bac 3 (fixed-time)
s.ft_lambda = [0.08; 0.08; 0.05]; % robust gain: tanh term (giam cho dual-loop)
s.ft_mu     = [0.08; 0.08; 0.05]; % robust gain: linear term
s.ft_alpha  = [0.08; 0.08; 0.05]; % robust gain: fractional power |z|^{p/q}*sign(z)
s.ft_beta   = [0.15; 0.15; 0.08]; % robust gain: cubic (giam manh, inner loop da reject dist)
s.ft_rho    = 0.1;                 % boundary layer cho tanh(z/rho)
s.ft_uo_max = 1.5;                 % gioi han feedback uo (chong phat tan dual-loop)
s.ft_W0     = [3; 3; 3; 0; 0; 0]; % warm start (tuong tu CL, can feedback ban dau)

% Toan tu chieu: giu V_hat = W'*phi xac dinh duong (Ioannou & Sun 1996)
% Khong co rang buoc nay, W co the troi sang vung W1 < 0 khi z0 lon
% => V_hat het la ham Lyapunov => mat bam. Da quan sat W1 = -1.486.
% Bat mac dinh tu 2026-08-13. Khong kich hoat o dieu kien binh thuong nen
% Jc tren 3 quy dao khong doi; chi co tac dung khi W co xu huong troi.
s.ft_proj      = true;
s.ft_proj_eps  = 0.05;             % tri rieng nho nhat cho phep cua P
s.ft_proj_wmax = 20;               % chan ||W||

%% === CRITIC-ONLY + CONCURRENT LEARNING (SM2) ===
% 1 mang Critic, khong can PE, dung history stack
% uo = -0.5 * R_inv * g' * nabla_phi' * W
% W_dot = -alpha*sigma_bar*delta - alpha_hist*CL_term - kappa*W

s.cl_alpha      = 0.5;         % online learning rate
s.cl_alpha_hist = 0.5;         % concurrent learning rate (tu history stack)
s.cl_kappa      = 0.01;        % sigma-modification (weight decay)
s.cl_stack_max  = 200;         % so diem lich su toi da
s.cl_record_dt  = 0.05;        % ghi moi 50ms = 20 Hz
s.cl_W0 = [3; 3; 3; 0; 0; 0]; % warm start (can feedback ban dau de on dinh)
s.cl_uo_max     = 1.5;         % gioi han feedback [m/s] va [rad/s]

end
