function s = sm1_params()
% SM1_PARAMS  Tham so cho Seminar 1: Actor-Critic ADP kinematic tracking
%
% Cach dung: s = sm1_params();
%
% Bao gom: tham so cost function, ADP learning, PE signal,
%           backstepping gains, quy dao tham chieu, mo phong.
%
% Tham khao:
%   - Vamvoudakis & Lewis (2010): Actor-Critic ADP
%   - Wang et al. (2025): basis function, Q, R
%   - Kanayama (1990): error dynamics, backstepping
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

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
s.Wc0 = [1; 1; 1; 0; 0; 0];
s.Wa0 = [1; 1; 1; 0; 0; 0];

%% === PE SIGNAL (Persistence of Excitation) ===
% n(t) = sum( A_i * sin(w_i * t) )
% Can >= l+1 = 7 tan so de thoa dieu kien PE cho basis 6 chieu

s.pe_A = [0.08, 0.06, 0.07, 0.05, 0.06, 0.04, 0.05, 0.03]; % bien do (~20% vr)
s.pe_w = [1.0, 1.5, 2.3, 3.1, 4.7, 5.3, 7.1, 11.0];       % tan so [rad/s]
s.pe_off_time = 30;             % tat PE sau 30s (weights da hoi tu)

%% === BACKSTEPPING GAINS (de so sanh) ===
% v   = vr*cos(zth) + k1*zx
% w   = wr + k2*vr*zy + k3*sin(zth)

s.k1 = 3;
s.k2 = 5;                      % lon hon vi zy hoi tu cham tren circle
s.k3 = 3;

%% === QUY DAO THAM CHIEU ===

s.traj_type = 'circle';        % 'line' hoac 'circle'

% Circle: tam (0,0), ban kinh R, di nguoc chieu kim dong ho
s.circle_R  = 2;                % [m]
s.circle_vr = 0.3;              % [m/s]

% Line: di thang theo huong angle
s.line_vr    = 0.3;             % [m/s]
s.line_angle = 0;               % [rad] huong di (0 = truc x)

%% === DIEU KIEN BAN DAU ===
% Robot bat dau lech so voi diem tham chieu tai t=0

s.q0_offset = [0.5; -0.3; 0.2]; % [dx; dy; dtheta]

%% === MO PHONG ===

s.dt    = 0.001;                % [s] buoc Euler (1 kHz)
s.T_sim = 120;                  % [s] tong thoi gian (du de ADP hoi tu)

%% === GIOI HAN DIEU KHIEN ===
% Clamp output de tranh mat on dinh

s.v_max = 1.0;                  % [m/s]
s.w_max = 2.0;                  % [rad/s]

end
