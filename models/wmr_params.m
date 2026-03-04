function p = wmr_params()
% WMR_PARAMS  Tham so robot WMR kieu vi sai (Pioneer 3-DX)
%
% Cach dung: p = wmr_params();
%
% Tham khao: Fierro & Lewis (1997), Pioneer 3-DX Datasheet
% Tac gia:   Nguyen Thanh Trung
% Ngay:      02/2026

%% === THONG SO CO HOC ===

% Khoi luong tong (than + pin + payload)
p.m = 10;               % [kg]

% Mo-men quan tinh quanh truc dung qua tam truc banh
p.I = 0.5;              % [kg*m^2]

% Ban kinh banh xe
p.r = 0.05;             % [m]

% Nua khoang cach truc (half-track width)
p.L = 0.15;             % [m]

%% === MA SAT ===

% Ma sat nhot (viscous)
p.fv = 0.5;             % [N*s/m]     tinh tien
p.fw = 0.1;             % [N*m*s/rad] quay

% Ma sat Coulomb (dry friction)
p.fc = 0.3;             % [N]         tinh tien
p.fcw = 0.1;            % [N*m]       quay

% Xap xi muot cho sign(): tanh(v/epsilon)
p.eps_sign = 0.01;

%% === GIOI HAN CO CAU CHAP HANH ===

p.tau_max = 5;          % [N*m] mo-men toi da moi banh
p.v_max   = 1.0;        % [m/s]
p.w_max   = 2.0;        % [rad/s]

%% === MA TRAN TINH TRUOC (HANG SO) ===

% Ma tran quan tinh M va nghich dao
p.M     = diag([p.m, p.I]);
p.M_inv = diag([1/p.m, 1/p.I]);

% Ma tran chuyen doi mo-men: tau_eq = B * [tauR; tauL]
p.B = [p.r/2,        p.r/2;
       p.r/(2*p.L), -p.r/(2*p.L)];

% Nghich dao: [tauR; tauL] = B_inv * [Fv; Tw]
p.B_inv = inv(p.B);

% Self-test
assert(norm(p.B * p.B_inv - eye(2)) < 1e-10, ...
    'LOI: B * B_inv khac I');

%% === THONG SO MO PHONG ===

p.dt    = 0.001;        % [s] buoc thoi gian (1 kHz)
p.T_sim = 60;           % [s] thoi gian mo phong mac dinh

end
