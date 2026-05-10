function [tau, info] = dynamic_backstepping(eta, eta_d, eta_d_prev, dt, p, s)
% DYNAMIC_BACKSTEPPING  Vong trong backstepping bac 2 (Fierro & Lewis 1997)
%
% [tau, info] = dynamic_backstepping(eta, eta_d, eta_d_prev, dt, p, s)
%
% Chuc nang: tinh tau = [tauR; tauL] tu sai lech van toc (v_d-v, w_d-w)
%   su dung mo hinh dong M*eta_dot = B*tau - F(eta)
%
% Luat dieu khien:
%   e = eta_d - eta                           (sai lech van toc)
%   eta_d_dot ≈ (eta_d - eta_d_prev) / dt    (dao ham so)
%   tau = B_inv * [ M*(eta_d_dot + Kd*e) + F(eta) ]
%
% Chung minh on dinh:
%   Chon V2 = 0.5*e'*M*e, lay dao ham:
%   V2_dot = e'*M*e_dot = e'*M*(eta_d_dot - eta_dot)
%          = e'*M*eta_d_dot - e'*(B*tau - F)
%   Thay tau: V2_dot = e'*M*eta_d_dot - e'*(M*eta_d_dot + M*Kd*e + F - F)
%          = -e'*M*Kd*e < 0    (voi Kd > 0)
%
% Input:
%   eta        = [v; omega]    (2x1) van toc hien tai
%   eta_d      = [v_d; w_d]   (2x1) van toc mong muon (tu vong ngoai)
%   eta_d_prev = [v_d; w_d]   (2x1) van toc mong muon buoc truoc
%   dt         -- buoc thoi gian [s]
%   p          -- wmr_params struct
%   s          -- sm1_params struct (chua Kd_v, Kd_w)
%
% Output:
%   tau  = [tauR; tauL]  (2x1) mo-men banh
%   info -- struct debug
%
% Tham khao: Fierro & Lewis (1997), Section III-B
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    04/2026

% Sai lech van toc
e_eta = eta_d - eta;

% Dao ham van toc mong muon (so, backward difference)
eta_d_dot = (eta_d - eta_d_prev) / dt;

% Vector ma sat F(eta) — giong wmr_dynamics.m
v = eta(1);
w = eta(2);
F = [p.fv * v + p.fc * tanh(v / p.eps_sign);
     p.fw * w + p.fcw * tanh(w / p.eps_sign)];

% Gain matrix
Kd = diag([s.Kd_v, s.Kd_w]);

% Luat dieu khien backstepping bac 2:
% tau_eq = M * (eta_d_dot + Kd * e_eta) + F(eta)
% tau = B_inv * tau_eq
tau_eq = p.M * (eta_d_dot + Kd * e_eta) + F;
tau = p.B_inv * tau_eq;

% Bao hoa
tau = max(-p.tau_max, min(p.tau_max, tau));

% Debug info
info.e_eta     = e_eta;
info.eta_d_dot = eta_d_dot;
info.tau_eq    = tau_eq;
info.F         = F;

end
