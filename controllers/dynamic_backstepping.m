function [tau, info] = dynamic_backstepping(eta, eta_d, eta_d_prev, dt, p, s, z)
% DYNAMIC_BACKSTEPPING  Buoc 2 cua bo dieu khien backstepping (dong luc hoc -> mo-men)
%
% [tau, info] = dynamic_backstepping(eta, eta_d, eta_d_prev, dt, p, s, z)
%
% Bo dieu khien la MOT bo backstepping hai buoc cho he bac hai:
%   Buoc 1 (dong hoc): luat van toc ao eta_d = u_f + u_o(z)   (ADP-FT hoac luat doi chung)
%   Buoc 2 (ham nay) : mo-men tau dua van toc thuc eta ve eta_d
% Mo hinh: M*eta_dot = B*tau - F(eta), B = (1/r)[1 1; L -L] (Fierro & Lewis 1997)
%
% Luat dieu khien:
%   e   = eta_d - eta                                  (sai lech giua hai buoc)
%   B*tau = M*( eta_d_dot + Kd*e + Ka*sig(e)^(p/q) + Kb*e.^3 ) + F(eta) - Kc*g(z)'*z
%   g(z) = [-1 zy; 0 -zx; 0 -1]  => -Kc*g(z)'*z = Kc*[zx; zth]
%
% Vi sao co so hang cheo -Kc*g'*z:
%   Van toc thuc eta = eta_d - e nen z_dot = f + g*uo - g*e. Voi V1 = 0.5*z'*z
%   (ham Lyapunov cua buoc 1) thi V1_dot co them -z'*g*e.
%   Chon ham chung V = V1 + 0.5*e'*inv(Kc)*M*e,  Kc = diag(kc_v, kc_w) > 0:
%     V_dot = V1_dot|_{eta = eta_d}
%             - e'*inv(Kc)*M*( Kd*e + Ka*sig(e)^(p/q) + Kb*e.^3 ) - e'*inv(Kc)*B*d
%   so hang cheo -z'*g*e va +e'*g'*z triet nhau => buoc 2 khong lam xau
%   tinh chat cua buoc 1, va e hoi tu co dinh thoi gian (thuc dung neu co nhieu d).
%   Kc = 0, Ka = Kb = 0: luat cu (truoc 2026-10-27), khong co ham Lyapunov chung.
%
% Input:
%   eta        = [v; omega]   (2x1) van toc hien tai
%   eta_d      = [v_d; w_d]   (2x1) van toc ao tu buoc 1
%   eta_d_prev = [v_d; w_d]   (2x1) van toc ao buoc truoc (dao ham so)
%   dt         -- buoc thoi gian [s]
%   p          -- wmr_params struct
%   s          -- ctrl_params struct (Kd_v, Kd_w, in_kc, in_alpha, in_beta, in_p, in_q)
%                 in_kc, in_alpha, in_beta la vector 2x1 [kenh v; kenh omega]
%   z          -- [zx; zy; zth] (3x1) sai so bam; bo trong => khong co so hang cheo
%
% Output:
%   tau  = [tauR; tauL]  (2x1) mo-men banh (da bao hoa o +-tau_max)
%   info -- struct debug
%
% Tham khao: Fierro & Lewis (1997) cho dang mo-men tinh toan; so hang cheo va
% cac so hang co dinh thoi gian: luan van Muc 4.3.
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    04/2026 (buoc 2 day du: 10/2026)

if nargin < 7 || isempty(z), z = zeros(3, 1); end

% Sai lech van toc giua hai buoc
e_eta = eta_d - eta;

% Dao ham van toc ao (so, backward difference)
eta_d_dot = (eta_d - eta_d_prev) / dt;

% Vector ma sat F(eta) — giong wmr_dynamics.m
v = eta(1);
w = eta(2);
F = [p.fv * v + p.fc * tanh(v / p.eps_sign);
     p.fw * w + p.fcw * tanh(w / p.eps_sign)];

% Gain (mac dinh 0 cho cac so hang moi => tuong thich script cu)
Kd = diag([s.Kd_v, s.Kd_w]);
Kc = [0; 0];  Ka = [0; 0];  Kb = [0; 0];  pq = 1;
if isfield(s, 'in_kc'),    Kc = s.in_kc;    end
if isfield(s, 'in_alpha'), Ka = s.in_alpha; end
if isfield(s, 'in_beta'),  Kb = s.in_beta;  end
if isfield(s, 'in_p') && isfield(s, 'in_q'), pq = s.in_p / s.in_q; end   % so mu rieng cua buoc 2

% So hang co dinh thoi gian theo e (giong cau truc buoc 1)
ft_terms = Ka .* sign(e_eta) .* abs(e_eta).^pq + Kb .* e_eta.^3;

% So hang cheo: triet -z'*g*e trong dao ham V1
g = [-1, z(2); 0, -z(1); 0, -1];
cross = -Kc .* (g' * z);           % = Kc.*[zx; zth]

tau_eq = p.M * (eta_d_dot + Kd * e_eta + ft_terms) + F + cross;
tau = p.B_inv * tau_eq;

% Bao hoa
tau = max(-p.tau_max, min(p.tau_max, tau));

% Debug info
info.e_eta     = e_eta;
info.eta_d_dot = eta_d_dot;
info.tau_eq    = tau_eq;
info.F         = F;
info.cross     = cross;

end
