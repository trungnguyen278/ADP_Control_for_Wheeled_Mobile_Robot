function [u, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s)
% ACTOR_CRITIC_ADP  Bo dieu khien Actor-Critic ADP cho kinematic WMR tracking
%
% [u, adp, info] = actor_critic_adp(t, q, qr, vr, omegar, adp, s)
%
% Input:
%   t      -- thoi gian hien tai [s]
%   q      = [x; y; theta]     (3x1) trang thai robot
%   qr     = [xr; yr; thetar]  (3x1) trang thai tham chieu
%   vr     -- van toc dai tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%   adp    -- struct chua weights: adp.Wc (6x1), adp.Wa (6x1)
%   s      -- struct tham so tu sm1_params()
%
% Output:
%   u    = [v; omega]  (2x1) lenh dieu khien tong
%   adp  -- struct da cap nhat weights
%   info -- struct chua thong tin debug
%
% Thuat toan (Vamvoudakis & Lewis 2010):
%   1. Sai so tracking z (body frame, Wang eq.3)
%   2. Basis phi(z), Jacobian nabla_phi (6x3)
%   3. Error dynamics: z_dot = f(z,ur) + g(z)*uo (Wang eq.5)
%   4. Actor: uo = -0.5 * R_inv * g' * nabla_phi' * Wa
%   5. Bellman error: delta = Wc'*sigma + z'Qz + uo'Ruo
%   6. Critic update: Wc -= dt*(alpha_c*sigma_bar*delta + kappa_c*Wc)
%   7. Actor update:  Wa -= dt*(alpha_a1*sigma_bar*e_a + alpha_a2*(Wa-Wc))
%      trong do e_a = sigma'*Wc + z'Qz + uo'Ruo (Critic danh gia Actor)
%   8. sigma-modification (kappa_c) chong weight drift
%
% Tham khao:
%   - Vamvoudakis & Lewis (2010): Online Actor-Critic, eq.(25)-(26)
%   - Wang et al. (2025): eq.(3)-(5), basis functions
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

%% A. Sai so tracking trong body frame
ex = qr(1) - q(1);
ey = qr(2) - q(2);
theta = q(3);

zx =  cos(theta)*ex + sin(theta)*ey;
zy = -sin(theta)*ex + cos(theta)*ey;
zth = atan2(sin(qr(3) - theta), cos(qr(3) - theta));

z = [zx; zy; zth];

%% B. Basis function phi(z) va Jacobian nabla_phi (6x3)
phi = [zx^2; zy^2; zth^2; zx*zy; zy*zth; zx*zth];

nabla_phi = [
    2*zx,    0,       0;
    0,       2*zy,    0;
    0,       0,       2*zth;
    zy,      zx,      0;
    0,       zth,     zy;
    zth,     0,       zx
];  % 6x3

%% C. Error dynamics: z_dot = f + g * uo
f_z = [zy*omegar; -zx*omegar + vr*sin(zth); 0];

g_z = [-1,  zy;
        0, -zx;
        0,  -1];  % 3x2

%% D. Actor control (feedback)
uo = -0.5 * s.R_inv * g_z' * nabla_phi' * adp.Wa;

% Clamp uo de tranh feedback qua lon khi weights chua hoi tu
uo(1) = max(-s.uo_max, min(s.uo_max, uo(1)));
uo(2) = max(-s.uo_max, min(s.uo_max, uo(2)));

%% E. PE signal
if t < s.pe_off_time
    n_pe = 0;
    for i = 1:length(s.pe_A)
        n_pe = n_pe + s.pe_A(i) * sin(s.pe_w(i) * t);
    end
    uo_applied = uo + [n_pe; 0.5*n_pe];
else
    uo_applied = uo;
end

%% F. Feedforward + tong dieu khien
uf = [vr * cos(zth); omegar];
u = uf + uo_applied;

u(1) = max(-s.v_max, min(s.v_max, u(1)));
u(2) = max(-s.w_max, min(s.w_max, u(2)));

%% G. Bellman error (dung uo KHONG PE)
sigma = nabla_phi * (f_z + g_z * uo);         % 6x1
delta = adp.Wc' * sigma + z'*s.Q*z + uo'*s.R*uo;

%% H. Critic update (V&L 2010 eq.25 + sigma-modification)
% Wc_dot = -alpha_c * sigma_bar * delta - kappa_c * Wc
sigma_bar = sigma / (1 + sigma'*sigma)^2;
adp.Wc = adp.Wc - s.dt * (s.alpha_c * sigma_bar * delta + s.kappa_c * adp.Wc);

%% I. Actor update (V&L 2010 eq.26)
% Critic danh gia Actor: dung Wc de tinh Bellman error cho Actor
% e_a = sigma' * Wc + z'Qz + uo'Ruo  (Wc, KHONG phai Wa)
% Gradient Actor: sigma_a tinh voi chinh sach Actor (uo dung Wa)
% Keo Wa ve Wc: alpha_a2 * (Wa - Wc)
e_a = sigma' * adp.Wc + z'*s.Q*z + uo'*s.R*uo;   % Critic danh gia

adp.Wa = adp.Wa - s.dt * (s.alpha_a1 * sigma_bar * e_a ...
                         + s.alpha_a2 * (adp.Wa - adp.Wc));

%% Output info
info.z     = z;
info.uo    = uo;
info.delta = delta;
info.phi   = phi;
info.sigma = sigma;

end
