function [u, W, info] = adp_fixed_time(q, qr, vr, omegar, W, s)
% ADP_FIXED_TIME  Critic-only ADP voi hoi tu co dinh thoi gian (Wang et al. 2025)
%
% [u, W, info] = adp_fixed_time(q, qr, vr, omegar, W, s)
%
% Implement eq.(16)-(18) tu Wang et al., IEEE RA-L, vol.10, no.1, 2025.
%
% Dieu khien:
%   u  = uf + uo                                                  (eq.14)
%   uf = [vr*cos(zth); omegar]                                    (feedforward)
%   uo = uo_adp - g_pinv * robust                                 (eq.18)
%   uo_adp = -0.5 * R_inv * g' * nabla_phi' * W                  (eq.16)
%   robust = lambda.*tanh(z/rho) + mu.*z + alpha.*sig(z,p/q) + beta.*z^3
%
% Cap nhat trong so (eq.17, Bellman error gradient):
%   sigma = nabla_phi*(f+g*uo_adp), epsilon = W'*sigma + Q(z) + R(uo_adp)
%   W_dot = -0.5*Gamma*(sigma_bar*epsilon + kappa1*W + kappa2*(W'W)*W)
%
% Hoi tu co dinh thoi gian nho:
%   - kappa2*(W'W)*W: sigma-modification bac 3 => W hoi tu fixed-time
%   - alpha.*z^{p/q}: fractional power => z hoi tu nhanh gan goc toa do
%   - beta.*z^3: cubic term => z hoi tu nhanh xa goc toa do
%
% Input:
%   q      = [x; y; theta]     (3x1) trang thai robot
%   qr     = [xr; yr; thetar]  (3x1) trang thai tham chieu
%   vr     -- van toc dai tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%   W      -- (lx1) trong so Critic
%   s      -- struct tham so tu sm1_params()
%
% Output:
%   u    = [v; omega]  (2x1) lenh dieu khien tong
%   W    -- (lx1) trong so da cap nhat
%   info -- struct debug
%
% Tham khao: Wang et al. (2025), eq.(3)-(5), eq.(16)-(18)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

%% Sai so tracking (body frame, eq.3)
ex = qr(1) - q(1);
ey = qr(2) - q(2);
theta = q(3);

zx =  cos(theta)*ex + sin(theta)*ey;
zy = -sin(theta)*ex + cos(theta)*ey;
zth = atan2(sin(qr(3) - theta), cos(qr(3) - theta));
z = [zx; zy; zth];

%% Basis function phi(z) va Jacobian nabla_phi (6x3)
phi = [zx^2; zy^2; zth^2; zx*zy; zy*zth; zx*zth];

nabla_phi = [2*zx, 0,    0;
             0,    2*zy, 0;
             0,    0,    2*zth;
             zy,   zx,   0;
             0,    zth,  zy;
             zth,  0,    zx];  % 6x3

%% g(z) matrix tu error dynamics (eq.5c)
% z_dot = f(z) + g(z)*uo
g_z = [-1, zy;
        0, -zx;
        0, -1];  % 3x2

%% Pseudo-inverse g_pinv = (g'g)^{-1}g'  (2x3)
% det(g'g) = zx^2 + 1 > 0 luon, nghich dao luon ton tai
g_pinv = (g_z' * g_z) \ g_z';

%% ADP feedback (eq.16): uo_adp = -0.5 * R_inv * g' * nabla_phi' * W
uo_adp = -0.5 * s.R_inv * g_z' * nabla_phi' * W;

%% Fixed-time robust terms (eq.18)
% sig(z, p/q) = |z|^{p/q} * sign(z), voi p/q < 1
pq = s.ft_p / s.ft_q;
z_pq = abs(z).^pq .* sign(z);

robust = s.ft_lambda .* tanh(z / s.ft_rho) ...
       + s.ft_mu .* z ...
       + s.ft_alpha .* z_pq ...
       + s.ft_beta .* (z.^3);

%% Total feedback (eq.18): uo = uo_adp - g_pinv * robust
uo = uo_adp - g_pinv * robust;

% Gioi han feedback (chong phat tan khi W chua hoi tu hoac nhieu lon)
if isfield(s, 'ft_uo_max')
    uo(1) = max(-s.ft_uo_max, min(s.ft_uo_max, uo(1)));
    uo(2) = max(-s.ft_uo_max, min(s.ft_uo_max, uo(2)));
end

%% Feedforward + total control
uf = [vr * cos(zth); omegar];
u = uf + uo;

% Bao hoa
u(1) = max(-s.v_max, min(s.v_max, u(1)));
u(2) = max(-s.w_max, min(s.w_max, u(2)));

%% Error dynamics drift f(z) — can cho Bellman error
f_z = [zy*omegar; -zx*omegar + vr*sin(zth); 0];  % 3x1

%% Cap nhat trong so (eq.17) — Bellman error gradient + fixed-time sigma-mod
sigma_w = nabla_phi * (f_z + g_z * uo_adp);                      % 6x1
epsilon = W' * sigma_w + z' * s.Q * z + uo_adp' * s.R * uo_adp;  % scalar (HJB residual)
sigma_bar = sigma_w / (1 + sigma_w' * sigma_w)^2;                 % 6x1 normalized

W_dot = -0.5 * s.ft_Gamma * (sigma_bar * epsilon ...
        + s.ft_kappa1 * W ...
        + s.ft_kappa2 * (W' * W) * W);
W = W + s.dt * W_dot;

%% Debug info
info.z       = z;
info.uo      = uo;
info.uo_adp  = uo_adp;
info.robust  = robust;
info.phi     = phi;

end
