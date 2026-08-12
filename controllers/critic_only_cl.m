function [u, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s)
% CRITIC_ONLY_CL  Critic-only ADP voi Concurrent Learning (SM2)
%
% [u, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s)
%
% Cai tien so voi Actor-Critic (SM1):
%   - 1 mang duy nhat (Critic) thay vi 2 (Actor + Critic)
%   - Khong can PE signal
%   - Concurrent Learning su dung du lieu lich su de hoc nhanh hon
%
% Dieu khien (giong eq.16 Wang nhung khong co robust terms):
%   uo = -0.5 * R_inv * g' * nabla_phi' * W
%
% Cap nhat trong so:
%   delta = W'*sigma + z'*Q*z + uo'*R*uo        (Bellman error)
%   W_dot = -alpha*sigma_bar*delta               (online gradient)
%           -alpha_hist*sum(sigma_j_bar*delta_j)/N (Concurrent Learning)
%           -kappa*W                              (sigma-modification)
%
% Input:
%   t      -- thoi gian hien tai [s]
%   q      = [x; y; theta]     (3x1) trang thai robot
%   qr     = [xr; yr; thetar]  (3x1) trang thai tham chieu
%   vr     -- van toc dai tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%   cl     -- struct: cl.W (6x1), cl.sigma_stack, cl.cost_stack,
%              cl.stack_count, cl.last_record_time
%   s      -- struct tham so tu ctrl_params()
%
% Output:
%   u    = [v; omega]  (2x1) lenh dieu khien
%   cl   -- struct da cap nhat (W, stack)
%   info -- struct debug
%
% Tham khao:
%   - Vamvoudakis & Lewis (2010): Critic-only ADP framework
%   - Chowdhary & Johnson (2010): Concurrent Learning
%   - Wang et al. (2025): basis functions, error dynamics
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    05/2026

%% Sai so tracking (body frame, Wang eq.3)
ex = qr(1) - q(1);
ey = qr(2) - q(2);
theta = q(3);

zx =  cos(theta)*ex + sin(theta)*ey;
zy = -sin(theta)*ex + cos(theta)*ey;
zth = atan2(sin(qr(3) - theta), cos(qr(3) - theta));
z = [zx; zy; zth];

%% Basis function va Jacobian (giong actor_critic_adp.m)
phi = [zx^2; zy^2; zth^2; zx*zy; zy*zth; zx*zth];

nabla_phi = [2*zx, 0,    0;
             0,    2*zy, 0;
             0,    0,    2*zth;
             zy,   zx,   0;
             0,    zth,  zy;
             zth,  0,    zx];  % 6x3

%% Error dynamics matrices (Wang eq.4-5)
f_z = [zy*omegar; -zx*omegar + vr*sin(zth); 0];  % 3x1
g_z = [-1, zy; 0, -zx; 0, -1];                    % 3x2

%% Critic-only control: uo = -0.5 * R_inv * g' * nabla_phi' * W
uo = -0.5 * s.R_inv * g_z' * nabla_phi' * cl.W;

% Gioi han feedback (tranh phat tan khi W chua hoi tu)
uo(1) = max(-s.cl_uo_max, min(s.cl_uo_max, uo(1)));
uo(2) = max(-s.cl_uo_max, min(s.cl_uo_max, uo(2)));

%% Feedforward + total control
uf = [vr * cos(zth); omegar];
u = uf + uo;

u(1) = max(-s.v_max, min(s.v_max, u(1)));
u(2) = max(-s.w_max, min(s.w_max, u(2)));

%% Bellman error (online)
sigma = nabla_phi * (f_z + g_z * uo);                  % 6x1
delta = cl.W' * sigma + z' * s.Q * z + uo' * s.R * uo; % scalar
sigma_bar = sigma / (1 + sigma' * sigma)^2;             % 6x1 normalized

%% Ghi du lieu vao history stack (dinh ky)
if cl.stack_count < s.cl_stack_max && ...
   (cl.stack_count == 0 || t - cl.last_record_time >= s.cl_record_dt)
    cl.stack_count = cl.stack_count + 1;
    cl.sigma_stack(:, cl.stack_count) = sigma;
    cl.cost_stack(cl.stack_count) = z' * s.Q * z + uo' * s.R * uo;
    cl.last_record_time = t;
end

%% Concurrent Learning update (vectorized cho toc do)
W_dot_cl = zeros(s.l, 1);
if cl.stack_count > 0
    S = cl.sigma_stack(:, 1:cl.stack_count);      % 6 x N_stack
    C = cl.cost_stack(1:cl.stack_count);            % 1 x N_stack

    % Bellman error cho tung diem lich su (dung W hien tai)
    deltas_hist = cl.W' * S + C;                    % 1 x N_stack

    % Normalized sigma cho tung diem
    norms_hist = 1 ./ (1 + sum(S.^2, 1)).^2;       % 1 x N_stack
    S_bar = S .* norms_hist;                        % 6 x N_stack

    % Tong CL gradient
    W_dot_cl = -s.cl_alpha_hist * (S_bar * deltas_hist') / cl.stack_count;
end

%% Total weight update
W_dot = -s.cl_alpha * sigma_bar * delta ...   % online gradient
        + W_dot_cl ...                        % concurrent learning
        - s.cl_kappa * cl.W;                  % sigma-modification
cl.W = cl.W + s.dt * W_dot;

%% Debug info
info.z     = z;
info.uo    = uo;
info.delta = delta;
info.phi   = phi;
info.sigma = sigma;
info.stack_count = cl.stack_count;

end
