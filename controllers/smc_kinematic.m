function [eta_d, z] = smc_kinematic(q, qr, vr, omegar, s)
% SMC_KINEMATIC  Bo dieu khien truot thong thuong cho vong ngoai kinematic
%
% [eta_d, z] = smc_kinematic(q, qr, vr, omegar, s)
%
% Luat dieu khien:
%   Dinh nghia mat truot:
%     sigma_1 = z_x
%     sigma_2 = z_theta + c_zy * z_y    (coupling zy vao mat truot)
%
%   Equivalent control + reaching law:
%     uo2 = lambda2*sigma_2 + eta2*tanh(sigma_2/delta)
%     uo1 = z_y*(omegar + uo2) + lambda1*z_x + eta1*tanh(z_x/delta)
%
%   Van toc mong muon:
%     v_d   = vr*cos(z_theta) + uo1
%     w_d   = omegar + uo2
%
% Giai thich:
%   - sigma_2 = zth + c_zy*zy: khi sigma_2=0, zth = -c_zy*zy
%     => zy_dot ~ vr*sin(-c_zy*zy) ~ -vr*c_zy*zy => zy hoi tu exponential
%   - lambda: equivalent control (phan tuyen tinh, on dinh exponential)
%   - eta*tanh: reaching term (chong nhieu, dam bao den mat truot)
%   - tanh thay sign de giam chattering
%
% Input:
%   q      = [x; y; theta]     (3x1)
%   qr     = [xr; yr; thetar]  (3x1)
%   vr     -- van toc tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%   s      -- ctrl_params struct (chua smc_lambda1, smc_eta1, smc_delta, ...)
%
% Output:
%   eta_d = [v_d; w_d]  (2x1) van toc mong muon
%   z     = [zx; zy; ztheta]  (3x1) sai so tracking
%
% Tham khao: sliding mode control for mobile robots (conventional)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    04/2026

%% Tinh sai so tracking (body frame, giong backstepping)
ex = qr(1) - q(1);
ey = qr(2) - q(2);
theta = q(3);

zx =  cos(theta)*ex + sin(theta)*ey;
zy = -sin(theta)*ex + cos(theta)*ey;
zth = atan2(sin(qr(3) - theta), cos(qr(3) - theta));

z = [zx; zy; zth];

%% Luat dieu khien SMC

% Mat truot sigma_2 = zth + c_zy * zy (coupling zy)
sigma2 = zth + s.smc_c_zy * zy;

% Feedback goc
uo2 = s.smc_lambda2 * sigma2 + s.smc_eta2 * tanh(sigma2 / s.smc_delta);

% Feedback doc (sigma_1 = z_x)
uo1 = zy*(omegar + uo2) + s.smc_lambda1 * zx + s.smc_eta1 * tanh(zx / s.smc_delta);

%% Van toc mong muon = feedforward + feedback
v_d = vr * cos(zth) + uo1;
w_d = omegar + uo2;

% Clamp
v_d = max(-s.v_max, min(s.v_max, v_d));
w_d = max(-s.w_max, min(s.w_max, w_d));

eta_d = [v_d; w_d];

end
