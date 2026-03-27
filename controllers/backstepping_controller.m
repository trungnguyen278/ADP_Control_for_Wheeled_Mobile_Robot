function [u, z] = backstepping_controller(q, qr, vr, omegar, s)
% BACKSTEPPING_CONTROLLER  Bo dieu khien backstepping kinematic cho WMR
%
% [u, z] = backstepping_controller(q, qr, vr, omegar, s)
%
% Input:
%   q      = [x; y; theta]     (3x1) trang thai robot hien tai
%   qr     = [xr; yr; thetar]  (3x1) trang thai tham chieu
%   vr     -- van toc dai tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%   s      -- struct tham so tu sm1_params()
%
% Output:
%   u = [v; omega]  (2x1) lenh dieu khien
%   z = [zx; zy; ztheta]  (3x1) sai so tracking (body frame)
%
% Phuong trinh (Kanayama 1990 / Wang eq.31):
%   v   = vr*cos(ztheta) + k1*zx
%   w   = omegar + k2*vr*zy + k3*sin(ztheta)
%
% Sai so tracking (body frame, Wang eq.3):
%   zx =  cos(theta)*(xr-x) + sin(theta)*(yr-y)
%   zy = -sin(theta)*(xr-x) + cos(theta)*(yr-y)
%   ztheta = thetar - theta   (wrapped to [-pi, pi])
%
% Tham khao:
%   - Kanayama et al. (1990): A stable tracking control method
%   - Wang et al. (2025): eq.(3), eq.(31)
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

%% Tinh sai so tracking trong body frame
ex = qr(1) - q(1);
ey = qr(2) - q(2);
theta = q(3);

zx =  cos(theta)*ex + sin(theta)*ey;
zy = -sin(theta)*ex + cos(theta)*ey;
ztheta = atan2(sin(qr(3) - theta), cos(qr(3) - theta));  % wrap [-pi,pi]

z = [zx; zy; ztheta];

%% Tinh dieu khien backstepping
v = vr * cos(ztheta) + s.k1 * zx;
omega = omegar + s.k2 * vr * zy + s.k3 * sin(ztheta);

%% Clamp output
v     = max(-s.v_max, min(s.v_max, v));
omega = max(-s.w_max, min(s.w_max, omega));

u = [v; omega];

end
