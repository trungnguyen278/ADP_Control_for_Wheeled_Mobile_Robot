function [eta_d, z] = ntsmc_kinematic(q, qr, vr, omegar, s)
% NTSMC_KINEMATIC  Non-singular Fast Terminal SMC cho vong ngoai kinematic
%
% [eta_d, z] = ntsmc_kinematic(q, qr, vr, omegar, s)
%
% ============================================================================
% PHAM VI: day la ban THICH NGHI, khong phai chep nguyen Feng et al. (2002).
%
%   NTSMC goc cua Feng thiet ke cho he BAC HAI (dieu khien u xuat hien o x_ddot),
%   mat truot s = x + (1/beta)*|x_dot|^(p/q)*sign(x_dot) voi 1 < p/q < 2.
%
%   Sai so kinematic WMR la BAC NHAT theo (v, omega): dieu khien xuat hien ngay
%   o z_dot. Do do khong the ap dung truc tiep. Ban nay giu 2 y tuong cot loi
%   cua NTSMC -- hoi tu HUU HAN THOI GIAN va KHONG KY DI -- ap cho he bac nhat.
%
%   Khi viet luan van: trich dan Feng et al. (2002) va Yu & Man (2002) nhu
%   NGUON Y TUONG, khong duoc trinh bay nhu ap dung nguyen ban.
% ============================================================================
%
% Mat truot (fast terminal, co so hang lu thua phan so tren z_y):
%   sigma_1 = z_x
%   sigma_2 = z_th + c_zy*z_y + (1/beta)*sig_ns(z_y, alpha_s)
%
%   Khi sigma_2 = 0:
%     z_th = -[c_zy*z_y + (1/beta)*sig_ns(z_y, alpha_s)]
%     z_y_dot ~ vr*sin(z_th) ~ -vr*[c_zy*z_y + (1/beta)*|z_y|^alpha_s*sign(z_y)]
%
%   Dang x_dot = -a*x - b*|x|^alpha voi 0 < alpha < 1 => HOI TU HUU HAN THOI GIAN.
%   (SMC thuong chi co so hang tuyen tinh => chi hoi tu mu, khong huu han)
%
% Luat tien toi (fast terminal reaching law):
%   uo2 = lambda2*sigma_2 + eta2*sig_ns(sigma_2, alpha_r)
%   uo1 = z_y*(omegar + uo2) + lambda1*sigma_1 + eta1*sig_ns(sigma_1, alpha_r)
%
%   So hang |sigma|^alpha_r voi 0 < alpha_r < 1 lam toc do tien toi mat truot
%   tang manh khi sigma nho => den mat truot trong huu han thoi gian.
%
% CHONG KY DI (non-singular):
%   sig_ns(x, a) = |x|^a * sign(x)      neu |x| >= eps_ns
%                = x / eps_ns^(1-a)      neu |x| <  eps_ns   (noi tuyen tinh)
%
%   Ly do: dao ham cua |x|^a*sign(x) la a*|x|^(a-1), phan ky khi x->0 voi a<1.
%   Do chinh la diem ky di kinh dien cua Terminal SMC. Noi tuyen tinh duoi
%   nguong eps_ns lam dao ham bi chan boi 1/eps_ns^(1-a) o moi noi
%   => luat dieu khien bi chan, khong bao gio no ra vo cung.
%   Doan noi cung lam ham lien tuc tai |x| = eps_ns (hai nhanh bang nhau).
%
% So sanh voi smc_kinematic.m (SMC thuong):
%   SMC   : uo = lambda*sigma + eta*tanh(sigma/delta)     -> hoi tu MU
%   NTSMC : uo = lambda*sigma + eta*sig_ns(sigma,alpha)   -> hoi tu HUU HAN
%   Mat truot SMC khong co so hang lu thua phan so tren z_y.
%
% Input:
%   q      = [x; y; theta]     (3x1) trang thai robot
%   qr     = [xr; yr; thetar]  (3x1) trang thai tham chieu
%   vr     -- van toc dai tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%   s      -- ctrl_params struct (chua nt_lambda1, nt_eta1, nt_alpha_s, ...)
%
% Output:
%   eta_d = [v_d; w_d]        (2x1) van toc mong muon cho vong trong
%   z     = [zx; zy; ztheta]  (3x1) sai so tracking trong body frame
%
% Tham khao:
%   Y. Feng, X. Yu, Z. Man (2002), "Non-singular terminal sliding mode control
%     of rigid manipulators," Automatica, vol. 38, no. 12, pp. 2159-2167.
%   X. Yu, Z. Man (2002), "Fast terminal sliding-mode control design for
%     nonlinear dynamical systems," IEEE TCAS-I, vol. 49, no. 2, pp. 261-264.
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    08/2026

%% Tinh sai so tracking (body frame, giong backstepping va smc_kinematic)
ex = qr(1) - q(1);
ey = qr(2) - q(2);
theta = q(3);

zx =  cos(theta)*ex + sin(theta)*ey;
zy = -sin(theta)*ex + cos(theta)*ey;
zth = atan2(sin(qr(3) - theta), cos(qr(3) - theta));

z = [zx; zy; zth];

%% Mat truot
% sigma_2 co them so hang lu thua phan so => hoi tu huu han thoi gian cua z_y
sigma1 = zx;
sigma2 = zth + s.nt_c_zy * zy + (1/s.nt_beta) * sig_ns(zy, s.nt_alpha_s, s.nt_eps);

%% Luat tien toi fast terminal
uo2 = s.nt_lambda2 * sigma2 + s.nt_eta2 * sig_ns(sigma2, s.nt_alpha_r, s.nt_eps);
uo1 = zy*(omegar + uo2) ...
    + s.nt_lambda1 * sigma1 + s.nt_eta1 * sig_ns(sigma1, s.nt_alpha_r, s.nt_eps);

%% Van toc mong muon = feedforward + feedback
v_d = vr * cos(zth) + uo1;
w_d = omegar + uo2;

% Clamp (giong cac controller khac de so sanh cong bang)
v_d = max(-s.v_max, min(s.v_max, v_d));
w_d = max(-s.w_max, min(s.w_max, w_d));

eta_d = [v_d; w_d];

end


function y = sig_ns(x, a, eps_ns)
% SIG_NS  |x|^a * sign(x) voi noi tuyen tinh gan 0 de chong ky di.
%
%   |x| >= eps_ns : y = |x|^a * sign(x)
%   |x| <  eps_ns : y = x / eps_ns^(1-a)
%
% Tai |x| = eps_ns hai nhanh bang nhau (lien tuc):
%   eps_ns^a  ==  eps_ns / eps_ns^(1-a)  = eps_ns^a   OK
%
% Dao ham bi chan boi 1/eps_ns^(1-a) => luat dieu khien khong bao gio phan ky.

ax = abs(x);
if ax >= eps_ns
    y = ax^a * sign(x);
else
    y = x / eps_ns^(1 - a);
end

end
