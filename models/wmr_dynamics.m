function deta = wmr_dynamics(~, eta, tau, p)
% WMR_DYNAMICS  Phuong trinh dong luc hoc WMR kieu vi sai
%
% Phuong trinh (rut gon, body frame, d=0):
%   M * deta/dt = B * tau - F(eta)
%   => deta/dt  = M_inv * (B * tau - F(eta))
%
% voi:
%   M = diag(m, I)
%   B = [r/2, r/2; r/(2L), -r/(2L)]
%   F = [fv*v + fc*tanh(v/eps); fw*w + fcw*tanh(w/eps)]
%
% Tham khao: Fierro & Lewis (1997), eq. rut gon voi d=0
%
% Input:
%   t    - thoi gian (khong dung)
%   eta  - [v; omega] van toc than xe (2x1)
%   tau  - [tauR; tauL] mo-men 2 banh (2x1)
%   p    - struct tham so tu wmr_params()
%
% Output:
%   deta - [dv/dt; domega/dt] (2x1)

% Tach trang thai
v = eta(1);
w = eta(2);

% Bao hoa mo-men (gioi han vat ly motor)
tau = max(-p.tau_max, min(p.tau_max, tau));

% Luc/mo-men tu motor: tau_eq = B * [tauR; tauL]
tau_eq = p.B * tau;

% Vector ma sat F(eta)
%   Nhot (viscous): ti le voi van toc
%   Coulomb (dry):  hang so, tanh xap xi sign
F = [p.fv * v + p.fc * tanh(v / p.eps_sign);
     p.fw * w + p.fcw * tanh(w / p.eps_sign)];

% Phuong trinh chinh: deta = M_inv * (tau_eq - F)
% Ghi chu: V_m = 0 vi d = 0 (trong tam trung truc banh)
deta = p.M_inv * (tau_eq - F);

end
