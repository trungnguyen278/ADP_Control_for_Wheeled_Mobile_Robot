function dstate = wmr_full_model(t, state, tau, p)
% WMR_FULL_MODEL  Mo hinh WMR day du: kinematic + dynamic
%
% State vector: [x; y; theta; v; omega] (5x1)
%   - x, y, theta: vi tri va huong (kinematic)
%   - v, omega:    van toc than xe (dynamic)
%
% Input:
%   t     - thoi gian
%   state - [x; y; theta; v; omega] (5x1)
%   tau   - [tauR; tauL] mo-men 2 banh (2x1)
%   p     - struct tham so
%
% Output:
%   dstate - dao ham (5x1)

% Tach trang thai
q   = state(1:3);      % [x; y; theta]
eta = state(4:5);      % [v; omega]

% Phan kinematic: dq = f(q, v, omega)
dq = wmr_kinematics(t, q, eta(1), eta(2));

% Phan dynamic: deta = g(eta, tau, p)
deta = wmr_dynamics(t, eta, tau, p);

% Ghep
dstate = [dq; deta];

end
