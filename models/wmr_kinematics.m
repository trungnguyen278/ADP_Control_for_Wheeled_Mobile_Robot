function dq = wmr_kinematics(~, q, v, omega)
% WMR_KINEMATICS  Phuong trinh kinematic WMR kieu vi sai
%
%   dx/dt    = v * cos(theta)
%   dy/dt    = v * sin(theta)
%   dtheta/dt = omega
%
% Input:
%   t     - thoi gian (khong dung)
%   q     - [x; y; theta] (3x1)
%   v     - van toc dai [m/s]
%   omega - van toc goc [rad/s]
%
% Output:
%   dq    - [dx; dy; dtheta] (3x1)

theta = q(3);

dq = zeros(3, 1);
dq(1) = v * cos(theta);
dq(2) = v * sin(theta);
dq(3) = omega;

end
