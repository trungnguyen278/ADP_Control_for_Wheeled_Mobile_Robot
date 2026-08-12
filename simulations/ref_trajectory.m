function [qr, vr, omegar] = ref_trajectory(t, s)
% REF_TRAJECTORY  Sinh quy dao tham chieu cho WMR tracking
%
% [qr, vr, omegar] = ref_trajectory(t, s)
%
% Input:
%   t -- thoi gian hien tai [s] (scalar)
%   s -- struct tham so tu sm1_params()
%
% Output:
%   qr     = [xr; yr; thetar]  (3x1) trang thai tham chieu
%   vr     -- van toc dai tham chieu [m/s]
%   omegar -- van toc goc tham chieu [rad/s]
%
% Dam bao: dxr/dt = vr*cos(thetar), dyr/dt = vr*sin(thetar), dthetar/dt = omegar
%
% Ho tro: 'circle' va 'line'
%
% Tac gia: Nguyen Thanh Trung
% Ngay:    03/2026

switch s.traj_type
    case 'circle'
        % Quy dao tron: tam (0, 0), ban kinh R
        % Robot bat dau tai (R, 0) huong Bac (theta=pi/2)
        % Di nguoc chieu kim dong ho
        R  = s.circle_R;
        vr = s.circle_vr;
        omegar = vr / R;            % omega = v/R

        thetar = pi/2 + omegar * t; % bat dau tu pi/2, quay tang
        xr = R * cos(omegar * t);   % x = R*cos(omega*t)
        yr = R * sin(omegar * t);   % y = R*sin(omega*t)

        qr = [xr; yr; thetar];

    case 'line'
        % Di thang theo huong angle
        vr = s.line_vr;
        omegar = 0;
        angle = s.line_angle;

        thetar = angle;
        xr = vr * t * cos(angle);
        yr = vr * t * sin(angle);

        qr = [xr; yr; thetar];

    case 'figure8'
        % Hinh so 8 (lemniscate): x = a*sin(wt), y = b*sin(2wt)
        a = s.fig8_a;
        b = s.fig8_b;
        w = s.fig8_omega;

        xr  = a * sin(w * t);
        yr  = b * sin(2 * w * t);
        dxr = a * w * cos(w * t);
        dyr = 2 * b * w * cos(2 * w * t);

        vr = sqrt(dxr^2 + dyr^2);
        thetar = atan2(dyr, dxr);

        ddxr = -a * w^2 * sin(w * t);
        ddyr = -4 * b * w^2 * sin(2 * w * t);
        omegar = (dxr * ddyr - dyr * ddxr) / (dxr^2 + dyr^2 + 1e-12);

        qr = [xr; yr; thetar];

    otherwise
        error('Loai quy dao khong ho tro: %s', s.traj_type);
end

end
