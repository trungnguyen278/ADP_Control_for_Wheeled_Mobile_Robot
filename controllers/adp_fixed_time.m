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
%   s      -- struct tham so tu ctrl_params()
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

%% Ghep nonholonomic zy -> kenh goc (TUY CHON, bat bang s.ft_c_zy > 0)
%
% Van de: bon so hang robust o tren tac dong len TUNG thanh phan z doc lap.
% Khi sai so don het vao kenh ngang (zy lon, zx va zth ~ 0), ca uo_adp lan
% robust deu ~ 0 => he ket o diem can bang gia, robot chay song song quy dao
% ma khong ve. Da quan sat: ||z||=2.85 nhung uo=-0.085.
%
% Nguyen nhan goc la cascade nonholonomic: zy khong co kenh dieu khien truc
% tiep, chi hoi tu gian tiep qua vr*sin(zth) — co che nay doi hoi zth ~= 0.
%
% Cach xu ly: chu dong lai zth theo zy, giong k2*vr*zy cua Backstepping va
% mat truot sigma_2 = zth + c*zy cua SMC.
% Hang 2 cua g_pinv la [0, -zx/(zx^2+1), -1/(zx^2+1)], nen them c*vr*zy vao
% thanh phan thu 3 cua robust se cho uo2 += c*vr*zy/(zx^2+1) — dung dau.
% s.ft_czy_sat > 0 => dung dang bao hoa c*vr*tanh(zy/sat) thay vi tuyen tinh.
% Dang bao hoa tranh tao xung lon khi zy lon (vong trong khong bam kip).
if isfield(s, 'ft_c_zy') && s.ft_c_zy > 0
    if isfield(s, 'ft_czy_sat') && s.ft_czy_sat > 0
        robust(3) = robust(3) + s.ft_c_zy * vr * tanh(zy / s.ft_czy_sat);
    else
        robust(3) = robust(3) + s.ft_c_zy * vr * zy;
    end
end

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

%% Cap nhat trong so
% Hai luat, chon bang s.ft_update_law (mac dinh 'bellman'):
%
%   'wang'    -- NGUYEN BAN Wang et al. eq.(17):
%                W_dot = +0.5*Gamma*(nabla_phi*g*R_inv*g'*z - kappa1*W
%                                    - kappa2*(W'W)*W)
%                Khong dung sai so Bellman (bai bao co y tranh de bo dieu
%                kien PE -- xem Remark 2 cua bai bao).
%
%   'bellman' -- LUAN VAN: gradient descent tren sai so Bellman.
%                W_dot = -0.5*Gamma*(sigma_bar*epsilon + kappa1*W
%                                    + kappa2*(W'W)*W)
%                Giu lai hai so hang sigma-modification cua Wang.
%
% LUU Y: chung minh fixed-time cua Wang (Theorem 1) dua tren luat 'wang'.
% Khi dung 'bellman', chung minh do KHONG con phu duoc.

if isfield(s, 'ft_update_law')
    update_law = s.ft_update_law;
else
    update_law = 'bellman';   % mac dinh: giu nguyen hanh vi cu
end

sigma_w = nabla_phi * (f_z + g_z * uo_adp);                      % 6x1

switch update_law
    case 'wang'
        % eq.(17) nguyen ban
        W_dot = 0.5 * s.ft_Gamma * (nabla_phi * g_z * s.R_inv * g_z' * z ...
                - s.ft_kappa1 * W ...
                - s.ft_kappa2 * (W' * W) * W);
        epsilon = W' * sigma_w + z' * s.Q * z + uo_adp' * s.R * uo_adp;

    case 'bellman'
        epsilon = W' * sigma_w + z' * s.Q * z + uo_adp' * s.R * uo_adp;  % HJB residual
        sigma_bar = sigma_w / (1 + sigma_w' * sigma_w)^2;                % 6x1 normalized
        W_dot = -0.5 * s.ft_Gamma * (sigma_bar * epsilon ...
                + s.ft_kappa1 * W ...
                + s.ft_kappa2 * (W' * W) * W);

    otherwise
        error('adp_fixed_time: ft_update_law khong hop le (dung ''wang'' hoac ''bellman'')');
end

W = W + s.dt * W_dot;

%% Toan tu chieu (projection operator) — TUY CHON, mac dinh TAT
% Bat bang s.ft_proj = true.
%
% Van de: khong co rang buoc nao giu W trong vung ma V_hat = W'*phi xac dinh
% duong. Khi z0 lon, W co the troi sang vung W1 < 0 (da quan sat: W1 = -1.486
% tai ||z0||=2.375) => V_hat khong con la ham Lyapunov => chinh sach suy tu no
% mat kha nang on dinh hoa => mat bam.
%
% Cach xu ly: V_hat = W'*phi = z'*P*z voi
%   P = [ W1    W4/2  W6/2
%         W4/2  W2    W5/2
%         W6/2  W5/2  W3   ]
% Sau moi buoc cap nhat, chieu W ve tap {W : P(W) >= eps_p * I}.
%
% Tham khao: Ioannou & Sun (1996), Robust Adaptive Control, muc 4.4;
%            Kamalapurkar et al. (2018), chuong 4 (projection cho ADP).
if isfield(s, 'ft_proj') && s.ft_proj
    W = project_W(W, s);
end

%% Debug info
info.z       = z;
info.uo      = uo;
info.uo_adp  = uo_adp;
info.robust  = robust;
info.phi     = phi;

end

%% ====================================================================
function W = project_W(W, s)
% PROJECT_W  Chieu W ve tap dam bao V_hat = W'*phi(z) xac dinh duong,
%            dong thoi chan ||W|| khong vuot nguong.
%
% Hai rang buoc:
%   1. P(W) >= eps_p * I   (P la ma tran bac hai tuong ung V_hat)
%   2. ||W|| <= W_max
%
% Rang buoc 1 duoc kiem tra truoc bang tieu chuan Sylvester (3 dinh thuc con,
% rat re). Chi khi vi pham moi lam phan tich tri rieng — nen chi phi trung binh
% gan nhu bang khong o che do xac lap.

    if isfield(s, 'ft_proj_eps'),   eps_p = s.ft_proj_eps;   else, eps_p = 0.05; end
    if isfield(s, 'ft_proj_wmax'),  W_max = s.ft_proj_wmax;  else, W_max = 20;   end

    % Dung ma tran P tu W:  V_hat = z'*P*z
    P = [ W(1),     W(4)/2,   W(6)/2;
          W(4)/2,   W(2),     W(5)/2;
          W(6)/2,   W(5)/2,   W(3)   ];

    % Tieu chuan Sylvester: cac dinh thuc con chinh deu > eps_p
    d1 = P(1,1) - eps_p;
    d2 = det(P(1:2,1:2) - eps_p*eye(2));
    d3 = det(P - eps_p*eye(3));

    if ~(d1 > 0 && d2 > 0 && d3 > 0)
        % Vi pham: chieu bang cach kep tri rieng ve san eps_p
        [V, D] = eig((P + P')/2);        % doi xung hoa cho chac
        lam = max(diag(D), eps_p);
        P = V * diag(lam) * V';
        % Lay lai W tu P (nghich dao cua phep dung P o tren)
        W = [P(1,1); P(2,2); P(3,3); 2*P(1,2); 2*P(2,3); 2*P(1,3)];
    end

    % Chan bien do trong so
    nW = norm(W);
    if nW > W_max
        W = W * (W_max / nW);
    end
end
