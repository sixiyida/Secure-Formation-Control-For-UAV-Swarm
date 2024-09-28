%% 龙格库塔求解器


function dksai = solver(t,ksai,K,sigma,ksai_o)
%% 初始化参数
B1 = [1, 0]';
B2 = [0, 1]';

global N;
global n;
global alpha;
global L_ff;
global L_lf;
global st;
global r;
global d;
global l;
global L_o;

r_a = 1/2 * r;
eps = 1;
k = 0.5;

global dhlist;
global hlist;

ksai_L = ksai(1:4);
ksai_F = ksai(5:4 * N);

%% 期望队形
h_F = [get_h(1, t); get_h(2, t); get_h(3, t)];

if (t >= st)
    last_h_F = [get_h(1, t-st); get_h(2, t-st); get_h(3, t-st)];
else
    last_h_F = [get_h(1, 0); get_h(2, 0); get_h(3, 0)];
end

dh_F = (h_F - last_h_F) ./ st;

%% 内部避碰矩阵W
W = zeros(N, N);
for i = 1:N
    for j = 1:N
        xi = ksai(4 * i - 3);
        yi = ksai(4 * i - 1);
        xj = ksai(4 * j - 3);
        yj = ksai(4 * j - 1);
        dij = sqrt((xi - xj)^2 + (yi - yj)^2);
        if dij < r && i ~= j
            W(i, j) = -(k^((r_a - dij)/(r_a - r)) * log(k)) / ((r_a - r) * dij);
        end
    end
end
%% 内部避碰拉普拉斯矩阵WL？
WL = - W + diag(sum(W,2));
WL_ff = WL(2:N, 2:N);
WL_lf = WL(2:N, 1);

%% 位置状态from range sensor
ksai_x = ksai;
for i = 1:N
    ksai_x(4*i - 3) = 0;
    ksai_x(4*i - 1) = 0;
end

ksai_Lx = ksai_x(1:4);
ksai_Fx = ksai_x(5:4 * N);

%% 外部避碰控制输入
u_o = zeros(1, 4 * N)';
for i = 1:N
    u_ix = 0;
    u_iy = 0;
    for k = 1:d
        xi = ksai(4 * i - 3);
        yi = ksai(4 * i - 1);
        xk = ksai_o(4 * k - 3);
        yk = ksai_o(4 * k - 1);
        dik = sqrt((xi - xk)^2 + (yi - yk)^2);
        if (dik > l && dik <= L_o)
            u_ix = u_ix + 4 * (dik^2 - L_o^2) * (L_o^2 - l^2) / (dik^2 - l^2)^3 * (xi - xk);
            u_iy = u_iy + 4 * (dik^2 - L_o^2) * (L_o^2 - l^2) / (dik^2 - l^2)^3 * (yi - yk);
        end 
    end
    u_o(4*i - 2) = u_ix * K(1);
    u_o(4*i - 0) = u_iy * K(1);
end

u_of = u_o(5:4 * N); 


%% 微分方程

dksai_L = kron(eye(n), (B1 * B2' + B2 * alpha)) * ksai_L;

dksai_F = (kron(eye(N - 1), kron(eye(n), B1 * B2' + B2 * alpha)) + kron(L_ff(:, :, sigma), kron(eye(n), B2 * K))) * ksai_F...
    + kron(L_lf(:, :, sigma), kron(eye(n), B2 * K)) * ksai_L + kron(eye(N - 1), kron(eye(n), B2 * B2')) * dh_F -...
    (kron(L_ff(:, :, sigma), kron(eye(n), B2 * K)) + kron(eye(N - 1), kron(eye(n), B2 * alpha))) * h_F -...
    eps * kron(WL_ff, kron(eye(n), B2 * K)) * ksai_Fx - eps * kron(WL_lf, kron(eye(n), B2 * K)) * ksai_Lx + u_of;
    %eps * kron(WL_ff, kron(eye(n), B2*K_w)) * ksai_Fx + eps * kron(WL_lf, kron(eye(n), B2*K_w)) * ksai_Lx;
    

dksai = [dksai_L; dksai_F];

end