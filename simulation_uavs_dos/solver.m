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


%% 微分方程

dksai_L = kron(eye(n), (B1 * B2' + B2 * alpha)) * ksai_L;

dksai_F = (kron(eye(N - 1), kron(eye(n), B1 * B2' + B2 * alpha)) + kron(L_ff(:, :, sigma), kron(eye(n), B2 * K))) * ksai_F...
    + kron(L_lf(:, :, sigma), kron(eye(n), B2 * K)) * ksai_L + kron(eye(N - 1), kron(eye(n), B2 * B2')) * dh_F -...
    (kron(L_ff(:, :, sigma), kron(eye(n), B2 * K)) + kron(eye(N - 1), kron(eye(n), B2 * alpha))) * h_F;
  
    

dksai = [dksai_L; dksai_F];

end