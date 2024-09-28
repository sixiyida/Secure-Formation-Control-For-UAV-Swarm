clear;
close all;

alpha_x = -3; 
alpha_v = -3;
global alpha; alpha = [alpha_x, alpha_v]; %无人机参数

global N; N  = 4; %无人机个数
global n; n  = 2; %维度

m = 3; %连接保持个数
b = 1; %中断图个数

B1 = [1, 0]';
B2 = [0, 1]';

G1 = [0 0 0 0;
      1 0 1 0;
      1 1 0 1;
      1 0 1 0;];

G2 = [0 0 0 0;
      1 0 0 0;
      0 0 0 1;
      1 0 1 0;];

G3 = [0 0 0 0;
      1 0 1 0;
      0 1 0 1;
      0 0 1 0;];


G4 = [0 0 0 0;
      0 0 1 0;
      1 1 0 1;
      0 0 1 0;];

% G4 = [0 0 0 0;
%       0 0 0 0;
%       0 0 0 0;
%       0 0 0 0;];

L1 = - G1 + diag(sum(G1,2));
L2 = - G2 + diag(sum(G2,2));
L3 = - G3 + diag(sum(G3,2));
L4 = - G4 + diag(sum(G4,2));

L(:, :, 1) = L1;
L(:, :, 2) = L2;
L(:, :, 3) = L3;
L(:, :, 4) = L4;

global L_ff; L_ff = L(2:N, 2:N, :);
global L_lf; L_lf = L(2:N, 1, :);
%% 求解Y_sigma
eig_L_ff_min = [];

for i = 1:m
    eig_L_ff_min = [eig_L_ff_min; min(real(eig(L_ff(:, :, i))))];
end

gama_sigma = 4.5/5 * eig_L_ff_min; 

eig_Y = [];

for i = 1:m
    setlmis([]);
    Y = lmivar(1, [N-1, 1]);
    lmiterm([-1 1 1 Y], 1, L_ff(:, :, i), 's');
    lmiterm([1 1 1 Y], 2 * gama_sigma(i), 1); 
    lmiterm([-2 1 1 Y], 1, 1);
    lmis = getlmis;
    [tmin, xfeas] = feasp(lmis);
    Y_list(:, :, i) = dec2mat(lmis, xfeas, Y);
    eig_Y = [eig_Y eig(Y_list(:, :, i))];
end
%% 求解u
eig_YiYj = [];

for i = 1:m
    for j = 1:m
        eig_YiYj = [eig_YiYj; eig(inv(Y_list(:, :, i)) * Y_list(:, :, j))];
    end
end

miu = max(eig_YiYj);

%% 求解P
T = B1 * B2' + B2 * alpha;
epsilon = miu / exp(1.1); % epi>u/e^t0
a = 1.5;

setlmis([]);
P = lmivar(1, [2, 1]);
lmiterm([1 1 1 P], 1, T', 's');
lmiterm([1 1 1 P], epsilon, 1);
lmiterm([-1 1 1 0], B2 * B2');
lmiterm([-2 1 1 P], 1, 1);
lmiterm([3 1 1 P], 1, T', 's');
lmiterm([3 1 1 P], a, 1);



lmis = getlmis;
[tmin, xfeas] = feasp(lmis);
P = dec2mat(lmis, xfeas, P);
eig_P = eig(P);

%% 求解delta
gama_bar = min(gama_sigma);
% 取delta = 2倍的1/(2gama_bar)
delta = 1.1 * 1 / (2 * gama_bar);

%% 求解K
K = -delta * B2' * inv(P);

%% 求解tao下界
tao_min = log(miu / epsilon);

%% 求解a
% T = B1 * B2' + B2 * alpha;
% ksai_tilde_B = P * T' + T * P;
% ksai_tilde_L = -delta * 2 * kron(eye(N-1), B2 * B2');
% A = kron(eye(N-1), ksai_tilde_B) + ksai_tilde_L;
% eig(A)
% 
% a = 0;
% eig(P * T' + T * P)
% 
% while (max(eig(P * T' + T * P - a * P))) > 0
%     a = a + 0.01;
%     eig(P * T' + T * P - a * P)
% end
% a = a - 0.001;
% 
% Q = -delta * kron ((L_ff(:, :, 1)' * Y_list(:, :, 1) + Y_list(:, :, 1) * L_ff(:, :, 1)'), B2 * B2');
% eig(kron(Y_list(:, :, 1),(P * T' + T * P)) +Q)


