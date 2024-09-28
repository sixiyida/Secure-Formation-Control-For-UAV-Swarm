clear;
close all;
%% 初始化

alpha_x = -3; 
alpha_v = -3;

global alpha; alpha = [alpha_x, alpha_v]; %无人机参数

global N; N  = 4; %无人机个数
global n; n  = 2; %维度
global m; m  = 3; %连通图个数
global r; r  = 20; %无人机间斥力最大距离
global l; l  = 0.3; %无人机-障碍物间斥力最小距离
global L_o; L_o  = 2; %无人机-障碍物间斥力最大距离
global d; d  = 4; %障碍物个数

ksai_o = [4 0 0 0 0 0 4 0 2 0 3 0 -2 0 -3 0]'; %障碍物状态：[px,vx,py,vy]


% 图邻接矩阵/拉普拉斯矩阵
% 注：Eij联通则Gji为1
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


% G4 = [0 0 0 0;
%       0 0 1 0;
%       1 1 0 1;
%       0 0 1 0;];

G4 = [0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 0;];

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


K = [-0.24, -0.24]; %参数K
 %K = [-11.02, -15.9548];

%% 初始状态
ksai_L = [0.49; 0.01; -1.5; -0.15];
% ksai_L = [100; 0.01; 100; -0.15];

ksai_F = [4.32; -0.05; -5.83; 0.07;
          5.94; 0.01; 2.55; -0.06;
          -5; 0.01; -5;   -0.02;
          ];

% ksai_F = [104.32; -0.05; 95.83; 0.07;
%           105.94; 0.01; 102.55; -0.06;
%           95; 0.01; 95;   -0.02;
%           ];


ksai = [ksai_L; ksai_F];

%% 仿真
t0 = 0; %开始时间
te = 50; %停止时间

global st; st = 0.01;%仿真步长

ot = 0.01; %输出步长（无用）

dout = zeros((te - t0) / ot, 4 * N);
tout = zeros((te - t0) / ot, 1);
sigma_t = zeros((te - t0) / ot, 1);
i = 0;

% function dksai = solver(t,ksai,K,sigma)
% 四阶龙科库塔求解
for t = t0: st: te
    sigma = get_sigma(t); % σ(t)
    k1 = st * solver(t, ksai, K, sigma, ksai_o);
    k2 = st * solver(t + 0.5 * st, ksai + 0.5 * k1, K, sigma, ksai_o);
    k3 = st * solver(t + 0.5 * st, ksai + 0.5 * k2, K, sigma, ksai_o);
    k4 = st * solver(t + st, ksai + k3, K, sigma, ksai_o);
    ksai_mid = ksai + 1/6 * (k1 + 2*k2 + 2*k3 + k4);
    ksai = ksai_mid;
    i = i + 1;
    dout(i, :) = ksai';
    tout(i) = t;
    sigma_t(i) = sigma;
end

