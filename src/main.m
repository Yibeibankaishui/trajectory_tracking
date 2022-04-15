clc;
clear all;
close all;

PLOT_DATA = 1;

[Latt, Long, Heading] = ReadData();

Xr = [Latt, Long, Heading];


% Xr = [zeros(size(Latt)),zeros(size(Long)),zeros(size(Heading))];

% 绘图
% if PLOT_DATA
%     Time = [1:1:size(Longitude)];
%     % 轨迹图
%     figure(1);
%     plot(Latt,Long);
%     % 朝向角变化
% %     figure(2);
% %     plot(Time,Heading);
% 
% end

% 定义模型参数
% 轴距
L = 0.2;
% 车轮半径
r_w = 1;

% 仿真时间
steps = 880;
% 仿真步长
T = 1;
% 预测区间大小
N = 4;

% 参数
Q = 2000*[0.1, 0, 0; 
    0, 0.1, 0;
    0, 0, 0.1];          %Q矩阵，对误差积累的重视程度
R = 1*[0.2, 0;
    0, 0.2];                     %R系数，表示对节省输入的重视程度
Q_ = [Q,zeros(size(Q,1),size(R,2));
      zeros(size(R,1),size(Q,2)),R];
R_ = [0.1, 0;
      0,  0.1];
F = 2000*[0.1, 0, 0;
    0, 0.1, 0;
    0, 0, 0.1];          %F矩阵，对终端误差的重视程度
F_ = [F,zeros(3,2);zeros(2,5)];
% 设置二次规划的约束
D = [];
b = [];
Aeq = [];
Beq = [];
% 上下限约束
lb = -50 * ones(N*2,1);
ub = 50 * ones(N*2,1);

% 初始状态
x_0 = Xr(1,1);
% x_0 = 10;
y_0 = Xr(1,2);
% y_0 = 10;
theta_0 = Xr(1,3);
X_0 = [x_0; y_0; theta_0];
% 初始控制量
v_0 = 0;
w_0 = 0;

% 需要记录各个时刻的状态
X_list = zeros(3, steps);
U_list = zeros(2, steps);

% Simulation
X_k = X_0;
v_k = v_0;
w_k = w_0;
for k = 1:steps
    % 当前时刻参考值
    Xr_k = (Xr(k,:))';
    xr_k = Xr_k(1);
    yr_k = Xr_k(2);
    thetar_k = Xr_k(3);
    
    X_list(:,k) = X_k;
    x_k = X_k(1);
    y_k = X_k(2);
    theta_k = X_k(3);

    % 计算每一时刻控制量
    % 首先计算线性化的控制矩阵
%     [A_, B_] = UpdateAB(vR_k, vL_k, thetar_k, L, T);
    [A_k, B_k] = UpdateAB_vw(v_k, w_k, theta_k, T);
    % 计算控制量
    % 需要对矩阵和状态量进行增广
    X_kaug = [X_k - Xr_k; v_k; w_k];
    [A_kaug,B_kaug,D_kaug]=increase_matrixDU(A_k, B_k ,D);
    delta_u = cal_MPC(A_kaug, B_kaug, N, X_kaug, Q_, R_, F_, D, b, Aeq, Beq, lb, ub);
    
    delta_v = delta_u(1);
    delta_w = delta_u(2);
    
    % 利用控制量和前一时刻状态进行状态更新
%     X_k_1 = UpdateModel(X_k, vL_k, vR_k, L,T, r_w);
    X_k_1 = UpdateModel_vw(X_k, v_k, w_k, delta_u, T);

    X_k = X_k_1;
        
    % 更新控制量并记录
    v_k = v_k + delta_v;
    w_k = w_k + delta_w;
    
    U_list(:,k) = [v_k;w_k];
    

end

% 画出运动轨迹
% 参考轨迹
figure(1)
plot(Latt,Long);
% 真实轨迹
figure(2)
plot(X_list(1,:), X_list(2,:));
% Lattitude真实轨迹
figure(3)
plot(X_list(1,:));
% Lattitude参考轨迹
figure(4)
plot(Xr(:,1));

% 输入量（控制量增量）
figure(5)
plot(U_list(1,:))
hold on;
plot(U_list(2,:))
