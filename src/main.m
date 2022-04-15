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
steps = 800;
% 仿真步长
T = 1;
% 预测区间大小
N = 10;

% 参数
Q = 10*[5, 0, 0; 
    0, 5, 0;
    0, 0, 1];          %Q矩阵，对误差积累的权重
R = 1*[5, 0;
    0, 500];                     %R系数，表示对输入的权重
Q_ = [Q,zeros(size(Q,1),size(R,2));
      zeros(size(R,1),size(Q,2)),R];
R_ = 1*[0, 0;
      0,  10];             % 对输入增量的权重
F = 0*[0.1, 0, 0;
    0, 0.1, 0;
    0, 0, 0.1];          %F矩阵，对终端误差的权重
F_ = [F,zeros(3,2);zeros(2,5)];
% 设置二次规划的约束
D = [];
b = [];
Aeq = [];
Beq = [];
% 上下限约束
% lb = -5 * ones(N*2,1);
lb = -(kron(ones(N,1),[2;0.5]));
% ub = 2 * ones(N*2,1);
ub = (kron(ones(N,1),[2;0.5]));

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
% 记录误差
e_list = zeros(3, steps);

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
    [A_k, B_k] = UpdateAB_vw(v_k, w_k, theta_k, T);
    % 计算控制量
    % 需要对矩阵和状态量进行增广
    X_kaug = [X_k(1) - Xr_k(1); X_k(2) - Xr_k(2); X_k(3) - Xr_k(3); v_k; w_k];
    [A_kaug,B_kaug,D_kaug]=increase_matrixDU(A_k, B_k, D);
    delta_u = cal_MPC(A_kaug, B_kaug, N, X_kaug, Q_, R_, F_, D, b, Aeq, Beq, lb, ub);
    
    delta_v = delta_u(1);
    delta_w = delta_u(2);
    
    % 利用控制量和前一时刻状态进行状态更新
    X_k_1 = UpdateModel_vw(X_k, v_k, w_k, delta_u, T);

    X_k = X_k_1;
        
    % 更新控制量并记录
    v_k = v_k + delta_v;
    w_k = w_k + delta_w;
    
    U_list(:,k) = [v_k;w_k];
    
    % 记录跟踪误差
    e_list(:,k) = [X_k(1) - Xr_k(1); X_k(2) - Xr_k(2); mod((X_k(3) - Xr_k(3)), pi)];

end

% 画出运动轨迹
% 参考轨迹
% 真实轨迹
figure(1)
plot(X_list(1,:), X_list(2,:),'r',"LineWidth",1.6);
hold on
plot(Latt,Long,'b',"LineWidth",1.6);
title("运动轨迹");
xlabel("lattitude");
ylabel("longitude");
% Lattitude真实轨迹
% Lattitude参考轨迹
figure(2)
plot(Xr(:,1),'b',"LineWidth",1.6);
hold on
plot(X_list(1,:),'r',"LineWidth",1.6);
title("Lattitude");
xlabel("时间")

figure(3)
plot(Xr(:,2),'b',"LineWidth",1.6);
hold on
plot(X_list(2,:),'r',"LineWidth",1.6);
title("Longitude");
xlabel("时间")

figure(4)
plot(Xr(:,3),'b',"LineWidth",1.6);
hold on
plot(X_list(3,:),'r',"LineWidth",1.6);
title("Heading");
xlabel("时间")

% 控制量
figure(5)
subplot(2,1,1);
% 线速度
plot(U_list(1,:));
title("线速度控制量变化");
subplot(2,1,2);
% 角速度
plot(U_list(2,:));
xlabel("时间");
title("角速度控制量变化");

% 误差
figure(6)
ex = e_list(1,:);
ey = e_list(2,:);
e_theta = e_list(3,:);
subplot(3,1,1)
% x误差
plot(ex);
title("Lattitude误差");
subplot(3,1,2)
% y误差
plot(ey);
title("Longitude误差");
subplot(3,1,3)
% 角度误差
plot(e_theta);
xlabel("时间");
title("角度误差");

disp(sum((ex.^2))/steps);
disp(sum((ey.^2))/steps);
disp(sum((e_theta.^2))/steps);
