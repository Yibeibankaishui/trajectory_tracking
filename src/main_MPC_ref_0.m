%h为一个更新周期，即积分步长，采样时间为n
%假定输入为零，输出即为状态值
clear
clc
format long 
%--------------------------初始参数---------------------------------%
h=0.1;                      %仿真步长              
n=300;                      %仿真时间   
NN=n/h;
A = [1,0.1;0,1];            %系统矩阵
B = [0;0.5];                %输入矩阵
Q = [2,0;0,2];              %Q矩阵，对误差积累的重视程度
R = 0.1;                    %R系数，表示对节省输入的重视程度
N = 3;                      %预测区间
F = [2,0;0,2];              %F矩阵，对终端误差的重视程度
x_0 = [100;100];            %初始位置
X1 = zeros(2,NN+1);          
t = zeros(1,NN+1);
U1 = zeros(1,NN+1);         %初始化
Eg1 = zeros(1,NN+1);
X1(:,1) = x_0;              %赋初值
%--------------------------仿真1开始---------------------------------%
for j = 1:NN
    U_all = MPC(A,B,N,X1(:,j),Q,R,F);
    X1(:,j+1) = A*X1(:,j) + B*U_all(1);       %这里只取预测估计的第一项
    U1(j) = U_all(1);
    t(j+1)=t(j)+h;
    Eg1(j+1) = Eg1(j)+U_all(1)^2;
end
%%为了比较不同参数的影响，选择另一组Q,R,F
Q = [0.1,0;0,0.1];          %Q矩阵，对误差积累的重视程度
R = 10;                     %R系数，表示对节省输入的重视程度
F = [0.1,0;0,0.1];          %F矩阵，对终端误差的重视程度
X2 = zeros(2,NN+1); 
U2 = zeros(1,NN+1);         %初始化
Eg2 = zeros(1,NN+1);
X2(:,1) = x_0;              %赋初值
%--------------------------仿真2开始---------------------------------%
for j = 1:NN
    U_all = MPC(A,B,N,X2(:,j),Q,R,F);
    X2(:,j+1) = A*X2(:,j) + B*U_all(1);     %这里只取预测估计的第一项
    U2(j) = U_all(1);
    t(j+1)=t(j)+h;
    Eg2(j+1) = Eg2(j)+U_all(1)^2;
end
figure(1)
subplot(2,1,1),plot(t,X1(1,:),'-','linewidth',3),title('状态向量'),ylabel('x_1');hold on;
plot(t,X2(1,:),'-','linewidth',2),title('状态向量'),ylabel('x_1');grid on;
legend('case1','case2');
subplot(2,1,2),plot(t,X1(2,:),'-','linewidth',3),xlabel('t/s'),ylabel('x_2');hold on
plot(t,X2(2,:),'-','linewidth',2),title('状态向量'),ylabel('x_2');grid on;
legend('case1','case2');
figure(2)
plot(t,U1,'linewidth',2),title('实际输入'),xlabel('t/s'),ylabel('u');hold on;
plot(t,U2,'linewidth',2),title('实际输入'),xlabel('t/s'),ylabel('u');grid on;
legend('case1','case2');
figure(3)
plot(t,Eg1,'linewidth',2),title('消耗能量'),xlabel('t/s'),ylabel('J');hold on;
plot(t,Eg2,'linewidth',2),title('消耗能量'),xlabel('t/s'),ylabel('J');grid on;
legend('case1','case2');