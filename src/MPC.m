function U_k=MPC(A,B,N,x_k,Q,R,F)
%%%%%%%%%%%%%%%%%%%%%%%%
n = size(A,1); %% A矩阵是n * n矩阵，得到A矩阵的维数
p = size(B,2); %% B矩阵是n * p矩阵，得到B矩阵的维数
M = [eye(n);zeros(N*n,n)]; %% 初始化M矩阵,第一个分块矩阵置单位阵，其余矩阵置零
C = zeros((N+1)*n,N*p); %% 初始化C矩阵，置零
%接下来计算完整的M矩阵与C矩阵
tmp = eye(n); %定义一个n阶单位阵
for i = 1:N
    rows = i*n + (1:n);%行数，因为是分块矩阵所以从1至n;
    C(rows, :) = [tmp*B, C(rows-n, 1:end-p)];%用遍历的方法将C矩阵填满;
    tmp = A*tmp;%每次都左乘一次A矩阵;
    M(rows,:) = tmp;%写满M矩阵;
end
%定义Q_bar和R_bar
S_q = size(Q,1);%得到Q矩阵维度
S_r = size(R,1);%得到R矩阵维度
Q_bar = zeros((N+1)*S_q,(N+1)*S_q);%定义Q_bar矩阵维度
R_bar = zeros(N*S_r,N*S_r);%定义R_bar矩阵维度
for i = 0:N-1
    Q_bar(i*S_q+1:(i+1)*S_q,i*S_q+1:(i+1)*S_q) = Q;%把对角线上写满Q
end
Q_bar(N*S_q+1:(N+1)*S_q, N*S_q+1:(N+1)*S_q) = F;%最后一块写上F
for i = 0:N-1
        R_bar(i*S_r+1:(i+1)*S_r, i*S_r+1:(i+1)*S_r) = R;%对角线上写满R
end
G = M'*Q_bar*M;%定义M矩阵，事实上在代价函数中，这和输入无关，并没有被用到
E = M'*Q_bar*C;%定义E矩阵
H = C'*Q_bar*C + R_bar;%定义H矩阵
%最优化，得到最优输入值
f = x_k'*E;%由于quadprog函数的定义，需要把其写成矩阵相乘形式

%基于实际情况，给输入加约束
D = eye(3);b = [10;10;10];Aep=[];Bep=[];c=[1;1;1];d=[-1;-1;-1];
U_k = quadprog(H,f,D,b,Aep,Bep,d,c);%求解最优的U_k值