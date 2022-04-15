function [A_k, B_k] = UpdateAB_vw(v_k, w_k, theta_k, T)
% 更新AB矩阵
%   此处显示详细说明
A_k = [1, 0, -v_k * sin(theta_k) * T;
      0, 1, v_k * cos(theta_k) * T;
      0, 0,             1          ];
B_k = [cos(theta_k) * T, -v_k * sin(theta_k) * T * T;
       sin(theta_k) * T, v_k * cos(theta_k) * T * T;
         0             ,    T                   ];
end


