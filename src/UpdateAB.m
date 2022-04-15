function [A_, B_] = UpdateAB(vR_k, vL_k, theta_k, L, T)
% 更新AB矩阵
%   此处显示详细说明
A_ = [1, 0, -0.5 * sin(theta_k) * (vR_k + vL_k) * T;
      0, 1, 0.5 * cos(theta_k) * (vR_k + vL_k)* T;
      0, 0,             1                       ];
B_ = [0.5 * cos(theta_k) * T, 0.5 * cos(theta_k) * T;
      0.5 * sin(theta_k) * T, 0.5 * sin(theta_k) * T;
      T/L                   , -T/L                    ];
end

