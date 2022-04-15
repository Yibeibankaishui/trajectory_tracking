function [X_k_1] = UpdateModel_vw(X_k, v_k, w_k, u, T)
% 更新模型状态,
% 输入 Xk，当前线速度，角速度
X_k_1 = [(v_k + u(1)) * T * cos(X_k(3));
            (v_k + u(1)) * T * sin(X_k(3));
            (w_k + u(2)) * T] + X_k;

end

