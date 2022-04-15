function [X_k_1] = UpdateModel(X_k,vL,vR,L,T,r_w)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
X_k_1 = [ 0.5 * cos(X_k(3)) * (vL + vR)*T;
            0.5 * sin(X_k(3)) * (vL + vR)*T;
            mod((1/L) * (vR - vL), pi)*T] + X_k;
        
end

