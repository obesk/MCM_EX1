function [R] = orthonormalize(R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    [U,D,V] = svd(R);
    D_tmp = eye(3);
    D_tmp(3,3) = det(U*V');
    R = U*D_tmp*V';
end

