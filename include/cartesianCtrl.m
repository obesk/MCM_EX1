function [v_des,omega_des] = cartesianCtrl(r_ab,rho_ab,v_ff,omega_ff)
%CARTESIANERROR Summary of this function goes here
%   Detailed explanation goes here
    linear_gain = 0.8;
    angular_gain = 0.8;
    % e_dot + lambda*e = feed-forward
    % Compute the linear error to reach the goal
    v_des = linear_gain*(r_ab)+v_ff; % v_desired
    
    % Compute the angular error to reach the goal     
    omega_des = angular_gain*(rho_ab)+omega_ff; % omega_desired

end

