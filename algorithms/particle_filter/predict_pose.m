function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

x_old = old_pose(1);
y_old = old_pose(2);
theta_old = old_pose(3);

v_r = motion_vector(1);
v_l = motion_vector(2);


v = (v_r + v_l) / 2;
omega = (v_r - v_l) / read_only_vars.agent_drive.interwheel_dist;

% v = v + randn() * (v * 0.1);                %šum podle velikosti rychlosti
% omega = omega + randn() * (omega * 0.1);



if omega ~= 0
    x_new = x_old - (v / omega) * sin(theta_old) + (v / omega) * sin(theta_old + omega * read_only_vars.sampling_period);
    y_new = y_old + (v / omega) * cos(theta_old) - (v / omega) * cos(theta_old + omega * read_only_vars.sampling_period);
    theta_new = theta_old + omega * read_only_vars.sampling_period;
else
    x_new = v * read_only_vars.sampling_period * cos(theta_old);  % cos(theta) = x/delta_t (přilehlá/přepona)
    y_new = v * read_only_vars.sampling_period * sin(theta_old);
    theta_new = theta_old;
end

sigma_pose_noise = 0.05;
sigma_theta_noise = 0.5;

x_new = x_new + randn() * sigma_pose_noise;
y_new = y_new + randn() * sigma_pose_noise;
theta_new = theta_new + randn() * sigma_theta_noise;

new_pose = [x_new, y_new, theta_new];

end

