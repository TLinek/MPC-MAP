function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars, public_vars)
%PREDICT_POSE Summary of this function goes here

x_old = old_pose(1);
y_old = old_pose(2);
theta_old = old_pose(3);

v_r = motion_vector(1);
v_l = motion_vector(2);


v = (v_r + v_l) / 2;
omega = (v_r - v_l) / read_only_vars.agent_drive.interwheel_dist;


if omega == 0
    theta_new = theta_old;
else
    theta_new = theta_old + omega * read_only_vars.sampling_period;
end

if v == 0 
    x_new = x_old;
    y_new = y_old;
elseif v ~=0 && omega == 0
    x_new = x_old + v * read_only_vars.sampling_period * cos(theta_old);  % cos(theta) = x/delta_t (přilehlá/přepona)
    y_new = y_old + v * read_only_vars.sampling_period * sin(theta_old);
elseif v ~=0 && omega ~= 0
    x_new = x_old - (v / omega) * sin(theta_old) + (v / omega) * sin(theta_old + omega * read_only_vars.sampling_period);
    y_new = y_old + (v / omega) * cos(theta_old) - (v / omega) * cos(theta_old + omega * read_only_vars.sampling_period);
end



if public_vars.state == "INITIALIZING"
    sigma_pose_noise = 0.5;%0.05
    sigma_theta_noise = 0.1;%0.5
else
    sigma_pose_noise = 0.05;%0.05
    sigma_theta_noise = 0.05;%0.05
end


x_new = x_new + randn() * sigma_pose_noise;
y_new = y_new + randn() * sigma_pose_noise;
theta_new = theta_new + randn() * sigma_theta_noise;






x_map_limit = read_only_vars.map.limits([1, 3]);
y_map_limit = read_only_vars.map.limits([2, 4]);
x_map_length = x_map_limit(2) - x_map_limit(1);
x_map_offset = x_map_limit(1);
y_map_length = y_map_limit(2) - y_map_limit(1);
y_map_offset = y_map_limit(1);


% Kontrola, zda není mimo mapu
if x_new < x_map_offset || x_new > (x_map_offset + x_map_length) || y_new < y_map_offset || y_new > (y_map_offset + y_map_length)
    % Pokud je mimo, náhodně zpět do mapy
    x_new = rand() * x_map_length + x_map_offset;
    y_new = rand() * y_map_length + y_map_offset;
    theta_new = rand() * 2*pi;
end
   


new_pose = [x_new, y_new, theta_new];

end

