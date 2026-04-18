function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target
epsilon = 0.1; %0.25
kappa = 1.0; %6

if public_vars.path_index >= length(public_vars.path)
    epsilon = 0.1;
end


% R_pose = read_only_vars.mocap_pose(1:2);
% R_theta = read_only_vars.mocap_pose(3);

R_pose = public_vars.estimated_pose(1:2);
R_theta = public_vars.estimated_pose(3);

P_pose =  R_pose + epsilon*[cos(R_theta) sin(R_theta)];
[G_pose, public_vars] = get_target(public_vars, P_pose);


% II. Compute motion vector

dP = kappa * (G_pose - P_pose);

%limit speed
computed_velocity = norm(dP);
max_velocity = read_only_vars.agent_drive.max_vel;
if computed_velocity > max_velocity
    dP = dP * (max_velocity / computed_velocity);
end



v = dP(1) * cos(R_theta) + dP(2) * sin(R_theta);
omega = (1/epsilon) * (-dP(1) * sin(R_theta) + dP(2) * cos(R_theta));

d = read_only_vars.agent_drive.interwheel_dist;
v_r = v + (omega*d)/2;
v_l = v - (omega*d)/2;
public_vars.motion_vector = [v_r, v_l];

end