function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

public_vars.motion_vector = [0.0, 0.0];


%% WEEK 2 - TASK 5
if read_only_vars.counter < 21
    public_vars.motion_vector = [0.5, 0.395];
elseif read_only_vars.counter < 145
    public_vars.motion_vector = [0.5, 0.5];
elseif read_only_vars.counter < 200
    public_vars.motion_vector = [0.4, 0.5];
elseif read_only_vars.counter < 320
    public_vars.motion_vector = [0.5, 0.5];
elseif read_only_vars.counter < 370
    public_vars.motion_vector = [0.5, 0.4];
elseif read_only_vars.counter < 500
    public_vars.motion_vector = [0.5, 0.5];
end




end