function [estimated_pose] = estimate_pose(public_vars, read_only_vars)
%ESTIMATE_POSE Summary of this function goes here

if any(isnan(read_only_vars.gnss_position))
    estimated_pose(1) = median(public_vars.particles(:,1)); 
    estimated_pose(2) = median(public_vars.particles(:,2));
    avg_sin = mean(sin(public_vars.particles(:,3)));
    avg_cos = mean(cos(public_vars.particles(:,3)));
    estimated_pose(3) = atan2(avg_sin, avg_cos);
else
    estimated_pose = public_vars.mu;
end

end
