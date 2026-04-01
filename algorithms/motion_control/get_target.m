function [target, public_vars] = get_target(public_vars, P_point)
%GET_TARGET Summary of this function goes here

    actual_path_index = public_vars.path_index+1;
    remaining_path_points = public_vars.path(actual_path_index:end, :);
    distances = vecnorm(remaining_path_points - P_point, 2, 2); %2 eukli. vz., 2 podle řádků v matici
    [min_distance, index] = min(distances);
    target_index = actual_path_index + index;

    if target_index >= length(public_vars.path)
        target_index = actual_path_index;
    end

    public_vars.path_index = target_index;
    target = public_vars.path(target_index, :);

end

