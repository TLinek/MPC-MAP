function [target, public_vars] = get_target(public_vars, P_point)
%GET_TARGET Summary of this function goes here

    actual_path_index = public_vars.path_index;
    path = public_vars.path;

    % robor už miří na konec cesty
    if actual_path_index >= length(path)
        public_vars.path_index = actual_path_index;
        target = path(end, :);
        return;
    end

    % hledání nejbližšího bodu maximálně v 10 následujících bodech
    remaining_points = path(actual_path_index:end, :);
    end_search = min(actual_path_index+10, length(remaining_points));
    selected_points = remaining_points(1:end_search,:);


    distances = vecnorm(selected_points - P_point, 2, 2); %2 eukli. vz., 2 podle řádků v matici
    [min_distance, index] = min(distances);

    if min_distance < 0.15
        if (actual_path_index + index - 1) < length(path)
            actual_path_index = actual_path_index + 1;
        end
    end

    public_vars.path_index = actual_path_index;
    target = path(actual_path_index, :);
end

