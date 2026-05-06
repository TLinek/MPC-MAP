function [weights] = weight_particles(particle_measurements, lidar_distances, public_vars)
%WEIGHT_PARTICLES Summary of this function goes here

    N = size(particle_measurements, 1);
    weights = ones(N,1) / N;

    if public_vars.state == "INITIALIZING"
        sigma_L = 0.9; % Task 1 0.0498
    else
        sigma_L = 0.2;
    end

    particle_measurements(isinf(particle_measurements)) = 100;
    lidar_distances(isinf(lidar_distances)) = 100;
    particle_measurements(isnan(particle_measurements)) = 100;
    lidar_distances(isnan(lidar_distances)) = 100;
    
    for n = 1:N
        % Rozdíly mezi měřením robota a simulací částice
        diffs = particle_measurements(n,:) - lidar_distances;
        sum_sq_err = sum(diffs.^2);      
        weights(n) = exp(-sum_sq_err / (2 * sigma_L^2)); % Gaussova váha
    end

    % Normalizace
    if sum(weights) > 0
        weights = weights / sum(weights);
    else
        weights = ones(N,1) / N;
    end


end