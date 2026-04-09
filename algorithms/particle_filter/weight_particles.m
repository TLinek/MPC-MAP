function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);
weights = ones(N,1) / N;

% lidar_sigma = 0.9; % lidar std z week 1 0.0497
% 
% for n = 1:N
%     summ = 0;
%     for i = 1:length(lidar_distances)
%         summ = summ + (lidar_distances(i) - particle_measurements(n,i))^2;
%     end
%     if ~isnan(summ)
%         weights(n) = exp( (-1 / ( 2* lidar_sigma^2 )) * summ);
%     else
%         weights(n) = 0.000001;
%     end
% end
% 
% weights = weights / sum(weights);


for n = 1:N
    summ = 0;
    for i = 1:length(lidar_distances)
        summ = summ + (particle_measurements(n,i) - lidar_distances(i))^2;
    end

    if summ == 0
        summ = 0.0000001;
    end

    if ~isnan(summ)
        weights(n) = 1 / sqrt(summ);
    else
        weights(n) = 0.00000001;
    end
end

weights = weights / sum(weights);

end

