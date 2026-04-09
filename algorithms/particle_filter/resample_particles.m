function [new_particles] = resample_particles(particles, weights, read_only_vars)
%RESAMPLE_PARTICLES Summary of this function goes here

N = length(particles(:,1));

N_random = floor(N * 0.1); %round down

index = randi(N);
w_max = max(weights);
new_particles = zeros(N,3);


for i = 1:N
    beta = rand() * 2 * w_max;
    while weights(index) < beta
        beta = beta - weights(index);
        index = index + 1;
        if index > N
            index = 1;
        end
    end
    new_particles(i,:) = particles(index,:);

end





x_map_limit = read_only_vars.map.limits([1, 3]);
y_map_limit = read_only_vars.map.limits([2, 4]);
x_map_length = x_map_limit(2) - x_map_limit(1);
x_map_offset = x_map_limit(1);
y_map_length = y_map_limit(2) - y_map_limit(1);
y_map_offset = y_map_limit(1);

for i = 1:N_random
    index = randi(N);
    new_particles(index,1) = rand() * x_map_length + x_map_offset;
    new_particles(index,2) = rand() * y_map_length + y_map_offset;
    new_particles(index,3) = rand() * 2*pi;
end


end

