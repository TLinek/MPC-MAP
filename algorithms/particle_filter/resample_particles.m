function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here


N = length(particles);
index = randi(N);
w_max = max(weights);

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


%new_particles = particles;

end

