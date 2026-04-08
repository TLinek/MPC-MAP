function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

N = length(particles(:,1));

N_random = floor(N * 0.1); %round down

index = randi(N);
w_max = max(weights);
weights_norm = weights / sum(weights);
new_particles = zeros(N,3);

for i = 1:N
    beta = rand() * 2 * w_max;
    while weights(index) < beta
        beta = beta - weights_norm(index);
        index = index + 1;
        if index > N
            index = 1;
        end
    end
    new_particles(i,:) = particles(index,:);

end


for i = 1:N_random
    index = randi(N);
    new_particles(index,1) = rand() * 15;
    new_particles(index,2) = rand() * 15;
    new_particles(index,3) = rand() * 2*pi;
end


end

