function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here


N = read_only_vars.max_particles/4;
particles = zeros(N,3);

for i = 1:N
    particles(i,1) = rand() * 10;
    particles(i,2) = rand() * 10;
    particles(i,3) = rand() * 2*pi;
end

public_vars.particles = particles;

end

