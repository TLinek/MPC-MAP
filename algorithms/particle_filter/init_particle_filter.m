function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here


N = read_only_vars.max_particles/2;
particles = zeros(N,3);

x_map_limit = read_only_vars.map.limits([1, 3]);
y_map_limit = read_only_vars.map.limits([2, 4]);
x_map_length = x_map_limit(2) - x_map_limit(1);
x_map_offset = x_map_limit(1);
y_map_length = y_map_limit(2) - y_map_limit(1);
y_map_offset = y_map_limit(1);


for i = 1:N
    particles(i,1) = rand() * x_map_length + x_map_offset;
    particles(i,2) = rand() * y_map_length + y_map_offset;
    particles(i,3) = rand() * 2*pi;
end

public_vars.particles = particles;

end

