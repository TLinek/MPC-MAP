function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));

%lidar_config ... úhel kanálu lidaru
for i = 1:length(lidar_config)
    lidar_theta = pose(3) + lidar_config(i);
    lidar_pose = pose(1:2);
    intersections = ray_cast(lidar_pose, map.walls, lidar_theta);
    wall_distances = vecnorm(intersections - lidar_pose, 2, 2); %2 eukli. vz., 2 podle řádků
    measurement(i) = min(wall_distances);
end

end

