function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    % WEEK 2
    %public_vars.gnss_data_history = [];
    %public_vars.lidar_data_history = [];
end

% WEEK 2
%public_vars.gnss_data_history = [public_vars.gnss_data_history; read_only_vars.gnss_position];
%public_vars.lidar_data_history = [public_vars.lidar_data_history; read_only_vars.lidar_distances];

% WEEK 3
% line path
    % x = [1:0.1:13];
    % y_line = linspace(1, 14, length(x));
    % public_vars.path  = [x', y_line'];
    % public_vars.path_index = 1;

% sine path
    % x = [1:0.1:13];
    % y_line = linspace(1, 12.8, length(x));
    % y_sine = y_line + 0.9*sin(4*x) +0.5;
    % public_vars.path  = [x', y_sine'];
    % public_vars.path_index = 1;

% circular path
    % x1_center = 1;
    % y1_center = 8;
    % R1 = 7;
    % angle1 = [-pi/2:0.005*pi:-0.01*pi];
    % x1 = x1_center+R1*cos(angle1);
    % y1 = y1_center+R1*sin(angle1);
    % public_vars.path  = [x1', y1'];
    % public_vars.path_index = 1;
    % 
    % x2_center = 14;
    % y2_center = 8;
    % R2 = 6;
    % angle2 = [pi:-0.005*pi:1.1*pi/2];
    % x2 = x2_center+R2*cos(angle2);
    % y2 = y2_center+R2*sin(angle2);
    % public_vars.path  = [public_vars.path; x2', y2'];
    % public_vars.path_index = 1;

% complex path
    % x_sine = [2:0.1:11];
    % y_sine = 0.8*sin(x_sine*0.7 - 1.4) +2;
    % public_vars.path  = [x_sine', y_sine'];
    % 
    % x = [12, 13, 10, 8, 6, 3, 3];
    % y = [2.2, 4 ,6, 7 ,10, 11, 13];
    % t = 0:0.05:6;
    % x_curve = spline(0:6, x, t);
    % y_curve = spline(0:6, y, t);
    % public_vars.path  = [public_vars.path; x_curve', y_curve'];
    % 
    % x = [3.3:0.1:13];
    % y_line = linspace(13, 14, length(x));
    % public_vars.path  = [public_vars.path; x', y_line'];
    % 
    % public_vars.path_index = 1;


% path indoor_3 map
    x = [1:0.1:6];
    y_line = linspace(1, 5, length(x));
    public_vars.path  = [x', y_line'];

    x = [6.05:0.1:7];
    y_line = linspace(5, 5, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = ones(1,40) * 7;
    y_line = linspace(5, 1, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = [7.05:0.1:9];
    y_line = linspace(1, 1, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = ones(1,80) * 9;
    y_line = linspace(1, 9, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = [9.05:-0.1:7];
    y_line = linspace(9.05, 9, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = [7.05:-0.1:1];
    y_line = linspace(9.05, 4, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = ones(1,30) * 1;
    y_line = linspace(4.05, 7, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    x = [1.05:0.1:3];
    y_line = linspace(7.05, 9, length(x));
    public_vars.path  = [public_vars.path; x', y_line'];

    public_vars.path_index = 1;









% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

