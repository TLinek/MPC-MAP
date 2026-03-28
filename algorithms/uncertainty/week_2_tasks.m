close all;
clear;
clc;

load('data_history_1.mat')


%% TASK 2
gnss_std = std(public_vars.gnss_data_history);
lidar_std = std(public_vars.lidar_data_history);

figure(1)
subplot(1,2,1)
histogram(public_vars.gnss_data_history(:,1))
title("GNSS (1. axis)")
xlabel("Distance [m]")
ylabel("Sample cnt. [-]")
subplot(1,2,2)
histogram(public_vars.gnss_data_history(:,2))
title("GNSS (2. axis)")
xlabel("Distance [m]")
ylabel("Sample cnt. [-]")

figure(2)
for i = 1:8
    subplot(2,4,i)
    histogram(public_vars.lidar_data_history(:,i))
    title("Lidar (channel " + string(i) + ")")
    xlabel("Distance [m]")
    ylabel("Sample cnt. [-]")
end


%% TASK 3
gnss_cov_matrix = cov(public_vars.gnss_data_history);
lidar_cov_matrix = cov(public_vars.lidar_data_history);
gnss_variance = gnss_std.^2;
lidar_variance = lidar_std.^2;

%% TASK 4
x_gnss = [-5*gnss_std(1):0.005:5*gnss_std(1)];
x_lidar = [-5*lidar_std(1):0.005:5*lidar_std(1)];

gnss_pdf = norm_pdf(x_gnss, 0, gnss_std(1));
lidar_pdf = norm_pdf(x_lidar, 0, lidar_std(1));

figure(3)
plot(x_gnss, gnss_pdf, LineWidth=2)
hold on
plot(x_lidar, lidar_pdf, LineWidth=2)
xlabel("Distance Error [m]")
ylabel("PD [-]")
legend("GNSS PDF","LiDAR PDF")





