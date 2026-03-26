close all;
clear;
clc;

load('data_history.mat')


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
    title("Lidar channel " + string(i))
    xlabel("Distance [m]")
    ylabel("Sample cnt. [-]")
end




