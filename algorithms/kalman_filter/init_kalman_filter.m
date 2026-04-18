function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0;
                     0, 1, 0]; % (k = 2 x n = 3)
 
public_vars.kf.R = [0.0001, 0,    0;
                    0,    0.0001, 0;
                    0,    0,    0.0001]; %proces noise covariance (n x n)

public_vars.kf.Q = [0.2643, 0.0011;
                    0.0011, 0.2518]; % measurement noise covariace gnss (k x k)


% public_vars.mu = [2, 2, pi/2];
% public_vars.sigma = zeros(3,3);
% 
public_vars.mu = [mean(read_only_vars.gnss_history), 0];

cov_gnss = cov(read_only_vars.gnss_history);
public_vars.sigma = [cov_gnss(1,1), cov_gnss(1,2), 0;   
                     cov_gnss(2,1), cov_gnss(2,2), 0;   
                     0,             0,             20];
end

