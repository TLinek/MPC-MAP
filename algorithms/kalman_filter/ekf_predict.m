function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

x = mu(1);
y = mu(2);
theta = mu(3);

v = u(1);
omega = u(2);

G = [1, 0, -sin(theta) * v * sampling_period;
     0, 1,  cos(theta) * v * sampling_period;
     0, 0, 1];

R = kf.R;

new_mu = [x + cos(theta) * v * sampling_period, y + sin(theta) * v * sampling_period, theta + omega * sampling_period];

new_sigma = G * sigma * G' + R;

end

