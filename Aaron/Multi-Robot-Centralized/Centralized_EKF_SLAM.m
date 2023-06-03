%% Centralized EKF SLAM
reload_data = false;
if (reload_data)
    clear; clc; close all;
    run("load_data.m");
end
nx = n_robots*3 + n_landmarks*2; % dim of state
nu = 2;        % dim of control input

%% Get measurement model and jacobian functions
run("gjac.m")

%% Simulate
run("ekf_simulate.m")

%% Plot
%run("ekf_plot.m")