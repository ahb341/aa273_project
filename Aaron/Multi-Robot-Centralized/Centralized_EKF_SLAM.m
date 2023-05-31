%% Centralized EKF SLAM
clear; clc; close all;
run("load_data.m");
nx = 3 + 15*2; % dim of state
nu = 2;        % dim of control input

%% Get measurement model and jacobian functions
run("gjac.m")