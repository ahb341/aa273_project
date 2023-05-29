clear all
close all
clc

%% System Parameter

M = 15;                     % number of landmark
simulationTime = 50000;     % how many time step we want to simulate
rcomm = 0.3;                % within this range, robot communicates

%% 
run("load_data.m")

%% robot 1 dataset

mu_1 = zeros(simulationTime, 3 + M*2);
mu_1(1,:) = [3 -3 2 zeros(1, M*2)];     % initialize mu_0|0
sigma_1 = 7 * eye( 3 + M*2 );           % only need to store the covariance matrix when we need to plot the error ellipse. skip right here.

%% robot 2 dataset

mu_2 = zeros(simulationTime, 3 + M*2);
mu_2(1,:) = [0.5 -1.5 1.5 zeros(1, M*2)];
sigma_2 = 7 * eye( 3 + M*2 );

%% robot 3 dataset 

mu_3 = zeros(simulationTime, 3 + M*2);
mu_3(1,:) = [4 2.5 -2.5 zeros(1, M*2)];
sigma_3 = 7 * eye( 3 + M*2 );

%% robot 4 dataset

mu_4 = zeros(simulationTime, 3 + M*2);
mu_4(1,:) = [1 2 -0.5 zeros(1, M*2)];
sigma_4 = 7 * eye( 3 + M*2 );

%% robot 5 dataset

mu_5 = zeros(simulationTime, 3 + M*2);
mu_5(1,:) = [2.5 -1.5 1 zeros(1, M*2)];
sigma_5 = 7 * eye( 3 + M*2 );

%% 


