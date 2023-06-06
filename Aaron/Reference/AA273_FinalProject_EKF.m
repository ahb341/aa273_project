clear all
close all
clc

%% 
run("load_data.m")

%% System Parameter

M = 15;                     % number of landmark
simulationTime = 1000;      % the time horizon of our simulation
dt = 0.0001;                % time increment
rcomm = 2;                  % within this range, robot communicates. Data retrieved from the paper.
m = 2;                      % number of control inputs

%% Dataset

maxNumRows = 100;                       % Maximum possible number of rows for u and y matrix
numColumns_u = (m+1) * 5;               % Number of columns of u matrix
numColumns_y = (3+1) * 5;               % Number of columns of y matrix

% robot1
mu_1 = zeros(simulationTime*(1/dt), 3 + M*2);
mu_1(1,:) = [3 -3 2 zeros(1, M*2)];     % initialize mu_0|0
sigma_1 = 7 * eye( 3 + M*2 );           % only need to store the covariance matrix when we need to plot the error ellipse. skip right here.
u1 = NaN(maxNumRows, numColumns_u);
y1 = NaN(maxNumRows, numColumns_y);     % tricky one. deal with it later

% robot2
mu_2 = zeros(simulationTime*(1/dt), 3 + M*2);
mu_2(1,:) = [0.5 -1.5 1.5 zeros(1, M*2)];
sigma_2 = 7 * eye( 3 + M*2 );
u2 = NaN(maxNumRows, numColumns_u);
y2 = NaN(maxNumRows, numColumns_y);     % tricky one. deal with it later

% robot3
mu_3 = zeros(simulationTime*(1/dt), 3 + M*2);
mu_3(1,:) = [4 2.5 -2.5 zeros(1, M*2)];
sigma_3 = 7 * eye( 3 + M*2 );
u3 = NaN(maxNumRows, numColumns_u);
y3 = NaN(maxNumRows, numColumns_y);     % tricky one. deal with it later

% robot4
mu_4 = zeros(simulationTime*(1/dt), 3 + M*2);
mu_4(1,:) = [1 2 -0.5 zeros(1, M*2)];
sigma_4 = 7 * eye( 3 + M*2 );
u4 = NaN(maxNumRows, numColumns_u);
y4 = NaN(maxNumRows, numColumns_y);     % tricky one. deal with it later

% robot5
mu_5 = zeros(simulationTime*(1/dt), 3 + M*2);
mu_5(1,:) = [2.5 -1.5 1 zeros(1, M*2)];
sigma_5 = 7 * eye( 3 + M*2 );
u5 = NaN(maxNumRows, numColumns_u);
y5 = NaN(maxNumRows, numColumns_y);     % tricky one. deal with it later

% isnan, nanmean, nanstd

%% Decentralized Cooperative SLAM Algorithm

i = 0;      % reference time

while i < 0.0002
    
    % if we don't have groundtruth data at any specific time step, skip the whole loop
    % the reference time is super ugly. so I round it
    if i ~= round(Robot1_Groundtruth(:,1), 3)
        i = i + dt;
        continue
    end

    % find the index of the row corresponding to time step i
    rowIndex = find(round(Robot1_Groundtruth(:, 1), 3) == i, 1);

    % import all u and y data before simulation time i for each robot
    u_Before_i = Robot1_Odometry(round(Robot1_Odometry(:, 1),3) <= i, :);
    numRows = size(u_Before_i, 1);
    u1(1:numRows, 1:3) = u_Before_i;

    u_Before_i = Robot2_Odometry(round(Robot2_Odometry(:, 1),3) <= i, :);
    numRows = size(u_Before_i, 1);
    u2(1:numRows, 4:6) = u_Before_i;

    u_Before_i = Robot3_Odometry(round(Robot3_Odometry(:, 1),3) <= i, :);
    numRows = size(u_Before_i, 1);
    u3(1:numRows, 7:9) = u_Before_i;

    u_Before_i = Robot4_Odometry(round(Robot4_Odometry(:, 1),3) <= i, :);
    numRows = size(u_Before_i, 1);
    u4(1:numRows, 10:12) = u_Before_i;

    u_Before_i = Robot5_Odometry(round(Robot5_Odometry(:, 1),3) <= i, :);
    numRows = size(u_Before_i, 1);
    u5(1:numRows, 13:15) = u_Before_i;

    y_Before_i = Robot1_Measurement(round(Robot1_Measurement(:, 1),3) <= i, :);
    numRows = size(y_Before_i, 1);
    y1(1:numRows, 1:4) = y_Before_i;

    y_Before_i = Robot2_Measurement(round(Robot2_Measurement(:, 1),3) <= i, :);
    numRows = size(y_Before_i, 1);
    y2(1:numRows, 5:8) = y_Before_i;

    y_Before_i = Robot3_Measurement(round(Robot3_Measurement(:, 1),3) <= i, :);
    numRows = size(y_Before_i, 1);
    y3(1:numRows, 9:12) = y_Before_i;

    y_Before_i = Robot4_Measurement(round(Robot4_Measurement(:, 1),3) <= i, :);
    numRows = size(y_Before_i, 1);
    y4(1:numRows, 13:16) = y_Before_i;

    y_Before_i = Robot5_Measurement(round(Robot5_Measurement(:, 1),3) <= i, :);
    numRows = size(y_Before_i, 1);
    y5(1:numRows, 17:20) = y_Before_i;

% % % % % % % % % Should write a function for this part % % % % % % % % % 

    % check if robot communicates
    % 1 & 2
    if norm([Robot1_Groundtruth(rowIndex, 2), Robot1_Groundtruth(rowIndex, 3)]...
            - [Robot2_Groundtruth(rowIndex, 2), Robot2_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u1(4:6) = u2(4:6);
        u2(1:3) = u1(1:3);
        y1(5:8) = y2(5:8);
        y2(1:4) = y1(1:4);
    
    end

    % 1 & 3
    if norm([Robot1_Groundtruth(rowIndex, 2), Robot1_Groundtruth(rowIndex, 3)]...
            - [Robot3_Groundtruth(rowIndex, 2), Robot3_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u1(7:9) = u3(7:9);
        u3(1:3) = u1(1:3);
        y1(9:12) = y3(9:12);
        y3(1:4) = y1(1:4);
    
    end

    % 1 & 4
    if norm([Robot1_Groundtruth(rowIndex, 2), Robot1_Groundtruth(rowIndex, 3)]...
            - [Robot4_Groundtruth(rowIndex, 2), Robot4_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u1(10:12) = u4(10:12);
        u4(1:3) = u1(1:3);
        y1(13:16) = y4(13:16);
        y4(1:4) = y1(1:4);
   
    end

    % 1 & 5
    if norm([Robot1_Groundtruth(rowIndex, 2), Robot1_Groundtruth(rowIndex, 3)]...
            - [Robot5_Groundtruth(rowIndex, 2), Robot5_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u1(13:15) = u5(13:15);
        u5(1:3) = u1(1:3);
        y1(17:20) = y5(17:20);
        y5(1:4) = y1(1:4);
    
    end

    % 2 & 3
    if norm([Robot2_Groundtruth(rowIndex, 2), Robot2_Groundtruth(rowIndex, 3)]...
            - [Robot3_Groundtruth(rowIndex, 2), Robot3_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u2(7:9) = u3(7:9);
        u3(4:6) = u2(4:6);
        y2(9:12) = y3(9:12);
        y3(5:8) = y2(5:8);
    
    end

    % 2 & 4
    if norm([Robot2_Groundtruth(rowIndex, 2), Robot2_Groundtruth(rowIndex, 3)]...
            - [Robot4_Groundtruth(rowIndex, 2), Robot4_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u2(10:12) = u4(10:12);
        u4(4:6) = u2(4:6);
        y2(13:16) = y4(13:16);
        y4(5:8) = y2(5:8);

    end

    % 2 & 5
    if norm([Robot2_Groundtruth(rowIndex, 2), Robot2_Groundtruth(rowIndex, 3)]...
            - [Robot5_Groundtruth(rowIndex, 2), Robot5_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u2(13:15) = u5(13:15);
        u5(4:6) = u2(4:6);
        y2(17:20) = y5(17:20);
        y5(5:8) = y2(5:8);
    
    end

    % 3 & 4
    if norm([Robot3_Groundtruth(rowIndex, 2), Robot3_Groundtruth(rowIndex, 3)]...
            - [Robot4_Groundtruth(rowIndex, 2), Robot4_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u3(10:12) = u4(10:12);
        u4(7:9) = u3(7:9);
        y3(13:16) = y4(13:16);
        y4(9:12) = y3(9:12);
    
    end

    % 3 & 5
    if norm([Robot3_Groundtruth(rowIndex, 2), Robot3_Groundtruth(rowIndex, 3)]...
            - [Robot5_Groundtruth(rowIndex, 2), Robot5_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u3(13:15) = u5(13:15);
        u5(7:9) = u3(7:9);
        y3(17:20) = y5(17:20);
        y5(9:12) = y3(9:12);
    
    end

    % 4 & 5
    if norm([Robot4_Groundtruth(rowIndex, 2), Robot4_Groundtruth(rowIndex, 3)]...
            - [Robot5_Groundtruth(rowIndex, 2), Robot5_Groundtruth(rowIndex, 3)]) <= rcomm
        
        u4(13:15) = u5(13:15);
        u5(10:12) = u4(10:12);
        y4(17:20) = y5(17:20);
        y5(13:16) = y4(13:16);
    
    end
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

    i = i + dt;
end

