% UTIAS Multi-Robot Cooperative Localization and Mapping Dataset
% produced by Keith Leung (keith.leung@robotics.utias.utoronto.ca) 2009
% Matlab script loadMRCLAMdataSet.m
% Description: This scripts parses the 17 text files that make up a 
% dataset into Matlab arrays. Run this script within the the dataset 
% directory.

clear; clc; close all;

%% Load Data
dir = '../../UTIAS-dataset/MRCLAM_Dataset1/';

n_robots = 5;

disp('Parsing Dataset')
disp('Reading barcode numbers')
[subject_num, barcode_num] = textread([dir,'Barcodes.dat'], '%u %u','commentstyle','shell');
Barcodes = [subject_num, barcode_num];
clear subject_num barcode_num;

disp('Reading landmark groundtruth')
[subject_num x y x_sd y_sd] = textread([dir,'Landmark_Groundtruth.dat'], '%f %f %f %f %f','commentstyle','shell');
Landmark_Groundtruth = [subject_num x y x_sd y_sd];
clear subject_num x y x_sd y_sd;
n_landmarks = length(Landmark_Groundtruth); 

for i=1:n_robots
 
    disp(['Reading robot ' num2str(i) ' groundtruth'])
    [gt x y theta] = textread([dir 'Robot' num2str(i) '_Groundtruth.dat'], '%f %f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Groundtruth = [gt-gt(1) x y theta];']); 
    clear x y theta;

    disp(['Reading robot ' num2str(i) ' odometry'])
    [time, v, w] = textread([dir 'Robot' num2str(i) '_Odometry.dat'], '%f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Odometry = [time-gt(1) v w];']);
    clear time v w;
    
    disp(['Reading robot ' num2str(i) ' measurements'])
    [time, barcode_num, r b] = textread([dir 'Robot' num2str(i) '_Measurement.dat'], '%f %f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Measurement = [time-gt(1) barcode_num r b];']);
    clear time barcode_num r b;

    disp(['Calculating robot ' num2str(i) ' covariance'])
    eval(['[N,~] = size(Robot' num2str(i) '_Groundtruth);']);
    eval(['xarr = Robot' num2str(i) '_Groundtruth(:,2);']); 
    eval(['yarr = Robot' num2str(i) '_Groundtruth(:,3);']);
    eval(['tharr = Robot' num2str(i) '_Groundtruth(:,4);']);
    eval(['Q' num2str(i) ' = calcQ(xarr,yarr,tharr,N);']);
    eval(['[N,~] = size(Robot' num2str(i) '_Measurement);']);
    eval(['rarr = Robot' num2str(i) '_Measurement(:,3);']);
    eval(['barr = Robot' num2str(i) '_Measurement(:,4);']);
    eval(['R' num2str(i) ' = calcR(rarr,barr,N);']);
    clear N xarr yarr tharr rarr barr

    % merge data
    disp(['Merging robot ' num2str(i) ' data'])
    eval(['z' num2str(i) ' = Robot' num2str(i) '_Measurement;']);
    eval(['u' num2str(i) ' = Robot' num2str(i) '_Odometry;']);
    eval(['z' num2str(i) '_sz = size(z' num2str(i) ');']);
    eval(['u' num2str(i) '_sz = size(u' num2str(i) ');']);
    % Initialization of proper size
    eval(['Robot' num2str(i) ' = zeros(z' num2str(i) '_sz(1) + u' ...
        num2str(i) '_sz(1), 1+max(u' num2str(i) '_sz(2),z' num2str(i) ...
        '_sz(2)));']);
    eval(['Robot' num2str(i) ' = zeros(z' num2str(i) '_sz(1)+u' ...
        num2str(i) '_sz(1), 1+max(u' num2str(i) '_sz(2),z' ...
        num2str(i) '_sz(2)));']);
    % Populate first measurements. 
    eval(['Robot' num2str(i) '(1:z' num2str(i) '_sz(1),:) = [z' ...
        num2str(i) ', ones(z' num2str(i) '_sz(1),1)];']); 
    % Populate odo, since it has a missing column, populate with NaN
    eval(['Robot' num2str(i) '(z' num2str(i) '_sz(1)+1:end,:) = [u' ...
        num2str(i) ', ones(u' num2str(i) '_sz(1),1)*NaN, zeros(u' ...
        num2str(i) '_sz(1),1)];']);
    % Change column order; [time, indicator, info1, info2, info3 or NaN]
    eval(['Robot' num2str(i) ' = Robot' num2str(i) '(:,[1 5 2 3 4]);']);
    % If a measurement, indicator column (2) will be =1. 
            % Info 1, info2, and info3 columns will be populated
    % If an odometry, indicator column will be =0,
            % info1, info2 columns will be populated. Last column will be NaN
    eval(['Robot' num2str(i) '=sortrows(Robot' num2str(i) ');']);
end
clear i dir
disp('Parsing Complete')