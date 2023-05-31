clear all; close all; clc
run("load_data.m");
measurements = Robot1_Measurement; odo = Robot1_Odometry;
meas_sz = size(measurements); odo_sz = size(odo);
% Initialization of proper size
Robot = zeros(meas_sz(1)+odo_sz(1), 1+max(odo_sz(2),meas_sz(2)));
% Populate first measurements. 
Robot(1:meas_sz(1),:) = [measurements, ones(meas_sz(1),1)]; 
% Populate odo, since it has a missing column, populate with NaN
Robot(meas_sz(1)+1:end,:) = [odo, ones(odo_sz(1),1)*NaN, zeros(odo_sz(1),1)];
% Change column order; [time, indicator, info1, info2, info3 or NaN]
Robot = Robot(:,[1 5 2 3 4]);
% If a measurement, indicator column (2) will be =1. 
        % Info 1, info2, and info3 columns will be populated
% If an odometry, indicator column will be =0,
        % info1, info2 columns will be populated. Last column will be NaN
Robot=sortrows(Robot);