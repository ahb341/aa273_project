% UTIAS Multi-Robot Cooperative Localization and Mapping Dataset
% produced by Keith Leung (keith.leung@robotics.utias.utoronto.ca) 2009
% Matlab script loadMRCLAMdataSet.m
% Description: This scripts parses the 17 text files that make up a 
% dataset into Matlab arrays. Run this script within the the dataset 
% directory.
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
% landmark 1-15 id
IDList = [72,27,54,70,36,18,25,9,81,16,90,61,45,7,63];

% First, read data
for i=1:n_robots
    % read groundtruth
    disp(['Reading robot ' num2str(i) ' groundtruth'])
    [gt x y theta] = textread([dir 'Robot' num2str(i) '_Groundtruth.dat'], '%f %f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Groundtruth = [gt-gt(1) x y theta];']); 
    clear x y theta;

    % read odometry
    disp(['Reading robot ' num2str(i) ' odometry'])
    [time, v, w] = textread([dir 'Robot' num2str(i) '_Odometry.dat'], '%f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Odometry = [time-gt(1) v w];']);
    clear time v w;
    eval(['u' num2str(i) ' = Robot' num2str(i) '_Odometry;']);
    eval(['u' num2str(i) '_sz = size(u' num2str(i) ');']);

    % read measurements
    disp(['Reading robot ' num2str(i) ' measurements'])
    [time, barcode_num, r b] = textread([dir 'Robot' num2str(i) '_Measurement.dat'], '%f %f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Measurement = [time-gt(1) barcode_num r b];']);
    % remove non-landmark measurements
    j = 1; len = eval(['length(Robot' num2str(i) '_Measurement)']);
    while j <= len
        if ~ismember(eval(['Robot' num2str(i) '_Measurement(j,2)']),IDList)
            eval(['Robot' num2str(i) '_Measurement(j,:) = [];']);
        else
            j = j+1;
        end
        len = eval(['length(Robot' num2str(i) '_Measurement)']);
    end
    clear time barcode_num r b gt len;
end

% Second, adjust odometry 
% [timestamp, v, omega, robot #]
% odo = zeros(u1_sz(1) + u2_sz(1) + u3_sz(1) + u4_sz(1) + u5_sz(1), ...
%     1+u1_sz(2));
% nxt = 1;
% for i = 1:n_robots
%     sz = eval(['u' num2str(i) '_sz(1);']);
%     u = eval(['u' num2str(i) ';']);
%     odo(nxt:nxt+sz-1,:) = [u i*ones(sz,1)];
%     nxt = nxt+sz;
% end
% clear u sz nxt
% odo = sortrows(odo);
% now copy odometry so there is an input at each time step
% UPDATE: handle the discrepancy in the simulation loop
% for i = 1:n_robots
%     last_control = [0 0];
%     j = 1;
%     while j < size(odo,1)
%         % copy the last control input
%         if odo(j,4) == i
%             last_control = odo(j,2:3);
%             j = j+1;
%         else
%             t = odo(j,1);
%             while j < size(odo,1) && odo(j,1) == t && odo(j,4) ~= i
%                 % see if we already have a matching time for our robot
%                 j = j+1;
%             end
%             if odo(j,1) ~= t
%                 % no matching time so insert last control
%                 odo = [odo(1:j,:); [odo(j,1) last_control i]; odo(j+1:end,:)];
%             elseif odo(j,4) == i
%                 % matching time, update last control
%                 last_control = odo(j,2:3);
%             end
%         end
% 
%     end
% end


% finally, merge data
for i=1:n_robots

    % merge data
    disp(['Merging robot ' num2str(i) ' data'])
    eval(['z' num2str(i) ' = Robot' num2str(i) '_Measurement;']);
    eval(['u' num2str(i) ' = Robot' num2str(i) '_Odometry;']);
    eval(['z' num2str(i) '_sz = size(z' num2str(i) ');']);
    eval(['u' num2str(i) '_sz = size(u' num2str(i) ');']);
    % Initialization of proper size
    eval(['Robot' num2str(i) ' = zeros(z' num2str(i) '_sz(1) + u' ...
        num2str(i) '_sz(1), 2+max(u' num2str(i) '_sz(2),z' num2str(i) ...
        '_sz(2)));']);
    % Populate first measurements. 
    eval(['Robot' num2str(i) '(1:z' num2str(i) '_sz(1),:) = [z' ...
        num2str(i) ', ones(z' num2str(i) '_sz(1),1)' ...
        ', ' num2str(i) '*ones(z' num2str(i) '_sz(1),1)];']); 
    % Populate odo, since it has a missing column, populate with NaN
    eval(['Robot' num2str(i) '(z' num2str(i) '_sz(1)+1:end,:) = [u' ...
        num2str(i) ', ones(u' num2str(i) '_sz(1),1)*NaN, zeros(u' ...
        num2str(i) '_sz(1),1)' ...
        ', ' num2str(i) '*ones(u' num2str(i) '_sz(1),1)];']);
    % Change column order; [time, indicator, info1, info2, info3 or NaN, robot #]
    eval(['Robot' num2str(i) ' = Robot' num2str(i) '(:,[1 5 2 3 4 6]);']);
    % If a measurement, indicator column (2) will be =1. 
            % Info 1, info2, and info3 columns will be populated
    % If an odometry, indicator column will be =0,
            % info1, info2 columns will be populated. Last column will be NaN
    eval(['Robot' num2str(i) '=sortrows(Robot' num2str(i) ');']);

    k = 1;
    Robot = eval(['Robot' num2str(i)]);
    while k < size(Robot,1)
        % Set next timestep after a set of measurements as previously recorded input
        j = k;
        if Robot(j,2) == 1
            Robot(j,3) = find(IDList==Robot(j,3));
            while Robot(j+1,2) == 1 && Robot(k,1) == Robot(j+1,1)
                j = j+ 1;
                Robot(j,3) = find(IDList==Robot(j,3));
            end
            control = Robot(k-1,:);
            control(1,1) = Robot(j,1);
            Robot = [Robot(1:j,:);control;Robot(j+1:end,:)];
            k = j;
        end
        k = k+1;
    end
    eval(['Robot' num2str(i) '=Robot;']);
end
clear i dir Robot j k u
% combine everything
Robot = sortrows([Robot1;Robot2;Robot3;Robot4;Robot5]);
disp('Parsing Complete')