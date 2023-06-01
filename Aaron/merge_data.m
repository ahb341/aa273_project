run("load_data.m");
robot1 = merge_dat(Robot1_Measurement, Robot1_Odometry); 
% robot2 = merge_dat(Robot2_Measurement, Robot2_Odometry); 
% robot3 = merge_dat(Robot3_Measurement, Robot3_Odometry); 
% robot4 = merge_dat(Robot4_Measurement, Robot4_Odometry); 
% robot5 = merge_dat(Robot5_Measurement, Robot5_Odometry); 
function Robot = merge_dat(measurements, odo)
%     measurements = eval("Robot"+RobotNum+"_Measurement");
%     odo = eval("Robot"+RobotNum+"_Odometry");
    meas_sz = size(measurements); odo_sz = size(odo);
    % Initialization of proper size
    Robot = zeros(meas_sz(1)+odo_sz(1), 1+max(odo_sz(2),meas_sz(2)));
    % Populate first measurements.
    Robot(1:meas_sz(1),:) = [measurements, ones(meas_sz(1),1)];
    % Remove columns with ID numbers
    IDList = [72,27,54,70,36,18,25,9,81,16,90,61,45,7,63];
    i = 1;
    while Robot(i,1) ~= 0
        if ~ismember(Robot(i,2),IDList)
            Robot(i,:) = [];
        else
            i = i+1;
        end
    end
    % Populate odo, since it has a missing column, populate with NaN
    Robot(i:end,:) = [odo, ones(odo_sz(1),1)*NaN, zeros(odo_sz(1),1)];
    % Change column order; [time, indicator, info1, info2, info3 or NaN]
    Robot = Robot(:,[1 5 2 3 4]);
    % If a measurement, indicator column (2) will be =1.
    % Info 1, info2, and info3 columns will be populated
    % If an odometry, indicator column will be =0,
    % info1, info2 columns will be populated. Last column will be NaN
    
    % Sort rows by time
    Robot=sortrows(Robot);
    i = 1;
    while i < size(Robot,1)
        % Set next timestep after a set of measurements as previously recorded input
        j = i;
        if Robot(j,2) == 1
            Robot(j,3) = find(IDList==Robot(j,3));
            while Robot(j+1,2) == 1 && Robot(i,1) == Robot(j+1,1)
                j = j+ 1;
                Robot(j,3) = find(IDList==Robot(j,3));
            end
            control = Robot(i-1,:);
            control(1,1) = Robot(j,1);
            Robot = [Robot(1:j,:);control;Robot(j+1:end,:)];
            i = j;
        end
        i = i+1;
    end
    % Code to test for any anomalies after a measurement
    % for i = 1:size(Robot,1)
    %     if Robot(i,2) == 1
    %         if Robot(i+1,1) ~= Robot(i,1)
    %             disp(i)
    %         end
    %     end
    % end
end