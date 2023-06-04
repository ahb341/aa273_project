run("load_data.m");
robot1 = merge_dat(Robot1_Measurement, Robot1_Odometry,1); 
robot2 = merge_dat(Robot2_Measurement, Robot2_Odometry,2); 
robot3 = merge_dat(Robot3_Measurement, Robot3_Odometry,3); 
robot4 = merge_dat(Robot4_Measurement, Robot4_Odometry,4); 
robot5 = merge_dat(Robot5_Measurement, Robot5_Odometry,5); 

robot = [robot1;robot2;robot3;robot4;robot5];
robot = sortrows(robot);
i = 1;
while i < size(robot,1)
    % Set next timestep after a set of measurements as previously recorded input
    j = i;
    if robot(j,2) == 1
        % Decode ID #
        IDList = [72,27,54,70,36,18,25,9,81,16,90,61,45,7,63, 5, 14, 41, 32, 23];
        robot(j,3) = find(IDList==robot(j,3));
        % While the next index is still a measurement and the same time step
        while robot(j+1,2) == 1 && robot(i,1) == robot(j+1,1)
            j = j+ 1;
            robot(j,3) = find(IDList==robot(j,3));
        end
        % Set control inputs at the end of last measurement 
        performedRobot = robot(1:j,:);
        currTime = robot(j,1);
        % Find latest control where we're control input && correct robot
        control1 = findControl(performedRobot, 1, currTime);
        control2 = findControl(performedRobot, 2, currTime);
        control3 = findControl(performedRobot, 3, currTime);
        control4 = findControl(performedRobot, 4, currTime);
        control5 = findControl(performedRobot, 5, currTime);
        control = [control1;control2;control3;control4;control5];
        robot = [robot(1:j,:);control;robot(j+1:end,:)];
        i = j;
    end
    i = i+1;
end

% function index = findIndex(X,value)
%     index = find(X==interp1(X,X,value,'nearest'));
% end

function row = findControl(performedRobot, num, time)
row = performedRobot(find(performedRobot(:,2)==0 & ...
    performedRobot(:,6)==num,1,'last'),:);
if isempty(row)
    row = [time, 0, 0, 0, nan, num];
else
    row(1,1) = time;
end
end

function Robot = merge_dat(measurements, odo, num)
%     measurements = eval("Robot"+RobotNum+"_Measurement");
%     odo = eval("Robot"+RobotNum+"_Odometry");
    meas_sz = size(measurements); odo_sz = size(odo);
    % Initialization of proper size
    Robot = zeros(meas_sz(1), 1+max(odo_sz(2),meas_sz(2)));
    % Populate first measurements.
    Robot(1:meas_sz(1),:) = [measurements, ones(meas_sz(1),1)];
    % Remove columns that do not contain ID numbers
    IDList = [72,27,54,70,36,18,25,9,81,16,90,61,45,7,63, 5, 14, 41, 32, 23];
    Robot(~ismember(Robot(:,2),IDList),:) = [];
%     i = 1;
%     while Robot(i,1) ~= 0
%         if ~ismember(Robot(i,2),IDList)
%             Robot(i,:) = [];
%         else
%             i = i+1;
%         end
%     end
    % Populate odo, since it has a missing column, populate with NaN
    Robot = [Robot;odo, ones(odo_sz(1),1)*NaN, zeros(odo_sz(1),1)];
    % Change column order; [time, indicator, info1, info2, info3 or NaN]
    Robot = Robot(:,[1 5 2 3 4]);
    % If a measurement, indicator column (2) will be =1.
    % Info 1, info2, and info3 columns will be populated
    % If an odometry, indicator column will be =0,
    % info1, info2 columns will be populated. Last column will be NaN
    
    % Sort rows by time
     Robot=sortrows(Robot);
%     i = 1;
%     while i < size(Robot,1)
%         % Set next timestep after a set of measurements as previously recorded input
%         j = i;
%         if Robot(j,2) == 1
%             Robot(j,3) = find(IDList==Robot(j,3));
%             while Robot(j+1,2) == 1 && Robot(i,1) == Robot(j+1,1)
%                 j = j+ 1;
%                 Robot(j,3) = find(IDList==Robot(j,3));
%             end
%             control = Robot(i-1,:);
%             control(1,1) = Robot(j,1);
%             Robot = [Robot(1:j,:);control;Robot(j+1:end,:)];
%             i = j;
%         end
%         i = i+1;
%     end
    Robot = [Robot , ones(size(Robot,1),1)*num];
    % Code to test for any anomalies after a measurement
    % for i = 1:size(Robot,1)
    %     if Robot(i,2) == 1
    %         if Robot(i+1,1) ~= Robot(i,1)
    %             disp(i)
    %         end
    %     end
    % end
end