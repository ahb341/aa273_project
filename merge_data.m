function Robot = merge_data(RobotNum)
run("load_data.m");
measurements = eval("Robot"+RobotNum+"_Measurement"); 
odo = eval("Robot"+RobotNum+"_Odometry");
meas_sz = size(measurements); odo_sz = size(odo);
% Initialization of proper size
Robot = zeros(meas_sz(1)+odo_sz(1), 1+max(odo_sz(2),meas_sz(2)));
% Populate first measurements. 
Robot(1:meas_sz(1),:) = [measurements, ones(meas_sz(1),1)]; 
% Remove columns with ID numbers
IDNumbers = [5 14 41 32 23];
i = 1;
while Robot(i,1) ~= 0
    if ismember(Robot(i,2),IDNumbers)
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
Robot=sortrows(Robot);
i = 1;
while i < size(Robot,1)
    j = i;
    if Robot(j,2) == 1 
        while Robot(j+1,2) == 1 && Robot(i,1) == Robot(j+1,1)
            j = j+ 1;
        end
        control = Robot(i-1,:);
        control(1,1) = Robot(j,1);
        Robot = [Robot(1:j,:);control;Robot(j+1:end,:)];
        i = j;
    end
    i = i+1;
end
% 
% for i = 1:size(Robot,1)
%     if Robot(i,2) == 1
%         if Robot(i+1,1) ~= Robot(i,1)
%             disp(i)
%         end
%     end
% end
end