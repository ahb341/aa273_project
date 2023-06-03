disp("Calculating measurement Jacobian")
%% generate symbolic variables for map
for j = 1:n_landmarks
    eval(['syms m' num2str(j) 'x m' num2str(j) 'y real']);
    % define mj for each landmark j
    eval(['m' num2str(j) '=[m' num2str(j) 'x;m' num2str(j) 'y];']);
end

%% generate symbolic variables, models, and jacobians for robots
for i = 1:n_robots
    eval(['syms p' num2str(i) 'x p' num2str(i) 'y th' num2str(i) ' real']);
    % define p i for each robot i
    eval(['p' num2str(i) '=[p' num2str(i) 'x;p' num2str(i) 'y];']);
    % define rotation matrix for each robot
    eval(['RR' num2str(i) '=[cos(th' num2str(i) ') sin(th' num2str(i) ...
        '); -sin(th' num2str(i) ') cos(th' num2str(i) ')];']);
    % define state
    % define the measurement model for each robot
    eval(['g' num2str(i) ' = ' meas_model_str(i, n_landmarks)]);
    eval(['x' num2str(i) ' = ' x_str(i, n_landmarks)]);
    eval(['C' num2str(i) '=jacobian(g' num2str(i) ', x' num2str(i) ');']);
end

% creates the g_t equation as a string
function str = meas_model_str(i, n_landmarks)
str = '[';
for j = 1:n_landmarks
    str = [str 'norm(m' num2str(j) ' - p' num2str(i) '); atan2(RR' ...
        num2str(i) '(2,:)*(m' num2str(j) '-p' num2str(i) '), RR' ...
        num2str(i) '(1,:)*(m' num2str(j) '-p' num2str(i) '));' ...
        'norm(m' num2str(j) '-p' num2str(i) ')'];
    if j ~= n_landmarks
        str = [str ';'];
    end
end
str = [str '];'];
end

function str = x_str(i, n_landmarks)
str = ['[p' num2str(i) ';th' num2str(i)];
for j = 1:n_landmarks
    str = [str '; m' num2str(j)];
end
str = [str '];'];
end