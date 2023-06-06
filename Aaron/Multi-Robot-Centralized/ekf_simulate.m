%% setup
tstart = 9.4250;
tfin = 100;
dt = 0.001;
tarr = tstart:dt:tfin;
tlen = length(tarr);

% pose is 
% [x1 ... x5 m1 ... m15]

% initial pose guess is the same as the true pose for now
x1_0 = Robot1_Groundtruth(1,2:4);
x2_0 = Robot2_Groundtruth(1,2:4);
x3_0 = Robot3_Groundtruth(1,2:4);
x4_0 = Robot4_Groundtruth(1,2:4);
x5_0 = Robot5_Groundtruth(1,2:4);
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0];

initial_guess_landmarks = [];
for k = 1:15
    initial_guess_landmarks = [initial_guess_landmarks Landmark_Groundtruth(k,2:3)];
end

mu_ekf = NaN(tlen,nx);  
mu_ekf(1,:) = [x0 initial_guess_landmarks]; 
% mu_pred = mu_ekf(1,:);
clear x1_0 x2_0 x3_0 x4_0 x5_0

sigma_ekf = NaN(nx*tlen,nx);
sigma_ekf(1:nx, 1:nx) = blkdiag(0.001*eye(15),1*eye(30));

%% simulate
k = 1;
% t = Robot(1,1);
u_prev = zeros(5,2); % row is for robot 1-5, col is the control input
u_curr = zeros(5,2);
control_update = false;
for mu_idx = 1:tlen
    t = tarr(mu_idx);
    disp(t)

    if (abs(t-Robot(k,1)) < dt)
        % update last control inputs and import measurement data, if any
        ii = k;
        y_tmp = [];
        while Robot(ii,1) == Robot(k,1)
            if Robot(ii,2) == 0
                % control data
                robot_num = Robot(ii,6);
                u_curr(robot_num,:) = Robot(ii,3:4);
                control_update = true;
            else
                % measurement data
                y_tmp = [y_tmp; Robot(ii,3:6)]; % includes the robot number
            end
            ii = ii+1;
        end
        % sort measurements by robot num
        if ~isempty(y_tmp)
            y_tmp = sortrows(y_tmp,4);
        end
        k = ii;
    end


    % PREDICTION
    mu_pred = f(mu_ekf(mu_idx,:),u_prev,dt,n_robots);

    A = fjac(mu_ekf(mu_idx,:),u_prev,dt,n_robots,n_landmarks);
%     Q = dt*[0.1*eye(3*n_robots) zeros(3*n_robots,2*n_landmarks); ...
%         zeros(2*n_landmarks,3*n_robots) 0.01*eye(2*n_landmarks)];
    Q = 0.01*dt*eye(45);
    sigma_pred = A * sigma_ekf(nx*mu_idx-nx+1:nx*mu_idx,:) * A' + Q;

    % UPDATE
    if ~isempty(y_tmp)
        p1x = mu_pred(1); p1y = mu_pred(2); th1 = mu_pred(3);
        p2x = mu_pred(4); p2y = mu_pred(5); th2 = mu_pred(6);
        p3x = mu_pred(7); p3y = mu_pred(8); th3 = mu_pred(9);
        p4x = mu_pred(10); p4y = mu_pred(11); th4 = mu_pred(12);
        p5x = mu_pred(13); p5y = mu_pred(14); th5 = mu_pred(15);
        m1x = mu_pred(16); m1y = mu_pred(17); 
        m2x = mu_pred(18); m2y = mu_pred(19);
        m3x = mu_pred(20); m3y = mu_pred(21);
        m4x = mu_pred(22); m4y = mu_pred(23);
        m5x = mu_pred(24); m5y = mu_pred(25);
        m6x = mu_pred(26); m6y = mu_pred(27);
        m7x = mu_pred(28); m7y = mu_pred(29);
        m8x = mu_pred(30); m8y = mu_pred(31);
        m9x = mu_pred(32); m9y = mu_pred(33);
        m10x = mu_pred(34); m10y = mu_pred(35);
        m11x = mu_pred(36); m11y = mu_pred(37);
        m12x = mu_pred(38); m12y = mu_pred(39);
        m13x = mu_pred(40); m13y = mu_pred(41);
        m14x = mu_pred(42); m14y = mu_pred(43);
        m15x = mu_pred(44); m15y = mu_pred(45);

        % get the symbolic expressions as values
%         gref = get_gref(m10x,m10y,m11x,m11y,m12x,m12y,m13x,m13y,m14x,...
%             m14y,m15x,m15y,m1x,m2x,m3x,m4x,m5x,m6x,m7x,m8x,m9x,m1y,m2y,...
%             m3y,m4y,m5y,m6y,m7y,m8y,m9y,p1x,p2x,p3x,p4x,p5x,p1y,p2y,p3y,...
%             p4y,p5y,th1,th2,th3,th4,th5);
gref = get_gref(m10x,m10y,m11x,m11y,m12x,m12y,m13x,m13y,m14x,m14y,m15x,m15y,m1x,m2x,m3x,m4x,m5x,m6x,m7x,m8x,m9x,m1y,m2y,m3y,m4y,m5y,m6y,m7y,m8y,m9y,p1x,p2x,p3x,p4x,p5x,p1y,p2y,p3y,p4y,p5y,th1,th2,th3,th4,th5);
%         Cref = get_Cref(m10x,m10y,m11x,m11y,m12x,m12y,m13x,m13y,m14x,...
%             m14y,m15x,m15y,m1x,m2x,m3x,m4x,m5x,m6x,m7x,m8x,m9x,m1y,m2y,...
%             m3y,m4y,m5y,m6y,m7y,m8y,m9y,p1x,p2x,p3x,p4x,p5x,p1y,p2y,p3y,...
%             p4y,p5y,th1,th2,th3,th4,th5);
Cref = get_Cref(m10x,m10y,m11x,m11y,m12x,m12y,m13x,m13y,m14x,m14y,m15x,m15y,m1x,m2x,m3x,m4x,m5x,m6x,m7x,m8x,m9x,m1y,m2y,m3y,m4y,m5y,m6y,m7y,m8y,m9y,p1x,p2x,p3x,p4x,p5x,p1y,p2y,p3y,p4y,p5y,th1,th2,th3,th4,th5);

        % determine the appropriate matrices
        yidx = 1;
        y = []; C = []; g = [];
        for i = 1:n_robots
            j = yidx;
            while ( j <= size(y_tmp,1) && y_tmp(j,4) == i)
                % add all the measurements for robot i
                % 1) add the range bearing measurement
                y = [y; y_tmp(j,2); y_tmp(j,3)]; 
                % 2) add the associated measurement model
                nm = 2*n_landmarks + 8; % 38
                if (y_tmp(j,1) <= n_robots)
                    % saw a robot
                    rob_num = y_tmp(j,1);
                    gidx = nm*(i-1) + 2*(rob_num-1) + 1;
                else
                    % saw a landmark
                    map_num = y_tmp(j,1)-n_robots;
                    gidx = nm*(i-1) + 2*(map_num-1) + 9;
                end
                g = [g; gref(gidx:gidx+1)];
                % 3) add the associated jacobian
                C = [C; Cref(gidx:gidx+1,:)];
                j = j + 1;
            end
            yidx = j;
        end

        R = 1*eye(size(y,1));
        K = sigma_pred * C' / (C*sigma_pred*C' + R);

        % finish up update step and fix indices
        mu_ekf(mu_idx+1,:) = mu_pred + (K * (y - g))';
        sigma_ekf(nx*mu_idx+1:nx*mu_idx+nx,:) = sigma_pred - K * C * sigma_pred;
        
%         for i = k+2:ii+1
%             mu_ekf(k,:) = mu_ekf(k+1,:);
%             sigma_ekf(nx*(i-1)+1:nx*(i-1)+nx,:) = sigma_ekf(nx*k+1:nx*k+nx,:);
%         end
%         k = ii + 1;
    else
       mu_ekf(mu_idx+1,:) = mu_pred;
       sigma_ekf(nx*mu_idx+1:nx*mu_idx+nx,:) = sigma_pred;

    end

    if (control_update)
        u_prev = u_curr;
        control_update = false;
    end
end

% dynamics model for 5 robots
% x: state 1x45
% u: control input for all 5 robots (5x2)
function xpred = f(x,u,dt,n_robots)
    % all the landmarks are static so just copy all of x for convenience
    xpred = x; 

    % calculate the robot dynamics 
    for i = 1:n_robots
        idx = 3*(i-1) + 1;
        
        % unpack variables
        px = x(idx); py = x(idx+1); th = x(idx+2);
        v = u(i,1); om = u(i,2);

        % calculate dynamics
        xpred(idx:idx+2) = [px py th] + dt*[v*cos(th) v*sin(th) om];
    end
end

function A = fjac(x,u,dt,n_robots,n_landmarks)
    A_pose = zeros(3*n_robots,3*n_robots);
    for i = 1:n_robots
        idx = 3*(i-1) + 1;
        th = x(idx+2); v = u(i,1);
        A_pose(idx:idx+2,idx:idx+2) = ...
            [1 0 -dt*v*sin(th);...
             0 1  dt*v*cos(th);...
             0 0             1];
    end
    A_landmark = eye(2*n_landmarks);

    A = [A_pose  zeros(3*n_robots,2*n_landmarks);...
         zeros(2*n_landmarks,3*n_robots)  A_landmark];
end