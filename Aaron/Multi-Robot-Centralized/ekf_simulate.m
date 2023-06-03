%% setup
% simulate the first 100000 rows 
sim_time = 1000;

% pose is 
% [x1 ... x5 m1 ... m15]

% initial pose guess is the same as the true pose for now
x1_0 = Robot1_Groundtruth(1,2:4);
x2_0 = Robot2_Groundtruth(1,2:4);
x3_0 = Robot3_Groundtruth(1,2:4);
x4_0 = Robot4_Groundtruth(1,2:4);
x5_0 = Robot5_Groundtruth(1,2:4);
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0];

initial_guess_landmarks = zeros(1,30);

mu_ekf = NaN(sim_time,nx);  
mu_ekf(1,:) = [x0 initial_guess_landmarks]; 
clear x1_0 x2_0 x3_0 x4_0 x5_0

sigma_ekf = NaN(nx*sim_time,nx);
sigma_ekf(1:nx, 1:nx) = 7*eye(nx);         % initialize sigma_0|0 as 7I

%% simulate
k = 1;
t = Robot(1,1);
u_prev = zeros(5,2); % row is for robot 1-5, col is the control input
while (k < sim_time)
    disp(k)

    dt = round(Robot(k,1) - t, 3);
    t = Robot(k,1);

    % update last control inputs and import measurement data, if any
    ii = k;
    y_tmp = [];
    while Robot(ii,1) == t
        if Robot(ii,2) == 0
            % control data
            robot_num = Robot(ii,6);
            u_prev(robot_num,:) = Robot(ii,3:4);
        else
            % measurement daata
            y_tmp = [y_tmp; Robot(ii,3:6)]; % includes the robot number
        end
        ii = ii+1;
    end
    % sort measurements by robot num
    if ~isempty(y_tmp)
        y_tmp = sortrows(y_tmp,4);
    end


    % PREDICTION
    mu_pred = f(mu_ekf(k,:),u_prev,dt,n_robots);
    A = fjac(mu_ekf(k,:),u_prev,dt,n_robots,n_landmarks);
    Q = dt*[0.1*eye(3*n_robots) zeros(3*n_robots,2*n_landmarks); ...
        zeros(2*n_landmarks,3*n_robots) 0.01*eye(2*n_landmarks)];
    sigma_pred = A * sigma_ekf(nx*k-nx+1:nx*k,:) * A' + Q;

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
        C1 = subs(C1); C2 = subs(C2); C3 = subs(C3); C4 = subs(C4); 
        C5 = subs(C5); RR1 = subs(RR1); RR2 = subs(RR2); RR3 = subs(RR3);
        RR4 = subs(RR4); RR5 = subs(RR5); g1 = subs(g1); g2 = subs(g2);
        g3 = subs(g3); g4 = subs(g4); g5 = subs(g5);

        % determine the appropriate matrices
        yidx = 1;
        y = []; C = []; g = [];
        for i = 1:n_robots
            if i == 1
                CC = C1; gg = g1;
            elseif i == 2
                CC = C2; gg = g2;
            elseif i == 3
                CC = C3; gg = g3;
            elseif i == 4
                CC = C4; gg = g4;
            else
                CC = C5; gg = g5;
            end
            j = yidx;
            while ( j < size(y_tmp,1) && y_tmp(j,4) == i)
                %% FIX C AND G JACOBIAN CALC SO THEY ARE THE RIGHT SIZE IN GJAC.M
                % add all the measurements for robot i
                y = [y; y_tmp(j,2); y_tmp(j,3)];
                C = [C; CC(2*y_tmp(j,1)-1,:); CC(2*y_tmp(j,1),:)];
                g = [g; gg(2*y_tmp(j,1)-1); gg(2*y_tmp(j,1))];
                j = j + 1;
            end
            yidx = j;
        end

        R = 0.1*eye(size(y,1));
        K = sigma_pred * C' / (C*sigma_pred*C' + R);

    end

    k = k + 1;
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