%% Centralized EKF SLAM
reload_data = true;
if (reload_data)
    clear; clc; close all;
    run("load_data.m");
end
nx = n_robots*3 + n_landmarks*2; % dim of state
nu = 2;        % dim of control input

%% Get measurement model and jacobian functions
run("gjac.m")

%% Simulate
run("ekf_simulate.m")

%% Plot
figure(1)
hold on
p1 = plot(Robot1_Groundtruth(:,2), ...
    Robot1_Groundtruth(:,3), '--', 'color', [.5 .5 .5], ...
    'LineWidth', 1); 
p2 = plot(mu_ekf(:,1),mu_ekf(:,2), 'b-', 'LineWidth', 2);
grid on
legend([p1 p2], {'x_true','x_est'})
legend('Interpreter','latex','Location','best','FontAngle','italic','FontSize',15)
xlabel('X Position')
ylabel('Y Position')
title('Robot 1 Trajectory')

figure(2)
hold on
p1 = plot(Robot2_Groundtruth(:,2), ...
    Robot2_Groundtruth(:,3), '--', 'color', [.5 .5 .5], ...
    'LineWidth', 1); 
p2 = plot(mu_ekf(:,4),mu_ekf(:,5), 'b-', 'LineWidth', 2);
grid on
legend([p1 p2], {'x_true','x_est'})
legend('Interpreter','latex','Location','best','FontAngle','italic','FontSize',15)
xlabel('X Position')
ylabel('Y Position')
title('Robot 2 Trajectory')

figure(3)
hold on; grid on
p1 = plot(Robot3_Groundtruth(:,2), ...
    Robot3_Groundtruth(:,3), '--', 'color', [.5 .5 .5], ...
    'LineWidth', 1); 
p2 = plot(mu_ekf(:,7),mu_ekf(:,8), 'b-', 'LineWidth', 2);
legend([p1 p2], {'x_true','x_est'})
legend('Interpreter','latex','Location','best','FontAngle','italic','FontSize',15)
xlabel('X Position')
ylabel('Y Position')
title('Robot 3 Trajectory')

figure(4)
hold on; grid on
p1 = plot(Robot4_Groundtruth(:,2), ...
    Robot4_Groundtruth(:,3), '--', 'color', [.5 .5 .5], ...
    'LineWidth', 1); 
p2 = plot(mu_ekf(:,10),mu_ekf(:,11), 'b-', 'LineWidth', 2);
legend([p1 p2], {'x_true','x_est'})
legend('Interpreter','latex','Location','best','FontAngle','italic','FontSize',15)
xlabel('X Position')
ylabel('Y Position')
title('Robot 4 Trajectory')

figure(5)
hold on; grid on
p1 = plot(Robot5_Groundtruth(:,2), ...
    Robot5_Groundtruth(:,3), '--', 'color', [.5 .5 .5], ...
    'LineWidth', 1); 
p2 = plot(mu_ekf(:,13),mu_ekf(:,14), 'b-', 'LineWidth', 2);
legend([p1 p2], {'x_true','x_est'})
legend('Interpreter','latex','Location','best','FontAngle','italic','FontSize',15)
xlabel('X Position')
ylabel('Y Position')
title('Robot 5 Trajectory')

figure(6)
hold on; grid on
p1 = scatter(mu_ekf(:,4),mu_ekf(:,5), '.');
p2 = scatter(mu_ekf(:,6),mu_ekf(:,7), '.');
p3 = scatter(mu_ekf(:,8),mu_ekf(:,9), '.');
p4 = scatter(mu_ekf(:,10),mu_ekf(:,11), '.');
p5 = scatter(mu_ekf(:,12),mu_ekf(:,13), '.');
p6 = scatter(mu_ekf(:,14),mu_ekf(:,15), '.');
p7 = scatter(mu_ekf(:,16),mu_ekf(:,17), '.');
p8 = scatter(mu_ekf(:,18),mu_ekf(:,19), '.');
p9 = scatter(mu_ekf(:,20),mu_ekf(:,21), '.');
p10 = scatter(mu_ekf(:,22),mu_ekf(:,23), '.');
p11 = scatter(mu_ekf(:,24),mu_ekf(:,25), '.');
p12 = scatter(mu_ekf(:,26),mu_ekf(:,27), '.');
p13 = scatter(mu_ekf(:,28),mu_ekf(:,29), '.');
p14 = scatter(mu_ekf(:,30),mu_ekf(:,31), '.');
p15 = scatter(mu_ekf(:,32),mu_ekf(:,33), '.');
% p3 = plot(m1(1), m1(2), 'ko', 'LineWidth', 2);
% p4 = plot(m2(1), m2(2), 'k+', 'LineWidth', 2);
% p5 = plot(m3(1), m3(2), 'k*', 'LineWidth', 2);
% p6 = plot(m4(1), m4(2), 'kx', 'LineWidth', 2);
% title('2D Actual Location Trajectory of the Robot + Estimated Position of Landmark')
% subtitle(sprintf('Simulation Time: %d sec, Time Step: %g sec', tFinal, dt))
subtitle(sprintf('Initial Guess of Landmarks: m1 = [%g %g], m2 = [%g %g], m3 = [%g %g], m4 = [%g %g], m5 = [%g %g], m6 = [%g %g], m7 = [%g %g], m8 = [%g %g], m9 = [%g %g], m10 = [%g %g], m11 = [%g %g], m12 = [%g %g], m13 = [%g %g], m14 = [%g %g], m15 = [%g %g]',...
                  initial_guess_landmarks))

legend([p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 p14 p15],...
        {'Estimated Location of Landmark 1',...
         'Estimated Location of Landmark 2',...
         'Estimated Location of Landmark 3',...
         'Estimated Location of Landmark 4',...
         'Estimated Location of Landmark 5',...
         'Estimated Location of Landmark 6',...
         'Estimated Location of Landmark 7',...
         'Estimated Location of Landmark 8',...
         'Estimated Location of Landmark 9',...
         'Estimated Location of Landmark 10',...
         'Estimated Location of Landmark 11',...
         'Estimated Location of Landmark 12',...
         'Estimated Location of Landmark 13',...
         'Estimated Location of Landmark 14',...
         'Estimated Location of Landmark 15'})

legend('Interpreter','latex','Location','bestoutside','FontAngle','italic','FontSize',15)
xlabel('X Position')
ylabel('Y Position')
