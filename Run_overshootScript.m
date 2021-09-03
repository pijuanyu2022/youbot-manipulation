% Input the initial configuration of the end-effector in the reference trajectory: Tse_initial
Tse_initial = [[0,0,1,0];[0,1,0,0];[-1,0,0,0.5];[0,0,0,1]];
% Input the cube's initial configuration and desired final configuration
Tsc_initial = [[1,0,0,1];[0,1,0,0];[0,0,1,0.025];[0,0,0,1]];
Tsc_goal = [[0,1,0,0];[-1,0,0,-1];[0,0,1,0.025];[0,0,0,1]];
% Assume the degree between the cube and end-effector is 45 degrees.
% Then we can obtain the end-effector's configuration relative to the cube 
% when it is grasping the cube.
Tce_grasp = [[-0.7071,0,0.7071, 1.0];[0,1,0,0];[-0.7071,0,-0.7071,0.0258];[0,0,0,1]]; 
% Assume the height of standoff is 0.3m.
% Then we input the end-effector's standoff configurations above the cube.
Tce_standoff1 = [[-0.7071,0,0.7071,1.0];[0,1,0,0];[-0.7071,0,-0.7071,0.100];[0,0,0,1]];
% Similarly, we can obtain the desired final end-effector's configuration
% and its standoff configuration.
Tce_standoff2 = [[0,1,0,0];[0.7071,0,-0.7071,-1];[-0.7071,0,-0.7071,0.100];[0,0,0,1]];
Tce_goal = [[0,1,0,0];[0.7071,0,-0.7071,-1];[-0.7071,0,-0.7071,0.0258];[0,0,0,1]];
% Assume k is 1, dt = 0.01.
k = 1;
dt = 0.01;

% input a speed limit
maxVal_joint = 3;
maxVal_wheel = 6; 
minVal_joint = -3;
minVal_wheel = -6;

% input the Feedback gain
Kp = 10.0;
Ki = 20.0;

% input the data of robot
Blist = [[0;0;1;0;0.033;0],...
    [0;-1;0;-0.5076;0;0],...
    [0;-1;0;-0.3526;0;0],...
    [0;-1;0;-0.2176;0;0],...
    [0;0;1;0;0;0]]; 
Tb0 = [[1,0,0,0.1662];[0,1,0,0];[0,0,1,0.0026];[0,0,0,1]];
M0e = [[1,0,0,0.033];[0,1,0,0];[0,0,1,0.6546];[0,0,0,1]];
Xerr_cul = zeros(6,1);% input the initial value of the cumulative of Xerr*dt

% V12 means a 12-vector representing the current configuration of the robot (3 variables for the chassis 
%configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
V12 = [-pi/6, -0.45, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0];
disp('Generating overshootScript csv file.')

% By using function Trajectory, get two Matrix of error data and trajectory
[Matrix_error, Matrix_conf] = TrajectoryGenerator(Tse_initial, Tce_standoff1, Tce_grasp, Tce_standoff2,...
    Tce_goal, k, dt, Kp, Ki,Blist, Tb0, M0e, V12, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul);
csvwrite('overshootScript.csv', Matrix_conf)%generate csv file
disp('Writing error plot data.')
csvwrite('Xerr data in overshoot result.csv', Matrix_error)%generate csv file of error data
% create the plot of error data
disp('plotting error data.')
Xerr0 = Matrix_error(:,1);
plot(Xerr0);
hold on

Xerr1 = Matrix_error(:,2);
plot(Xerr1);
hold on
    
Xerr2 = Matrix_error(:,3);
plot(Xerr2);
hold on
    
Xerr3 = Matrix_error(:,4);
plot(Xerr3);
hold on
    
Xerr4 = Matrix_error(:,5);
plot(Xerr4);
hold on
    
Xerr5 = Matrix_error(:,6);
plot(Xerr5);
hold off
legend({'Xerr0', 'Xerr1', 'Xerr2', 'Xerr3', 'Xerr4', 'Xerr5'})
title('Xerr as a function of time')
xlabel('timestep')
ylabel('Xerr')
disp('Done.')