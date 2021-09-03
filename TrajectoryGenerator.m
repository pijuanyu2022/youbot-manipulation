function  [Matrix_error, Matrix_conf] = TrajectoryGenerator(Tse_initial, Tce_standoff1, Tce_grasp,...
    Tce_standoff2, Tce_goal, k, dt, Kp, Ki, Blist, Tb0, M0e, V12, maxVal_joint, maxVal_wheel,...
    minVal_joint, minVal_wheel, Xerr_cul)
% Takes:Tse_initial: initial configuration of the end-effector in the reference trajectory
%       Tce_standoff1: the end-effector's standoff configurations above the cube
%       Tce_grasp: the end-effector's configuration relative to the cube when it is grasping the cube
%       Tce_standoff2: the end-effector's standoff configurations above the goal position
%       Tce_goal: the desired final end-effector's configuration
%       k: The number of trajectory reference configurations per 0.01 s
%       dt: timestep
%       Kp: P control gain
%       Ki: I control gain
%       Blist: 
%       Tb0: The fixed offset from the chassis frame {b} to the base frame of the arm {0}
%       M0e: When the arm is at its home configuration, the end-effector frame {e} relative to the arm 
%            base frame {0} 
%       V12 : a 12-vector representing the current configuration of the robot
%       maxVal_joint: the maximum value of joint speed
%       maxVal_wheel: the maximum value of wheel speed
%       minVal_joint: the minimum value of joint speed
%       minVal_wheel: the minimum value of wheel speed
%       Xerr_cul: the cumulative of Xerr*dt
% return: Matrix_error: a matrix of Xerr data
%         Matrix_conf: a matrix of trajectory during the whole process
%
% Segment 1: A trajectory to move the gripper from its initial configuration to a "standoff" configuration 
%           a few cm above the block.
Xstart1 = Tse_initial;
Xend1 = Tce_standoff1;
% Assume the time-scaling method is 3.
method1 = 3;
% Assume the total time of the motion is 5 seconds.
Tf1 = 5;
N1 = (Tf1*k)/0.01;
% At the segment 1, gripper is open. So, gripper1 = 0.
gripper1 = 0;
% Using function FullprogramCartesian to get two matrix of Xerr and trajectory in segment 1,the configuration 
% and the cumulative of Xerr*dt at then end of segment1.
% The trajectory of this motion is a screw path, so we use FullprogramScrew.
[Matrix_error1, Matrix_conf1, V12_1, Xerr_cul_1] = FullprogramCartesian(Xstart1,Xend1, Tf1, N1, method1,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper1, V12, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul);


% Segment 2: A trajectory to move the gripper down to the grasp position.
Xstart2 = Tce_standoff1;
Xend2 = Tce_grasp;
% Assume the time-scaling method is 3.
method2 = 3;
% Assume the total time of the motion is 3 seconds.
Tf2 = 3;
N2 = (Tf2*k)/0.01;
% At the segment 2, gripper is open. So, gripper2 = 0.
gripper2 = 0;
% The trajectory of this motion is decoupled rotation and translation, so we use FullprogramCartesian.
[Matrix_error2, Matrix_conf2, V12_2, Xerr_cul_2] = FullprogramCartesian(Xstart2, Xend2, Tf2, N2, method2,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper2, V12_1, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul_1);


% Segment 3: Closing of the gripper.
Xstart3 = Tce_grasp;
Xend3 = Tce_grasp;
% Assume the time-scaling method is 3.
method3 = 3;
% Assume the total time of the motion is 0.625 seconds.
Tf3 = 0.625;
N3 = (Tf3*k)/0.01;
% At the segment 3, gripper is closed. So, gripper3 = 1.
gripper3 = 1;
% The trajectory of this motion is decoupled rotation and translation, so we use FullprogramCartesian.
[Matrix_error3, Matrix_conf3, V12_3, Xerr_cul_3] = FullprogramCartesian(Xstart3, Xend3, Tf3, N3, method3,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper3, V12_2, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul_2);


% Segment 4:A trajectory to move the gripper back up to the "standoff" configuration.
Xstart4 = Tce_grasp;
Xend4 = Tce_standoff1;
% Assume the time-scaling method is 3.
method4 = 3;
% Assume the total time of the motion is 3 seconds.
Tf4 = 3;
N4 = (Tf4*k)/0.01;
% At the segment 4, gripper is closed. So, gripper4 = 1.
gripper4 = 1;
% The trajectory of this motion is decoupled rotation and translation, so we use FullprogramCartesian.
[Matrix_error4, Matrix_conf4, V12_4, Xerr_cul_4] = FullprogramCartesian(Xstart4, Xend4, Tf4, N4, method4,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper4, V12_3, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul_3);

% Segment 5:A trajectory to move the gripper to a "standoff" configuration above the final configuration.
Xstart5 = Tce_standoff1;
Xend5 = Tce_standoff2;
% Assume the time-scaling method is 3.
method5 =3;
% Assume the total time of the motion is 5 seconds.
Tf5 = 5;
N5 = (Tf5*k)/0.01;
% At the segment 5, gripper is closed. So, gripper5 = 1.
gripper5 = 1;
% The trajectory of this motion is a screw path, so we use FullprogramScrew.
[Matrix_error5, Matrix_conf5, V12_5, Xerr_cul_5] = FullprogramScrew(Xstart5, Xend5, Tf5, N5, method5,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper5, V12_4,maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul_4);

% Segment 6:A trajectory to move the gripper to the final configuration of the object.
Xstart6 = Tce_standoff2;
Xend6 = Tce_goal;
% Assume the time-scaling method is 3.
method6 = 3;
% Assume the total time of the motion is 3 seconds.
Tf6 = 3;
N6 = (Tf6*k)/0.01;
% At the segment 6, gripper is closed. So, gripper6 = 1.
gripper6 = 1;
% The trajectory of this motion is decoupled rotation and translation, so we use FullprogramCartesian.
[Matrix_error6, Matrix_conf6, V12_6, Xerr_cul_6] = FullprogramCartesian(Xstart6, Xend6, Tf6, N6, method6,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper6, V12_5, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul_5);


% Segment 7:Opening of the gripper.
Xstart7 = Tce_goal;
Xend7 = Tce_goal;
% Assume the time-scaling method is 3.
method7 = 3;
% Assume the total time of the motion is 0.625 seconds.
Tf7 = 0.625;
N7 = (Tf7*k)/0.01;
% At the segment 7, gripper is open. So, gripper7 = 0.
gripper7 = 0;
% The trajectory of this motion is decoupled rotation and translation, so we use FullprogramCartesian.
[Matrix_error7, Matrix_conf7, V12_7, Xerr_cul_7] = FullprogramCartesian(Xstart7, Xend7, Tf7, N7, method7,...
    dt, Kp, Ki, Blist, Tb0, M0e, gripper7, V12_6, maxVal_joint, maxVal_wheel, minVal_joint,...
    minVal_wheel, Xerr_cul_6);


% Segment 8:A trajectory to move the gripper back to the "standoff" configuration.
Xstart8 = Tce_goal;
Xend8 = Tce_standoff2;
% Assume the time-scaling method is 3.
method8 = 3;
% Assume the total time of the motion is 1 seconds.
Tf8 = 1;
N8 = (Tf8*k)/0.01;
% At the segment 8, gripper is open. So, gripper8 = 0.
gripper8 = 0;
% The trajectory of this motion is decoupled rotation and translation, so we use FullprogramCartesian.
[Matrix_error8, Matrix_conf8, ~, ~] = FullprogramCartesian(Xstart8, Xend8, Tf8, N8, method8, dt, Kp, Ki,...
    Blist, Tb0, M0e, gripper8, V12_7, maxVal_joint, maxVal_wheel, minVal_joint, minVal_wheel, Xerr_cul_7);

% Combined these matrixes into two matrixes
Matrix_error = [Matrix_error1;Matrix_error2;Matrix_error3;Matrix_error4;Matrix_error5;Matrix_error6;...
    Matrix_error7;Matrix_error8];
Matrix_conf = [Matrix_conf1;Matrix_conf2;Matrix_conf3;Matrix_conf4;Matrix_conf5;Matrix_conf6;...
    Matrix_conf7;Matrix_conf8];
end