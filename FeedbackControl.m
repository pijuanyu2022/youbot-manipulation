function [V, Xerr_cul] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_cul)
% Takes:X:the current actual end-effector configuration
%       Xd:the current end-effector reference configuration
%       Xd_next:the end-effector reference configuration at the next timestep in the reference trajectory
%       Kp: P control gain
%       Ki: I control gain
%       dt: timestep
%       Xerr_cul: the cumulative of Xerr*dt
% return: V: the end-effector twist V
%        Xerr_cul: the new cumulative of Xerr*dt
%
% Example Input:
% 
% clear; clc;
% X = [[0.170, 0, 0.985, 0.387];[0, 1, 0, 0];[-0.985, 0, 0.170, 0.570];[0, 0, 0, 1]];
% Xd = [[0,0,1,0.5];[0,1,0,0];[-1,0,0,0.5];[0,0,0,1]];
% Xd_next = [[0,0,1,0.6];[0,1,0,0];[-1,0,0,0.3];[0,0,0,1]];
% Kp = 0;
% Ki = 0;
% dt = 0.01;
% Xerr_cul = [0;0;0;0;0;0];
%
% Output:
% V = 
%     0
%     0
%     0
%  21.4
%     0
%  6.45
%
% Xerr_cul =
%          0
%     0.0017
%          0
%     0.0008
%          0
%     0.0011
Xerr_mat = MatrixLog6(TransInv(X)*Xd);%calculate 4*4 se(3) matrix [Xerr]
Xerr = se3ToVec(Xerr_mat);%get Xerr from [Xerr]
Vd_mat = (1/dt)*MatrixLog6(TransInv(Xd)*Xd_next);%calculate [Vd]
Vd = se3ToVec(Vd_mat);%get Vd from [Vd]
Xerr_cul = Xerr_cul+Xerr*dt;%cumulate Xerr*dt
V = vpa(Adjoint(TransInv(X)*Xd)*Vd + Kp*Xerr + Ki*Xerr_cul);% calculate the end-effector twist V
end