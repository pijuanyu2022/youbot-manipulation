function [Matrix_error, Matrix_conf, V12, Xerr_cul] = FullprogramScrew(Xstart, Xend, Tf, N, method, dt,...
    Kp, Ki, Blist, Tb0, M0e, gripper, V12, maxVal_joint, maxVal_wheel, minVal_joint, minVal_wheel, Xerr_cul)
% Takes:Xstart: The initial end-effector configuration
%       Xend: The final end-effector configuration
%       Tf: the total time of the motion
%       N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory
%       method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling and 
%                5 indicates quintic (fifth-order polynomial) time scaling
%       dt: timestep
%       Kp: P control gain
%       Ki: I control gain
%       Blist:The joint screw axes in the end-effector frame when the manipulator is at the home position, 
%       Tb0: The fixed offset from the chassis frame {b} to the base frame of the arm {0}
%       M0e: When the arm is at its home configuration, the end-effector frame {e} relative to the arm 
%            base frame {0}
%       gripper : the condition of gripper
%       V12 : a 12-vector representing the current configuration of the robot
%       maxVal_joint: the maximum value of joint speed
%       maxVal_wheel: the maximum value of wheel speed
%       minVal_joint: the minimum value of joint speed
%       minVal_wheel: the minimum value of wheel speed
%       Xerr_cul: the cumulative of Xerr*dt
%
% return: Matrix_error: a matrix of Xerr data
%         Matrix_conf: a matrix of trajectory during the whole process
%         V12: the new V12 after dt
%         Xerr_cul: the new cumulative of Xerr*dt
%
% This function is based on ScrewTrajectory in MR library.
timegap = Tf / (N - 1);
Matrix_conf = [];
Matrix_error = [];
for i = 1: N
    if method == 3
        s1 = CubicTimeScaling(Tf, timegap * (i - 1));%Use CubicTimeScaling to get the path parameter s(t) 
        %and assume s1 = s(t)
        s2 = CubicTimeScaling(Tf, timegap * (i));%assume s2 = s(t) at the next timestep
    else
        s1 = QuinticTimeScaling(Tf, timegap * (i - 1));
        s2 = QuinticTimeScaling(Tf, timegap * (i));
    end
    % calculate the current end-effector reference configuration Xd and next timestep end-effector reference 
    % configuration Xd_next corresponding to the screw motion
    Xd = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s1);
    Xd_next = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s2);
    
    % Calculation of X
    thetalist = [V12(1,4);V12(1,5);V12(1,6);V12(1,7);V12(1,8)];
    % get the configuration of the frame {b} of the mobile base relative to the frame {s} on the floor
    Tsb = [[cos(V12(1,1)), -sin(V12(1,1)), 0, V12(1, 2)];...
        [sin(V12(1,1)), cos(V12(1,1)), 0, V12(1, 3)];...
        [0, 0, 1, 0.0963];...
        [0, 0, 0, 1]];
    T0e = FKinBody(M0e, Blist, thetalist);%get the configuration of the end-factor {e} relative to arm frame 
    X = Tsb*Tb0*T0e;%calculate the current actual end-effector configuration
    
    % generate error matrix
    Xerr_mat = MatrixLog6(TransInv(X)*Xd);
    Xerr = se3ToVec(Xerr_mat);
    Matrix_error = vertcat(Matrix_error, rot90(Xerr));
    [V, Xerr_cul] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_cul);% get end-effector twist V
    
    % calculation of Je
    r = 0.0475;
    L = 0.235;
    w = 0.15;
    F = (r/4)*[[-1/(L+w), 1/(L+w), 1/(L+w), -1/(L+w)];[1, 1, 1, 1];[-1, 1, -1, 1]];
    F6 = [[0,0,0,0];[0,0,0,0];[F(1,:)];[F(2,:)];[F(3,:)];[0,0,0,0]];
    Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6;
    Jb = JacobianBody(Blist, thetalist);
    Je = [Jbase Jb];
    
    % calculation of wheel and arm joint speeds (u, thetadot)
    % assume V9_cur as (u, thetadot) before setting the range of speed
    T = vpa(pinv(Je, 1e-3)*V);
    V9_cur = [T(5,1) T(6,1) T(7,1) T(8,1) T(9,1) T(1,1) T(2,1) T(3,1) T(4,1)];
    % limit the joint speed
    for j = 1:5
        if V9_cur(1,j) < minVal_joint
            V9_cur(1,j) = minVal_joint;
        end
        if V9_cur(1,j) > maxVal_joint
            V9_cur(1,j) = maxVal_joint;
        end
    end
    % limit the wheel speed
    for j = 6:9
        if V9_cur(1,j) < minVal_wheel
            V9_cur(1,j) = minVal_wheel;
        end
        if V9_cur(1,j) > maxVal_wheel
            V9_cur(1,j) = maxVal_wheel;
        end
    end
    V9 = V9_cur;
    % calculate the new V12 by using NextState function
    V12_new = double(NextState(V12, V9, dt));
    offendingJoint = testJointLimits(V12_new);%get the list of offending joint
    for Wrongcolumn = find(offendingJoint)% get the position of column of Je corresponding to an offending joint
        Jb(:,Wrongcolumn)=zeros(1,6);% change the column of Je to all zeros
        Je = [Jbase Jb];%get new Je
        T = vpa(pinv(Je, 1e-3)*V);
        V9 = [T(5,1) T(6,1) T(7,1) T(8,1) T(9,1) T(1,1) T(2,1) T(3,1) T(4,1)];
        V12_new = double(NextState(V12, V9, dt));%obtain the new V12
    end
    
    V12 = V12_new;
    if gripper == 0
        Matrix_conf = vertcat(Matrix_conf, [V12 0]);%generate a matrix of trajectory when gripper is open
    else
        Matrix_conf = vertcat(Matrix_conf, [V12 1]);%generate a matrix of trajectory when gripper is closed
    end
end   
end