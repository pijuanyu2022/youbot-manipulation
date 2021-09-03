function V12_new = NextState(V12, V9, dt)
% Takes: V12:a 12-vector representing the current configuration of the robot
%        V9:a 9-vector of wheel and arm joint speeds (u, thetadot)
%        dt:timestep
% Return: V12_new:a new value of V12 at a time dt later
%
% Example Input:
% 
% clear; clc;
% 
% V12 = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0]
% V9 = [1, 1, 1, 1, 1, 10, 10, 10, 10]
% dt = 0.01
%
% Output:
% V12_new = [0, 0.0047, 0, 0.1, 0.1, 0.21, -1.59, 0.01, 0.1, 0.1, 0.1, 0.1]
theta1 = V12(1,4) + V9(1,1)*dt;%calculate new arm joint angles
theta2 = V12(1,5) + V9(1,2)*dt;
theta3 = V12(1,6) + V9(1,3)*dt;
theta4 = V12(1,7) + V9(1,4)*dt;
theta5 = V12(1,8) + V9(1,5)*dt;
W1 = V12(1,9) + V9(1,6)*dt;%calculate new wheel angles
W2 = V12(1,10) + V9(1,7)*dt;
W3 = V12(1,11) + V9(1,8)*dt;
W4 = V12(1,12) + V9(1,9)*dt;

dtheta = [V9(1,6); V9(1,7); V9(1,8); V9(1,9)];%get the wheel speeds u
r = 0.0475;%radius of each wheel
L = 0.235;%the half value of the forward-backward distance between the wheels
w = 0.15;%the half value of the  side-to-side distance between the wheels
F = (r/4)*[[-1/(L+w), 1/(L+w), 1/(L+w), -1/(L+w)];[1, 1, 1, 1];[-1, 1, -1, 1]];% F is  the pseudoinverse of H(0)
Vb = F*dtheta;%calculate the body twist Vb
phi = V12(1,1) + Vb(1,1)*dt;%calculate the new phi, x, and y
x = V12(1,2) + Vb(2,1)*dt;
y = V12(1,3) + Vb(3,1)*dt;
V12_new = [phi, x, y, theta1, theta2, theta3, theta4, theta5, W1, W2, W3, W4];% get a new V12
end