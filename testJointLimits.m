function offendingJoint = testJointLimits(V12_new)
% Takes:V12_new: the new V12
% Returns: the list of offending joint matrix.
% for example:
% If joint 4 exceed the joint limit
% offendingJoint = [0 0 0 1 0];
% assume Wrongcolumn = find(offendingJoint)
% Then Wrongcolumn = 4.
% The fourth column of Je will be changed to all zeros.
%
% Example Input:
% 
% clear; clc;
% V12_new = [0, 0, 0, 0, -1.5, 0, 0, 0, 0, 0, 0, 0]
% 
% Output: 
% offendingJoint = [0 1 0 0 0]
a = 0;
b = 0;
c = 0;
d = 0;
e = 0;
% limit joint 2 > 0.5 and < -1.116
% According to Scene3:Interactive youBot, the range of joint 2 is [-1.117,1.553]
% So, joint2 must larger than 1.116.
if V12_new(1,5) > 0.5 || V12_new(1,5) < -1.116
    b = 1;%If joint 2 exceed the limit, it will change from 0 to 1.
end
% limit joint 3 < -2.5
if V12_new(1,6) < -2.5
    c = 1;%If joint 3 exceed the limit, it will change from 0 to 1.
end
% limit joint 4 > 0.8
if V12_new(1,7) > 0.8
    d = 1;%If joint 4 exceed the limit, it will change from 0 to 1.
end
% If any joint exceed the limit, the column of this matrix will change from
% 0 to 1.
% By using "find" function, the column of Je which need to be zeros will be found.
offendingJoint = double([a b c d e]);
end
