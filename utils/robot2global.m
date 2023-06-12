function [xyG] = robot2global(pose, xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
%
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
%
%   INPUTS
%       pose    robot's current pose [x y theta(radians)]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
%
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
robot_x = pose(1);
robot_y = pose(2);
theta = pose(3);

%R_IB = [cos(theta) -sin(theta) 0
%       sin(theta)  cos(theta) 0
%             0           0     1];

R_IB = [cos(theta), -sin(theta); ...
    sin(theta), cos(theta)];


T_IB = [R_IB, [robot_x; robot_y]; 0, 0, 1];


xyG = T_IB * ([xyR, 1].');
xyG = xyG(1:2).';

end