function [xyR] = global2robot(pose, xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
%
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
%
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
%
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
%
%
%   Cornell University
%   Autonomous Mobile Robots

x_rotated = xyG(1) - pose(1);
y_rotated = xyG(2) - pose(2);
s = sin(-pose(3));
c = cos(-pose(3));
x_unrotated = x_rotated * c - y_rotated * s;
y_unrotated = x_rotated * s + y_rotated * c;
xyR(1) = x_unrotated;
xyR(2) = y_unrotated;
end
