    function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   Kumar, KUNAL
%Initialize Variables
N = length(d);
finalPose = zeros(3,N+1);
finalPose(:,1) = initPose;

%Loop through the values of d and phi to find:
for i = 1:N
    %if phi=0, robot heading straight in x-axis 
    if phi(i) == 0
        xyR = [d(i),0];
    else
        %x/y in robot frame
        r = (d(i)/phi(i));
        xyR = [r*sin(phi(i)), r*(1-cos(phi(i)))];
    end
    finalPose(1:2,i+1) = robot2global(finalPose(:,i)', xyR)';
    finalPose(3,i+1) = finalPose(3,i) + phi(i);
end
finalPose = finalPose(:,2:end);
end
       
