function H = HjacDepth(x, map, sensor_pos, K)
% HjacDepth: output the jacobian of the depth measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       map          of environment, n x [x1 y1 x2 y2] walls
%       sensor_pos   sensor position in the body frame [1x2]
%       K            number of measurements (rays) the sensor gets between 27 to -27 
%
%   OUTPUTS
%       H            Kx3 jacobian matrix
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Koblov, Slava

angles = linspace(27*(pi/180),-27*(pi/180),K);

depth = depthPredict(x, map, sensor_pos, angles');

diff = 0.0001;

xdiff = [x(1,1)+diff;x(2,1);x(3,1)];
ydiff = [x(1,1);x(2,1)+diff;x(3,1)];
thetadiff = [x(1,1);x(2,1);x(3,1)+diff];

depthx = depthPredict(xdiff,map, sensor_pos, angles');
depthy = depthPredict(ydiff,map, sensor_pos, angles');
depththeta = depthPredict(thetadiff,map, sensor_pos, angles');

H = zeros(K,3);

H(:,1) = (depthx-depth)/(diff);
H(:,2) = (depthy-depth)/(diff);
H(:,3) = (depththeta-depth)/(diff);

function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2

function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
% 
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Koblov, Slava

xpos = pose(1);
ypos = pose(2);
theta = pose(3);

Tib = [cos(theta) -sin(theta) xpos; sin(theta) cos(theta) ypos; 0 0 1];

coordglobal = Tib*[xyR(1); xyR(2); 1];

xyG = [coordglobal(1,1) coordglobal(2,1)];

end

function[xyR] = global2robot(pose,xyG)
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
%   Homework #1
%   Koblov, Slava

xpos = pose(1);
ypos = pose(2);
theta = pose(3);

Tbi = [cos(theta) sin(theta) (-xpos*cos(theta)-ypos*sin(theta)); -sin(theta)...
    cos(theta) (xpos*sin(theta)-ypos*cos(theta)); 0 0 1];

robotcoord = Tbi*[xyG(1); xyG(2); 1];

xyR = [robotcoord(1,1) robotcoord(2,1)];
end

[sensorGlobal] = robot2global(robotPose', sensorOrigin);
sensorGx = sensorGlobal(1);
sensorGy = sensorGlobal(2);
[mapRows, mapColoumns] = size(map);
[angleRows, angleColoumns] = size(angles);
maxR = 10000000000;
depth = [];

for i = 1:angleRows
    depths = [];
    index = 0;
    for j = 1:mapRows
        [isect,xx,y]= intersectPoint(sensorGx, sensorGy, ...
            sensorGx+(maxR*cos(robotPose(3,1)+angles(i,1))), ...
            sensorGy+(maxR*sin(robotPose(3,1)+angles(i,1))),...
            map(j,1), map(j,2), map(j,3), map(j,4));
        if isect == true
%             xy = global2robot([sensorGx sensorGy robotPose(3,1)], [x y]);
%             x1 = xy(1);
%             y1 = xy(2);
            magnitude = sqrt(((xx-sensorGx)^2)+((y-sensorGy)^2));
            index = index+1;
            depths(index) = magnitude*cos(angles(i,1));
        end
    end
    if depths ~= 0
        m = min(depths);
        depth(i,1) = m;
    else
        depth(i,1) = 0;
    end
end
% temp = depth';
% temp2 = fliplr(temp);
% depth = temp2';
end
end