function stopRobot(Robot)
% Immediately stop the robot.
%
% Inputs:
%  - Robot         The robot instance.

SetFwdVelAngVelCreate(Robot, 0, 0);

end