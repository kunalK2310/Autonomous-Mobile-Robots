function [cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, theta, epsilon)
% Transforms Vx and Vy commands into V and omega commands using feedback
% linearization techniques
%
% Inputs:
%  - cmdVx         Input velocity in x direction wrt inertial frame.
%  - cmdVy         Input velocity in y direction wrt inertial frame.
%  - theta         Orientation of the robot.
%  - epsilon       Turn radius.
%
% Outputs:
%  - cmdV          Fwd velocity.
%  - cmdW          Angular velocity.

cmdW = 1 / epsilon * (-cmdVx * sin(theta) + cmdVy * cos(theta));
cmdV = cmdVx * cos(theta) + cmdVy * sin(theta);

end