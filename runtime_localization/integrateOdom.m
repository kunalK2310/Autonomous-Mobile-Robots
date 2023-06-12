function [finalPose] = integrateOdom(initPose, d, phi)
% Calculate the robot pose in the initial frame based on the odometry.
%
% Inputs:
%  - initPose      Robot's initial pose [x y theta] (3-by-1)
%  - d             Distance vectors returned by DistanceSensorRoomba 
%                  (1-by-N)
%  - phi           Angle vectors returned by AngleSensorRoomba (1-by-N)
%
% Outputs:
%  - finalPose     The final pose of the robot in the inertial frame
%                  (3-by-N)

N = length(d);

finalPose = zeros(3, N);
xo = initPose(1);
yo = initPose(2);
thetao = initPose(3, :);
phio = phi(1);
do = d(1);

if phio ~= 0
    theta1 = thetao + phio;
    x1 = xo + do / phio * (sin(thetao+phio) - sin(thetao));
    y1 = yo + do / phio * (cos(thetao) - cos(thetao+phio));
else
    theta1 = thetao;
    x1 = xo + do * cos(thetao);
    y1 = yo + do * sin(thetao);
end

finalPose(:, 1) = [x1; y1; theta1];

for i = 2:N
    phi_i = phi(i);
    xi = finalPose(1, i-1);
    yi = finalPose(2, i-1);
    thetai = finalPose(3, i-1);
    di = d(i);

    if phi_i ~= 0
        theta_i1 = thetai + phi_i;
        xi1 = xi + di / phi_i * (sin(thetai+phi_i) - sin(thetai));
        yi1 = yi + di / phi_i * (cos(thetai) - cos(thetai+phi_i));
    else
        theta_i1 = thetai;
        xi1 = xi + di * cos(thetai);
        yi1 = yi + di * sin(thetai);
    end
    
    finalPose(:, i) = [xi1; yi1; theta_i1];

end

end