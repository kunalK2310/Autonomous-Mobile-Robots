function [mu, sigma] = EKF(mu_prev, u_prev, sigma_prev, R, z, Q, ...
    Hfunction, Gfunction, hfunction,g)
%   INPUTS
%       mu_prev           previous vector of pose state (mu(t-1))
%       u_prev           previous command [d; phi]
%       sigma_prev        previous covariance matrix
%       R            state model noise covariance matrix
%       z            Measurement
%       Q            measurement noise covariance matrix
%       map          map of the environment
%       sensor_pos   sensor position in the robot frame [x y]
%       n_rs_rays    number of evenly distributed realsense depth rays
%       (27...-27) degrees

muBar = g(mu_prev,u_prev(1,1),u_prev(2,1));

G = Gfunction(mu_prev,u_prev);
sigmaBar = G*sigma_prev*G'+R;

H = Hfunction(muBar);
h = hfunction(muBar);

[rr, ~] = size(z);
sliceIndex = 0;
for ii = 1:rr
    if isnan(z(ii-sliceIndex,1))
        z(ii-sliceIndex,:) = [];
        h(ii-sliceIndex,:) = [];
        H(ii-sliceIndex,:) = [];
        Q(ii-sliceIndex,:) = [];
        Q(:,ii-sliceIndex) = [];
        sliceIndex = sliceIndex+1;
    elseif abs(z(ii-sliceIndex,1)-h(ii-sliceIndex,1)) >= 0.3
        z(ii-sliceIndex,:) = [];
        h(ii-sliceIndex,:) = [];
        H(ii-sliceIndex,:) = [];
        Q(ii-sliceIndex,:) = [];
        Q(:,ii-sliceIndex) = [];
        sliceIndex = sliceIndex+1;
    end
end


K = sigmaBar*H'*(H*sigmaBar*H'+Q)^-1;
[rows,columns] = size(K);


mu = muBar+K*(z-h);
sigma = (eye(rows)-K*H)*sigmaBar;

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
% Koblov, Slava

Xout = initPose(1,1);
Yout = initPose(2,1);
Thetaout = initPose(3,1);

finalPose = zeros(3,length(d));

for i = 1:length(d)
    if phi(i) ~= 0
        RR = d(i)/phi(i);
        dx = 2*RR*sin(phi(i)/2)*cos(Thetaout+(phi(i)/2));
        Xout = Xout + dx;
        dy = 2*RR*sin(phi(i)/2)*sin(Thetaout+(phi(i)/2));
        Yout = Yout+dy;
        Thetaout = Thetaout+phi(i);
    else 
        Thetaout = Thetaout;
        Xout = Xout + d(i)*cos(Thetaout);
        Yout = Yout + d(i)*sin(Thetaout);
    end
    finalPose(1,i) = Xout;
    finalPose(2,i) = Yout;
    finalPose(3,i) = Thetaout;
end
end
end