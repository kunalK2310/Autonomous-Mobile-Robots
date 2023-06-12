function [weights, pset] = PF(pset_prev, u_prev, z, Q, dynamicsFun, measureFun)
% PF: Generic Particle Filter
%
%   INPUTS
%       pset_prev    previous particle set [n x 3]
%       u_prev       previous command [d; phi]
%       z            depth measurement vector [1 x k]
%       Q            measurement noise covariance matrix
%       dynamicsFun  Function to calculate the dynamics (g)
%       measureFun   Function to calculate the measurements (h)
%
%   OUTPUTS
%       pset         current particle set [n x 3]
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4

numParticles = size(pset_prev, 1);
pset_bar = zeros(numParticles, 3);
weights = zeros(1,numParticles);

for m = 1:numParticles
    finalPose = dynamicsFun(pset_prev(m,:), u_prev);
    zExpected = measureFun(finalPose)';
    
    % Acount for NaN values
    zFinal = [];
    zExpectedFinal = [];
    index = 1;
    for i = 1:length(z)
        if ~isnan(z(i)) && ~isnan(zExpected(i)) && zExpected(i) ~= 0
            zFinal(index) = z(i);
            zExpectedFinal(index) = zExpected(i);
            index = index + 1;
        end
    end
    QFinal = Q(1:index-1, 1:index-1);
    
    if index ~= 1
        weight = mvnpdf(zExpectedFinal, zFinal, QFinal);
    else
        weight = 0.00001;
    end
    
    weights(1,m) = weight;
    pset_bar(m,:) = finalPose';
end
weights = weights / sum(weights);

pset = zeros(numParticles, 3);
for m = 1:numParticles
    selectedParticle = randsample(numParticles,1,true,weights);
    pset(m,:) = pset_bar(selectedParticle, :);
end

