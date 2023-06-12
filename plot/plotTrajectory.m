function plotTrajectory(allEstimatedPose)
% Plot trajectory.
%
% Inputs:
%  - allEstimatedPose

persistent traj1;
persistent traj2;

delete(traj1);
delete(traj2);

legend = "Trajectory";
lineWidth = 2;
color = "#EC407A";

if size(allEstimatedPose, 1) <= 2
    return;
end

traj1 = plot( ...
    allEstimatedPose(1:2, 2)', allEstimatedPose(1:2, 3)', '-', ...
    LineWidth = lineWidth, ...
    Color = color, DisplayName = legend ...
);
traj2 = plot( ...
    allEstimatedPose(2:end, 2)', allEstimatedPose(2:end, 3)', '-', ...
    LineWidth = lineWidth, ...
    Color = color, HandleVisibility = "off" ...
);

end
