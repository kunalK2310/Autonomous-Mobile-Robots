function [wpInd] = findInitialWaypoint(waypoints, pose)
% Find the index of initial waypoint.
%
% Inputs:
%  - waypoints     Waypoints in N-by-2 format.
%  - pose          Initial pose in 1-by-3 format.
%
% Outputs:
%  - wpInd         Index of initial waypoint.

nWaypoints = size(waypoints, 1);
waypointsToInitial = zeros(nWaypoints, 1);

for i = 1:nWaypoints
    waypointsToInitial(i) = norm(pose(1:2) - waypoints(i, :));
end

[~, wpInd] = min(waypointsToInitial);

end
