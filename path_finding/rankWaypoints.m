function [sortedInd, waypointsInOrder] = rankWaypoints( ...
    combinedWaypoints, weightMat, initial)
% Rank waypoints using nearest neighbor algorithm.
%
% Inputs:
%  - combinedWaypoints All waypoints (EC and regular) in N-by-2 format.
%  - weightMat     Weight matrix in N-by-N format.
%  - initial       Index of initial waypoint.
%
% Outputs:
%  - sortedInd     A row vector of waypoint indices sorted from first to
%                  last. Will include first and final waypoint indices.
%  - waypointsInOrder The list of waypoints but in the ranked order.

nWaypoints = size(combinedWaypoints, 1);
sortedInd = zeros(1, nWaypoints);
sortedInd(1) = initial;

curr = initial;
for i = 2:nWaypoints
    [~, sortInd] = sort(weightMat(curr, :));
    nextCurr = sortInd(1);
    weightMat(curr, :) = inf;
    weightMat(:, curr) = inf;
    sortedInd(i) = nextCurr;
    curr = nextCurr;
end

waypointsInOrder = combinedWaypoints(sortedInd, :);

end
