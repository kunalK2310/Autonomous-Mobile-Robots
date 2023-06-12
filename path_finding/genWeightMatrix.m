function [weightMat] = genWeightMatrix( ...
    combinedWaypoints, roadmap, mapPoly, roadmapOffset)
% Generate weight matrix between every pair of waypoints.
%
% Inputs:
%  - combinedWaypoints All waypoints (EC and regular) in N-by-2 format.
%  - mapPoly       Polygons in N-by-8 format.
%  - roadmap       Roadmap of Z rows of [x1 y1 x2 y2].
%  - roadmapOffset The amount of offset to avoid from the edges of
%                  obstacles.
%
% Outputs:
%  - weightMat     Weight matrix in N-by-N format.

waypoints = combinedWaypoints;
nWaypoints = size(waypoints, 1);

weightMat = zeros(nWaypoints, nWaypoints);
for i = 1:nWaypoints
    for j = 1:nWaypoints
        if i == j
            weightMat(i, j) = inf;
            continue;
        elseif i > j
            continue;
        end

        bestPath = findPath( ...
            roadmap, ...
            mapPoly, ...
            waypoints(j, :), ...
            waypoints(i, :), ...
            roadmapOffset);

        n = size(bestPath, 1);
        d = 0;
        for i2 = 1:(n - 1)
            d = d + norm(bestPath(i2 + 1, :) - bestPath(i2, :));
        end

        weightMat(i, j) = d;
        weightMat(j, i) = d;
    end
end
end
