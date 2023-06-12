function [map, optMap, waypoints, ...
    ecWaypoints, combinedWaypoints, beacons, ...
    minX, maxX, minY, maxY] = loadCompetition()
% Load competition map.

MAP_URL = "compMap.mat";

compData = load(MAP_URL);

map = compData.map;
optMap = compData.optWalls;
waypoints = compData.waypoints;
ecWaypoints = compData.ECwaypoints;
combinedWaypoints = [waypoints; ecWaypoints];
beacons = compData.beaconLoc;

minX = min(map(:, [1, 3]), [], "all");
maxX = max(map(:, [1, 3]), [], "all");
minY = min(map(:, [2, 4]), [], "all");
maxY = max(map(:, [2, 4]), [], "all");
for i=1:size(beacons, 1)
    pos = beacons(i, 2:3);
    if pos(1) == minX
        beacons(i, 2) = beacons(i, 2) + 0.01;
    elseif pos(1) == maxX
        beacons(i, 2) = beacons(i, 2) - 0.01;
    elseif pos(2) == minY
        beacons(i, 3) = beacons(i, 3) + 0.01;
    elseif pos(2) == maxY
        beacons(i, 3) = beacons(i, 3) - 0.01;
    end
end

end
