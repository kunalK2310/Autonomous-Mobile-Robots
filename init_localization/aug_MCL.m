function [pset_curr, beaconSeen] = aug_MCL( ...
    pset_prev, u_prev, z, ...
    beaconData, beaconLoc, beaconSeen, ...
    waypoints, Q, map, ...
    dynamicsFun, measureFun, ...
    wslow, wfast, alpha_slow, alpha_fast)
% Augmented MCL Algorithm
%
% Inputs:
%  - pset_prev     previous particle set [n x 3]
%  - u_prev        control input [d, phi]
%  - z             measurement input [1 x k]
%  - wslow         slow average weight
%  - wfast         fast average weight
%  - alpha_slow    alpha slow weight
%  - alpha_fast    alpha fast weight
%
% Outputs:
%  - pset          current particle set [n x 3]

% Initiate particle set
M = size(pset_prev, 1);
pset_curr = pset_prev;
wavg = 0;
existing_particles = zeros(M, 1); % Check later for correctness

for m = 1:M
    finalPose = dynamicsFun(pset_prev(m, 1:3), u_prev);
    zExpected = measureFun(finalPose)';

    % Account for NaN values
    zFinal = [];
    zExpectedFinal = [];
    index = 1;

    for i = 1:length(z)
        if ~isnan(z(i)) && ~isnan(zExpected(i))
            zFinal(index) = z(i);
            zExpectedFinal(index) = zExpected(i);
            index = index + 1;
        end
    end

    QFinal = Q(1:index - 1, 1:index - 1);
    if index ~= 1
        weight = mvnpdf(zExpectedFinal, zFinal, QFinal);

    else
        weight = 0.00001;
    end

    wavg = wavg + weight / M;
    if rand() < 1 - (wfast / wslow)
        existing_particles(m) = 1;
    end
    pset_curr(m, 1:4) = [finalPose', weight];
end

% Calculate short and long term weights
wslow = wslow + alpha_slow * (wavg - wslow);
wfast = wfast + alpha_fast * (wavg - wfast);

% Update particle set
pset_update = [];

tagID = beaconData(end, 3);

% See if a new beacon has been seen
if tagID ~= 0 && ~any(beaconSeen == tagID) && beaconData(end, 4) <= 1.5
    debug('Beacon spotted', beaconData(end, 3:5));
    beaconSeen = [beaconSeen tagID];

    % Beacon Coordinates
    beaconPos = beaconLoc(beaconLoc(:, 1) == tagID, 2:3);
    x1 = beaconPos(1);
    y1 = beaconPos(2);

    % Draw lines from each waypoint and remove waypoints whose lines
    % intersect with a wall on the map
    noIntersectWaypoints = [];
    for i = 1:size(waypoints, 1)
        x2 = waypoints(i, 1);
        y2 = waypoints(i, 2);

        debug("We are now looking at waypoint", i);
        intersect = false;
        for j = 1:size(map, 1)
            x3 = map(j, 1);
            y3 = map(j, 2);
            x4 = map(j, 3);
            y4 = map(j, 4);
            [isect, ~, ~] = intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4);
            if isect
                debug("Intersecting. Cannot be this one.");
                % Waypoint has no clear view of the beacon
                intersect = true;
                break;
            end
        end

        if ~intersect
            noIntersectWaypoints = [noIntersectWaypoints i];
            debug("This is a good one.");
        end
    end

    eligibleVP = length(noIntersectWaypoints);
    if eligibleVP > 0
        pset_update = [];
        for i=1:eligibleVP
            n = floor(1000 / eligibleVP);
            loc = repmat([waypoints(noIntersectWaypoints(i), :)], n, 1);
            t = linspace(deg2rad(180), deg2rad(-180), n)';
            w = ones(n, 1);
            id = repmat(noIntersectWaypoints(i), n, 1);
            ps = [loc t w id];
            pset_update = [pset_update; ps];
        end
    end
else
    for m = 1:M
        if max(0, (1 - wfast / wslow)) > rand() && toc < 5
            % Inject more particles around top k waypoints with highest
            % weights
            k = min(3, size(waypoints, 1)); % Number of waypoints to choose
            % Sort particles by weight in descending order
            bestWeights = zeros(1, size(waypoints, 1));
            for i = 1:size(waypoints, 1)
                psetForThisWaypoint = find(pset_curr(:,5) == i);
                sortedWeights = sort(psetForThisWaypoint, "descend");
                if size(sortedWeights, 1) > 0
                    bestWeights(i) = sortedWeights(1);
                end
            end
            k = min(k, length(bestWeights(bestWeights > 0)));
            [~, idx] = sort(bestWeights, "descend");
            % Select top k waypoints with highest weights
            topk_waypoints = unique(waypoints(idx(1:k), 1:2), "rows"); 
            % Number of particles to inject around each waypoint
            num_particles_per_waypoint = 2;
            for i = 1:size(topk_waypoints, 1)
                x = topk_waypoints(i, 1);
                y = topk_waypoints(i, 2);
                for j = 1:num_particles_per_waypoint
                    % Random orientation between 0 and 360 degrees
                    theta = deg2rad(rand() * 360 - 180);
                    weight = 1 - wfast / wslow;
                    pset_update = [
                        pset_update;
                        x, y, theta, weight, idx(i)
                    ];
                end
            end
        else
            % Normal random sampling, no new particles added
            selectParticle = randsample(M, 1, true, pset_curr(:, 4));
            pset_update = [pset_update; pset_curr(selectParticle, :)];
        end
    end
end

pset_curr = pset_update;

end