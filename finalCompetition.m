function [dataStore] = finalCompetition(Robot, maxTime, offset_x, offset_y)
% finalCompetition is the main entry of the competition.
%
% Inputs:
%  - Robot         The robot instance
%  - maxTime       The maximum time that the robot is allowed to run
%  - offset_x      The offset on X-axis of the robot sensors
%  - offset_y      The offset on Y-axis of the robot sensors
%
% Outputs:
%  - dataStore:    A data structure containing runtime data. Also stored in
%                  global.

%% Welcome message
disp("====================================================");
disp("= FINAL COMPETITION                                =");
disp("= by Leo, Gabe, and Kunal                          =");
disp("= MAY 9, 2023                                      =");
disp("====================================================");

%% Default variables
if ~exist("maxTime", "var")
    maxTime = 7 * 60;
end
if ~exist("offset_x", "var")
    offset_x = 0;
end
if ~exist("offset_y", "var")
    offset_y = 0.08;
end

%% Path set up
addpath("utils/");
addpath("path_finding/");
addpath("plot/");
addpath("runtime_localization/");
addpath("init_localization/");
addpath("motion/");
addpath("sensor/");

%% Constants
% ROBOT SETTINGS
VELOCITY_LIMIT = 0.2;

% MAP PREPROCESSING OFFSET
POLYGON_OFFSET = 0.05;
ROADMAP_OFFSET = 0.3;

% TRAVEL & MOTION CONTROL
CLOSE_ENOUGH = 0.05;
FEEDBACK_LIN_EPSILON = 0.1;

% DEBUG
isDebug(1);
N_METHODS = 2; % Method 1: PF, Method 2: EKF
CHOSEN_METHOD = 2;

% PF SETTINGS
PF_N_PARTICLES = 1000;
PF_STD_DYNAMICS = [0.05; 0.01];
PF_STD_SENSE_PARTICLE = 0.35;
SENSOR_POS = [offset_x, offset_y];

%% Robot setup
try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end
stopRobot(Robot);

%% Set up dataStore
global dataStore;
dataStore = setupDataStore();
pause(0.5);
dataStore = updateDataStore(Robot, dataStore);

%% Load map
[map, optMap, waypoints, ~, combinedWaypoints, beacons, ...
    minX, maxX, minY, maxY] = ...
    loadCompetition();
nWaypoints = size(combinedWaypoints, 1);
dataStore.map = map;
dataStore.optMap = optMap;

%% Convert map to polygons and roadmap
[mapPoly, roadmap] = preprocessMap(map, POLYGON_OFFSET, ROADMAP_OFFSET);
dataStore.roadmap = roadmap;

%% Initial position
[dataStore, currentLoc] = initLocalize(Robot, SENSOR_POS);
methods = repmat(currentLoc, N_METHODS, 1);
if isDebug() && size(dataStore.truthPose, 1) >= 1
    debug("Truth pose is: ", dataStore.truthPose(end, 2:4));
    debug("Initial pose is: ", currentLoc);
    debug("Off by: ", currentLoc - dataStore.truthPose(end, 2:4));
end

%% Figure out which waypoint is initial.
initialWaypointInd = findInitialWaypoint(waypoints, currentLoc);

%% Do weight matrix
weightMat = genWeightMatrix( ...
    combinedWaypoints, roadmap, mapPoly, ROADMAP_OFFSET);

%% Set up waypoints
[order, waypointsInOrder] = ...
    rankWaypoints(combinedWaypoints, weightMat, initialWaypointInd);
dataStore.waypointsOrder = order;
waypointsState = zeros(1, nWaypoints);
waypointsState(1) = 2;

%% Set up path finding
path = [];
    function updatePath()
        path = findPath( ...
            roadmap, ...
            mapPoly, ...
            waypointsInOrder(order_curr + 1, :), ...
            waypointsInOrder(order_curr, :), ...
            ROADMAP_OFFSET);
    end

%% Prepare for travel
order_curr = 1;
path_curr = 1;
updatePath();

%% Set up plotting
initPlot();
    function plotAll()
        plotRoadmap(roadmap);
        plotWalls(map, optMap);
        plotTrajectory(dataStore.estimatedPose);
        plotWaypoints(waypointsInOrder, waypointsState);
        plotBeacons(beacons);
        if order_curr < nWaypoints
            plotPath(path, path_curr);
        else
            plotPath([]);
        end
        if size(dataStore.truthPose, 1) > 0
            plotRobot(methods, dataStore.truthPose(end, 2:4), ...
                CHOSEN_METHOD, [ ...
                    "PF" ...
                    "EKF" ...
                ], ...
                [minX maxX minY maxY]);
        else
            plotRobot(methods, [], ...
                CHOSEN_METHOD, [ ...
                    "PF" ...
                    "EKF" ...
                ], ...
                [minX maxX minY maxY]);
        end
    end
plotAll();

%% Debug pause
if isDebug()
    pause(1);
end

%% Shared code for all methods
n_rs_rays = 9;
angles = deg2rad(linspace(27, -27, n_rs_rays))';
    function [expected] = beaconFunc(pose, beaconXY)
        intersect = false;
        for ij=1:size(map, 1)
            [it, ~, ~] = intersectPoint(pose(1), pose(2), ...
                beaconXY(1), beaconXY(2), ...
                map(ij, 1), map(ij, 3), ...
                map(ij, 2), map(ij, 4));
            if it
                intersect = true;
                break;
            end
        end
        if intersect
            expected = [9999 9999];
        else
            expected = global2robot( ...
                [robot2global(pose, SENSOR_POS), pose(3)], beaconXY);
        end
    end

%% Method 1 Setup: PF
if isDebug() || CHOSEN_METHOD == 1
    dynamic_func = @(x, y, z) integrateOdom(x, y, z);
    depth_func = @(x, y) depthPredict(x, y, SENSOR_POS, angles);
    optCounter = [zeros(size(optMap, 1), 1), (1:(size(optMap, 1)))'];
end

%% Method 2 Setup: EKF
if isDebug() || CHOSEN_METHOD == 2
    mu = currentLoc';
    sigma = [0.1 0 0; 0 0.1 0; 0 0 0.1];
    R = [
        PF_STD_DYNAMICS(1) 0 0;
        0 PF_STD_DYNAMICS(1) 0;
        0 0 PF_STD_DYNAMICS(2);
    ];
    Q_depth = PF_STD_SENSE_PARTICLE*eye(n_rs_rays); 
    Hfunction = @(x) HjacDepth(x,map,SENSOR_POS,n_rs_rays);
    Gfunction = @(x,u) GjacDiffDrive(x,u);
    hfunction = @(x) depthPredict(x,map,SENSOR_POS,angles);
    gfunction = @(p,u1,u2) integrateOdom(p,u1,u2);
end

tic
while toc < maxTime

    %% Updata data store
    dataStore = updateDataStore(Robot, dataStore);
    odom = dataStore.odometry(end, 2:3);
    depths = dataStore.rsdepth(end, 3:11)';
    beaconRead = dataStore.beacon(end, 2:6);

    %% Method 1: PF
    if isDebug() || CHOSEN_METHOD == 1
        particles = repmat(methods(1, :)', 1, PF_N_PARTICLES);
        [particles, pose, map, optMap, optCounter, mapChanges] = PF3( ...
            particles, odom, depths, ...
            PF_STD_DYNAMICS, PF_STD_SENSE_PARTICLE, ...
            map, optMap, optCounter, dynamic_func, depth_func);
        zipped = particles(:)';
        methods(1, :) = pose';
        dataStore.particles = [dataStore.particles; toc, zipped];
    end

    %% Method 2: EKF
    if isDebug() || CHOSEN_METHOD == 2
        [mu,sigma] = EKF( ...
            mu, odom', sigma, ...
            R, depths, Q_depth, ...
            Hfunction, Gfunction, hfunction, gfunction ...
        );
        methods(2, :) = mu';
    end

    %% Set current location.
    currentLoc = methods(CHOSEN_METHOD, :);
    dataStore.estimatedPose = [dataStore.estimatedPose; toc, currentLoc];

    %% If there is a map change, adjust.
%     if mapChanges
%         dataStore.optMapDecisions = [...
%             dataStore.optMapDecisions;
%             repmat(toc, size(mapChanges, 1), 1) mapChanges
%         ];
%         if any(mapChanges(:, 2))
%             [mapPoly, roadmap] = preprocessMap( ...
%                 map, POLYGON_OFFSET, ROADMAP_OFFSET);
%             dataStore.roadmap = roadmap;
%             updatePath();
%             order_curr = 1;
%         end
%     end

    %% Update plotting
    plotAll();

    %% Set start and end points
    p1 = currentLoc(1:2);
    p2 = path(path_curr+1, :);

    %% Check proximity to next path node
    if sqrt((p2(1) - p1(1))^2+(p2(2) - p1(2))^2) <= CLOSE_ENOUGH
        path_curr = path_curr + 1;
    end

    %% Check reaching waypoint
    if path_curr == size(path, 1)
        puts("WAYPOINT REACHED");
        BeepCreate(Robot);
        waypointsState(order_curr + 1) = 2;
        dataStore.visitedWaypoints = [
            dataStore.visitedWaypoints; ...
            toc waypointsInOrder(order_curr + 1, 1:2)
        ];
        if isDebug()
            truthPoseXY = dataStore.truthPose(end, 2:3);
            dist = norm(truthPoseXY-waypointsInOrder(order_curr + 1, 1:2));
            debug(" - Order current:", order_curr);
            puts(" - Actual pose:", truthPoseXY);
            puts(" - Waypoint:", waypointsInOrder(order_curr + 1, 1:2));
            puts(" - Distance to actual waypoint:", dist);
            if dist > .2
                puts(" - !!! BAD DETECTION");
                waypointsState(order_curr+1) = 1;
            else
                puts(" -:) GOOD DETECTION");
            end
        end
        order_curr = order_curr + 1;
        if order_curr == nWaypoints
            puts("ALL WAYPOINTS VISITED");
            break;
        end
        path_curr = 1;
        updatePath();
    end

    %% Send motion
    local = global2robot(currentLoc, p2);
    localX = local(1);
    localY = local(2);
    [cmdV, cmdW] = feedbackLin(localX, localY, 0, FEEDBACK_LIN_EPSILON);
    [sanitizedV, sanitizedW] = limitCmds(cmdV, cmdW, VELOCITY_LIMIT, 0.2);
    dataStore.control = [dataStore.control; toc, sanitizedV, sanitizedW];
    SetFwdVelAngVelCreate(Robot, sanitizedV, sanitizedW);

    pause(0.3);
end

%% Stop robot after loop.
plotAll();
stopRobot(Robot);

end
