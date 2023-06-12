function [dataStore, readyPose] = initLocalize(Robot, sensorPos)
% Initial localization.

global dataStore;

if ~exist("sensorPos", "var")
    sensorPos = [0, 0.08];
end

%% Constants
NUM_PARTICLES_PER_WAYPOINT = 20;
WSLOW = 0.5;
WFAST = 0.5;
ALPHA_SLOW = 0.001;
ALPHA_FAST = 0.5;

%% Load map
[map, ~, waypoints, ~, ~, beacons] = ...
    loadCompetition();

%% Initialize particles
nWaypoints = size(waypoints, 1);
num_particles = min(1000, nWaypoints * NUM_PARTICLES_PER_WAYPOINT * 3);
% 4 cols: x, y, theta, weight
pset_initial = repmat(-999, num_particles, 5);

for i = 1:nWaypoints
    x = waypoints(i, 1);
    y = waypoints(i, 2);
    for j = 1:NUM_PARTICLES_PER_WAYPOINT
        theta = deg2rad(360 / NUM_PARTICLES_PER_WAYPOINT * j - 180);
        weight = 1;
        pset_initial((i-1) * NUM_PARTICLES_PER_WAYPOINT + j, :) = ...
            [x y theta weight i]; %last column for waypoint number
    end
end

dataStore.initParticles.x = [toc pset_initial(:, 1)'];
dataStore.initParticles.y = [toc pset_initial(:, 2)'];
dataStore.initParticles.theta = [toc pset_initial(:, 3)'];
dataStore.initParticles.weight = [toc pset_initial(:, 4)'];
dataStore.initParticles.wpID =  [toc pset_initial(:, 5)'];

tic   

%% Set up plotting
initPlot(69, "Final Competition - Initial Localization");
plotWalls(map, []);
plotParticles(dataStore.initParticles);
    function updatePlot()
        if isDebug()
            truthPose = dataStore.truthPose(end, 2:4);
            plotRobot([], truthPose);
        else
            plotRobot([], []);
        end
        plotParticles(dataStore.initParticles);
        plotBeacons(beacons);
    end

%% Prepare for augmented MCL
n_rs_rays = 9;
angles = deg2rad(linspace(27, -27, n_rs_rays));
Q = eye(n_rs_rays);
dynamicsFun = @(x,u) integrateOdom(x,u(1),u(2));
measureFun = @(x) depthPredict(x, map, sensorPos, angles');
beaconSeen = [];

while true
    dataStore = updateDataStore(Robot, dataStore);

    odometryData = dataStore.odometry(end,2:3);
    beaconsRead = dataStore.beacon;
    
    depthRead = dataStore.rsdepth(end,3:11);
    depthRead(depthRead < 0.175) = NaN;

    particles = [dataStore.initParticles.x(end, 2:end); ...
         dataStore.initParticles.y(end, 2:end); ...
         dataStore.initParticles.theta(end, 2:end); ...
         dataStore.initParticles.weight(end, 2:end); ...
         dataStore.initParticles.wpID(end, 2:end)];
    particles = particles(:, any(particles > -998))'; % 5-by-N

    % Converged is true iff only one particle is left or particles are on
    % the same spot and only within a limited range of theta.
    if size(particles, 2) == 1
        converged = true;
    else
        converged = ...
            size(unique(particles(:, 1:2), "rows"), 1) == 1 && ...
            max(particles(:, 3)) - min(particles(:, 3)) ...
                < deg2rad(5);
    end

    % Stop init loc if:
    % 1) Converged and more than 25 seconds passed, or
    % 2) Converged, more than 8 seconds passed, and a beacon has been seen
    if (converged && toc >= 25) || ...
        (converged && ~isempty(beaconSeen) && toc >= 8)
        break;
    end

    % Run aug-MCL.
    [pset, beaconSeen] = aug_MCL( ...
         particles, ...
         odometryData, depthRead, ...
         beaconsRead, beacons, beaconSeen, ...
         waypoints, ...
         Q, map, dynamicsFun, measureFun, ...
         WSLOW, WFAST, ALPHA_SLOW, ALPHA_FAST);

    % Convert aug-MCL output back to dataStore particles.
    nP = size(pset, 1);
    nContainer = size(dataStore.initParticles.x, 2) - 1;
    pset = pset(1:min(nP, nContainer), :);
    fill = repmat(-999, 1, max(0, nContainer - nP));
    dataStore.initParticles.x = ...
        [dataStore.initParticles.x; toc pset(:, 1)' fill];
    dataStore.initParticles.y = ...
        [dataStore.initParticles.y; toc pset(:, 2)' fill];
    dataStore.initParticles.theta = ...
        [dataStore.initParticles.theta; toc pset(:, 3)' fill];
    dataStore.initParticles.weight = ...
        [dataStore.initParticles.weight; toc pset(:, 4)' fill];
    dataStore.initParticles.wpID = ...
        [dataStore.initParticles.wpID; toc pset(:, 5)' fill];

    % Update plot.
    updatePlot();

    cmdV = 0;
    cmdW = 0.2; % Tunable parameter
    [sanitizedV, sanitizedW] = limitCmds(cmdV, cmdW, 0.2, 0.13);
        
    SetFwdVelAngVelCreate(Robot, sanitizedV, sanitizedW);
    
    pause(0.1);
end

stopRobot(Robot);

% Converge leftover points to an initial pose.
particles = [dataStore.initParticles.x(end, 2:end); ...
     dataStore.initParticles.y(end, 2:end); ...
     dataStore.initParticles.theta(end, 2:end); ...
     dataStore.initParticles.wpID(end, 2:end)];
particles = particles(:, any(particles > -998))'; % N-by-4
readyPose = [particles(1, 1), particles(1, 2), mean(particles(:, 3))];

updatePlot();

end