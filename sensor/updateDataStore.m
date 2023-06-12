function [dataStore] = updateDataStore(Robot, dataStore)
% Update data store.

if ~exist("readTruePose", "var")
    readTruePose = isDebug();
end

try
    % When running with the real robot, we need to define the appropriate
    % ports. This will fail when NOT connected to a physical robot
    CreatePort = Robot.CreatePort;
    DistPort = Robot.DistPort;
    TagPort = Robot.TagPort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
    DistPort = Robot;
    TagPort = Robot;
end

%% Read truth pose (from overhead localization system)
if readTruePose
    try
        [px, py, pt] = OverheadLocalizationCreate(Robot);
        poseX = px;
        poseY = py;
        poseTheta = pt;
        dataStore.truthPose = [dataStore.truthPose; ...
            toc, poseX, poseY, poseTheta];
    catch
        disp("Overhead localization lost the robot!");
    end
end

%% Read odometry distance & angle
try
    deltaD = DistanceSensorRoomba(CreatePort);
    deltaA = AngleSensorRoomba(CreatePort);
    dataStore.odometry = [dataStore.odometry; ...
        toc, deltaD, deltaA];
catch
    disp("Error retrieving or saving odometry data.");
end


%% Read bump data
try
    [BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);
    dataStore.bump = [dataStore.bump; toc, ...
        BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront];
catch
    disp("Error retrieving or saving bump sensor data.");
end


%% Read Real Sense depth data
try

    depth_array = RealSenseDist(Robot);
    dataStore.rsdepth = [dataStore.rsdepth; toc, depth_array'];
catch
    disp("Error retrieving or saving RealSense depth data.");
end

%% Read camera data (beacons)
try
    tags = RealSenseTag(Robot);
    if ~isempty(tags)
        dataStore.beacon = [dataStore.beacon; repmat(toc, size(tags, 1), 1), tags];
    else
        dataStore.beacon = [dataStore.beacon; toc 0 0 0 0 0];
    end
catch
    disp("Error retrieving or saving beacon (AprilTag) data.");
end
