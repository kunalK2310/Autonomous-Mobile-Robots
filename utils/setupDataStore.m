function [dataStore] = setupDataStore()
% Set up the data store.
%
% Outputs:
%  - dataStore     Initial data store.

dataStore = struct( ...
    "truthPose", [], ...
    "odometry", [], ...
    "rsdepth", [], ...
    "bump", [], ...
    "beacon", [], ...
    "particles", [], ...
    "initParticles", [], ...
    "estimatedPose", [], ...
    "control", [], ...
    "roadmap", [], ...
    "waypointsOrder", [], ...
    "map", [], ...
    "optMap", [], ...,
    "optMapDecisions", [], ...
    "visitedWaypoints", []);

end
