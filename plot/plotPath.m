function plotPath(path, pathCurr)
% Plot path to next waypoint.
%
% Inputs:
%  - path          Path to next waypoint.
%  - pathCurr      Current (visited) node on the path.
%  - pose          Current pose.

persistent np11;
persistent np12;
persistent np2;

delete(np11);
delete(np12);
delete(np2);

activateCurrentPlot();

if isempty(path)
    return;
end

legend1 = "Path to next waypoint";
legend2 = "Next node";
color1 = "#7E57C2";

next = path(pathCurr + 1, :);
np11 = plot([path(1, 1) path(2, 1)], [path(1, 2) path(2, 2)], ...
    "-", Color = color1, LineWidth = 2, ...
    DisplayName = legend1);
np12 = plot(path(2:end, 1), path(2:end, 2), ...
    "-", Color = color1, LineWidth = 2, ...
    DisplayName = legend1, HandleVisibility = "off");
np2 = scatter(next(1), next(2), 120, ...
    LineWidth = 2, DisplayName = legend2);

end