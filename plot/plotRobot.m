function plotRobot(methods, truthPose, chosen, labels, bounds)
% Plot robot.

persistent robotPlots;
persistent robotDirPlots;
persistent robotPlot2;
persistent robotDirPlot2;

legend2 = "Truth Pose";
markerSize = 15;
lineWidth = 3;
dirLineWidth = 2;
dirLength = .3;

delete(robotPlots);
delete(robotDirPlots);
delete(robotPlot2);
delete(robotDirPlot2);

COLORS = [
    "#FF0000" ... % Slate
    "#F48FB1" ... % Magenta
    "#C5E1A5" ... % Green
    "#B2EBF2" ... % Tiffany Blue
];

if exist("truthPose", "var") && ~isempty(truthPose)
    [robotPlot2, robotDirPlot2] = subPlotRobot( ...
        truthPose, COLORS(1), legend2, ...
        markerSize, lineWidth, dirLineWidth, dirLength);
end

for i=1:size(methods, 1)
    pose = methods(i, :);
    if pose(1) < bounds(1) || pose(1) > bounds(2) || ...
        pose(2) < bounds(3) || pose(2) > bounds(4)
        continue;
    end
    if i == chosen
        c = "black";
        l = strcat("Pose (CHOSEN, ", labels(i) ,")");
    else
        c = COLORS(i + 1);
        l = strcat("Pose (", labels(i) ,")");
    end
    [p, p2] = subPlotRobot( ...
        pose, c, l, ...
        markerSize, lineWidth, dirLineWidth, dirLength);
    robotPlots = [robotPlots; p];
    robotDirPlots = [robotDirPlots; p2];
end

end


function [p, dirplot] = subPlotRobot(cl, color, legend, ...
    markerSize, lineWidth, dirLineWidth, dirLength)
    lex = cl(1) + dirLength * cos(cl(3));
    ley = cl(2) + dirLength * sin(cl(3));
    activateCurrentPlot();
    p = plot(cl(1), cl(2), ...
        "o", Color = color, ...
        DisplayName = legend, ...
        MarkerSize = markerSize, LineWidth = lineWidth);
    dirplot = plot([cl(1), lex], [cl(2), ley], ...
        "-", LineWidth = dirLineWidth, Color = color, ...
        HandleVisibility = "off");
end
