function plotWalls(map, optMap)
% Plot walls.
%
% Inputs:
%  - map
%  - optMap

persistent mapPlot1;
persistent mapPlot2;
persistent optMapPlot1;
persistent optMapPlot2;
persistent lastPlot;

plotId = activateCurrentPlot();

if lastPlot == plotId
    delete(mapPlot1);
    delete(mapPlot2);
    delete(optMapPlot1);
    delete(optMapPlot2);
else
    lastPlot = plotId;
end

legend = "Walls";
legendOpt = "Optional Walls";
lineWidth = 1;

[x, y] = multiSegmentPlot(map);
mapPlot1 = plot(x(:, 1), y(:, 1), LineWidth = lineWidth, ...
    Color = "black", DisplayName = legend);
mapPlot2 = plot(x(:, 2:end), y(:, 2:end), LineWidth = lineWidth, ...
    Color = "black", DisplayName = legend, HandleVisibility = "off");

if ~isempty(optMap)
    [x, y] = multiSegmentPlot(optMap);
    optMapPlot1 = plot(x(:, 1), y(:, 1), LineWidth = lineWidth, ...
        Color = "red", DisplayName = legendOpt);
    optMapPlot2 = plot(x(:, 2:end), y(:, 2:end), LineWidth = lineWidth, ...
        Color = "red", DisplayName = legendOpt, HandleVisibility = "off");
end

end
