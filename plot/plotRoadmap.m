function plotRoadmap(roadmap)
% Plot roadmap.
%
% Inputs:
%  - roadmap       Roadmap of Z rows of [x1 y1 x2 y2].

persistent roadmapPlot1;
persistent roadmapPlot2;

delete(roadmapPlot1);
delete(roadmapPlot2);

activateCurrentPlot();

[x, y] = multiSegmentPlot(roadmap);
color = "#dddddd";
legend = "Roadmap";
lineWidth = 5;

roadmapPlot1 = plot(x(:, 1), y(:,1), ...
    LineWidth = lineWidth, Color = color, HandleVisibility = "on", ...
    DisplayName = legend);

roadmapPlot2 = plot(x(:, 2:end), y(:,2:end), ...
    LineWidth = 5, Color = color, HandleVisibility = "off", ...
    DisplayName = legend);

end
