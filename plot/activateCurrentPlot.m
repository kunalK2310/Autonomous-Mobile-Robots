function [pid] = activateCurrentPlot(plotId)
% Make sure plot is going onto current active plot.

global currentPlot;

if ~exist("plotId", "var")
    plotId = currentPlot;
end

set(0, "CurrentFigure", plotId);
pid = plotId;

end
