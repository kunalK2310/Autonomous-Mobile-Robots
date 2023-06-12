function initPlot(fgId, titleStr)
% Initialize the plot.

global currentPlot;

if ~exist("fgId", "var")
    fgId = 42;
end
if ~exist("titleStr", "var")
    titleStr = "Final Competition";
end

try
    close(fgId);
catch
end
fig = figure(fgId);
currentPlot = fig;
activateCurrentPlot(fgId);
hold on
title(titleStr);
xlabel("X(m)");
ylabel("Y(m)");
legend(Location="bestoutside");

end
