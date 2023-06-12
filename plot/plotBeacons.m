function plotBeacons(beaconLoc)
% Plot beacons.

persistent beacons;
persistent texts;
persistent lastPlot;

plotId = activateCurrentPlot();

if lastPlot == plotId
    delete(beacons);
    delete(texts);
else
    lastPlot = plotId;
end

legend = "Beacons";
circleSize = 30;

beacons = ...
    scatter(beaconLoc(:, 2), beaconLoc(:, 3), ...
    circleSize, 'o', DisplayName=legend, MarkerEdgeColor="black");
for i=1:size(beaconLoc, 1)
    texts = [
        texts;
        text(beaconLoc(i, 2) + 0.1, beaconLoc(i, 3), ...
            num2str(beaconLoc(i, 1)))
    ];
end

end
