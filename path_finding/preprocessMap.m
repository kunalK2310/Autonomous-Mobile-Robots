function [mapPoly, roadmap] = preprocessMap(map, sideOffset, roadmapOffset)
% Convert map lines to a roadmap.
%
% Inputs:
%  - map           Map lines with N rows of [x1 y1 x2 y2];
%  - sideOffset    The amount of offset to inflate the lines into polygons.
%  - roadmapOffset The amount of offset to avoid from the edges of
%                  obstacles.
%
% Outputs:
%  - mapPoly       Polygons in N-by-8 format.
%  - roadmap       Roadmap of Z rows of [x1 y1 x2 y2].

mapPoly = lines2obstacles(map, sideOffset);
roadmap = obstacles2roadmap(mapPoly, roadmapOffset);

end
