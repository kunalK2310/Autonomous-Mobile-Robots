function [x, y] = multiSegmentPlot(segments)
% A helper function that converts line segments to plot-friendly format.
%
% Inputs:
%  - segments      N rows of [x1 y1 x2 y2].
%
% Outputs:
%  - x             To be provided to plot as X list.
%  - y             To be provided to plot as Y list.

if isempty(segments)
    x = [];
    y = [];
    return;
end
x = [segments(:, 1)'; segments(:, 3)'];
y = [segments(:, 2)'; segments(:, 4)'];

end
