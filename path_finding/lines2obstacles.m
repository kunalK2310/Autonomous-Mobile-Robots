function [map_obstacles] = lines2obstacles(map_lines, offset)
% Convert map lines to a series of obstacles.
%
% Inputs:
%  - map_lines     Map lines with N rows of [x1 y1 x2 y2];
%  - offset        The amount of offset to inflate the lines into polygons.
%
% Outputs:
%  - map_obstacles Polygons in N-by-8 format.

[n_lines, ~] = size(map_lines);
map_obstacles = zeros(n_lines, 8);

for i = 1:n_lines
    line_i = map_lines(i, :);
    line_pt1 = line_i(1:2);
    line_pt2 = line_i(3:4);

    xi1 = line_pt1(1);
    xi2 = line_pt2(1);

    if xi2 > xi1
        right_pt = line_pt2;
        left_pt = line_pt1;
    elseif xi2 < xi1
        right_pt = line_pt1;
        left_pt = line_pt2;
    else
        yi1 = line_pt1(2);
        yi2 = line_pt2(2);

        if yi2 > yi1
            right_pt = line_pt2;
            left_pt = line_pt1;
        else
            right_pt = line_pt1;
            left_pt = line_pt2;
        end
    end

    slope = (right_pt(2) - left_pt(2)) / (right_pt(1) - left_pt(1));

    if slope == 0
        offset_slope = -Inf;
    else
        offset_slope = -1 / slope;
    end

    angle1 = atan2(offset_slope, 1);
    new_point1 = offset_point(left_pt, angle1, offset);

    angle2 = angle1;
    new_point2 = offset_point(right_pt, angle2, offset);

    angle3 = angle1 + pi;
    new_point3 = offset_point(right_pt, angle3, offset);

    angle4 = angle3;
    new_point4 = offset_point(left_pt, angle4, offset);

    if slope >= 0
        map_obstacles(i, :) = [new_point1, new_point2, new_point3, new_point4];
    else
        map_obstacles(i, :) = [new_point4, new_point3, new_point2, new_point1];
    end

end

end


function new_point = offset_point(origin, angle, offset)
new_point(1) = origin(1) + offset * cos(angle);
new_point(2) = origin(2) + offset * sin(angle);
end
