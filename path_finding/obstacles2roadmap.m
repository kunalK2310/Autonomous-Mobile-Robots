function [roadmap3] = obstacles2roadmap(obstacles, side_offset)
% Create a roadmap from a list of obstacles.
%
% Inputs:
%  - obstacles     Obstacles in N-by-8 format.
%  - side_offset   The amount of offset to avoid from the edges of
%                  obstacles.
%
% Outputs:
%  - roadmap3      Roadmap of Z rows of [x1 y1 x2 y2].

[n_obstacles, ~] = size(obstacles);

obstacles2 = obstacle_offset(obstacles, side_offset);
obstacles3 = obstacles_return(obstacles2);

pot_vert_list = [];

% Iterate thru all vertices on all offset obstacles
% Add to potential vertices list IF they do not lie within another obstacle
for i = 1:n_obstacles
    obstacle_i = obstacles3(i, :);

    for j = 1:4 % All vertices on this obstacle
        vertex_j = obstacle_i((2 * j - 1):(2 * j));

        obstacles2checkin = obstacles3(all((obstacles3 ~= obstacle_i), 2), :);

        in22 = obstacle_with_in_finder(vertex_j, obstacles2checkin);
        if (~in22)
            pot_vert_list = [pot_vert_list; vertex_j];
        end
    end
end

[n_pot_ver, ~] = size(pot_vert_list);

roadmap = [];

obstacles4 = obstacle_offset(obstacles, side_offset*0.9);
obstacles5 = obstacles_return(obstacles4);

for i = 1:n_pot_ver
    pot_vert1 = pot_vert_list(i, :);

    for j = i + 1:n_pot_ver
        pot_vert2 = pot_vert_list(j, :);

        edgeij = [pot_vert1, pot_vert2];


        isect = obstacleIsectFinder(edgeij, obstacles5);
        if (~isect)
            roadmap = [roadmap; edgeij];
        end

    end

end

[n_edges, ~] = size(roadmap);
roadmap2 = [];

for i = 1:n_edges
    edge_i = roadmap(i, :);

    test = edge_i - [2.7323, -0.18706, 1.9749, -0.18706];
    if all(abs(test) < 0.01)
        5;
    end

    tan1 = find_tangent(edge_i(1:2), edge_i(3:4), obstacles, side_offset);
    if tan1
        roadmap2 = [roadmap2; edge_i];
    end

end

roadmap3 = roadmap2;
thru_all = false;
edge_index = 1;

while ((~thru_all))
    edge_i = roadmap3(edge_index, :);
    test = roadmap3 - edge_i;
    roadmap3 = roadmap3(any(0.2 < abs(test), 2), :);
    roadmap3 = [edge_i; roadmap3];
    [n_edge_now, ~] = size(roadmap3);
    if n_edge_now == edge_index
        thru_all = true;
    else
        edge_index = edge_index + 1;
    end
end

end


function [tan1] = find_tangent( ...
    pot_vert1, pot_vert2, obstacles, side_offset)

obstacles2 = obstacle_offset(obstacles, side_offset*0.99);
obstacles3 = obstacles_return(obstacles2);

mean_length = mean(((obstacles(:, 1) - obstacles(:, 3)).^2 + (obstacles(:, 2) - obstacles(:, 4)).^2).^0.5);
extend_length = 0.01 * mean_length;
offset_length = 0.0001 * mean_length;

if pot_vert1(1) ~= pot_vert2(1)
    if pot_vert1(1) > pot_vert2(1)
        left_vertex = pot_vert2;
        right_vertex = pot_vert1;

    else
        left_vertex = pot_vert1;
        right_vertex = pot_vert2;

    end
    line_slope = (right_vertex(2) - left_vertex(2)) / (right_vertex(1) - left_vertex(1));
    theta = atan2(line_slope, 1);

else
    theta = pi / 2;
    if pot_vert1(2) > pot_vert2(2)
        left_vertex = pot_vert2;
        right_vertex = pot_vert1;
    else
        left_vertex = pot_vert1;
        right_vertex = pot_vert2;
    end
end


line_extend_leftx = left_vertex(1) - extend_length * cos(theta);
line_extend_lefty = left_vertex(2) - extend_length * sin(theta);
line_offset_leftx = left_vertex(1) - offset_length * cos(theta);
line_offset_lefty = left_vertex(2) - offset_length * sin(theta);

line_extend_rightx = right_vertex(1) + extend_length * cos(theta);
line_extend_righty = right_vertex(2) + extend_length * sin(theta);
line_offset_rightx = right_vertex(1) + offset_length * cos(theta);
line_offset_righty = right_vertex(2) + offset_length * sin(theta);

line_left = [line_extend_leftx, line_extend_lefty, line_offset_leftx, line_offset_lefty];
line_right = [line_offset_rightx, line_offset_righty, line_extend_rightx, line_extend_righty];

[isect1] = obstacleIsectFinder(line_left, obstacles3);
[isect2] = obstacleIsectFinder(line_right, obstacles3);

in22 = obstacle_with_in_finder(line_left(1:2), obstacles3);
in23 = obstacle_with_in_finder(line_left(3:4), obstacles3);
in24 = obstacle_with_in_finder(line_right(1:2), obstacles3);
in25 = obstacle_with_in_finder(line_right(3:4), obstacles3);

if any([isect1, isect2, in22, in23, in24, in25])
    tan1 = false;
else
    tan1 = true;
end

end


function obstacles3 = obstacle_offset(obstacles, side_offset)

[n_obstacles, max_dim] = size(obstacles);

max_dim = max_dim / 2;

obstacles2 = obstacles_reform(obstacles);

% Initialize matrix with the lengths of all obstacle edges
lengths = zeros(n_obstacles, max_dim);

% Iterate thru all obstacle edges to find length

for i = 1:n_obstacles % Iterate thru all obstacles
    ob_i = obstacles2((2 * i - 1):(2 * i), :);

    ob_i_x = ob_i(1, :);
    ob_i_y = ob_i(2, :);

    n_points_i = length(ob_i_x);

    for j = 1:n_points_i - 1 % Iterate thru all lines on ith obstacle
        xj = ob_i_x(j);
        yj = ob_i_y(j);
        xjp1 = ob_i_x(j+1);
        yjp1 = ob_i_y(j+1);

        length_j = norm(xjp1-xj, yjp1-yj);
        lengths(i, j) = length_j;

    end
end

% Initialize matrix that will store corners of new set of obstacles that
% are offset from current obstacles

% This new set of larger obstacles will prvent the robot from running into
% an obstacle if it follows an edge that is along the side of an obstacle

obstacles3 = zeros(n_obstacles*2, max_dim+1);

for i = 1:n_obstacles % Iterate thru all obstacles
    ob_i = obstacles2((2 * i - 1):(2 * i), :);

    ob_i_x = ob_i(1, :);
    ob_i_y = ob_i(2, :);

    n_points_i = length(ob_i_x);

    for j = 1:n_points_i - 1 % Iterate thru all lines on ith obstacle

        % Find points of lines on either side of this point
        if j ~= 1
            xjm1 = ob_i_x(j-1);
            yjm1 = ob_i_y(j-1);
        else
            xjm1 = ob_i_x(n_points_i-1);
            yjm1 = ob_i_y(n_points_i-1);
        end

        xj = ob_i_x(j);
        yj = ob_i_y(j);
        xjp1 = ob_i_x(j+1);
        yjp1 = ob_i_y(j+1);

        % Find vectors of two lines on either side of point
        vec1 = [xjm1 - xj, yjm1 - yj];
        vec2 = [xjp1 - xj, yjp1 - yj];

        % Find angles of vectors
        angle1 = atan2(vec1(2), vec1(1));
        angle2 = atan2(vec2(2), vec2(1));

        if angle1 <= 0
            angle1 = 2 * pi - angle1 * -1;
        end

        if angle2 <= 0
            angle2 = 2 * pi - angle2 * -1;
        end

        % Find angle of the new, extended point wrt the old point
        if angle1 > angle2
            d_theta = angle2 + (2 * pi - angle1);
        else
            d_theta = angle2 - angle1;
        end

        if d_theta > pi
            complement = 2 * pi - d_theta;
            corner_offset = side_offset / sin(complement/2);
        else
            corner_offset = side_offset / sin(d_theta/2);
        end

        angle = angle1 + d_theta / 2;
        new_x = xj + corner_offset * cos(angle);
        new_y = yj + corner_offset * sin(angle);

        obstacles3(2*i-1:2*i, j) = [new_x; new_y];

    end
end

for i = 1:n_obstacles
    obx_i = obstacles3(2*i-1, :);
    oby_i = obstacles3(2*i, :);

    obx_i(end) = obx_i(1);
    oby_i(end) = oby_i(1);


    obstacles3(2*i-1, :) = obx_i;
    obstacles3(2*i, :) = oby_i;
end

end


function [obstacles2] = obstacles_reform(obstacles)

[n_obstacles, max_dim] = size(obstacles);


obstacles2 = zeros(n_obstacles*2, max_dim/2+1);

%reorganize obstacle matrix
for i = 1:n_obstacles
    ob_i = obstacles(i, :);
    obix = ob_i(1:2:end);
    obiy = ob_i(2:2:end);
    obix(end+1) = obix(1);
    obiy(end+1) = obiy(1);

    obix(end+1:max_dim/2+1) = 0;
    obiy(end+1:max_dim/2+1) = 0;

    obstacles2(2*i-1, :) = obix;
    obstacles2(2*i, :) = obiy;
end

end


function [obstacles4] = obstacles_return(obstacles3)

[n_obstacles2, max_dim] = size(obstacles3);
n_obstacles = n_obstacles2 / 2;

obstacles4 = zeros(n_obstacles2/2, max_dim*2);

% Reorganize obstacle matrix
for i = 1:n_obstacles
    ob_i_pre = obstacles3(2*i-1:2*i, :);
    ob_i = ob_i_pre(ob_i_pre ~= 0);
    ob_i = ob_i.';

    obstacles4(i, 1:length(ob_i)) = ob_i;

end

end


function [in22] = obstacle_with_in_finder(vertex_j, obstacles)

[n_obstacles, ~] = size(obstacles);
in22 = false;

for i = 1:n_obstacles
    obi = obstacles(i, :);
    obx = obi(1:2:end);
    oby = obi(2:2:end);
    pt_in = inpolygon(vertex_j(1), vertex_j(2), obx, oby);
    if pt_in
        in22 = true;
    end

end

end
