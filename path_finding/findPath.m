function [best_path_return] = findPath( ...
    roadmap, obstacles, q_goal, q_start, side_offset)
% Find best path from start point to end point.
%
% Inputs:
%  - roadmap
%  - obstacles
%  - q_goal
%  - q_start
%  - side_offset
%
% Outputs:
%  - best_path_return

% Reorganize roadmap matrix s.t. each row is only one vertex
roadmap_points = [roadmap(:, 1:2); roadmap(:, 3:4)];

% Remove duplicate points
roadmap_points = unique(roadmap_points, "rows");

% Add goal and start to roadmap
roadmap_points = [roadmap_points; q_goal; q_start];

[n_points, ~] = size(roadmap_points);

% Call functions which connect q_goal and start to roadmap and update
% roadmap
roadmap = point2roadmap(obstacles, roadmap, q_goal, side_offset);
roadmap = point2roadmap(obstacles, roadmap, q_start, side_offset);

[n_vert, ~] = size(roadmap_points);

% Q is n_points x 4 (x,y, index, cost to come)

[q_start_index, ~] = size(roadmap_points);
q_goal_index = q_start_index - 1;

% Cell array that will contain indices of edges on optimal path to vertex
best_path = cell(n_vert, 1);

% Don't need to take any path to get from start to start
best_path{n_vert} = [];

% Vector with booleans that determine if point is visited
visited = zeros(n_points, 1);

% Initial cost to come to all points is infinity
cost_to_come_vec = Inf * ones(n_points, 1); 

index_vec = (1:n_points).';

% Append cost vector, visitied vector, and index vector to points
roadmap_points = [roadmap_points, cost_to_come_vec, visited, index_vec]; 

% Cost to come to start is zero
roadmap_points(q_start_index, 3) = 0; 

% First point to in Q is start
Q = [roadmap_points(q_start_index, :)];

x = 0;

while (~isempty(Q))
    x = x + 1;
    analyze = Q(1, :); % Analyze top of Q

    Q = Q(2:end, :); % Removed point being analyzed from Q

    % Call function which finds children(x,y,indez) of point
    [children, best_path, roadmap_points] = ...
        find_children(analyze, roadmap, roadmap_points, best_path);

    Q = [Q; children];

    Q = sortrows(Q, 3);
end

best_path_goal = best_path{q_goal_index};
n_points_path = length(best_path_goal);
best_path_return = zeros(n_points_path, 2);

for i = 1:n_points_path
    best_path_point_index = best_path_goal(i);
    best_point_finder = best_path_point_index == roadmap_points(:, 5);

    bpf = best_point_finder;

    best_point_finder2 = [bpf, bpf, bpf, bpf, bpf];
    best_point_i = roadmap_points(best_point_finder2);
    best_point_i = best_point_i(1:2);
    best_path_return(i, :) = best_point_i;
end

best_path_return = [best_path_return; q_goal];

end


function [roadmap] = point2roadmap(obstacles, roadmap, point, side_offset)

obstacles4 = obstacle_offset(obstacles, side_offset*0.95);
obstacles5 = obstacles_return(obstacles4);

roadmap_points = [roadmap(:, 1:2); roadmap(:, 3:4)];

roadmap_points = unique(roadmap_points, "rows");

% Find distance b/w point and all roadmap vertices
distance_q_xy = roadmap_points - point; 

% Find norm of all distances
distance_q_norm = vecnorm(distance_q_xy, 2, 2);

% Sort distances s.t. shortest is first, index of all vertices found
[~, distance_q_index] = sort(distance_q_norm);

% Bool that turns true once a vertex is found that q_goal can connect to
% w/o hitting an obstacle
found_q_connect_point = false;

% Counting variable corresponding to index of vertex analyzed
% Smallest index is roadmap vertex closest to q_goal
% Largest index is roadmap vertex furthest to q_goal
vertex_index = 1;

while (~found_q_connect_point)
    if vertex_index > 12
        5;
    end
    vertex_index2 = distance_q_index(vertex_index);
    vertex_i = roadmap_points(vertex_index2, :);

    q2vertex_i = [point, vertex_i];
    [isect] = obstacleIsectFinder(q2vertex_i, obstacles5);

    % The line b/w this roadmap vertex and the goal does not intersect any 
    % obstacles
    if (~isect)
        found_q_connect_point = true;
        roadmap = [roadmap; q2vertex_i]; % Append this line to the road map

    % The line b/w this roadmap vertex and the goal does intersect an
    % obstacle
    else
        vertex_index = vertex_index + 1; % Move on to next line
    end

end

end


function [children, best_path, roadmap_points] = find_children( ...
    analyze, roadmap, roadmap_points, best_path)

analyze_xy = analyze(1:2); % X and Y of point being analyzed
analyze_cost = analyze(3); % Cost of point being analyzed
analyze_index = analyze(5); % Index of point being analyzed

% Update visited status of analyze in roadmap points
roadmap_points(all(roadmap_points(:, 1:2) == analyze_xy, 2), :) = ...
    [analyze_xy, analyze_cost, 1, analyze_index];

roadmap_points_xy = roadmap_points(:, 1:2);

% Find edges that contain the point being analyzed
roadmap1 = roadmap(:, 1:2);
roadmap2 = roadmap(:, 3:4);

where_is_analyze1 = analyze_xy == roadmap1;
where_is_analyze2 = analyze_xy == roadmap2;

where_is_analyze1 = all(where_is_analyze1, 2);
where_is_analyze2 = all(where_is_analyze2, 2);

where_is_analyze = any([where_is_analyze1, where_is_analyze2], 2);

edges_w_analyze = roadmap(where_is_analyze, :);

% Initialize vector that will store children
children = [];

% Find vertices connected to analyze that have yet to be analyzed
points_of_edges_w_analyze = ...
    [edges_w_analyze(:, 1:2); edges_w_analyze(:, 3:4)];
other_vertices_index = any(points_of_edges_w_analyze ~= analyze_xy, 2);

% Other_vertices_index = (~other_vertices_index);
other_vertices = points_of_edges_w_analyze(other_vertices_index, :);

[n_other, ~] = size(other_vertices);

for i = 1:n_other
    other_i = other_vertices(i, :);

    ith_index_tf = (roadmap_points_xy == other_i);
    ith_index_tf = all(ith_index_tf, 2);
    ith_index = roadmap_points(ith_index_tf, 5);

    ith_visited = roadmap_points(ith_index, 4);

    if (~ith_visited)
        ith_xy = roadmap_points(ith_index, 1:2);
        ith_cost = roadmap_points(ith_index, 3);
        sep_ith = sqrt((ith_xy(1) - analyze_xy(1))^2 + ...
            (ith_xy(2) - analyze_xy(2))^2);
        cost_ith_from_analyze = sep_ith + analyze_cost;

        if cost_ith_from_analyze < ith_cost
            roadmap_points(ith_index, 3) = cost_ith_from_analyze;
            best_path{ith_index} = ...
                [best_path{analyze_index}, analyze_index];
        end

        children = [children; roadmap_points(ith_index, :)];

    end

end

if ~isempty(children)
    children = sortrows(children, 3);
end

end


function [obstacles3] = obstacle_offset(obstacles, side_offset)

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
% An obstacle if it follows an edge that is along the side of an obstacle

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

function obstacles2 = obstacles_reform(obstacles)

[n_obstacles, max_dim] = size(obstacles);

obstacles2 = zeros(n_obstacles*2, max_dim/2+1);

% Reorganize obstacle matrix
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


function obstacles4 = obstacles_return(obstacles3)

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


function [isect] = obstacleIsectFinder(line, obstacles)

[n_obstacles, ~] = size(obstacles);

linex1 = line(1);
liney1 = line(2);
linex2 = line(3);
liney2 = line(4);

isect = false;

for i = 1:n_obstacles
    obstacle_iw0 = obstacles(i, :);
    obstacle_i = obstacle_iw0(obstacle_iw0 ~= 0);

    [n_edges2] = length(obstacle_i);

    n_edges = n_edges2 / 2 - 1;

    for j = 1:n_edges
        edge_j = obstacle_i(2*j-1:2*j+2);

        edgex1 = edge_j(1);
        edgex2 = edge_j(3);
        edgey1 = edge_j(2);
        edgey2 = edge_j(4);

        [isect_ij, ~, ~] = intersectPoint(linex1, liney1, linex2, liney2, edgex1, edgey1, edgex2, edgey2);

        if isect_ij
            isect = true;
        end

    end

end

end
