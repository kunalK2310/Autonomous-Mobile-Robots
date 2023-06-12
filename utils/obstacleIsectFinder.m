function [isect] = obstacleIsectFinder(line, obstacles)
% Find if a line and a polygon map intersects at all.

n_obstacles = size(obstacles, 1);

linex1 = line(1);
liney1 = line(2);
linex2 = line(3);
liney2 = line(4);

isect = false;

for i = 1: n_obstacles
    obstacle_iw0 = obstacles(i,:);
    obstacle_i = obstacle_iw0(obstacle_iw0 ~= 0);

    [n_edges2] = length(obstacle_i);

    n_edges = n_edges2/2 - 1;

    for j = 1:n_edges
        edge_j = obstacle_i(2*j-1: 2*j+2);

        edgex1 = edge_j(1);
        edgex2 = edge_j(3);
        edgey1 = edge_j(2);
        edgey2 = edge_j(4);

        [isect_ij, ~, ~]= ...
            intersectPoint(...
                linex1, liney1, linex2, liney2, ...
                edgex1, edgey1, edgex2, edgey2);

        if isect_ij
            isect = true;
            break;
        end
    end

    if isect
        break;
    end

end

end

