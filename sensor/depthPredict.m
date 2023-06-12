function [depth] = depthPredict(robotPose, map, sensorOrigin, angles)
% Predict the depth measurements for a robot given its pose and the map.
%
% Inputs:
%  - robotPose     3-by-1 pose vector in global coordinates (x,y,theta)
%  - map           N-by-4 matrix containing the coordinates of walls in the
%                  environment: [x1, y1, x2, y2]
%  - sensorOrigin  origin of the sensor-fixed frame in the robot frame: [x y]
%  - angles        K-by-1 vector of the angular orientation of the range
%                  sensor(s) in the sensor-fixed frame, where 0 points
%                  forward. All sensors are located at the origin of the
%                  sensor-fixed frame.
%
% Outputs:
%  - depth         K-by-1 vector of depths (meters)

n_walls = size(map, 1);

n_sensors = length(angles);
sensor_global = robot2global(robotPose.', sensorOrigin);
x_sensor_global = sensor_global(1);
y_sensor_global = sensor_global(2);
theta = robotPose(3);

depth = zeros(n_sensors, 1);

sensor_ln_pts = [ ...
    x_sensor_global + 10 * cos(angles + theta), ...
    y_sensor_global + 10 * sin(angles + theta) ...
];

for sensor = 1:n_sensors
    sensor_pt = sensor_ln_pts(sensor, :);

    min_depth = inf;

    for wall = 1:n_walls
        wallx1 = map(wall, 1);
        wally1 = map(wall, 2);
        wallx2 = map(wall, 3);
        wally2 = map(wall, 4);

        [ISECT, X, Y] = intersectPoint( ...
            x_sensor_global, y_sensor_global, ...
            sensor_pt(1), sensor_pt(2), ...
            wallx1, wally1, wallx2, wally2);

        if ISECT
            range_wall = ...
                sqrt((X - x_sensor_global) ^ 2 + ...
                (Y - y_sensor_global) ^ 2);
            depth_wall = range_wall * cos(angles(sensor));
            if depth_wall < min_depth
                min_depth = depth_wall;
            end

        end

    end
    
    depth(sensor) = min_depth;

end

end