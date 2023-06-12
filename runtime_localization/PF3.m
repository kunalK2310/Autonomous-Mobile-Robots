function [particles, pose, map, op_walls, op_walls_counter, map_change] = ...
    PF3(particles0, u, z, std_dynamics, std_sense, map, op_walls, op_walls_counter, dynamics_func, depth_predict_func)

%% Optional Wall Constants
likely_bias = 1.5;
wall_add_thresh = 7;
wall_remove_thresh = -20;

% particles0: # number of dimensions of particle x number of particles
% std_dynamics: # number of dimensions of particle x 1
% z: # number of sensor readings per particle x number of particles
% std_sense: # number of sensor readings per particle x 1

[~, n_part] = size(particles0);
n_sense = length(z);

map_x = [map(:, 1), map(:, 3)];
map_y = [map(:, 2), map(:, 4)];
max_x = max(map_x, [], 'all');
min_x = min(map_x, [], 'all');
max_y = max(map_y, [], 'all');
min_y = min(map_y, [], 'all');

%% Particles bar

% Initialize matrix in which prediciton of all particles stored
particles_bar = [];

% u: 1x2
d_noise_term = std_dynamics(1);
phi_noise_term = std_dynamics(2);
d_noise_vec = ...
    normrnd(u(1), linspace(d_noise_term, d_noise_term, n_part));
phi_noise_vec = ...
    normrnd(u(2), linspace(phi_noise_term, phi_noise_term, n_part));

for k = 1:n_part % Iterate thru all particles to find prediction
    dynamics_kth = ...
        dynamics_func(particles0(:, k), d_noise_vec(k), phi_noise_vec(k));
    particles_bar = [particles_bar, dynamics_kth];
end

%% z

% Initialize matrix storing predicted measurement(s) for each particle
z_bar = [];
for p = 1:n_part %iterate thru all particles to find prediction
    z_bar_kth = depth_predict_func(particles_bar(:, p), map);
    z_bar = [z_bar, z_bar_kth];
end

% Create matrix where the std of the noise of all terms in the expected
% sensor measurement matrix are stored
std_sense_mat = std_sense * ones(n_sense, n_part);


% Input noise into expected measurements
z_bar = normrnd(z_bar, std_sense_mat);

% Initialize matrix storing the actual measurements, one column for each
% particle
z_mat = [];
for o = 1:n_part %iterate thru all particles
    z_mat = [z_mat, z];
end

%% Weights

% Find value of each actual measurement on a guassian distribution with
% mean of the expected measuremen
weights = normpdf(z_mat, z_bar, std_sense_mat);

% Find the sum of all the weights
weights_total = sum(weights, "all");

if weights_total == 0
    puts("Lost robot!");

    x_reset_vec = (max_x - min_x) * rand(1, n_part) + min_x;
    y_reset_vec = (max_y - min_y) * rand(1, n_part) + min_y;
    theta_reset_vec = 2 * pi * rand(1, n_part);

    particles = [x_reset_vec; y_reset_vec; theta_reset_vec];

    pose = [-999; -999; -999];
    map_change = 0;

else

    % Find the sum of the weights for each term of each particle
    weights = sum(weights, 1);

    % Normalize weights of each particle
    weights_norm = weights / weights_total;

    % Find cumulative weights starting at particle 1 and ending at particle
    % 0
    weights_cum = cumsum(weights_norm);

    % Randomly sample particles by generating random numbers
    rand_vec = rand(1, n_part);

    % Initialize boolean matrix, values in each column correspond to which
    % cumulative weights are larger than the randomly sampled number
    rand_mat_bool = weights_cum.' > rand_vec;

    % Initialize output matrix
    particles = [];

    % Find what cumulative weights each randomly generated number lies
    % between for a randomly generated number, the larger cumulative weight
    % that it lies between corresponds to the particle that is randomly
    % sampled
    for v = 1:n_part % Iterate thru all particles
        rand_vec_bool = rand_mat_bool(:, v);
        index = find(rand_vec_bool);
        particle_v = particles_bar(:, index(1));
        particles = [particles, particle_v];
    end

    pose = zeros(3, 1);
    pose(1) = mean(particles(1, :));
    pose(2) = mean(particles(2, :));
    pose(3) = mean(particles(3, :));

    [n_op_walls, ~] = size(op_walls);

    angles = linspace(27, -27, n_sense).';
    angles = angles * pi / 180;

    for i = 1:n_sense
        ray_l = z(i);
        ray_l_extend = ray_l + 2 * std_sense;
        ray_theta = pose(3) + angles(i);
        ray_x = pose(1) + ray_l_extend * cos(ray_theta);
        ray_y = pose(2) + ray_l_extend * sin(ray_theta);

        ray_line = [pose(1), pose(2), ray_x, ray_y];

        for j = 1:n_op_walls
            op_wall_j = op_walls(j, :);
            [isect_j, x_isect, y_isect] = ...
                intersectPoint( ...
                    ray_line(1), ray_line(2), ray_line(3), ray_line(4), ...
                    op_wall_j(1), op_wall_j(2), ...
                    op_wall_j(3), op_wall_j(4));

            if isect_j
                sep_j = norm([x_isect - pose(1), y_isect - pose(2)]);
                sep_diff = abs(sep_j-ray_l);
                op_wall_likely_j = -sep_diff / std_sense + likely_bias;
                op_walls_counter(j, 1) = ...
                    op_walls_counter(j, 1) + op_wall_likely_j;

            end

        end

    end


    if ~isempty(op_walls)
        walls_add_bool = op_walls_counter(:, 1) > wall_add_thresh;
        walls_add = op_walls(walls_add_bool, :);

        op_walls = op_walls(~walls_add_bool, :);
        adding = op_walls_counter(walls_add_bool, 2);
        op_walls_counter = op_walls_counter(~walls_add_bool, :);
        map = [map; walls_add];

        walls_remove_bool = op_walls_counter(:, 1) > wall_remove_thresh;
        op_walls = op_walls(walls_remove_bool, :);
        removing = op_walls_counter(~walls_remove_bool, 2);
        op_walls_counter = op_walls_counter(walls_remove_bool, :);


        if any(walls_add_bool) || any(~walls_remove_bool)
            map_change = [ ...
                adding ones(length(adding), 1); ...
                removing zeros(length(removing), 1) ...
            ];
        else
            map_change = [];
        end

    else
        map_change = [];
    end

end

end