function [p_t,weight_pt] = PFbeacon(p_0,weight_0,z,u,map,sensor_pos,angles,dynamic_func,measure_func,Q,R,boundaries,numBeacons,beacons,reading_type)
% INPUTS
%       p_0           the initial set of particles
%       u           dynamic matrix of all timestamps 
%       sigma_zero        initial estiamte of the covariance matrix of the state
%       R            state model noise covariance matrix
%       z        measurement matrix of all timestamps
%       Q        Measurement noise covariance matrix
%       map          map of the environment
%       sensor_pos   sensor position in the robot frame [x y]
%       angles    The vector that represents all angles of the range measurements
%       dynamic_func       the dynamics model function that estimates the expected next
%                    state of each particle in the previous particle set
%       measure_func       the measurement model function that estimates the
%                    expected measurement z_bar_t of the current states of the current estimated
%                    set of particles
%       boundaries   The boundaries of the map
%   OUTPUTS
%       p_t      The states of the final set of particles

[numPt,dimPt] = size(p_0);
[numTime,numDimM] = size(z);

% Get the boundareis of the map
    maxX = boundaries(1,2);
    maxY = boundaries(2,2);
    minX = boundaries(1,1);
    minY = boundaries(2,1);
    [numWall,LenWall] = size(map);
    
% Sampling from the points based on the given weights
p_t = [];
sampled_index = randsample(linspace(1,numPt,numPt),numPt,true,weight_0);
p_t = p_0(sampled_index,:);

% If using beacon reading, go into the following module
if reading_type == "beacon"
% Only use the z and x information from the beacon reading

%beacon_zs = zeros(numTime,2);
% for t=1:numTime
% beacon_zs(t,1) = z(t,4);
% beacon_zs(t,2) = z(t,5);
% end

beacon_zs = z(:,[4,5]);
beacon_ids = z(:,3);

% Loop through all rows of the beacon reading
% Each row means the information regarding one beacon in sight
% Normally there will be at most one beacon in sight
for t=1:numTime
    
    
    p_prev= p_t;
    p_t = [];
    p_t_probs = [];
    for i=1:length(p_prev)
       % Get the location estimate based on the dynamic function 
       loc_bar_it = dynamic_func(p_prev(i,:),u(1,1),u(2,1));
       % Add noise to the dynamics
       loc_bar_it = loc_bar_it + transpose(mvnrnd([0 0 0], R));
       % Check if the result gets out of the map
       % If so, set the probability to zero
       loc_prob_it = 1;
       if(loc_bar_it(1,1) > maxX | loc_bar_it(1,1) < minX | loc_bar_it(2,1) > maxY | loc_bar_it(2,1) < minY)
           loc_prob_it = 0;
           
       else

        
    
        %look up apriltag id
        id = find(beacons(:,1)==beacon_ids(t,1),1); %find which line of beaconpoints to take
        beaconpoint = beacons(id,2:3); %beaconpoint location 
        

        % Call the measurement function for the beacon
        z_bar_it = measure_func(loc_bar_it,sensor_pos,beaconpoint);
        sensorGlobal = robot2global(transpose(loc_bar_it),sensor_pos);
        sensorX = sensorGlobal(1);
        sensorY = sensorGlobal(2);
        % Calculatet he probability of the beacon estiamtion given the
        % beacon reading
        loc_prob_it = mvnpdf(z_bar_it,beacon_zs(t,:),sqrt(Q));
        
        % Sometimes the beacon might be close to the robot in location
        % But not visible in sight because of walls
        % In those cases, we need to set the probabilities to zero
        % Using the following for loop
        for widx=1:numWall
            [interExist, interX, interY] = intersectPoint(map(widx,1), map(widx,2), map(widx,3), map(widx,4), sensorX, sensorY, beaconpoint(1), beaconpoint(2));
               
                if interExist
                    loc_prob_it = zeros(size(loc_prob_it));
                    break
                end
        end
       
       end
       p_t = [p_t; transpose(loc_bar_it)];
       p_t_probs = [p_t_probs; loc_prob_it];
    end
    % Normalize the probabilities
    if sum(p_t_probs) >0 
       weight_pt = p_t_probs/sum(p_t_probs);
    else
        weight_pt = repelem(1/numPt,numPt); 
    end
       



end
% If using depth reading, go into the following module
else
   
% For depth
for t=1:numTime
    
    
    p_prev= p_t;
    p_t = [];
    p_t_probs = [];
    for i=1:length(p_prev)
       loc_bar_it = dynamic_func(p_prev(i,:),u(1,t),u(2,t));
       % Add noise to the dynamics
       loc_bar_it = loc_bar_it + transpose(mvnrnd([0 0 0], R));
       % Check if the result gets out of the map
       loc_prob_it = 1;
       if(loc_bar_it(1,1) > maxX | loc_bar_it(1,1) < minX | loc_bar_it(2,1) > maxY | loc_bar_it(2,1) < minY)
           loc_prob_it = 0;
           
       else
% Use the measurement function of the depth
z_bar_it = measure_func(loc_bar_it,map,sensor_pos,angles);
z_bar_it(z_bar_it<0.135) = 0;
loc_prob_it = mvnpdf(z_bar_it,transpose(z(t,:)),sqrt(Q));
       end
       p_t = [p_t; transpose(loc_bar_it)];
       p_t_probs = [p_t_probs; loc_prob_it];
    end
    % Normalize the probabilities
    if sum(p_t_probs) >0 
       weight_pt = p_t_probs/sum(p_t_probs);
    else
        weight_pt = repelem(1/numPt,numPt); 
    end
       



end
end


