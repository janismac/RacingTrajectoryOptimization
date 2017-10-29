function p = params(controller_name, track_name)
p = struct;
addpath track_polygons

p.Hp = 50;    % Number of prediction steps
p.dt = 0.1;   % Time per prediction step
p.nx = 4;     % Number of state variables
p.nu = 2;     % Number of control variables
p.ns = p.nx + p.nu;
p.ix = 1:4;   % Indices of state variables
p.ipos = 1:2; % Indices of position variables
p.iu = 5:6;   % Indices of control variables
p.R = 0.05;   % Penalty weight for control changes over time
p.n_acceleration_limits = 16; % Number of tangents around the acceleration ellipse
p.name = controller_name;

if strcmp(controller_name, 'SL') 
    p.trust_region = 3;
    p.iterations = 2;

    % Empirically determined maximum accelerations in the forwards, backwards
    % and lateral directions, for varying speeds.
    p.a_lateral_max_list = interp1([0 10 43 52 200],[1 14 28 33 33], 0:0.01:120);
    p.a_forward_max_list = interp1([0 10 20 35 52 79 83 200],[2 13 18 18 15 6 1 1], 0:0.01:120);
    p.a_backward_max_list = interp1([0 30 40 52 76 200],[11 13 24 30 40 40], 0:0.01:120);
    p.v_idx = @(v) min(12001,max(1, round(100*v+1) ));
    
    
    if strcmp(track_name, 'Track1')
        p.checkpoints = testTrack1;
    else
        error('Invalid track name');
    end
    
    p.controller = @SL_optimizer;
elseif strcmp(controller_name, 'SCR')
    
    p.iterations = 1;
    p.a_max = 22;
    
    if strcmp(track_name, 'Track1')
        p.checkpoints = testTrack1;
    else
        error('Invalid track name');
    end
    
    track = generate_track_polygons(p.checkpoints);
    p.track_polygons = track.polygons;
    
    p.controller = @SCR_optimizer;
end

end