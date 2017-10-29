function controller_output = SCR_optimizer(p,x0,controller_output_previous)

controller_output = struct;
timer = tic;

if isempty(controller_output_previous) % First time step (start of the race)
    % Initial guessed trajectory: Standing still at x0.
    x = repmat(x0,1,p.Hp);
else
    % Shift previous trajectory by one time step.
    x = controller_output_previous.x;    
    x(:,1:end-1) = x(:,2:end);
end

for i = 1:p.iterations
    % For each trajectory point, find the closest track polygon.
    track_polygon_indices = nan(p.Hp,1);
    for k = 1:p.Hp
        position = x(p.ipos,k);
    
        min_signed_distance = 1e300;
        argmin_signed_distance = 0;
            
        for j = length(p.track_polygons):-1:1
            signed_distance = max(p.track_polygons(j).A * position - p.track_polygons(j).b);
            if min_signed_distance > signed_distance
                min_signed_distance = signed_distance;
                argmin_signed_distance = j;
            end
        end
        track_polygon_indices(k) = argmin_signed_distance;
    end
    
    % Formulate and solve the restricted trajectory optimization problem.
    [x, U, optimization_log] = SCR_QP(p, x0, track_polygon_indices);
end

opt_time = toc(timer);
fprintf('optT %6.0fms slack %6.2f fval %6.1f\n', opt_time*1000, optimization_log.slack, optimization_log.fval);

controller_output.x = x;
controller_output.U = U;
controller_output.opt_time = opt_time;
controller_output.optimization_log = optimization_log;
controller_output.track_polygon_indices = track_polygon_indices;

end