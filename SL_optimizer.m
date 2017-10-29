function controller_output = SL_optimizer(p,x0,controller_output_previous)

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
    % For each trajectory point, find the closest track checkpoint.
    checkpoint_indices = nan(1, p.Hp);
    for k=1:p.Hp
        [~,checkpoint_indices(k)] = min(sum(([p.checkpoints.center]-repmat(x(1:2,k),1,length(p.checkpoints))).^2));
    end
    
    % Formulate and solve the linearized trajectory optimization problem.
    [x, U, optimization_log] = SL_QP(p,x0,checkpoint_indices,x);
end

opt_time = toc(timer);
fprintf('optT %6.0fms slack %6.2f fval %6.1f\n', opt_time*1000, optimization_log.slack_lateral, optimization_log.fval);

controller_output.checkpoint_indices = checkpoint_indices;
controller_output.x = x;
controller_output.U = U;
controller_output.optimization_log = optimization_log;
controller_output.opt_time = opt_time;

end