function [x_new, U_new, optimization_log] = SL_QP(p, x0, checkpoint_indices, x_previous)

assert(size(checkpoint_indices, 1) == 1);
assert(size(checkpoint_indices, 2) == p.Hp);


%% Index mapping
IDX = reshape((1:p.ns*p.Hp),p.ns,p.Hp)';
idx_x = IDX(:,p.ix);
idx_pos = IDX(:,p.ipos);
idx_u = IDX(:,p.iu);
idx_slack_lateral = p.ns*p.Hp+1;

%% Problem size
n_Vars = p.ns * p.Hp + 1;
n_Eqns = p.nx * p.Hp + 2;
n_Ineq = (2 + p.n_acceleration_limits) * p.Hp;

lin_objective = zeros(n_Vars,1);
quad_objective = zeros(n_Vars,n_Vars);
Aeq = zeros(n_Eqns,n_Vars);
beq = zeros(n_Eqns,1);
Aineq = zeros(n_Ineq,n_Vars);
bineq = zeros(n_Ineq,1);
lb = -inf(n_Vars,1);
ub =  inf(n_Vars,1);


%% Equations
rows = 0;

% Linear model: x1 = Ax + Bu
dt = p.dt;
ddt = dt * dt / 2;
Ad = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bd = [ddt 0; 0 ddt; dt 0; 0 dt];

rows = rows(end) + (1:p.nx);
Aeq(rows, idx_x(1,:)) = eye(p.nx);
Aeq(rows, idx_u(1,:)) = -Bd;
beq(rows) = Ad * x0;

for k=2:p.Hp
    rows = rows(end) + (1:p.nx);
    Aeq(rows, idx_x(k,:)) = eye(p.nx);
    Aeq(rows, idx_x(k-1,:)) = -Ad;
    Aeq(rows, idx_u(k,:)) = -Bd;
end

% v_end = 0
rows = rows(end) + (1:2);
Aeq(rows, idx_x(k,3:4)) = eye(2);

assert(rows(end) == n_Eqns);

%% Inequalities

rows = 0;
for k=1:p.Hp
    % Left, right linearized track limits
    left_unit_vector = [0 -1;1 0] * p.checkpoints(checkpoint_indices(k)).forward_vector;
    
    rows = rows(end) + (1);
    Aineq(rows, idx_slack_lateral) = -1;
    Aineq(rows, idx_pos(k,:)) = left_unit_vector';
    bineq(rows) = left_unit_vector' * p.checkpoints(checkpoint_indices(k)).left;
    
    
    rows = rows(end) + (1);
    Aineq(rows, idx_slack_lateral) = -1;
    Aineq(rows, idx_pos(k,:)) = -left_unit_vector';
    bineq(rows) = -left_unit_vector' * p.checkpoints(checkpoint_indices(k)).right;

    % Acceleration limits    
    for i = 1:p.n_acceleration_limits
        rows = rows(end) + (1);
        [Au_acc,b_acc] = SL_acceleration_constraint_tangent(p,i,x_previous(:,k));
        Aineq(rows, idx_u(k,:)) = Au_acc;
        bineq(rows) = b_acc;
    end
end

assert(rows(end) == n_Ineq);

%% Objective

% Maximize position along track
lin_objective(idx_pos(p.Hp,:)) = -p.checkpoints(checkpoint_indices(p.Hp)).forward_vector;

% Minimize control change over time
quad_objective(idx_u(p.Hp,:),idx_u(p.Hp,:)) = p.R * eye(2);
for k=1:p.Hp-1
    quad_objective(idx_u(  k,:),idx_u(  k,:)) = 2 * p.R * eye(2);    
    quad_objective(idx_u(  k,:),idx_u(k+1,:)) = -p.R * eye(2);    
    quad_objective(idx_u(k+1,:),idx_u(  k,:)) = -p.R * eye(2);    
end
quad_objective(idx_u(1,:),idx_u(1,:)) = p.R * eye(2);

% Minimize slack var
quad_objective(idx_slack_lateral, idx_slack_lateral) = 1e5;


%% Bounds

% Slack Var is positive
lb(idx_slack_lateral) = 0;

% Bounded acceleration
a_max = max([p.a_backward_max_list p.a_forward_max_list p.a_lateral_max_list]);
ub(idx_u(:)) =  a_max;
lb(idx_u(:)) = -a_max;

% Trust region for change in position
for k=1:p.Hp
    ub(idx_pos(k,:)) = x_previous(1:2,k) + p.trust_region;
    lb(idx_pos(k,:)) = x_previous(1:2,k) - p.trust_region;
end


optimization_log = struct;

%% Solve the QP
if exist('cplexqp') == 6
    [vars,optimization_log.fval,optimization_log.exitflag,optimization_log.output,optimization_log.lambda] = ...
        cplexqp(quad_objective,lin_objective,Aineq,bineq,Aeq,beq,lb,ub,[],[]);
else
    options = optimoptions('quadprog', 'Display', 'none');
    quad_objective_sp = sparse(quad_objective);
    [vars,optimization_log.fval,optimization_log.exitflag,optimization_log.output,optimization_log.lambda] = ...
        quadprog(quad_objective_sp,lin_objective,Aineq,bineq,Aeq,beq,lb,ub,[],options); 
end

x_new = nan(p.nx,p.Hp);
U_new = nan(p.nu,p.Hp);
optimization_log.slack_lateral = nan;

if length(vars) == n_Vars
    for k=1:p.Hp
        x_new(:,k) = vars(idx_x(k,:));
        U_new(:,k) = vars(idx_u(k,:));
        optimization_log.slack_lateral = vars(idx_slack_lateral);
    end
else
    error('Solver failed!\n');
end


end