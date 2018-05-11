function [x_new, U_new, optimization_log] = SCR_QP(p, x0, track_polygon_indices)

assert(size(track_polygon_indices, 1) == p.Hp);
assert(size(track_polygon_indices, 2) == 1);

%% Index mapping
IDX = reshape((1:p.ns*p.Hp),p.ns,p.Hp)';
idx_x = IDX(:,p.ix);
idx_pos = IDX(:,p.ipos);
idx_u = IDX(:,p.iu);
idx_slack = p.ns * p.Hp + 1;

%% Problem size
n_Vars = p.ns * p.Hp + 1;
n_Eqns = p.nx * p.Hp + 2;
n_Ineq = p.n_acceleration_limits * p.Hp;
for k = 1:p.Hp
    n_Ineq = n_Ineq + length(p.track_polygons(track_polygon_indices(k)).b);
end

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
for k = 1:p.Hp
    track_polygon = p.track_polygons(track_polygon_indices(k));
    rows = rows(end) + (1:length(track_polygon.b));
    Aineq(rows, idx_pos(k,:)) = track_polygon.A;
    Aineq(rows, idx_slack) = -1;
    bineq(rows) = track_polygon.b;
    
    % Diagonal acceleration limits    
    for i = 1:p.n_acceleration_limits
        rows = rows(end) + (1);
        Aineq(rows, idx_u(k,:)) = [cos(2*pi*i/p.n_acceleration_limits) sin(2*pi*i/p.n_acceleration_limits)];
        bineq(rows) = p.a_max;
    end
end

assert(rows(end) == n_Ineq);


%% Objective

% Maximize position along track
lin_objective(idx_pos(p.Hp,:)) = -p.track_polygons(track_polygon_indices(p.Hp)).forward_direction;

% Minimize control change over time
quad_objective(idx_u(p.Hp,:),idx_u(p.Hp,:)) = p.R * eye(2);
for k=1:p.Hp-1
    quad_objective(idx_u(  k,:),idx_u(  k,:)) = 2 * p.R * eye(2);    
    quad_objective(idx_u(  k,:),idx_u(k+1,:)) = -p.R * eye(2);    
    quad_objective(idx_u(k+1,:),idx_u(  k,:)) = -p.R * eye(2);    
end
quad_objective(idx_u(1,:),idx_u(1,:)) = p.R * eye(2);

% Minimize slack var
lin_objective(idx_slack) = 1e5;

%% Bounds

% Slack Var
lb(idx_slack) = 0;

% Bounded acceleration
ub(idx_u(:)) =  p.a_max;
lb(idx_u(:)) = -p.a_max;


optimization_log = struct;
optimization_log.slack = nan;



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

if length(vars) == n_Vars
    for k=1:p.Hp
        x_new(:,k) = vars(idx_x(k,:));
        U_new(:,k) = vars(idx_u(k,:));
        optimization_log.slack = vars(idx_slack);
    end
else
    error('Solver failed!\n');
end


end