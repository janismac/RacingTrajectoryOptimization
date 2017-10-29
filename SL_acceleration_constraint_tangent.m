% Calculates a tangent to the elliptical acceleration constraints.
% p = parameter struct
% i = index of tangent
% x = [px,py,vx,vy] (previous state vector)

% Resulting constraint: Au * [ax,ay] <= b
function [Au,b] = acceleration_constraint_tangent(p,i,x)

    vx = x(3);
    vy = x(4);
    v_sq = vx*vx + vy*vy;
    v = sqrt(v_sq);
    
    delta_angle = 2*pi / p.n_acceleration_limits;
    
    c = cos(i*delta_angle);
    s = sin(i*delta_angle);
    
    v_idx = p.v_idx(v);
    ay_max = p.a_lateral_max_list(v_idx);
    ax_forward_max = p.a_forward_max_list(v_idx);
    ax_backward_max = p.a_backward_max_list(v_idx);
    
    if c > 0
        ax_max = ax_forward_max;
    else
        ax_max = ax_backward_max;
    end
    
    
    Au = 1/sqrt(v_sq + 0.01) * [ay_max*c ax_max*s] * [vx vy; -vy vx];
    b = ax_max * ay_max;

end